#!/bin/bash
# start-host.sh -- start the sim host, retrying while EC2 has no capacity for it (T39).
#
# Runs on the Mac. `aws ec2 start-instances` fails with InsufficientInstanceCapacity
# when the instance's Availability Zone has no free capacity for its type at that
# moment (a restart of this host once needed six attempts). This retries that error
# only, waiting longer each time (SPOT_START_BACKOFF seconds, doubling, at most 300),
# up to SPOT_START_ATTEMPTS attempts, then waits until the instance passes its status
# checks. Any other error stops at once. For a scheduled live demo, do not rely on
# retries: reserve the capacity first (docs/RUNBOOK.md, "Capacity for a scheduled
# demo"); a start then uses the reservation.
#
#   scripts/start-host.sh
#
# Exit: 0 running and status checks OK; 1 any other error; 2 bad settings; 3 still
# no capacity after the last attempt (it prints the reservation commands).
# Env: SPOT_SIM_INSTANCE (required), AWS_REGION (default us-west-2),
#      SPOT_START_ATTEMPTS (default 6, 1-20), SPOT_START_BACKOFF (first wait in
#      seconds, default 30, 0-300)
# Needs: AWS CLI v2; ec2:StartInstances, ec2:DescribeInstances and
#        ec2:DescribeInstanceStatus on the instance.
set -euo pipefail
I=${SPOT_SIM_INSTANCE:?set SPOT_SIM_INSTANCE to the sim host instance id}; R=${AWS_REGION:-us-west-2}
ATTEMPTS=${SPOT_START_ATTEMPTS:-6}; WAIT=${SPOT_START_BACKOFF:-30}
[[ "$I" =~ ^i-[0-9a-f]{8,17}$ ]] || { echo "bad SPOT_SIM_INSTANCE '$I'" >&2; exit 2; }
[[ "$R" =~ ^[a-z]{2}(-[a-z]+)+-[0-9]$ ]] || { echo "bad AWS_REGION '$R'" >&2; exit 2; }
[[ "$ATTEMPTS" =~ ^[0-9]{1,2}$ ]] && [ "$ATTEMPTS" -ge 1 ] && [ "$ATTEMPTS" -le 20 ] \
  || { echo "bad SPOT_START_ATTEMPTS '$ATTEMPTS' (1-20)" >&2; exit 2; }
[[ "$WAIT" =~ ^[0-9]{1,3}$ ]] && [ "$WAIT" -le 300 ] \
  || { echo "bad SPOT_START_BACKOFF '$WAIT' (0-300 seconds)" >&2; exit 2; }

advice() {
  local type="<instance-type>" az="<availability-zone>" found
  if found=$(aws ec2 describe-instances --region "$R" --instance-ids "$I" \
               --query 'Reservations[0].Instances[0].[InstanceType,Placement.AvailabilityZone]' \
               --output text 2>/dev/null); then
    read -r t z <<< "$found"
    [[ "${t:-}" =~ ^[a-z0-9]+\.[a-z0-9]+$ ]] && type=$t
    [[ "${z:-}" =~ ^[a-z]{2}(-[a-z]+)+-[0-9][a-z]$ ]] && az=$z
  fi
  cat >&2 <<EOF
start-host: EC2 still has no $type capacity in $az after $ATTEMPTS attempts.
Try again later. For a scheduled demo, reserve the capacity ahead of time instead
(docs/RUNBOOK.md, "Capacity for a scheduled demo"). It bills at the On-Demand rate
from the moment it is active, whether the instance runs or not, until it ends:
  aws ec2 create-capacity-reservation --region $R --instance-type $type \\
    --instance-platform Linux/UNIX --availability-zone $az --instance-count 1 \\
    --instance-match-criteria open --end-date-type limited --end-date <YYYY-MM-DDThh:mm:ssZ>
EOF
}

attempt=1
while :; do
  if out=$(aws ec2 start-instances --region "$R" --instance-ids "$I" \
             --query 'StartingInstances[0].CurrentState.Name' --output text 2>&1); then
    echo "start-host: start accepted on attempt $attempt ($out)"
    break
  fi
  if ! grep -q 'InsufficientInstanceCapacity' <<< "$out"; then
    echo "start-host: start failed (not a capacity error, not retried): $out" >&2
    exit 1
  fi
  if [ "$attempt" -ge "$ATTEMPTS" ]; then advice; exit 3; fi
  echo "start-host: no capacity (attempt $attempt of $ATTEMPTS); trying again in ${WAIT}s" >&2
  sleep "$WAIT"
  attempt=$((attempt + 1))
  WAIT=$((WAIT * 2)); [ "$WAIT" -le 300 ] || WAIT=300
done

echo "start-host: waiting for the instance to run and pass its status checks (a few minutes)"
aws ec2 wait instance-running --region "$R" --instance-ids "$I"
aws ec2 wait instance-status-ok --region "$R" --instance-ids "$I"
echo "start-host: $I is running and its status checks pass"
