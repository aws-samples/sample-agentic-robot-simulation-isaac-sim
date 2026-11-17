#!/bin/bash
# usage: ssmrun.sh <script-file> [timeout-seconds]  -> prints status + last 40 lines of stdout/stderr
# Runs the script on the sim host as root through SSM Run Command. The script and
# its output live in a fresh mktemp directory on the host that is removed after
# the run, so nothing is left in /tmp (threat model T37), unless the run is
# killed by the timeout, which leaves that directory behind. The script itself
# (in the command's parameters) and the last 40 lines of output stay in Systems
# Manager's command history: put no secrets or personal data in either.
set -euo pipefail
I=${SPOT_SIM_INSTANCE:?set SPOT_SIM_INSTANCE}; R=${AWS_REGION:-us-west-2}
SCRIPT=${1:?usage: ssmrun.sh <script-file> [timeout-seconds]}; T=${2:-3600}
[[ "$I" =~ ^i-[0-9a-f]{8,17}$ ]] || { echo "bad SPOT_SIM_INSTANCE '$I'" >&2; exit 2; }
[[ "$R" =~ ^[a-z]{2}(-[a-z]+)+-[0-9]$ ]] || { echo "bad AWS_REGION '$R'" >&2; exit 2; }
[[ "$T" =~ ^[0-9]+$ ]] || { echo "bad timeout '$T'" >&2; exit 2; }
B64=$(base64 < "$SCRIPT" | tr -d '\n')
REQ=$(mktemp); trap 'rm -f "$REQ"' EXIT
python3 - "$I" "$T" "$B64" > "$REQ" <<'PY'
import json, sys
instance, timeout, b64 = sys.argv[1], int(sys.argv[2]), sys.argv[3]
cmd = ('J=$(mktemp -d) && echo %s | base64 -d > "$J/s.sh" && bash "$J/s.sh" > "$J/out.log" 2>&1; '
       'rc=$?; tail -40 "$J/out.log"; rm -rf "$J"; exit $rc') % b64
json.dump({"DocumentName": "AWS-RunShellScript", "InstanceIds": [instance],
           "TimeoutSeconds": timeout, "Parameters": {"commands": [cmd]}}, sys.stdout)
PY
C=$(aws ssm send-command --region "$R" --cli-input-json "file://$REQ" --query Command.CommandId --output text)
while :; do S=$(aws ssm get-command-invocation --region "$R" --command-id "$C" --instance-id "$I" --query Status --output text 2>/dev/null || echo Pending)
  case $S in Success|Failed|TimedOut|Cancelled) break;; esac; sleep 10; done
echo "STATUS: $S"; aws ssm get-command-invocation --region "$R" --command-id "$C" --instance-id "$I" --query StandardOutputContent --output text
