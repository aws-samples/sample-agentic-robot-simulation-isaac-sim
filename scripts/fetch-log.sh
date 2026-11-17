#!/usr/bin/env bash
# fetch-log.sh -- pull the mission's JSONL audit logs from the EC2 sim host to
# the Mac, SSM ONLY (no SSH, no S3): scripts/deploy-to-host.sh in reverse.
#
# What it does (runs on the Mac):
#   1. one SSM command packs $SPOT_DEMO_DIR/logs/<pattern> on the host into a
#      tar.gz, base64-encodes it as ONE line (no trailing newline: some base64
#      builds add one even with -w0) into /tmp/spot-logs-<token>.b64
#      and prints the file list, the b64 byte count and its sha256
#   2. pulls that line in 20000-char chunks, one SSM command each, strictly in
#      order (get-command-invocation returns at most 24000 chars of stdout,
#      so a bigger chunk would be silently truncated)
#   3. checks the byte count AND sha256 locally, decodes, untars into a temp
#      dir and moves the fetched files (*.jsonl / *.log / *.csv / *.json /
#      *.txt / *.png) into FETCH_DEST, flattened (a SUBDIR/ prefix
#      in the pattern is not recreated). The host logs are append-only, so a
#      newer fetch overwrites an older copy.
#   4. removes the remote temp file (also on failure, best effort)
#
# Read-only on the host apart from that temp file, created 0600 under a random
# name in /tmp. The default destination is outside this checkout, so fetched
# operator data cannot be committed by accident (threat model T36). The file
# contents travel in Run Command output, which Systems Manager keeps in its
# command history: fetch only what you need (T37).
#
# Usage: scripts/fetch-log.sh [PATTERN]     PATTERN default '*.jsonl'
#        e.g. scripts/fetch-log.sh 'mission-2026*.jsonl'
#             scripts/fetch-log.sh 'sim-2026*.log'      (per-run sim logs, host/run_sim.sh)
#             scripts/fetch-log.sh 'e2e-*.log'          (host/e2e_ros_test.sh)
#             scripts/fetch-log.sh 'screens/*.png'      (one subdir of logs/ allowed)
# Env:   SPOT_SIM_INSTANCE (required: the sim host's instance id)
#        AWS_REGION        (default us-west-2)
#        SPOT_DEMO_DIR     remote install dir (default /opt/spot-demo)
#        FETCH_DEST        local destination dir (default ~/spot-demo-logs/<instance id>)
#
# Needs the AWS CLI with credentials for the sim account, python3 (builds the
# request JSON) and shasum (both ship with macOS). Fails loudly on any SSM
# command status other than Success and on a size/sha256 mismatch.
set -euo pipefail

INSTANCE_ID="${SPOT_SIM_INSTANCE:?set SPOT_SIM_INSTANCE to the sim host instance id}"
REGION="${AWS_REGION:-us-west-2}"
REMOTE_DIR="${SPOT_DEMO_DIR:-/opt/spot-demo}"
PATTERN="${1:-*.jsonl}"
CHUNK_SIZE=20000
POLL_INTERVAL=1
MAX_POLLS=300 # per SSM command: 300 x 1s = 5 min

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${FETCH_DEST:-$HOME/spot-demo-logs/$INSTANCE_ID}"

command -v aws >/dev/null 2>&1 || { echo "FATAL: aws CLI not found" >&2; exit 1; }
command -v python3 >/dev/null 2>&1 || { echo "FATAL: python3 not found" >&2; exit 1; }
# Both strings end up inside a remote shell command: allow only plain path /
# glob characters so nothing can break out of the quoting.
[[ "$INSTANCE_ID" =~ ^i-[0-9a-f]{8,17}$ ]] || { echo "FATAL: bad SPOT_SIM_INSTANCE '$INSTANCE_ID'" >&2; exit 2; }
[[ "$REGION" =~ ^[a-z]{2}(-[a-z]+)+-[0-9]$ ]] || { echo "FATAL: bad AWS_REGION '$REGION'" >&2; exit 2; }
[[ "$REMOTE_DIR" =~ ^/[A-Za-z0-9_./-]+$ ]] || { echo "FATAL: bad SPOT_DEMO_DIR '$REMOTE_DIR'" >&2; exit 2; }
# Optional single SUBDIR/ prefix (no '..': the dir part may not start with '.').
[[ "$PATTERN" =~ ^([A-Za-z0-9_][A-Za-z0-9_-]*/)?[A-Za-z0-9_.*?-]+\.(jsonl|log|csv|json|txt|png)$ ]] || {
  echo "FATAL: PATTERN must be a plain file glob (optionally SUBDIR/glob) ending in .jsonl/.log/.csv/.json/.txt/.png (got '$PATTERN')" >&2; exit 2; }

LOG_DIR="$REMOTE_DIR/logs"
TOKEN="$(od -An -N16 -tx1 /dev/urandom | tr -d ' \n')"
REMOTE_B64="/tmp/spot-logs-$TOKEN.b64"
WORKDIR="$(mktemp -d)"
REMOTE_PACKED=0

# ssm_request <comment> <command>... -> writes $WORKDIR/req.json
ssm_request() {
  python3 - "$INSTANCE_ID" "$@" > "$WORKDIR/req.json" <<'PY'
import json, sys
instance, comment, commands = sys.argv[1], sys.argv[2], sys.argv[3:]
print(json.dumps({"DocumentName": "AWS-RunShellScript", "InstanceIds": [instance],
                  "TimeoutSeconds": 600, "Comment": comment,
                  "Parameters": {"commands": commands}}))
PY
}

# ssm_send -> prints the CommandId of $WORKDIR/req.json
ssm_send() {
  aws ssm send-command --region "$REGION" --cli-input-json "file://$WORKDIR/req.json" \
    --query 'Command.CommandId' --output text
}

# wait_status <command-id> -> prints the terminal status (or TIMEOUT)
wait_status() {
  local cid="$1" status i
  for ((i = 0; i < MAX_POLLS; i++)); do
    # get-command-invocation can 404 right after send-command; treat as pending
    status="$(aws ssm get-command-invocation --region "$REGION" \
      --command-id "$cid" --instance-id "$INSTANCE_ID" \
      --query 'Status' --output text 2>/dev/null || echo NotYet)"
    case "$status" in
      Pending | InProgress | Delayed | NotYet) sleep "$POLL_INTERVAL" ;;
      *) echo "$status"; return 0 ;;
    esac
  done
  echo TIMEOUT
}

# run_remote <label> <command>... -> prints stdout; dies unless Success
run_remote() {
  local label="$1" cid status
  shift
  ssm_request "spot-demo fetch-log: $label" "$@"
  cid="$(ssm_send)"
  status="$(wait_status "$cid")"
  if [ "$status" != Success ]; then
    echo "FATAL: $label ended with status '$status' (command $cid)" >&2
    aws ssm get-command-invocation --region "$REGION" --command-id "$cid" \
      --instance-id "$INSTANCE_ID" \
      --query '[StandardOutputContent,StandardErrorContent]' --output text >&2 || true
    exit 1
  fi
  aws ssm get-command-invocation --region "$REGION" --command-id "$cid" \
    --instance-id "$INSTANCE_ID" --query 'StandardOutputContent' --output text
}

cleanup() {
  local rc=$?
  if [ "$REMOTE_PACKED" = 1 ]; then
    # Best effort, bounded: never let cleanup hang or mask the real exit code.
    if ssm_request "spot-demo fetch-log: cleanup" "rm -f $REMOTE_B64" 2>/dev/null; then
      aws ssm send-command --region "$REGION" --cli-input-json "file://$WORKDIR/req.json" \
        --query 'Command.CommandId' --output text >/dev/null 2>&1 || true
    fi
  fi
  rm -rf "$WORKDIR"
  exit "$rc"
}
trap cleanup EXIT

# 1. pack on the host. POSIX sh (AWS-RunShellScript), nothing but paths from
#    the validated variables above. An unmatched glob stays literal -> NO-LOGS.
echo "Packing $LOG_DIR/$PATTERN on $INSTANCE_ID ($REGION)"
REMOTE_PACKED=1
PACK_OUT="$(run_remote "pack" \
  "set -e" \
  "cd '$LOG_DIR' 2>/dev/null || { echo NO-LOGDIR; exit 0; }" \
  "set -- $PATTERN" \
  "[ -f \"\$1\" ] || { echo NO-LOGS; exit 0; }" \
  "for f in \"\$@\"; do echo \"FILE \$(wc -c < \"\$f\" | tr -d ' ') \$f\"; done" \
  "umask 077" \
  "tar czf - -- \"\$@\" | base64 -w0 | tr -d '\\n' > $REMOTE_B64" \
  "echo BYTES=\$(wc -c < $REMOTE_B64 | tr -d ' ')" \
  "echo SHA=\$(sha256sum $REMOTE_B64 | cut -d' ' -f1)")"

if grep -q '^NO-LOGDIR' <<<"$PACK_OUT"; then
  echo "No log directory $LOG_DIR on the host (no mission has written a log yet). Nothing fetched."
  exit 0
fi
if grep -q '^NO-LOGS' <<<"$PACK_OUT"; then
  echo "No files matching $LOG_DIR/$PATTERN on the host. Nothing fetched."
  exit 0
fi
grep '^FILE ' <<<"$PACK_OUT" | sed 's/^FILE /  remote: /' || true
B64_BYTES="$(sed -n 's/^BYTES= *\([0-9][0-9]*\).*/\1/p' <<<"$PACK_OUT" | head -1)"
B64_SHA="$(sed -n 's/^SHA=\([0-9a-f]\{64\}\).*/\1/p' <<<"$PACK_OUT" | head -1)"
[ -n "$B64_BYTES" ] && [ -n "$B64_SHA" ] || {
  echo "FATAL: could not parse the pack step output:" >&2; echo "$PACK_OUT" >&2; exit 1; }

# 2. pull the chunks strictly in order (tail -c +N is 1-based).
total=$(((B64_BYTES + CHUNK_SIZE - 1) / CHUNK_SIZE))
echo "Fetching $B64_BYTES b64 bytes in $total chunk(s)"
: > "$WORKDIR/payload.b64"
for ((n = 0; n < total; n++)); do
  offset=$((n * CHUNK_SIZE + 1))
  run_remote "chunk $((n + 1))/$total" "tail -c +$offset $REMOTE_B64 | head -c $CHUNK_SIZE" \
    | tr -d '\r\n' >> "$WORKDIR/payload.b64"
  echo "  chunk $((n + 1))/$total OK"
done

# 3. verify, decode, unpack, install.
GOT_BYTES="$(wc -c < "$WORKDIR/payload.b64" | tr -d ' ')"
[ "$GOT_BYTES" -eq "$B64_BYTES" ] || {
  echo "FATAL: SIZE-MISMATCH: got $GOT_BYTES b64 bytes, want $B64_BYTES" >&2; exit 1; }
GOT_SHA="$(shasum -a 256 "$WORKDIR/payload.b64" | cut -d' ' -f1)"
[ "$GOT_SHA" = "$B64_SHA" ] || {
  echo "FATAL: SHA-MISMATCH: got $GOT_SHA, want $B64_SHA" >&2; exit 1; }
mkdir "$WORKDIR/untar"
base64 --decode < "$WORKDIR/payload.b64" | tar xzf - -C "$WORKDIR/untar"
mkdir -p "$DEST"
case "$(cd "$DEST" && pwd -P)/" in
  "$(cd "$REPO_ROOT" && pwd -P)"/*)
    echo "WARNING: $DEST is inside this checkout. Fetched logs hold operator data; do not commit them." >&2 ;;
esac
count=0
fetched=()
while IFS= read -r -d '' f; do
  mv -f "$f" "$DEST/"
  fetched+=("$(basename "$f")")
  count=$((count + 1))
done < <(find "$WORKDIR/untar" -type f -print0)
echo "Fetched $count log file(s) into $DEST:"
if [ "$count" -gt 0 ]; then (cd "$DEST" && ls -l -- "${fetched[@]}"); fi
