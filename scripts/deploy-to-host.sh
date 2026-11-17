#!/usr/bin/env bash
# deploy-to-host.sh -- ship the repo's sim/ host/ agent/ hud/ tests/ tools/
# dirs to the EC2 sim host, SSM ONLY (no SSH, no S3). agent/ + hud/ are the
# build contexts for host/run_agent.sh and host/run_hud.sh (images spot-agent,
# spot-hud); tests/ is needed because agent/Dockerfile copies
# tests/test_agent_tools.py (FakeRobot for --dry-run); tools/ holds the
# rehearsal analysis helpers (rh_analyze.py, hud_watch.py, odom_rec.py).
#
# What it does (runs on the Mac):
#   1. refuses a dirty tree (threat model T30): the payload is `git archive` of
#      HEAD, so what runs on the host is a commit you can name. --allow-dirty
#      ships the working tree instead and records "<sha>-dirty".
#   2. writes VERSION (commit, dirty flag, UTC time) into the payload
#   3. base64-encodes the tarball (one long line) and splits it into
#      30000-char chunks
#   4. creates a fresh 0700 staging directory on the host under a random
#      name (mkdir fails if anything already holds the name), then sends each
#      chunk via `aws ssm send-command` (AWS-RunShellScript), appending to the
#      staged file; WAITS for each command to reach Success before sending the
#      next, because chunk ordering matters
#   5. final SSM command checks the byte count AND sha256 of the reassembled
#      b64 file, wipes the stale shipped dirs (all of them, tools/ included, so
#      files deleted or renamed locally do not linger), decodes + untars into
#      /opt/spot-demo (sudo mkdir -p, chown ubuntu), removes the staging
#      directory and prints VERSION and the file list
#
# Usage: scripts/deploy-to-host.sh [--allow-dirty]
# Env:   SPOT_SIM_INSTANCE (required: the sim host instance id)
#        AWS_REGION        (default us-west-2)
#        SPOT_DEMO_DIR     remote install dir (default /opt/spot-demo; at least two
#                          levels deep, not under a system directory)
#
# Every remote command is built with a JSON encoder and every path is checked
# against a plain-path pattern first, so no value can break out of the quoting.
# Fails loudly on any SSM command status other than Success.
set -euo pipefail

ALLOW_DIRTY=0
case "${1:-}" in
  "") ;;
  --allow-dirty) ALLOW_DIRTY=1 ;;
  *) echo "usage: $0 [--allow-dirty]" >&2; exit 2 ;;
esac

INSTANCE_ID="${SPOT_SIM_INSTANCE:?set SPOT_SIM_INSTANCE to the sim host instance id}"
REGION="${AWS_REGION:-us-west-2}"
REMOTE_DIR="${SPOT_DEMO_DIR:-/opt/spot-demo}"
CHUNK_SIZE=30000
POLL_INTERVAL=2
MAX_POLLS=150 # per SSM command: 150 x 2s = 5 min

[[ "$INSTANCE_ID" =~ ^i-[0-9a-f]{8,17}$ ]] || { echo "FATAL: bad SPOT_SIM_INSTANCE '$INSTANCE_ID'" >&2; exit 2; }
[[ "$REGION" =~ ^[a-z]{2}(-[a-z]+)+-[0-9]$ ]] || { echo "FATAL: bad AWS_REGION '$REGION'" >&2; exit 2; }
# Same plain-path rule as fetch-log.sh, plus: no '.' or '..' segment, at least
# two levels deep, and not under a system directory. root runs rm -rf and
# chown on this path (recursively only on what this script ships).
[[ "$REMOTE_DIR" =~ ^(/[A-Za-z0-9_][A-Za-z0-9_.-]*){2,}$ ]] || { echo "FATAL: bad SPOT_DEMO_DIR '$REMOTE_DIR'" >&2; exit 2; }
case "/${REMOTE_DIR#/}/" in
  */./* | */../*) echo "FATAL: bad SPOT_DEMO_DIR '$REMOTE_DIR'" >&2; exit 2 ;;
esac
TOP="${REMOTE_DIR#/}"; TOP="${TOP%%/*}"
case "$TOP" in
  bin | boot | dev | etc | lib | lib32 | lib64 | libx32 | proc | root | run | sbin | snap | sys | usr | var)
    echo "FATAL: SPOT_DEMO_DIR '$REMOTE_DIR' is under /$TOP" >&2; exit 2 ;;
esac

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

command -v aws >/dev/null 2>&1 || { echo "FATAL: aws CLI not found" >&2; exit 1; }
command -v git >/dev/null 2>&1 || { echo "FATAL: git not found" >&2; exit 1; }
command -v python3 >/dev/null 2>&1 || { echo "FATAL: python3 not found" >&2; exit 1; }
SHIP_DIRS=(sim host agent hud tests tools)
for d in "${SHIP_DIRS[@]}"; do
  [ -d "$REPO_ROOT/$d" ] || { echo "FATAL: $REPO_ROOT/$d not found" >&2; exit 1; }
done

COMMIT="$(git -C "$REPO_ROOT" rev-parse HEAD)"
DIRTY="$(git -C "$REPO_ROOT" status --porcelain --untracked-files=all -- "${SHIP_DIRS[@]}")"
if [ -n "$DIRTY" ] && [ "$ALLOW_DIRTY" -ne 1 ]; then
  echo "FATAL: uncommitted or untracked files in the shipped dirs:" >&2
  echo "$DIRTY" >&2
  echo "Commit them, or pass --allow-dirty to ship the working tree (recorded as $COMMIT-dirty)." >&2
  exit 1
fi
VERSION="$COMMIT"; [ -n "$DIRTY" ] && VERSION="$COMMIT-dirty"

WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT

# 1-2. payload: the commit's tree (or the working tree with --allow-dirty), plus
#      VERSION. COPYFILE_DISABLE stops macOS bsdtar adding ._* AppleDouble files.
mkdir "$WORKDIR/tree"
if [ -n "$DIRTY" ]; then
  (cd "$REPO_ROOT" && COPYFILE_DISABLE=1 tar cf - --exclude '__pycache__' --exclude '*.pyc' \
    --exclude '.DS_Store' "${SHIP_DIRS[@]}") | tar xf - -C "$WORKDIR/tree"
else
  git -C "$REPO_ROOT" archive --format=tar HEAD "${SHIP_DIRS[@]}" | tar xf - -C "$WORKDIR/tree"
fi
printf 'commit=%s\ndeployed_utc=%s\n' "$VERSION" "$(date -u +%Y-%m-%dT%H:%M:%SZ)" > "$WORKDIR/tree/VERSION"
(cd "$WORKDIR/tree" && COPYFILE_DISABLE=1 tar czf "$WORKDIR/payload.tgz" VERSION "${SHIP_DIRS[@]}")

# 3. base64 as one long line, then fixed-size chunks. split's alphabetical
#    suffixes (chunk_aa, chunk_ab, ...) preserve order under a sorted glob.
base64 < "$WORKDIR/payload.tgz" | tr -d '\n' > "$WORKDIR/payload.b64"
B64_BYTES="$(wc -c < "$WORKDIR/payload.b64" | tr -d ' ')"
# sha256 catches what a pure byte count cannot (same-length corruption, a
# duplicated + dropped chunk pair). shasum ships with macOS; sha256sum with
# Ubuntu coreutils on the host.
B64_SHA="$(shasum -a 256 "$WORKDIR/payload.b64" | cut -d' ' -f1)"
mkdir "$WORKDIR/chunks"
split -b "$CHUNK_SIZE" "$WORKDIR/payload.b64" "$WORKDIR/chunks/chunk_"

# Random staging name: 128 bits, so no local user on the host can pre-create it.
STAGE="/tmp/spot-demo-deploy.$(od -An -N16 -tx1 /dev/urandom | tr -d ' \n')"
REMOTE_B64="$STAGE/payload.b64"

# make_req <comment> <command>... -> writes $WORKDIR/req.json (JSON-encoded)
make_req() {
  python3 - "$INSTANCE_ID" "$@" > "$WORKDIR/req.json" <<'PY'
import json, sys
instance, comment, *commands = sys.argv[1:]
json.dump({"DocumentName": "AWS-RunShellScript", "InstanceIds": [instance],
           "TimeoutSeconds": 600, "Comment": comment,
           "Parameters": {"commands": commands}}, sys.stdout)
PY
}

# ssm_run -> sends $WORKDIR/req.json, prints the CommandId
ssm_run() {
  aws ssm send-command --region "$REGION" --cli-input-json "file://$WORKDIR/req.json" \
    --query 'Command.CommandId' --output text
}

# wait_success <command-id> <label> -- poll until Success; die loudly otherwise
wait_success() {
  local cid="$1" label="$2" status i
  for ((i = 0; i < MAX_POLLS; i++)); do
    # get-command-invocation can 404 right after send-command; treat as pending
    status="$(aws ssm get-command-invocation --region "$REGION" \
      --command-id "$cid" --instance-id "$INSTANCE_ID" \
      --query 'Status' --output text 2>/dev/null || echo NotYet)"
    case "$status" in
      Success) return 0 ;;
      Pending | InProgress | Delayed | NotYet) sleep "$POLL_INTERVAL" ;;
      *)
        echo "FATAL: $label ended with status '$status' (command $cid)" >&2
        aws ssm get-command-invocation --region "$REGION" --command-id "$cid" \
          --instance-id "$INSTANCE_ID" \
          --query '[StandardOutputContent,StandardErrorContent]' \
          --output text >&2 || true
        exit 1
        ;;
    esac
  done
  echo "FATAL: $label still not Success after $((MAX_POLLS * POLL_INTERVAL))s (command $cid)" >&2
  exit 1
}

chunks=("$WORKDIR"/chunks/chunk_*)
total="${#chunks[@]}"
echo "Deploying ${SHIP_DIRS[*]} at $VERSION to $INSTANCE_ID ($REGION): $B64_BYTES b64 bytes in $total chunk(s)"

# 4. staging directory first; mkdir without -p fails if the name exists.
make_req "spot-demo stage" "umask 077" "mkdir -m 0700 '$STAGE'"
wait_success "$(ssm_run)" "create staging directory"

#    Then the chunks, strictly in order. Chunk text is base64 alphabet only
#    (A-Za-z0-9+/=), so it is safe inside the remote single quotes.
n=0
for f in "${chunks[@]}"; do
  n=$((n + 1))
  make_req "spot-demo b64 chunk $n/$total" "printf '%s' '$(cat "$f")' >> '$REMOTE_B64'"
  wait_success "$(ssm_run)" "chunk $n/$total"
  echo "  chunk $n/$total OK"
done

# 5. verify size + sha256, wipe the shipped dirs, decode + untar, clean up, list.
# The new tree goes to ubuntu: the install directory itself and, recursively,
# only what this script ships, so an existing directory that is not a demo
# install keeps the owners of everything else in it (T30).
WIPE=()
for d in "${SHIP_DIRS[@]}"; do WIPE+=("'$REMOTE_DIR/$d'"); done
make_req "spot-demo unpack" \
  "set -e" \
  "[ \"\$(wc -c < '$REMOTE_B64')\" -eq $B64_BYTES ] || { echo SIZE-MISMATCH; exit 1; }" \
  "sha256sum '$REMOTE_B64' | grep -q '^$B64_SHA ' || { echo SHA-MISMATCH; exit 1; }" \
  "sudo mkdir -p '$REMOTE_DIR'" \
  "sudo rm -rf ${WIPE[*]} '$REMOTE_DIR/VERSION'" \
  "base64 -d '$REMOTE_B64' | sudo tar xzf - -C '$REMOTE_DIR'" \
  "sudo chown ubuntu:ubuntu '$REMOTE_DIR'" \
  "sudo chown -R ubuntu:ubuntu ${WIPE[*]} '$REMOTE_DIR/VERSION'" \
  "rm -rf '$STAGE'" \
  "echo DEPLOYED VERSION:; cat '$REMOTE_DIR/VERSION'" \
  "echo DEPLOYED FILES:; find '$REMOTE_DIR' -type f -not -path '*/logs/*' | sort"
cid="$(ssm_run)"
wait_success "$cid" "unpack into $REMOTE_DIR"
aws ssm get-command-invocation --region "$REGION" --command-id "$cid" \
  --instance-id "$INSTANCE_ID" --query 'StandardOutputContent' --output text
echo "Deploy complete: $VERSION"
