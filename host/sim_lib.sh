# shellcheck shell=bash
# sim_lib.sh -- shared helpers for the host scripts (source it; not executable).
#
# Why this exists (2026-09-27 critic P0-1): the old runbook redirected the sim
# to /tmp/spot-sim.log and grepped it for FACADE-READY. /tmp is sticky and
# the host runs with fs.protected_regular=2, so a root-owned log left by an
# SSM run could not be opened by ubuntu: the redirect failed, no sim started,
# and the grep matched the OLD file's FACADE-READY line -- the presenter was
# told the sim was ready when it was not. Now:
#   * every run writes a NEW file under $SPOT_DEMO_DIR/logs/ (ubuntu-owned,
#     not sticky), created with noclobber after a writability check, plus a
#     <prefix>-latest.log symlink;
#   * readiness is read from `docker logs spot-sim` (the CURRENT container
#     only) and must carry this launch's run id (isaac_app.py prints
#     "FACADE-READY ... run_id=<SPOT_RUN_ID>").
#
# Functions:
#   spot_run_id                     new id, e.g. 20260927T141502Z-4242
#   spot_log_dir                    ensure the logs dir is writable; print it
#   spot_new_log PREFIX RUN_ID      create logs/PREFIX-RUN_ID.log; print its path
#   spot_sim_ready_line [RUN_ID]    print the running spot-sim's FACADE-READY
#                                   line (for RUN_ID if given); rc 1 if none
#   spot_wait_sim_ready RUN_ID TIMEOUT_S LOG [pid:PID|cid:CONTAINER_ID]
#                                   wait for that line; rc 1 (with the log
#                                   tail) if the watched launcher process or
#                                   container died, or the timeout passed
#   spot_fastdds_profile            write the loopback-only Fast DDS profile
#                                   (if changed); print its host path
#   spot_dds_args                   set SPOT_DDS_ARGS: the DDS isolation
#                                   options every container gets; rc 1 on a
#                                   bad ROS_DOMAIN_ID
#   spot_container_args KIND [--writable-root]
#                                   set SPOT_ARGS for an agent|hud|oneshot
#                                   container: its own uid, no capabilities,
#                                   no setuid escalation, read-only root, the
#                                   DDS isolation; rc 1 on a bad ROS_DOMAIN_ID
#   spot_shared_dir_gid DIR         make DIR group-writable and print its gid
#                                   (for --group-add: a container writes there
#                                   without being the dir's owner)
#   spot_pinned_image VAR           rc 1 (with why) unless VAR's image is
#                                   pinned by digest (name[:tag]@sha256:...)
#   spot_nonloopback_listeners [-] print "PROTO LOCAL PROCESS PID" for every
#                                   listening TCP / unconnected UDP socket not
#                                   bound to loopback (from `sudo ss -Hltnup`,
#                                   or that output on stdin with -)
#   spot_running_agents             print the running agent/mission containers
#   spot_guard_sim_stop FORCE WHAT  rc 1 (with why) when an agent or mission
#                                   container runs and FORCE is not 1 (T41)
#   spot_require_imds_guard WHAT    rc 1 (with why) unless host/imds_guard.sh's
#                                   rules are in place; SPOT_IMDS_GUARD=off
#                                   skips the check with a warning (T26)
#   spot_test_domain                export ROS_DOMAIN_ID as the test domain
#                                   (SPOT_TEST_DOMAIN_ID, default 43) for a
#                                   smoke or e2e sim; rc 1 if it is the demo's
#   spot_rm_own_sim RUN_ID          remove spot-sim only if it is RUN_ID's
#                                   launch, never a sim started since (T41)
# Env: SPOT_DEMO_DIR (default /opt/spot-demo), SPOT_LOG_DIR (default
#      $SPOT_DEMO_DIR/logs), ROS_DOMAIN_ID (default 42), SPOT_TEST_DOMAIN_ID
#      (default 43)

SPOT_DEMO_DIR="${SPOT_DEMO_DIR:-/opt/spot-demo}"
SPOT_LOG_DIR="${SPOT_LOG_DIR:-$SPOT_DEMO_DIR/logs}"
SPOT_SIM_CONTAINER=spot-sim

# --- images, pinned by digest (T31) -------------------------------------------
# One place for every image the scripts run. A tag can be re-pushed; a digest
# cannot. The tag stays in each reference for humans (the digest wins); the
# digests are the multi-arch index digests `docker buildx imagetools inspect
# <tag>` printed on 2026-10-29. agent/Dockerfile and hud/Dockerfile carry the
# same ros:jazzy-ros-base digest in FROM (a test checks). To move to a newer
# image: re-resolve, update here and in both FROM lines, rebuild, re-scan.
# An override (SPOT_ROS_IMAGE, ISAAC_IMAGE, ROS_IMAGE) must be pinned too.
# ros:jazzy-ros-base comes from ECR Public's mirror of the Docker official
# image (public.ecr.aws/docker/library/ros): its index digest is the same as
# docker.io/library/ros:jazzy-ros-base's (both checked 2026-10-29), so the
# content is identical; ECR Public needs no Docker Hub pull quota.
SPOT_ROS_IMAGE="${SPOT_ROS_IMAGE:-public.ecr.aws/docker/library/ros:jazzy-ros-base@sha256:8eed0a4669bbf43ee331878a4c39bd8ebddea8e3312a58b4fae8edb14526b617}"
# shellcheck disable=SC2034  # read by run_sim.sh
SPOT_ISAAC_IMAGE="nvcr.io/nvidia/isaac-sim:6.1.0@sha256:af1d2b4e75d553bfa27beb5a401198654aa8d607f3b7a6749196e9ce253def20"
# shellcheck disable=SC2034  # read by tools/snap_screens.sh
SPOT_PLAYWRIGHT_IMAGE="mcr.microsoft.com/playwright:v1.49.0-noble@sha256:0fc07c73230cb7c376a528d7ffc83c4bdcdcd3fc7efbe54a2eed72b1ec118377"

spot_pinned_image() {
  local var="$1" ref
  ref="${!var}"
  if ! [[ "$ref" =~ ^[a-z0-9][a-z0-9./_:-]*@sha256:[0-9a-f]{64}$ ]]; then
    echo "FATAL: $var='$ref' is not pinned by digest (name[:tag]@sha256:<64 hex>)" >&2
    return 1
  fi
}

# --- container identity (T28, T26) -------------------------------------------
# Each container runs as its own non-root uid, stated here once.
# agent/Dockerfile and hud/Dockerfile set the same USER (a test checks), and
# host/imds_guard.sh tells containers apart by these uids. None is 1000:
# GDM gives ubuntu's X display to SI:localuser:ubuntu, and every container
# on --network=host can reach that display through the abstract X socket,
# so a container running as uid 1000 would get the desktop. Only the sim's
# uid gets the X grant run_sim.sh --gui adds.
# shellcheck disable=SC2034  # read by run_sim.sh's checks, imds_guard.sh and the tests
SPOT_SIM_UID=1234       # isaac-sim, baked into the NVIDIA image (not ours to choose)
SPOT_AGENT_UID=10001    # spot-agent / spot-mission: the only container that calls AWS
SPOT_HUD_UID=10002      # spot-hud
SPOT_ONESHOT_UID=10003  # inject_gas, reset_spot, set_view, the e2e client, spot-rec, Playwright

# --- DDS isolation (T25) -----------------------------------------------------
# Every container the host scripts start (sim, agent, HUD, one-shots) gets
# the same two things, so they find each other and nothing else does:
#   * ROS_DOMAIN_ID ${ROS_DOMAIN_ID:-42}: off domain 0, which every stock
#     ROS 2 install on the network uses;
#   * the Fast DDS profile below, which binds every RTPS socket to 127.0.0.1
#     and sends discovery only to 127.0.0.1 (its initial peers).
# ROS_AUTOMATIC_DISCOVERY_RANGE is deliberately not set. LOCALHOST alone is
# not enough: measured with two --network=host containers and a third on a
# Docker bridge network (the threat-model evidence dds-discovery-test.txt),
# its sockets still listen on every interface, so a peer that sets
# ROS_STATIC_PEERS to this host's address joins the graph and reads and
# publishes topics, and the shared-memory transport it adds loses every
# sample between containers, because each container has its own /dev/shm.
# And LOCALHOST on top of the profile is worse than the profile alone:
# measured on the GPU host (2026-11-01) with Isaac Sim 6.1's Fast DDS 2.14.6
# and ros:jazzy's, it makes rmw_fastrtps open two more send sockets, on
# the wildcard address and on the host's own address, which run_sim.sh's listener check
# refuses (T44). With the profile alone, every socket is on loopback.
# FASTDDS_BUILTIN_TRANSPORTS=UDPv4 stays for the same shared-memory reason
# (it applies when a container runs without the profile).
SPOT_FASTDDS_PROFILE_PATH=/etc/spot/fastdds-loopback.xml   # inside the container
SPOT_FASTDDS_PROFILE_XML='<?xml version="1.0" encoding="UTF-8" ?>
<!-- Written by host/sim_lib.sh (spot_fastdds_profile); do not edit here. -->
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>spot_udp_loopback</transport_id>
      <type>UDPv4</type>
      <interfaceWhiteList><address>127.0.0.1</address></interfaceWhiteList>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="spot_loopback" is_default_profile="true">
    <rtps>
      <userTransports><transport_id>spot_udp_loopback</transport_id></userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
      <defaultUnicastLocatorList>
        <locator><udpv4><address>127.0.0.1</address></udpv4></locator>
      </defaultUnicastLocatorList>
      <builtin>
        <metatrafficUnicastLocatorList>
          <locator><udpv4><address>127.0.0.1</address></udpv4></locator>
        </metatrafficUnicastLocatorList>
        <initialPeersList>
          <locator><udpv4><address>127.0.0.1</address></udpv4></locator>
        </initialPeersList>
      </builtin>
    </rtps>
  </participant>
</profiles>'

# Kept outside every container mount that is writable (only the scripts'
# user writes $SPOT_DEMO_DIR/dds), so no container can loosen it for the next
# launch. Rewritten only when its content differs.
spot_fastdds_profile() {
  local dir="$SPOT_DEMO_DIR/dds" f tmp
  f="$dir/fastdds-loopback.xml"
  if [ -f "$f" ] && [ ! -L "$f" ] && [ "$(cat "$f")" = "$SPOT_FASTDDS_PROFILE_XML" ]; then
    echo "$f"
    return 0
  fi
  if ! { mkdir -p "$dir" && tmp="$(mktemp "$dir/.fastdds.XXXXXX")"; } 2>/dev/null; then
    echo "FATAL: cannot write the Fast DDS profile under $dir as $(id -un)." \
         "Fix: sudo mkdir -p $dir && sudo chown ubuntu:ubuntu $dir" >&2
    return 1
  fi
  if ! { printf '%s\n' "$SPOT_FASTDDS_PROFILE_XML" > "$tmp" && chmod 0644 "$tmp" && mv -f "$tmp" "$f"; }; then
    rm -f "$tmp"
    echo "FATAL: cannot write $f as $(id -un)" >&2
    return 1
  fi
  # Written by root (SSM): hand it to the demo dir's owner (ubuntu), as logs/.
  if [ "$(id -u)" = 0 ] && [ -d "$SPOT_DEMO_DIR" ]; then
    chown --reference="$SPOT_DEMO_DIR" "$dir" "$f" 2>/dev/null || true
  fi
  echo "$f"
}

spot_dds_args() {
  local domain="${ROS_DOMAIN_ID:-42}" profile
  if ! [[ "$domain" =~ ^(0|[1-9][0-9]{0,2})$ ]] || [ "$domain" -gt 232 ]; then
    echo "FATAL: ROS_DOMAIN_ID '$domain' is not a ROS 2 domain id (0-232)" >&2
    return 1
  fi
  if [ "$domain" = 0 ]; then
    echo "WARN: ROS_DOMAIN_ID is 0, the default domain of every ROS 2 install (unset it for 42)" >&2
  fi
  profile="$(spot_fastdds_profile)" || return 1
  # shellcheck disable=SC2034  # read by the scripts that source this file
  SPOT_DDS_ARGS=(-e "ROS_DOMAIN_ID=$domain"
                 -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4
                 -v "$profile:$SPOT_FASTDDS_PROFILE_PATH:ro"
                 -e "FASTRTPS_DEFAULT_PROFILES_FILE=$SPOT_FASTDDS_PROFILE_PATH"
                 -e "FASTDDS_DEFAULT_PROFILES_FILE=$SPOT_FASTDDS_PROFILE_PATH")
}

# Hardened options for every container except the sim (run_sim.sh says why
# the sim keeps its own). --writable-root is for the Playwright helper only:
# npx installs and runs its CLI from $HOME.
spot_container_args() {
  local kind="${1:-}" uid
  case "$kind" in
    agent) uid=$SPOT_AGENT_UID ;;
    hud) uid=$SPOT_HUD_UID ;;
    oneshot) uid=$SPOT_ONESHOT_UID ;;
    *) echo "spot_container_args: unknown kind '$kind'" >&2; return 2 ;;
  esac
  spot_require_imds_guard "start a $kind container" || return 1
  spot_dds_args || return 1
  # shellcheck disable=SC2034  # read by the scripts that source this file
  SPOT_ARGS=(--user "$uid:$uid" --cap-drop=ALL --security-opt no-new-privileges
             -e HOME=/tmp -e ROS_HOME=/tmp/.ros "${SPOT_DDS_ARGS[@]}")
  if [ "${2:-}" != --writable-root ]; then
    SPOT_ARGS+=(--read-only --tmpfs "/tmp:rw,nosuid,nodev,noexec,size=64m")
  fi
  # Only the agent calls AWS (Bedrock). Every other container is told not to
  # use the instance metadata service (T26). This stops SDK clients only
  # (boto3, the AWS CLI): any process can still send a raw HTTP request to
  # 169.254.169.254. The real controls are the documented instance-role
  # policy (no secret reads) and host/imds_guard.sh, which blocks IMDS for
  # every uid but root and the agent's.
  if [ "$kind" != agent ]; then
    SPOT_ARGS+=(-e AWS_EC2_METADATA_DISABLED=true)
  fi
}

spot_shared_dir_gid() {
  local d="$1"
  # Sticky as well as group-writable: a container that writes there through
  # the group cannot rename, delete or replace a file it does not own, so it
  # cannot swap a host script's file for a symlink once it exists. The host
  # scripts create their own files there with O_EXCL (noclobber), so a link
  # planted under a name before they start is refused, not followed.
  if ! { [ -d "$d" ] && chmod g+rwx,+t "$d"; } 2>/dev/null; then
    echo "FATAL: cannot make $d group-writable as $(id -un) (owner: $(stat -c %U "$d" 2>/dev/null))." \
         "Fix: sudo chown ubuntu:ubuntu $d" >&2
    return 1
  fi
  stat -c %g "$d"
}

# Loopback is 127.0.0.0/8, ::1, ::ffff:127.x, or any address bound to the lo
# device (%lo); the IPv4 and IPv6 wildcards (*) and every interface address are not.
spot_nonloopback_listeners() {
  local src
  if [ "${1:-}" = - ]; then
    src="$(cat)"
  else
    src="$(sudo ss -Hltnup 2>/dev/null)" || return 2
  fi
  printf '%s\n' "$src" | awk '
    NF >= 5 {
      addr = $5
      sub(/:[^:]*$/, "", addr)
      gsub(/^\[|\]$/, "", addr)
      if (addr ~ /%lo$/ || addr ~ /^127\./ || addr == "::1" || addr ~ /^::ffff:127\./) next
      name = "-"; pid = "-"
      for (i = 7; i <= NF; i++) {
        if (name == "-" && match($i, /"[^"]+"/)) name = substr($i, RSTART + 1, RLENGTH - 2)
        if (pid == "-" && match($i, /pid=[0-9]+/)) pid = substr($i, RSTART + 4, RLENGTH - 4)
      }
      print $1, $5, name, pid
    }'
}

# The sim carries a mission's gas and odometry: stopping or replacing it
# while the agent or a scripted mission runs leaves Spot without telemetry
# mid-entry (the guard raises a telemetry loss and the retreat ends
# incomplete). Every wrapper that stops or restarts the sim asks first (T41).
SPOT_AGENT_CONTAINERS_RE='spot-agent|spot-mission'

spot_running_agents() {
  local names
  names="$(docker ps --format '{{.Names}}' 2>/dev/null)" || return 0
  printf '%s\n' "$names" | grep -xE "$SPOT_AGENT_CONTAINERS_RE" | tr '\n' ' ' | sed 's/ $//' || true
}

spot_guard_sim_stop() {
  local force="$1" what="$2" running
  running="$(spot_running_agents)"
  [ -n "$running" ] || return 0
  if [ "$force" = 1 ]; then
    echo "WARN: --force: $what while $running is running; its mission loses gas and odometry" >&2
    return 0
  fi
  echo "FATAL: refusing to $what: $running is running and would lose the sim mid-mission." \
       "Finish or quit the session first (quit in the agent REPL; a scripted mission ends by itself)," \
       "or pass --force to do it anyway." >&2
  return 1
}

# Every container runs with --network=host, so without host/imds_guard.sh any
# of them can take the instance role's credentials from IMDS. Its rules live in
# the kernel and a reboot drops them, so every launch checks they are in place
# and refuses otherwise (fail closed). SPOT_IMDS_GUARD=off is for a host where
# the guard cannot run; it says what that costs (T26).
spot_require_imds_guard() {
  local what="$1" guard
  guard="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/imds_guard.sh"
  if [ "${SPOT_IMDS_GUARD:-on}" = off ]; then
    echo "WARN: SPOT_IMDS_GUARD=off: about to $what without the IMDS guard; every container" \
         "on this host can take the instance role's credentials (T26)" >&2
    return 0
  fi
  if ! bash "$guard" status >/dev/null 2>&1; then
    echo "FATAL: refusing to $what: the IMDS guard is not in place (a reboot removes it)." \
         "Run: sudo bash $guard apply  (or set SPOT_IMDS_GUARD=off to accept the risk, T26)" >&2
    return 1
  fi
}

# A smoke or e2e sim runs on its own DDS domain, so an agent, mission or HUD
# started on the demo's domain while the test runs finds no facade, instead of
# driving or reading the test's sim (T41).
spot_test_domain() {
  local demo="${ROS_DOMAIN_ID:-42}" test="${SPOT_TEST_DOMAIN_ID:-43}"
  if [ "$test" = "$demo" ]; then
    echo "FATAL: SPOT_TEST_DOMAIN_ID ($test) must differ from the demo's ROS_DOMAIN_ID ($demo)" >&2
    return 1
  fi
  export ROS_DOMAIN_ID="$test"
}

# Remove spot-sim only if this run launched it (its SPOT_RUN_ID), so a test's
# exit never takes down a sim someone started after it (T41).
spot_rm_own_sim() {
  local id="$1"
  if docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$SPOT_SIM_CONTAINER" 2>/dev/null \
      | grep -qx "SPOT_RUN_ID=$id"; then
    docker rm -f "$SPOT_SIM_CONTAINER" >/dev/null 2>&1 || true
  fi
}

spot_run_id() {
  printf '%s-%s%04d\n' "$(date -u +%Y%m%dT%H%M%SZ)" "$$" "$((RANDOM % 10000))"
}

spot_log_dir() {
  local d="$SPOT_LOG_DIR" me
  me="$(id -un)"
  if [ ! -d "$d" ]; then
    if ! mkdir -p "$d" 2>/dev/null; then
      echo "FATAL: cannot create the log directory $d as $me." \
           "Fix: sudo mkdir -p $d && sudo chown ubuntu:ubuntu $d" >&2
      return 1
    fi
    # Created by root (SSM): hand it to the demo dir's owner (ubuntu).
    if [ "$(id -u)" = 0 ] && [ -d "$SPOT_DEMO_DIR" ]; then
      chown --reference="$SPOT_DEMO_DIR" "$d" 2>/dev/null || true
    fi
  fi
  if [ ! -w "$d" ] || [ ! -x "$d" ]; then
    echo "FATAL: the log directory $d is not writable by $me (owner: $(stat -c %U "$d" 2>/dev/null))." \
         "Fix: sudo chown ubuntu:ubuntu $d" >&2
    return 1
  fi
  echo "$d"
}

spot_new_log() {
  local prefix="$1" id="$2" d f
  d="$(spot_log_dir)" || return 1
  f="$d/$prefix-$id.log"
  # noclobber: create a NEW file or fail; never reuse (or trust) an old one.
  # noclobber alone would still open an existing device or FIFO (or a link to
  # one) under that name, so any existing entry is refused first (T28).
  if [ -e "$f" ] || [ -L "$f" ] || ! ( set -C; : > "$f" ) 2>/dev/null; then
    echo "FATAL: cannot create a new log file $f as $(id -un)" \
         "($( [ -e "$f" ] && echo 'it already exists' || echo 'permission denied'))" >&2
    return 1
  fi
  if [ "$(id -u)" = 0 ]; then chown -h --reference="$d" "$f" 2>/dev/null || true; fi
  # Convenience pointer for humans (tail -f logs/sim-latest.log). Readiness
  # never reads it.
  if ln -sfn "$(basename "$f")" "$d/$prefix-latest.log" 2>/dev/null; then
    if [ "$(id -u)" = 0 ]; then chown -h --reference="$d" "$d/$prefix-latest.log" 2>/dev/null || true; fi
  else
    echo "WARN: could not update $d/$prefix-latest.log (continuing)" >&2
  fi
  echo "$f"
}

spot_sim_ready_line() {
  local want="${1:-}" running pat line
  running="$(docker inspect -f '{{.State.Running}}' "$SPOT_SIM_CONTAINER" 2>/dev/null)" || return 1
  [ "$running" = true ] || return 1
  if [ -n "$want" ]; then pat="^FACADE-READY .*run_id=${want}( |\$)"; else pat='^FACADE-READY '; fi
  # grep reads the whole stream (no -m1): an early exit would SIGPIPE docker
  # and, under pipefail, turn a match into a failure.
  line="$(docker logs "$SPOT_SIM_CONTAINER" 2>&1 | grep -E "$pat" | tail -n 1)" || true
  [ -n "$line" ] || return 1
  echo "$line"
}

spot_wait_sim_ready() {
  local id="$1" timeout="$2" log="$3" watch="${4:-}" end line
  end=$((SECONDS + timeout))
  while [ "$SECONDS" -lt "$end" ]; do
    if line="$(spot_sim_ready_line "$id")"; then
      echo "$line"
      return 0
    fi
    case "$watch" in
      pid:*)
        if ! kill -0 "${watch#pid:}" 2>/dev/null; then
          echo "FATAL: the sim launcher exited before FACADE-READY run_id=$id (log: $log). Last lines:" >&2
          tail -n 40 "$log" >&2
          return 1
        fi ;;
      cid:*)
        if [ "$(docker inspect -f '{{.State.Running}}' "${watch#cid:}" 2>/dev/null)" != true ]; then
          echo "FATAL: sim container ${watch#cid:} exited (or was replaced) before FACADE-READY run_id=$id (log: $log). Last lines:" >&2
          tail -n 40 "$log" >&2
          return 1
        fi ;;
    esac
    sleep 2
  done
  echo "FATAL: no FACADE-READY run_id=$id from container $SPOT_SIM_CONTAINER within ${timeout}s (log: $log). Last lines:" >&2
  tail -n 40 "$log" >&2
  return 1
}
