# Operator agent image: Strands Agents + Bedrock driving the Spot facade over ROS 2.
#
# Build from the REPO ROOT (context must contain agent/ and tests/):
#   docker build -f agent/Dockerfile -t spot-agent .
# Run it on the sim host with host/run_agent.sh, which adds the DDS isolation
# every container shares (host/sim_lib.sh spot_dds_args, T25). By hand, from
# /opt/spot-demo (REPL needs -it; $PROFILE is the file spot_dds_args writes,
# dds/fastdds-loopback.xml, which binds DDS to 127.0.0.1):
#   docker run --rm -it --name spot-agent --network=host \
#     -e ROS_DOMAIN_ID=42 \
#     -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
#     -v "$PROFILE:/etc/spot/fastdds-loopback.xml:ro" \
#     -e FASTRTPS_DEFAULT_PROFILES_FILE=/etc/spot/fastdds-loopback.xml \
#     -e AWS_REGION=us-west-2 -v "$PWD/logs:/workspace/logs" spot-agent
#   (the same options) spot-agent --once "Run the full confined space entry"
#   add -e SPOT_DEMO_PRESENTER=1 only for the presenter's session (gas-spike tool).
#
# AWS credentials come from the instance role via IMDSv2 (--network=host keeps
# the IMDS hop count at 1); the role needs bedrock:InvokeModel* on the
# us.anthropic.claude-sonnet-5 inference profile. No keys are baked in.
# ros:jazzy-ros-base pinned by digest (T31): the multi-arch index digest of
# that tag on 2026-10-29, the same as SPOT_ROS_IMAGE in host/sim_lib.sh. From
# ECR Public's mirror of the Docker official image: the mirror's index digest
# equals docker.io/library/ros:jazzy-ros-base's (checked 2026-10-29), so the
# image is the same one the Docker Hub tag pointed to.
FROM public.ecr.aws/docker/library/ros:jazzy-ros-base@sha256:00b8d47a6e6384a2834e3d01fa87ae767876543baa574f2d64a75c04ea33c698

# The ROS base carries the C/C++ toolchain, and with it the kernel headers
# (linux-libc-dev): over 300 High and Critical advisories with no fix in
# Ubuntu 24.04 (T32). Nothing here compiles: Python comes as wheels only,
# so the headers are removed. --force-depends leaves libc6-dev and the
# compilers installed but unusable for kernel-facing code; nothing runs them.
RUN apt-get update \
 && apt-get install -y --no-install-recommends python3-venv python3-numpy \
 && dpkg --purge --force-depends linux-libc-dev \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
# The hash lock first so code edits don't invalidate the pip layer. Every
# package is pinned with its hashes and installed as a wheel only, with no
# dependency resolution: pip installs exactly the lock or fails (T31).
# pip itself is then removed from the venv: nothing at run time needs it.
COPY agent/requirements.lock /workspace/agent/requirements.lock
# A venv that still sees the apt/ROS Python packages (rclpy deps, numpy,
# yaml). NOT `pip install --break-system-packages` into the system Python:
# strands-agents needs typing_extensions>=4.13.2, Ubuntu 24.04 ships 4.10 as a
# Debian package without a RECORD file, and pip aborts with "Cannot uninstall
# typing_extensions 4.10.0, RECORD file not found" (reproduced locally with a
# RECORD-less 4.10 and pip 24.0). Inside the venv pip installs the newer copy
# alongside and leaves Debian's untouched ("outside environment").
RUN python3 -m venv --system-site-packages /opt/agent-venv \
 && /opt/agent-venv/bin/pip install --no-cache-dir --require-hashes --no-deps \
      --only-binary :all: -r /workspace/agent/requirements.lock \
 && /opt/agent-venv/bin/python3 -m pip uninstall -y -q pip
COPY agent/ /workspace/agent/
# `--dry-run` (no ROS needed, still calls Bedrock): the kinematic FakeRobot in
# tests/test_mission.py and the sim's gas model (pure Python; numpy from apt).
COPY tests/test_agent_tools.py tests/test_mission.py /workspace/tests/
COPY sim/gas_field.py /workspace/sim/gas_field.py

# Non-root (T28): uid/gid 10001, SPOT_AGENT_UID in host/sim_lib.sh, which
# host/imds_guard.sh lets reach the instance metadata service. The image's
# files stay root-owned and read-only to it. host/run_agent.sh mounts the
# host's logs/ at /workspace/logs and adds that dir's group (--group-add), so
# the mission JSONL is written there; ROS writes its own logs under /tmp.
RUN groupadd --gid 10001 spot-agent \
 && useradd --uid 10001 --gid 10001 --no-create-home --home-dir /tmp \
      --shell /usr/sbin/nologin spot-agent

# Fast DDS shared-memory transport silently drops samples between containers
# running as different uids (Isaac runs as uid 1234): pin UDPv4, same as
# host/run_sim.sh and hud/Dockerfile.
ENV FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    HOME=/tmp \
    ROS_HOME=/tmp/.ros
USER 10001:10001

# Source ROS (PYTHONPATH for rclpy), then exec the venv's python3 (same
# /usr/bin/python3.12 underneath, so rclpy's C extensions load) so it replaces
# bash as PID 1 and receives SIGTERM from `docker stop` (spot_agent.py turns
# it into a clean stop).
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && exec /opt/agent-venv/bin/python3 /workspace/agent/spot_agent.py \"$@\"", "spot_agent"]
CMD []
