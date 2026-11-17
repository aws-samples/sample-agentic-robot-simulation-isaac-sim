# Presenter HUD image: FastAPI/uvicorn dashboard + read-only rclpy bridge.
#
# Build from the REPO ROOT (context must contain hud/):
#   docker build -f hud/Dockerfile -t spot-hud .
# Run it on the sim host with host/run_hud.sh, which adds the DDS isolation
# every container shares (host/sim_lib.sh spot_dds_args, T25). By hand, from
# /opt/spot-demo ($PROFILE is the file spot_dds_args writes,
# dds/fastdds-loopback.xml, which binds DDS to 127.0.0.1):
#   docker run --rm --name spot-hud --network=host \
#     -e ROS_DOMAIN_ID=42 \
#     -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
#     -v "$PROFILE:/etc/spot/fastdds-loopback.xml:ro" \
#     -e FASTRTPS_DEFAULT_PROFILES_FILE=/etc/spot/fastdds-loopback.xml \
#     -v "$PWD/agent:/workspace/agent:ro" spot-hud
#
# --network=host + --host 127.0.0.1: the SSM port-forwarding session reaches
# the host loopback, so the HUD never listens on an external interface.
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
 && apt-get install -y --no-install-recommends python3-venv \
 && dpkg --purge --force-depends linux-libc-dev \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
# The hash lock first so code edits don't invalidate the pip layer. Every
# package is pinned with its hashes and installed as a wheel only, with no
# dependency resolution: pip installs exactly the lock or fails (T31).
# pip itself is then removed from the venv: nothing at run time needs it.
COPY hud/requirements.lock /workspace/hud/requirements.lock
# A venv that still sees the apt/ROS Python packages (rclpy + its deps), same
# as agent/Dockerfile. NOT `pip install --break-system-packages` into the
# system Python: fastapi/pydantic need a newer typing_extensions than
# Ubuntu 24.04's Debian-packaged 4.10, which has no RECORD file, so pip aborts
# with "Cannot uninstall typing_extensions". Inside the venv pip installs the
# newer copy alongside and leaves Debian's untouched.
RUN python3 -m venv --system-site-packages /opt/hud-venv \
 && /opt/hud-venv/bin/pip install --no-cache-dir --require-hashes --no-deps \
      --only-binary :all: -r /workspace/hud/requirements.lock \
 && /opt/hud-venv/bin/python3 -m pip uninstall -y -q pip
COPY hud/ /workspace/hud/

# Non-root (T28): uid/gid 10002, SPOT_HUD_UID in host/sim_lib.sh. The HUD
# writes nothing: host/run_hud.sh runs it with a read-only root filesystem,
# agent/ mounted read-only and ROS's own logs under /tmp.
RUN groupadd --gid 10002 spot-hud \
 && useradd --uid 10002 --gid 10002 --no-create-home --home-dir /tmp \
      --shell /usr/sbin/nologin spot-hud

# Fast DDS shared-memory transport silently drops samples between containers
# running as different uids (Isaac runs as uid 1234): pin UDPv4, same as
# host/run_sim.sh.
ENV FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    HOME=/tmp \
    ROS_HOME=/tmp/.ros
USER 10002:10002

EXPOSE 8080
# ros_entrypoint.sh (inherited ENTRYPOINT) sources /opt/ros/jazzy first; the
# venv's python3 is the same /usr/bin/python3.12, so rclpy's C extensions load.
CMD ["/opt/hud-venv/bin/python3", "/workspace/hud/server.py", "--host", "127.0.0.1", "--port", "8080"]
