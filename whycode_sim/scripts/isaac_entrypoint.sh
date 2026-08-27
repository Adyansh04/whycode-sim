#!/bin/bash
#
# Container entrypoint for the Isaac Sim service. Replaces the image's runheadless.sh so
# the simulation starts from our scene builder. The image is unmodified; everything
# project-specific arrives through the /root/workspace bind mount.
#
# Usage: isaac_entrypoint.sh [args passed through to run_sim.py]

set -euo pipefail

WORKSPACE="${WORKSPACE:-/root/workspace}"
PACKAGE_DIR="${WORKSPACE}/src/whycode_sim/whycode_sim"
RUN_SIM="${PACKAGE_DIR}/whycode_sim/isaac/run_sim.py"

# No system ROS 2 or python3 in this image; everything runs through Isaac's bundled
# interpreter. It ships a full ROS 2 under isaacsim.ros2.core built for the same Python
# 3.12 as ros-dev, which is what lets our message package load here (rclpy 7.1.11, jazzy).
ISAAC_ROS="/isaac-sim/exts/isaacsim.ros2.core/${ROS_DISTRO:-jazzy}"
if [[ ! -d "${ISAAC_ROS}" ]]; then
    echo "[isaac] no bundled ROS 2 for '${ROS_DISTRO:-jazzy}' at ${ISAAC_ROS}" >&2
    echo "[isaac] available: $(/bin/ls -1 /isaac-sim/exts/isaacsim.ros2.core 2>/dev/null | tr '\n' ' ')" >&2
    exit 1
fi

export PYTHONPATH="${ISAAC_ROS}/rclpy:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="${ISAAC_ROS}/lib:${LD_LIBRARY_PATH:-}"
export AMENT_PREFIX_PATH="${ISAAC_ROS}:${AMENT_PREFIX_PATH:-}"

# Overlay the colcon workspace so whycode_sim_msgs resolves. It is built in ros-dev;
# this container only reads it.
MSGS_PREFIX="${WORKSPACE}/install/whycode_sim_msgs"
if [[ -d "${MSGS_PREFIX}" ]]; then
    export PYTHONPATH="${MSGS_PREFIX}/lib/python3.12/site-packages:${PYTHONPATH}"
    export LD_LIBRARY_PATH="${MSGS_PREFIX}/lib:${LD_LIBRARY_PATH}"
    export AMENT_PREFIX_PATH="${MSGS_PREFIX}:${AMENT_PREFIX_PATH}"
else
    echo "[isaac] ${MSGS_PREFIX} not found. Build it first, from ros-dev:" >&2
    echo "[isaac]   colcon build --packages-select whycode_sim_msgs whycode_sim" >&2
    exit 1
fi

# Shared config and geometry modules, imported by both containers from the same path.
export PYTHONPATH="${PACKAGE_DIR}:${PYTHONPATH}"

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

if [[ ! -f "${RUN_SIM}" ]]; then
    echo "[isaac] ${RUN_SIM} not found; is ./workspace mounted?" >&2
    exit 1
fi

echo "[isaac] ROS 2      : ${ISAAC_ROS}"
echo "[isaac] messages   : ${MSGS_PREFIX}"
echo "[isaac] domain id  : ${ROS_DOMAIN_ID:-0}"
echo "[isaac] headless   : ${ISAAC_HEADLESS:-1}"

cd /isaac-sim
exec ./python.sh "${RUN_SIM}" "$@"
