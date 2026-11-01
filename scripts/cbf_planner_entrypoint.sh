#!/bin/bash
# Startup for the cbf-planner container (Humble simulation and Jazzy robot).
# Kept as a script rather than an inline compose command because YAML folded scalars turn
# indented continuation lines into real newlines, which breaks multi-line shell logic.
set -e

if [ ! -f /workspace/install/setup.bash ]; then
    echo "workspace not built - run: just cbf-build (simulation) or just -f justfile.jazzy cbf-real-build (robot)"
    exit 1
fi

# ROS_DISTRO is set by the base image (humble in the simulation, jazzy on the robot)
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /workspace/install/setup.bash

# must match the Nav2 container: True in Gazebo (/clock), False on the physical robot
exec ros2 launch cbf_rrt_planner cbf_planner.launch.py \
    "use_sim_time:=${USE_SIM_TIME:-True}"
