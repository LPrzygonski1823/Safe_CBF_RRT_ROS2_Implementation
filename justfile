set dotenv-load

# simulation stack plus the Safe-CBF-RRT* planner overlay
compose_cbf := "docker compose -f compose.simulation.yaml -f compose.cbf.yaml"

[private]
default:
    @just --list --unsorted

[private]
alias husarnet := connect-husarnet
[private]
alias flash := flash-firmware
[private]
alias rosbot := start-rosbot

[private]
gazebo: (start-simulation "gazebo")

[private]
webots: (start-simulation "webots")

[private]
pre-commit:
    #!/bin/bash
    if ! command -v pre-commit &> /dev/null; then
        pip install pre-commit
        pre-commit install
    fi
    pre-commit run -a

# connect to Husarnet VPN network
connect-husarnet joincode hostname: _run-as-root
    #!/bin/bash
    if ! command -v husarnet > /dev/null; then
        echo "Husarnet is not installed. Installing now..."
        curl https://install.husarnet.com/install.sh | bash
    fi
    husarnet join {{joincode}} {{hostname}}

# Copy repo content to remote host with 'rsync' and watch for changes
sync hostname="${ROBOT_NAMESPACE}" password="husarion": _install-rsync _run-as-user
    #!/bin/bash
    mkdir -m 775 -p maps
    sshpass -p "{{password}}" rsync -vRr --exclude='.git/' --exclude='maps/' --delete ./ husarion@{{hostname}}:/home/husarion/${PWD##*/}
    while inotifywait -r -e modify,create,delete,move ./ --exclude='.git/' --exclude='maps/' ; do
        sshpass -p "{{password}}" rsync -vRr --exclude='.git/' --exclude='maps/' --delete ./ husarion@{{hostname}}:/home/husarion/${PWD##*/}
    done

# flash the proper firmware for STM32 microcontroller in ROSbot XL
flash-firmware: _install-yq _run-as-user
    #!/bin/bash
    echo "Stopping all running containers"
    docker ps -q | xargs -r docker stop

    echo "Flashing the firmware for STM32 microcontroller in ROSbot"
    docker run \
        --rm -it \
        --device /dev/ttyUSBDB \
        --device /dev/bus/usb/ \
        $(yq .services.rosbot.image compose.yaml) \
        flash-firmware.py -p /dev/ttyUSBDB # todo
        # ros2 run rosbot_utils flash_firmware

# start containers on a physical ROSbot XL
start-rosbot: _run-as-user
    #!/bin/bash
    mkdir -m 775 -p maps
    docker compose down
    docker compose pull
    docker compose up

# start the simulation (available options: gazebo, webots)
start-simulation engine="gazebo": _run-as-user
    #!/bin/bash
    xhost +local:docker
    if [[ "{{engine}}" == "gazebo" ]]; then
        export SIMULATION_DOCKER_IMAGE="husarion/rosbot-xl-gazebo:humble-0.9.1-20240131"
        export SIMULATION_COMMAND="ros2 launch rosbot_xl_gazebo simulation.launch.py mecanum:=${MECANUM:-True}"
    elif [[ "{{engine}}" == "webots" ]]; then
        export SIMULATION_DOCKER_IMAGE="husarion/webots:humble-2023.0.4-20230809-stable"
        export SIMULATION_COMMAND="ros2 launch webots_ros2_husarion rosbot_xl_launch.py"
    else
        echo -e "\e[1;33mUnknown ROS 2 simulation engine: {{engine}}\e[0m"
        exit 1
    fi
    docker compose -f compose.simulation.yaml down
    docker compose -f compose.simulation.yaml pull
    docker compose -f compose.simulation.yaml up

# Restart the Nav2 container
restart-navigation: _run-as-user
    #!/bin/bash
    docker compose down navigation
    docker compose up -d navigation

# build every package in the workspace inside the dev container
# --symlink-install keeps config/ and launch/ as symlinks into src/, so editing a params file
# takes effect on the next node start instead of needing a rebuild to be copied
cbf-build: _run-as-user
    #!/bin/bash
    set -e
    {{compose_cbf}} run --rm --no-deps cbf-planner bash -c "\
        source /opt/ros/humble/setup.bash && \
        colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# wipe build artifacts and build from scratch, for when a stale CMake cache is suspected
cbf-rebuild: _run-as-user
    #!/bin/bash
    set -e
    {{compose_cbf}} run --rm --no-deps cbf-planner bash -c "\
        rm -rf /workspace/build /workspace/install /workspace/log && \
        source /opt/ros/humble/setup.bash && \
        colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# build, then start the simulation, Nav2, Foxglove and the planner in one go
# detached on purpose: Ctrl-C only stops following the logs, the stack keeps running
cbf-up: cbf-build _run-as-user
    #!/bin/bash
    set -e
    xhost +local:docker
    {{compose_cbf}} up -d
    # plain `up -d` leaves an already running container alone, which would keep executing the
    # binary loaded before the build above - recreating the planner guarantees the fresh one
    {{compose_cbf}} up -d --force-recreate --no-deps cbf-planner
    {{compose_cbf}} logs -f cbf-planner

# rebuild the planner and restart only its container (simulation keeps running)
cbf-restart: cbf-build _run-as-user
    #!/bin/bash
    set -e
    {{compose_cbf}} up -d --force-recreate --no-deps cbf-planner
    {{compose_cbf}} logs -f cbf-planner

# freeze the current map and (re)generate the PSF
cbf-refresh-map: _run-as-user
    #!/bin/bash
    {{compose_cbf}} exec cbf-planner bash -c "\
        source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && \
        ros2 service call /refresh_map std_srvs/srv/Trigger"

# send a planning goal, e.g. `just cbf-goal 2.5 -1.0`
cbf-goal x y: _run-as-user
    #!/bin/bash
    {{compose_cbf}} exec cbf-planner bash -c "\
        source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && \
        ros2 topic pub --once /cbf_goal_pose geometry_msgs/msg/PoseStamped \
        '{header: {frame_id: map}, pose: {position: {x: {{x}}, y: {{y}}, z: 0.0}, orientation: {w: 1.0} } }'"

# interactive ROS 2 shell on the stack network (ros2 topic/node/tf debugging)
cbf-shell: _run-as-user
    #!/bin/bash
    {{compose_cbf}} run --rm --no-deps cbf-planner bash

# follow the planner logs
cbf-logs: _run-as-user
    #!/bin/bash
    {{compose_cbf}} logs -f cbf-planner

# stop the whole stack including the planner
cbf-down: _run-as-user
    #!/bin/bash
    {{compose_cbf}} down

_run-as-root:
    #!/bin/bash
    if [ "$EUID" -ne 0 ]; then
        echo -e "\e[1;33mPlease re-run as root user to install dependencies\e[0m"
        exit 1
    fi

_run-as-user:
    #!/bin/bash
    if [ "$EUID" -eq 0 ]; then
        echo -e "\e[1;33mPlease re-run as non-root user\e[0m"
        exit 1
    fi

_install-rsync:
    #!/bin/bash
    if ! command -v rsync &> /dev/null || ! command -v sshpass &> /dev/null || ! command -v inotifywait &> /dev/null; then
        if [ "$EUID" -ne 0 ]; then
            echo -e "\e[1;33mPlease run as root to install dependencies\e[0m"
            exit 1
        fi
        apt install -y rsync sshpass inotify-tools
    fi

_install-yq:
    #!/bin/bash
    if ! command -v /usr/bin/yq &> /dev/null; then
        if [ "$EUID" -ne 0 ]; then
            echo -e "\e[1;33mPlease run as root to install dependencies\e[0m"
            exit 1
        fi

        YQ_VERSION=v4.35.1
        ARCH=$(arch)

        if [ "$ARCH" = "x86_64" ]; then
            YQ_ARCH="amd64"
        elif [ "$ARCH" = "aarch64" ]; then
            YQ_ARCH="arm64"
        else
            YQ_ARCH="$ARCH"
        fi

        curl -L https://github.com/mikefarah/yq/releases/download/${YQ_VERSION}/yq_linux_${YQ_ARCH} -o /usr/bin/yq
        chmod +x /usr/bin/yq
        echo "yq installed successfully!"
    fi