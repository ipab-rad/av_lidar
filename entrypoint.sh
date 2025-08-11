#!/usr/bin/env bash
# shellcheck disable=SC1090,SC1091

# Get the user ID and group ID of the local user
USER_ID=${LOCAL_UID}
USER_NAME=${LOCAL_USER}
GROUP_ID=${LOCAL_GID}
GROUP_NAME=${LOCAL_GROUP}

# Check if any of the variables are empty
if [[ -z $USER_ID || -z $USER_NAME || -z $GROUP_ID || -z $GROUP_NAME ]]; then
    source "/opt/ros/$ROS_DISTRO/setup.bash"
    [ -f /opt/ros_ws/install/setup.bash ] && source /opt/ros_ws/install/setup.bash

    # If no command is provided, start an interactive bash session
    if [ $# -eq 0 ]; then
        exec bash
    else
        exec "$@"
    fi

else
    echo "Starting with user: $USER_NAME >> UID $USER_ID, GID: $GROUP_ID"

    # Create group and user with GID/UID
    groupadd -g "$GROUP_ID" "$GROUP_NAME"
    useradd -u "$USER_ID" -g "$GROUP_ID" -s /bin/bash -m -d /home/"$USER_NAME" "$USER_NAME"

    # Add sudo privileges to the user
    echo "$USER_NAME ALL=(ALL) NOPASSWD:ALL" >>/etc/sudoers

    # Ensure the user owns ROS_WS
    chown -R "$USER_NAME":"$USER_NAME" "$ROS_WS"

    # Source ROS2
    # hadolint ignore=SC1090
    source "/opt/ros/$ROS_DISTRO/setup.bash"
    [ -f /opt/ros_ws/install/setup.bash ] && source /opt/ros_ws/install/setup.bash

    if [ $# -eq 0 ]; then
        exec gosu "$USER_NAME" bash
    else
        exec gosu "$USER_NAME" "$@"
    fi
fi
