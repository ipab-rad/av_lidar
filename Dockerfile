FROM ros:humble-ros-base-jammy AS base

ENV NEBULA_VERSION=0.2.9

# Install basic dev tools (And clean apt cache afterwards)
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive \
        apt-get -y --quiet --no-install-recommends install \
        # git \
        # gosu \
        wget \
        # nlohmann-json3-dev \
        # ros-"$ROS_DISTRO"-pcl-ros \
        # ros-"$ROS_DISTRO"-tf2-eigen \
        # ros-"$ROS_DISTRO"-mcap-vendor \
        # ros-"$ROS_DISTRO"-rosbag2-storage-mcap \
        # ros-"$ROS_DISTRO"-velodyne-msgs \
        # ros-"$ROS_DISTRO"-udp-msgs \
        # ros-"$ROS_DISTRO"-angles \
        # ros-"$ROS_DISTRO"-diagnostic-updater \
        # ros-"$ROS_DISTRO"-radar-msgs \
        # ros-"$ROS_DISTRO"-can-msgs \
        # Install Cyclone DDS ROS RMW
        ros-"$ROS_DISTRO"-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS workspace folder
ENV ROS_WS=/opt/ros_ws
WORKDIR $ROS_WS

# Set cyclone DDS ROS RMW
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

COPY ./cyclone_dds.xml $ROS_WS/

# Configure Cyclone cfg file
ENV CYCLONEDDS_URI=file://${ROS_WS}/cyclone_dds.xml

# Enable ROS log colorised output
ENV RCUTILS_COLORIZED_OUTPUT=1

COPY ./entrypoint.sh /

# Download and setup Nebula Driver
WORKDIR $ROS_WS/src
RUN wget https://github.com/tier4/nebula/archive/refs/tags/v$NEBULA_VERSION.tar.gz
RUN tar -xvzf v$NEBULA_VERSION.tar.gz
WORKDIR $ROS_WS/src/nebula-$NEBULA_VERSION
RUN vcs import < build_depends.repos
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive \
        rosdep install --from-paths . --ignore-src -y -r
WORKDIR $ROS_WS/src

# -----------------------------------------------------------------------

FROM base AS prebuilt

# Import av_lidar_launch
COPY ./ $ROS_WS/src/av_lidar_launch

# Source ROS setup for dependencies and build our code
RUN . /opt/ros/"$ROS_DISTRO"/setup.sh \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# -----------------------------------------------------------------------

FROM base AS dev

# Copy artifacts/binaries from prebuilt
COPY --from=prebuilt $ROS_WS/install $ROS_WS/install

# Install basic dev tools (And clean apt cache afterwards)
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive \
        apt-get -y --quiet --no-install-recommends install \
        # Command-line editor
        nano \
        # Ping network tools
        inetutils-ping \
        # Bash auto-completion for convenience
        bash-completion \
        # RVIZ
        ros-"$ROS_DISTRO"-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Add sourcing local workspace command to bashrc for
#    convenience when running interactively
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc && \
    # Add colcon build alias for convenience
    echo 'alias colcon_build="colcon build --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release && \
        source install/setup.bash"' >> /etc/bash.bashrc

# Enter bash for development
# CMD ["bash"]
ENTRYPOINT [ "/entrypoint.sh" ]

# -----------------------------------------------------------------------

FROM base AS runtime

# Copy artifacts/binaries from prebuilt
COPY --from=prebuilt $ROS_WS/install $ROS_WS/install


# launch ros package
ENTRYPOINT [ "/entrypoint.sh" ]

