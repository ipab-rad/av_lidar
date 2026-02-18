FROM ros:humble-ros-base-jammy AS base

ENV NEBULA_VERSION=0.2.15.3

# Install basic dev tools (And clean apt cache afterwards)
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive \
        apt-get -y --quiet --no-install-recommends install \
        wget \
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
RUN wget --progress=dot:giga https://github.com/tier4/nebula/archive/refs/tags/v$NEBULA_VERSION.tar.gz \
    && tar -xvzf v$NEBULA_VERSION.tar.gz
WORKDIR $ROS_WS/src/nebula-$NEBULA_VERSION
RUN vcs import < build_depends.repos \
    && apt-get update \
    && DEBIAN_FRONTEND=noninteractive \
        rosdep install --from-paths . --ignore-src -y -r \
    && rm -rf /var/lib/apt/lists/*

# Source ROS setup for dependencies and build our code
WORKDIR $ROS_WS
RUN . /opt/ros/"$ROS_DISTRO"/setup.sh \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# -----------------------------------------------------------------------

FROM base AS prebuilt

# Copy nebula artifacts/binaries from base to avoid re-compiling them
RUN mkdir -p "$ROS_WS"/install
COPY --from=base $ROS_WS/install "$ROS_WS"/install
RUN mkdir -p "$ROS_WS"/build
COPY --from=base "$ROS_WS"/build "$ROS_WS"/build

# Import av_lidar_launch
COPY ./ "$ROS_WS"/src/av_lidar_launch

# Source ROS setup for dependencies and build our code
RUN . /opt/ros/"$ROS_DISTRO"/setup.sh \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# -----------------------------------------------------------------------

FROM base AS dev

# Copy prebuild nebula ros driver from base
RUN mkdir -p "$ROS_WS"/install
COPY --from=base "$ROS_WS"/install "$ROS_WS"/install
RUN mkdir -p "$ROS_WS"/build
COPY --from=base "$ROS_WS"/build "$ROS_WS"/build
RUN mkdir -p "$ROS_WS"/log
COPY --from=base "$ROS_WS"/log "$ROS_WS"/log

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
COPY --from=prebuilt "$ROS_WS"/src "$ROS_WS"/src
COPY --from=prebuilt "$ROS_WS"/install "$ROS_WS"/install
COPY --from=prebuilt "$ROS_WS"/build "$ROS_WS"/build

# Add command to docker entrypoint to source newly compiled
#   code when running docker container
RUN sed --in-place --expression \
        "\$isource \"$ROS_WS/install/setup.bash\" " \
        /ros_entrypoint.sh

# launch ros package
CMD ["ros2", "launch", "av_lidar_launch", "all_lidars.launch.xml"]
