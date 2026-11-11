# Native public-source verification only, not a replacement cross/deploy pipeline.
FROM ros:humble-ros-base-jammy
USER root
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
    build-essential cmake git unzip python3-colcon-common-extensions python3-pytest python3-yaml iproute2 util-linux openssh-client \
    libyaml-cpp-dev libspdlog-dev libgtest-dev \
    ros-humble-ament-cmake-gtest ros-humble-ament-cmake-pytest \
    ros-humble-moveit-ros-move-group ros-humble-moveit-kinematics \
    ros-humble-moveit-planners-ompl ros-humble-moveit-simple-controller-manager \
    ros-humble-moveit-ros-visualization ros-humble-pilz-industrial-motion-planner \
    ros-humble-controller-manager ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller ros-humble-gripper-controllers \
    ros-humble-xacro ros-humble-rviz2 \
 && rm -rf /var/lib/apt/lists/*
WORKDIR /work
