# Reproducible dev environment for the GPS-denied drone portfolio.
#
# Goals:
#   - ROS 2 Humble + Gazebo Harmonic + Python 3.10 toolchain
#   - colcon-buildable workspace
#   - Stable across host machines so Day 1 of the sprint cannot fail
#     because of mixed Ubuntu sources
#
# Not included (intentionally, to keep the image small):
#   - PX4-Autopilot source. Clone into a bind-mounted volume.
#   - ORB-SLAM3. Build per `contracts/vio_pipeline.md` inside the
#     container; cache `/root/orb-slam3-build/` as a volume.
#   - GPU drivers. Use `nvidia/cuda` base + nvidia-docker if you need
#     GPU inference locally; otherwise train on Brev.
#
# Build:   docker build -t gps-drone:humble .
# Run:     docker run --rm -it --net=host -v $PWD:/ws/src/gps_denied_drone \
#                     gps-drone:humble bash

FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Toolchain + build deps. opencv-contrib + Eigen are needed for the
# OpenCV-ORB feature density backend; the same package set covers
# ORB-SLAM3's prereqs.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git wget curl unzip pkg-config \
        python3-pip python3-colcon-common-extensions \
        libeigen3-dev libopencv-dev libopencv-contrib-dev \
        libpython3-dev python3-numpy \
        ros-humble-cv-bridge ros-humble-image-transport \
        ros-humble-tf2-ros ros-humble-rclpy \
    && rm -rf /var/lib/apt/lists/*

# Gazebo Harmonic + ros_gz bridge.
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
        | tee /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg > /dev/null \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        gz-harmonic ros-humble-ros-gzharmonic \
    && rm -rf /var/lib/apt/lists/*

# Python deps for the simulator + EuRoC eval + learning scaffold.
RUN pip install --no-cache-dir \
        numpy scipy matplotlib pyyaml casadi \
        opencv-python transforms3d \
        evo onnxruntime \
        pytest

# Workspace layout
RUN mkdir -p /ws/src
WORKDIR /ws

# (At runtime, bind-mount the repo onto /ws/src/gps_denied_drone and
# build with: `colcon build --symlink-install`.)
CMD ["bash"]
