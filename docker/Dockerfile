# Simulation / development image: Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic.
# The official ros image is multi-architecture, so this builds natively on Apple Silicon
# Macs (arm64) as well as on Linux PCs and servers (amd64).
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO

ENV DEBIAN_FRONTEND=noninteractive
# libegl1 + libgl1-mesa-dri: software OpenGL so Gazebo can render the lidar and camera without a GPU.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-gz \
      ros-${ROS_DISTRO}-xacro \
      ros-${ROS_DISTRO}-robot-state-publisher \
      ros-${ROS_DISTRO}-teleop-twist-keyboard \
      ros-${ROS_DISTRO}-slam-toolbox \
      ros-${ROS_DISTRO}-navigation2 \
      ros-${ROS_DISTRO}-nav2-bringup \
      python3-pytest \
      python3-websockets python3-pil \
      libegl1 libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
