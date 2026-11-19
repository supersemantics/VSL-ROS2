# TurtleBot3 + Nav2 + Gazebo (headless) with vsl_ros2 built by colcon.
#
# From the repository root:
#   docker build -f demo/turtlebot3/Dockerfile -t vsl-ros2-tb3-demo .
#   docker run --rm vsl-ros2-tb3-demo
#
# ROS 2 Jazzy: Nav2's own TurtleBot3 simulation (nav2_minimal_tb3_sim, Gazebo Harmonic).
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-navigation2 \
        ros-jazzy-nav2-bringup \
        ros-jazzy-nav2-minimal-tb3-sim \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

# vsl-core comes from PyPI (the rosdep key in rosdep/vsl_ros2.yaml does the same).
RUN PIP_BREAK_SYSTEM_PACKAGES=1 pip install --no-cache-dir "super-semantics-vsl>=0.3,<0.4"

COPY . /ws/src/vsl_ros2
RUN rm -rf /ws/src/vsl_ros2/src/*.egg-info \
    && . /opt/ros/jazzy/setup.sh \
    && cd /ws && colcon build --packages-select vsl_ros2

ENV TURTLEBOT3_MODEL=waffle
WORKDIR /ws
CMD ["bash", "/ws/src/vsl_ros2/demo/turtlebot3/run_demo.sh"]
