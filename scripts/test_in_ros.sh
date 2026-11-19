#!/bin/bash
# Runs vsl-ros2's full test suite inside a ROS 2 container, installed the way the
# README tells users to: a venv that sees the ROS 2 Python packages.
# Usage, from the repository root:
#   docker run --rm -v "$PWD:/src:ro" ros:jazzy-ros-base bash /src/scripts/test_in_ros.sh jazzy
#   docker run --rm -v "$PWD:/src:ro" ros:humble-ros-base bash /src/scripts/test_in_ros.sh humble
set -e
DISTRO=${1:-jazzy}
source /opt/ros/$DISTRO/setup.bash
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq >/dev/null
apt-get install -y -qq python3-venv python3-pip python3-pytest \
    ros-$DISTRO-example-interfaces ros-$DISTRO-nav2-msgs ros-$DISTRO-diagnostic-msgs >/dev/null 2>&1
source /opt/ros/$DISTRO/setup.bash  # again: the new packages are now on the path
# Work on a copy, so a read-only mount stays untouched. SRC overrides /src (CI).
rm -rf /work && cp -r "${SRC:-/src}" /work && cd /work
rm -rf src/*.egg-info
python3 -m venv --system-site-packages /venv
source /venv/bin/activate
pip install -q --upgrade pip
# Not ".[dev]": the tests use the distro's python3-pytest, because the ROS pytest
# plugins it loads (launch_testing on Humble) break under a newer pytest.
pip install -q .
python3 -c "import vsl_core, vsl_ros2, rclpy, sys; print('vsl-core', vsl_core.__version__, 'vsl-ros2', vsl_ros2.__version__, 'python', sys.version.split()[0])"
python3 -m pytest -q -p no:cacheprovider -rs
python3 -m pytest -q -p no:cacheprovider tests/test_ros2_live.py tests/test_nav2_guard_live.py -v
