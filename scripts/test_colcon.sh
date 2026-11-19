#!/bin/bash
# Builds vsl_ros2 as a ROS 2 package with colcon in a fresh workspace and checks
# what a robot builder uses: ros2 run, ros2 launch with the shipped config,
# the release tool, colcon test, and the rosdep key for vsl-core.
# Usage, from the repository root:
#   docker run --rm -v "$PWD:/src:ro" ros:jazzy-ros-base bash /src/scripts/test_colcon.sh jazzy
set -e
DISTRO=${1:-jazzy}
source /opt/ros/$DISTRO/setup.bash
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq >/dev/null
apt-get install -y -qq python3-pip python3-pytest \
    ros-$DISTRO-nav2-msgs ros-$DISTRO-diagnostic-msgs ros-$DISTRO-example-interfaces >/dev/null 2>&1
source /opt/ros/$DISTRO/setup.bash

echo "--- rosdep key"
echo "yaml file:///ws/src/vsl_ros2/rosdep/vsl_ros2.yaml" > /etc/ros/rosdep/sources.list.d/50-vsl-ros2.list
mkdir -p /ws/src && cp -r "${SRC:-/src}" /ws/src/vsl_ros2 && rm -rf /ws/src/vsl_ros2/src/*.egg-info
rosdep update -q >/dev/null 2>&1 || true
rosdep resolve python3-super-semantics-vsl-pip
export PIP_BREAK_SYSTEM_PACKAGES=1
rosdep install --from-paths /ws/src --ignore-src -y -q --rosdistro "$DISTRO" 2>&1 | tail -2

echo "--- colcon build"
cd /ws
colcon build --packages-select vsl_ros2 2>&1 | tail -3
source install/setup.bash
ros2 pkg executables vsl_ros2
test -f "$(ros2 pkg prefix vsl_ros2)/share/vsl_ros2/config/nav2_guard.yaml"
test -f "$(ros2 pkg prefix vsl_ros2)/share/vsl_ros2/launch/vsl_guard.launch.py"

echo "--- ros2 launch with the shipped config (paths moved to /tmp)"
sed -e 's#/var/lib/vsl/#/tmp/vsl/#' "$(ros2 pkg prefix vsl_ros2)/share/vsl_ros2/config/nav2_guard.yaml" > /tmp/guard.yaml
timeout 8 ros2 launch vsl_ros2 vsl_guard.launch.py params_file:=/tmp/guard.yaml > /tmp/launch.log 2>&1 || true
grep -q "guarding /navigate_to_pose (nav2_msgs/action/NavigateToPose) as /vsl_guard/navigate_to_pose" /tmp/launch.log \
    || { cat /tmp/launch.log; exit 1; }
grep "guarding" /tmp/launch.log | sed 's/^.*\] //'
test -f /tmp/vsl/guard_ledger.jsonl

echo "--- a bad config fails loudly"
printf 'vsl_guard:\n  ros__parameters:\n    conditions: [x]\n    x:\n      type: magic\n' > /tmp/bad.yaml
if ros2 run vsl_ros2 vsl_guard --ros-args --params-file /tmp/bad.yaml > /tmp/bad.log 2>&1; then exit 1; fi
grep "configuration error" /tmp/bad.log

echo "--- release tool"
ros2 run vsl_ros2 vsl_guard_release --latch-path /tmp/vsl/guard_latch.json --status

echo "--- colcon test"
colcon test --packages-select vsl_ros2 --event-handlers console_direct- > /tmp/colcon_test.log 2>&1 || true
grep -E "passed|failed|error" /tmp/colcon_test.log | tail -3
colcon test-result --test-result-base build/vsl_ros2 || { tail -40 /tmp/colcon_test.log; exit 1; }
colcon test-result --test-result-base build/vsl_ros2 | grep -q "Summary: 0 tests" && { echo "colcon ran no tests"; exit 1; }
echo "colcon OK"
