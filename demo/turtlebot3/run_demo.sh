#!/bin/bash
# Runs the demo headless: Nav2's TurtleBot3 simulation (Gazebo Harmonic), the
# real Nav2 stack, vsl_guard launched with its YAML config, then the scenario.
# Copies the ledger, latch and results to /results when it is mounted.
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
export GZ_IP=127.0.0.1          # Gazebo transport inside a container: loopback discovery
export TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:-waffle}
mkdir -p /tmp/vsl /tmp/logs
rm -f /tmp/vsl/*

echo "=== Nav2's default parameters, plus a collision-monitor stop zone"
python3 /ws/src/vsl_ros2/demo/turtlebot3/nav2_params_with_stop_zone.py /tmp/nav2_params.yaml
echo "=== starting Nav2 + TurtleBot3 simulation (headless)"
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=True use_rviz:=False \
    params_file:=/tmp/nav2_params.yaml > /tmp/logs/nav2_sim.log 2>&1 &
SIM=$!
echo "=== starting vsl_guard"
ros2 launch vsl_ros2 vsl_guard.launch.py params_file:=/ws/src/vsl_ros2/demo/turtlebot3/guard.yaml > /tmp/logs/vsl_guard.log 2>&1 &
GUARD=$!

python3 /ws/src/vsl_ros2/demo/turtlebot3/scenario.py
RC=$?

echo "=== vsl_guard log (warnings and errors)"
grep -E "WARN|ERROR|guarding|released|rejected|cancelling" /tmp/logs/vsl_guard.log | sed 's/^\[vsl_guard-1\] //' | tail -40
if [ -d /results ]; then
    cp /tmp/vsl/* /tmp/logs/* /results/ 2>/dev/null
    echo "=== copied ledger, results and logs to /results"
fi
echo "=== scenario exit code $RC; shutting down"
kill -INT $GUARD $SIM 2>/dev/null
for _ in $(seq 1 15); do kill -0 $SIM 2>/dev/null || break; sleep 1; done
pkill -9 -f "gz sim" 2>/dev/null || true
exit $RC
