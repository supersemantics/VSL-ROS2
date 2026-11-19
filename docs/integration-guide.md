# Integrating vsl_guard with your robot

A step-by-step guide for putting `vsl_guard` in front of your robot's Nav2 stack. It assumes you know ROS 2 and Nav2. Plan for half a day in simulation before you touch hardware. [The pilot test plan](pilot-test-plan.md) is what you run afterwards.

**Before you start, the one rule.** `vsl_guard` reads verdicts your stack already produces; it never re-implements them. Integration is mostly finding out what your stack already reports (step 1) and telling the guard where to read it (step 3).

## 0. Prerequisites

- ROS 2 Humble or Jazzy (tested), Ubuntu 22.04 or 24.04.
- Nav2, or any ROS 2 action server you want to guard.
- Python 3.10+ (comes with ROS 2).
- An e-stop state on a `std_msgs/Bool` topic, `true` = engaged. If your e-stop reports differently, see step 1d.
- Recommended first: run the [TurtleBot3 demo](../demo/turtlebot3/README.md) once (`docker build` + `docker run`, about 15 minutes) to see the expected behaviour end to end.

## 1. Inventory your stack

Run these with your full stack up, in simulation or on the robot. Write the answers down; step 3 turns them into the configuration.

### a. Who publishes the command topic your driver listens to?

```bash
ros2 topic info -v /cmd_vel
```

List every node under "Publisher". On a stock Nav2 Jazzy bringup that is `/collision_monitor` **and `/docking_server`**: the docking server publishes there directly. Anything else listed (teleop, a joystick node, a safety controller) is either intended, and goes in `allowed`, or is a bypass you didn't know about.

If your driver listens on a different topic (`/cmd_vel_safe`, `/diff_drive_controller/cmd_vel_unstamped`, ...), use that topic.

### b. Which lifecycle nodes must be active for a goal to make sense?

```bash
ros2 lifecycle nodes
```

At minimum `/bt_navigator`, and `/collision_monitor` if you run it. The guard gates on these being `active`.

### c. Where does the collision monitor publish its state?

```bash
ros2 param get /collision_monitor state_topic
```

On Nav2's default parameters this is `collision_monitor_state`. If it's empty, the collision monitor publishes no state: set one in your Nav2 parameters, or leave the `collision_monitor` condition out.

### d. What reports the e-stop?

The guard needs a `std_msgs/Bool` topic, `true` = engaged, that keeps publishing. Most bases expose this from the motor driver or safety PLC bridge. If yours reports it another way (a status bitfield, a diagnostic, a service), add a ten-line bridge node that republishes it as a Bool, and set `max_age_sec` so a silent bridge counts as engaged.

### e. Which diagnostics matter? (optional)

```bash
ros2 topic echo /diagnostics_agg --once --field status | tr ',' '\n' | grep "name="
```

Pick the groups whose ERROR or STALE should stop new tasks, such as `/Drive` or `/Sensors/Lidar`.

## 2. Install

Build it in your workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/supersemantics/VSL-ROS2.git vsl_ros2
```

vsl-core comes from PyPI through a rosdep key. Register it once per machine:

```bash
echo "yaml file://$HOME/ros2_ws/src/vsl_ros2/rosdep/vsl_ros2.yaml" | sudo tee /etc/ros/rosdep/sources.list.d/50-vsl-ros2.list
```

```bash
rosdep update
```

Install the dependencies. On Jazzy (Ubuntu 24.04), set `PIP_BREAK_SYSTEM_PACKAGES=1` for this step:

```bash
cd ~/ros2_ws && rosdep install --from-paths src --ignore-src -y
```

```bash
colcon build --packages-select vsl_ros2 && source install/setup.bash
```

Check it installed:

```bash
ros2 pkg executables vsl_ros2
```

You should see `vsl_guard`, `vsl_guard_release` and `vsl_guard_report`.

## 3. Write the configuration

Copy the shipped Nav2 configuration and edit it with your inventory:

```bash
cp $(ros2 pkg prefix vsl_ros2)/share/vsl_ros2/config/nav2_guard.yaml ~/robot_config/vsl_guard.yaml
```

| Inventory answer | Configuration |
|---|---|
| 1a: command-topic publishers | `cmd_vel_publishers.topic`, `cmd_vel_publishers.allowed` |
| 1b: lifecycle nodes | one `type: lifecycle` condition per node |
| 1c: collision monitor state topic | `collision_monitor.topic`; keep `gate: false` |
| 1d: e-stop topic | `e_stop.topic`; `max_age_sec` if the source can go silent |
| 1e: diagnostics groups | a `type: diagnostics` condition (`components: [...]`), with `cancel_running: true` if unhealthy mid-task should stop the task |
| where state lives | `ledger_path`, `latch_path`: a persistent directory (see [operations](operations.md)) |
| who releases | `release.authority`, `release.contact` |

Every key is in the [configuration reference](configuration.md). Two rules matter most:

1. **Don't gate on a ROS 2 function that already acts on its own verdict.** The collision monitor stays `gate: false`, which means record only. Gating new goals on it can deadlock the robot after a stop.
2. **Mark only real stop conditions `terminal: true`.** A terminal condition latches the robot until a human releases it. That's right for an e-stop, and wrong for anything that clears on its own.

## 4. Launch it with your stack

On its own:

```bash
ros2 launch vsl_ros2 vsl_guard.launch.py params_file:=$HOME/robot_config/vsl_guard.yaml
```

Or from your bringup launch file:

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

vsl_guard = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare("vsl_ros2"), "launch", "vsl_guard.launch.py"])),
    launch_arguments={"params_file": "/etc/robot/vsl_guard.yaml"}.items(),
)
```

The launch file respawns the guard if it dies. The Terminal State survives that, because the latch is on disk.

Then wait for the guard to report ready:

```bash
ros2 topic echo /diagnostics --field status | grep vsl_guard
```

```
[diagnostic_msgs.msg.DiagnosticStatus(level=b'\x00', name='vsl_guard: /vsl_guard', message='accepting goals for /navigate_to_pose', ...)]
```

`level=b'\x00'` (OK) with `accepting goals for /navigate_to_pose` means it's ready. `b'\x01'` (WARN) and `b'\x02'` (ERROR, latched) come with a message saying why. A WARN message names the conditions that don't hold yet; the [troubleshooting table](operations.md#troubleshooting) explains each one.

## 5. Point your planner at the guard

Your planner sends goals to `served_action` (default `/vsl_guard/navigate_to_pose`) instead of `/navigate_to_pose`. The action type, the goal, feedback and result are unchanged.

```python
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

client = ActionClient(node, NavigateToPose, "/vsl_guard/navigate_to_pose")   # was "/navigate_to_pose"
```

What the planner sees changes in three ways:

| Situation | Before | With the guard |
|---|---|---|
| Robot not fit for a task (e-stop, Nav2 inactive, bypass) | goal accepted | goal **rejected**; the reason is on `/diagnostics` and in the ledger |
| Robot stops being fit mid-goal | goal continues or resumes | goal **aborted** (cancelled in Nav2) |
| Guard restarting | — | action server briefly unavailable; wait for `/diagnostics` OK |

Handle a rejected goal the way you handle any Nav2 rejection: don't retry in a tight loop, and wait for `/diagnostics` OK.

If your planner hard-codes `/navigate_to_pose` (some helper libraries do), change it, or use ROS 2 name remapping for that node. Check that the remap applies to action names on your distro before relying on it.

## 6. Lock the bypass (recommended)

Without access control, nothing in ROS 2 stops a node from calling Nav2 directly or publishing on `/cmd_vel`. The guard detects both:

- a direct goal is cancelled, as `foreign`;
- an extra `cmd_vel` publisher blocks goals.

Detection happens after the fact. To *prevent* the bypass, use SROS2:

1. Create a keystore and one enclave per process: the guard, the planner and Nav2.
2. Give the planner's enclave permission to call `/vsl_guard/navigate_to_pose` only, not `/navigate_to_pose`.
3. Give only `/collision_monitor` (and `/docking_server` if you use docking) permission to publish on `/cmd_vel`.
4. Give the guard's enclave permission to call `/navigate_to_pose` and its `_action/cancel_goal`, and to subscribe to `_action/status`.

> We haven't validated an SROS2 policy for this layout on a reference stack yet. If you do this in your pilot, please send us your policy and what you found.

## 7. Verify

Walk through these in simulation before hardware. Each takes a minute:

1. `/diagnostics` shows the guard OK.
2. A goal through the guard reaches Nav2 and completes.
3. Engage the e-stop mid-goal. The goal is aborted within about 0.3 s, the robot stops, and **releasing the button does not restart it**. New goals are rejected, and `/diagnostics` shows ERROR.
4. Release it with evidence:

   ```bash
   ros2 run vsl_ros2 vsl_guard_release --latch-path <latch_path> \
       --authorised-by "Your Name" --root-cause "verification test" \
       --spec-update "None required." --wait 5
   ```

5. Read the record:

   ```bash
   ros2 run vsl_ros2 vsl_guard_report <ledger_path>
   ```

   `hash chain intact: yes`, `audit passed: yes`, and your test is listed.

[The pilot test plan](pilot-test-plan.md) is the full set, with pass criteria.

## 8. Beyond Nav2's NavigateToPose

- **Another Nav2 action** (`NavigateThroughPoses`, `FollowWaypoints`): set `action_type`, `served_action` and `upstream_action`. Run one guard per action.
- **Another action server** (a manipulation or docking action): the guard works with any action type. Choose conditions from what that stack reports.
- **A rule no ROS 2 function decides** (a task outside the planner's mandate, a goal outside the zone this job allows): build your own guard node with the Python API (`ROS2Adapter`, the conditions, `GatedActionClient`, `GoalSupervisor`, `InterventionWitness`). `src/vsl_ros2/guard_node.py` is the reference composition.
