# TurtleBot3 + real Nav2 + Gazebo: `vsl_guard` end to end

A simulated TurtleBot3 (waffle) in Gazebo Harmonic, driven by the real Nav2 stack (ROS 2 Jazzy, Nav2's own `nav2_minimal_tb3_sim`). `vsl_guard` is built with colcon and launched with its YAML config. Everything runs headless in one container:

```
operator / planner ──/vsl_guard/navigate_to_pose──▶ vsl_guard ──/navigate_to_pose──▶ Nav2 ──▶ Gazebo TurtleBot3
     e-stop panel ──/e_stop──────────────────────────▲
```

Build the image (about 5 GB):

```bash
docker build -f demo/turtlebot3/Dockerfile -t vsl-ros2-tb3-demo .
```

Run the demo (about 2 minutes):

```bash
docker run --rm vsl-ros2-tb3-demo
```

To keep the ledger, results and logs, mount a folder at `/results`:

```bash
docker run --rm -v "$PWD/results:/results" vsl-ros2-tb3-demo
```

The scenario (`scenario.py`) measures everything it claims:
- robot motion from `/odom`;
- goal outcomes from the action results;
- the collision monitor's verdicts from its own `/collision_monitor_state`;
- the guard's decisions from its ledger and latch files.

It exits non-zero if any check fails.

## What it shows (44 checks)

| Step | What happens | Who acts | Checked |
|---|---|---|---|
| 1 | A goal through the guard | Nav2 drives the robot 1.2 m | accepted, `SUCCEEDED` |
| 2 | A box is dropped into Gazebo 0.55 m ahead of the moving robot | **Nav2's collision monitor** stops the robot | real `STOP` reported; robot stops; the guard **records** the stop and does **not** cancel the goal; box removed → the robot resumes and completes the same goal; the clear is recorded |
| 3 | E-stop pressed 0.26 m into a goal | **vsl_guard** latches and cancels the goal in Nav2 | goal cancelled within 0.3 s; robot stops; latch on disk; **button released → robot does not move (0.000 m in 8 s)**; new goals refused |
| 4 | A release request while the e-stop is still engaged | vsl_guard | refused, with the reason |
| 5 | `vsl_guard_release` by a named human, with evidence | vsl_guard | released and recorded; the next goal completes |
| 6 | A rogue node starts publishing on `/cmd_vel` mid-goal | vsl_guard (ROS graph check) | goal cancelled; new goals refused; goals flow again once it's gone |
| 7 | A goal sent to Nav2 directly, around the guard | vsl_guard (`upstream_goals_supervised`) | Nav2 accepts it; the guard cancels it before the robot moves (0.00 m); recorded as `foreign` |
| 8 | The guard is killed (`kill -9`) mid-goal; `ros2 launch` respawns it | vsl_guard, next run | Nav2 kept driving the orphaned goal 0.24–0.30 m with no guard running. The respawned guard cancelled it and the robot stopped 3–5 s after the kill and stayed stopped. The crash (`previous_run_ended_unexpectedly`) and the orphan are recorded |
| 9 | The guard is stopped cleanly (SIGINT, as `ros2 launch` sends) mid-goal | vsl_guard | it cancels the goal it supervised; the robot stops; `guard_stopped` recorded with the cancelled goal; the respawned guard records `previous_run: stopped cleanly` and reports OK on `/diagnostics` before goals flow again |
| — | The record | — | ledger chain intact; `VerbaLedger.audit()` passes |

It ran three times in a row with 44/44 checks passing, on a 20-CPU Docker host with Gazebo at real-time factor 1.0.

Step 3 is the gap vsl_guard exists for. Without it, Nav2's goal stays active through an e-stop and the robot drives off again the moment the button is released. Step 2 is the rule it follows: the collision monitor already handles obstacles, so the guard records what it does and stays out of its way.

## What real Nav2 taught the design

Each of these failed against the real stack or real process signals, although the simulated graph in `tests/` had passed. Each is now fixed in the package:

1. **Nav2 Jazzy has two publishers on `cmd_vel`.** One is `collision_monitor`; the other is `docking_server`, which publishes there directly, not through the collision monitor. The shipped `config/nav2_guard.yaml` allowed only the first, so the guard (correctly) refused every goal. Its `allowed` list now names both, with a note to list exactly what your bringup runs.
2. **The guard's timers ran on sim time.** With `use_sim_time`, the graph check ran on the simulation clock while verdict expiry used the wall clock, so a healthy check looked stale whenever the sim clock lagged. The guard's own timers (graph polling, expiry, goal supervision, release requests) now run on the steady clock.
3. **Gating new goals on the collision monitor deadlocks the robot.** The collision monitor publishes its state only when its action *changes*. After a goal ends during a STOP, the last state stays STOP until Nav2 commands the robot again, and that needs a goal the guard was refusing. The collision monitor condition is now `gate: false` in the Nav2 config: recorded, never blocking. The guard gates on the monitor being *active* instead.
4. **Startup is not an intervention.** At boot, Nav2 servers report "configuring". The witness now treats a failing first verdict as an intervention only for latching conditions (an e-stop engaged at boot), and for the collision monitor, whose first message is always a change.
5. **A restart lost track of running goals.** The latch survived a restart; the list of goals the guard had forwarded didn't. A crash mid-goal left Nav2 driving with nothing supervising it. Now the Invariant `upstream_goals_supervised`, read from the action's status topic, finds and cancels such goals, and the guard's start, stop and crashes are recorded.
6. **A clean stop could miss a just-accepted goal, then hang.** At a stop the executor is no longer spinning, so the guard decided what to cancel from stale state, and its forwarding thread waited forever for a result nothing would deliver. It now drains pending callbacks, cancels by goal id from the server's own status, and ends the planner's goal itself.
7. **A planner can't tell when a restarted guard is ready.** A respawned guard refuses goals until it has heard every verdict, which is correct but invisible. It now reports its state on `/diagnostics`, and the scenario waits for OK.

## Notes

- **The stop zone is the demo's.** Nav2 Jazzy's default collision monitor only has a slow-down polygon (`FootprintApproach`), which never reports STOP. `nav2_params_with_stop_zone.py` adds a `PolygonStop` strip 0.25–0.40 m ahead of the base, as a robot builder would.
- **The e-stop is a topic here.** On a real robot a hardware e-stop cuts the motors. In this demo the guard's cancel is what stops Nav2's goal, and the point being shown is that the goal can't come back when the e-stop is released.
- **Gazebo transport inside a container** needs `GZ_IP=127.0.0.1`; `run_demo.sh` sets it.
- **Jazzy only.** Humble's TurtleBot3 simulation uses Gazebo Classic and a different bringup.
