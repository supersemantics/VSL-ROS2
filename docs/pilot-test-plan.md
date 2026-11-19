# vsl_guard pilot test plan

For robotics teams piloting `vsl_guard` on their own stack. The plan has four phases, each with explicit pass criteria, and ends with what to send back. Everything is measured from your robot's odometry, the action results and the guard's ledger, not from what the guard says about itself.

| Phase | Where | Time | Goal |
|---|---|---|---|
| 0 | Our TurtleBot3 demo, in Docker | 15 min | See the expected behaviour; prove your machine can build it |
| 1 | Your stack, in simulation | ½–1 day | Every acceptance test passes against your configuration |
| 2 | Your robot, supervised, low speed | ½ day | The tests that matter on hardware pass with your real e-stop |
| 3 | Your robot, normal operation | 1–2 weeks | Nothing gets in the way; the record is complete |

## Before you start

- [ ] [Integration guide](integration-guide.md) steps 1–4 done: inventory, install, configuration, launch.
- [ ] `/diagnostics` shows the guard OK.
- [ ] You know where `ledger_path` and `latch_path` are, and can run `vsl_guard_report` and `vsl_guard_release`.
- [ ] **For phase 2: a working hardware e-stop, independent of ROS, within reach.** The guard governs software decisions; it is not your safety system.

## Phase 0: the reference demo

Build and run it:

```bash
docker build -f demo/turtlebot3/Dockerfile -t vsl-ros2-tb3-demo .
```

```bash
docker run --rm vsl-ros2-tb3-demo
```

**Pass:** the run ends with `RESULT  44/44 checks passed`. If it doesn't, send us the output: it's our bug, not yours.

## Phase 1: your stack in simulation

Run each test from a clean start: the guard OK on `/diagnostics`, not latched. Record the measurements in the results sheet at the end. The checks assume the shipped condition names; adapt them if yours differ.

| # | Test | Steps | Pass criteria |
|---|---|---|---|
| T1 | Normal goal | Send a goal to `served_action` | Accepted; Nav2 completes it; the report lists it as forwarded |
| T2 | **E-stop mid-goal** | Send a long goal; engage the e-stop when the robot is moving; after the robot stops, release the e-stop button and wait 10 s | Goal ends `ABORTED` within 1 s of the e-stop. **The robot does not move after the button is released.** A new goal is rejected. `/diagnostics` ERROR |
| T3 | Release refused, then granted | While still latched, re-engage the e-stop and request a release; then release the button and request again | First request rejected, naming `e_stop`; second applied; a new goal completes |
| T4 | Collision monitor stop | Only if your collision monitor has a stop zone: put an obstacle in front of the moving robot, then remove it | The robot stops (Nav2's doing). The goal is **not** cancelled by the guard and completes after the obstacle is removed. The report lists a `collision_monitor` intervention |
| T5 | Direct goal to Nav2 (bypass) | Send a goal to `upstream_action` directly, not through the guard | Cancelled before the robot travels more than about 0.2 m; the report lists a `foreign` goal |
| T6 | Extra command publisher (bypass) | Mid-goal, start a node that publishes on the command topic (e.g. `teleop_twist_keyboard` without pressing keys) | The running goal is aborted within 2 s; new goals are rejected until that node stops |
| T7 | Nav2 not ready | `ros2 lifecycle set /bt_navigator deactivate` | Goals rejected; `/diagnostics` WARN naming `bt_navigator`. Reactivate and goals flow |
| T8 | **Guard crash mid-goal** | Mid-goal: `pkill -9 -f lib/vsl_ros2/vsl_guard`. Let `ros2 launch` respawn it | Robot stops within 10 s of the kill. Report: `previous run crashed` and an `orphaned` goal, cancel confirmed. Measure the distance driven in the gap |
| T9 | Clean stop mid-goal | Mid-goal: `pkill -INT -f lib/vsl_ros2/vsl_guard` (as `Ctrl-C`/`ros2 launch` shutdown does) | Robot stops; report: `stopped cleanly`, `goals_cancelled: 1`; the next start reports `previous_run: stopped cleanly` |
| T10 | Configuration change | Change one value (e.g. `release_poll_sec`) and restart | The guard logs `configuration changed since the last run`; the report lists it |
| T11 | The record | After T1–T10: `vsl_guard_report <ledger>` | `hash chain intact: yes`, `audit passed: yes`, and every test above is visible |

**Phase 1 passes** when T1–T3 and T5–T11 pass, and T4 passes if it applies. Any failure is something we want to hear about, with the report and the guard log.

## Phase 2: your robot, supervised

Low speed: cap linear velocity at 0.2–0.3 m/s for this phase, in the velocity smoother's `max_velocity` and your controller's own limit (`vx_max` for MPPI, `max_vel_x` for DWB). Clear area, two people: one operating, one on the hardware e-stop.

| # | Test | Pass criteria |
|---|---|---|
| H1 | T1 on the robot | Goal completes |
| H2 | T2 with **your real e-stop signal** feeding the `lock` condition | Running goal aborted; robot doesn't move after the e-stop is released; released only through `vsl_guard_release` |
| H3 | T5 (direct goal) | Cancelled before the robot travels more than about 0.2 m |
| H4 | T8 (guard crash) | Robot stops; measure the gap distance at your test speed |
| H5 | Reboot while latched: latch the guard (H2), reboot the robot's computer, bring the stack up | Guard starts latched (`/diagnostics` ERROR); goals rejected until released |
| H6 | T11 on the robot's ledger | Intact and audit passed |

H5 checks that `latch_path` is on persistent storage. If the robot comes back unlatched, the latch file was on `tmpfs` or got deleted at boot.

## Phase 3: normal operation

Run the guard in front of your normal workload for one to two weeks, with no special tests. At the end of each day or shift, run:

```bash
ros2 run vsl_ros2 vsl_guard_report /var/lib/vsl/guard_ledger.jsonl --since <shift start> --json > report_<date>.json
```

What we look for in these reports:

- **False refusals:** goals rejected when the robot was fit. Which condition, and why?
- **Interventions** by ROS 2 functions: how often, how long, during which tasks.
- **Unsupervised goals:** anything `foreign` means something in your system talks to Nav2 directly.
- Crashes, restarts and configuration changes.
- Whether the audit stays clean.

## What to send back

- [ ] The results sheet below.
- [ ] `vsl_guard_report --json` for each phase.
- [ ] The guard's configuration YAML. Remove anything confidential, such as site names or contacts.
- [ ] The guard's log for any failed test (`ros2 launch` output, or `~/.ros/log`).
- [ ] Your answers to the questions below.

The ledger files themselves are optional. They contain goal poses and timings from your site; send them only if you're comfortable to.

### Results sheet

| Test | Pass / fail / n.a. | Measurement | Notes |
|---|---|---|---|
| Phase 0 | | checks passed: __/44 | |
| T1 | | | |
| T2 | | goal aborted after __ s; distance after button release: __ m | |
| T3 | | | |
| T4 | | | |
| T5 | | robot travelled __ m before the cancel | |
| T6 | | goal aborted after __ s | |
| T7 | | | |
| T8 | | robot stopped __ s after the kill; distance in the gap __ m | |
| T9 | | | |
| T10 | | | |
| T11 | | entries: __ | |
| H1–H6 | | speed used: __ m/s; H4 gap distance: __ m | |
| Phase 3 | | days: __; goals: __; false refusals: __ | |

### Questions

1. How long did integration take (steps 1–4)? Which step was hardest?
2. What did your inventory find that you didn't expect, such as extra command publishers or direct Nav2 clients?
3. Which conditions did you configure, and which roles (gate, record only, `cancel_running`, `terminal`)?
4. Did the guard ever get in the way of legitimate work? When?
5. Is the ledger useful to you as it is? What would you need from it for incident review, customers or certification?
6. What is your e-stop wiring, and did you need a bridge to a Bool topic?
7. Did you try SROS2 lockdown? What policy did you use, and what happened?
8. What other action servers would you want guarded (manipulation, docking, fleet tasks)?
9. What is missing, wrong or confusing in these documents?
