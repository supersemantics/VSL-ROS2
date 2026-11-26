# Changelog

What changed in each release, and why. Newest first.

## 0.2.0 (unreleased): vsl-core 0.4, no F2 claims

**Before:** vsl-ros2 required vsl-core 0.3. Every condition's Invariant, and `vsl_guard`'s
supervision Invariant, carried `NO_MODEL` (F1 True, F2 `NONE`, level LOW), and `ROS2Adapter` read
a monitor's or rule's source to warn when it claimed `F2Modification.FULL` but appeared to call a
hosted model API. The README had a table of F1/F2 levels.

**After:** vsl-core is `super-semantics-vsl>=0.4,<0.5` (the import check needs 0.4.0). Gates are
built without an assurance basis, and the hosted-API heuristic is gone. `Condition.invariant()`
still takes `assurance_basis` but passes it on only when given, so vsl-core warns. `NO_MODEL`
still imports and is deprecated, removed in 1.0. The README's F1/F2 section is now "What a gate
reports", and the ledger section shows `write_action()` and `enforcement()`. `vsl_guard`'s
behaviour and its record are unchanged.

**Why:** vsl-core 0.4.0 no longer takes a declared F2 level; enforcement is computed, not claimed.

## 0.1.1 (2026-11-20, tag `v0.1.1`): the rest of the pre-release code review

0.1.0 shipped with the review's four pilot-critical fixes. These are the other six: one that could
leave the guard refusing every goal, one about the record's accuracy, and four efficiency and
duplication fixes. Configuration and behaviour are otherwise unchanged.

- **A lost lifecycle reply no longer leaves the guard stuck.** `LifecycleCondition` asked
  `get_state` once. If the reply never came, or came back as an error, the condition stayed
  unknown (not holding) until the node happened to publish a transition. A Nav2 server that was
  already active never does, so the guard rejected every goal until Nav2 restarted. It now asks
  again after `query_timeout_sec` (2 s) or on an error, until the state is known.
- **The record shows the verdicts a goal was decided on.** The guard used to record each
  condition's detail at write time, which could differ from the verdict actually used. A condition
  flipping in between produced "allowed" next to "e-stop engaged". Forwarding now takes one
  snapshot of the verdicts, decides from it and records it. A goal the snapshot refuses is not
  sent, and is recorded as such.
- **Startup reads the ledger once,** not twice. Startup time is part of the unsupervised gap after
  a crash, and the ledger grows for the robot's whole life.
- **Finished goals leave no bookkeeping behind.** The guard kept every goal id it ever forwarded or
  found unsupervised, and scanned them on every status message. Ids are now dropped once the
  upstream server reports the goal finished, or when it rejects it.
- **One cancel-by-id path.** Cancelling an unsupervised goal and cancelling at shutdown were two
  copies of the same code. The shutdown copy didn't check the cancel service was there, so it
  could wait its whole timeout for answers that would never come. It now uses the shared helper,
  waits up to 1 s for the service, and otherwise records "cancel service unavailable; the next
  start cancels it as an orphan". `guard_stopped`'s `cancels_confirmed` now counts cancels the
  server accepted, not just answered.
- **One GetResult request per goal.** Goal tracking and the guard's own wait each asked Nav2 for
  the result. `GatedActionClient.result_future(handle)` now asks once and shares it.

**Tests:** 79 pass on ROS 2 Humble and Jazzy, installed with pip and built with colcon, including
new unit tests for the lifecycle retry and the single result request; the live guard test checks
that no goal ids remain once goals finish. The real-Nav2 TurtleBot3 demo still passes 44/44.

## 0.1.0 (2026-11-19, pushed to `main`; tag `v0.1.0` pending): first release

The first VSL adapter for robots. Its rule: where ROS 2 already decides something, vsl-ros2 reads
that decision as a vsl-core Invariant and never re-implements it. It adds only what ROS 2 lacks.

**Before:** VSL had adapters for agent frameworks (LangGraph, LangChain, the Microsoft Agent
Framework), where the side effect is a tool call. A ROS 2 robot acts differently, and its
ecosystem already enforces much of its own safety: Nav2's collision monitor stops the robot, the
velocity smoother and controllers clamp commands, diagnostics rate components, lifecycle nodes
track their state. A VSL adapter that wrote its own speed and obstacle gates would duplicate those
and eventually disagree with them. What ROS 2 does not have is a check before a task is accepted
or sent, a decision record of its own interventions, Terminal States that need a human to leave,
or a check that nothing bypasses a gated topic.

**After:**

- **Conditions** (`conditions.py`): ROS 2 functions' own verdicts, held from the node's callbacks
  and exposed as vsl-core Invariants with `condition.invariant()`. `CollisionMonitorCondition`
  (Nav2 `CollisionMonitorState`), `DiagnosticsCondition` (`/diagnostics_agg`),
  `LifecycleCondition` (`get_state` + `transition_event`), `LockCondition` (e-stop /
  `twist_mux`-style Bool locks), `SolePublisherCondition` (the ROS graph: bypass detection) and
  `TopicCondition` (any other verdict on a topic). All fail closed: not heard yet, stale, or
  unreadable means not holding. Gates read them and never call into ROS, so they cannot deadlock
  the executor.
- **`InterventionWitness`** (`witness.py`): writes each intervention by those ROS 2 functions to the
  application's `VerbaLedger` (MONITOR + PRE_NODE, then VERIFICATION when it clears), and for
  e-stop-like conditions the TERMINAL entry plus the latch trip. `release()` writes the
  HUMAN_AUTHORISED_TRANSITION before releasing. It writes only when the application constructs it
  with a ledger; compiled gates never write.
- **`GoalSupervisor`** (`supervisor.py`): `GatedActionClient` tracks every accepted goal until it
  finishes, and the supervisor re-checks them on a node timer. It cancels any goal that no longer
  passes: always while the latch is tripped, and when one of the gates the application lists as
  "must keep holding" denies. Without it, a goal still running when an e-stop latched the guard
  would resume as soon as the e-stop button was released. The cancel policy follows the same rule
  as everything else: a collision-monitor stop does not cancel a goal, because the collision monitor
  already stops and resumes the robot. `on_cancel` and `on_cancel_done` let the application record
  the cancel, and a refused cancel is logged as an error.
- **`TerminalLatch`** (`runner.py`): holds every wrapper sharing it in a Terminal State until a
  `HumanAuthorisedTransition`. With `state_path` it survives restarts (atomic, fsynced file;
  unreadable = still tripped).
- **`ROS2Adapter`**: compiles PreNodes and Invariants; passes `run_conformance_suite`; never
  imports rclpy; same heuristic warnings as the other adapters.
- **Wrappers** that run gates immediately before a ROS 2 side effect: `GatedActionClient` and
  `GatedClient` (for C++ servers such as Nav2's, and untrusted callers such as LLM planners),
  `gated_goal_callback`, `gated_service_callback`, `gated_subscription_callback`,
  `GatedPublisher`, and the `create_*` helpers. `GateRunner` runs async gates from rclpy callbacks
  with a 1 s fail-closed timeout. Every block is one `VSLGateBlocked`, and executor-run callbacks
  never raise one.
- **`vsl_guard`** (`guard_node.py`): a ready-made guard node, configured entirely by a ROS 2
  parameters file, that needs no Python. It serves the same action type as the server it
  protects (any action; `config/nav2_guard.yaml` is Nav2's `NavigateToPose`), so a planner calls
  it as it would call Nav2.
  - It accepts a goal only while the configured conditions hold.
  - It forwards the goal under the same goal id, relays feedback, results and the planner's cancels.
  - It cancels running goals upstream when a `terminal` or `cancel_running` condition fails.
  - It records every decision, cancel and intervention so `audit()` passes.
  - It creates its state directory on a fresh robot.
  - A configuration error stops it with a message naming the key.
- **`vsl_guard_release`** (`release_cli.py`): a named human, with evidence, asks the guard to leave
  its Terminal State. The guard applies the request only when the terminal conditions hold again,
  and otherwise rejects it with the reason; on success it records the HUMAN_AUTHORISED_TRANSITION.
  There is no ROS service for this: a release needs write access to the guard's state directory
  on the robot.
- **ROS 2 packaging:** `package.xml` (ament_python), `setup.py` + `setup.cfg` (one metadata source
  for colcon and PyPI; a pyproject `[project]` table can't be read by Humble's system setuptools 59),
  `resource/`, `config/nav2_guard.yaml`, `launch/vsl_guard.launch.py` (respawns; the Terminal
  State survives that), and `rosdep/vsl_ros2.yaml` with the key that installs vsl-core from PyPI.
  `colcon build`, `ros2 run`, `ros2 launch` and `colcon test` all work. The PyPI wheel carries the
  same commands and share files.
- **`demo/turtlebot3/`**: `vsl_guard` in front of the real Nav2 Jazzy stack driving a TurtleBot3 in
  Gazebo Harmonic, headless in one container (`docker build -f demo/turtlebot3/Dockerfile ...`).
  The scenario's checks are all measured from `/odom`, the action results and Nav2's own topics:
  - a box dropped in front of the moving robot makes the real collision monitor STOP it; the guard
    records the stop, leaves the goal alone, and the robot resumes and finishes the goal;
  - an e-stop cancels the running goal in 0.3 s, and the robot does not move after the button is
    released;
  - a release request while the e-stop is engaged is refused, and the named human's release works;
  - a rogue `cmd_vel` publisher cancels the running goal;
  - a goal sent to Nav2 directly is cancelled before the robot moves;
  - `kill -9` on the guard mid-goal: Nav2 kept driving the orphan 0.24–0.30 m, then the respawned
    guard cancelled it and the robot stopped 3–5 s after the kill;
  - a clean SIGINT stop cancels the supervised goal, and the restart records "stopped cleanly";
  - the ledger audits clean.

  44 checks; it passed three runs in a row.
- **The guard's own lifecycle is governed and recorded** (`vsl_guard`). The ledger now records:
  - its start, with the configuration and its hash, and how the previous run ended;
  - a `SPECIFICATION_UPDATE` when the configuration changed since the last run;
  - a crash, as a drift → check → verification chain;
  - a clean stop.

  A new declared Invariant, `upstream_goals_supervised`, is read continuously from the upstream
  action's status topic. It finds goals left running by a run that ended (`orphaned`) and goals sent
  to the server around the guard (`foreign`, a bypass), cancels them (`orphaned_goals: cancel`, or
  `record`), and records each one.

  A clean stop (SIGINT or SIGTERM) cancels the goals the guard supervised before exiting. It handles
  the signals itself, so the ROS context is still alive to send the cancels, and drains pending
  callbacks first so a just-accepted goal isn't missed.

  The guard reports its state on `/diagnostics` (OK, WARN with what doesn't hold, ERROR when
  latched).
- **`vsl_guard_report`** (`report.py`): summarises a guard ledger for pilots and incident review.
  It reports hash-chain integrity and vsl-core `audit()`; the guard's lifecycle; goals forwarded,
  refused (by which condition) and cancelled; unsupervised goals; interventions with durations; and
  Terminal States with who released them. Text, or `--json` to send back; `--since` for one shift.
  Exit code 2 if the chain is broken.
- **Documentation for robotics teams** (`docs/`): architecture, integration guide, configuration
  reference, operations, and a four-phase pilot test plan with pass criteria, a results sheet and
  feedback questions.
- **Fixed, from a code review before the first release:**
  - *A restored latch could become impossible to release.* If the guard restarted latched under a
    configuration with no `terminal` condition (one renamed or no longer terminal), every release
    was refused and every goal rejected. The latch now stays releasable under any configuration.
  - *A goal Nav2 accepted too late kept running.* When the upstream server didn't accept a goal
    within `upstream_accept_timeout_sec`, the planner was told it failed, but the request wasn't
    withdrawn. If the server accepted it later, the robot drove a goal nobody thought was running.
    The guard now cancels such a goal as soon as it's accepted, and records it.
  - *Goals arriving during a clean stop were left running.* While stopping, the guard still
    accepted and forwarded new goals after deciding what to cancel. It now refuses them; the check
    and the goal's registration share the stop's lock, so no goal slips between them.
  - *`vsl_guard_report` undercounted forwarded goals.* A goal later cancelled mid-run was counted
    as neither forwarded nor refused. A refusal without a failing condition (e.g. "the guard is
    stopping") is now listed under its reason.
- **Fixed: the guard records a goal decision before forwarding it.** If the record can't be written,
  the goal isn't sent. Previously the goal was sent first, so a failed write aborted the planner's
  goal while Nav2 was already driving. A failed write while rejecting a goal is now logged instead
  of raising inside the goal callback.
- **Found against real Nav2, and fixed:**
  - The guard's own timers now run on the steady clock. With `use_sim_time`, they ran on sim time
    while verdict expiry used the wall clock, so healthy checks looked stale whenever the sim
    clock lagged.
  - New `gate: false` (record, don't block), and the Nav2 config's collision monitor uses it. The
    collision monitor publishes its state only when its action changes, so after a goal ends in a
    STOP the state stays STOP until Nav2 drives again. Gating new goals on it deadlocked the robot.
    For a recorded-only collision monitor, "no message yet" now counts as clear, so its first STOP
    is recorded.
  - The witness no longer records Nav2 servers "configuring" at boot as interventions; a failing
    first verdict counts only for latching conditions.
  - `config/nav2_guard.yaml` now allows `/docking_server` on `cmd_vel`. Nav2 Jazzy's docking
    server publishes there directly, beside the collision monitor, so the old config refused every
    goal.
- No speed, acceleration, joint-limit or obstacle gates: ROS 2 enforces those. (A Python
  `examples/nav2_guard` built during development is replaced by `vsl_guard`.)

**Tests:** 77 pass on ROS 2 Jazzy (Python 3.12) and Humble (Python 3.10) in the official
`ros:*-ros-base` images. They pass both when installed with pip (`scripts/test_in_ros.sh`) and when
built and run by colcon (`scripts/test_colcon.sh`, which also checks the rosdep key, `ros2 launch`
with the shipped config, refusal of a bad config, and the release tool). They cover the
conformance suite, the runner's fail-closed paths, every condition, the witness, the persistent
latch, goal tracking, the supervisor and the release tool, and real nodes over DDS (topics,
services, actions).

`vsl_guard`, loaded with `config/nav2_guard.yaml`, runs against a simulated Nav2 graph whose goals
run until they arrive or are cancelled:
- an allowed goal arrives, with feedback relayed;
- a collision-monitor stop mid-goal is recorded and leaves the goal running;
- the planner's own cancel is relayed;
- a rogue publisher mid-goal cancels the goal;
- an e-stop mid-goal cancels the goal and latches;
- a release while the e-stop is engaged is rejected;
- a restart stays latched;
- the human release through the tool lets goals through again;
- the ledger audits clean.

A second scenario covers the guard's own lifecycle: a direct-to-Nav2 goal is cancelled; a crash
mid-goal leaves an orphan that the next start cancels; a clean stop cancels the supervised goal; a
configuration change is recorded; the ledger audits clean.

The first scenario passed 10 consecutive runs on each distro (before the lifecycle work). Without
ROS 2, the 65 tests that need no rclpy run. Not yet run on a physical robot.

**Requires:** vsl-core (`super-semantics-vsl`) `>=0.3,<0.4`; a ROS 2 installation (rclpy is not on
PyPI); `nav2_msgs` for `CollisionMonitorCondition` and the Nav2 config.

**Why:** governance belongs where ROS 2 has none: before a task is accepted, in the record, and
in leaving a Terminal State. It does not belong in a second copy of Nav2's safety.
