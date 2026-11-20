# vsl-ros2

A [ROS 2](https://docs.ros.org/) (`rclpy`) adapter for [vsl-core](https://github.com/supersemantics/VSL-Core).

> **Robotics teams:** start with the [documentation](docs/README.md): [architecture](docs/architecture.md), [integration guide](docs/integration-guide.md), [configuration](docs/configuration.md), [operations](docs/operations.md) and the [pilot test plan](docs/pilot-test-plan.md). To pilot it or ask anything: **hello@supersemantics.org**.

**The rule this adapter follows: where ROS 2 already decides something, vsl-ros2 reads that decision and never re-implements it.** Nav2's collision monitor already decides when the robot must stop. The diagnostic aggregator decides whether a component is healthy, a lifecycle node knows whether it is active, and an e-stop topic says whether motion is locked out. vsl-ros2 turns each of those verdicts into a vsl-core `Invariant` whose rule simply reads it, so the ROS 2 function stays the authority. Then it adds what ROS 2 does not have.

Requires vsl-core 0.3.0 or later and a ROS 2 installation (tested on Humble with Python 3.10 and Jazzy with Python 3.12).

## What ROS 2 already does, and what vsl-ros2 adds

| ROS 2 function | What it already does | How vsl-ros2 uses it | What vsl-ros2 adds |
|---|---|---|---|
| Nav2 `collision_monitor` | Stops or slows the robot on `cmd_vel` | `CollisionMonitorCondition` reads its `CollisionMonitorState` | Every stop and clear is recorded against the goal it interrupted. It is not gated on: the monitor already acts on its own verdict, and its state stays at STOP after a goal ends, so gating would deadlock (found against real Nav2). Tasks are gated on the monitor being *active* |
| Diagnostics (`diagnostic_aggregator`) | Rates components OK / WARN / ERROR / STALE | `DiagnosticsCondition` reads `/diagnostics_agg` | No task starts on an unhealthy robot; a component going quiet counts as unhealthy |
| Lifecycle nodes (Nav2 servers, drivers) | Know whether they are active | `LifecycleCondition` reads `get_state` and `transition_event` | Tasks wait until the servers they need are active |
| E-stop or `twist_mux`-style lock topics | Lock out motion while engaged | `LockCondition` reads the `std_msgs/Bool` topic | A Terminal State: releasing the button is not enough, a human-authorised transition is, and a restart does not clear it |
| ROS graph, SROS2 | Anyone may publish anywhere; SROS2 restricts it by identity | `SolePublisherCondition` reads `get_publishers_info_by_topic` | A runtime check that nothing bypasses the gated topic |
| Velocity limits (`velocity_smoother`, controller and joint limits) | Clamp commands | **Not gated.** ROS 2 does this and vsl-ros2 leaves it there | — |
| Anything else with a verdict on a topic (a safety PLC bridge, a battery manager) | Its own | `TopicCondition(node, type, topic, predicate)` | As above |

What ROS 2 lacks, and vsl-ros2 provides:

1. **Pre-commitment at the task level, and supervision while the task runs.** ROS 2's safety functions act on motion that is already happening. Nothing checks *before a goal is accepted or sent* that the robot is in a state to take on a task. vsl-ros2 puts one gate chain, built from those verdicts, in front of the goal (F1 pre-commitment). `GoalSupervisor` then cancels a running goal when the robot stops being fit for it: always in a Terminal State, so a goal can't resume when an e-stop button is released.
2. **A decision record.** rosbag records messages, not decisions. vsl-ros2's `InterventionWitness` writes each intervention by a ROS 2 function to a tamper-evident `VerbaLedger`: which rule, when, what the robot was held in, and when it cleared. Your gates' own decisions go to the same ledger, in the shape vsl-core's `audit()` checks.
3. **Human-authorised Terminal States.** ROS 2 e-stops reset when the topic says so. A `TerminalLatch` holds until a `HumanAuthorisedTransition` (a named approver, an authority and evidence) releases it, and with `state_path` it survives a restart.
4. **Bypass detection.** A gate only governs what passes through it. `SolePublisherCondition` checks the ROS graph for anything else writing to the gated topic.
5. **Gating untrusted callers.** When the code deciding to act is an LLM planner or another fleet's system, `GatedActionClient` and `GatedClient` stop the request leaving it.

Is not:

- A functional-safety system, or a second collision monitor. A VSL gate runs in your Python process over DDS. It does not replace a hardware e-stop, a safety-rated controller or the certification your robot needs (ISO 10218, ISO 13482, IEC 61508 and similar). It governs what the software decides to do, upstream of those layers.
- A replacement for `vsl-core`, which holds the constructs, the ledger and the governance vocabulary.

## Quick start: `vsl_guard` in front of Nav2, no code

`vsl_guard` is a ready-made node, configured entirely by a ROS 2 parameters file. It serves the same action type as the server it protects, so your planner (an LLM, a fleet manager, a behaviour tree) calls `/vsl_guard/navigate_to_pose` exactly as it would call Nav2's `/navigate_to_pose`:

```
planner ──/vsl_guard/navigate_to_pose──▶ vsl_guard ──/navigate_to_pose──▶ Nav2
```

What the guard does with each goal:

- **Accepts it** only while every configured condition holds. Otherwise the goal is rejected.
- **Forwards it** to Nav2 under the same goal id, relays feedback and the result, and passes the planner's own cancel requests through.
- **Cancels it in Nav2** if a condition marked `terminal` or `cancel_running` fails mid-goal. The planner then sees the goal aborted.
- **Records** every decision, cancel and ROS 2 intervention in its ledger.

### 1. Build it in your workspace

```bash
cd ~/ros2_ws/src
git clone https://github.com/supersemantics/VSL-ROS2.git vsl_ros2
```

vsl-core is on PyPI only, so register its rosdep key once per machine:

```bash
echo "yaml file://$HOME/ros2_ws/src/vsl_ros2/rosdep/vsl_ros2.yaml" | sudo tee /etc/ros/rosdep/sources.list.d/50-vsl-ros2.list
```

```bash
rosdep update
```

On Jazzy (Ubuntu 24.04), set `PIP_BREAK_SYSTEM_PACKAGES=1` for this step:

```bash
cd ~/ros2_ws && rosdep install --from-paths src --ignore-src -y
```

```bash
colcon build --packages-select vsl_ros2 && source install/setup.bash
```

### 2. Configure it

`config/nav2_guard.yaml` (installed to `share/vsl_ros2/config/`) is a working configuration for Nav2. Copy it and edit it:

```yaml
vsl_guard:
  ros__parameters:
    action_type: nav2_msgs/action/NavigateToPose
    served_action: /vsl_guard/navigate_to_pose
    upstream_action: /navigate_to_pose
    ledger_path: /var/lib/vsl/guard_ledger.jsonl
    latch_path: /var/lib/vsl/guard_latch.json
    conditions: [e_stop, cmd_vel_publishers, bt_navigator, collision_monitor_active, collision_monitor]
    e_stop:             { type: lock, topic: /e_stop, terminal: true }
    cmd_vel_publishers: { type: sole_publisher, topic: /cmd_vel, allowed: [/collision_monitor, /docking_server], cancel_running: true }
    bt_navigator:       { type: lifecycle, node: /bt_navigator }
    collision_monitor_active: { type: lifecycle, node: /collision_monitor }
    collision_monitor:  { type: collision_monitor, topic: /collision_monitor_state, gate: false }
```

| `type` | Reads (a ROS 2 function's own verdict) | Keys |
|---|---|---|
| `lock` | `std_msgs/Bool`, true = engaged (e-stop, `twist_mux`-style lock) | `topic`, `max_age_sec` |
| `sole_publisher` | the ROS graph | `topic`, `allowed`, `period_sec` |
| `lifecycle` | `<node>/get_state`, `<node>/transition_event` | `node`, `required_state` (`active`) |
| `collision_monitor` | `nav2_msgs/CollisionMonitorState` | `topic`, `blocking_actions` (`[STOP]`), `assume_clear_until_first` (default: true when `gate: false`) |
| `diagnostics` | `diagnostic_msgs/DiagnosticArray` | `components`, `topic` (`/diagnostics_agg`), `worst_allowed` (`WARN`), `max_age_sec` |

Three keys on any condition set what the guard does with its verdict:

- **`gate: false`** — record it, never block on it. The witness still writes every intervention and clear to the ledger. Use it where the ROS 2 function already acts on its own verdict. In the Nav2 config this is the collision monitor: it stops and resumes the robot itself, and because it publishes its state only when its action *changes*, a goal that ends during a STOP leaves the last state at STOP until Nav2 commands the robot again. Gating new goals on it would deadlock the robot; the TurtleBot3 demo found this against real Nav2. Gate on the monitor being *active* instead (`collision_monitor_active`).
- **`terminal: true`** — the condition failing enters the Terminal State. The guard latches (and stays latched across restarts), cancels running goals, and refuses goals until a human releases it. Use it for e-stops.
- **`cancel_running: true`** — the condition failing mid-goal cancels the goal in the upstream server. Use it for a bypass on `cmd_vel` or unhealthy diagnostics. It needs `gate: true`.

Leave `terminal` and `cancel_running` off where the ROS 2 function already handles it mid-goal: Nav2 aborts its own goals when a server deactivates.

Other keys:
- `orphaned_goals` (`cancel`, or `record` to log without cancelling)
- `startup_check_sec` (3)
- `upstream_accept_timeout_sec` (10)
- `identity_key` (default: the node's name)
- `terminal_state` (`robot-held`)
- `release.authority`, `release.contact`
- `release_poll_sec` (1)
- `diagnostics_period_sec` (1)

`action_type` can be any action, not just Nav2's. A configuration error stops the node with a message naming the key.

### 3. Launch it with your stack

```bash
ros2 launch vsl_ros2 vsl_guard.launch.py params_file:=/etc/robot/vsl_guard.yaml
```

Or include `vsl_guard.launch.py` from your Nav2 bringup. The launch file respawns the guard if it dies, and the Terminal State survives that, because the latch is on disk.

### The guard's own lifecycle is governed and recorded too

A guard that restarts, crashes or is bypassed is exactly where a record usually has a hole. `vsl_guard` puts its own lifecycle in the ledger as VSL constructs, so `audit()` checks them like everything else:

| Event | In the ledger |
|---|---|
| Start | `MONITOR` `guard_started`: the configuration and its hash, the gating and recorded-only conditions, whether it starts latched, and how the previous run ended |
| Configuration changed since the last run | `SPECIFICATION_UPDATE`: old and new hash, and which keys changed. The YAML is the guard's policy-as-code |
| The previous run ended without a stop record (crash, `kill -9`, power loss) | drift `MONITOR` → `PRE_NODE` (check the upstream server for goals left unsupervised) → `VERIFICATION` (what the check found) |
| A goal running upstream that this guard isn't supervising | a denial of the Invariant **`upstream_goals_supervised`** → `PRE_NODE` (cancel it) → `VERIFICATION` (the server's answer). Recorded as `orphaned` (forwarded by an earlier run of this guard) or `foreign` (sent to the server directly, a bypass) |
| Clean stop (SIGINT/SIGTERM, as `ros2 launch` sends) | each goal it supervised is cancelled and recorded, then `MONITOR` `guard_stopped` |

`upstream_goals_supervised` is checked continuously, from the upstream action's own status topic, so it covers two failures:

- **A crash mid-goal.** Nav2 keeps driving the goal while no guard is running; in the demo it drove 0.24–0.30 m. The respawned guard finds the goal and cancels it: the robot stopped within 3–5 s of the `kill -9`. Orphaned goals are cancelled, not re-adopted, because the planner's goal died with the old process and nobody is waiting for that result.
- **A direct-to-Nav2 bypass.** A goal sent to Nav2 around the guard is cancelled within the same status update. SROS2 should stop the bypass; this catches it if SROS2 isn't there.

The guard also reports its state on `/diagnostics`, ROS 2's own channel for that: OK when it accepts goals, WARN with the conditions that don't hold, ERROR when latched. A planner, `rqt_robot_monitor` or your diagnostic aggregator can wait for OK after a restart instead of guessing.

### 4. Releasing a Terminal State

A latched guard is released only by a named human with evidence, using the release tool on the robot:

```bash
ros2 run vsl_ros2 vsl_guard_release --latch-path /var/lib/vsl/guard_latch.json --status
```

```bash
ros2 run vsl_ros2 vsl_guard_release --latch-path /var/lib/vsl/guard_latch.json \
    --authorised-by "J. Smith (site safety)" \
    --root-cause "E-stop pressed by the operator; bay cleared and inspected." \
    --spec-update "None required." --wait 5
```

The guard applies the request only once every terminal condition holds again: an e-stop still engaged means the request is rejected, with the reason, in `<latch>.release.rejected.json`. On success the guard records the `HUMAN_AUTHORISED_TRANSITION` in its ledger and releases the latch. There is deliberately no ROS service for this. A release needs write access to the guard's state directory on the robot, so the access control is the robot's own, not "anyone on the DDS domain". Make `/var/lib/vsl` writable only by the guard's user and your operators.

### Install with pip instead

For use from your own Python nodes, or without a workspace. rclpy is not on PyPI; it comes with ROS 2. Install into a virtual environment that can see the ROS 2 Python packages:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages ~/vsl-venv
source ~/vsl-venv/bin/activate
pip install --upgrade pip
pip install vsl-ros2
```

This installs vsl-core (`super-semantics-vsl`, imported as `vsl_core`) and the `vsl_guard` and `vsl_guard_release` commands. On Humble, upgrade pip inside the venv first: the system pip 22.0 builds this package as `UNKNOWN`.

## The Python API: your own guard

Use the API when your guard needs a rule that no ROS 2 function decides (whether this task is within the planner's mandate, whether a goal lies in the zone its job allows) or a side effect other than an action goal. `vsl_guard` is built from exactly these pieces:

```python
from vsl_ros2 import (
    CollisionMonitorCondition, GatedActionClient, GoalSupervisor, InterventionWitness,
    LifecycleCondition, LockCondition, ROS2Adapter, SolePublisherCondition, TerminalLatch,
)

# Inside your node's __init__ (self is an rclpy Node):
e_stop = LockCondition(self, "/e_stop", name="ros2.e_stop")
publishers = SolePublisherCondition(self, "/cmd_vel", ["/collision_monitor"])
conditions = [
    e_stop,
    publishers,
    LifecycleCondition(self, "/bt_navigator"),
    LifecycleCondition(self, "/collision_monitor"),
    CollisionMonitorCondition(self, assume_clear_until_first=True),
]

adapter = ROS2Adapter(identity_key=self.get_fully_qualified_name())
gates = [adapter.compile_invariant(c.invariant()) for c in conditions]
publishers_gate = gates[1]

latch = TerminalLatch(state_path="/var/lib/robot/latch.json")      # survives a restart
witness = InterventionWitness(self, ledger, conditions,             # records ROS 2's own interventions
                              latch=latch, latch_on=[e_stop], terminal_state=E_STOPPED)

nav = GatedActionClient(ActionClient(self, NavigateToPose, "/navigate_to_pose"), gates, latch=latch)
nav.send_goal_async(goal)   # raises VSLGateBlocked instead of sending, if any verdict says no

# Cancels running goals: always while latched, and when these gates deny mid-goal.
# Not the collision monitor: it stops and resumes the robot itself.
supervisor = GoalSupervisor(self, nav, gates=[publishers_gate])
```

`src/vsl_ros2/guard_node.py` is the complete composition, including how it records each goal decision and cancel so the ledger passes `VerbaLedger.audit()`.

## Conditions

A `Condition` holds one ROS 2 function's latest verdict, fed by the node's own callbacks. `condition.invariant()` returns a vsl-core `Invariant` (F1 True, F2 `NONE`: no model at the guarded call) whose rule reads that verdict. Compile it with `ROS2Adapter.compile_invariant` like any other Invariant.

| Condition | Reads | Holds while |
|---|---|---|
| `CollisionMonitorCondition(node, topic="/collision_monitor_state", blocking_actions=("STOP",))` | `nav2_msgs/CollisionMonitorState` | the monitor's action is not blocking. Slowing down is the monitor's job, so SLOWDOWN doesn't block unless you add it |
| `DiagnosticsCondition(node, components, topic="/diagnostics_agg", worst_allowed=DIAG_WARN, max_age_sec=5)` | `diagnostic_msgs/DiagnosticArray` | every named component reports at or below `worst_allowed`, recently |
| `LifecycleCondition(node, target_node, required_state="active")` | `<node>/get_state`, `<node>/transition_event` | the managed node is in `required_state` |
| `LockCondition(node, topic, max_age_sec=None)` | `std_msgs/Bool` | the lock is False (released) |
| `SolePublisherCondition(node, topic, allowed_publishers, period_sec=1)` | the ROS graph | only the allowed nodes publish on `topic` |
| `TopicCondition(node, msg_type, topic, predicate, name=..., description=...)` | any topic | `predicate(latest message)` is true |

Every condition fails closed. A condition that hasn't heard from its ROS 2 function yet does not hold. Neither does one whose verdict is older than its `max_age_sec`, or whose predicate raises. The collision monitor publishes only when its action changes, so it has no expiry. Until its first message it doesn't hold, unless you pass `assume_clear_until_first=True`. That is the right setting when you only *record* it, as `vsl_guard`'s Nav2 config does (`gate: false`): then its first STOP is an intervention, and nothing is let through on the assumption. Don't compile it into a gate in front of new goals. Its state stays at STOP after a goal ends during a stop, so the gate would refuse the very goal that would let it clear. Gate on a `LifecycleCondition` for the collision monitor instead.

Gates read conditions; they never call into ROS. A gate therefore can't deadlock the executor, and checking a whole chain costs a few microseconds plus the runner's thread hand-off.

**When to write your own PreNode or Invariant.** Only for policy no ROS 2 function decides: whether this task is permitted for this robot today, whether a goal lies in the zone its job allows, whether a planner's request is within its mandate. Don't write gates for speed, acceleration, joint limits or obstacle distance. ROS 2 already enforces those, and a second copy would disagree with the first sooner or later.

## Recording ROS 2's interventions: `InterventionWitness`

```python
witness = InterventionWitness(node, ledger, conditions,
                              latch=latch, latch_on=[e_stop], terminal_state=E_STOPPED)
```

The witness listens to the conditions and polls them on a node timer, so expiry is noticed. Each time a condition stops holding it writes:

- **intervention:** a `MONITOR` gate decision (`allowed=False`, `primitive_name` = the condition's name, the ROS 2 function's detail in the payload), then a `PRE_NODE` recording that the ROS 2 function is holding the robot;
- **clear:** a `VERIFICATION` (`SUFFICIENT`, the hold was released).

For a condition in `latch_on` it writes `VERIFICATION` and `TERMINAL` and trips the latch. `witness.release(transition)` later writes the `HUMAN_AUTHORISED_TRANSITION` (`caused_by` the `TERMINAL` entry) and then releases the latch. If that write fails, the latch stays tripped. A condition's first verdict counts as an intervention only for a `latch_on` condition (an e-stop engaged at boot trips the latch). For the rest, a failing first verdict is the stack starting up (Nav2 servers report "configuring"), not an event.

Nothing is recorded unless you construct a witness with your own ledger. vsl-core requires gate compilation never to write a ledger, and the gates compiled from conditions don't.

## Running goals: `GoalSupervisor`

A goal is checked once, when it is sent, and a navigation goal can run for minutes. Suppose an e-stop latches the guard mid-goal. ROS 2 stops the motion, but the goal is still active in Nav2, so when someone releases the e-stop button Nav2 carries on driving to it, even though the guard is still waiting for a human. The latch has to cancel running goals, not only refuse new ones.

`GatedActionClient` tracks every goal sent with `send_goal_async()` that the server accepted, until it finishes (`active_goals`). `GoalSupervisor(node, client, gates=[...])` re-checks them on a node timer (0.1 s by default) and asks the server to cancel any that no longer pass:

- **always** while the client's `TerminalLatch` is tripped;
- **when any of `gates` denies**: the gates that must keep holding *while* a goal runs.

Choose those gates per ROS 2 function, the same rule as everywhere else: don't take over what ROS 2 already does.

| Mid-goal, this happens | Cancel the goal? | Why |
|---|---|---|
| E-stop (the latch trips) | Yes, always | Otherwise the goal resumes when the button is released |
| A foreign publisher appears on the driver's `cmd_vel` | Yes | The command path is no longer the one that was checked |
| Diagnostics for a component go ERROR / STALE | Yes, if you list it | The robot is no longer fit for the task |
| The collision monitor stops the robot | **No** | The collision monitor already stops the robot and resumes when the obstacle clears; that's its job |
| A Nav2 server deactivates | **No** | Nav2 aborts its own goals |

Cancelling is a request, and the server decides. `on_cancel(active, blocked)` runs when the request is sent and `on_cancel_done(active, blocked, accepted)` when the server answers, so the application can record both. A refused cancel is logged as an error, because the goal is still running. The example records it as an `INSUFFICIENT` verification, which `audit()` then flags until someone writes a specification update. To join a cancel to the goal's original decision in the ledger, send the goal with your own `goal_uuid` and use it as the `decision_id`, as the example does.

## Terminal States: `TerminalLatch`

vsl-core's `TerminalState` is a state "from which no further automated transitions are permitted", left only through a Human-Authorised Transition. Every wrapper sharing a latch blocks every action, without running its gates, once the latch trips. It trips when the witness sees a `latch_on` condition fail, or when a gate's Invariant names an `on_violation` TerminalState.

- `TerminalLatch(state_path=...)` writes the trip to that file (atomically, fsynced) and reads it back on start, so relaunching the node or rebooting the robot does not release it. A file that exists but can't be read keeps the latch tripped. Without `state_path` the latch is held in memory: use that only for tests and simulation.
- `release()` requires a `vsl_core.governance.HumanAuthorisedTransition`. There is no timeout, default or automatic release.

## Bypass: gate the only path

The guard only governs goals sent through it, and a gated topic is only gated if nothing else writes to it.

- **Prevent** the bypass with SROS2 access control: the planner gets no permission on Nav2's actions or the driver's `cmd_vel`, and only the intended nodes may publish there.
- **Detect** it with `SolePublisherCondition` on the topic the driver actually listens to (for Nav2, the collision monitor's output `cmd_vel`). A foreign publisher fails the condition, which blocks new goals and, through the witness, is recorded.

## Gating actions directly: the wrappers

The conditions plug into these wrappers, which also take any compiled PreNode or Invariant. Each calls the gates immediately before its ROS 2 side effect:

| ROS 2 side effect | Wrapper | On a block |
|---|---|---|
| Send an action goal | `GatedActionClient(action_client, gates)` | `send_goal()` / `send_goal_async()` raise `VSLGateBlocked`; nothing is sent. Accepted goals are tracked for `GoalSupervisor` |
| Call a service | `GatedClient(client, gates)` | `call()` / `call_async()` raise `VSLGateBlocked`; nothing is sent |
| Accept an action goal (a Python action server) | `ActionServer(..., goal_callback=gated_goal_callback(gates))` | `GoalResponse.REJECT`; the execute callback never runs |
| Handle a service request | `gated_service_callback(callback, gates)`, `create_gated_service(node, ...)` | The callback is not called; the response gets `success=False` and the reason |
| Act on a received message | `gated_subscription_callback(callback, gates)`, `create_gated_subscription(node, ...)` | The callback is not called |
| Publish on a topic | `GatedPublisher(publisher, gates)`, `create_gated_publisher(node, ...)` | Not published; `safe_message` published instead if given |

Nav2's servers are C++, so gate goals for them on the client side (`GatedActionClient`, as `vsl_guard` does). Use `gated_goal_callback` for your own Python action servers. Prefer gating tasks and requests over gating `cmd_vel`: on a Nav2 robot the velocity path already belongs to the controller, the velocity smoother and the collision monitor.

Every wrapper takes one gate or a list. A list runs in order and the first denial stops it. Common options: `candidate_input_fn` (what the gates see), `on_blocked(blocked, raw)` (where the application records the block), `logger`, `latch`, `timeout_sec`, `runner`.

## What counts as a block, and how it reaches you

Every way a gate can fail to say "allowed" blocks the action, and all of them surface as a `VSLGateBlocked`:

- a gate denies (`blocked.denial` is the `InvariantViolation` or `AutomationDeniedException`; for a condition, `blocked.denial.invariant_name` is its name);
- a gate raises, or `candidate_input_fn` does (`blocked.error`);
- a gate doesn't decide within `timeout_sec` (default 1 s);
- a shared `TerminalLatch` is tripped (`blocked.latched`).

Executor-run callbacks (subscriptions, services, goal callbacks, `GatedPublisher.publish`) never raise on a block, because an exception escaping an rclpy callback stops the executor. They log and call `on_blocked`. The client wrappers, which your own code calls, raise.

vsl-core gates are `async` and rclpy executors aren't asyncio loops, so the wrappers run gates on a `GateRunner`: one private asyncio loop on a daemon thread (`default_runner()`), with a timeout. Call `default_runner().check(gates, candidate)` yourself wherever a node is about to commit, for example before each step of a long-running action.

## Recording decisions: the ledger doesn't write itself

Nothing in the adapter or its gates writes to a `VerbaLedger`. The application records its own gate decisions with `write_gate_decision()`, and closes a block with a `PRE_NODE` (the fallback: the goal not sent) and a `VERIFICATION`, as `vsl_guard`'s `_record_decision` does. The `InterventionWitness`, which the application constructs explicitly with its ledger, records ROS 2's interventions. Record decisions that change something (goals, service calls, interventions), not every 50 Hz message.

For a PreNode of your own, vsl-core never measures Gamma: your monitor computes `gamma_hat` as a ratio where 1.0 is break-even, returns `0.0` when its input is missing or stale, and keeps thresholds above 1.0 (`ROBUST_GAMMA_HIGH_STAKES_THRESHOLD`, 1.2, near people).

## Multi-robot systems: handovers and Cluster Gamma

When one node or robot hands part of a task to another (a fleet manager assigning a pick), record the hand-off before the receiver acts with `ledger.write_handover(...)` (vsl-core 0.3.0), and pass its `entry_id` as `caused_by` on the receiver's first gate decision. Robots sharing an aisle, a lift or a workcell form a `vsl_core.cluster.Cluster`: every robot's gates can pass while the shared resource is still unsafe, so put the Cluster's Invariants on whichever node arbitrates that resource.

## The F1/F2 assurance distinction

| Gate | F1 | F2 | Level |
|---|---|---|---|
| A ROS 2 verdict (collision monitor, diagnostics, lifecycle, e-stop) before a goal | True | `NONE` | LOW |
| Gate before a hosted model (an LLM task planner, a hosted VLM) whose prompt the application writes | True | `PARTIAL` | MEDIUM |
| Gate before a self-hosted policy where the application steers logits or activations | True | `FULL` | HIGH |

The level measures influence on a model, not enforcement: a LOW gate still blocks every goal it denies. `ROS2Adapter` warns when a monitor or rule claims `FULL` but appears to call a hosted model API, and when a PreNode sets a `Fallback` the wrappers don't consume.

## Layout

```
package.xml, setup.py, setup.cfg, resource/   the ROS 2 package (ament_python) and the PyPI package, one source
config/nav2_guard.yaml        vsl_guard's configuration for Nav2
launch/vsl_guard.launch.py    ros2 launch vsl_ros2 vsl_guard.launch.py params_file:=...
rosdep/vsl_ros2.yaml          the rosdep key for vsl-core (PyPI)
demo/turtlebot3/              vsl_guard + real Nav2 + TurtleBot3 in Gazebo, headless (docker)
src/vsl_ros2/
├── guard_node.py   vsl_guard: the configurable guard node (an action proxy)
├── release_cli.py  vsl_guard_release: a human leaves the Terminal State
├── conditions.py   ROS 2 functions' verdicts as Invariants
├── witness.py      InterventionWitness: ROS 2's interventions into the ledger
├── supervisor.py   GoalSupervisor: cancel running goals the robot is no longer fit for
├── runner.py       GateRunner, TerminalLatch (persistent), VSLGateBlocked
├── integration.py  the gated wrappers and create_* helpers
└── adapter.py      ROS2Adapter (VSLAdapter conformance contract); never imports rclpy
tests/
├── test_conformance.py, test_runner.py, test_integration.py,
│   test_conditions.py, test_supervisor.py, test_release_cli.py   no ROS 2 needed
├── test_ros2_live.py        real rclpy nodes over DDS: topics, services, actions
└── test_nav2_guard_live.py  vsl_guard with config/nav2_guard.yaml against a simulated Nav2 graph
```

Without ROS 2: `pip install -e ".[dev]" && pytest` runs the first group and skips the live tests. With Docker, from the repository root, either install route:

```bash
docker run --rm -v "$PWD:/src:ro" ros:jazzy-ros-base bash /src/scripts/test_in_ros.sh jazzy
```

```bash
docker run --rm -v "$PWD:/src:ro" ros:jazzy-ros-base bash /src/scripts/test_colcon.sh jazzy
```

`test_colcon.sh` builds the package in a fresh workspace and checks it end to end:
1. resolves vsl-core through the rosdep key;
2. `colcon build`;
3. `ros2 launch` with the shipped config;
4. a bad config is refused;
5. the release tool runs;
6. `colcon test`.

## Status

Alpha. Conformance-suite green. 79 tests pass on ROS 2 Jazzy (Python 3.12) and Humble (Python 3.10) in the official `ros:*-ros-base` images, both when installed with pip and when built and tested with colcon. In those tests Nav2 is simulated at the interfaces the guard reads.

**Against real Nav2:** [`demo/turtlebot3/`](demo/turtlebot3/README.md) runs `vsl_guard` in front of the full Nav2 Jazzy stack driving a TurtleBot3 in Gazebo, headless in one container. It passes 44 checks, measured from odometry and Nav2's own topics:

- a collision-monitor stop that the guard records but leaves to Nav2;
- an e-stop that cancels the running goal, after which the robot doesn't move when the button is released;
- the human release;
- a `cmd_vel` bypass, and a goal sent to Nav2 around the guard;
- `kill -9` on the guard mid-goal, after which the respawned guard cancels the orphan;
- a clean stop that cancels the supervised goal;
- a clean ledger audit.

It passed three runs in a row. Running against real Nav2 and real process signals found seven things the simulated graph didn't, now fixed; see the demo README.

Not yet run on a physical robot, or with MoveIt or ros2_control.

See [CHANGELOG.md](https://github.com/supersemantics/VSL-ROS2/blob/HEAD/CHANGELOG.md).

## Contact

hello@supersemantics.org

## License

MIT -- see LICENSE. Copyright Super Semantics.
