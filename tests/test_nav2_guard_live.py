"""The vsl_guard node, configured by config/nav2_guard.yaml, against a simulated
Nav2 graph over real DDS.

The simulation stands in for Nav2 only as far as the guard can see it:
bt_navigator and collision_monitor lifecycle services and transition events,
the collision monitor's state topic and cmd_vel output, an e-stop panel, and a
NavigateToPose server whose goals run until they arrive or are cancelled. The
planner calls the guard's action exactly as it would call Nav2's.

Skipped without ROS 2 and nav2_msgs.
"""

from __future__ import annotations

import json
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")
NavigateToPose = pytest.importorskip("nav2_msgs.action").NavigateToPose
CollisionMonitorState = pytest.importorskip("nav2_msgs.msg").CollisionMonitorState
yaml = pytest.importorskip("yaml")

from geometry_msgs.msg import Twist  # noqa: E402
from lifecycle_msgs.msg import State, TransitionEvent  # noqa: E402
from lifecycle_msgs.srv import GetState  # noqa: E402
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402

from vsl_ros2.guard_node import GuardConfigError, guard_class  # noqa: E402
from vsl_ros2.release_cli import main as release_cli  # noqa: E402

CONFIG = Path(__file__).resolve().parents[1] / "config" / "nav2_guard.yaml"


def _flatten(prefix, value, out):
    if isinstance(value, dict):
        for key, inner in value.items():
            _flatten(f"{prefix}.{key}" if prefix else key, inner, out)
    else:
        out[prefix] = value


def params_from_config(**overrides):
    """config/nav2_guard.yaml as rclpy Parameters, the way --params-file loads it."""
    data = yaml.safe_load(CONFIG.read_text())["vsl_guard"]["ros__parameters"]
    flat: dict = {}
    _flatten("", data, flat)
    flat.update(overrides)
    return [Parameter(name, value=value) for name, value in flat.items()]


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


class FakeLifecycleNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.label = "active"
        self.create_service(GetState, f"/{name}/get_state", self._get_state)
        self.events = self.create_publisher(TransitionEvent, f"/{name}/transition_event", 10)

    def _get_state(self, _req, res):
        res.current_state = State(id=3 if self.label == "active" else 2, label=self.label)
        return res

    def go(self, label):
        self.label = label
        event = TransitionEvent()
        event.goal_state = State(id=3 if label == "active" else 2, label=label)
        self.events.publish(event)


class FakeNav2:
    def __init__(self, executor):
        self.bt = FakeLifecycleNode("bt_navigator")
        self.cm = FakeLifecycleNode("collision_monitor")
        self.cm_state = self.cm.create_publisher(CollisionMonitorState, "/collision_monitor_state", 10)
        self.cmd_vel = self.cm.create_publisher(Twist, "/cmd_vel", 10)
        self.goals, self.outcome, self.finish = [], {}, set()
        self.accept_delay = 0.0  # a slow Nav2: seconds before a goal is accepted
        self.server = ActionServer(
            self.bt, NavigateToPose, "/navigate_to_pose",
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _h: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )
        self.panel = Node("e_stop_panel")
        self.e_stop_engaged = False
        self.e_stop = self.panel.create_publisher(Bool, "/e_stop", 10)
        self.panel.create_timer(0.1, lambda: self.e_stop.publish(Bool(data=self.e_stop_engaged)))
        for n in (self.bt, self.cm, self.panel):
            executor.add_node(n)

    def _on_goal(self, _request):
        if self.accept_delay:
            time.sleep(self.accept_delay)
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        x = goal_handle.request.pose.pose.position.x
        self.goals.append(x)
        self.outcome[x] = "running"
        feedback = NavigateToPose.Feedback()
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.outcome[x] = "canceled"
                return NavigateToPose.Result()
            if x in self.finish:
                break
            feedback.distance_remaining = 1.0
            goal_handle.publish_feedback(feedback)
            time.sleep(0.05)
        goal_handle.succeed()
        self.outcome[x] = "succeeded"
        return NavigateToPose.Result()

    def collision(self, action, polygon=""):
        self.cm_state.publish(CollisionMonitorState(action_type=action, polygon_name=polygon))

    def destroy(self):
        for n in (self.bt, self.cm, self.panel):
            n.destroy_node()


def goal(x):
    g = NavigateToPose.Goal()
    g.pose.header.frame_id = "map"
    g.pose.pose.position.x = x
    return g


def test_bad_config_is_refused():
    VslGuard = guard_class()
    with pytest.raises(GuardConfigError, match="conditions"):
        VslGuard(parameter_overrides=[Parameter("action_type", value="nav2_msgs/action/NavigateToPose")])
    with pytest.raises(GuardConfigError, match="type must be one of"):
        VslGuard(parameter_overrides=params_from_config(**{"e_stop.type": "magic"}))
    with pytest.raises(GuardConfigError, match="needs 'topic'"):
        VslGuard(parameter_overrides=params_from_config(**{"e_stop.topic": ""}))
    with pytest.raises(GuardConfigError, match="cancel_running needs gate"):
        VslGuard(parameter_overrides=params_from_config(**{"collision_monitor.cancel_running": True}))


def test_vsl_guard_node(tmp_path):
    VslGuard = guard_class()
    executor = MultiThreadedExecutor(num_threads=6)
    nav2 = FakeNav2(executor)
    latch_path = tmp_path / "latch.json"
    params = params_from_config(
        ledger_path=str(tmp_path / "ledger.jsonl"), latch_path=str(latch_path), release_poll_sec=0.1, startup_check_sec=0.5
    )

    planner = Node("planner")
    client = ActionClient(planner, NavigateToPose, "/vsl_guard/navigate_to_pose")
    feedback_seen = []
    executor.add_node(planner)

    def new_guard():
        g = VslGuard(parameter_overrides=params)
        executor.add_node(g)
        return g

    def spin_until(cond, timeout=10.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.05)
            if cond():
                return True
        return False

    def cond(g, name):
        return next(c for c in g.conditions if c.name == name)

    def all_hold(g):
        return all(c.holds() for c in g.conditions)

    def send(x):
        """Returns the planner's goal handle, or None if the guard rejected the goal."""
        f = client.send_goal_async(goal(x), feedback_callback=feedback_seen.append)
        assert spin_until(f.done)
        handle = f.result()
        return handle if handle.accepted else None

    def result_status(handle):
        f = handle.get_result_async()
        assert spin_until(f.done)
        return f.result().status  # 4 succeeded, 5 canceled, 6 aborted

    def running(x):
        return spin_until(lambda: nav2.outcome.get(x) == "running")

    guard = new_guard()
    rogue = None
    try:
        assert spin_until(lambda: all_hold(guard) and guard.upstream.server_is_ready() and client.server_is_ready()), [
            c.status() for c in guard.conditions
        ]

        # 1. All clear: the goal goes through the guard to Nav2, feedback comes back, it arrives.
        h = send(1.0)
        assert h is not None and running(1.0)
        assert spin_until(lambda: len(feedback_seen) > 0)
        nav2.finish.add(1.0)
        assert result_status(h) == 4

        # 2. Collision-monitor STOP mid-goal: Nav2's job. The guard records it (gate: false),
        #    does not cancel the goal, and does not gate on it (a stale STOP would deadlock).
        h = send(1.5)
        assert h is not None and running(1.5)
        nav2.collision(CollisionMonitorState.STOP, "stop_zone")
        assert spin_until(lambda: "collision_monitor" in guard.witness.open_interventions)
        assert "collision_monitor" not in [c.name for c in guard.gating_conditions]
        spin_until(lambda: False, timeout=0.5)
        assert nav2.outcome[1.5] == "running"
        nav2.collision(CollisionMonitorState.DO_NOTHING)
        assert spin_until(lambda: not guard.witness.open_interventions)
        nav2.finish.add(1.5)
        assert result_status(h) == 4

        # 3. The planner cancels its own goal: relayed to Nav2.
        h = send(2.2)
        assert h is not None and running(2.2)
        cf = h.cancel_goal_async()
        assert spin_until(cf.done)
        assert result_status(h) == 5 and nav2.outcome[2.2] == "canceled"

        # 4. A rogue publisher on /cmd_vel mid-goal (cancel_running: true): cancelled upstream,
        #    aborted to the planner, and new goals refused.
        h = send(2.5)
        assert h is not None and running(2.5)
        rogue = Node("teleop_rogue")
        rogue.create_publisher(Twist, "/cmd_vel", 10)
        executor.add_node(rogue)
        assert spin_until(lambda: nav2.outcome.get(2.5) == "canceled")
        assert result_status(h) == 6
        assert send(3.0) is None
        executor.remove_node(rogue)
        rogue.destroy_node()
        rogue = None
        assert spin_until(lambda: cond(guard, "cmd_vel_publishers").holds())

        # 5. E-stop mid-goal (terminal: true): cancelled, latched; the button alone doesn't release it.
        h = send(4.5)
        assert h is not None and running(4.5)
        nav2.e_stop_engaged = True
        assert spin_until(lambda: guard.latch.tripped and nav2.outcome.get(4.5) == "canceled")
        assert result_status(h) == 6
        from diagnostic_msgs.msg import DiagnosticStatus

        assert guard.diagnostic_status().level == DiagnosticStatus.ERROR  # latched, on /diagnostics

        # A release while the e-stop is still engaged is refused.
        assert release_cli(["--latch-path", str(latch_path), "--authorised-by", "J. Smith",
                            "--root-cause", "bay inspected", "--spec-update", "None required."]) == 0
        assert spin_until(lambda: latch_path.with_name("latch.json.release.rejected.json").exists())
        assert "e_stop" in json.loads(latch_path.with_name("latch.json.release.rejected.json").read_text())["rejected"]
        assert guard.latch.tripped

        nav2.e_stop_engaged = False
        assert spin_until(lambda: cond(guard, "e_stop").holds())
        assert all_hold(guard) and send(5.0) is None

        # 6. Restart (launch respawns it; this one "crashes", no stop record): still latched.
        executor.remove_node(guard)
        guard.destroy_node()
        guard = new_guard()
        assert guard.latch.tripped
        spin_until(lambda: False, timeout=1.0)  # its startup check runs and is recorded
        assert spin_until(lambda: all_hold(guard) and guard.upstream.server_is_ready())
        assert send(6.0) is None

        # 7. A named human releases it with evidence, through the release tool.
        assert release_cli(["--latch-path", str(latch_path), "--status"]) == 0
        assert release_cli(["--latch-path", str(latch_path), "--authorised-by", "J. Smith (site safety)",
                            "--root-cause", "E-stop pressed by operator; bay cleared.",
                            "--spec-update", "None required."]) == 0
        assert spin_until(lambda: not guard.latch.tripped)
        h = send(7.0)
        assert h is not None and running(7.0)
        nav2.finish.add(7.0)
        assert result_status(h) == 4

        assert nav2.goals == [1.0, 1.5, 2.2, 2.5, 4.5, 7.0]  # nothing refused ever reached Nav2
        # Finished goals leave no bookkeeping behind (a long-running guard must not grow).
        assert spin_until(lambda: not guard._sent_ids and not guard.upstream.active_goals)
        report = guard.ledger.audit()
        assert report.all_passed, report.violations
        assert guard.ledger.verify_integrity()
        entries = list(guard.ledger.store.all_entries())
        hats = [e for e in entries if e.entry_type.value == "HUMAN_AUTHORISED_TRANSITION"]
        assert len(hats) == 1 and hats[0].payload["authorised_by"] == "J. Smith (site safety)"
        cancels = [e for e in entries if e.entry_type.value == "PRE_NODE" and e.payload.get("fallback") == "cancel the running goal"]
        assert len(cancels) == 2
    finally:
        if rogue is not None:
            rogue.destroy_node()
        guard.destroy_node()
        executor.shutdown()
        planner.destroy_node()
        nav2.destroy()


def _events(ledger):
    return [e.payload.get("event") for e in ledger.store.all_entries() if e.payload.get("event")]


def test_guard_lifecycle_and_unsupervised_goals(tmp_path):
    """The guard's own start/stop, a crash, a config change and goals it doesn't
    supervise are all in the ledger, as VSL constructs audit() checks."""
    VslGuard = guard_class()
    executor = MultiThreadedExecutor(num_threads=6)
    nav2 = FakeNav2(executor)
    base = dict(ledger_path=str(tmp_path / "ledger.jsonl"), latch_path=str(tmp_path / "latch.json"), startup_check_sec=0.5)
    planner = Node("lifecycle_planner")
    via_guard = ActionClient(planner, NavigateToPose, "/vsl_guard/navigate_to_pose")
    direct = ActionClient(planner, NavigateToPose, "/navigate_to_pose")  # bypassing the guard
    executor.add_node(planner)

    def spin_until(cond, timeout=10.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.05)
            if cond():
                return True
        return False

    def start(**overrides):
        g = VslGuard(parameter_overrides=params_from_config(**{**base, **overrides}))
        executor.add_node(g)
        assert spin_until(lambda: all(c.holds() for c in g.gating_conditions) and g.upstream.server_is_ready()
                          and via_guard.server_is_ready())
        return g

    def send(client, x):
        f = client.send_goal_async(goal(x))
        assert spin_until(f.done)
        return f.result()

    def entries(g):
        return list(g.ledger.store.all_entries())

    guard = start()
    try:
        from diagnostic_msgs.msg import DiagnosticStatus

        assert guard.diagnostic_status().level == DiagnosticStatus.OK
        assert _events(guard.ledger) == ["guard_started"]
        assert entries(guard)[0].payload["previous_run"] == "first run"

        # A goal sent to Nav2 directly, around the guard: upstream_goals_supervised is
        # violated, the goal is cancelled, and the bypass is recorded as such.
        h = send(direct, 10.0)
        assert h.accepted and spin_until(lambda: nav2.outcome.get(10.0) == "canceled")
        foreign = [e for e in entries(guard) if e.payload.get("unsupervised") == "foreign"]
        assert foreign and foreign[0].payload["primitive_name"] == "upstream_goals_supervised"

        # The guard crashes mid-goal (no stop record). Nav2 keeps driving the goal...
        assert send(via_guard, 11.0).accepted and spin_until(lambda: nav2.outcome.get(11.0) == "running")
        executor.remove_node(guard)
        guard.destroy_node()
        spin_until(lambda: False, timeout=0.5)
        assert nav2.outcome[11.0] == "running"  # unsupervised: an e-stop now would not cancel it
        # ...until the respawned guard finds it, recognises it as its own orphan, and cancels it.
        guard = start()
        assert spin_until(lambda: nav2.outcome.get(11.0) == "canceled")
        orphaned = [e for e in entries(guard) if e.payload.get("unsupervised") == "orphaned"]
        assert orphaned
        starts = [e for e in entries(guard) if e.payload.get("event") == "guard_started"]
        assert starts[-1].payload["previous_run"] == "ended without a stop record"
        assert "previous_run_ended_unexpectedly" in _events(guard.ledger)

        # A clean stop mid-goal cancels what the guard supervised and records it.
        assert send(via_guard, 12.0).accepted and spin_until(lambda: nav2.outcome.get(12.0) == "running")
        guard.shutdown_cleanly(spin=lambda t: executor.spin_once(timeout_sec=t))
        assert spin_until(lambda: nav2.outcome.get(12.0) == "canceled")
        stop = [e for e in entries(guard) if e.payload.get("event") == "guard_stopped"][-1]
        assert stop.payload["goals_cancelled"] == 1 and stop.payload["cancels_confirmed"] == 1
        executor.remove_node(guard)
        guard.destroy_node()

        # Restarted with a changed configuration: a clean previous run, and a SPECIFICATION_UPDATE.
        guard = start(release_poll_sec=0.25)
        starts = [e for e in entries(guard) if e.payload.get("event") == "guard_started"]
        assert starts[-1].payload["previous_run"] == "stopped cleanly"
        updates = [e for e in entries(guard) if e.entry_type.value == "SPECIFICATION_UPDATE"]
        assert len(updates) == 1 and updates[0].payload["changed"] == ["release_poll_sec"]

        spin_until(lambda: False, timeout=1.0)
        report = guard.ledger.audit()
        assert report.all_passed, report.violations
        assert guard.ledger.verify_integrity()
    finally:
        guard.destroy_node()  # first: releases its execute loops, so the executor can join them
        executor.shutdown()
        planner.destroy_node()
        nav2.destroy()


def test_review_fixes(tmp_path):
    """Code-review findings 1-3: a restored latch is releasable under any configuration;
    a goal Nav2 accepts too late is cancelled; a stopping guard sends nothing new."""
    VslGuard = guard_class()
    executor = MultiThreadedExecutor(num_threads=6)
    nav2 = FakeNav2(executor)
    latch_path = tmp_path / "latch.json"
    base = dict(ledger_path=str(tmp_path / "ledger.jsonl"), latch_path=str(latch_path),
                startup_check_sec=0.5, release_poll_sec=0.1)
    planner = Node("review_planner")
    client = ActionClient(planner, NavigateToPose, "/vsl_guard/navigate_to_pose")
    executor.add_node(planner)
    from rclpy._rclpy_pybind11 import InvalidHandle

    def spin_until(cond, timeout=10.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            try:
                executor.spin_once(timeout_sec=0.05)
            except InvalidHandle:
                # Humble's executor can still hold work for a node this test just destroyed
                # (an in-process restart); a real guard restarts as a new process.
                continue
            if cond():
                return True
        return False

    def start(**overrides):
        g = VslGuard(parameter_overrides=params_from_config(**{**base, **overrides}))
        executor.add_node(g)
        assert spin_until(lambda: all(c.holds() for c in g.gating_conditions if c.name != "e_stop")
                          and g.upstream.server_is_ready() and client.server_is_ready())
        return g

    def send(x):
        f = client.send_goal_async(goal(x))
        assert spin_until(f.done)
        return f.result()

    def result_status(handle, timeout=10.0):
        f = handle.get_result_async()
        assert spin_until(f.done, timeout)
        return f.result().status

    guard = start()
    try:
        # 1. Latched under a configuration where e_stop is terminal...
        nav2.e_stop_engaged = True
        assert spin_until(lambda: guard.latch.tripped)
        nav2.e_stop_engaged = False
        executor.remove_node(guard)
        guard.destroy_node()
        # ...restarted with e_stop no longer terminal: still latched, and still releasable.
        guard = start(**{"e_stop.terminal": False})
        assert guard.latch.tripped and not guard.terminal_conditions
        assert send(20.0).accepted is False
        assert release_cli(["--latch-path", str(latch_path), "--authorised-by", "J. Smith",
                            "--root-cause", "e-stop test", "--spec-update", "e_stop no longer terminal"]) == 0
        assert spin_until(lambda: not guard.latch.tripped)
        hats = [e for e in guard.ledger.store.all_entries() if e.entry_type.value == "HUMAN_AUTHORISED_TRANSITION"]
        assert len(hats) == 1

        # 2. Nav2 accepts a goal after the guard gave up waiting: the planner is told it
        #    failed, and the guard cancels the goal in Nav2 as soon as it is accepted.
        executor.remove_node(guard)
        guard.destroy_node()
        guard = start(upstream_accept_timeout_sec=0.5)
        nav2.accept_delay = 1.5
        h = send(21.0)
        assert h.accepted
        assert result_status(h) == 6  # aborted after the 0.5 s accept timeout
        assert spin_until(lambda: nav2.outcome.get(21.0) == "canceled")
        late = [e for e in guard.ledger.store.all_entries()
                if e.entry_type.value == "PRE_NODE" and "accepted after the guard stopped waiting" in str(e.payload.get("reason"))]
        assert late
        nav2.accept_delay = 0.0

        # 3. Once the guard is stopping, it accepts and forwards nothing new.
        guard.shutdown_cleanly(spin=lambda t: executor.spin_once(timeout_sec=t))
        assert send(22.0).accepted is False
        spin_until(lambda: False, timeout=0.5)
        assert 22.0 not in nav2.goals

        spin_until(lambda: False, timeout=1.0)
        report = guard.ledger.audit()
        assert report.all_passed, report.violations
    finally:
        guard.destroy_node()
        executor.shutdown()
        planner.destroy_node()
        nav2.destroy()
