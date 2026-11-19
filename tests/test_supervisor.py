"""GatedActionClient goal tracking and GoalSupervisor, against stand-ins; no ROS 2 needed."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from conftest import FakeLogger, Twist, finite_speed_invariant, speed_headroom_gate
from vsl_core.exceptions import InvariantViolation
from vsl_core.governance import GovernanceAuthority, HumanAuthorisedTransition
from vsl_core.identity import Evidence

from vsl_ros2 import GatedActionClient, GoalSupervisor, TerminalLatch, cancel_accepted, goal_id_hex


class FakeFuture:
    def __init__(self):
        self._done, self._result, self._callbacks = False, None, []

    def add_done_callback(self, fn):
        if self._done:
            fn(self)
        else:
            self._callbacks.append(fn)

    def set_result(self, value):
        self._done, self._result = True, value
        for fn in self._callbacks:
            fn(self)

    def result(self):
        return self._result

    def done(self):
        return self._done


class FakeGoalHandle:
    def __init__(self, n, accepted=True, cancel_ok=True):
        self.accepted = accepted
        self.goal_id = SimpleNamespace(uuid=[n] * 16)
        self.result_future = FakeFuture()
        self.cancel_requests = 0
        self.cancel_ok = cancel_ok

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_requests += 1
        future = FakeFuture()
        future.set_result(SimpleNamespace(return_code=0 if self.cancel_ok else 1, goals_canceling=[1] if self.cancel_ok else []))
        return future


class FakeActionClient:
    _action_name = "/navigate_to_pose"

    def __init__(self):
        self.handles, self.pending = [], []

    def send_goal_async(self, goal, **kwargs):
        future = FakeFuture()
        self.pending.append(future)
        return future

    def accept(self, cancel_ok=True):
        handle = FakeGoalHandle(len(self.handles) + 1, cancel_ok=cancel_ok)
        self.handles.append(handle)
        self.pending.pop(0).set_result(handle)
        return handle


class FakeNode:
    def __init__(self):
        self.logger, self.timers = FakeLogger(), []

    def get_logger(self):
        return self.logger

    def create_timer(self, period, fn):
        self.timers.append(fn)


def _transition():
    return HumanAuthorisedTransition.authorise(
        GovernanceAuthority(name="Safety officer", escalation_contact="s@example.test"),
        authorised_by="A. Operator",
        evidence=Evidence(root_cause_analysis="r", specification_update_proposal="p"),
    )


def _trip(latch):
    latch.trip(InvariantViolation(reason="e-stop", identity_key="/guard", invariant_name="ros2.e_stop", terminal_state_name="robot-e-stopped"))


def test_accepted_goals_are_tracked_until_they_finish():
    raw = FakeActionClient()
    client = GatedActionClient(raw, speed_headroom_gate())
    client.send_goal_async(Twist(0.1))
    assert client.active_goals == []  # not accepted yet
    handle = raw.accept()
    [active] = client.active_goals
    assert active.goal_id == goal_id_hex(handle.goal_id) == ("01" * 16)
    handle.result_future.set_result(SimpleNamespace(status=4))
    assert client.active_goals == []


def test_rejected_goals_are_not_tracked():
    raw = FakeActionClient()
    client = GatedActionClient(raw, speed_headroom_gate())
    future = client.send_goal_async(Twist(0.1))
    raw.pending.pop(0).set_result(FakeGoalHandle(1, accepted=False))
    assert future.done() and client.active_goals == []


def test_supervisor_cancels_running_goal_when_latched():
    raw, latch, node = FakeActionClient(), TerminalLatch(), FakeNode()
    client = GatedActionClient(raw, speed_headroom_gate(), latch=latch)
    cancelled, done = [], []
    supervisor = GoalSupervisor(
        node, client, on_cancel=lambda a, b: cancelled.append(b), on_cancel_done=lambda a, b, ok: done.append(ok)
    )
    client.send_goal_async(Twist(0.1))
    handle = raw.accept()
    supervisor.check_now()
    assert handle.cancel_requests == 0  # fit: keeps running
    _trip(latch)
    supervisor.check_now()
    supervisor.check_now()  # asks once
    assert handle.cancel_requests == 1 and cancelled[0].latched and done == [True]
    assert node.timers  # runs on a node timer


def test_supervisor_cancels_when_a_must_hold_gate_denies():
    raw, node = FakeActionClient(), FakeNode()
    state = {"healthy": True}

    async def healthy(_goal):
        return state["healthy"]

    from vsl_core.constructs import Invariant
    from vsl_ros2 import NO_MODEL, ROS2Adapter

    health = ROS2Adapter().compile_invariant(Invariant(name="diag", description="d", rule=healthy, assurance_basis=NO_MODEL))
    client = GatedActionClient(raw, [health, speed_headroom_gate()])
    reasons = []
    supervisor = GoalSupervisor(node, client, gates=[health], on_cancel=lambda a, b: reasons.append(b.denial.invariant_name))
    client.send_goal_async(Twist(0.1))
    handle = raw.accept()
    supervisor.check_now()
    assert handle.cancel_requests == 0
    state["healthy"] = False
    supervisor.check_now()
    assert handle.cancel_requests == 1 and reasons == ["diag"]


def test_refused_cancel_is_reported():
    raw, latch, node = FakeActionClient(), TerminalLatch(), FakeNode()
    client = GatedActionClient(raw, speed_headroom_gate(), latch=latch)
    done = []
    supervisor = GoalSupervisor(node, client, on_cancel_done=lambda a, b, ok: done.append(ok))
    client.send_goal_async(Twist(0.1))
    raw.accept(cancel_ok=False)
    _trip(latch)
    supervisor.check_now()
    assert done == [False] and "still running" in node.logger.errors[0]


def test_handler_errors_do_not_stop_supervision():
    raw, latch, node = FakeActionClient(), TerminalLatch(), FakeNode()
    client = GatedActionClient(raw, speed_headroom_gate(), latch=latch)

    def boom(*_):
        raise RuntimeError("bug")

    supervisor = GoalSupervisor(node, client, on_cancel=boom)
    for _ in range(2):
        client.send_goal_async(Twist(0.1))
        raw.accept()
    _trip(latch)
    supervisor.check_now()
    assert all(h.cancel_requests == 1 for h in raw.handles)


def test_supervisor_needs_something_to_check():
    with pytest.raises(ValueError):
        GoalSupervisor(FakeNode(), GatedActionClient(FakeActionClient(), finite_speed_invariant()))


def test_cancel_accepted():
    assert cancel_accepted(SimpleNamespace(return_code=0, goals_canceling=[1]))
    assert not cancel_accepted(SimpleNamespace(return_code=0, goals_canceling=[]))
    assert not cancel_accepted(SimpleNamespace(return_code=2, goals_canceling=[1]))
