"""Conditions and the witness against a stand-in node; runs without ROS 2."""

from __future__ import annotations

import importlib
import sys
import time
import types
from types import SimpleNamespace

import pytest

from vsl_core.constructs import TerminalState
from vsl_core.governance import GovernanceAuthority, HumanAuthorisedTransition
from vsl_core.identity import Evidence
from vsl_core.ledger import InMemoryLedgerStore, VerbaLedger

from conftest import FakeLogger, Twist
from vsl_ros2 import (
    DIAG_ERROR,
    DIAG_OK,
    DIAG_WARN,
    CollisionMonitorCondition,
    Condition,
    DiagnosticsCondition,
    GateRunner,
    InterventionWitness,
    LifecycleCondition,
    LockCondition,
    ROS2Adapter,
    SolePublisherCondition,
    TerminalLatch,
    TopicCondition,
    VSLGateBlocked,
)

E_STOPPED = TerminalState(name="robot-e-stopped", description="held")


class _CMS:
    DO_NOTHING, STOP, SLOWDOWN, APPROACH = 0, 1, 2, 3


_FAKES = {
    "diagnostic_msgs.msg": {"DiagnosticArray": type("DiagnosticArray", (), {})},
    "lifecycle_msgs.msg": {"TransitionEvent": type("TransitionEvent", (), {})},
    "lifecycle_msgs.srv": {"GetState": type("GetState", (), {"Request": type("Request", (), {})})},
    "nav2_msgs.msg": {"CollisionMonitorState": _CMS},
    "std_msgs.msg": {"Bool": type("Bool", (), {})},
}


@pytest.fixture(autouse=True)
def ros_msgs(monkeypatch):
    """Use the real message packages when ROS 2 is sourced, stand-ins otherwise."""
    for module, attrs in _FAKES.items():
        try:
            importlib.import_module(module)
        except ImportError:
            package = module.split(".")[0]
            pkg = sys.modules.get(package) or types.ModuleType(package)
            sub = types.ModuleType(module)
            for name, value in attrs.items():
                setattr(sub, name, value)
            setattr(pkg, module.split(".")[1], sub)
            monkeypatch.setitem(sys.modules, package, pkg)
            monkeypatch.setitem(sys.modules, module, sub)


class FakeTimer:
    def __init__(self, fn):
        self.fn, self.cancelled = fn, False

    def cancel(self):
        self.cancelled = True


class FakeNode:
    def __init__(self):
        self.subs, self.timers, self.publishers_by_topic = {}, [], {}
        self.logger = FakeLogger()
        self.logger.info = lambda text: None

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subs[topic] = callback

    def create_timer(self, period, fn):
        timer = FakeTimer(fn)
        self.timers.append(timer)
        return timer

    def create_client(self, srv_type, name):
        return SimpleNamespace(service_is_ready=lambda: False)

    def get_publishers_info_by_topic(self, topic):
        return [SimpleNamespace(node_name=n.rsplit("/", 1)[1], node_namespace=n.rsplit("/", 1)[0] or "/")
                for n in self.publishers_by_topic.get(topic, [])]

    def get_logger(self):
        return self.logger

    def get_fully_qualified_name(self):
        return "/guard"

    def tick(self):
        for t in list(self.timers):
            if not t.cancelled:
                t.fn()


def diag(*statuses):
    return SimpleNamespace(status=[SimpleNamespace(name=n, level=bytes([lv]), message=m) for n, lv, m in statuses])


def check(condition: Condition) -> bool:
    gate = ROS2Adapter(identity_key="/guard").compile_invariant(condition.invariant())
    try:
        GateRunner().check(gate, None)
        return True
    except VSLGateBlocked:
        return False


# -- each ROS function's verdict -----------------------------------------------


def test_diagnostics():
    node = FakeNode()
    c = DiagnosticsCondition(node, ["/Drive", "/Lidar"])
    assert not c.holds() and "no message" in c.status().detail
    node.subs["/diagnostics_agg"](diag(("/Drive", DIAG_OK, "")))
    assert not c.holds() and "/Lidar: not reported" in c.status().detail
    node.subs["/diagnostics_agg"](diag(("/Drive", DIAG_OK, ""), ("/Lidar", DIAG_WARN, "dusty")))
    assert c.holds() and check(c)
    node.subs["/diagnostics_agg"](diag(("/Lidar", DIAG_ERROR, "no scans")))
    assert not c.holds() and "ERROR no scans" in c.status().detail and not check(c)
    node.subs["/diagnostics_agg"](diag(("/Other", DIAG_ERROR, "")))  # not ours: ignored
    assert "no scans" in c.status().detail


def test_diagnostics_stale_and_int_levels():
    node = FakeNode()
    c = DiagnosticsCondition(node, ["/Drive"], max_age_sec=0.05)
    node.subs["/diagnostics_agg"](SimpleNamespace(status=[SimpleNamespace(name="/Drive", level=0, message="")]))
    assert c.holds()
    time.sleep(0.08)
    assert not c.holds() and "stale" in c.status().detail
    node.subs["/diagnostics_agg"](diag(("/Drive", 3, "stale")))  # STALE never passes
    assert not c.holds()


def test_lifecycle():
    node = FakeNode()
    c = LifecycleCondition(node, "bt_navigator")
    assert c.name == "ros2.lifecycle:/bt_navigator" and not c.holds()
    event = lambda label: SimpleNamespace(goal_state=SimpleNamespace(label=label))  # noqa: E731
    node.subs["/bt_navigator/transition_event"](event("active"))
    assert c.holds()
    node.subs["/bt_navigator/transition_event"](event("inactive"))
    assert not c.holds() and "inactive" in c.status().detail


def test_lifecycle_seeded_by_get_state():
    node = FakeNode()
    c = LifecycleCondition(node, "/controller_server")
    future = SimpleNamespace(result=lambda: SimpleNamespace(current_state=SimpleNamespace(label="active")))
    c._on_state(future)
    assert c.holds()


def test_lifecycle_query_retries_a_lost_or_failed_reply():
    node = FakeNode()
    calls = []

    class Pending:
        def __init__(self):
            self.cb = None

        def add_done_callback(self, cb):
            self.cb = cb

    def call_async(_request):
        calls.append(Pending())
        return calls[-1]

    node.create_client = lambda srv, name: SimpleNamespace(service_is_ready=lambda: True, call_async=call_async)
    c = LifecycleCondition(node, "/bt_navigator", query_timeout_sec=0.05)
    node.tick()
    node.tick()
    assert len(calls) == 1  # asked once, the answer may still come
    time.sleep(0.06)
    node.tick()
    assert len(calls) == 2  # no answer in time: asked again

    def failing():
        raise RuntimeError("service went away")

    calls[-1].cb(SimpleNamespace(result=failing))
    node.tick()
    assert len(calls) == 3 and not c.holds()  # an error answer: asked again, still fails closed
    calls[-1].cb(SimpleNamespace(result=lambda: SimpleNamespace(current_state=SimpleNamespace(label="active"))))
    assert c.holds()
    node.tick()
    assert len(calls) == 3  # known now: no more queries


def test_collision_monitor():
    node = FakeNode()
    c = CollisionMonitorCondition(node)
    assert not c.holds()  # fail closed until it speaks
    node.subs["/collision_monitor_state"](SimpleNamespace(action_type=1, polygon_name="stop_zone"))
    assert not c.holds() and "STOP (stop_zone)" in c.status().detail
    node.subs["/collision_monitor_state"](SimpleNamespace(action_type=2, polygon_name="slow_zone"))
    assert c.holds()  # slowing down is the monitor's job, not a reason to refuse a goal
    strict = CollisionMonitorCondition(FakeNode(), blocking_actions=("STOP", "SLOWDOWN"), assume_clear_until_first=True)
    assert strict.holds()
    with pytest.raises(ValueError):
        CollisionMonitorCondition(FakeNode(), blocking_actions=("PANIC",))


def test_lock_and_topic_condition():
    node = FakeNode()
    lock = LockCondition(node, "/e_stop")
    assert not lock.holds()  # engaged until heard
    node.subs["/e_stop"](SimpleNamespace(data=False))
    assert lock.holds()
    node.subs["/e_stop"](SimpleNamespace(data=True))
    assert not lock.holds()

    battery = TopicCondition(node, object, "/battery", lambda m: m.percentage > 0.2, name="battery", description="d")
    node.subs["/battery"](SimpleNamespace(percentage=0.5))
    assert battery.holds()
    node.subs["/battery"](SimpleNamespace())  # predicate raises: does not hold
    assert not battery.holds() and "predicate failed" in battery.status().detail


def test_sole_publisher():
    node = FakeNode()
    node.publishers_by_topic["/cmd_vel"] = ["/collision_monitor"]
    c = SolePublisherCondition(node, "/cmd_vel", ["collision_monitor"])
    assert c.holds()
    node.publishers_by_topic["/cmd_vel"] = ["/collision_monitor", "/teleop/joy_node"]
    node.tick()
    assert not c.holds() and "/teleop/joy_node" in c.status().detail


def test_invariant_metadata():
    c = LockCondition(FakeNode(), "/e_stop", name="ros2.e_stop")
    inv = c.invariant(on_violation=E_STOPPED)
    assert inv.name == "ros2.e_stop" and inv.on_violation is E_STOPPED
    assert inv.assurance_basis.f1_pre_commitment and inv.assurance_basis.f2_modification.value == "NONE"


# -- persistent latch -------------------------------------------------------------


def _transition():
    return HumanAuthorisedTransition.authorise(
        GovernanceAuthority(name="Safety officer", escalation_contact="s@example.test"),
        authorised_by="A. Operator",
        evidence=Evidence(root_cause_analysis="reviewed", specification_update_proposal="none"),
    )


def test_latch_survives_restart(tmp_path):
    from vsl_core.exceptions import InvariantViolation

    path = tmp_path / "latch.json"
    latch = TerminalLatch(state_path=path)
    latch.trip(
        InvariantViolation(reason="e-stop", identity_key="/guard", invariant_name="ros2.e_stop", terminal_state_name="robot-e-stopped"),
        terminal_entry_id="entry-1",
    )
    restarted = TerminalLatch(state_path=path)
    assert restarted.tripped and restarted.terminal_state_name == "robot-e-stopped"
    assert restarted.terminal_entry_id == "entry-1" and restarted.tripped_by.invariant_name == "ros2.e_stop"
    restarted.release(_transition())
    assert not path.exists() and not TerminalLatch(state_path=path).tripped


def test_unreadable_latch_file_stays_tripped(tmp_path):
    path = tmp_path / "latch.json"
    path.write_text("{not json")
    latch = TerminalLatch(state_path=path)
    assert latch.tripped and "unreadable" in latch.tripped_by.reason


# -- witness ------------------------------------------------------------------------


def _types(ledger):
    return [e.entry_type.value for e in ledger.store.all_entries()]


def test_witness_records_intervention_and_clear():
    node, ledger = FakeNode(), VerbaLedger(InMemoryLedgerStore())
    cm = CollisionMonitorCondition(node, assume_clear_until_first=True)
    InterventionWitness(node, ledger, [cm])
    node.subs["/collision_monitor_state"](SimpleNamespace(action_type=1, polygon_name="stop_zone"))
    assert _types(ledger) == ["MONITOR", "PRE_NODE"]
    monitor = next(iter(ledger.store.all_entries()))
    assert monitor.payload["primitive_name"] == "nav2.collision_monitor" and monitor.payload["allowed"] is False
    assert not ledger.audit().all_passed  # the hold is still open
    node.subs["/collision_monitor_state"](SimpleNamespace(action_type=1, polygon_name="stop_zone"))  # repeat: no new record
    node.subs["/collision_monitor_state"](SimpleNamespace(action_type=0, polygon_name=""))
    assert _types(ledger) == ["MONITOR", "PRE_NODE", "VERIFICATION"]
    assert ledger.audit().all_passed


def test_witness_records_expiry():
    node, ledger = FakeNode(), VerbaLedger(InMemoryLedgerStore())
    d = DiagnosticsCondition(node, ["/Drive"], max_age_sec=0.05)
    InterventionWitness(node, ledger, [d])
    node.subs["/diagnostics_agg"](diag(("/Drive", DIAG_OK, "")))
    assert _types(ledger) == []  # first verdict healthy: nothing to record
    time.sleep(0.08)
    node.tick()  # the witness timer notices the expiry
    assert _types(ledger) == ["MONITOR", "PRE_NODE"]


def test_witness_e_stop_terminal_state_across_restart(tmp_path):
    node, ledger = FakeNode(), VerbaLedger(InMemoryLedgerStore())
    path = tmp_path / "latch.json"
    e_stop = LockCondition(node, "/e_stop", name="ros2.e_stop")
    latch = TerminalLatch(state_path=path)
    InterventionWitness(node, ledger, [e_stop], latch=latch, latch_on=[e_stop], terminal_state=E_STOPPED)
    node.subs["/e_stop"](SimpleNamespace(data=False))
    node.subs["/e_stop"](SimpleNamespace(data=True))
    assert latch.tripped and _types(ledger) == ["MONITOR", "PRE_NODE", "VERIFICATION", "TERMINAL"]
    node.subs["/e_stop"](SimpleNamespace(data=False))
    assert latch.tripped  # releasing the button does not leave the Terminal State

    # The node restarts: a new latch and witness, the same files.
    node2 = FakeNode()
    e_stop2 = LockCondition(node2, "/e_stop", name="ros2.e_stop")
    latch2 = TerminalLatch(state_path=path)
    witness2 = InterventionWitness(node2, ledger, [e_stop2], latch=latch2, latch_on=[e_stop2], terminal_state=E_STOPPED)
    assert latch2.tripped
    node2.subs["/e_stop"](SimpleNamespace(data=False))
    witness2.release(_transition())
    assert not latch2.tripped
    assert _types(ledger)[-1] == "HUMAN_AUTHORISED_TRANSITION"
    assert ledger.audit().all_passed, ledger.audit().violations


def test_witness_engaged_at_startup_trips():
    node, ledger = FakeNode(), VerbaLedger(InMemoryLedgerStore())
    e_stop = LockCondition(node, "/e_stop")
    latch = TerminalLatch()
    InterventionWitness(node, ledger, [e_stop], latch=latch, latch_on=[e_stop], terminal_state=E_STOPPED)
    node.subs["/e_stop"](SimpleNamespace(data=True))
    assert latch.tripped


def test_witness_arguments():
    node = FakeNode()
    lock = LockCondition(node, "/e_stop")
    with pytest.raises(ValueError):
        InterventionWitness(node, VerbaLedger(), [lock], latch_on=[lock])
    with pytest.raises(ValueError):
        InterventionWitness(node, VerbaLedger(), [], latch=TerminalLatch(), latch_on=[lock], terminal_state=E_STOPPED)


def test_conditions_chain_in_front_of_a_goal():
    node = FakeNode()
    e_stop = LockCondition(node, "/e_stop")
    cm = CollisionMonitorCondition(node)
    adapter = ROS2Adapter(identity_key="/guard")
    gates = [adapter.compile_invariant(c.invariant()) for c in (e_stop, cm)]
    runner = GateRunner()
    node.subs["/e_stop"](SimpleNamespace(data=False))
    with pytest.raises(VSLGateBlocked) as info:
        runner.check(gates, Twist())
    assert info.value.denial.invariant_name == "nav2.collision_monitor"
    node.subs["/collision_monitor_state"](SimpleNamespace(action_type=0, polygon_name=""))
    runner.check(gates, Twist())
