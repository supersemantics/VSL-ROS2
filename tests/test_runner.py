import asyncio
import threading
import time

import pytest

from conftest import E_STOP, Twist, finite_speed_invariant, speed_headroom_gate
from vsl_core.exceptions import AutomationDeniedException, InvariantViolation
from vsl_core.governance import GovernanceAuthority, HumanAuthorisedTransition
from vsl_core.identity import Evidence

from vsl_ros2 import GateRunner, TerminalLatch, VSLGateBlocked


@pytest.fixture
def runner():
    r = GateRunner(timeout_sec=0.5)
    yield r
    r.close()


def _transition() -> HumanAuthorisedTransition:
    authority = GovernanceAuthority(name="Safety officer", escalation_contact="safety@example.test")
    evidence = Evidence(
        root_cause_analysis="Planner emitted NaN after a localisation reset.",
        specification_update_proposal="Reject commands until localisation reports healthy.",
    )
    return HumanAuthorisedTransition.authorise(authority, authorised_by="A. Operator", evidence=evidence)


def test_allows(runner):
    runner.check(speed_headroom_gate(), Twist(0.5))


def test_denial_is_blocked_with_the_denial(runner):
    with pytest.raises(VSLGateBlocked) as info:
        runner.check(speed_headroom_gate(), Twist(2.0), target="/cmd_vel")
    blocked = info.value
    assert isinstance(blocked.denial, AutomationDeniedException)
    assert blocked.error is None and blocked.target == "/cmd_vel"
    assert "speed-headroom" in str(blocked)


def test_chain_stops_at_first_denial(runner):
    calls = []

    async def second(_):
        calls.append("second")

    with pytest.raises(VSLGateBlocked) as info:
        runner.check([finite_speed_invariant(), second], Twist(float("nan")))
    assert isinstance(info.value.denial, InvariantViolation)
    assert info.value.terminal_state_name == E_STOP.name
    assert calls == []


def test_gate_error_fails_closed(runner):
    async def broken(_):
        raise RuntimeError("lidar driver gone")

    with pytest.raises(VSLGateBlocked) as info:
        runner.check(broken, Twist(0.1))
    assert isinstance(info.value.error, RuntimeError) and info.value.denial is None


def test_timeout_fails_closed(runner):
    async def hung(_):
        await asyncio.sleep(5)

    start = time.monotonic()
    with pytest.raises(VSLGateBlocked, match="did not decide"):
        runner.check(hung, Twist(0.1), timeout_sec=0.1)
    assert time.monotonic() - start < 1.0


def test_extraction_error_fails_closed(runner):
    with pytest.raises(VSLGateBlocked, match="could not extract"):
        runner.evaluate(speed_headroom_gate(), {"no": "twist"}, lambda raw: raw["linear"])


def test_gate_reused_across_threads(runner):
    gate = speed_headroom_gate()
    results = []

    def worker(speed):
        try:
            runner.check(gate, Twist(speed))
            results.append((speed, True))
        except VSLGateBlocked:
            results.append((speed, False))

    threads = [threading.Thread(target=worker, args=(s,)) for s in (0.2, 3.0, 0.4, 5.0) * 5]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert all(ok == (speed < 1.0) for speed, ok in results) and len(results) == 20


def test_latch_trips_on_terminal_invariant_and_blocks_everything(runner):
    latch = TerminalLatch()
    ran = []

    async def tracking(_):
        ran.append(1)

    with pytest.raises(VSLGateBlocked):
        runner.check(finite_speed_invariant(), Twist(float("inf")), latch=latch)
    assert latch.tripped and latch.terminal_state_name == E_STOP.name

    with pytest.raises(VSLGateBlocked) as info:
        runner.check(tracking, Twist(0.0), latch=latch)
    assert info.value.latched and ran == []

    latch.release(_transition())
    runner.check(tracking, Twist(0.0), latch=latch)
    assert ran == [1] and len(latch.releases) == 1


def test_invariant_without_terminal_state_does_not_latch(runner):
    latch = TerminalLatch()
    with pytest.raises(VSLGateBlocked):
        runner.check(finite_speed_invariant(on_violation=None), Twist(float("nan")), latch=latch)
    assert not latch.tripped


def test_prenode_denial_does_not_latch(runner):
    latch = TerminalLatch()
    with pytest.raises(VSLGateBlocked):
        runner.check(speed_headroom_gate(), Twist(9.0), latch=latch)
    assert not latch.tripped


def test_latch_release_requires_a_human_authorised_transition():
    latch = TerminalLatch()
    with pytest.raises(ValueError):
        latch.release(_transition())  # not tripped
    latch.trip(InvariantViolation(reason="r", identity_key="k", invariant_name="i", terminal_state_name="t"))
    with pytest.raises(TypeError):
        latch.release("ok")  # type: ignore[arg-type]
    assert latch.tripped


def test_bad_arguments():
    with pytest.raises(ValueError):
        GateRunner(timeout_sec=0)
    with pytest.raises(ValueError):
        GateRunner().check([], None)
    with pytest.raises(TypeError):
        GateRunner().check([object()], None)


def test_runner_restarts_after_close(runner):
    runner.check(speed_headroom_gate(), Twist(0.1))
    runner.close()
    runner.check(speed_headroom_gate(), Twist(0.1))
