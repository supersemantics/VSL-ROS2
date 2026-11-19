"""GoalSupervisor: cancel a running goal when the robot stops being fit for it.

A goal is checked once, when ``GatedActionClient`` sends it. A navigation or
manipulation goal can then run for minutes. If the robot enters a Terminal
State meanwhile (an e-stop latched), ROS 2 stops the motion, but the goal is
still active on the server: once the e-stop button is released the server
carries on driving to it, although the guard is still latched and waiting for
a human. The latch blocks new goals; it has to cancel the running ones too.

``GoalSupervisor`` re-checks every goal its ``GatedActionClient`` is tracking,
on a node timer (``period_sec``, 0.1 s by default), and asks the server to
cancel any goal that no longer passes:

- always, while the latch is tripped (the Terminal State);
- and when any of ``gates`` denies -- the gates that must keep holding while a
  goal runs. Choose them per ROS 2 function, not "all of them": a Nav2
  collision-monitor STOP is not a reason to cancel, because the collision
  monitor already stops the robot and resumes when the obstacle clears, which
  is its job. A component's diagnostics going to ERROR, or a foreign publisher
  appearing on the driver's ``cmd_vel``, is.

Cancelling is a request: the server decides. ``on_cancel(active, blocked)``
runs when the request is sent and ``on_cancel_done(active, blocked, accepted)``
when the server has answered, so the application can record both (the
supervisor writes no ledger). A refused cancel is logged as an error: the
goal is still running and someone needs to know.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from typing import Any

from vsl_core.conformance.protocol import CompiledGate

from .conditions import steady_timer
from .integration import ActiveGoal, GatedActionClient
from .runner import GateRunner, TerminalLatch, VSLGateBlocked, default_runner

OnCancel = Callable[[ActiveGoal, VSLGateBlocked], None]
OnCancelDone = Callable[[ActiveGoal, VSLGateBlocked, bool], None]

_SAME_AS_CLIENT = object()


def cancel_accepted(response: Any) -> bool:
    """True if an ``action_msgs/CancelGoal`` response says the goal is being cancelled."""
    return getattr(response, "return_code", 1) == 0 and bool(getattr(response, "goals_canceling", ()))


class GoalSupervisor:
    def __init__(
        self,
        node: Any,
        client: GatedActionClient,
        *,
        gates: Sequence[CompiledGate] = (),
        latch: TerminalLatch | None | object = _SAME_AS_CLIENT,
        period_sec: float = 0.1,
        runner: GateRunner | None = None,
        on_cancel: OnCancel | None = None,
        on_cancel_done: OnCancelDone | None = None,
    ) -> None:
        self.client = client
        self.gates = tuple(gates)
        self.latch = client.latch if latch is _SAME_AS_CLIENT else latch
        if not self.gates and self.latch is None:
            raise ValueError("GoalSupervisor needs gates to re-check, a latch, or both.")
        self._runner = runner
        self._on_cancel = on_cancel
        self._on_cancel_done = on_cancel_done
        self._logger = node.get_logger()
        self._timer = steady_timer(node, period_sec, self.check_now)

    def _blocked(self, active: ActiveGoal) -> VSLGateBlocked | None:
        target = self.client.target
        if self.latch is not None and self.latch.tripped:
            return VSLGateBlocked(
                f"in terminal state '{self.latch.terminal_state_name}'; cancelling the running goal",
                target=target,
                denial=self.latch.tripped_by,
                latched=True,
            )
        if not self.gates:
            return None
        try:
            (self._runner or default_runner()).check(self.gates, active.goal, target=target, latch=self.latch)
        except VSLGateBlocked as blocked:
            return blocked
        return None

    def check_now(self) -> None:
        """Re-check every running goal and cancel those that no longer pass."""
        for active in self.client.active_goals:
            if active.cancel_requested:
                continue
            blocked = self._blocked(active)
            if blocked is None:
                continue
            future = self.client.cancel_goal(active, blocked)
            if future is None:
                continue
            self._logger.warning(f"VSL: cancelling goal {active.goal_id[:8]}: {blocked.reason}")
            self._call(self._on_cancel, active, blocked)
            future.add_done_callback(lambda done, a=active, b=blocked: self._done(a, b, done))

    def _done(self, active: ActiveGoal, blocked: VSLGateBlocked, future: Any) -> None:
        try:
            accepted = cancel_accepted(future.result())
        except Exception:  # noqa: BLE001
            accepted = False
        if not accepted:
            self._logger.error(
                f"VSL: the server refused to cancel goal {active.goal_id[:8]} ({blocked.reason}); it is still running"
            )
        self._call(self._on_cancel_done, active, blocked, accepted)

    def _call(self, fn: Callable[..., None] | None, *args: Any) -> None:
        if fn is None:
            return
        try:
            fn(*args)
        except Exception as error:  # noqa: BLE001 -- a handler bug must not stop supervision
            self._logger.error(f"vsl-ros2 GoalSupervisor handler raised {type(error).__name__}: {error}")
