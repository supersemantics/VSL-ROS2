"""Running async VSL gates from synchronous ROS 2 callbacks, fail-closed.

vsl-core gates are ``async`` callables. rclpy callbacks (subscriptions,
services, action goal callbacks, timers) are usually plain functions run by
an rclpy executor, which is not an asyncio event loop. ``GateRunner`` bridges
the two: it owns one private asyncio loop on a daemon thread, submits each
gate to it and waits for the result, with a timeout.

Every way a gate can fail to say "allowed" blocks the action:

- the gate raises ``AutomationDeniedException`` (or ``InvariantViolation``): a denial;
- the gate raises anything else, or ``candidate_input_fn`` does: a gate error;
- the gate does not finish within ``timeout_sec``: a gate error (a hung monitor
  must not leave a robot moving);
- a ``TerminalLatch`` the action shares has tripped: blocked without running the gates.

All four surface as ``VSLGateBlocked``.

A gate's monitor must not wait on a ROS callback of the same node (for example
by spinning, or waiting for a service response delivered by the executor that
is currently blocked in this gate): that deadlocks until the timeout, and the
action is then blocked. Read state the monitor needs from attributes the node's
own subscriptions keep up to date.
"""

from __future__ import annotations

import asyncio
import concurrent.futures
import json
import os
import threading
import time
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import Any, Union

from vsl_core.conformance.protocol import CompiledGate
from vsl_core.exceptions import AutomationDeniedException, InvariantViolation
from vsl_core.governance import HumanAuthorisedTransition

DEFAULT_TIMEOUT_SEC = 1.0

Gates = Union[CompiledGate, Sequence[CompiledGate]]


class VSLGateBlocked(Exception):
    """A guarded ROS 2 action did not happen because a VSL gate did not allow it.

    ``denial`` is the gate's ``AutomationDeniedException`` (or the more specific
    ``InvariantViolation``) when a gate denied; ``error`` is the exception when the
    gate could not decide (it raised, timed out, or the input could not be
    extracted). Exactly one of the two is set, unless ``latched`` is True: then a
    tripped ``TerminalLatch`` blocked the action before any gate ran, and
    ``denial`` is the violation that tripped it. ``target`` names what was blocked
    (a topic, service or action name).
    """

    def __init__(
        self,
        reason: str,
        *,
        target: str | None = None,
        denial: AutomationDeniedException | None = None,
        error: BaseException | None = None,
        latched: bool = False,
    ) -> None:
        where = f" on '{target}'" if target else ""
        super().__init__(f"VSL gate blocked{where}: {reason}")
        self.reason = reason
        self.target = target
        self.denial = denial
        self.error = error
        self.latched = latched

    @property
    def terminal_state_name(self) -> str | None:
        """The TerminalState a violated Invariant entered, if any."""
        if isinstance(self.denial, InvariantViolation):
            return self.denial.terminal_state_name
        return None


class TerminalLatch:
    """Holds a robot (or one capability of it) in a Terminal State.

    vsl-core's TerminalState is a state "from which no further automated
    transitions are permitted", left only through a Human-Authorised Transition.
    Share one latch between the wrappers that guard a capability: once an
    ``InvariantViolation`` whose Invariant names an ``on_violation`` TerminalState
    blocks any of them, the latch trips and every wrapper sharing it blocks every
    action, without running its gates, until ``release()`` is called with a
    ``HumanAuthorisedTransition``.

    Pass ``state_path`` to make the Terminal State survive a restart: the trip is
    written to that file (atomically, fsynced) and read back when the latch is
    created, so relaunching the node or rebooting the robot does not release it.
    Only ``release()`` deletes the file. A file that exists but cannot be read
    keeps the latch tripped. Without ``state_path`` the latch lives in memory and
    a restart clears it; use that only for tests and simulation.

    Thread-safe; rclpy's MultiThreadedExecutor may run the guarded callbacks
    concurrently.
    """

    def __init__(self, state_path: str | os.PathLike[str] | None = None) -> None:
        self._lock = threading.Lock()
        self._tripped_by: InvariantViolation | None = None
        self._terminal_entry_id: str | None = None
        self._released_by: list[HumanAuthorisedTransition] = []
        self.state_path = Path(state_path) if state_path is not None else None
        if self.state_path is not None and self.state_path.exists():
            self._restore()

    def _restore(self) -> None:
        assert self.state_path is not None
        try:
            data = json.loads(self.state_path.read_text(encoding="utf-8"))
            self._tripped_by = InvariantViolation(
                reason=str(data["reason"]),
                identity_key=str(data["identity_key"]),
                invariant_name=str(data.get("invariant_name", "")),
                terminal_state_name=data.get("terminal_state_name") or "unknown",
                timestamp=float(data.get("timestamp", time.time())),
            )
            self._terminal_entry_id = data.get("terminal_entry_id")
        except Exception as error:  # noqa: BLE001 -- unreadable state stays tripped
            self._tripped_by = InvariantViolation(
                reason=f"latch state at {self.state_path} is unreadable ({type(error).__name__}); held until reviewed",
                identity_key="vsl-ros2",
                invariant_name="",
                terminal_state_name="unknown",
            )

    def _persist(self) -> None:
        if self.state_path is None or self._tripped_by is None:
            return
        v = self._tripped_by
        data = {
            "reason": v.reason,
            "identity_key": v.identity_key,
            "invariant_name": v.invariant_name,
            "terminal_state_name": v.terminal_state_name,
            "timestamp": v.timestamp,
            "terminal_entry_id": self._terminal_entry_id,
        }
        self.state_path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.state_path.with_name(self.state_path.name + ".tmp")
        with open(tmp, "w", encoding="utf-8") as handle:
            json.dump(data, handle)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp, self.state_path)

    @property
    def terminal_entry_id(self) -> str | None:
        """The ledger entry (TERMINAL) recorded for the trip, if the application
        passed one to ``trip()``; a HUMAN_AUTHORISED_TRANSITION names it as
        ``caused_by``. Survives a restart with ``state_path``."""
        with self._lock:
            return self._terminal_entry_id

    @property
    def tripped(self) -> bool:
        with self._lock:
            return self._tripped_by is not None

    @property
    def tripped_by(self) -> InvariantViolation | None:
        with self._lock:
            return self._tripped_by

    @property
    def terminal_state_name(self) -> str | None:
        with self._lock:
            return self._tripped_by.terminal_state_name if self._tripped_by is not None else None

    @property
    def releases(self) -> tuple[HumanAuthorisedTransition, ...]:
        """Every transition that has released this latch, oldest first."""
        with self._lock:
            return tuple(self._released_by)

    def trip(self, violation: InvariantViolation, *, terminal_entry_id: str | None = None) -> None:
        """Enter the Terminal State. A latch already tripped keeps its first cause.
        ``terminal_entry_id`` links the trip to its TERMINAL ledger entry."""
        if not isinstance(violation, InvariantViolation):
            raise TypeError("TerminalLatch.trip() takes the InvariantViolation that entered the Terminal State.")
        with self._lock:
            if self._tripped_by is None:
                self._tripped_by = violation
                self._terminal_entry_id = terminal_entry_id
                self._persist()

    def note_terminal_entry(self, terminal_entry_id: str) -> None:
        """Attach the TERMINAL ledger entry to a trip that happened inside a gate
        (where no ledger entry existed yet)."""
        with self._lock:
            if self._tripped_by is not None and self._terminal_entry_id is None:
                self._terminal_entry_id = terminal_entry_id
                self._persist()

    def release(self, transition: HumanAuthorisedTransition) -> None:
        """Leave the Terminal State. Requires a HumanAuthorisedTransition (a named
        human approver, a GovernanceAuthority and evidence): there is no timeout,
        default or automatic release. Record the transition in your VerbaLedger
        (HUMAN_AUTHORISED_TRANSITION); this package does not write ledgers."""
        if not isinstance(transition, HumanAuthorisedTransition):
            raise TypeError("TerminalLatch.release() requires a vsl_core HumanAuthorisedTransition.")
        with self._lock:
            if self._tripped_by is None:
                raise ValueError("TerminalLatch.release(): the latch is not tripped.")
            if self.state_path is not None and self.state_path.exists():
                self.state_path.unlink()
            self._tripped_by = None
            self._terminal_entry_id = None
            self._released_by.append(transition)


def _as_gate_tuple(gates: Gates) -> tuple[CompiledGate, ...]:
    if callable(gates):
        return (gates,)
    result = tuple(gates)
    if not result:
        raise ValueError("At least one compiled gate is required.")
    for gate in result:
        if not callable(gate):
            raise TypeError(f"Not a compiled gate: {gate!r}")
    return result


class GateRunner:
    """Runs compiled gates on a private asyncio loop, from any thread.

    One runner can serve every wrapper in a process; ``default_runner()`` returns
    a shared one. The loop thread starts on first use and is a daemon thread.
    """

    def __init__(self, *, timeout_sec: float | None = DEFAULT_TIMEOUT_SEC) -> None:
        if timeout_sec is not None and timeout_sec <= 0:
            raise ValueError("timeout_sec must be positive, or None for no timeout.")
        self.timeout_sec = timeout_sec
        self._lock = threading.Lock()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None

    def _ensure_loop(self) -> asyncio.AbstractEventLoop:
        with self._lock:
            if self._loop is None or self._loop.is_closed():
                loop = asyncio.new_event_loop()
                ready = threading.Event()

                def run() -> None:
                    asyncio.set_event_loop(loop)
                    loop.call_soon(ready.set)
                    loop.run_forever()

                thread = threading.Thread(target=run, name="vsl-ros2-gates", daemon=True)
                thread.start()
                ready.wait()
                self._loop, self._thread = loop, thread
            return self._loop

    def close(self) -> None:
        """Stop the loop thread. The runner restarts it if used again."""
        with self._lock:
            loop, thread = self._loop, self._thread
            self._loop = self._thread = None
        if loop is not None:
            loop.call_soon_threadsafe(loop.stop)
            if thread is not None:
                thread.join(timeout=5)
            loop.close()

    def check(
        self,
        gates: Gates,
        candidate_input: Any,
        *,
        target: str | None = None,
        latch: TerminalLatch | None = None,
        timeout_sec: float | None | object = ...,
    ) -> None:
        """Run ``gates`` in order on ``candidate_input``. Returns only if every gate
        allowed; otherwise raises ``VSLGateBlocked``. The first gate that denies
        stops the chain. ``timeout_sec`` overrides the runner's default for this
        call (``None`` waits indefinitely)."""
        gate_tuple = _as_gate_tuple(gates)
        timeout = self.timeout_sec if timeout_sec is ... else timeout_sec

        if latch is not None and latch.tripped:
            cause = latch.tripped_by
            raise VSLGateBlocked(
                f"in terminal state '{latch.terminal_state_name}' until a human-authorised transition",
                target=target,
                denial=cause,
                latched=True,
            )

        loop = self._ensure_loop()
        if threading.current_thread() is self._thread:
            raise RuntimeError("GateRunner.check() called from inside a gate; that would deadlock.")

        for gate in gate_tuple:
            future = None
            try:
                future = asyncio.run_coroutine_threadsafe(gate(candidate_input), loop)
                future.result(timeout=timeout)
            except AutomationDeniedException as denial:
                if latch is not None and isinstance(denial, InvariantViolation) and denial.terminal_state_name:
                    latch.trip(denial)
                raise VSLGateBlocked(denial.reason, target=target, denial=denial) from denial
            except concurrent.futures.TimeoutError as timed_out:
                if future is not None:
                    future.cancel()
                raise VSLGateBlocked(
                    f"gate did not decide within {timeout}s; blocked fail-closed",
                    target=target,
                    error=timed_out,
                ) from timed_out
            except Exception as error:  # noqa: BLE001 -- any other failure blocks, fail-closed
                raise VSLGateBlocked(
                    f"gate failed ({type(error).__name__}: {error}); blocked fail-closed",
                    target=target,
                    error=error,
                ) from error

    def evaluate(
        self,
        gates: Gates,
        raw: Any,
        candidate_input_fn: Callable[[Any], Any] | None,
        *,
        target: str | None = None,
        latch: TerminalLatch | None = None,
        timeout_sec: float | None | object = ...,
    ) -> None:
        """``check()`` after extracting the candidate input from ``raw``; an
        extraction error blocks too."""
        if latch is not None and latch.tripped:
            self.check(gates, None, target=target, latch=latch, timeout_sec=timeout_sec)
        try:
            candidate = candidate_input_fn(raw) if candidate_input_fn is not None else raw
        except Exception as error:  # noqa: BLE001
            raise VSLGateBlocked(
                f"could not extract the gate's input ({type(error).__name__}: {error}); blocked fail-closed",
                target=target,
                error=error,
            ) from error
        self.check(gates, candidate, target=target, latch=latch, timeout_sec=timeout_sec)


_default_runner: GateRunner | None = None
_default_runner_lock = threading.Lock()


def default_runner() -> GateRunner:
    """The process-wide GateRunner the wrappers use when none is passed."""
    global _default_runner
    with _default_runner_lock:
        if _default_runner is None:
            _default_runner = GateRunner()
        return _default_runner
