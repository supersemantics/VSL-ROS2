"""InterventionWitness: a decision record of the ROS 2 functions' own interventions.

When Nav2's collision monitor stops the robot, when a component's diagnostics
go to ERROR, when an e-stop engages, ROS 2 acts: the robot stops. What ROS 2
does not keep is a governance record of it: which rule intervened, when, what
the robot was held in, when it cleared, and who authorised leaving a Terminal
State. rosbag records the messages; it does not record decisions.

``InterventionWitness`` watches a set of ``Condition`` objects and writes each
intervention to a ``VerbaLedger`` in the shape vsl-core's ``audit()`` expects:

    intervention   MONITOR (gate decision, allowed=False, primitive_name = condition name)
                   PRE_NODE  (the fallback the ROS function took: the robot is held)
    it clears      VERIFICATION (SUFFICIENT: the ROS function released the hold)

For conditions in ``latch_on`` (an e-stop), the intervention enters a Terminal
State instead:

    intervention   MONITOR, PRE_NODE, VERIFICATION, TERMINAL; the TerminalLatch trips
    human release  HUMAN_AUTHORISED_TRANSITION (caused_by the TERMINAL entry), via release()

The record is the application's choice, as vsl-core requires: nothing is
written unless the application constructs a witness with its own ledger. The
gates compiled from conditions never write.

Listeners run on the node's executor (the conditions are fed from its
callbacks, and expiry is polled on a node timer), so ledger writes never run
inside a gate.
"""

from __future__ import annotations

import threading
import uuid
from collections.abc import Iterable
from typing import Any

from vsl_core.constructs import TerminalState
from vsl_core.exceptions import InvariantViolation
from vsl_core.governance import HumanAuthorisedTransition
from vsl_core.ledger import LedgerEntryType, VerbaLedger, VerificationResult

from .conditions import Condition, ConditionStatus, steady_timer
from .runner import TerminalLatch


class InterventionWitness:
    def __init__(
        self,
        node: Any,
        ledger: VerbaLedger,
        conditions: Iterable[Condition],
        *,
        identity_key: str | None = None,
        latch: TerminalLatch | None = None,
        latch_on: Iterable[Condition] = (),
        terminal_state: TerminalState | None = None,
        period_sec: float = 0.5,
    ) -> None:
        self.conditions = tuple(conditions)
        self.latch_on = frozenset(id(c) for c in latch_on)
        if self.latch_on and (latch is None or terminal_state is None):
            raise ValueError("latch_on needs both a latch and the terminal_state it enters.")
        if not self.latch_on.issubset({id(c) for c in self.conditions}):
            raise ValueError("Every latch_on condition must also be in conditions.")
        self.ledger = ledger
        self.identity_key = identity_key or node.get_fully_qualified_name()
        self.latch = latch
        self.terminal_state = terminal_state
        self._logger = node.get_logger()
        self._lock = threading.Lock()
        self._open: dict[str, tuple[str, Any]] = {}  # condition name -> (decision_id, PRE_NODE entry)
        for condition in self.conditions:
            condition.add_listener(self._on_change)
        self._timer = steady_timer(node, period_sec, self._poll)

    @property
    def open_interventions(self) -> dict[str, str]:
        """Interventions recorded and not yet cleared: condition name -> decision_id."""
        with self._lock:
            return {name: decision_id for name, (decision_id, _) in self._open.items()}

    def _poll(self) -> None:
        for condition in self.conditions:
            condition.poll()

    def _on_change(self, condition: Condition, previous: ConditionStatus, current: ConditionStatus) -> None:
        try:
            # A first verdict that is already failing is an intervention only for a
            # latching condition (an e-stop engaged at boot). For the rest it is the
            # stack starting up -- Nav2 servers report "configuring" -- not an event.
            first_and_failing = not previous.heard and id(condition) in self.latch_on
            if not current.holds and (previous.holds or first_and_failing):
                self._record_intervention(condition, current)
            elif current.holds and not previous.holds:
                self._record_clear(condition, current)
        except Exception as error:  # noqa: BLE001 -- never let recording crash the executor
            self._logger.error(f"vsl-ros2 witness could not record {condition.name}: {type(error).__name__}: {error}")

    def _record_intervention(self, condition: Condition, status: ConditionStatus) -> None:
        with self._lock:
            if condition.name in self._open:
                return  # already open (e.g. it went stale while held)
            decision_id = uuid.uuid4().hex
            payload = {"source": "ros2", "detail": status.detail, **({"values": status.values} if status.values else {})}
            monitor = self.ledger.write_gate_decision(
                identity_key=self.identity_key,
                primitive_name=condition.name,
                allowed=False,
                decision_id=decision_id,
                extra_payload=payload,
            )
            held = self.ledger.write(
                LedgerEntryType.PRE_NODE,
                identity_key=self.identity_key,
                decision_id=decision_id,
                caused_by=monitor.entry_id,
                payload={"fallback": f"held by the ROS function behind {condition.name}", "detail": status.detail},
            )
            self._logger.warning(f"VSL witness: {condition.name} intervened: {status.detail}")
            if id(condition) not in self.latch_on:
                self._open[condition.name] = (decision_id, held)
                return

            assert self.latch is not None and self.terminal_state is not None
            already = self.latch.tripped
            verified = self.ledger.write_verification(
                identity_key=self.identity_key,
                result=VerificationResult.SUFFICIENT,
                decision_id=decision_id,
                caused_by=held.entry_id,
                extra_payload={
                    "verified": "robot held by the terminal latch"
                    + (" (already in a terminal state)" if already else "")
                },
            )
            if already:
                return  # one Terminal State at a time; the first cause stands
            terminal = self.ledger.write(
                LedgerEntryType.TERMINAL,
                identity_key=self.identity_key,
                decision_id=decision_id,
                caused_by=verified.entry_id,
                payload={"terminal_state": self.terminal_state.name, "reason": status.detail, "condition": condition.name},
            )
            self.latch.trip(
                InvariantViolation(
                    reason=f"{condition.name}: {status.detail}",
                    identity_key=self.identity_key,
                    invariant_name=condition.name,
                    terminal_state_name=self.terminal_state.name,
                ),
                terminal_entry_id=terminal.entry_id,
            )
            self._logger.error(f"VSL: entered terminal state '{self.terminal_state.name}' ({condition.name}); needs a human-authorised transition")

    def _record_clear(self, condition: Condition, status: ConditionStatus) -> None:
        with self._lock:
            opened = self._open.pop(condition.name, None)
            if opened is None:
                return  # a latched condition, or one that was never recorded
            decision_id, held = opened
            self.ledger.write_verification(
                identity_key=self.identity_key,
                result=VerificationResult.SUFFICIENT,
                decision_id=decision_id,
                caused_by=held.entry_id,
                extra_payload={"verified": f"the ROS function released the hold: {status.detail}"},
            )
            self._logger.info(f"VSL witness: {condition.name} cleared: {status.detail}")

    def release(self, transition: HumanAuthorisedTransition) -> None:
        """Leave the Terminal State: record the HUMAN_AUTHORISED_TRANSITION (caused_by
        the TERMINAL entry, which survives a restart in a persistent latch), then
        release the latch. If the record cannot be written the latch stays tripped."""
        if self.latch is None:
            raise ValueError("This witness has no latch to release.")
        if not isinstance(transition, HumanAuthorisedTransition):
            raise TypeError("release() requires a vsl_core HumanAuthorisedTransition.")
        if not self.latch.tripped:
            raise ValueError("The latch is not tripped.")
        self.ledger.write(
            LedgerEntryType.HUMAN_AUTHORISED_TRANSITION,
            identity_key=self.identity_key,
            caused_by=self.latch.terminal_entry_id,
            payload={
                "authorised_by": transition.authorised_by,
                "authority": transition.authority.name,
                "authorisation_evidence_hash": transition.authorisation_evidence_hash,
                "terminal_state": self.latch.terminal_state_name,
            },
        )
        self.latch.release(transition)
