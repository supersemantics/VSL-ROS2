"""ROS2Adapter: a VSLAdapter conformant with vsl-core's conformance contract,
for robots built on ROS 2 (rclpy).

In ROS 2 the side effects a governance gate must precede are the ones that
move hardware or change the world: a message published on a command topic
(``cmd_vel``, a joint trajectory), a service request handled, an action goal
accepted. The wrappers in integration.py call a compiled gate immediately
before each of those, so the gate completes strictly before the side effect
(F1 pre-commitment).

This module is framework-agnostic -- it never imports rclpy. The compiled
gate it returns is the same async callable vsl-core's own
PlainPythonReferenceAdapter produces, so it can also be called inline from
any code path that is about to act.
"""

from __future__ import annotations

import warnings
from typing import Any, Callable

from vsl_core.conformance.protocol import CompiledGate
from vsl_core.constructs import Fallback, Invariant, PreNode
from vsl_core.exceptions import AutomationDeniedException, InvariantViolation

_DEFAULT_FALLBACK = Fallback()

class ROS2Adapter:
    """VSLAdapter for ROS 2.

    Compiles PreNode/Invariant into callable gates for the wrappers in
    integration.py (GatedPublisher, gated_subscription_callback,
    gated_service_callback, gated_goal_callback, GatedClient), or for direct
    calls through GateRunner before any guarded action.

    Never imports rclpy.
    """

    def __init__(self, *, identity_key: str = "vsl-ros2") -> None:
        """`identity_key` is recorded on every denial this adapter's gates
        raise; pass the robot's or node's own identity (for example the
        fully-qualified node name) so a denial says what it stopped."""
        if not identity_key:
            raise ValueError("identity_key must be a non-empty string.")
        self.identity_key = identity_key

    def compile_pre_node(self, pre_node: PreNode) -> CompiledGate:
        if pre_node.fallback != _DEFAULT_FALLBACK:
            warnings.warn(
                f"PreNode '{pre_node.name}' sets a non-default Fallback "
                f"({pre_node.fallback!r}), but the vsl-ros2 wrappers do not consume "
                "Fallback -- a denial blocks once, immediately, with no retry loop. "
                "Use safe_message / on_blocked for the robot's own recovery behaviour.",
                stacklevel=2,
            )
        identity_key = self.identity_key

        async def gate(candidate_input: Any) -> None:
            estimate = await pre_node.monitor(candidate_input)
            if not estimate.sufficient(pre_node.gamma_threshold):
                raise AutomationDeniedException(
                    reason=f"Pre-node '{pre_node.name}' insufficient Gamma",
                    identity_key=identity_key,
                )

        return gate

    def compile_invariant(self, invariant: Invariant) -> CompiledGate:
        identity_key = self.identity_key

        async def gate(candidate_input: Any) -> None:
            holds = await invariant.rule(candidate_input)
            if not holds:
                reason = f"Invariant '{invariant.name}' violated"
                terminal_state_name = invariant.on_violation.name if invariant.on_violation is not None else None
                if terminal_state_name is not None:
                    reason += f" -- entering terminal state '{terminal_state_name}'"
                raise InvariantViolation(
                    reason=reason,
                    identity_key=identity_key,
                    invariant_name=invariant.name,
                    terminal_state_name=terminal_state_name,
                )

        return gate
