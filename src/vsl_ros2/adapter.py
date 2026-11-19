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

import inspect
import warnings
from typing import Any, Callable

from vsl_core.conformance.protocol import CompiledGate
from vsl_core.constructs import Fallback, Invariant, PreNode
from vsl_core.exceptions import AutomationDeniedException, InvariantViolation
from vsl_core.metrics import F2Modification

_DEFAULT_FALLBACK = Fallback()

# Source-text markers for hosted, black-box model APIs. A robot whose planner
# or monitor asks a hosted model (an LLM task planner, a hosted VLM judging a
# camera frame) cannot honestly claim F2Modification.FULL for that check:
# FULL requires direct access to the energy function (logits, weights,
# activations), which a hosted API call structurally cannot expose. The
# honest ceiling is PARTIAL (MEDIUM). This is a heuristic over the
# monitor/rule's own source plus one level of directly-called helpers, the
# same check vsl-langgraph and vsl-maf carry; it can miss deeper indirection
# and cannot confirm a genuine FULL claim either.
_HOSTED_API_MARKERS = (
    "openai",
    "anthropic",
    "google.genai",
    "google.generativeai",
    "cohere",
    "mistralai",
    "litellm",
    "azure.ai.inference",
)


def _reaches_hosted_api(fn: Callable[..., Any], _seen: set | None = None) -> bool:
    if _seen is None:
        _seen = set()
    if fn in _seen:
        return False
    _seen.add(fn)

    if inspect.getmodule(fn) is None:
        return False
    try:
        source = inspect.getsource(fn)
    except (OSError, TypeError):
        return False
    source_lower = source.lower()
    if any(marker in source_lower for marker in _HOSTED_API_MARKERS):
        return True

    code = getattr(fn, "__code__", None)
    globalns = getattr(fn, "__globals__", None)
    if code is None or globalns is None:
        return False
    for referenced_name in code.co_names:
        candidate = globalns.get(referenced_name)
        if callable(candidate) and candidate is not fn:
            if _reaches_hosted_api(candidate, _seen):
                return True
    return False


def _warn_if_full_claim_reaches_hosted_api(name: str, assurance_basis: Any, fn: Callable[..., Any]) -> None:
    if assurance_basis.f2_modification != F2Modification.FULL:
        return
    if not _reaches_hosted_api(fn):
        return
    warnings.warn(
        f"'{name}' claims F2Modification.FULL, but its monitor/rule appears to "
        "call a hosted model API (openai/anthropic/google.genai/cohere/mistralai/"
        "litellm detected in its source). A hosted, black-box API call cannot "
        "directly modify the energy function -- FULL is not achievable this way. "
        "The honest ceiling is F2Modification.PARTIAL (MEDIUM assurance). This is "
        "a heuristic check, not proof -- it can miss indirect calls, and passing "
        "it does not confirm FULL is correct either.",
        stacklevel=3,
    )


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
        _warn_if_full_claim_reaches_hosted_api(f"PreNode '{pre_node.name}'", pre_node.assurance_basis, pre_node.monitor)
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
        _warn_if_full_claim_reaches_hosted_api(f"Invariant '{invariant.name}'", invariant.assurance_basis, invariant.rule)
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
