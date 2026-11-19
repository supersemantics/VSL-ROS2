"""vsl-ros2: VSL (VERBA Specification Language) governance for ROS 2.

Where ROS 2 already decides (collision monitor, diagnostics, lifecycle,
e-stop locks), vsl-ros2 reads that decision as a vsl-core Invariant instead of
re-implementing it (``conditions``). What it adds is what ROS 2 lacks: those
verdicts checked together before a task is accepted or sent (the gated
wrappers), a decision record of ROS 2's own interventions
(``InterventionWitness``), a Terminal State that survives restarts and needs a
Human-Authorised Transition (``TerminalLatch``), and a check that nothing
bypasses a gated topic (``SolePublisherCondition``).
"""

import vsl_core as _vsl_core

from .adapter import ROS2Adapter
from .conditions import (
    DIAG_ERROR,
    DIAG_OK,
    DIAG_STALE,
    DIAG_WARN,
    NO_MODEL,
    CollisionMonitorCondition,
    Condition,
    ConditionStatus,
    DiagnosticsCondition,
    LifecycleCondition,
    LockCondition,
    SolePublisherCondition,
    TopicCondition,
)
from .integration import (
    GatedActionClient,
    GatedClient,
    GatedPublisher,
    create_gated_publisher,
    create_gated_service,
    create_gated_subscription,
    gated_goal_callback,
    gated_service_callback,
    gated_subscription_callback,
)
from .integration import ActiveGoal, goal_id_hex
from .runner import DEFAULT_TIMEOUT_SEC, GateRunner, TerminalLatch, VSLGateBlocked, default_runner
from .supervisor import GoalSupervisor, cancel_accepted
from .witness import InterventionWitness

__version__ = "0.1.0"

# write_gate_decision / write_handover and the 2026.10 Gamma rule are what this
# package's docs and examples rely on; handovers arrived in vsl-core 0.3.0.
_MIN_VSL_CORE = (0, 3, 0)
_found = tuple(int(part) for part in _vsl_core.__version__.split(".")[:3])
if _found < _MIN_VSL_CORE:
    raise ImportError(
        f"vsl-ros2 {__version__} needs vsl-core >= 0.3.0, found {_vsl_core.__version__}. "
        "Upgrade: pip install --upgrade super-semantics-vsl"
    )

__all__ = [
    "ROS2Adapter",
    # conditions: ROS 2 functions' own verdicts as Invariants
    "Condition",
    "ConditionStatus",
    "DiagnosticsCondition",
    "LifecycleCondition",
    "CollisionMonitorCondition",
    "LockCondition",
    "TopicCondition",
    "SolePublisherCondition",
    "NO_MODEL",
    "DIAG_OK",
    "DIAG_WARN",
    "DIAG_ERROR",
    "DIAG_STALE",
    # recording ROS 2's interventions
    "InterventionWitness",
    # running goals
    "GoalSupervisor",
    "ActiveGoal",
    "cancel_accepted",
    "goal_id_hex",
    "GateRunner",
    "default_runner",
    "DEFAULT_TIMEOUT_SEC",
    "TerminalLatch",
    "VSLGateBlocked",
    "GatedPublisher",
    "GatedClient",
    "GatedActionClient",
    "gated_subscription_callback",
    "gated_service_callback",
    "gated_goal_callback",
    "create_gated_publisher",
    "create_gated_subscription",
    "create_gated_service",
]
