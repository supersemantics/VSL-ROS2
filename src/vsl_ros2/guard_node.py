"""vsl_guard: a configurable VSL guard in front of any ROS 2 action server.

    planner / LLM ──<served_action>──▶ vsl_guard ──<upstream_action>──▶ Nav2, MoveIt, ...

The guard serves the same action type as the server it protects, so a planner
calls it exactly as it would call the server. It accepts a goal only while
every configured condition holds, forwards it upstream (relaying feedback and
the result), cancels it upstream when the robot stops being fit for it, and
records every decision, cancel and ROS 2 intervention in a VerbaLedger. An
e-stop-like condition enters a Terminal State that survives restarts and is
left only by a human release (``vsl_guard_release``).

Everything is ROS 2 parameters, loaded from a YAML file with ``--params-file``
(see config/nav2_guard.yaml)::

    vsl_guard:
      ros__parameters:
        served_action: /vsl_guard/navigate_to_pose
        upstream_action: /navigate_to_pose
        action_type: nav2_msgs/action/NavigateToPose
        ledger_path: /var/lib/vsl/guard_ledger.jsonl
        latch_path: /var/lib/vsl/guard_latch.json
        conditions: [e_stop, cmd_vel_publishers, bt_navigator, collision_monitor]
        e_stop:
          type: lock               # lock | sole_publisher | lifecycle | collision_monitor | diagnostics
          topic: /e_stop
          terminal: true           # enters the Terminal State; running goals are cancelled
        cmd_vel_publishers:
          type: sole_publisher
          topic: /cmd_vel
          allowed: [/collision_monitor]
          cancel_running: true     # must keep holding while a goal runs
        ...

Conditions only read verdicts ROS 2 already produces; nothing here
re-implements a ROS 2 safety function. A rule no ROS 2 function decides
belongs in your own node, built with the Python API.
"""

from __future__ import annotations

import hashlib
import json
import os
import signal
import threading
import time
import uuid
from collections.abc import Callable
from pathlib import Path
from typing import Any

from vsl_core.constructs import TerminalState
from vsl_core.governance import GovernanceAuthority, HumanAuthorisedTransition
from vsl_core.identity import Evidence
from vsl_core.ledger import JsonlLedgerStore, LedgerEntryType, VerbaLedger, VerificationResult

from .adapter import ROS2Adapter
from .conditions import (
    DIAG_ERROR,
    DIAG_OK,
    DIAG_WARN,
    CollisionMonitorCondition,
    Condition,
    DiagnosticsCondition,
    LifecycleCondition,
    LockCondition,
    SolePublisherCondition,
    steady_timer,
)
from .conditions import NO_MODEL
from .integration import ActiveGoal, GatedActionClient, goal_id_hex
from .runner import TerminalLatch, VSLGateBlocked, default_runner
from .supervisor import GoalSupervisor, cancel_accepted
from .witness import InterventionWitness

CONDITION_TYPES = ("lock", "sole_publisher", "lifecycle", "collision_monitor", "diagnostics")
_DIAG_LEVELS = {"OK": DIAG_OK, "WARN": DIAG_WARN, "ERROR": DIAG_ERROR}

# action_msgs/GoalStatus
_STATUS_SUCCEEDED, _STATUS_CANCELED = 4, 5
_STATUS_ACTIVE = (1, 2, 3)  # ACCEPTED, EXECUTING, CANCELING

# The guard's own lifecycle, as ledger events (MONITOR entries, payload "event").
EVENT_STARTED, EVENT_STOPPED, EVENT_UNCLEAN = "guard_started", "guard_stopped", "previous_run_ended_unexpectedly"

# The Invariant the guard holds over the upstream server itself.
SUPERVISION_INVARIANT = "upstream_goals_supervised"
_UNSUPERVISED = {
    "orphaned": "forwarded by an earlier run of this guard, left running when that run ended",
    "foreign": "never passed through this guard: sent to the upstream server directly (a bypass)",
}


class GuardConfigError(ValueError):
    """The guard's parameters do not describe a valid guard."""


def release_request_path(latch_path: str | os.PathLike[str]) -> Path:
    """Where ``vsl_guard_release`` drops a release request for the guard to apply."""
    path = Path(latch_path)
    return path.with_name(path.name + ".release.json")


def _make_node_class():
    """Built lazily so this module imports without rclpy (the release tool and
    the unit tests use its helpers)."""
    from action_msgs.msg import GoalStatusArray
    from action_msgs.srv import CancelGoal
    from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rosidl_runtime_py.utilities import get_action
    from unique_identifier_msgs.msg import UUID
    from vsl_core.constructs import Invariant

    class VslGuard(Node):
        def __init__(self, **node_kwargs: Any) -> None:
            node_kwargs.setdefault("automatically_declare_parameters_from_overrides", True)
            self._destroyed = False
            super().__init__(node_kwargs.pop("node_name", "vsl_guard"), **node_kwargs)
            self.identity = self._param("identity_key", "") or self.get_fully_qualified_name()
            self.action_type_name = self._param("action_type", "nav2_msgs/action/NavigateToPose")
            try:
                action_type = get_action(self.action_type_name)
            except Exception as error:  # noqa: BLE001
                self.destroy_node()
                raise GuardConfigError(f"action_type {self.action_type_name!r} cannot be loaded: {error}") from error
            self.served_action = self._param("served_action", "/vsl_guard/navigate_to_pose")
            self.upstream_action = self._param("upstream_action", "/navigate_to_pose")
            self.upstream_timeout = float(self._param("upstream_accept_timeout_sec", 10.0))
            ledger_path = Path(self._param("ledger_path", "vsl_guard_ledger.jsonl"))
            self.latch_path = self._param("latch_path", "vsl_guard_latch.json")
            for directory in {ledger_path.parent, Path(self.latch_path).parent}:
                directory.mkdir(parents=True, exist_ok=True)  # a fresh robot has no state directory yet
            self.ledger = VerbaLedger(JsonlLedgerStore(ledger_path, fsync=True))
            self.latch = TerminalLatch(state_path=self.latch_path)
            self.release_path = release_request_path(self.latch_path)
            self.authority = GovernanceAuthority(
                name=self._param("release.authority", "Site safety officer"),
                escalation_contact=self._param("release.contact", "unset"),
            )
            self.terminal_state = TerminalState(
                name=self._param("terminal_state", "robot-held"),
                description="A terminal condition fired; no goal is accepted until a human-authorised release.",
            )

            try:
                self.conditions, terminal, keep, record_only = self._build_conditions()
            except Exception:
                self.destroy_node()  # don't leave a half-configured node on the graph
                raise
            adapter = ROS2Adapter(identity_key=self.identity)
            # gate: false conditions are witnessed (recorded) but never block a goal.
            self.gating_conditions = [c for c in self.conditions if c not in record_only]
            compiled = {c.name: adapter.compile_invariant(c.invariant()) for c in self.gating_conditions}
            self.gates = [compiled[c.name] for c in self.gating_conditions]
            self.terminal_conditions = terminal
            if not self.gates:
                self.destroy_node()
                raise GuardConfigError("Every condition has gate: false; the guard would gate nothing.")

            # The witness always holds the latch, even when no condition is terminal now:
            # a latch restored from disk (tripped under an earlier configuration) must
            # still be releasable, or the robot would be held forever with no recorded way out.
            self.witness = InterventionWitness(
                self,
                self.ledger,
                self.conditions,
                latch=self.latch,
                latch_on=terminal,
                terminal_state=self.terminal_state,
            )
            group = ReentrantCallbackGroup()
            self.upstream = GatedActionClient(
                ActionClient(self, action_type, self.upstream_action, callback_group=group),
                self.gates,
                latch=self.latch,
                logger=self.get_logger(),
            )
            self._cancelling: dict[str, Any] = {}
            self.supervisor = GoalSupervisor(
                self,
                self.upstream,
                gates=[compiled[c.name] for c in keep],
                latch=self.latch,
                on_cancel=self._record_cancel,
                on_cancel_done=self._record_cancel_done,
            )
            self._action_type = action_type
            self._uuid_type = UUID
            self._GoalResponse = GoalResponse
            steady_timer(self, float(self._param("release_poll_sec", 1.0)), self._check_release_request)

            # -- the Invariant over the upstream server: every goal it runs is ours --
            # Goals forwarded by an earlier run that ended (a crash, a kill) and goals
            # sent to the upstream server directly are both unsupervised: nothing
            # would cancel them on an e-stop. Read from the action's own status topic.
            self.orphaned_goals = str(self._param("orphaned_goals", "cancel"))
            if self.orphaned_goals not in ("cancel", "record"):
                self.destroy_node()
                raise GuardConfigError("orphaned_goals must be 'cancel' or 'record'.")
            self._supervision_lock = threading.Lock()
            self._sent_ids: set[str] = set()
            self._unsupervised_handled: set[str] = set()
            self._upstream_active: list[str] = []
            self._known_ids = {e.decision_id for e in self.ledger.store.all_entries() if e.decision_id}

            async def all_supervised(unsupervised: list[str]) -> bool:
                return not unsupervised

            self._supervision_gate = adapter.compile_invariant(
                Invariant(
                    name=SUPERVISION_INVARIANT,
                    description=f"Every goal {self.upstream_action} is executing was sent through this guard and is supervised by it.",
                    rule=all_supervised,
                    assurance_basis=NO_MODEL,
                )
            )
            status_qos = QoSProfile(  # what rclpy/rclcpp action servers publish status with
                depth=1,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                GoalStatusArray, f"{self.upstream_action}/_action/status", self._on_upstream_status, status_qos,
                callback_group=group,
            )
            self._cancel_client = self.create_client(
                CancelGoal, f"{self.upstream_action}/_action/cancel_goal", callback_group=group
            )
            self._CancelGoal = CancelGoal
            steady_timer(self, 1.0, self._check_upstream_supervision)

            # -- the guard's own lifecycle goes in the ledger, before it serves anything --
            self._stopped = False
            self._startup_check: tuple[Any, Any] | None = None
            self._record_start()

            # -- its state, on ROS 2's own channel for that: /diagnostics ----------------
            from diagnostic_msgs.msg import DiagnosticArray

            self._diagnostics = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
            steady_timer(self, float(self._param("diagnostics_period_sec", 1.0)), self._publish_diagnostics)

            self.server = ActionServer(
                self,
                action_type,
                self.served_action,
                execute_callback=self._execute,
                goal_callback=self._on_goal,
                cancel_callback=lambda _handle: CancelResponse.ACCEPT,
                callback_group=group,
            )

            summary = ", ".join(f"{c.name}" for c in self.conditions)
            self.get_logger().info(
                f"guarding {self.upstream_action} ({self.action_type_name}) as {self.served_action}; conditions: {summary}"
            )
            if self.latch.tripped:
                self.get_logger().error(
                    f"starting in terminal state '{self.latch.terminal_state_name}': {self.latch.tripped_by.reason}. "
                    f"Release with: vsl_guard_release --latch-path {self.latch_path} ..."
                )
                if not terminal:
                    self.get_logger().warning(
                        "the latch was tripped under an earlier configuration; no condition is terminal now, "
                        "so a release only needs a named person and evidence"
                    )

        def destroy_node(self) -> None:
            self._destroyed = True  # lets execute loops still waiting on upstream return
            super().destroy_node()

        # -- configuration ------------------------------------------------------

        def _param(self, name: str, default: Any) -> Any:
            if self.has_parameter(name):
                value = self.get_parameter(name).value
                return default if value is None else value
            return default

        def _build_conditions(self) -> tuple[list[Condition], list[Condition], list[Condition], list[Condition]]:
            names = list(self._param("conditions", []))
            if not names:
                raise GuardConfigError("vsl_guard needs a 'conditions' list (e.g. [e_stop, collision_monitor]).")
            if len(set(names)) != len(names):
                raise GuardConfigError("Condition names in 'conditions' must be unique.")
            conditions, terminal, keep, record_only = [], [], [], []
            for name in names:
                p = lambda key, default=None, n=name: self._param(f"{n}.{key}", default)  # noqa: E731
                kind = p("type")
                if kind not in CONDITION_TYPES:
                    raise GuardConfigError(f"Condition '{name}': type must be one of {', '.join(CONDITION_TYPES)} (got {kind!r}).")
                if kind == "lock":
                    c = LockCondition(self, _required(p, name, "topic"), name=name, max_age_sec=_age(p("max_age_sec")))
                elif kind == "sole_publisher":
                    c = SolePublisherCondition(
                        self, _required(p, name, "topic"), list(_required(p, name, "allowed")),
                        name=name, period_sec=float(p("period_sec", 1.0)),
                    )
                elif kind == "lifecycle":
                    c = LifecycleCondition(self, _required(p, name, "node"), required_state=p("required_state", "active"), name=name)
                elif kind == "collision_monitor":
                    # The monitor publishes only when its action changes, so "no message
                    # yet" means it has not acted. Recorded-only (gate: false), that is the
                    # right baseline -- its first STOP is then an intervention -- and it
                    # cannot let anything through. A gating monitor stays fail-closed
                    # unless the config says otherwise.
                    c = CollisionMonitorCondition(
                        self,
                        topic=p("topic", "/collision_monitor_state"),
                        blocking_actions=list(p("blocking_actions", ["STOP"])),
                        assume_clear_until_first=bool(p("assume_clear_until_first", not bool(p("gate", True)))),
                        name=name,
                    )
                else:  # diagnostics
                    level = str(p("worst_allowed", "WARN")).upper()
                    if level not in _DIAG_LEVELS:
                        raise GuardConfigError(f"Condition '{name}': worst_allowed must be OK, WARN or ERROR.")
                    c = DiagnosticsCondition(
                        self, list(_required(p, name, "components")),
                        topic=p("topic", "/diagnostics_agg"), worst_allowed=_DIAG_LEVELS[level],
                        max_age_sec=_age(p("max_age_sec", 5.0)), name=name,
                    )
                conditions.append(c)
                gate = bool(p("gate", True))
                if not gate:
                    if bool(p("cancel_running", False)):
                        raise GuardConfigError(f"Condition '{name}': cancel_running needs gate: true.")
                    record_only.append(c)
                if bool(p("terminal", False)):
                    terminal.append(c)
                elif bool(p("cancel_running", False)):
                    keep.append(c)
            return conditions, terminal, keep, record_only

        # -- goals ----------------------------------------------------------------

        def _on_goal(self, goal_request: Any) -> Any:
            try:
                if self._stopped:  # nothing will supervise a goal accepted now
                    raise VSLGateBlocked("the guard is stopping", target=self.served_action)
                default_runner().check(self.gates, goal_request, target=self.served_action, latch=self.latch)
            except VSLGateBlocked as blocked:
                self.get_logger().warning(f"goal rejected: {blocked.reason}")
                try:
                    self._record_decision(uuid.uuid4().hex, blocked, fallback="goal rejected")
                except Exception as error:  # noqa: BLE001 -- still rejected; say the record is missing
                    self.get_logger().error(f"could not record the rejection ({type(error).__name__}: {error})")
                return self._GoalResponse.REJECT
            return self._GoalResponse.ACCEPT

        def _wait(self, future: Any, timeout: float | None) -> bool:
            event = threading.Event()
            future.add_done_callback(lambda _f: event.set())
            return event.wait(timeout)

        def _execute(self, goal_handle: Any) -> Any:
            try:
                return self._forward(goal_handle)
            except Exception as error:  # noqa: BLE001 -- fail closed: the planner's goal fails
                self.get_logger().error(f"forwarding the goal failed ({type(error).__name__}: {error}); aborting it")
                goal_handle.abort()
                return self._action_type.Result()

        def _forward(self, goal_handle: Any) -> Any:
            uuid_bytes = [int(b) for b in goal_handle.goal_id.uuid]  # numpy uint8 on some distros
            decision_id = bytes(uuid_bytes).hex()
            Result = self._action_type.Result
            # Checked and registered under the lock shutdown_cleanly sets _stopped with:
            # a goal either is registered before the stop (and is cancelled by it) or is
            # never sent at all.
            with self._supervision_lock:
                stopping = self._stopped
                if not stopping:
                    self._sent_ids.add(decision_id)  # ours: upstream_goals_supervised holds for it
            if stopping:
                self._record_decision(decision_id, VSLGateBlocked("the guard is stopping", target=self.upstream_action),
                                      fallback="goal not sent upstream")
                goal_handle.abort()
                return Result()
            # Record first: if the decision can't be recorded, the goal is not sent
            # (the exception aborts the planner's goal in _execute). Fail closed.
            self._record_decision(decision_id, None)
            try:
                sent = self.upstream.send_goal_async(
                    goal_handle.request,
                    feedback_callback=lambda fb: goal_handle.publish_feedback(fb.feedback),
                    goal_uuid=self._uuid_type(uuid=uuid_bytes),  # the same id upstream: one decision
                )
            except VSLGateBlocked as blocked:  # the robot changed between accepting and sending
                self._record_decision(decision_id, blocked, fallback="goal not sent upstream")
                goal_handle.abort()
                return Result()
            if not self._wait(sent, self.upstream_timeout):
                # The planner is told the goal failed, so the upstream server must not run it
                # either: if it accepts it after all, cancel it then.
                self.get_logger().error(f"{self.upstream_action} did not accept the goal within {self.upstream_timeout}s")
                sent.add_done_callback(lambda f: self._cancel_late_acceptance(decision_id, f))
                goal_handle.abort()
                return Result()
            if sent.result() is None or not sent.result().accepted:
                self.get_logger().error(f"{self.upstream_action} rejected the goal")
                goal_handle.abort()
                return Result()
            upstream_handle = sent.result()
            result_future = upstream_handle.get_result_async()
            cancel_sent = False
            while not result_future.done():
                if self._destroyed:  # the node is going away; its next start adopts nothing, cancels the orphan
                    return Result()
                if self._stopped:
                    # A clean stop has cancelled the upstream goal; nothing is spinning to
                    # deliver its result now, so end the planner's goal here: it failed.
                    goal_handle.abort()
                    return Result()
                if goal_handle.is_cancel_requested and not cancel_sent:
                    upstream_handle.cancel_goal_async()  # the planner asked; Nav2 decides
                    cancel_sent = True
                self._wait(result_future, 0.05)
            outcome = result_future.result()
            status = getattr(outcome, "status", None)
            if status == _STATUS_SUCCEEDED:
                goal_handle.succeed()
            elif status == _STATUS_CANCELED and goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:  # aborted upstream, or cancelled by the guard: to the planner the goal failed
                goal_handle.abort()
            return outcome.result if outcome is not None else Result()

        def _cancel_late_acceptance(self, decision_id: str, future: Any) -> None:
            """The upstream server accepted a goal after the guard had given up on it."""
            try:
                handle = future.result()
            except Exception:  # noqa: BLE001 -- never accepted: nothing to cancel
                return
            if handle is None or not getattr(handle, "accepted", False):
                return
            blocked = VSLGateBlocked(
                f"accepted after the guard stopped waiting ({self.upstream_timeout}s); the planner was told it failed",
                target=self.upstream_action,
            )
            target = ActiveGoal(goal=None, goal_handle=handle, goal_id=decision_id)
            for active in self.upstream.active_goals:
                if active.goal_id == decision_id:
                    active.cancel_requested = True  # the supervisor must not ask again
            self.get_logger().warning(f"cancelling goal {decision_id[:8]}: {blocked.reason}")
            try:
                self._record_cancel(target, blocked)
            except Exception as error:  # noqa: BLE001 -- cancel regardless
                self.get_logger().error(f"could not record the cancel ({type(error).__name__}: {error})")

            def done(f: Any) -> None:
                try:
                    ok = cancel_accepted(f.result())
                except Exception:  # noqa: BLE001
                    ok = False
                self._record_cancel_done(target, blocked, ok)

            handle.cancel_goal_async().add_done_callback(done)

        # -- the application's record (vsl-ros2's gates never write) --------------

        def _record_decision(self, decision_id: str, blocked: VSLGateBlocked | None, fallback: str = "") -> None:
            subject = f"{self.action_type_name} goal"
            denied = getattr(blocked.denial, "invariant_name", None) if blocked and not blocked.latched else None
            cause = None
            if blocked is None or denied:
                for c in self.gating_conditions:  # the chain runs in this order
                    allowed = c.name != denied
                    entry = self.ledger.write_gate_decision(
                        identity_key=self.identity, primitive_name=c.name, allowed=allowed, decision_id=decision_id,
                        extra_payload={"subject": subject, "detail": c.status().detail},
                    )
                    if not allowed:
                        cause = entry
                        break
            if blocked is None:
                return
            if cause is None:
                cause = self.ledger.write_monitor(
                    identity_key=self.identity, drift_detected=True, decision_id=decision_id,
                    extra_payload={"subject": subject, "reason": blocked.reason, "latched": blocked.latched},
                )
            held = self.ledger.write(
                LedgerEntryType.PRE_NODE, identity_key=self.identity, decision_id=decision_id,
                caused_by=cause.entry_id, payload={"fallback": fallback, "reason": blocked.reason},
            )
            self.ledger.write_verification(
                identity_key=self.identity, result=VerificationResult.SUFFICIENT, decision_id=decision_id,
                caused_by=held.entry_id, extra_payload={"verified": f"{fallback}; nothing sent to {self.upstream_action}"},
            )

        def _record_cancel(self, active: ActiveGoal, blocked: VSLGateBlocked) -> None:
            denied = getattr(blocked.denial, "invariant_name", None) if not blocked.latched else None
            payload = {"subject": f"running {self.action_type_name} goal", "reason": blocked.reason}
            if denied:
                cause = self.ledger.write_gate_decision(
                    identity_key=self.identity, primitive_name=denied, allowed=False,
                    decision_id=active.goal_id, extra_payload=payload,
                )
            else:
                cause = self.ledger.write_monitor(
                    identity_key=self.identity, drift_detected=True, decision_id=active.goal_id,
                    extra_payload={**payload, "latched": blocked.latched},
                )
            self._cancelling[active.goal_id] = self.ledger.write(
                LedgerEntryType.PRE_NODE, identity_key=self.identity, decision_id=active.goal_id,
                caused_by=cause.entry_id, payload={"fallback": "cancel the running goal", "reason": blocked.reason},
            )

        def _record_cancel_done(self, active: ActiveGoal, blocked: VSLGateBlocked, accepted: bool) -> None:
            held = self._cancelling.pop(active.goal_id, None)
            self.ledger.write_verification(
                identity_key=self.identity,
                result=VerificationResult.SUFFICIENT if accepted else VerificationResult.INSUFFICIENT,
                decision_id=active.goal_id,
                caused_by=getattr(held, "entry_id", None),
                extra_payload={"verified": "upstream accepted the cancel" if accepted else "upstream refused the cancel; goal still running"},
            )

        # -- state on /diagnostics ---------------------------------------------------------

        def diagnostic_status(self) -> Any:
            """OK: accepting goals. WARN: a gating condition does not hold (not ready,
            or a ROS 2 function says no). ERROR: in a Terminal State."""
            from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

            status = DiagnosticStatus()
            status.name = f"vsl_guard: {self.get_fully_qualified_name()}"
            status.hardware_id = self.upstream_action
            failing = [c.name for c in self.gating_conditions if not c.holds()]
            if self.latch.tripped:
                status.level, status.message = DiagnosticStatus.ERROR, f"terminal state '{self.latch.terminal_state_name}': needs a human release"
            elif failing:
                status.level, status.message = DiagnosticStatus.WARN, f"not accepting goals: {', '.join(failing)}"
            else:
                status.level, status.message = DiagnosticStatus.OK, f"accepting goals for {self.upstream_action}"
            status.values = [KeyValue(key=c.name, value=("holds" if c.holds() else "does not hold") + f": {c.status().detail}")
                             for c in self.conditions]
            status.values.append(KeyValue(key="supervised_goals", value=str(len(self.upstream.active_goals))))
            return status

        def _publish_diagnostics(self) -> None:
            from diagnostic_msgs.msg import DiagnosticArray

            msg = DiagnosticArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.status = [self.diagnostic_status()]
            self._diagnostics.publish(msg)

        # -- upstream_goals_supervised -------------------------------------------------

        def _on_upstream_status(self, msg: Any) -> None:
            active = [goal_id_hex(s.goal_info.goal_id) for s in msg.status_list if s.status in _STATUS_ACTIVE]
            with self._supervision_lock:
                self._upstream_active = active
            self._check_upstream_supervision()

        def _check_upstream_supervision(self) -> None:
            with self._supervision_lock:
                unsupervised = [
                    g for g in self._upstream_active if g not in self._sent_ids and g not in self._unsupervised_handled
                ]
            if not unsupervised:
                return
            try:
                default_runner().check(self._supervision_gate, unsupervised, target=self.upstream_action)
            except VSLGateBlocked as blocked:
                for goal_id in unsupervised:
                    self._handle_unsupervised(goal_id, blocked)

        def _handle_unsupervised(self, goal_id: str, blocked: VSLGateBlocked) -> None:
            cancel = self.orphaned_goals == "cancel"
            if cancel and not self._cancel_client.service_is_ready():
                return  # try again on the next status or tick, before recording anything
            with self._supervision_lock:
                if goal_id in self._unsupervised_handled:
                    return
                self._unsupervised_handled.add(goal_id)
            kind = "orphaned" if goal_id in self._known_ids else "foreign"
            subject = f"{self.upstream_action} goal"
            cause = self.ledger.write_gate_decision(
                identity_key=self.identity, primitive_name=SUPERVISION_INVARIANT, allowed=False, decision_id=goal_id,
                extra_payload={"subject": subject, "unsupervised": kind, "detail": _UNSUPERVISED[kind]},
            )
            if not cancel:
                held = self.ledger.write(
                    LedgerEntryType.PRE_NODE, identity_key=self.identity, decision_id=goal_id, caused_by=cause.entry_id,
                    payload={"fallback": "recorded only (orphaned_goals: record)", "unsupervised": kind},
                )
                self.ledger.write_verification(
                    identity_key=self.identity, result=VerificationResult.INSUFFICIENT, decision_id=goal_id,
                    caused_by=held.entry_id, extra_payload={"verified": "the goal is still running, unsupervised"},
                )
                self.get_logger().error(f"unsupervised {kind} goal {goal_id[:8]} on {self.upstream_action}: left running (orphaned_goals: record)")
                return
            held = self.ledger.write(
                LedgerEntryType.PRE_NODE, identity_key=self.identity, decision_id=goal_id, caused_by=cause.entry_id,
                payload={"fallback": "cancel the unsupervised goal", "unsupervised": kind},
            )
            request = self._CancelGoal.Request()
            request.goal_info.goal_id.uuid = list(bytes.fromhex(goal_id))
            self.get_logger().warning(f"cancelling {kind} goal {goal_id[:8]} on {self.upstream_action}: {_UNSUPERVISED[kind]}")

            def done(future: Any) -> None:
                try:
                    accepted = cancel_accepted(future.result())
                except Exception:  # noqa: BLE001
                    accepted = False
                self.ledger.write_verification(
                    identity_key=self.identity,
                    result=VerificationResult.SUFFICIENT if accepted else VerificationResult.INSUFFICIENT,
                    decision_id=goal_id, caused_by=held.entry_id,
                    extra_payload={"verified": "upstream accepted the cancel" if accepted else "upstream refused the cancel; goal still running"},
                )
                if not accepted:
                    self.get_logger().error(f"{self.upstream_action} refused to cancel unsupervised goal {goal_id[:8]}; it is still running")

            self._cancel_client.call_async(request).add_done_callback(done)

        # -- the guard's own lifecycle -----------------------------------------------------

        def _specification(self) -> dict[str, Any]:
            """The guard's configuration: its policy-as-code, as the ledger sees it."""
            if hasattr(self, "list_parameters"):  # Iron and later
                all_names = self.list_parameters([], 0).names
            else:  # Humble's rclpy has no list_parameters; its parameter table is the source
                all_names = list(self._parameters.keys())
            names = sorted(n for n in all_names if n not in ("use_sim_time",))
            return {p.name: p.value for p in self.get_parameters(names)}

        def _lifecycle_events(self) -> list[Any]:
            return [
                e for e in self.ledger.store.all_entries()
                if e.identity_key == self.identity and e.payload.get("event") in (EVENT_STARTED, EVENT_STOPPED)
            ]

        def _record_start(self) -> None:
            spec = self._specification()
            spec_hash = hashlib.sha256(json.dumps(spec, sort_keys=True, default=str).encode()).hexdigest()
            events = self._lifecycle_events()
            last = events[-1] if events else None
            if last is None:
                previous_run = "first run"
            elif last.payload["event"] == EVENT_STOPPED:
                previous_run = "stopped cleanly"
            else:
                previous_run = "ended without a stop record"
            started = self.ledger.write_monitor(
                identity_key=self.identity, drift_detected=False,
                extra_payload={
                    "event": EVENT_STARTED, "previous_run": previous_run, "latched": self.latch.tripped,
                    "specification_hash": spec_hash, "specification": spec,
                    "gating": [c.name for c in self.gating_conditions],
                    "recorded_only": [c.name for c in self.conditions if c not in self.gating_conditions],
                    "invariants": [SUPERVISION_INVARIANT],
                },
            )
            previous_start = next((e for e in reversed(events) if e.payload["event"] == EVENT_STARTED), None)
            if previous_start is not None and previous_start.payload.get("specification_hash") != spec_hash:
                before = previous_start.payload.get("specification", {})
                changed = sorted(k for k in set(before) | set(spec) if before.get(k) != spec.get(k))
                self.ledger.write(
                    LedgerEntryType.SPECIFICATION_UPDATE, identity_key=self.identity, caused_by=started.entry_id,
                    payload={"from_hash": previous_start.payload.get("specification_hash"), "to_hash": spec_hash, "changed": changed},
                )
                self.get_logger().warning(f"configuration changed since the last run: {', '.join(changed)} (recorded)")
            if last is not None and last.payload["event"] == EVENT_STARTED:
                # The previous run crashed or was killed: whatever it had forwarded may
                # still be running upstream with nothing supervising it.
                unclean = self.ledger.write_monitor(
                    identity_key=self.identity, drift_detected=True, caused_by=started.entry_id,
                    extra_payload={"event": EVENT_UNCLEAN, "previous_start": last.entry_id},
                )
                held = self.ledger.write(
                    LedgerEntryType.PRE_NODE, identity_key=self.identity, caused_by=unclean.entry_id,
                    payload={"fallback": f"check {self.upstream_action} for goals left unsupervised and {self.orphaned_goals} them"},
                )
                self.get_logger().warning("the previous run ended without a stop record; checking for goals it left running")
                timer = steady_timer(self, float(self._param("startup_check_sec", 3.0)), lambda: self._finish_startup_check(held))
                self._startup_check = (timer, held)

        def _finish_startup_check(self, held: Any) -> None:
            if self._startup_check is None:
                return
            timer, _ = self._startup_check
            self._startup_check = None
            timer.cancel()
            with self._supervision_lock:
                handled = len(self._unsupervised_handled)
            self.ledger.write_verification(
                identity_key=self.identity, result=VerificationResult.SUFFICIENT, caused_by=held.entry_id,
                extra_payload={
                    "verified": f"checked {self.upstream_action}: {handled} unsupervised goal(s) found, each recorded under "
                                f"{SUPERVISION_INVARIANT}",
                    "upstream_status_heard": self.count_publishers(f"{self.upstream_action}/_action/status") > 0,
                },
            )

        def shutdown_cleanly(self, spin: Callable[[float], None] | None = None, timeout_sec: float = 3.0) -> None:
            """Stop supervising: cancel the goals this guard forwarded (nothing will
            watch them once it is gone), record that, and write the stop record.
            ``spin(timeout)`` lets the executor deliver the cancel responses."""
            with self._supervision_lock:  # the lock _forward registers goals under
                if self._stopped:
                    return
                self._stopped = True
            if self._startup_check is not None:
                self._finish_startup_check(self._startup_check[1])
            blocked = VSLGateBlocked("the guard is stopping; nothing would supervise this goal", target=self.upstream_action)
            # At a stop the executor is no longer spinning, so goal-response and status
            # callbacks may still be queued: drain them first, or a goal accepted moments
            # ago is missed and left running unsupervised.
            drain_until = time.monotonic() + 0.5
            while spin is not None and time.monotonic() < drain_until:
                spin(0.02)
            # Every goal this run forwarded that is still running: the ones being
            # tracked, and -- from the upstream server's own status -- any accepted so
            # recently that tracking has not caught up. Cancelled by goal id through
            # the action's cancel service, which needs no client-side handle.
            with self._supervision_lock:
                running = [g for g in self._upstream_active if g in self._sent_ids]
            goal_ids = sorted({a.goal_id for a in self.upstream.active_goals} | set(running))
            for active in self.upstream.active_goals:
                active.cancel_requested = True  # the supervisor must not ask again
            pending = []
            for goal_id in goal_ids:
                target = ActiveGoal(goal=None, goal_handle=None, goal_id=goal_id)
                self._record_cancel(target, blocked)
                request = self._CancelGoal.Request()
                request.goal_info.goal_id.uuid = list(bytes.fromhex(goal_id))
                state = {"done": False}

                def done(f: Any, a: ActiveGoal = target, s: dict = state) -> None:
                    try:
                        ok = cancel_accepted(f.result())
                    except Exception:  # noqa: BLE001
                        ok = False
                    self._record_cancel_done(a, blocked, ok)
                    s["done"] = True

                self._cancel_client.call_async(request).add_done_callback(done)
                pending.append(state)
            deadline = time.monotonic() + timeout_sec
            while spin is not None and any(not s["done"] for s in pending) and time.monotonic() < deadline:
                spin(0.05)
            self.ledger.write_monitor(
                identity_key=self.identity, drift_detected=False,
                extra_payload={
                    "event": EVENT_STOPPED, "goals_cancelled": len(pending),
                    "cancels_confirmed": sum(s["done"] for s in pending),
                },
            )
            self.get_logger().info(f"stopped cleanly; cancelled {len(pending)} running goal(s)")

        # -- leaving the Terminal State ---------------------------------------------

        def _check_release_request(self) -> None:
            path = self.release_path
            if not path.exists():
                return
            try:
                request = json.loads(path.read_text(encoding="utf-8"))
                authorised_by = str(request["authorised_by"])
                evidence = Evidence(
                    root_cause_analysis=str(request["root_cause_analysis"]),
                    specification_update_proposal=str(request["specification_update_proposal"]),
                )
                transition = HumanAuthorisedTransition.authorise(self.authority, authorised_by=authorised_by, evidence=evidence)
            except Exception as error:  # noqa: BLE001
                return self._reject_release(f"unreadable release request ({type(error).__name__}: {error})")
            if not self.latch.tripped:
                path.unlink(missing_ok=True)
                self.get_logger().info("release request ignored: not in a terminal state")
                return
            still = [c.name for c in self.terminal_conditions if not c.holds()]
            if still:
                return self._reject_release(f"terminal condition(s) still not holding: {', '.join(still)}")
            try:
                self.witness.release(transition)
            except Exception as error:  # noqa: BLE001 -- the latch stays tripped
                return self._reject_release(f"release could not be recorded ({type(error).__name__}: {error})")
            path.unlink(missing_ok=True)
            self.get_logger().warning(f"terminal state released by {authorised_by} (recorded)")

        def _reject_release(self, reason: str) -> None:
            rejected = self.release_path.with_name(self.release_path.name.replace(".release.json", ".release.rejected.json"))
            try:
                data = json.loads(self.release_path.read_text(encoding="utf-8"))
            except Exception:  # noqa: BLE001
                data = {}
            data.update({"rejected": reason, "rejected_at": time.time()})
            rejected.write_text(json.dumps(data, indent=2), encoding="utf-8")
            self.release_path.unlink(missing_ok=True)
            self.get_logger().error(f"release request rejected: {reason}")

    return VslGuard


def _required(p: Any, name: str, key: str) -> Any:
    value = p(key)
    if value in (None, "", []):
        raise GuardConfigError(f"Condition '{name}' needs '{key}'.")
    return value


def _age(value: Any) -> float | None:
    if value in (None, 0, 0.0, ""):
        return None
    return float(value)


_VslGuard = None


def guard_class():
    """The VslGuard rclpy Node class (needs rclpy)."""
    global _VslGuard
    if _VslGuard is None:
        _VslGuard = _make_node_class()
    return _VslGuard


def main(args: list[str] | None = None) -> None:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.signals import SignalHandlerOptions

    # rclpy's own SIGINT handler shuts the context down at once, so the guard could
    # no longer cancel the goals it supervises. Handle SIGINT/SIGTERM (what
    # `ros2 launch` sends) ourselves and stop cleanly; a kill -9 cannot be handled,
    # and the next start finds and cancels what that run left behind.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    def _stop(_signum: int, _frame: Any) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)
    try:
        node = guard_class()()
    except GuardConfigError as error:
        print(f"vsl_guard: configuration error: {error}")
        rclpy.try_shutdown()
        raise SystemExit(2) from error
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            node.shutdown_cleanly(spin=lambda t: executor.spin_once(timeout_sec=t))
        except Exception as error:  # noqa: BLE001 -- the next start's check covers what this misses
            print(f"vsl_guard: clean stop incomplete: {error}")
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
