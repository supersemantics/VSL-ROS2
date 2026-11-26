"""Conditions: verdicts a ROS 2 function already produces, as vsl-core Invariants.

ROS 2 and its ecosystem already decide many of the things a governance gate
needs to know. Nav2's collision monitor decides whether the robot must stop;
the diagnostic aggregator decides whether a component is healthy; a lifecycle
node knows whether it is active; an e-stop or ``twist_mux`` lock topic says
whether motion is locked out. vsl-ros2 does not recompute any of those. A
``Condition`` subscribes to the ROS function's own output and holds its latest
verdict; ``condition.invariant()`` turns that verdict into a vsl-core
``Invariant`` whose rule simply reads it. The ROS function stays the authority,
and the Invariant is a view onto it.

What VSL adds on top is what ROS does not have: checking those verdicts
together before a task is accepted or sent (one gate chain in front of a goal),
a decision record of the ROS functions' own interventions (``InterventionWitness``),
and a Terminal State that only a Human-Authorised Transition can leave
(``TerminalLatch``).

Every condition fails closed: a condition that has not heard from its ROS
function yet, or whose latest message is older than ``max_age_sec``, does not
hold.

Conditions are updated only from the node's own callbacks (subscriptions,
timers, service responses), which the node's executor runs. A gate reading a
condition never calls into ROS, so it cannot deadlock the executor.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable, Iterable, Sequence
from dataclasses import dataclass, field
from typing import Any

from vsl_core.constructs import Invariant, TerminalState
from vsl_core.metrics import AssuranceBasis, F2Modification

# Deprecated: vsl-core 0.4 takes no assurance basis, and a condition's
# Invariant is built without one. Kept so `from vsl_ros2 import NO_MODEL` still
# imports; passing it to a PreNode or Invariant makes vsl-core warn. Removed in 1.0.
NO_MODEL = AssuranceBasis(f1_pre_commitment=True, f2_modification=F2Modification.NONE)


def steady_timer(node: Any, period_sec: float, callback: Callable[[], None]) -> Any:
    """A node timer on the steady (wall) clock, whatever ``use_sim_time`` says.

    The guard's own checks (graph polling, expiry, goal supervision, release
    requests) must keep running when simulation time is paused or slow; and
    verdict expiry is measured on the monotonic clock, so a timer on sim time
    would make a healthy condition look stale. Found running against Nav2's
    Gazebo simulation with use_sim_time.
    """
    try:
        from rclpy.clock import Clock, ClockType

        return node.create_timer(period_sec, callback, clock=Clock(clock_type=ClockType.STEADY_TIME))
    except (ImportError, TypeError):  # stand-in nodes in tests, or an rclpy without the clock argument
        return node.create_timer(period_sec, callback)


@dataclass(frozen=True)
class ConditionStatus:
    """A condition's verdict at one moment."""

    name: str
    holds: bool
    detail: str
    heard: bool  # False until the ROS function has said anything
    age_sec: float | None  # since the verdict was last refreshed
    values: dict[str, Any] = field(default_factory=dict)


Listener = Callable[["Condition", ConditionStatus, ConditionStatus], None]


class Condition:
    """Base class: one ROS 2 function's latest verdict, thread-safe.

    Subclasses feed it from their ROS callbacks with ``_set(holds, detail, values)``.
    ``max_age_sec`` makes the verdict expire: a source that should publish
    continuously (diagnostics) must keep refreshing it, or the condition stops
    holding. Leave it None for sources that publish only on change (a collision
    monitor's state, a lifecycle transition).
    """

    def __init__(self, name: str, description: str, *, max_age_sec: float | None = None) -> None:
        if not name:
            raise ValueError("A condition needs a name; it becomes the Invariant's name and the ledger's primitive_name.")
        if max_age_sec is not None and max_age_sec <= 0:
            raise ValueError("max_age_sec must be positive, or None.")
        self.name = name
        self.description = description
        self.max_age_sec = max_age_sec
        self._lock = threading.Lock()
        self._holds: bool | None = None
        self._detail = "no message from the ROS function yet"
        self._values: dict[str, Any] = {}
        self._updated: float | None = None
        self._listeners: list[Listener] = []
        self._last_reported = ConditionStatus(name, False, self._detail, False, None)

    # -- reading ------------------------------------------------------------

    def status(self) -> ConditionStatus:
        with self._lock:
            return self._status_locked()

    def _status_locked(self) -> ConditionStatus:
        now = time.monotonic()
        age = None if self._updated is None else now - self._updated
        if self._holds is None:
            return ConditionStatus(self.name, False, self._detail, False, age, dict(self._values))
        if self.max_age_sec is not None and age is not None and age > self.max_age_sec:
            return ConditionStatus(
                self.name, False, f"stale: no update for {age:.1f}s (max {self.max_age_sec}s)", True, age, dict(self._values)
            )
        return ConditionStatus(self.name, self._holds, self._detail, True, age, dict(self._values))

    def holds(self) -> bool:
        return self.status().holds

    # -- vsl-core -------------------------------------------------------------

    def invariant(
        self,
        *,
        on_violation: TerminalState | None = None,
        assurance_basis: AssuranceBasis | None = None,
        name: str | None = None,
        description: str | None = None,
    ) -> Invariant:
        """A vsl-core Invariant whose rule is this condition's current verdict.

        The rule ignores the candidate input: the verdict is about the robot's
        state, which the ROS function owns. Compile it with
        ``ROS2Adapter.compile_invariant`` like any other Invariant. Name an
        ``on_violation`` TerminalState only for verdicts that must hold the robot
        until a human reviews it (an e-stop), not for ones that clear by
        themselves (an obstacle in the stop zone).

        ``assurance_basis`` is deprecated: vsl-core 0.4 ignores it and warns.
        Leave it out.
        """
        condition = self

        async def rule(_candidate_input: Any) -> bool:
            return condition.holds()

        deprecated = {} if assurance_basis is None else {"assurance_basis": assurance_basis}
        return Invariant(
            name=name or self.name,
            description=description or self.description,
            rule=rule,
            on_violation=on_violation,
            **deprecated,
        )

    # -- change notification --------------------------------------------------

    def add_listener(self, listener: Listener) -> None:
        """Call ``listener(condition, previous, current)`` whenever the effective
        verdict changes: the first verdict heard from the ROS function, a change
        between holding and not, and expiry. Expiry is only noticed when
        ``poll()`` runs; ``InterventionWitness`` polls on a node timer."""
        with self._lock:
            self._listeners.append(listener)

    def poll(self) -> None:
        """Re-evaluate (so expiry is noticed) and notify listeners of a change."""
        with self._lock:
            current = self._status_locked()
            previous = self._last_reported
            changed = previous.holds != current.holds or previous.heard != current.heard
            self._last_reported = current
            listeners = list(self._listeners) if changed else []
        for listener in listeners:
            listener(self, previous, current)

    # -- feeding (subclasses) -------------------------------------------------

    def _set(self, holds: bool, detail: str, values: dict[str, Any] | None = None) -> None:
        with self._lock:
            self._holds = bool(holds)
            self._detail = detail
            self._values = dict(values or {})
            self._updated = time.monotonic()
        self.poll()


# -- the ROS 2 functions ------------------------------------------------------


def _level(value: Any) -> int:
    """diagnostic_msgs levels are a ``byte`` field: bytes in rclpy, int elsewhere."""
    if isinstance(value, (bytes, bytearray)):
        return value[0] if value else 0
    return int(value)


DIAG_OK, DIAG_WARN, DIAG_ERROR, DIAG_STALE = 0, 1, 2, 3
_DIAG_NAMES = {DIAG_OK: "OK", DIAG_WARN: "WARN", DIAG_ERROR: "ERROR", DIAG_STALE: "STALE"}


class DiagnosticsCondition(Condition):
    """Holds while every named component's diagnostic level is at or below
    ``worst_allowed`` (WARN by default) and recently reported.

    Reads ``diagnostic_msgs/DiagnosticArray`` from the diagnostic aggregator
    (``/diagnostics_agg``, names like ``/Drive/Motors``) or raw ``/diagnostics``.
    The health judgement is the ROS diagnostics' own: this condition never
    inspects sensor values. A component that has not reported yet, reports
    STALE, or goes quiet for longer than ``max_age_sec`` does not hold.
    """

    def __init__(
        self,
        node: Any,
        components: Sequence[str],
        *,
        topic: str = "/diagnostics_agg",
        worst_allowed: int = DIAG_WARN,
        max_age_sec: float | None = 5.0,
        name: str = "ros2.diagnostics",
        qos_profile: Any = 10,
    ) -> None:
        if not components:
            raise ValueError("DiagnosticsCondition needs at least one component name.")
        super().__init__(
            name,
            f"Diagnostics for {', '.join(components)} at or below {_DIAG_NAMES.get(worst_allowed, worst_allowed)}.",
            max_age_sec=max_age_sec,
        )
        self.components = tuple(components)
        self.worst_allowed = min(int(worst_allowed), DIAG_ERROR)  # STALE never passes
        self._latest: dict[str, tuple[int, str, float]] = {}
        from diagnostic_msgs.msg import DiagnosticArray

        node.create_subscription(DiagnosticArray, topic, self._on_array, qos_profile)

    def _on_array(self, msg: Any) -> None:
        now = time.monotonic()
        for status in msg.status:
            if status.name in self.components:
                self._latest[status.name] = (_level(status.level), status.message, now)
        if not any(c in self._latest for c in self.components):
            return  # nothing about our components in this array
        self._evaluate(now)

    def _evaluate(self, now: float) -> None:
        problems, values = [], {}
        for component in self.components:
            seen = self._latest.get(component)
            if seen is None:
                problems.append(f"{component}: not reported")
                continue
            level, message, at = seen
            values[component] = _DIAG_NAMES.get(level, level)
            if self.max_age_sec is not None and now - at > self.max_age_sec:
                problems.append(f"{component}: no report for {now - at:.1f}s")
            elif level > self.worst_allowed:
                problems.append(f"{component}: {_DIAG_NAMES.get(level, level)} {message}".rstrip())
        if problems:
            self._set(False, "; ".join(problems), values)
        else:
            self._set(True, "all components healthy", values)


class LifecycleCondition(Condition):
    """Holds while a managed (lifecycle) node is in ``required_state``
    (``active`` by default).

    Seeds itself with one ``<node>/get_state`` call and then follows
    ``<node>/transition_event``. Until the state is known the condition does not
    hold. Typical use: Nav2's ``bt_navigator`` or ``controller_server``, or the
    ``collision_monitor`` itself (a collision monitor that is not active is not
    protecting anything).
    """

    def __init__(
        self,
        node: Any,
        target_node: str,
        *,
        required_state: str = "active",
        name: str | None = None,
        query_on_start: bool = True,
        query_timeout_sec: float = 2.0,
    ) -> None:
        target = "/" + target_node.strip("/")
        super().__init__(name or f"ros2.lifecycle:{target}", f"{target} is {required_state}.")
        self.target_node = target
        self.required_state = required_state.lower()
        self.query_timeout_sec = query_timeout_sec
        self._pending_since: float | None = None  # when the outstanding get_state was sent
        from lifecycle_msgs.msg import TransitionEvent

        node.create_subscription(TransitionEvent, f"{target}/transition_event", self._on_event, 10)
        self._client = None
        if query_on_start:
            from lifecycle_msgs.srv import GetState

            self._client = node.create_client(GetState, f"{target}/get_state")
            self._request = GetState.Request()
            self._timer = steady_timer(node, 0.5, self._query)

    def _query(self) -> None:
        """Ask get_state until the state is known. A reply that never comes (a busy or
        restarting node, a lost response) is asked again after query_timeout_sec: a node
        that is already active publishes no further transition_event to learn it from."""
        if self.status().heard:
            self._timer.cancel()
            return
        if self._pending_since is not None and time.monotonic() - self._pending_since < self.query_timeout_sec:
            return  # an answer may still come
        if self._client.service_is_ready():
            self._pending_since = time.monotonic()
            self._client.call_async(self._request).add_done_callback(self._on_state)

    def _on_state(self, future: Any) -> None:
        self._pending_since = None
        try:
            label = future.result().current_state.label
        except Exception:  # noqa: BLE001 -- unknown stays unknown (fails closed); asked again next tick
            return
        if not self.status().heard:  # a transition event may have arrived meanwhile
            self._apply(label)

    def _on_event(self, msg: Any) -> None:
        self._apply(msg.goal_state.label)

    def _apply(self, label: str) -> None:
        label = (label or "").lower()
        self._set(label == self.required_state, f"{self.target_node} is {label or 'unknown'}", {"state": label})


_COLLISION_ACTIONS = ("DO_NOTHING", "STOP", "SLOWDOWN", "APPROACH", "LIMIT")  # LIMIT: Jazzy and later


class CollisionMonitorCondition(Condition):
    """Holds while Nav2's collision monitor is not stopping the robot.

    Reads ``nav2_msgs/CollisionMonitorState`` from the collision monitor's
    ``state_topic``. The collision monitor already stops (or slows) the robot on
    ``cmd_vel`` itself; this condition does not touch velocities. It makes the
    monitor's verdict visible to the task layer, so a new goal is not accepted
    or sent while the monitor is holding the robot, and so the intervention can
    be recorded (``InterventionWitness``).

    ``blocking_actions`` names the actions that count as "not clear": STOP by
    default; add SLOWDOWN or APPROACH to be stricter. The collision monitor
    publishes its state when its action changes, so this condition has no
    expiry. Until its first message the condition does not hold, unless
    ``assume_clear_until_first=True`` -- only reasonable when a
    ``LifecycleCondition`` on the collision monitor is in the same gate chain.
    """

    def __init__(
        self,
        node: Any,
        *,
        topic: str = "/collision_monitor_state",
        blocking_actions: Iterable[str] = ("STOP",),
        assume_clear_until_first: bool = False,
        name: str = "nav2.collision_monitor",
    ) -> None:
        super().__init__(name, "Nav2 collision monitor is not stopping the robot.")
        from nav2_msgs.msg import CollisionMonitorState

        # rosidl puts message constants on the metaclass, so look them up by name.
        self._names = {
            getattr(CollisionMonitorState, key): key for key in _COLLISION_ACTIONS if hasattr(CollisionMonitorState, key)
        }
        unknown = [a for a in blocking_actions if not hasattr(CollisionMonitorState, a)]
        if unknown:
            raise ValueError(f"Unknown CollisionMonitorState action(s): {', '.join(unknown)}")
        self.blocking = {getattr(CollisionMonitorState, a) for a in blocking_actions}
        node.create_subscription(CollisionMonitorState, topic, self._on_state, 10)
        if assume_clear_until_first:
            self._set(True, "no collision monitor action reported yet (assumed clear)", {"action": "DO_NOTHING"})

    def _on_state(self, msg: Any) -> None:
        action = self._names.get(msg.action_type, str(msg.action_type))
        polygon = f" ({msg.polygon_name})" if msg.polygon_name else ""
        self._set(
            msg.action_type not in self.blocking,
            f"collision monitor action {action}{polygon}",
            {"action": action, "polygon": msg.polygon_name},
        )


class TopicCondition(Condition):
    """Holds while the latest message on a topic satisfies ``predicate(msg)``.

    For any ROS function that publishes its verdict on a topic and has no
    dedicated condition here: a safety PLC bridge, a battery manager, a
    localisation-quality flag. ``predicate`` interprets the function's verdict;
    it should not recompute it.
    """

    def __init__(
        self,
        node: Any,
        msg_type: Any,
        topic: str,
        predicate: Callable[[Any], bool],
        *,
        name: str,
        description: str,
        max_age_sec: float | None = None,
        qos_profile: Any = 10,
    ) -> None:
        super().__init__(name, description, max_age_sec=max_age_sec)
        self._predicate = predicate
        self.topic = topic
        node.create_subscription(msg_type, topic, self._on_msg, qos_profile)

    def _on_msg(self, msg: Any) -> None:
        try:
            ok = bool(self._predicate(msg))
        except Exception as error:  # noqa: BLE001 -- an unreadable verdict does not hold
            self._set(False, f"predicate failed on {self.topic}: {type(error).__name__}: {error}")
            return
        self._set(ok, f"{self.topic}: {'ok' if ok else 'not ok'}")


class LockCondition(TopicCondition):
    """Holds while a ``std_msgs/Bool`` lock topic is False: an e-stop topic, or a
    ``twist_mux``-style lock. ``True`` means locked.

    Until the first message, and after ``max_age_sec`` without one when set,
    the lock is treated as engaged (fail-closed).
    """

    def __init__(self, node: Any, topic: str, *, name: str | None = None, max_age_sec: float | None = None) -> None:
        from std_msgs.msg import Bool

        super().__init__(
            node,
            Bool,
            topic,
            lambda msg: not msg.data,
            name=name or f"ros2.lock:{topic}",
            description=f"{topic} is not engaged.",
            max_age_sec=max_age_sec,
        )

    def _on_msg(self, msg: Any) -> None:
        engaged = bool(msg.data)
        self._set(not engaged, f"{self.topic} {'engaged' if engaged else 'released'}", {"engaged": engaged})


def _fq(name: str, namespace: str) -> str:
    return f"{namespace.rstrip('/')}/{name}" if namespace not in ("", "/") else f"/{name}"


class SolePublisherCondition(Condition):
    """Holds while only the allowed nodes publish on ``topic``.

    A gate only governs what passes through it. ROS 2 lets any node publish on
    any topic, so a gated command topic is only gated if nothing else writes to
    it. This condition checks the ROS graph itself (``get_publishers_info_by_topic``)
    every ``period_sec``. Use it on the topic the driver actually listens to
    (for Nav2, the collision monitor's output ``cmd_vel``) with the nodes that
    are meant to publish there.

    The graph check detects a bypass; it does not prevent one. Prevent it with
    SROS2 access control (only the allowed nodes get publish permission on the
    topic) and keep this condition as the runtime check that the permission set
    is what you think it is.
    """

    def __init__(
        self,
        node: Any,
        topic: str,
        allowed_publishers: Iterable[str],
        *,
        period_sec: float = 1.0,
        name: str | None = None,
    ) -> None:
        allowed = {"/" + n.strip("/") for n in allowed_publishers}
        super().__init__(
            name or f"ros2.sole_publishers:{topic}",
            f"Only {', '.join(sorted(allowed)) or 'no node'} publish on {topic}.",
            max_age_sec=max(3 * period_sec, period_sec + 1.0),
        )
        self.topic = topic
        self.allowed = frozenset(allowed)
        self._node = node
        steady_timer(node, period_sec, self._check)
        self._check()

    def _check(self) -> None:
        try:
            infos = self._node.get_publishers_info_by_topic(self.topic)
        except Exception as error:  # noqa: BLE001
            self._set(False, f"could not read the ROS graph: {type(error).__name__}: {error}")
            return
        publishers = sorted({_fq(i.node_name, i.node_namespace) for i in infos})
        foreign = [p for p in publishers if p not in self.allowed]
        if foreign:
            self._set(False, f"unexpected publisher(s) on {self.topic}: {', '.join(foreign)}", {"publishers": publishers})
        else:
            self._set(True, f"{self.topic} publishers: {', '.join(publishers) or 'none'}", {"publishers": publishers})
