"""Wiring compiled VSL gates into the places a ROS 2 node acts.

vsl-core does not intercept anything on its own: a compiled gate only raises
or returns, when called. Each wrapper below is that call site for one ROS 2
side effect, and calls the gate immediately before it, so the gate completes
strictly before the side effect (F1 pre-commitment):

==========================  =====================================  ===================================
Side effect                 Wrapper                                On a block
==========================  =====================================  ===================================
publish on a topic          ``GatedPublisher.publish()``           not published; ``safe_message``
                                                                   published instead if given
act on a received message   ``gated_subscription_callback()``      callback not called
handle a service request    ``gated_service_callback()``           callback not called; response
                                                                   marked failed
accept an action goal       ``gated_goal_callback()``              ``GoalResponse.REJECT``
call a service              ``GatedClient.call()/call_async()``    request not sent; raises
send an action goal         ``GatedActionClient.send_goal*()``     goal not sent; raises
==========================  =====================================  ===================================

Every block -- a denial, a gate that raised or timed out, an input that could
not be extracted, a tripped ``TerminalLatch`` -- is a ``VSLGateBlocked``,
logged through the node's logger when one is given and passed to
``on_blocked(blocked, raw)`` when that is given. Callbacks the executor runs
(subscriptions, services, goal callbacks) never raise on a denial: an
exception escaping an rclpy callback stops the executor, which on a robot is
an uncontrolled stop. The client wrappers, which your own code calls, raise.

rclpy is imported only where a ROS 2 type is needed (``GoalResponse`` and the
``create_*`` helpers), so the wrappers can be unit-tested without ROS 2.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

from .runner import Gates, GateRunner, TerminalLatch, VSLGateBlocked, _as_gate_tuple, default_runner

OnBlocked = Callable[[VSLGateBlocked, Any], None]


class _Guard:
    """The settings every wrapper shares, and the one place a block is handled."""

    def __init__(
        self,
        gates: Gates,
        *,
        candidate_input_fn: Callable[[Any], Any] | None,
        on_blocked: OnBlocked | None,
        latch: TerminalLatch | None,
        runner: GateRunner | None,
        timeout_sec: float | None | object,
        logger: Any,
        target: str | None,
    ) -> None:
        self.gates = _as_gate_tuple(gates)
        self.candidate_input_fn = candidate_input_fn
        self.on_blocked = on_blocked
        self.latch = latch
        self.runner = runner
        self.timeout_sec = timeout_sec
        self.logger = logger
        self.target = target

    def evaluate(self, raw: Any, target: str | None = None) -> None:
        runner = self.runner or default_runner()
        runner.evaluate(
            self.gates,
            raw,
            self.candidate_input_fn,
            target=target or self.target,
            latch=self.latch,
            timeout_sec=self.timeout_sec,
        )

    def report(self, blocked: VSLGateBlocked, raw: Any) -> None:
        if self.logger is not None:
            try:
                self.logger.warning(str(blocked))
            except Exception:  # noqa: BLE001 -- logging must never unblock or crash
                pass
        if self.on_blocked is not None:
            try:
                self.on_blocked(blocked, raw)
            except Exception as error:  # noqa: BLE001
                # The action stays blocked whatever on_blocked does; say so.
                if self.logger is not None:
                    try:
                        self.logger.error(f"vsl-ros2 on_blocked handler raised {type(error).__name__}: {error}")
                    except Exception:  # noqa: BLE001
                        pass


def _guard(gates: Gates, target: str | None, kwargs: dict[str, Any]) -> _Guard:
    return _Guard(
        gates,
        candidate_input_fn=kwargs.get("candidate_input_fn"),
        on_blocked=kwargs.get("on_blocked"),
        latch=kwargs.get("latch"),
        runner=kwargs.get("runner"),
        timeout_sec=kwargs.get("timeout_sec", ...),
        logger=kwargs.get("logger"),
        target=target,
    )


_GUARD_KWARGS = ("candidate_input_fn", "on_blocked", "latch", "runner", "timeout_sec", "logger")


def _check_kwargs(kwargs: dict[str, Any], extra: tuple[str, ...] = ()) -> None:
    unknown = set(kwargs) - set(_GUARD_KWARGS) - set(extra)
    if unknown:
        raise TypeError(f"Unexpected keyword argument(s): {', '.join(sorted(unknown))}")


class GatedPublisher:
    """Wraps an rclpy ``Publisher`` so every ``publish()`` passes the gates first.

    ``publish(msg)`` returns True if the message was published and False if it
    was blocked. On a block the message is not published; if ``safe_message`` is
    given, it is published instead -- a message, or ``callable(msg, blocked)``
    returning one (or None to publish nothing). For a velocity or trajectory
    topic that is how a block becomes a stop rather than "keep doing the last
    command until the driver's watchdog fires". ``safe_message`` is published
    without passing the gates, also while a ``TerminalLatch`` is tripped, so it
    must itself be safe (zero velocity, hold position).

    ``raise_on_block=True`` raises ``VSLGateBlocked`` from ``publish()`` after the
    safe message, for code that publishes from its own control flow rather than
    from an executor callback.

    Other attributes (``topic_name``, ``get_subscription_count()``, ...) are the
    wrapped publisher's.
    """

    def __init__(
        self,
        publisher: Any,
        gates: Gates,
        *,
        safe_message: Any = None,
        raise_on_block: bool = False,
        **kwargs: Any,
    ) -> None:
        _check_kwargs(kwargs)
        self._publisher = publisher
        self._safe_message = safe_message
        self._raise_on_block = raise_on_block
        self._guard = _guard(gates, getattr(publisher, "topic_name", None), kwargs)

    @property
    def publisher(self) -> Any:
        """The wrapped, ungated publisher."""
        return self._publisher

    def publish(self, msg: Any) -> bool:
        try:
            self._guard.evaluate(msg)
        except VSLGateBlocked as blocked:
            self._publish_safe(msg, blocked)
            self._guard.report(blocked, msg)
            if self._raise_on_block:
                raise
            return False
        # Only reached if every gate allowed (F1 pre-commitment).
        self._publisher.publish(msg)
        return True

    def _publish_safe(self, msg: Any, blocked: VSLGateBlocked) -> None:
        if self._safe_message is None:
            return
        safe = self._safe_message(msg, blocked) if callable(self._safe_message) else self._safe_message
        if safe is not None:
            self._publisher.publish(safe)

    def __getattr__(self, name: str) -> Any:
        return getattr(self._publisher, name)


def gated_subscription_callback(
    callback: Callable[[Any], Any],
    gates: Gates,
    *,
    target: str | None = None,
    **kwargs: Any,
) -> Callable[[Any], Any]:
    """Wrap a subscription callback so it runs only for messages the gates allow.

    The usual shape is a governor node between a planner and a driver: subscribe
    to the planner's command topic through this wrapper and republish what is
    allowed (often through a ``GatedPublisher`` with a ``safe_message``). A
    blocked message is dropped: the callback is not called. ``target`` names
    the topic in block reports.
    """
    _check_kwargs(kwargs)
    guard = _guard(gates, target, kwargs)

    def gated(msg: Any) -> Any:
        try:
            guard.evaluate(msg)
        except VSLGateBlocked as blocked:
            guard.report(blocked, msg)
            return None
        return callback(msg)

    gated.__wrapped__ = callback  # type: ignore[attr-defined]
    return gated


def _mark_response_failed(response: Any, blocked: VSLGateBlocked) -> bool:
    if not hasattr(response, "success"):
        return False
    response.success = False
    if hasattr(response, "message"):
        response.message = str(blocked)
    return True


def gated_service_callback(
    callback: Callable[[Any, Any], Any],
    gates: Gates,
    *,
    target: str | None = None,
    on_blocked_response: Callable[[Any, Any, VSLGateBlocked], Any] | None = None,
    **kwargs: Any,
) -> Callable[[Any, Any], Any]:
    """Wrap a service callback ``(request, response) -> response`` so it runs only
    for requests the gates allow. The gates see the request (or
    ``candidate_input_fn(request)``).

    On a block the callback is not called and the client gets a response that
    says so:

    - ``on_blocked_response(request, response, blocked)``, if given, fills and
      returns it;
    - otherwise, if the response type has a ``success`` field (``std_srvs``
      ``Trigger`` and ``SetBool``, most command services), ``success`` is set to
      False and ``message``, when present, to the reason.

    A response type with neither cannot say "refused", and an empty default
    response could read as success to the client, so the wrapper then raises
    ``VSLGateBlocked`` -- which stops the executor. Give such services an
    ``on_blocked_response``.
    """
    _check_kwargs(kwargs)
    guard = _guard(gates, target, kwargs)

    def gated(request: Any, response: Any) -> Any:
        try:
            guard.evaluate(request)
        except VSLGateBlocked as blocked:
            guard.report(blocked, request)
            if on_blocked_response is not None:
                return on_blocked_response(request, response, blocked)
            if _mark_response_failed(response, blocked):
                return response
            raise
        return callback(request, response)

    gated.__wrapped__ = callback  # type: ignore[attr-defined]
    return gated


def _goal_response() -> Any:
    try:
        from rclpy.action import GoalResponse
    except ImportError as error:  # pragma: no cover - exercised only without ROS 2
        raise ImportError("gated_goal_callback needs rclpy (a ROS 2 installation).") from error
    return GoalResponse


def gated_goal_callback(
    gates: Gates,
    *,
    goal_callback: Callable[[Any], Any] | None = None,
    target: str | None = None,
    **kwargs: Any,
) -> Callable[[Any], Any]:
    """A goal callback for ``rclpy.action.ActionServer(goal_callback=...)`` that
    rejects every goal the gates do not allow.

    This is ROS 2's own pre-commitment point for actions: a rejected goal never
    reaches the execute callback. The gates see the goal request (or
    ``candidate_input_fn(goal_request)``). When they allow, ``goal_callback``
    decides if given; otherwise the goal is accepted.

    Acceptance is checked once. A long-running goal should also re-check before
    each step it commits to (each waypoint, each grasp): call
    ``GateRunner.check()`` inside the execute callback and ``goal_handle.abort()``
    on ``VSLGateBlocked``.
    """
    _check_kwargs(kwargs)
    GoalResponse = _goal_response()
    guard = _guard(gates, target, kwargs)

    def gated(goal_request: Any) -> Any:
        try:
            guard.evaluate(goal_request)
        except VSLGateBlocked as blocked:
            guard.report(blocked, goal_request)
            return GoalResponse.REJECT
        if goal_callback is not None:
            return goal_callback(goal_request)
        return GoalResponse.ACCEPT

    return gated


class GatedClient:
    """Wraps an rclpy service ``Client`` so a request is sent only if the gates
    allow it. ``call()`` and ``call_async()`` raise ``VSLGateBlocked`` instead of
    sending. This is the gate to use when the code deciding to act is itself the
    risk -- an LLM task planner calling robot services, say."""

    def __init__(self, client: Any, gates: Gates, **kwargs: Any) -> None:
        _check_kwargs(kwargs)
        self._client = client
        self._guard = _guard(gates, getattr(client, "srv_name", None), kwargs)

    @property
    def client(self) -> Any:
        return self._client

    def _check(self, request: Any) -> None:
        try:
            self._guard.evaluate(request)
        except VSLGateBlocked as blocked:
            self._guard.report(blocked, request)
            raise

    def call(self, request: Any, *args: Any, **kwargs: Any) -> Any:
        self._check(request)
        return self._client.call(request, *args, **kwargs)

    def call_async(self, request: Any) -> Any:
        self._check(request)
        return self._client.call_async(request)

    def __getattr__(self, name: str) -> Any:
        return getattr(self._client, name)


def goal_id_hex(goal_id: Any) -> str:
    """A goal's ``unique_identifier_msgs/UUID`` (or raw bytes) as a hex string."""
    raw = getattr(goal_id, "uuid", goal_id)
    return bytes(bytearray(raw)).hex()


@dataclass
class ActiveGoal:
    """A goal the server accepted and has not finished. ``goal`` is the goal
    message as sent, so the gates can be re-run against it."""

    goal: Any
    goal_handle: Any
    goal_id: str
    accepted_at: float = field(default_factory=time.monotonic)
    cancel_requested: bool = False
    cancel_reason: VSLGateBlocked | None = None


class GatedActionClient:
    """Wraps an ``rclpy.action.ActionClient`` so a goal is sent only if the gates
    allow it; ``send_goal_async()`` and ``send_goal()`` raise ``VSLGateBlocked``
    instead of sending.

    Goals sent with ``send_goal_async()`` that the server accepts are tracked
    until they finish (``active_goals``), so they can be cancelled when the robot
    stops being fit for them -- by hand with ``cancel_active_goals()``, or by a
    ``GoalSupervisor``. Checking a goal once, when it is sent, is not enough for a
    goal that runs for minutes: without this, a goal still running when an
    e-stop latched would carry on as soon as the button was released.
    ``send_goal()`` blocks until the goal finishes and is not tracked.
    """

    def __init__(self, action_client: Any, gates: Gates, **kwargs: Any) -> None:
        _check_kwargs(kwargs)
        self._client = action_client
        self._guard = _guard(gates, getattr(action_client, "_action_name", None), kwargs)
        self._active: dict[str, ActiveGoal] = {}
        self._results: dict[str, Any] = {}  # goal id -> the one result future asked for
        self._active_lock = threading.Lock()

    @property
    def action_client(self) -> Any:
        return self._client

    def result_future(self, handle: Any) -> Any:
        """The goal's result future, asked of the server once per goal. Callers that
        need the result (tracking here, the caller's own wait) share it, rather than
        each sending the server another GetResult request."""
        goal_id = goal_id_hex(handle.goal_id)
        with self._active_lock:
            future = self._results.get(goal_id)
            if future is None:
                future = self._results[goal_id] = handle.get_result_async()
            return future

    @property
    def gates(self) -> tuple:
        return self._guard.gates

    @property
    def latch(self) -> TerminalLatch | None:
        return self._guard.latch

    @property
    def target(self) -> str | None:
        return self._guard.target

    @property
    def active_goals(self) -> list[ActiveGoal]:
        with self._active_lock:
            return list(self._active.values())

    def _check(self, goal: Any) -> None:
        try:
            self._guard.evaluate(goal)
        except VSLGateBlocked as blocked:
            self._guard.report(blocked, goal)
            raise

    def send_goal(self, goal: Any, *args: Any, **kwargs: Any) -> Any:
        self._check(goal)
        return self._client.send_goal(goal, *args, **kwargs)

    def send_goal_async(self, goal: Any, *args: Any, **kwargs: Any) -> Any:
        self._check(goal)
        future = self._client.send_goal_async(goal, *args, **kwargs)
        add_done_callback = getattr(future, "add_done_callback", None)
        if callable(add_done_callback):
            add_done_callback(lambda done: self._track(goal, done))
        return future

    def _track(self, goal: Any, future: Any) -> None:
        try:
            handle = future.result()
        except Exception:  # noqa: BLE001 -- the caller sees the failure on its own future
            return
        if handle is None or not getattr(handle, "accepted", False):
            return
        active = ActiveGoal(goal=goal, goal_handle=handle, goal_id=goal_id_hex(handle.goal_id))
        with self._active_lock:
            self._active[active.goal_id] = active
        self.result_future(handle).add_done_callback(lambda _done: self._finished(active.goal_id))

    def _finished(self, goal_id: str) -> None:
        with self._active_lock:
            self._active.pop(goal_id, None)
            self._results.pop(goal_id, None)

    def cancel_goal(self, active: ActiveGoal, reason: VSLGateBlocked) -> Any:
        """Ask the server to cancel one tracked goal (once). Returns the cancel
        future, or None if a cancel was already requested."""
        with self._active_lock:
            if active.cancel_requested:
                return None
            active.cancel_requested = True
            active.cancel_reason = reason
        return active.goal_handle.cancel_goal_async()

    def cancel_active_goals(self, reason: VSLGateBlocked) -> list[tuple[ActiveGoal, Any]]:
        """Cancel every tracked goal not already being cancelled."""
        cancelled = []
        for active in self.active_goals:
            future = self.cancel_goal(active, reason)
            if future is not None:
                cancelled.append((active, future))
        return cancelled

    def __getattr__(self, name: str) -> Any:
        return getattr(self._client, name)


# -- Node helpers: create the ROS 2 entity and its gate in one call ----------


def create_gated_publisher(node: Any, msg_type: Any, topic: str, qos_profile: Any, gates: Gates, **kwargs: Any) -> GatedPublisher:
    """``node.create_publisher(...)`` wrapped in a ``GatedPublisher`` that logs
    blocks through ``node.get_logger()``."""
    kwargs.setdefault("logger", node.get_logger())
    return GatedPublisher(node.create_publisher(msg_type, topic, qos_profile), gates, **kwargs)


def create_gated_subscription(
    node: Any, msg_type: Any, topic: str, callback: Callable[[Any], Any], qos_profile: Any, gates: Gates, **kwargs: Any
) -> Any:
    """``node.create_subscription(...)`` with a gated callback."""
    kwargs.setdefault("logger", node.get_logger())
    extra = {k: kwargs.pop(k) for k in ("callback_group", "event_callbacks", "raw") if k in kwargs}
    wrapped = gated_subscription_callback(callback, gates, target=topic, **kwargs)
    return node.create_subscription(msg_type, topic, wrapped, qos_profile, **extra)


def create_gated_service(node: Any, srv_type: Any, srv_name: str, callback: Callable[[Any, Any], Any], gates: Gates, **kwargs: Any) -> Any:
    """``node.create_service(...)`` with a gated callback."""
    kwargs.setdefault("logger", node.get_logger())
    extra = {k: kwargs.pop(k) for k in ("qos_profile", "callback_group") if k in kwargs}
    wrapped = gated_service_callback(callback, gates, target=srv_name, **kwargs)
    return node.create_service(srv_type, srv_name, wrapped, **extra)
