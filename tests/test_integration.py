from types import SimpleNamespace

import pytest

from conftest import (
    FakeLogger,
    FakePublisher,
    Twist,
    finite_speed_invariant,
    speed_headroom_gate,
)
from vsl_core.exceptions import InvariantViolation

from vsl_ros2 import (
    GatedActionClient,
    GatedClient,
    GatedPublisher,
    TerminalLatch,
    VSLGateBlocked,
    create_gated_publisher,
    create_gated_service,
    create_gated_subscription,
    gated_goal_callback,
    gated_service_callback,
    gated_subscription_callback,
)

# ---------------------------------------------------------------- publisher


def test_publisher_allows_then_publishes():
    pub = FakePublisher()
    gated = GatedPublisher(pub, speed_headroom_gate())
    msg = Twist(0.5)
    assert gated.publish(msg) is True
    assert pub.published == [msg]


def test_publisher_blocks_and_publishes_safe_message():
    pub, log, seen = FakePublisher(), FakeLogger(), []
    stop = Twist(0.0)
    gated = GatedPublisher(
        pub,
        speed_headroom_gate(),
        safe_message=stop,
        logger=log,
        on_blocked=lambda blocked, raw: seen.append((blocked, raw)),
    )
    fast = Twist(3.0)
    assert gated.publish(fast) is False
    assert pub.published == [stop]  # the fast command never went out
    assert seen[0][1] is fast and seen[0][0].target == "/cmd_vel"
    assert "speed-headroom" in log.warnings[0]


def test_publisher_without_safe_message_publishes_nothing():
    pub = FakePublisher()
    assert GatedPublisher(pub, speed_headroom_gate()).publish(Twist(3.0)) is False
    assert pub.published == []


def test_publisher_safe_message_callable_and_raise():
    pub = FakePublisher()
    gated = GatedPublisher(
        pub, speed_headroom_gate(), safe_message=lambda msg, blocked: Twist(0.0), raise_on_block=True
    )
    with pytest.raises(VSLGateBlocked):
        gated.publish(Twist(3.0))
    assert [m.linear_x for m in pub.published] == [0.0]


def test_publisher_delegates_attributes():
    gated = GatedPublisher(FakePublisher(), speed_headroom_gate())
    assert gated.topic_name == "/cmd_vel" and gated.get_subscription_count() == 1


def test_on_blocked_raising_keeps_the_block():
    pub, log = FakePublisher(), FakeLogger()

    def bad(_b, _r):
        raise RuntimeError("handler bug")

    gated = GatedPublisher(pub, speed_headroom_gate(), on_blocked=bad, logger=log)
    assert gated.publish(Twist(3.0)) is False
    assert pub.published == [] and "handler bug" in log.errors[0]


def test_latch_blocks_publisher_but_safe_message_still_goes_out():
    pub, latch = FakePublisher(), TerminalLatch()
    stop = Twist(0.0)
    gated = GatedPublisher(
        pub, [finite_speed_invariant(), speed_headroom_gate()], safe_message=stop, latch=latch
    )
    assert gated.publish(Twist(float("nan"))) is False
    assert latch.tripped
    assert gated.publish(Twist(0.2)) is False  # normally allowed, but e-stopped
    assert pub.published == [stop, stop]


def test_unknown_kwarg_rejected():
    with pytest.raises(TypeError, match="safe_msg"):
        GatedPublisher(FakePublisher(), speed_headroom_gate(), safe_msg=Twist())


# ------------------------------------------------------------- subscription


def test_subscription_callback_only_for_allowed():
    received = []
    cb = gated_subscription_callback(received.append, speed_headroom_gate(), target="/planner/cmd_vel")
    cb(Twist(0.3))
    cb(Twist(4.0))
    cb(Twist(float("nan")))  # gate error / denial: dropped, never raises
    assert [m.linear_x for m in received] == [0.3]


def test_subscription_with_candidate_input_fn():
    received = []
    cb = gated_subscription_callback(
        received.append, speed_headroom_gate(), candidate_input_fn=lambda m: Twist(m["speed"])
    )
    cb({"speed": 0.2})
    cb({"oops": 1})  # extraction fails: blocked, no raise
    assert received == [{"speed": 0.2}]


# ----------------------------------------------------------------- services


def _handler(request, response):
    response.success = True
    response.message = f"moved {request.linear_x}"
    return response


def test_service_allowed_runs_callback():
    cb = gated_service_callback(_handler, speed_headroom_gate(), target="/move")
    response = cb(Twist(0.4), SimpleNamespace(success=None, message=""))
    assert response.success is True and response.message == "moved 0.4"


def test_service_blocked_marks_response_failed():
    cb = gated_service_callback(_handler, speed_headroom_gate(), target="/move")
    response = cb(Twist(5.0), SimpleNamespace(success=None, message=""))
    assert response.success is False and "speed-headroom" in response.message


def test_service_blocked_custom_response():
    def refuse(request, response, blocked):
        response.accepted = False
        return response

    cb = gated_service_callback(_handler, speed_headroom_gate(), on_blocked_response=refuse)
    response = cb(Twist(5.0), SimpleNamespace(accepted=None))
    assert response.accepted is False


def test_service_without_success_field_raises():
    cb = gated_service_callback(_handler, speed_headroom_gate())
    with pytest.raises(VSLGateBlocked):
        cb(Twist(5.0), SimpleNamespace(result=0))


# ------------------------------------------------------------------ actions


def test_goal_callback(fake_rclpy_action):
    GoalResponse = fake_rclpy_action
    cb = gated_goal_callback(speed_headroom_gate(), target="/drive")
    assert cb(Twist(0.5)) == GoalResponse.ACCEPT
    assert cb(Twist(5.0)) == GoalResponse.REJECT


def test_goal_callback_delegates_after_allow(fake_rclpy_action):
    GoalResponse = fake_rclpy_action
    asked = []

    def inner(goal):
        asked.append(goal)
        return GoalResponse.REJECT

    cb = gated_goal_callback(speed_headroom_gate(), goal_callback=inner)
    assert cb(Twist(5.0)) == GoalResponse.REJECT and asked == []  # gate rejected first
    assert cb(Twist(0.1)) == GoalResponse.REJECT and len(asked) == 1  # inner decided


# ------------------------------------------------------------------ clients


class FakeClient:
    srv_name = "/arm/move"

    def __init__(self):
        self.sent = []

    def call(self, request, timeout_sec=None):
        self.sent.append(request)
        return "response"

    def call_async(self, request):
        self.sent.append(request)
        return "future"


def test_client_blocks_before_sending():
    client = FakeClient()
    gated = GatedClient(client, finite_speed_invariant())
    assert gated.call(Twist(0.1)) == "response"
    with pytest.raises(VSLGateBlocked) as info:
        gated.call_async(Twist(float("nan")))
    assert isinstance(info.value.denial, InvariantViolation) and info.value.target == "/arm/move"
    assert len(client.sent) == 1


class FakeActionClient:
    _action_name = "/navigate"

    def __init__(self):
        self.sent = []

    def send_goal_async(self, goal, feedback_callback=None):
        self.sent.append(goal)
        return "future"

    def wait_for_server(self, timeout_sec=None):
        return True


def test_action_client_blocks_before_sending():
    client = FakeActionClient()
    gated = GatedActionClient(client, speed_headroom_gate())
    assert gated.wait_for_server() is True
    assert gated.send_goal_async(Twist(0.1)) == "future"
    with pytest.raises(VSLGateBlocked):
        gated.send_goal_async(Twist(9.0))
    assert len(client.sent) == 1


# ------------------------------------------------------------- node helpers


class FakeNode:
    def __init__(self):
        self.logger = FakeLogger()
        self.subscriptions, self.services = [], []

    def get_logger(self):
        return self.logger

    def create_publisher(self, msg_type, topic, qos):
        pub = FakePublisher()
        pub.topic_name = topic
        return pub

    def create_subscription(self, msg_type, topic, callback, qos, **kw):
        self.subscriptions.append((topic, callback, kw))
        return callback

    def create_service(self, srv_type, name, callback, **kw):
        self.services.append((name, callback, kw))
        return callback


def test_node_helpers_use_node_logger():
    node = FakeNode()
    pub = create_gated_publisher(node, Twist, "/cmd_vel_safe", 10, speed_headroom_gate())
    pub.publish(Twist(7.0))
    assert "/cmd_vel_safe" in node.logger.warnings[0]

    got = []
    cb = create_gated_subscription(node, Twist, "/cmd_vel_raw", got.append, 10, speed_headroom_gate(), callback_group="g")
    cb(Twist(0.1))
    cb(Twist(7.0))
    assert len(got) == 1 and node.subscriptions[0][2] == {"callback_group": "g"}

    svc = create_gated_service(node, object, "/move", _handler, speed_headroom_gate())
    assert svc(Twist(7.0), SimpleNamespace(success=None, message="")).success is False
