"""End-to-end through real rclpy nodes, executors and DDS.

Skipped unless rclpy is importable (a sourced ROS 2 installation). To run the
whole suite in a ROS 2 container, from the repository root:

    docker run --rm -v "$PWD:/src:ro" ros:jazzy-ros-base bash /src/scripts/test_in_ros.sh jazzy
"""

from __future__ import annotations

import time

import pytest

rclpy = pytest.importorskip("rclpy")
Twist = pytest.importorskip("geometry_msgs.msg").Twist
SetBool = pytest.importorskip("std_srvs.srv").SetBool
Fibonacci = pytest.importorskip("example_interfaces.action").Fibonacci

from rclpy.action import ActionClient, ActionServer  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402

from vsl_core.constructs import Invariant, PreNode, TerminalState  # noqa: E402
from vsl_core.governance import GovernanceAuthority, HumanAuthorisedTransition  # noqa: E402
from vsl_core.identity import Evidence  # noqa: E402
from vsl_core.ledger import GATE_ALLOWED_KEY, InMemoryLedgerStore, VerbaLedger  # noqa: E402
from vsl_core.metrics import GammaEstimate  # noqa: E402

from vsl_ros2 import (  # noqa: E402
    GatedActionClient,
    GatedClient,
    ROS2Adapter,
    TerminalLatch,
    VSLGateBlocked,
    create_gated_publisher,
    create_gated_service,
    gated_goal_callback,
)

MAX_SPEED = 1.0
E_STOP = TerminalState(name="robot-e-stopped", description="Motion halted pending human review.")


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def executor():
    ex = MultiThreadedExecutor(num_threads=4)
    yield ex
    ex.shutdown()


def spin_until(executor, condition, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if condition():
            return True
    return False


def _gates(ledger: VerbaLedger, identity: str):
    adapter = ROS2Adapter(identity_key=identity)

    async def headroom(msg) -> GammaEstimate:
        speed = abs(float(msg.linear.x))
        estimate = GammaEstimate(gamma_hat=float("inf") if speed == 0 else MAX_SPEED / speed)
        ledger.write_gate_decision(
            identity_key=identity, primitive_name="speed-headroom", estimate=estimate, gamma_threshold=1.1
        )
        return estimate

    async def finite(msg) -> bool:
        x = float(msg.linear.x)
        holds = x == x and abs(x) != float("inf")
        ledger.write_gate_decision(identity_key=identity, primitive_name="finite-command", allowed=holds)
        return holds

    speed = adapter.compile_pre_node(
        PreNode(name="speed-headroom", description="d", monitor=headroom, gamma_threshold=1.1)
    )
    sane = adapter.compile_invariant(
        Invariant(name="finite-command", description="d", rule=finite, on_violation=E_STOP)
    )
    return [sane, speed]


def twist(x: float) -> Twist:
    msg = Twist()
    msg.linear.x = x
    return msg


def test_publisher_to_driver_over_dds(executor):
    ledger = VerbaLedger(store=InMemoryLedgerStore())
    latch = TerminalLatch()
    governor = Node("vsl_governor")
    driver = Node("vsl_driver")
    received: list[float] = []
    driver.create_subscription(Twist, "/vsl_live/cmd_vel", lambda m: received.append(m.linear.x), 10)
    pub = create_gated_publisher(
        governor, Twist, "/vsl_live/cmd_vel", 10, _gates(ledger, "/vsl_governor"), safe_message=twist(0.0), latch=latch
    )
    executor.add_node(governor)
    executor.add_node(driver)
    try:
        assert spin_until(executor, lambda: pub.get_subscription_count() > 0)

        assert pub.publish(twist(0.5)) is True
        assert pub.publish(twist(3.0)) is False  # denied: a stop goes out instead
        assert spin_until(executor, lambda: len(received) >= 2)
        assert received == [0.5, 0.0]

        assert pub.publish(twist(float("nan"))) is False  # Invariant -> Terminal State
        assert latch.tripped
        assert pub.publish(twist(0.2)) is False  # held until a human releases it
        assert spin_until(executor, lambda: len(received) >= 4)
        assert received == [0.5, 0.0, 0.0, 0.0]

        authority = GovernanceAuthority(name="Safety officer", escalation_contact="safety@example.test")
        evidence = Evidence(root_cause_analysis="NaN from planner", specification_update_proposal="reject NaN upstream")
        latch.release(HumanAuthorisedTransition.authorise(authority, authorised_by="A. Operator", evidence=evidence))
        assert pub.publish(twist(0.2)) is True
        assert spin_until(executor, lambda: len(received) >= 5)
        assert received[-1] == pytest.approx(0.2)

        decisions = [e.payload[GATE_ALLOWED_KEY] for e in ledger.store.all_entries()]
        # 0.5: two allows; 3.0: allow+deny; nan: deny (chain stops); latched: none; 0.2: two allows
        assert decisions == [True, True, True, False, False, True, True]
    finally:
        governor.destroy_node()
        driver.destroy_node()


def test_service_and_client(executor):
    server = Node("vsl_gripper")
    caller = Node("vsl_planner")
    released = []

    def release(request, response):
        released.append(request.data)
        response.success = True
        response.message = "released"
        return response

    async def not_over_person(request) -> bool:
        return request.data is False  # in this test, True means "release over a person"

    gate = ROS2Adapter(identity_key="/vsl_gripper").compile_invariant(
        Invariant(name="not-over-person", description="d", rule=not_over_person)
    )
    create_gated_service(server, SetBool, "/vsl_live/release", release, gate)
    raw_client = caller.create_client(SetBool, "/vsl_live/release")
    executor.add_node(server)
    executor.add_node(caller)
    try:
        assert spin_until(executor, lambda: raw_client.service_is_ready())

        # Server-side gate: an ungated client gets success=False back.
        future = raw_client.call_async(SetBool.Request(data=True))
        assert spin_until(executor, future.done)
        assert future.result().success is False and "not-over-person" in future.result().message
        future = raw_client.call_async(SetBool.Request(data=False))
        assert spin_until(executor, future.done)
        assert future.result().success is True
        assert released == [False]

        # Client-side gate: the request never leaves the caller.
        gated = GatedClient(raw_client, gate)
        with pytest.raises(VSLGateBlocked):
            gated.call_async(SetBool.Request(data=True))
        assert released == [False]
    finally:
        server.destroy_node()
        caller.destroy_node()


def test_action_goal_rejected_before_execute(executor):
    server_node = Node("vsl_counter")
    client_node = Node("vsl_counter_client")
    executed = []

    async def small_order(goal) -> GammaEstimate:
        return GammaEstimate(gamma_hat=10.0 / goal.order if goal.order > 0 else 0.0)

    gate = ROS2Adapter(identity_key="/vsl_counter").compile_pre_node(
        PreNode(name="order-headroom", description="d", monitor=small_order, gamma_threshold=1.1)
    )

    def execute(goal_handle):
        executed.append(goal_handle.request.order)
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = [0, 1]
        return result

    ActionServer(server_node, Fibonacci, "/vsl_live/fib", execute_callback=execute, goal_callback=gated_goal_callback(gate))
    client = ActionClient(client_node, Fibonacci, "/vsl_live/fib")
    executor.add_node(server_node)
    executor.add_node(client_node)
    try:
        assert spin_until(executor, lambda: client.server_is_ready())

        f = client.send_goal_async(Fibonacci.Goal(order=50))
        assert spin_until(executor, f.done)
        assert f.result().accepted is False

        f = client.send_goal_async(Fibonacci.Goal(order=5))
        assert spin_until(executor, f.done)
        assert f.result().accepted is True
        assert spin_until(executor, lambda: executed == [5])

        gated = GatedActionClient(client, gate)
        with pytest.raises(VSLGateBlocked):
            gated.send_goal_async(Fibonacci.Goal(order=50))
        time.sleep(0.2)
        assert executed == [5]
    finally:
        server_node.destroy_node()
        client_node.destroy_node()
