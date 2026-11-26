"""Shared helpers: gates built from real vsl-core constructs, and stand-ins for
the rclpy objects the wrappers touch (so these tests run without ROS 2)."""

from __future__ import annotations

import sys
import types
from enum import Enum

import pytest

from vsl_core.constructs import Invariant, PreNode, TerminalState
from vsl_core.metrics import GammaEstimate

from vsl_ros2 import ROS2Adapter

MAX_SPEED = 1.0

E_STOP = TerminalState(name="robot-e-stopped", description="Motion halted pending human review.")


class Twist:
    """Just enough of geometry_msgs/Twist."""

    def __init__(self, linear_x: float = 0.0) -> None:
        self.linear_x = linear_x

    def __repr__(self) -> str:
        return f"Twist({self.linear_x})"


class FakePublisher:
    topic_name = "/cmd_vel"

    def __init__(self) -> None:
        self.published: list = []

    def publish(self, msg) -> None:
        self.published.append(msg)

    def get_subscription_count(self) -> int:
        return 1


class FakeLogger:
    def __init__(self) -> None:
        self.warnings: list[str] = []
        self.errors: list[str] = []

    def warning(self, text: str) -> None:
        self.warnings.append(text)

    def error(self, text: str) -> None:
        self.errors.append(text)


def adapter() -> ROS2Adapter:
    return ROS2Adapter(identity_key="test-robot")


def speed_headroom_gate():
    """PreNode: Gamma = MAX_SPEED / |speed|; a stop (0) is always allowed."""

    async def headroom(msg) -> GammaEstimate:
        speed = abs(float(msg.linear_x))
        return GammaEstimate(gamma_hat=float("inf") if speed == 0 else MAX_SPEED / speed)

    return adapter().compile_pre_node(
        PreNode(
            name="speed-headroom",
            description="Commanded speed must stay inside the limit with margin.",
            monitor=headroom,
            gamma_threshold=1.1,
        )
    )


def finite_speed_invariant(on_violation=E_STOP):
    async def finite(msg) -> bool:
        value = float(msg.linear_x)
        return value == value and abs(value) != float("inf")

    return adapter().compile_invariant(
        Invariant(
            name="finite-command",
            description="A NaN or infinite command means the planner is broken.",
            rule=finite,
            on_violation=on_violation,
        )
    )


class _GoalResponse(Enum):
    REJECT = 1
    ACCEPT = 2


@pytest.fixture
def fake_rclpy_action(monkeypatch):
    """Provide rclpy.action.GoalResponse when ROS 2 isn't installed."""
    try:
        from rclpy.action import GoalResponse  # noqa: F401

        return GoalResponse
    except ImportError:
        rclpy = types.ModuleType("rclpy")
        action = types.ModuleType("rclpy.action")
        action.GoalResponse = _GoalResponse
        rclpy.action = action
        monkeypatch.setitem(sys.modules, "rclpy", rclpy)
        monkeypatch.setitem(sys.modules, "rclpy.action", action)
        return _GoalResponse
