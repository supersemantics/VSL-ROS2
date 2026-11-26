import warnings

import pytest

from vsl_core.conformance.suite import run_conformance_suite
from vsl_core.constructs import Fallback, Invariant, PreNode
from vsl_core.metrics import GammaEstimate

from vsl_ros2 import ROS2Adapter


def test_conformance_suite_passes():
    failures = run_conformance_suite(ROS2Adapter())
    assert failures == [], "\n".join(failures)


def test_identity_key_required():
    with pytest.raises(ValueError):
        ROS2Adapter(identity_key="")


def test_non_default_fallback_warns():
    async def monitor(_):
        return GammaEstimate(gamma_hat=5.0)

    with pytest.warns(UserWarning, match="do not consume"):
        ROS2Adapter().compile_pre_node(
            PreNode(
                name="p",
                description="d",
                monitor=monitor,
                fallback=Fallback(max_retries=5),
            )
        )


def test_compiling_gates_without_a_basis_warns_about_nothing():
    async def monitor(_):
        return GammaEstimate(gamma_hat=5.0)

    async def rule(_):
        return True

    with warnings.catch_warnings():
        warnings.simplefilter("error")
        ROS2Adapter().compile_pre_node(PreNode(name="p", description="d", monitor=monitor))
        ROS2Adapter().compile_invariant(Invariant(name="i", description="d", rule=rule))
