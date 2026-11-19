import warnings

import pytest

from vsl_core.conformance.suite import run_conformance_suite
from vsl_core.constructs import Fallback, PreNode
from vsl_core.metrics import AssuranceBasis, F2Modification, GammaEstimate

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
                assurance_basis=AssuranceBasis(f1_pre_commitment=True, f2_modification=F2Modification.NONE),
                fallback=Fallback(max_retries=5),
            )
        )


def _hosted_judge(frame):  # pragma: no cover - never called, only its source is read
    import openai  # noqa: F401

    return openai.OpenAI().responses.create(model="x", input=frame)


def test_full_claim_on_hosted_model_warns():
    async def monitor(frame):
        return GammaEstimate(gamma_hat=_hosted_judge(frame))

    full = AssuranceBasis(f1_pre_commitment=True, f2_modification=F2Modification.FULL)
    with pytest.warns(UserWarning, match="hosted model API"):
        ROS2Adapter().compile_pre_node(PreNode(name="vlm", description="d", monitor=monitor, assurance_basis=full))


def test_none_claim_does_not_warn():
    async def monitor(frame):
        return GammaEstimate(gamma_hat=_hosted_judge(frame))

    none = AssuranceBasis(f1_pre_commitment=True, f2_modification=F2Modification.NONE)
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        ROS2Adapter().compile_pre_node(PreNode(name="vlm", description="d", monitor=monitor, assurance_basis=none))
