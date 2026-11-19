"""vsl_guard_release, without ROS 2: it only reads and writes files next to the latch."""

import json

import pytest

from vsl_core.exceptions import InvariantViolation

from vsl_ros2 import TerminalLatch
from vsl_ros2.guard_node import release_request_path
from vsl_ros2.release_cli import main

ARGS = ["--authorised-by", "J. Smith", "--root-cause", "bay inspected", "--spec-update", "None required."]


def _tripped(tmp_path):
    path = tmp_path / "latch.json"
    TerminalLatch(state_path=path).trip(
        InvariantViolation(reason="e-stop", identity_key="/vsl_guard", invariant_name="e_stop", terminal_state_name="robot-e-stopped")
    )
    return path


def test_request_is_written_for_the_guard(tmp_path, capsys):
    path = _tripped(tmp_path)
    assert main(["--latch-path", str(path), *ARGS]) == 0
    request = json.loads(release_request_path(path).read_text())
    assert request["authorised_by"] == "J. Smith" and request["specification_update_proposal"] == "None required."
    assert path.exists()  # only the guard releases; the tool just asks


def test_status(tmp_path, capsys):
    path = _tripped(tmp_path)
    assert main(["--latch-path", str(path), "--status"]) == 0
    assert "IN TERMINAL STATE 'robot-e-stopped'" in capsys.readouterr().out
    assert main(["--latch-path", str(tmp_path / "none.json"), "--status"]) == 0
    assert "not in a terminal state" in capsys.readouterr().out


def test_evidence_is_required(tmp_path):
    path = _tripped(tmp_path)
    with pytest.raises(SystemExit):
        main(["--latch-path", str(path), "--authorised-by", "J. Smith", "--root-cause", "x", "--spec-update", "  "])
    assert not release_request_path(path).exists()


def test_nothing_to_release(tmp_path):
    assert main(["--latch-path", str(tmp_path / "latch.json"), *ARGS]) == 1
