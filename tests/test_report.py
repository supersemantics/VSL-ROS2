"""vsl_guard_report against a ledger written the way vsl_guard and its witness write it."""

import json

from vsl_core.ledger import JsonlLedgerStore, LedgerEntryType, VerbaLedger, VerificationResult

from vsl_ros2.report import build_report, main

ID = "/vsl_guard"


def _ledger(tmp_path):
    path = tmp_path / "ledger.jsonl"
    led = VerbaLedger(JsonlLedgerStore(path))
    led.write_monitor(identity_key=ID, drift_detected=False,
                      extra_payload={"event": "guard_started", "previous_run": "first run", "specification_hash": "ab" * 32})
    # a forwarded goal
    for name in ("e_stop", "bt_navigator"):
        led.write_gate_decision(identity_key=ID, primitive_name=name, allowed=True, decision_id="g1",
                                extra_payload={"subject": "nav2_msgs/action/NavigateToPose goal"})
    # a refused goal
    led.write_gate_decision(identity_key=ID, primitive_name="e_stop", allowed=True, decision_id="g2",
                            extra_payload={"subject": "nav2_msgs/action/NavigateToPose goal"})
    d = led.write_gate_decision(identity_key=ID, primitive_name="bt_navigator", allowed=False, decision_id="g2",
                                extra_payload={"subject": "nav2_msgs/action/NavigateToPose goal"})
    p = led.write(LedgerEntryType.PRE_NODE, identity_key=ID, decision_id="g2", caused_by=d.entry_id, payload={"fallback": "goal rejected"})
    led.write_verification(identity_key=ID, result=VerificationResult.SUFFICIENT, decision_id="g2", caused_by=p.entry_id)
    # the running goal g1 cancelled
    c = led.write_monitor(identity_key=ID, drift_detected=True, decision_id="g1", extra_payload={"latched": True})
    p = led.write(LedgerEntryType.PRE_NODE, identity_key=ID, decision_id="g1", caused_by=c.entry_id,
                  payload={"fallback": "cancel the running goal", "reason": "in terminal state"})
    led.write_verification(identity_key=ID, result=VerificationResult.SUFFICIENT, decision_id="g1", caused_by=p.entry_id)
    # an e-stop intervention entering the terminal state, then a human release
    m = led.write_gate_decision(identity_key=ID, primitive_name="e_stop", allowed=False, decision_id="w1",
                                extra_payload={"source": "ros2", "detail": "/e_stop engaged"})
    p = led.write(LedgerEntryType.PRE_NODE, identity_key=ID, decision_id="w1", caused_by=m.entry_id, payload={"fallback": "held"})
    v = led.write_verification(identity_key=ID, result=VerificationResult.SUFFICIENT, decision_id="w1", caused_by=p.entry_id,
                               extra_payload={"verified": "robot held by the terminal latch"})
    t = led.write(LedgerEntryType.TERMINAL, identity_key=ID, decision_id="w1", caused_by=v.entry_id,
                  payload={"terminal_state": "robot-e-stopped", "reason": "/e_stop engaged"})
    led.write(LedgerEntryType.HUMAN_AUTHORISED_TRANSITION, identity_key=ID, caused_by=t.entry_id,
              payload={"authorised_by": "J. Smith"})
    # an orphaned goal found after a crash
    u = led.write_gate_decision(identity_key=ID, primitive_name="upstream_goals_supervised", allowed=False, decision_id="g9",
                                extra_payload={"unsupervised": "orphaned"})
    p = led.write(LedgerEntryType.PRE_NODE, identity_key=ID, decision_id="g9", caused_by=u.entry_id,
                  payload={"fallback": "cancel the unsupervised goal"})
    led.write_verification(identity_key=ID, result=VerificationResult.SUFFICIENT, decision_id="g9", caused_by=p.entry_id)
    led.write_monitor(identity_key=ID, drift_detected=False, extra_payload={"event": "guard_stopped", "goals_cancelled": 0})
    return path


def test_report(tmp_path):
    r = build_report(_ledger(tmp_path))
    assert r["integrity_ok"] and r["audit_passed"]
    assert [x["event"] for x in r["lifecycle"]] == ["started", "stopped cleanly"]
    assert r["goals"]["forwarded"] == 1 and r["goals"]["refused"] == {"bt_navigator": 1}
    assert r["goals"]["cancelled_while_running"][0]["upstream_confirmed"]
    assert r["unsupervised_goals"][0]["kind"] == "orphaned" and r["unsupervised_goals"][0]["cancel_confirmed"]
    assert r["interventions"][0]["entered_terminal_state"] == "robot-e-stopped"
    assert r["terminal_states"][0]["released_by"] == "J. Smith"


def test_forwarded_then_cancelled_still_counts_as_forwarded(tmp_path):
    """A goal cancelled mid-run by a condition (the guard's _record_cancel shape)
    was still forwarded; the cancel is its own decision about the running goal."""
    path = _ledger(tmp_path)
    led = VerbaLedger(JsonlLedgerStore(path))
    for name in ("e_stop", "cmd_vel_publishers"):
        led.write_gate_decision(identity_key=ID, primitive_name=name, allowed=True, decision_id="g3",
                                extra_payload={"subject": "nav2_msgs/action/NavigateToPose goal"})
    d = led.write_gate_decision(identity_key=ID, primitive_name="cmd_vel_publishers", allowed=False, decision_id="g3",
                                extra_payload={"subject": "running nav2_msgs/action/NavigateToPose goal"})
    p = led.write(LedgerEntryType.PRE_NODE, identity_key=ID, decision_id="g3", caused_by=d.entry_id,
                  payload={"fallback": "cancel the running goal", "reason": "Invariant 'cmd_vel_publishers' violated"})
    led.write_verification(identity_key=ID, result=VerificationResult.SUFFICIENT, decision_id="g3", caused_by=p.entry_id)
    r = build_report(path)
    assert r["goals"]["forwarded"] == 2 and r["goals"]["refused_total"] == 1
    assert len(r["goals"]["cancelled_while_running"]) == 2


def test_refusal_without_a_condition_names_its_reason(tmp_path):
    path = tmp_path / "ledger.jsonl"
    led = VerbaLedger(JsonlLedgerStore(path))
    m = led.write_monitor(identity_key=ID, drift_detected=True, decision_id="g1",
                          extra_payload={"subject": "nav2_msgs/action/NavigateToPose goal", "reason": "the guard is stopping"})
    p = led.write(LedgerEntryType.PRE_NODE, identity_key=ID, decision_id="g1", caused_by=m.entry_id,
                  payload={"fallback": "goal rejected"})
    led.write_verification(identity_key=ID, result=VerificationResult.SUFFICIENT, decision_id="g1", caused_by=p.entry_id)
    assert build_report(path)["goals"]["refused"] == {"the guard is stopping": 1}


def test_cli(tmp_path, capsys):
    path = _ledger(tmp_path)
    assert main([str(path)]) == 0
    out = capsys.readouterr().out
    assert "hash chain intact: yes" in out and "released by J. Smith" in out
    assert main([str(path), "--json"]) == 0
    assert json.loads(capsys.readouterr().out)["goals"]["refused_total"] == 1
    assert main([str(tmp_path / "missing.jsonl")]) == 1


def test_tampering_is_reported(tmp_path):
    path = _ledger(tmp_path)
    lines = path.read_text().splitlines()
    lines[2] = lines[2].replace('"allowed": true', '"allowed": false')
    path.write_text("\n".join(lines) + "\n")
    assert not build_report(path)["integrity_ok"]
    assert main([str(path)]) == 2
