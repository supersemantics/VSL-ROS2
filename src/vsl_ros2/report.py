"""vsl_guard_report: read a vsl_guard ledger and summarise what happened.

For pilot teams and incident reviews. Reads the JSONL ledger file only (no ROS 2
needed), checks the hash chain and vsl-core's audit, and summarises:

- the guard's own lifecycle: starts, clean stops, crashes, configuration changes;
- goal decisions: forwarded, refused (and by which condition), cancelled while running;
- goals the guard found running without its supervision (orphaned or foreign);
- interventions by ROS 2 functions (collision monitor stops, e-stops, health), with durations;
- Terminal States entered, and the human-authorised transitions that left them.

    vsl_guard_report /var/lib/vsl/guard_ledger.jsonl
    vsl_guard_report /var/lib/vsl/guard_ledger.jsonl --json > pilot_report.json
    vsl_guard_report /var/lib/vsl/guard_ledger.jsonl --since 2026-11-16T09:00
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter, defaultdict
from datetime import datetime
from pathlib import Path
from typing import Any

from vsl_core.ledger import JsonlLedgerStore, VerbaLedger

REFUSED_FALLBACKS = ("goal rejected", "goal not sent upstream")
CANCEL_FALLBACK = "cancel the running goal"
UNSUPERVISED_FALLBACK = "cancel the unsupervised goal"


def _when(ts: float) -> str:
    return datetime.fromtimestamp(ts).isoformat(timespec="seconds")


def build_report(ledger_path: str | Path, since: float | None = None) -> dict[str, Any]:
    ledger = VerbaLedger(JsonlLedgerStore(ledger_path))
    entries = list(ledger.store.all_entries())
    audit = ledger.audit()
    by_id = {e.entry_id: e for e in entries}
    caused = defaultdict(list)
    for e in entries:
        if e.caused_by:
            caused[e.caused_by].append(e)
    scoped = [e for e in entries if since is None or e.timestamp >= since]

    def kind(e) -> str:
        return e.entry_type.value

    # -- the guard's own lifecycle ------------------------------------------------
    lifecycle = []
    for e in scoped:
        event = e.payload.get("event")
        if event == "guard_started":
            lifecycle.append({"at": _when(e.timestamp), "event": "started", "identity": e.identity_key,
                              "previous_run": e.payload.get("previous_run"), "latched": e.payload.get("latched"),
                              "specification_hash": (e.payload.get("specification_hash") or "")[:12]})
        elif event == "guard_stopped":
            lifecycle.append({"at": _when(e.timestamp), "event": "stopped cleanly", "identity": e.identity_key,
                              "goals_cancelled": e.payload.get("goals_cancelled")})
        elif event == "previous_run_ended_unexpectedly":
            lifecycle.append({"at": _when(e.timestamp), "event": "previous run crashed", "identity": e.identity_key})
        elif kind(e) == "SPECIFICATION_UPDATE":
            lifecycle.append({"at": _when(e.timestamp), "event": "configuration changed", "identity": e.identity_key,
                              "changed": e.payload.get("changed")})

    # -- goals ----------------------------------------------------------------------
    decisions: dict[str, list] = defaultdict(list)
    for e in scoped:
        if e.decision_id:
            decisions[e.decision_id].append(e)
    forwarded, refused, cancelled, unsupervised = 0, Counter(), [], []
    for decision_id, items in decisions.items():
        fallbacks = [e.payload.get("fallback") for e in items if kind(e) == "PRE_NODE"]
        # The goal's own admission checks only: a later cancel of the running goal
        # ("running ... goal") or an upstream_goals_supervised finding about it is a
        # separate decision about the same goal, not a refusal of it.
        gate_records = [e for e in items if kind(e) == "MONITOR" and "allowed" in e.payload
                        and str(e.payload.get("subject", "")).endswith(" goal")
                        and not str(e.payload.get("subject", "")).startswith("running ")
                        and e.payload.get("primitive_name") != "upstream_goals_supervised"]
        if any(f in REFUSED_FALLBACKS for f in fallbacks):
            denied = next((e.payload.get("primitive_name") for e in gate_records if e.payload.get("allowed") is False), None)
            drift = [e for e in items if kind(e) == "MONITOR" and "allowed" not in e.payload and e.payload.get("reason")]
            latched = any(e.payload.get("latched") for e in drift)
            reason = "terminal state (latched)" if latched else (drift[0].payload["reason"] if drift else "gate error")
            refused[denied or reason] += 1
        elif gate_records and all(e.payload.get("allowed") for e in gate_records):
            forwarded += 1
        for e in items:
            if kind(e) == "PRE_NODE" and e.payload.get("fallback") == CANCEL_FALLBACK:
                result = next((v.payload.get("result") for v in caused[e.entry_id] if kind(v) == "VERIFICATION"), None)
                cancelled.append({"at": _when(e.timestamp), "goal": decision_id[:8], "reason": e.payload.get("reason"),
                                  "upstream_confirmed": result == "SUFFICIENT"})
            if kind(e) == "MONITOR" and e.payload.get("unsupervised"):
                pre = next((p for p in caused[e.entry_id] if kind(p) == "PRE_NODE"), None)
                result = next((v.payload.get("result") for v in caused[pre.entry_id] if kind(v) == "VERIFICATION"), None) if pre else None
                unsupervised.append({"at": _when(e.timestamp), "goal": decision_id[:8], "kind": e.payload["unsupervised"],
                                     "action": (pre.payload.get("fallback") if pre else None),
                                     "cancel_confirmed": result == "SUFFICIENT"})

    # -- interventions by ROS 2 functions (the witness) ----------------------------
    interventions = []
    for e in scoped:
        if kind(e) == "MONITOR" and e.payload.get("source") == "ros2" and e.payload.get("allowed") is False:
            pre = next((p for p in caused[e.entry_id] if kind(p) == "PRE_NODE"), None)
            ver = next((v for v in caused[pre.entry_id] if kind(v) == "VERIFICATION"), None) if pre else None
            terminal = next((t for t in caused[ver.entry_id] if kind(t) == "TERMINAL"), None) if ver else None
            # A latching condition that fires while already latched is closed at once by
            # the witness ("held by the terminal latch"); that is not a clear.
            already_latched = bool(ver) and "terminal latch" in str(ver.payload.get("verified", ""))
            interventions.append({
                "at": _when(e.timestamp), "condition": e.payload.get("primitive_name"), "detail": e.payload.get("detail"),
                "entered_terminal_state": terminal.payload.get("terminal_state") if terminal else None,
                "while_already_latched": already_latched and not terminal,
                "cleared_after_sec": round(ver.timestamp - e.timestamp, 2) if (ver and not already_latched) else None,
                "still_open": ver is None,
            })

    # -- Terminal States and releases ----------------------------------------------
    terminal_states = []
    for e in scoped:
        if kind(e) == "TERMINAL":
            release = next((h for h in caused[e.entry_id] if kind(h) == "HUMAN_AUTHORISED_TRANSITION"), None)
            terminal_states.append({
                "at": _when(e.timestamp), "state": e.payload.get("terminal_state"), "reason": e.payload.get("reason"),
                "released_by": release.payload.get("authorised_by") if release else None,
                "released_at": _when(release.timestamp) if release else None,
                "held_for_sec": round(release.timestamp - e.timestamp, 1) if release else None,
            })

    return {
        "ledger": str(ledger_path),
        "entries": len(entries),
        "period": {"from": _when(entries[0].timestamp), "to": _when(entries[-1].timestamp)} if entries else None,
        "integrity_ok": ledger.verify_integrity(),
        "audit_passed": audit.all_passed,
        "audit_violations": audit.violations,
        "entry_types": dict(Counter(kind(e) for e in scoped)),
        "lifecycle": lifecycle,
        "goals": {"forwarded": forwarded, "refused": dict(refused), "refused_total": sum(refused.values()),
                  "cancelled_while_running": cancelled},
        "unsupervised_goals": unsupervised,
        "interventions": interventions,
        "terminal_states": terminal_states,
        "integrity_note": "integrity_ok checks the hash chain; audit_passed checks every decision is closed (vsl-core audit()).",
    }


def _print(report: dict[str, Any]) -> None:
    ok = lambda b: "yes" if b else "NO"  # noqa: E731
    print(f"vsl_guard ledger: {report['ledger']}")
    if report["period"]:
        print(f"  {report['entries']} entries, {report['period']['from']} .. {report['period']['to']}")
    print(f"  hash chain intact: {ok(report['integrity_ok'])}    audit passed: {ok(report['audit_passed'])}")
    for check, ids in (report["audit_violations"] or {}).items():
        print(f"    audit: {check}: {len(ids)} open")
    print("\nGuard lifecycle")
    for item in report["lifecycle"] or [{"at": "", "event": "(none)"}]:
        extra = {k: v for k, v in item.items() if k not in ("at", "event", "identity") and v not in (None, "", [])}
        print(f"  {item['at']}  {item['event']}  {extra if extra else ''}")
    g = report["goals"]
    print(f"\nGoals: {g['forwarded']} forwarded, {g['refused_total']} refused, {len(g['cancelled_while_running'])} cancelled while running")
    for reason, n in sorted(g["refused"].items(), key=lambda kv: -kv[1]):
        print(f"  refused by {reason}: {n}")
    for c in g["cancelled_while_running"]:
        print(f"  {c['at']}  cancelled {c['goal']}: {c['reason']}  (upstream confirmed: {ok(c['upstream_confirmed'])})")
    if report["unsupervised_goals"]:
        print("\nGoals found running without the guard's supervision")
        for u in report["unsupervised_goals"]:
            print(f"  {u['at']}  {u['kind']} goal {u['goal']}: {u['action']} (confirmed: {ok(u['cancel_confirmed'])})")
    print(f"\nInterventions by ROS 2 functions: {len(report['interventions'])}")
    for i in report["interventions"]:
        tail = (f"-> terminal state {i['entered_terminal_state']}" if i["entered_terminal_state"]
                else "while already in the terminal state" if i["while_already_latched"]
                else "STILL OPEN" if i["still_open"] else f"verdict cleared after {i['cleared_after_sec']}s")
        print(f"  {i['at']}  {i['condition']}: {i['detail']}  {tail}")
    print(f"\nTerminal States: {len(report['terminal_states'])}")
    for t in report["terminal_states"]:
        rel = f"released by {t['released_by']} after {t['held_for_sec']}s" if t["released_by"] else "NOT RELEASED"
        print(f"  {t['at']}  {t['state']}: {t['reason']}  {rel}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="vsl_guard_report", description=__doc__.split("\n\n")[0])
    parser.add_argument("ledger", help="the guard's ledger_path (JSONL)")
    parser.add_argument("--json", action="store_true", help="machine-readable output")
    parser.add_argument("--since", help="only events at or after this local time (ISO 8601, e.g. 2026-11-16T09:00)")
    args = parser.parse_args(argv)
    if not Path(args.ledger).exists():
        print(f"no ledger at {args.ledger}", file=sys.stderr)
        return 1
    since = datetime.fromisoformat(args.since).timestamp() if args.since else None
    report = build_report(args.ledger, since)
    if args.json:
        print(json.dumps(report, indent=2, default=str))
    else:
        _print(report)
    return 0 if report["integrity_ok"] else 2


if __name__ == "__main__":
    sys.exit(main())
