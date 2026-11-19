"""vsl_guard_release: ask a running vsl_guard to leave its Terminal State.

A Terminal State is left only by a Human-Authorised Transition: a named
approver plus evidence (a root-cause analysis and a specification-update
proposal). This tool writes that request next to the guard's latch file. The
guard applies it within ``release_poll_sec``: it checks that the terminal
conditions hold again (the e-stop is released), records the
HUMAN_AUTHORISED_TRANSITION in its ledger and releases the latch. A request it
refuses is moved to ``<latch>.release.rejected.json`` with the reason.

There is deliberately no ROS service for this. A release needs write access to
the guard's state directory on the robot, so the access control is the robot's
own (who can log in and write there), not "anyone on the DDS domain".

    vsl_guard_release --latch-path /var/lib/vsl/guard_latch.json --status
    vsl_guard_release --latch-path /var/lib/vsl/guard_latch.json \\
        --authorised-by "J. Smith (site safety)" \\
        --root-cause "E-stop pressed by the operator; bay cleared and inspected." \\
        --spec-update "None required."
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

from .guard_node import release_request_path


def _status(latch_path: Path) -> int:
    request = release_request_path(latch_path)
    rejected = request.with_name(request.name.replace(".release.json", ".release.rejected.json"))
    if not latch_path.exists():
        print(f"not in a terminal state ({latch_path} absent)")
    else:
        try:
            data = json.loads(latch_path.read_text(encoding="utf-8"))
            print(f"IN TERMINAL STATE '{data.get('terminal_state_name')}' since {time.ctime(data.get('timestamp', 0))}")
            print(f"  cause: {data.get('reason')}")
        except Exception as error:  # noqa: BLE001
            print(f"IN TERMINAL STATE (latch file unreadable: {error})")
    if request.exists():
        print(f"release request pending: {request}")
    if rejected.exists():
        print(f"last rejected request: {rejected}")
        print("  " + json.loads(rejected.read_text(encoding="utf-8")).get("rejected", ""))
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="vsl_guard_release", description=__doc__.split("\n\n")[0])
    parser.add_argument("--latch-path", required=True, help="the guard's latch_path parameter")
    parser.add_argument("--status", action="store_true", help="show the latch state and exit")
    parser.add_argument("--authorised-by", help="the named human approving the release")
    parser.add_argument("--root-cause", help="root-cause analysis (evidence)")
    parser.add_argument("--spec-update", help="specification-update proposal (evidence; 'None required.' is a valid answer)")
    parser.add_argument("--wait", type=float, default=0.0, help="seconds to wait for the guard to apply the request")
    args = parser.parse_args(argv)

    latch_path = Path(args.latch_path)
    if args.status:
        return _status(latch_path)
    missing = [f"--{n.replace('_', '-')}" for n in ("authorised_by", "root_cause", "spec_update") if not (getattr(args, n) or "").strip()]
    if missing:
        parser.error(f"a release needs {', '.join(missing)}")
    if not latch_path.exists():
        print(f"not in a terminal state ({latch_path} absent); nothing to release")
        return 1

    request = release_request_path(latch_path)
    data = {
        "authorised_by": args.authorised_by.strip(),
        "root_cause_analysis": args.root_cause.strip(),
        "specification_update_proposal": args.spec_update.strip(),
        "requested_at": time.time(),
        "requested_by_user": os.environ.get("USER") or os.environ.get("USERNAME") or "",
    }
    tmp = request.with_name(request.name + ".tmp")
    tmp.write_text(json.dumps(data, indent=2), encoding="utf-8")
    os.replace(tmp, request)
    print(f"release requested ({request}); the guard applies it and records it in its ledger")

    deadline = time.monotonic() + args.wait
    while args.wait and time.monotonic() < deadline:
        if not request.exists():
            if latch_path.exists():
                print("REJECTED -- see --status")
                return 3
            print("released")
            return 0
        time.sleep(0.2)
    return 0 if not args.wait else 4


if __name__ == "__main__":
    sys.exit(main())
