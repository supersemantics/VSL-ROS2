# Operating vsl_guard

For the people who run the robot day to day: where the guard keeps its state, how to tell if it's healthy, how to release a Terminal State, how to read the record, and what to do when it refuses goals.

## Files

| File | Configured by | Contents | Rule |
|---|---|---|---|
| Ledger | `ledger_path` | Every decision, intervention, cancel, release and lifecycle event; append-only, hash-chained JSONL | Persistent storage. Writable by the guard's user only |
| Latch | `latch_path` | Present only while in the Terminal State: the cause and the TERMINAL entry id | Must survive reboots, or a reboot releases the robot |
| Release request | `<latch_path>.release.json` | Written by `vsl_guard_release`, consumed by the guard | Whoever can write here can request a release |
| Rejected request | `<latch_path>.release.rejected.json` | The last refused request and the reason | — |
| Lock | `<ledger_path>.lock` | Cross-process write lock | Leave it |

A typical setup on the robot:

```bash
sudo mkdir -p /var/lib/vsl
sudo chown robot:robot-safety /var/lib/vsl     # the guard's user; the safety team's group
sudo chmod 2770 /var/lib/vsl                   # nobody else can request a release or edit the ledger
```

Keep the directory off `tmpfs`. Don't use `/tmp` on hardware: a reboot would clear the latch.

## Is the guard healthy?

The guard publishes its state on `/diagnostics` every second, under the name `vsl_guard: /vsl_guard`:

```bash
ros2 topic echo /diagnostics --field status | grep vsl_guard
```

| Level | Message | Meaning |
|---|---|---|
| OK | `accepting goals for /navigate_to_pose` | Every gating condition holds |
| WARN | `not accepting goals: bt_navigator, e_stop` | Those conditions don't hold; goals are rejected until they do |
| ERROR | `terminal state 'robot-e-stopped': needs a human release` | Latched |

The status values list every condition and its current detail (`e_stop: holds: /e_stop released`), plus how many goals the guard is supervising. Any diagnostic aggregator, `rqt_robot_monitor` or fleet dashboard picks this up like any other diagnostic.

## Leaving a Terminal State

When a `terminal: true` condition fires (an e-stop):

- the guard cancels running goals;
- it latches (`/diagnostics` ERROR);
- it rejects every goal until a human releases it;
- releasing the e-stop button alone does not resume anything.

To release:

1. Make the area safe and clear the cause. The e-stop must be released: the guard refuses a release while any terminal condition still fails.
2. Check the state:

   ```bash
   ros2 run vsl_ros2 vsl_guard_release --latch-path /var/lib/vsl/guard_latch.json --status
   ```

3. Release, as a named person, with evidence:

   ```bash
   ros2 run vsl_ros2 vsl_guard_release --latch-path /var/lib/vsl/guard_latch.json \
       --authorised-by "J. Smith (site safety)" \
       --root-cause "Operator pressed the e-stop: pallet in the aisle. Aisle cleared and inspected." \
       --spec-update "None required." --wait 5
   ```

   `released` means it's done and recorded (a `HUMAN_AUTHORISED_TRANSITION` entry). `REJECTED` means it was refused; `--status` shows why.

The latch survives guard restarts and reboots, as long as `latch_path` is on persistent storage. There is no timeout and no automatic release.

## Reading the record

```bash
ros2 run vsl_ros2 vsl_guard_report /var/lib/vsl/guard_ledger.jsonl
```

```
vsl_guard ledger: /var/lib/vsl/guard_ledger.jsonl
  79 entries, 2026-11-16T21:22:27 .. 2026-11-16T21:23:45
  hash chain intact: yes    audit passed: yes

Guard lifecycle
  2026-11-16T21:22:27  started  {'previous_run': 'first run', 'latched': False, 'specification_hash': '42e39f6a1ace'}
  2026-11-16T21:23:32  started  {'previous_run': 'ended without a stop record', ...}
  2026-11-16T21:23:32  previous run crashed
  2026-11-16T21:23:39  stopped cleanly  {'goals_cancelled': 1}

Goals: 7 forwarded, 2 refused, 3 cancelled while running
  refused by terminal state (latched): 1
  refused by cmd_vel_publishers: 1
  2026-11-16T21:23:00  cancelled 92e7a60e: in terminal state 'robot-e-stopped'; ...  (upstream confirmed: yes)

Goals found running without the guard's supervision
  2026-11-16T21:23:26  foreign goal e5fbe6e5: cancel the unsupervised goal (confirmed: yes)
  2026-11-16T21:23:35  orphaned goal feebdb9c: cancel the unsupervised goal (confirmed: yes)

Interventions by ROS 2 functions: 4
  2026-11-16T21:22:45  collision_monitor: collision monitor action STOP (PolygonStop)  verdict cleared after 0.14s
  2026-11-16T21:23:00  e_stop: /e_stop engaged  -> terminal state robot-e-stopped

Terminal States: 1
  2026-11-16T21:23:00  robot-e-stopped: /e_stop engaged  released by J. Smith (site safety) after 11.8s
```

(This is a real report from the TurtleBot3 demo, abridged.)

- `--json` gives the same report as machine-readable output, which is what to send back from a pilot.
- `--since 2026-11-16T09:00` limits it to one shift.
- The exit code is `2` if the hash chain is broken: someone edited the file.

"audit passed: no" lists the decisions that aren't closed. A cancel that Nav2 refused, for example, stays open until someone records a specification update. That's deliberate: a goal that couldn't be cancelled needs a human to look at it.

## Restarts and crashes

- **Clean stop** (`Ctrl-C` on the launch, `systemctl stop`, SIGTERM): the guard cancels every goal it supervises, records them, and writes `guard_stopped`.
- **Crash, `kill -9`, power loss:** the guard can't act. Nav2 keeps driving the current goal until the guard is back; `ros2 launch` respawns it after 2 s. On start, the guard:
  - records that the previous run ended without a stop record;
  - finds the orphaned goal on Nav2's status topic and cancels it.

  In the demo the robot stopped 3–5 s after the kill. A hardware e-stop, not the guard, is what protects against this window.
- **Configuration changed** between runs: recorded as a `SPECIFICATION_UPDATE` with the changed keys. The guard logs `configuration changed since the last run: ...`.

## Ledger housekeeping

The ledger grows with decisions, not with messages. The demo wrote 79 entries in 2 minutes of continuous testing; a robot doing a few hundred tasks a day writes a few thousand entries a day. vsl-ros2 doesn't rotate the file itself. To archive it:

1. Make sure the guard isn't latched (`vsl_guard_release --status`).
2. Stop the guard cleanly.
3. Run `vsl_guard_report <ledger> --json > <archive>.report.json`, then move the ledger to the archive.
4. Start the guard. It begins a new chain, starting with a `guard_started` record that says `first run`.

Copy archived ledgers off the robot if they must outlive it. The hash chain makes tampering detectable, not impossible.

## Troubleshooting

| Symptom (`/diagnostics` or log) | Cause | Fix |
|---|---|---|
| `not accepting goals: cmd_vel_publishers` | A node not in `allowed` publishes on the command topic | Run `ros2 topic info -v /cmd_vel`. Add it to `allowed` if intended (Nav2 Jazzy's `/docking_server` is), otherwise find out why it's there |
| `not accepting goals: e_stop` with `no message from the ROS function yet` | Nothing publishing the e-stop topic | Check the topic name and that its publisher runs continuously |
| `not accepting goals: bt_navigator` with `/bt_navigator is inactive` | Nav2 not active (bringup still running, or lifecycle manager failed) | Bring Nav2 up. For Nav2, this usually means it's waiting for an initial pose |
| Not ready a few seconds after a restart | Lifecycle state and graph checks take one to three seconds to arrive | Wait for OK on `/diagnostics` before sending goals |
| `release request rejected: terminal condition(s) still not holding: e_stop` | E-stop still engaged | Release the e-stop, then request again |
| `configuration error: ...` and the node exits | Invalid YAML for the guard | The message names the key; see the [configuration reference](configuration.md) |
| `cancelling foreign goal ... (a bypass)` | Something sent a goal straight to Nav2 | Find the client; move it to `served_action`; lock it down with SROS2 |
| `the previous run ended without a stop record` | The guard crashed or was killed | Read the guard log for why; the report lists any orphan it cancelled |
| `... refused to cancel ... it is still running` | Nav2 refused a cancel | Stop the robot by other means; investigate. The audit stays open until reviewed |
| `audit passed: no` with `pre_node_has_verification` | A decision still in progress when the report ran (a hold not yet cleared, a cancel awaiting Nav2's answer) | Re-run once the robot is idle. If it persists, the listed entries show which decision |
| Install from source on Humble produces package `UNKNOWN` | Ubuntu 22.04's pip 22.0 | Use colcon, or `pip install --upgrade pip` in the venv first |
