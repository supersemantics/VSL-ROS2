# vsl_guard configuration reference

`vsl_guard` is configured entirely with ROS 2 parameters, normally from a YAML file passed with `--params-file`, or with the `params_file` argument of `vsl_guard.launch.py`. Nested keys (`release.authority`, `e_stop.topic`) follow ROS 2's usual dotted-parameter convention. Start from `config/nav2_guard.yaml`.

A configuration error stops the node with a message naming the key:

```
vsl_guard: configuration error: Condition 'x': type must be one of lock, sole_publisher, lifecycle, collision_monitor, diagnostics (got 'magic').
```

The configuration is the guard's policy. Its hash is recorded at every start, and a change between runs is recorded as a `SPECIFICATION_UPDATE` naming the keys that changed.

## Top-level keys

| Key | Type | Default | Meaning |
|---|---|---|---|
| `action_type` | string | `nav2_msgs/action/NavigateToPose` | The action guarded. Any ROS 2 action type |
| `served_action` | string | `/vsl_guard/navigate_to_pose` | What the planner calls |
| `upstream_action` | string | `/navigate_to_pose` | The real server the guard forwards to |
| `upstream_accept_timeout_sec` | float | `10.0` | How long the upstream server has to accept a forwarded goal before the planner's goal is aborted |
| `conditions` | string list | — (required) | The conditions, in the order they are checked and recorded. Each name is also a key holding that condition's settings |
| `ledger_path` | string | `vsl_guard_ledger.jsonl` | The ledger file. Use a persistent directory; created if missing |
| `latch_path` | string | `vsl_guard_latch.json` | The Terminal State file. Present means latched. Release requests go next to it |
| `terminal_state` | string | `robot-held` | Name of the Terminal State entered by `terminal: true` conditions |
| `release.authority` | string | `Site safety officer` | The `GovernanceAuthority` recorded on every release |
| `release.contact` | string | `unset` | Its escalation contact |
| `release_poll_sec` | float | `1.0` | How often release requests are checked |
| `orphaned_goals` | `cancel` \| `record` | `cancel` | What to do with goals running upstream that the guard doesn't supervise: orphaned by a crash, or sent around the guard |
| `startup_check_sec` | float | `3.0` | After a crash, how long the guard watches for orphans before recording its startup check |
| `diagnostics_period_sec` | float | `1.0` | How often the guard publishes its state on `/diagnostics` |
| `identity_key` | string | the node's name | The identity on every ledger entry. Set it per robot if ledgers are merged |
| `use_sim_time` | bool | `false` | As usual. The guard's own timers use the steady clock either way |

## Keys every condition accepts

| Key | Type | Default | Meaning |
|---|---|---|---|
| `type` | string | — (required) | `lock`, `sole_publisher`, `lifecycle`, `collision_monitor` or `diagnostics` |
| `gate` | bool | `true` | `false`: record interventions, never block goals. For ROS 2 functions that act on their own verdict |
| `cancel_running` | bool | `false` | Also cancel a running goal when this fails. Needs `gate: true` |
| `terminal` | bool | `false` | A failure enters the Terminal State: latch, cancel running goals, refuse goals until a human releases. Also counts as an intervention when the first message after start already fails (an e-stop engaged at boot) |

## Condition types

### `lock`: an e-stop or lock topic

Reads `std_msgs/Bool`; `true` means engaged. It holds while the latest message is `false`.

| Key | Default | Meaning |
|---|---|---|
| `topic` | — (required) | The Bool topic |
| `max_age_sec` | none | If set, no message for this long counts as engaged. Set it when the source can die silently |

Until its first message, the lock counts as engaged (fail closed).

### `sole_publisher`: nothing bypasses the command topic

Reads the ROS graph (`get_publishers_info_by_topic`). It holds while only the allowed nodes publish on the topic.

| Key | Default | Meaning |
|---|---|---|
| `topic` | — (required) | The topic the driver listens to |
| `allowed` | — (required) | Fully qualified node names, e.g. `[/collision_monitor, /docking_server]` |
| `period_sec` | `1.0` | How often the graph is checked |

### `lifecycle`: a managed node is active

Reads `<node>/get_state` once, then follows `<node>/transition_event`.

| Key | Default | Meaning |
|---|---|---|
| `node` | — (required) | e.g. `/bt_navigator` |
| `required_state` | `active` | The label that counts as holding |

### `collision_monitor`: Nav2's collision monitor

Reads `nav2_msgs/CollisionMonitorState`. It holds while the action isn't one of `blocking_actions`.

| Key | Default | Meaning |
|---|---|---|
| `topic` | `/collision_monitor_state` | The collision monitor's `state_topic` |
| `blocking_actions` | `[STOP]` | Any of `DO_NOTHING`, `STOP`, `SLOWDOWN`, `APPROACH`, `LIMIT` (`LIMIT` exists on Jazzy and later) |
| `assume_clear_until_first` | `true` if `gate: false`, else `false` | Whether "no message yet" counts as clear |

**Use `gate: false`.** The collision monitor publishes only when its action changes. After a goal ends during a STOP, the state stays STOP until Nav2 drives again, so gating new goals on it deadlocks the robot. Record it, and gate on its lifecycle instead.

### `diagnostics`: component health

Reads `diagnostic_msgs/DiagnosticArray`. It holds while every named component reports at or below `worst_allowed`, recently.

| Key | Default | Meaning |
|---|---|---|
| `components` | — (required) | Status names, e.g. `[/Drive, /Sensors/Lidar]` |
| `topic` | `/diagnostics_agg` | Aggregated (recommended) or raw `/diagnostics` |
| `worst_allowed` | `WARN` | `OK`, `WARN` or `ERROR`. STALE never passes |
| `max_age_sec` | `5.0` | A component not reported for this long fails |

## Example: Nav2 with a diagnostics gate

```yaml
vsl_guard:
  ros__parameters:
    action_type: nav2_msgs/action/NavigateToPose
    served_action: /vsl_guard/navigate_to_pose
    upstream_action: /navigate_to_pose
    ledger_path: /var/lib/vsl/guard_ledger.jsonl
    latch_path: /var/lib/vsl/guard_latch.json
    terminal_state: robot-e-stopped
    release: { authority: Site safety officer, contact: safety@your-site.example }
    conditions: [e_stop, cmd_vel_publishers, bt_navigator, collision_monitor_active, collision_monitor, drive_health]
    e_stop:                   { type: lock, topic: /e_stop, max_age_sec: 1.0, terminal: true }
    cmd_vel_publishers:       { type: sole_publisher, topic: /cmd_vel, allowed: [/collision_monitor, /docking_server], cancel_running: true }
    bt_navigator:             { type: lifecycle, node: /bt_navigator }
    collision_monitor_active: { type: lifecycle, node: /collision_monitor }
    collision_monitor:        { type: collision_monitor, topic: /collision_monitor_state, gate: false }
    drive_health:             { type: diagnostics, components: [/Drive], worst_allowed: WARN, cancel_running: true }
```

## Command-line tools

| Tool | Use |
|---|---|
| `ros2 run vsl_ros2 vsl_guard_release --latch-path P --status` | Show whether the guard is latched, why, and any pending or rejected request |
| `ros2 run vsl_ros2 vsl_guard_release --latch-path P --authorised-by NAME --root-cause TEXT --spec-update TEXT [--wait S]` | Request a release. The guard applies it only when the terminal conditions hold again |
| `ros2 run vsl_ros2 vsl_guard_report LEDGER [--json] [--since 2026-11-16T09:00]` | Summarise a ledger: integrity, audit, lifecycle, decisions, interventions, Terminal States. Exit code 2 if the hash chain is broken |
