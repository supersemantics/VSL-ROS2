"""The TurtleBot3 demo scenario: an operator, an e-stop panel and a planner,
driving a simulated TurtleBot3 through vsl_guard into the real Nav2 stack.

Everything the scenario claims, it measures: robot motion comes from /odom,
goal outcomes from the action results, the guard's decisions from its ledger
and latch files. It exits non-zero if any step does not behave as claimed.
"""

from __future__ import annotations

import json
import math
import subprocess
import sys
import threading
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool

from vsl_core.ledger import JsonlLedgerStore, VerbaLedger

LATCH = Path("/tmp/vsl/guard_latch.json")
LEDGER = Path("/tmp/vsl/guard_ledger.jsonl")
START = (-2.0, -0.5)  # where nav2_minimal_tb3_sim spawns the robot
STATUS = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED", GoalStatus.STATUS_CANCELED: "CANCELED", GoalStatus.STATUS_ABORTED: "ABORTED"}

results: list[dict] = []


def log(text: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {text}", flush=True)


class Operator(Node):
    """The e-stop panel, the planner (calls the guard, never Nav2), and a robot observer."""

    def __init__(self) -> None:
        super().__init__("demo_operator", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.e_stop_engaged = False
        self._e_stop = self.create_publisher(Bool, "/e_stop", 10)
        self.create_timer(0.1, lambda: self._e_stop.publish(Bool(data=self.e_stop_engaged)))
        self.initial_pose = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.amcl_seen = False
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", lambda _m: setattr(self, "amcl_seen", True), 10)
        self.pose = None
        self.speed = 0.0
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.collision_states: list[str] = []
        names = {0: "DO_NOTHING", 1: "STOP", 2: "SLOWDOWN", 3: "APPROACH", 4: "LIMIT"}
        self.create_subscription(
            CollisionMonitorState, "/collision_monitor_state",
            lambda m: self.collision_states.append(names.get(m.action_type, str(m.action_type))), 10,
        )
        self.guard = ActionClient(self, NavigateToPose, "/vsl_guard/navigate_to_pose")
        # The guard's own state, on /diagnostics: 0 OK (accepting goals), 1 WARN, 2 ERROR (latched).
        self.guard_level: int | None = None
        self.guard_message = ""
        self.create_subscription(DiagnosticArray, "/diagnostics", self._on_diagnostics, 10)
        self.bt_state = self.create_client(GetState, "/bt_navigator/get_state")

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        for s in msg.status:
            if s.name.startswith("vsl_guard:"):
                self.guard_level = s.level[0] if isinstance(s.level, (bytes, bytearray)) else int(s.level)
                self.guard_message = s.message

    def guard_ready(self) -> bool:
        return self.guard_level == 0 and self.guard.server_is_ready()

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.pose = (p.x, p.y)
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        v = msg.twist.twist
        self.speed = math.hypot(v.linear.x, v.linear.y) + abs(v.angular.z) * 0.1

    def world_point_ahead(self, ahead: float) -> tuple[float, float]:
        """A point `ahead` metres in front of the robot, in Gazebo's world frame
        (odom starts at the spawn pose, which is where the world puts the robot)."""
        x, y = self.pose
        return START[0] + x + ahead * math.cos(self.yaw), START[1] + y + ahead * math.sin(self.yaw)

    def publish_initial_pose(self) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x, msg.pose.pose.position.y = START
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.07
        self.initial_pose.publish(msg)


def wait_for(cond, timeout: float, what: str, poll: float = 0.1) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(poll)
    log(f"  timed out after {timeout:.0f}s waiting for {what}")
    return False


def check(name: str, ok: bool, detail: str = "") -> bool:
    results.append({"check": name, "ok": bool(ok), "detail": detail})
    log(f"  {'PASS' if ok else 'FAIL'}  {name}{(' -- ' + detail) if detail else ''}")
    return ok


def goal(x: float, y: float) -> NavigateToPose.Goal:
    g = NavigateToPose.Goal()
    g.pose.header.frame_id = "map"
    g.pose.pose.position.x, g.pose.pose.position.y = x, y
    g.pose.pose.orientation.w = 1.0
    return g


class Goal:
    def __init__(self, op: Operator, x: float, y: float) -> None:
        self.op, self.target = op, (x, y)
        future = op.guard.send_goal_async(goal(x, y))
        wait_for(future.done, 15, "the guard to answer")
        self.handle = future.result() if future.done() else None
        self.accepted = bool(self.handle and self.handle.accepted)
        self._result = self.handle.get_result_async() if self.accepted else None

    def wait(self, timeout: float) -> str:
        if not self.accepted:
            return "REJECTED"
        if not wait_for(self._result.done, timeout, f"goal {self.target} to finish"):
            return "TIMEOUT"
        return STATUS.get(self._result.result().status, str(self._result.result().status))


def distance(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


OBSTACLE_SDF = (
    "<sdf version='1.6'><model name='vsl_obstacle'><static>true</static><link name='l'>"
    "<collision name='c'><geometry><box><size>0.15 0.15 0.4</size></box></geometry></collision>"
    "<visual name='v'><geometry><box><size>0.15 0.15 0.4</size></box></geometry></visual>"
    "</link></model></sdf>"
)


def gz_service(service: str, reqtype: str, req: str) -> bool:
    out = subprocess.run(
        ["gz", "service", "-s", service, "--reqtype", reqtype, "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", req],
        capture_output=True, text=True,
    )
    return "data: true" in out.stdout


def spawn_obstacle(x: float, y: float) -> bool:
    return gz_service("/world/default/create", "gz.msgs.EntityFactory",
                      f'sdf: "{OBSTACLE_SDF}" pose: {{position: {{x: {x}, y: {y}, z: 0.2}}}} name: "vsl_obstacle"')


def remove_obstacle() -> bool:
    return gz_service("/world/default/remove", "gz.msgs.Entity", 'name: "vsl_obstacle" type: MODEL')


A, B = (-0.55, -0.55), (-2.0, 0.55)  # two free spots in the TurtleBot3 world, either side of a pillar


def other(target):
    return B if target == A else A


def ledger_summary() -> dict:
    ledger = VerbaLedger(JsonlLedgerStore(LEDGER))
    entries = list(ledger.store.all_entries())
    kinds: dict[str, int] = {}
    for e in entries:
        kinds[e.entry_type.value] = kinds.get(e.entry_type.value, 0) + 1
    report = ledger.audit()
    return {"entries": len(entries), "by_type": kinds, "audit_passed": report.all_passed,
            "violations": report.violations, "intact": ledger.verify_integrity(), "ledger": ledger, "list": entries}


def main() -> int:
    rclpy.init()
    op = Operator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(op)
    threading.Thread(target=executor.spin, daemon=True).start()

    log("Waiting for the simulated TurtleBot3 and Nav2 ...")
    wait_for(lambda: op.pose is not None, 120, "odometry from Gazebo")
    log(f"  robot at ({op.pose[0]:.2f}, {op.pose[1]:.2f}) in odom")
    # Nav2 activates only once AMCL has a pose: the operator gives it the spawn pose.
    def nav2_active():
        if not op.amcl_seen:
            op.publish_initial_pose()
        if not op.bt_state.service_is_ready():
            return False
        f = op.bt_state.call_async(GetState.Request())
        wait_for(f.done, 3, "bt_navigator state")
        return f.done() and f.result().current_state.label == "active"
    if not wait_for(nav2_active, 240, "Nav2 to become active", poll=2.0):
        return 2
    log("Nav2 is active (bt_navigator, controller, collision monitor)")
    if not wait_for(lambda: op.guard.server_is_ready(), 60, "vsl_guard"):
        return 2

    publishers = sorted({f"{i.node_namespace.rstrip('/')}/{i.node_name}" for i in op.get_publishers_info_by_topic("/cmd_vel")})
    log(f"  /cmd_vel publishers in the real stack: {publishers}")
    if not wait_for(op.guard_ready, 60, "vsl_guard to report OK on /diagnostics"):
        log(f"  guard says: {op.guard_message}")
        return 2
    log(f"  vsl_guard on /diagnostics: {op.guard_message}")

    # ---------------------------------------------------------------- 1
    log("STEP 1  A goal through the guard, everything Nav2 reports is fine")
    start = op.pose
    g = Goal(op, -0.55, -0.55)
    check("guard accepts the goal", g.accepted)
    outcome = g.wait(120)
    check("Nav2 drives the robot there", outcome == "SUCCEEDED", f"{outcome}, moved {distance(start, op.pose):.2f} m")

    # ---------------------------------------------------------------- 2
    log("STEP 2  Nav2's collision monitor stops the robot mid-goal: Nav2's job, not the guard's")
    start = op.pose
    g = Goal(op, *B)
    check("guard accepts the goal", g.accepted)
    wait_for(lambda: distance(start, op.pose) > 0.3 and op.speed > 0.1, 60, "the robot to get going")
    seen_before = len(op.collision_states)
    ox, oy = op.world_point_ahead(0.55)
    log(f"  box dropped into Gazebo 0.55 m ahead of the moving robot at ({ox:.2f}, {oy:.2f})")
    check("obstacle spawned in Gazebo", spawn_obstacle(ox, oy))
    stop_seen = wait_for(lambda: "STOP" in op.collision_states[seen_before:], 5, "the collision monitor to report STOP")
    check("the real collision monitor reports STOP", stop_seen, f"states: {op.collision_states[seen_before:]}")
    check("the robot stops (the collision monitor did it)", wait_for(lambda: op.speed < 0.03, 5, "the robot to stop"),
          f"speed {op.speed:.3f}")
    def witnessed(allowed: bool):
        return any(e.payload.get("primitive_name") == "collision_monitor" and e.payload.get("source") == "ros2"
                   for e in VerbaLedger(JsonlLedgerStore(LEDGER)).store.all_entries() if e.payload.get("allowed") is allowed) \
            if allowed is False else any(
                e.entry_type.value == "VERIFICATION" and "collision monitor" in str(e.payload.get("verified", ""))
                for e in VerbaLedger(JsonlLedgerStore(LEDGER)).store.all_entries())
    check("the guard records the collision monitor's STOP", wait_for(lambda: witnessed(False), 5, "the record"))
    time.sleep(2.0)
    check("the running goal is NOT cancelled by the guard", not g._result.done())
    check("obstacle removed", remove_obstacle())
    check("the collision monitor clears", wait_for(lambda: op.collision_states[-1:] != ["STOP"], 10, "the collision monitor to clear"),
          f"states: {op.collision_states[seen_before:]}")
    outcome = g.wait(120)
    check("the robot resumes and completes the same goal", outcome == "SUCCEEDED", f"{outcome}, moved {distance(start, op.pose):.2f} m")
    check("the guard records the release of the hold", wait_for(lambda: witnessed(True), 5, "the record"))

    # ---------------------------------------------------------------- 3
    log("STEP 3  E-stop mid-goal")
    start = op.pose
    g = Goal(op, *A)
    check("guard accepts the goal", g.accepted)
    wait_for(lambda: distance(start, op.pose) > 0.25, 60, "the robot to get going")
    moving_at = op.pose
    log(f"  robot moving ({op.speed:.2f}), {distance(start, op.pose):.2f} m into the goal: e-stop pressed")
    op.e_stop_engaged = True
    t0 = time.monotonic()
    outcome = g.wait(20)
    check("the running goal is cancelled in Nav2 (planner sees it fail)", outcome == "ABORTED", f"{outcome} after {time.monotonic() - t0:.1f}s")
    check("the guard latches (Terminal State on disk)", wait_for(LATCH.exists, 5, "latch file"))
    stopped = wait_for(lambda: op.speed < 0.02, 10, "the robot to stop")
    check("the robot stops", stopped, f"speed {op.speed:.3f}")
    held_at = op.pose

    log("  e-stop button released")
    op.e_stop_engaged = False
    time.sleep(8.0)
    drift = distance(held_at, op.pose)
    check("the robot does NOT resume the old goal after the button is released", drift < 0.05 and op.speed < 0.02,
          f"moved {drift:.3f} m in 8 s")
    g = Goal(op, *B)
    check("a new goal is refused while latched", not g.accepted)

    # ---------------------------------------------------------------- 4
    log("STEP 4  A release request while the e-stop is engaged is refused")
    op.e_stop_engaged = True
    time.sleep(1.0)
    subprocess.run(["ros2", "run", "vsl_ros2", "vsl_guard_release", "--latch-path", str(LATCH),
                    "--authorised-by", "Demo operator", "--root-cause", "test", "--spec-update", "None required."], check=True)
    rejected = LATCH.with_name("guard_latch.json.release.rejected.json")
    check("release refused (e-stop still engaged)", wait_for(rejected.exists, 5, "rejection") and LATCH.exists(),
          json.loads(rejected.read_text()).get("rejected", "") if rejected.exists() else "")
    op.e_stop_engaged = False
    time.sleep(1.0)

    log("STEP 5  A named human releases it, with evidence")
    rc = subprocess.run(["ros2", "run", "vsl_ros2", "vsl_guard_release", "--latch-path", str(LATCH),
                         "--authorised-by", "J. Smith (site safety)",
                         "--root-cause", "Operator pressed the e-stop during the demo; area clear.",
                         "--spec-update", "None required.", "--wait", "10"]).returncode
    check("release applied and recorded", rc == 0 and not LATCH.exists())
    start = op.pose
    g = Goal(op, -2.0, 0.55)
    outcome = g.wait(120)
    check("goals flow again; the robot completes the goal", g.accepted and outcome == "SUCCEEDED",
          f"{outcome}, moved {distance(start, op.pose):.2f} m")

    # ---------------------------------------------------------------- 6
    log("STEP 6  A rogue node starts publishing on /cmd_vel mid-goal (bypass)")
    start = op.pose
    g = Goal(op, -0.55, -0.55)
    check("guard accepts the goal", g.accepted)
    wait_for(lambda: distance(start, op.pose) > 0.2, 60, "the robot to get going")
    rogue = Node("teleop_rogue")
    rogue_pub = rogue.create_publisher(Twist, "/cmd_vel", 10)
    executor.add_node(rogue)
    outcome = g.wait(20)
    check("the goal is cancelled when the command path is bypassed", outcome == "ABORTED", outcome)
    g2 = Goal(op, -2.0, 0.55)
    check("new goals refused while the bypass exists", not g2.accepted)
    executor.remove_node(rogue)
    rogue.destroy_node()
    del rogue_pub
    time.sleep(3.0)
    g3 = Goal(op, -2.0, 0.55)
    outcome = g3.wait(120)
    check("goals flow again once the rogue publisher is gone", g3.accepted and outcome == "SUCCEEDED", outcome)

    # ---------------------------------------------------------------- 7
    log("STEP 7  A goal sent to Nav2 directly, around the guard")
    direct = ActionClient(op, NavigateToPose, "/navigate_to_pose")
    wait_for(direct.server_is_ready, 10, "Nav2's action server")
    start = op.pose
    f = direct.send_goal_async(goal(*A))
    wait_for(f.done, 10, "Nav2 to answer")
    h = f.result()
    check("Nav2 accepts it (nothing in ROS 2 stops that)", bool(h and h.accepted))
    r = h.get_result_async()
    check("vsl_guard cancels it (upstream_goals_supervised)", wait_for(r.done, 10, "the bypass goal to end")
          and r.result().status == GoalStatus.STATUS_CANCELED, f"robot moved {distance(start, op.pose):.2f} m")
    check("recorded as a bypass", wait_for(lambda: any(e.payload.get("unsupervised") == "foreign"
                                                       for e in ledger_summary()["list"]), 5, "the record"))
    wait_for(lambda: op.speed < 0.02, 10, "the robot to settle")

    def guard_pid() -> str:
        return subprocess.run(["pgrep", "-f", "lib/vsl_ros2/vsl_guard"], capture_output=True, text=True).stdout.split()

    def wait_for_guard_restart(old: list[str]) -> bool:
        return wait_for(lambda: guard_pid() and guard_pid() != old and op.guard.server_is_ready(), 30, "the guard to respawn")

    def wait_for_guard_ready() -> bool:
        op.guard_level = None  # only a report from the new process counts
        ok = wait_for(op.guard_ready, 30, "the guard to report OK on /diagnostics")
        log(f"  vsl_guard on /diagnostics: {op.guard_message}")
        return ok

    # ---------------------------------------------------------------- 8
    log("STEP 8  The guard is killed (kill -9) mid-goal; ros2 launch respawns it")
    start = op.pose
    g = Goal(op, *A)
    check("guard accepts the goal", g.accepted)
    wait_for(lambda: distance(start, op.pose) > 0.25 and op.speed > 0.1, 60, "the robot to get going")
    old = guard_pid()
    subprocess.run(["pkill", "-9", "-f", "lib/vsl_ros2/vsl_guard"])
    killed_at = op.pose
    t0 = time.monotonic()
    check("the guard comes back", wait_for_guard_restart(old), f"after {time.monotonic() - t0:.1f}s")
    gap = distance(killed_at, op.pose)
    log(f"  while no guard was running, Nav2 kept driving the orphaned goal: {gap:.2f} m")
    check("the respawned guard cancels the orphaned goal; the robot stops",
          wait_for(lambda: op.speed < 0.02, 15, "the robot to stop"), f"{time.monotonic() - t0:.1f}s after the kill")
    held_at = op.pose
    time.sleep(4.0)
    check("and it stays stopped", distance(held_at, op.pose) < 0.05, f"moved {distance(held_at, op.pose):.3f} m in 4 s")
    s = ledger_summary()["list"]
    check("the crash is recorded (previous run ended without a stop record)",
          any(e.payload.get("event") == "previous_run_ended_unexpectedly" for e in s))
    check("the orphaned goal is recorded and cancelled", any(e.payload.get("unsupervised") == "orphaned" for e in s))
    check("the respawned guard reports ready", wait_for_guard_ready())

    # ---------------------------------------------------------------- 9
    log("STEP 9  The guard is stopped cleanly (SIGINT, as ros2 launch does) mid-goal")
    start = op.pose
    g = Goal(op, *B)
    check("guard accepts the goal", g.accepted)
    wait_for(lambda: distance(start, op.pose) > 0.25 and op.speed > 0.1, 60, "the robot to get going")
    old = guard_pid()
    subprocess.run(["pkill", "-INT", "-f", "lib/vsl_ros2/vsl_guard"])
    stopped = wait_for(lambda: op.speed < 0.02, 10, "the robot to stop")
    check("the stopping guard cancels the goal it supervised; the robot stops", stopped)
    check("the guard comes back", wait_for_guard_restart(old))
    check("the respawned guard reports ready", wait_for_guard_ready())
    s = ledger_summary()["list"]
    stops = [e for e in s if e.payload.get("event") == "guard_stopped"]
    check("the clean stop is recorded with the goal it cancelled", bool(stops) and stops[-1].payload.get("goals_cancelled") == 1,
          str(stops[-1].payload if stops else ""))
    starts = [e for e in s if e.payload.get("event") == "guard_started"]
    check("the restart knows the previous run stopped cleanly", starts[-1].payload.get("previous_run") == "stopped cleanly")
    start = op.pose
    g = Goal(op, *B)
    outcome = g.wait(120)
    check("goals flow again", g.accepted and outcome == "SUCCEEDED", f"{outcome}, moved {distance(start, op.pose):.2f} m")
    time.sleep(1.0)

    # ---------------------------------------------------------------- record
    log("THE RECORD")
    s = ledger_summary()
    check("ledger chain intact", s["intact"])
    check("ledger passes vsl-core audit()", s["audit_passed"], json.dumps(s["violations"]) if s["violations"] else "")
    log(f"  {s['entries']} entries: {s['by_type']}")
    hat = [e for e in s["list"] if e.entry_type.value == "HUMAN_AUTHORISED_TRANSITION"]
    if hat:
        log(f"  human-authorised transition by {hat[0].payload.get('authorised_by')!r}, caused_by {hat[0].caused_by[:8]}")
    cm = sorted(set(op.collision_states))
    log(f"  collision monitor states seen from the real Nav2 collision_monitor: {cm or 'none (no polygon triggered)'}")

    passed = sum(r["ok"] for r in results)
    log(f"RESULT  {passed}/{len(results)} checks passed")
    Path("/tmp/vsl/demo_results.json").write_text(json.dumps(
        {"checks": results, "cmd_vel_publishers": publishers, "collision_states": cm,
         "ledger": {k: v for k, v in s.items() if k not in ("ledger", "list")}}, indent=2, default=str))
    executor.shutdown()
    rclpy.try_shutdown()
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
