# vsl-ros2 documentation

`vsl_guard` sits between your planner and Nav2 (or any ROS 2 action server). It does four things:

- accepts a task only while the verdicts your stack already produces say the robot is fit for it;
- cancels the task if that stops being true;
- holds the robot in a Terminal State after an e-stop until a named human releases it;
- keeps a tamper-evident record of every decision, including the ones ROS 2's own safety functions made.

It never re-implements a ROS 2 safety function. It reads their verdicts.

## Start here, by role

| You are | Read |
|---|---|
| Deciding whether it fits your robot | [Architecture](architecture.md): sections 1–3, 9 and 11 |
| Integrating it | [Integration guide](integration-guide.md), then the [configuration reference](configuration.md) |
| Running the robot | [Operations](operations.md): health, releasing a Terminal State, reading the record, troubleshooting |
| Testing it in a pilot | [Pilot test plan](pilot-test-plan.md) |
| Seeing it work first | [TurtleBot3 + Nav2 demo](../demo/turtlebot3/README.md), in Docker, about 15 minutes |

## The documents

- **[Architecture](architecture.md)**: where the guard sits, what happens to a goal, the components, the four roles a condition can play, the guard's own lifecycle, the ledger, threading, failure modes, the security model, and what it is not.
- **[Integration guide](integration-guide.md)**: prerequisites, inventorying your stack, installing with colcon and rosdep, writing the configuration, launching, pointing your planner at the guard, SROS2 lockdown, and verification.
- **[Configuration reference](configuration.md)**: every key, its default and its meaning, plus the command-line tools.
- **[Operations](operations.md)**: files and permissions, health on `/diagnostics`, the release procedure, `vsl_guard_report`, restarts and crashes, ledger housekeeping, troubleshooting.
- **[Pilot test plan](pilot-test-plan.md)**: four phases, from demo to normal operation, with pass criteria, a results sheet and the questions we'd like answered.

The [main README](../README.md) also covers the Python API, for building your own guard with rules no ROS 2 function decides.

## Status

Alpha. Tested on ROS 2 Humble and Jazzy (77 tests, installed with pip and with colcon) and end to end against the real Nav2 Jazzy stack driving a simulated TurtleBot3 (44 checks). It has not yet been run on a physical robot: that's what the pilots are for.
