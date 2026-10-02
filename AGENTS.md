# Repository Guidelines

## Project Structure & Module Organization

This repository is a ROS1 Noetic catkin workspace. `src/uav_system/` is the single native ROS package: its `src/` contains state source management, state adaptation, supervision, PX4 backend, commander, and shared `support/uav_core` modules. State messages live in `src/support/msg` within that package.

Package-local `config/`, `launch/`, `test/`, and `docs/` hold configuration, launch files, tests, and architecture/validation records. Real source configurations use `config/state_sources/`; mock configuration uses `config/test/mock.yaml`. `src/open_vins/` contains independent OpenVINS packages. PX4 firmware resides in `src/uav_system/third_party/px4_autopilot/` and is excluded from catkin discovery. Keep generated `build/`, `devel/`, logs, and `.legacy_catkin/` backups out of commits.

## Build, Test, and Development Commands

Run from the workspace root:

```bash
./scripts/build.sh                 # Noetic build; defaults to -j2 -p1
source devel/setup.bash            # Load generated packages/messages
python3 -m pip install -r src/uav_system/test/requirements.txt
bash src/uav_system/scripts/run_checks.sh
roslaunch uav_system mock_system.launch  # Software-only base chain
```

`catkin build -j2 -p1` also works with a correctly prepared Noetic environment. Never mix Humble into this workspace.

For the current SSHFS checkout, edit workspace files directly; execute builds and runtime checks on Jetson:

```bash
ssh jetson-uav-odom 'cd /home/jetson/uav_odom_px4 && ./scripts/build.sh'
```

## Coding Style & Naming Conventions

Use four-space indentation for new Python code, `snake_case` functions/files, and `PascalCase` classes/message types. Follow surrounding style and keep changes focused. Bash scripts should quote paths and use `set -euo pipefail`. No repository-wide formatter is configured; checks include Python syntax, XML/YAML structure, and Git whitespace validation.

## Testing Guidelines

Tests use Python `unittest`: name files `test_*.py` and methods `test_*`. Preserve mathematical, readiness, watchdog, mission, protocol, and layout coverage; add relevant regression cases. No numerical coverage threshold is configured. Software checks do not establish ROS integration or hardware success; record those separately in package-local `docs/VALIDATION.md`.

## Commit & Pull Request Guidelines

Recent commits use `feat:` and `refactor:` with imperative descriptions. Match that pattern with an appropriate type. PRs should explain the problem, resulting behavior, validation commands/results, and limitations; link relevant issues. Keep third-party changes separate and explicitly justified.

## Hardware & Configuration Boundaries

Read package-local `docs/LOCAL_VALIDATION.md` before hardware work. Prevent duplicate camera/MAVROS processes. Keep calibration flags false until verified. Starting Commander can automatically request Offboard and arming: require explicit task authorization for flight actions, parameter writes, or firmware flashing.
