# CLAUDE.md

Guidance for Claude Code (and humans) working in this repository.

## Project

`husky-validation-suite` is a ROS 2 workspace for building and validating a waypoint-following controller for the Clearpath Husky in simulation. Every behavior is tied to a requirement ID, and every requirement is verified by tests.

## Stack

| Component | Version / choice |
| --- | --- |
| ROS 2 | Jazzy Jalisco, run in Docker (`osrf/ros:jazzy-desktop` or your own Jazzy image) |
| Simulator | Gazebo Harmonic (the Gazebo release paired with Jazzy), via `ros_gz` |
| Robot | Clearpath Husky (`clearpath_simulator`) |
| Python | 3.12 (the Jazzy / Ubuntu 24.04 system Python) |
| Build | colcon, `ament_python` packages |
| Tests | pytest (unit); `launch_testing` + Gazebo (integration, later) |
| Lint / format | ruff, black |

Target only APIs available in Jazzy and Python 3.12. Do not use 3.13+ syntax or stdlib features. ROS runs inside Docker; the host only needs Python for `make unit` and `make lint`.

## Folder layout

```
husky-validation-suite/
├── CLAUDE.md                 # this file
├── README.md
├── CONTRIBUTING.md
├── CHANGELOG.md              # Keep a Changelog
├── LICENSE                   # MIT
├── Makefile                  # unit / lint / format / build / test
├── pyproject.toml            # ruff, black, pytest config (workspace-wide)
├── docs/                     # requirements, architecture, traceability
└── src/                      # colcon workspace source space
    └── husky_waypoint_controller/        # ament_python package
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── resource/husky_waypoint_controller
        ├── husky_waypoint_controller/    # Python module
        │   └── __init__.py
        └── test/
            └── conftest.py               # enforces requirement ID markers
```

- New ROS packages go under `src/<package_name>/`. Run colcon from the repo root.
- `build/`, `install/` and `log/` are colcon output and are gitignored. Never commit them.
- Keep ROS-independent logic (math, waypoint handling, state machines) in plain Python modules that don't import `rclpy`, so unit tests run without ROS. Keep `rclpy` nodes thin wrappers around that logic.

## Coding standards

- **Type hints everywhere.** Annotate all function parameters and return types, including `-> None`. Use built-in generics (`list[float]`) and `X | None`. Use `type` aliases and PEP 695 generics only where they genuinely help readability.
- **black** formats the code (line length 110). Don't hand-format against it.
- **ruff** is the linter. The rules live in `pyproject.toml`. Fix warnings; don't add blanket `# noqa`. If a suppression is truly needed, scope it to one rule and explain why.
- Public modules, classes and functions get docstrings. Tests don't need them.
- Use SI units (meters, radians, seconds) and REP 103 / REP 105 frame conventions. Name units in variables when it helps (`timeout_s`, `max_speed_mps`).
- No magic numbers in controller logic. Use ROS parameters or named constants.
- Run `make format` and `make lint` before committing.

## Requirements and tests

Every test must carry a requirement ID marker:

```python
import pytest

@pytest.mark.req("REQ-NAV-001")
def test_reaches_single_waypoint_within_tolerance() -> None:
    ...
```

- ID format: `REQ-<AREA>-<NNN>`, e.g. `REQ-NAV-001` or `REQ-SAFE-002`. A test may cover more than one ID: `@pytest.mark.req("REQ-NAV-001", "REQ-NAV-004")`.
- `src/husky_waypoint_controller/test/conftest.py` fails the whole test run if any test is missing the marker or has a malformed ID. Don't weaken or bypass that check. New packages must copy that conftest into their own `test/` directory.
- Requirement IDs must exist in the requirements doc (`docs/`). Don't invent IDs in tests.
- Mark tests that need ROS or Gazebo with `@pytest.mark.integration`. `make unit` skips them.

## Commits

Use [Conventional Commits](https://www.conventionalcommits.org/):

```
<type>(<scope>): <summary>

[body]

[footer: Refs: REQ-NAV-001]
```

- Types: `feat`, `fix`, `test`, `docs`, `refactor`, `perf`, `build`, `ci`, `chore`.
- Scope is usually the package name or area, e.g. `feat(waypoint_controller): ...` or `ci: ...`.
- Write the summary in the imperative mood, lowercase, with no trailing period, in 72 characters or fewer.
- Reference the requirement IDs the change implements or verifies in the footer.
- Breaking changes use `!` after the type/scope and a `BREAKING CHANGE:` footer.
- User-visible changes also get an entry under `## [Unreleased]` in `CHANGELOG.md`.

## Definition of done

Nothing is "done" until **`make unit` passes**. Run it and read the output before saying a task is complete. If it fails, the task isn't finished. Report the failure; don't hide it, skip tests or delete them to get green.

Also expected before a change is ready for review:
- `make lint` is clean.
- New behavior has tests carrying the right requirement IDs.
- `CHANGELOG.md` is updated for user-visible changes.

## Commands

```bash
make unit      # pytest unit tests, no ROS needed (the definition-of-done gate)
make lint      # ruff check + black --check
make format    # black + ruff --fix
make build     # colcon build --symlink-install (needs ROS 2 Jazzy sourced)
make test      # colcon test + test-result (needs ROS 2 Jazzy sourced)
make docker-shell  # shell in a Jazzy container with the repo mounted at /ws
make clean     # remove build/ install/ log/
```

`make build` and `make test` run inside the Jazzy container, not on the host:

```bash
make docker-shell                 # or: make docker-shell ROS_IMAGE=<your-jazzy-image>
source /opt/ros/jazzy/setup.bash  # inside the container
make build && make test
```
