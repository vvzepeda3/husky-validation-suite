# husky-validation-suite

Requirements-driven validation of a waypoint-following controller for the Clearpath Husky, in ROS 2 Jazzy and Gazebo Harmonic, run in Docker.

> 🚧 **Status: work in progress.** The pure-Python controller core, the ROS 2 node, draft requirements
> ([docs/requirements.md](docs/requirements.md)) and design ([docs/design.md](docs/design.md)) are in place,
> with 73 passing unit tests. Not done yet: the ROS build hasn't been verified in the Jazzy container, and the
> integration tests, Gazebo simulation tests, CI and traceability matrix haven't been written. Sections marked
> _TODO_ below are still to be written.

## Overview

_TODO: what the controller does, what "validated" means here, and who this is for._

## Requirements

**Software**

- Docker (ROS runs in a container; Ubuntu 24.04 base)
- ROS 2 Jazzy (`osrf/ros:jazzy-desktop` or your own Jazzy image)
- Gazebo Harmonic (`ros-jazzy-ros-gz`)
- Clearpath simulator (`clearpath_simulator`)
- Python 3.12
- colcon

**Functional requirements**

_TODO: link to the requirements document in `docs/`. IDs use the format `REQ-<AREA>-<NNN>` (e.g. `REQ-NAV-001`)._

## Architecture

_TODO: node graph, topics and frames, and how the waypoint controller sits on top of the Husky platform in simulation._

## Test Strategy

_TODO: describe each test level._

| Level | Tooling | Needs ROS / Gazebo | Command |
| --- | --- | --- | --- |
| Unit | pytest | No | `make unit` |
| Integration | `launch_testing` | ROS | _TBD_ |
| Simulation | Gazebo Harmonic + Husky | Both | _TBD_ |

Every test carries a requirement ID marker, e.g. `@pytest.mark.req("REQ-NAV-001")`. The test run fails if one is missing.

## Traceability

_TODO: requirement → test → result matrix, and how it's generated._

| Requirement | Description | Tests | Status |
| --- | --- | --- | --- |
| _REQ-…_ | | | |

## Quickstart

_TODO: fill in once the package has runnable nodes._

```bash
# Clone
git clone <repo-url> husky-validation-suite
cd husky-validation-suite

# Unit tests (no ROS needed)
make unit

# Build the workspace inside a ROS 2 Jazzy container
make docker-shell
# --- inside the container ---
source /opt/ros/jazzy/setup.bash
apt-get update && rosdep update
rosdep install --from-paths src --ignore-src -y
make build
source install/setup.bash
```

## CI

_TODO: pipeline stages (lint → unit → build → integration/sim), where it runs, and how results feed the traceability matrix._

## Life Cycle

_TODO: how work flows from requirement → branch → tests → review → release, versioning (SemVer) and changelog policy._

See [CONTRIBUTING.md](CONTRIBUTING.md) for branch naming, commit style and the PR checklist, and [CHANGELOG.md](CHANGELOG.md) for release history.

## License

[MIT](LICENSE)
