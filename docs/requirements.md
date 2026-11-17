# Requirements: Husky Waypoint Controller

| | |
| --- | --- |
| **Document** | `docs/requirements.md` |
| **Version** | 0.1 (**DRAFT**, awaiting review) |
| **System** | `husky_waypoint_controller` on a Clearpath Husky A200 |
| **Target** | ROS 2 Jazzy, Gazebo Harmonic (`clearpath_simulator`); Humble supported via parameters |

## 1. Purpose and scope

The waypoint controller drives a Husky through an ordered list of 2D waypoints using odometry feedback. It stops safely on request, on emergency stop, and when its inputs go stale.

**In scope:** waypoint following on flat ground, velocity limiting, waypoint validation, the controller state machine, the watchdog, emergency-stop handling and the ROS interface.

**Out of scope (v0.1):** obstacle detection and avoidance, global path planning, GPS / map-frame navigation, reversing, docking, multi-robot coordination.

## 2. Conventions

- **IDs:** `REQ-<AREA>-<NNN>`. IDs are never reused or renumbered. Retired requirements are marked *Deleted* and kept.
- **Areas:**
  - `NAV`: navigation behavior
  - `WPT`: waypoint validation
  - `CTL`: command limits
  - `SM`: state machine
  - `SAFE`: safety and watchdog
  - `IF`: ROS interface
  - `ARCH`: architecture and testability
- **"shall"** marks a mandatory requirement and **"should"** a recommended one.
- **Verification method:**
  - **U:** unit test (pytest, no ROS)
  - **I:** integration test (`launch_testing`, ROS, no simulator)
  - **S:** simulation test (Gazebo + `clearpath_simulator`)
  - **X:** inspection or review
- **Units:** SI units (m, rad, s). Angles are wrapped to [-π, π). Frames follow REP 103 / REP 105.
- Every requirement must be verified by at least one test carrying its ID (`@pytest.mark.req("REQ-...")`), or by a recorded inspection.

## 3. Platform facts and assumptions

These come from Clearpath's `clearpath_common` and `clearpath_simulator` (`humble` / `jazzy` branches) and constrain the requirements below.

| # | Fact / assumption | Consequence |
| --- | --- | --- |
| A1 | Topics live under the robot namespace (default = serial, e.g. `a200_0000`). | Namespace is a parameter (REQ-IF-001). |
| A2 | Velocity input is `cmd_vel`. It is `geometry_msgs/TwistStamped` on Jazzy and `geometry_msgs/Twist` on Humble. | Message type is selectable (REQ-IF-002). |
| A3 | `cmd_vel` enters Clearpath's `twist_mux` at priority 1 (lowest). Teleop overrides it, and `platform/emergency_stop` locks the mux. | Teleop and E-stop can preempt us at any time, so the controller must not assume its commands reach the wheels. |
| A4 | The mux and the diff-drive controller both zero the wheels if no command arrives for 0.5 s. | Our watchdog must trip sooner (REQ-SAFE-006). The platform timeout is the backstop if our node dies. |
| A5 | The A200 diff-drive controller caps speed at 1.0 m/s and 1.0 rad/s. | Our limits must not exceed these (REQ-CTL-001, REQ-CTL-002). |
| A6 | `platform/odom/filtered` (EKF, wheels + IMU) is on by default. `platform/odom` (wheel only, 50 Hz) is always present. | The odometry topic is a parameter (REQ-IF-003). |
| A7 | Ground is flat and obstacle-free in the validation worlds. | No obstacle requirements in v0.1. |

## 4. Requirements

### 4.1 Navigation (NAV)

| ID | Requirement | Acceptance criteria | Verif. |
| --- | --- | --- | --- |
| REQ-NAV-001 | The controller shall consider a waypoint reached when the robot's position is within `goal_tolerance_m` of it. | Default 0.20 m. Reached iff Euclidean distance ≤ tolerance. | U, S |
| REQ-NAV-002 | The controller shall visit waypoints strictly in the order given, advancing to the next one only after the current one is reached. | No waypoint is skipped or reordered. | U, S |
| REQ-NAV-003 | On reaching the final waypoint, the controller shall command zero velocity and report the mission as complete. | Zero command published and state → IDLE (REQ-SM-003). | U, I |
| REQ-NAV-004 | The commanded angular velocity shall turn the robot toward the current target, using the heading error wrapped to [-π, π). | Sign of ω equals sign of the wrapped heading error, including across the ±π boundary. | U |
| REQ-NAV-005 | When the absolute heading error exceeds `rotate_in_place_threshold_rad`, the commanded linear velocity shall be zero. | Default 0.785 rad (45°). | U |
| REQ-NAV-006 | The controller should reduce linear speed when approaching the final waypoint, so it stops within tolerance without overshoot. | Linear speed scales down within `slowdown_radius_m` (default 1.0 m). In sim, final position error ≤ `goal_tolerance_m` with no reversal. | U, S |
| REQ-NAV-007 | In simulation, the robot shall complete the reference course: a 5 m × 5 m square (4 waypoints, starting at the origin). | Every waypoint reached within tolerance. Course completes within 60 s and ends in IDLE. | S |

### 4.2 Waypoint validation (WPT)

| ID | Requirement | Acceptance criteria | Verif. |
| --- | --- | --- | --- |
| REQ-WPT-001 | The controller shall reject an empty waypoint list. | Goal rejected with reason `EMPTY`. | U |
| REQ-WPT-002 | The controller shall reject any waypoint list containing a non-finite coordinate (NaN or ±inf). | Goal rejected with reason `NON_FINITE`. | U |
| REQ-WPT-003 | The controller shall reject waypoint lists longer than `max_waypoints`. | Default 100. Goal rejected with reason `TOO_MANY`. | U |
| REQ-WPT-004 | The controller shall reject any waypoint farther than `max_waypoint_range_m` from the robot's position when the goal is received. | Default 50 m. Goal rejected with reason `OUT_OF_RANGE`. | U |
| REQ-WPT-005 | The controller shall reject waypoints whose frame does not match the odometry frame. | Goal rejected with reason `WRONG_FRAME`. | U, I |
| REQ-WPT-006 | A rejected goal shall not change the controller's state or its current mission. The rejection and its reason shall be reported to the caller. | State and active waypoints unchanged after rejection, and the reason is returned. | U, I |

### 4.3 Command limits (CTL)

| ID | Requirement | Acceptance criteria | Verif. |
| --- | --- | --- | --- |
| REQ-CTL-001 | Commanded linear speed shall stay within [0, `max_linear_speed_mps`]. | Default 0.5 m/s. The parameter must be ≤ 1.0 (A5). | U |
| REQ-CTL-002 | Commanded angular speed magnitude shall not exceed `max_angular_speed_radps`. | Default 0.8 rad/s. The parameter must be ≤ 1.0 (A5). | U |
| REQ-CTL-003 | The change in commanded linear speed between consecutive cycles shall not exceed `max_linear_accel_mps2` × cycle period, except when commanding a stop. | Default 0.5 m/s². Stops (REQ-SAFE-*) may jump straight to zero. | U |
| REQ-CTL-004 | The controller shall never command reverse motion. | v ≥ 0 for every published command. | U |
| REQ-CTL-005 | The controller shall publish commands at `control_rate_hz` while NAVIGATING. | Default 20 Hz. Measured rate within ±10 % over 10 s. | I |
| REQ-CTL-006 | If the control math produces a non-finite command, the controller shall publish zero velocity and enter FAULT. | Non-finite inputs never reach `cmd_vel`. | U |

### 4.4 State machine (SM)

States: **IDLE** (no active mission), **NAVIGATING**, **STOPPED** (mission paused), **FAULT** (unsafe condition detected).

| ID | Requirement | Acceptance criteria | Verif. |
| --- | --- | --- | --- |
| REQ-SM-001 | The controller shall start in IDLE. | State is IDLE after startup. | U, I |
| REQ-SM-002 | A valid goal received in IDLE or STOPPED shall start a new mission and enter NAVIGATING. A valid goal in NAVIGATING shall replace the current mission. | Transitions as stated. The previous mission is discarded. | U |
| REQ-SM-003 | Completing the final waypoint shall transition NAVIGATING → IDLE. | — | U |
| REQ-SM-004 | A stop request shall transition NAVIGATING → STOPPED, keeping the remaining waypoints. | Remaining waypoints preserved. | U |
| REQ-SM-005 | A resume request shall transition STOPPED → NAVIGATING only if the E-stop is released and odometry is fresh. Otherwise it is rejected with a reason. | — | U |
| REQ-SM-006 | FAULT shall be left only by an explicit reset request, which succeeds only if the fault condition has cleared. Reset leads to IDLE and clears the mission. | Reset with stale odometry is rejected and the controller stays in FAULT. | U |
| REQ-SM-007 | Goals received in FAULT shall be rejected. | Rejected with reason `IN_FAULT`. | U |
| REQ-SM-008 | In every state other than NAVIGATING, the controller shall command only zero velocity. On entering any such state, it shall publish at least one zero command. | No non-zero command outside NAVIGATING. | U, I |
| REQ-SM-009 | The controller shall publish its current state and the reason for its last transition, latched for late subscribers. | A late subscriber receives the current state. | I |
| REQ-SM-010 | All state transitions shall be determined by the state machine's transition table. Transitions not in the table shall be refused and logged. | Every (state, event) pair has a defined outcome. | U |

### 4.5 Safety and watchdog (SAFE)

| ID | Requirement | Acceptance criteria | Verif. |
| --- | --- | --- | --- |
| REQ-SAFE-001 | When the E-stop is asserted (`platform/emergency_stop` = true), the controller shall command zero velocity within one control cycle and enter STOPPED. If in FAULT, it stays in FAULT. | Zero command within 1 / `control_rate_hz` (50 ms at 20 Hz). | U, I |
| REQ-SAFE-002 | Releasing the E-stop shall not resume motion automatically. Motion resumes only after an explicit resume request. | No non-zero command after release until resume. | U, I |
| REQ-SAFE-003 | While NAVIGATING or STOPPED, if no odometry message has arrived within `odom_timeout_s`, the watchdog shall command zero velocity and enter FAULT with reason `ODOM_STALE`. | Default 0.2 s. | U, I |
| REQ-SAFE-004 | The watchdog shall detect a timeout no later than `odom_timeout_s` + one control period after the last odometry message. | ≤ 0.25 s with the defaults. | U, I |
| REQ-SAFE-005 | A goal shall be rejected if no odometry has been received yet, or if odometry is stale. | Rejected with reason `NO_ODOM`. | U |
| REQ-SAFE-006 | `odom_timeout_s` shall be configured below the platform's 0.5 s command timeout (A4). | The node refuses to start if `odom_timeout_s` ≥ 0.5. | U, I |
| REQ-SAFE-007 | The watchdog shall use a monotonic clock in real runs and simulation time in simulation (`use_sim_time`). A clock jump backwards shall be treated as a timeout. | No crash or false "fresh" reading on clock reset. | U |
| REQ-SAFE-008 | If the controller node stops publishing (crash or hang), the robot shall stop within 0.5 s via the platform command timeout. | Verified in sim by killing the node mid-mission. | S |

### 4.6 ROS interface (IF)

| ID | Requirement | Acceptance criteria | Verif. |
| --- | --- | --- | --- |
| REQ-IF-001 | All Husky topics shall be resolved relative to the `namespace` parameter. | Default `a200_0000`. Topic names are never hard-coded with a namespace. | I, X |
| REQ-IF-002 | The controller shall publish velocity on `<namespace>/cmd_vel` as `TwistStamped` (Jazzy, the default) or `Twist` (Humble), selected by the `cmd_vel_stamped` parameter. Stamped messages carry `frame_id = base_link` and the current time. | Both types verified. | I |
| REQ-IF-003 | The controller shall subscribe to odometry on `<namespace>/<odom_topic>`. | Default `platform/odom/filtered`. Can be set to `platform/odom`. | I |
| REQ-IF-004 | The controller shall subscribe to `<namespace>/platform/emergency_stop` (`std_msgs/Bool`). | — | I |
| REQ-IF-005 | The controller shall offer interfaces to submit a goal (waypoint list), stop, resume and reset. Each one returns success or failure with a reason. | Interface names and types are defined in `docs/design.md`. | I |
| REQ-IF-006 | All parameters shall be validated at startup against their allowed ranges. On invalid configuration the node shall log every violation and refuse to start. | Covers REQ-CTL-001, REQ-CTL-002 and REQ-SAFE-006 limits. | U, I |
| REQ-IF-007 | The controller shall never publish to `platform/cmd_vel` or otherwise bypass `twist_mux`. | — | X |

### 4.7 Architecture and testability (ARCH)

| ID | Requirement | Acceptance criteria | Verif. |
| --- | --- | --- | --- |
| REQ-ARCH-001 | All control math (heading control, velocity limiting, waypoint validation, state transitions, watchdog timing) shall live in a pure-Python module that doesn't import `rclpy` or any ROS message package. | A test imports the module with `rclpy` unavailable. Static check for forbidden imports. | U, X |
| REQ-ARCH-002 | The pure-Python core shall be deterministic. Time is passed in rather than read from a clock, so identical inputs give identical outputs. | Repeated runs with identical inputs give identical outputs. | U |
| REQ-ARCH-003 | The ROS node shall be a thin adapter: it converts messages to and from core types, holds timers and parameters, and contains no control logic. | Review checklist. | X |
| REQ-ARCH-004 | `make unit` shall run every unit test without a ROS installation. | Passes on a host without ROS. | U, X |

## 5. Parameters (defaults referenced above)

| Parameter | Type | Default | Allowed range | Requirement |
| --- | --- | --- | --- | --- |
| `namespace` | string | `a200_0000` | non-empty | REQ-IF-001 |
| `cmd_vel_stamped` | bool | `true` | — | REQ-IF-002 |
| `odom_topic` | string | `platform/odom/filtered` | non-empty | REQ-IF-003 |
| `goal_tolerance_m` | float | 0.20 | (0, 2.0] | REQ-NAV-001 |
| `rotate_in_place_threshold_rad` | float | 0.785 | (0, π] | REQ-NAV-005 |
| `slowdown_radius_m` | float | 1.0 | [0, 10] | REQ-NAV-006 |
| `max_linear_speed_mps` | float | 0.5 | (0, 1.0] | REQ-CTL-001 |
| `max_angular_speed_radps` | float | 0.8 | (0, 1.0] | REQ-CTL-002 |
| `max_linear_accel_mps2` | float | 0.5 | (0, 2.0] | REQ-CTL-003 |
| `control_rate_hz` | float | 20.0 | [5, 100] | REQ-CTL-005 |
| `max_waypoints` | int | 100 | [1, 1000] | REQ-WPT-003 |
| `max_waypoint_range_m` | float | 50.0 | (0, 500] | REQ-WPT-004 |
| `odom_timeout_s` | float | 0.2 | (0, 0.5) | REQ-SAFE-003, REQ-SAFE-006 |

## 6. Open questions for review

1. **Controller law:** proportional heading control (simpler) or pure pursuit (smoother on curves)? The requirements above are written to allow either.
2. **Goal interface:** ROS action (feedback and cancel built in), or topic plus services? This affects REQ-IF-005.
3. **Default speed:** is 0.5 m/s right for validation, or should runs go closer to the 1.0 m/s platform cap?
4. **Behavior on E-stop:** should E-stop during NAVIGATING keep the mission for resume (as drafted), or clear it?
5. **Reference course:** is a 5 m square in an empty world enough for REQ-NAV-007, or do you want a second course with sharp turns (e.g. a figure-8)?

## 7. Change log

| Version | Date | Change |
| --- | --- | --- |
| 0.1 | 2026-11-02 | Initial draft for review. |
