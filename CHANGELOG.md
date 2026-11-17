# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- `husky_waypoint_controller` core (pure Python): state machine (IDLE, NAVIGATING, STOPPED, FAULT), proportional heading control, waypoint validation, `clamp_velocity()` and an injectable-clock `Watchdog`.
- `waypoint_controller` ROS 2 node: odometry and E-stop inputs, `cmd_vel` output (`TwistStamped` or `Twist`), `waypoints` topic, `stop`/`resume`/`reset` services and a latched JSON `status` topic.
- `config/params.yaml` with every limit, and `waypoint_controller.launch.py`.
- Draft `docs/requirements.md` (v0.1) and `docs/design.md` (v0.1).
- Unit tests tagged with requirement IDs, including static checks that `core.py` has no ROS imports and that `cmd_vel` is only published through the clamp.
