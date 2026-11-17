"""Static checks on the package layout. They read source files and never import rclpy."""

from __future__ import annotations

import ast
from dataclasses import fields
from pathlib import Path

import pytest
from husky_waypoint_controller.core import ControllerParams, validate_params

PKG_DIR = Path(__file__).resolve().parents[1]
CORE = PKG_DIR / "husky_waypoint_controller" / "core.py"
NODE = PKG_DIR / "husky_waypoint_controller" / "node.py"
PARAMS_YAML = PKG_DIR / "config" / "params.yaml"
ROS_ONLY_MODULES = ("rclpy", "geometry_msgs", "nav_msgs", "std_msgs", "std_srvs", "builtin_interfaces")


def _imported_modules(path: Path) -> set[str]:
    tree = ast.parse(path.read_text())
    names: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            names.add(node.module.split(".")[0])
    return names


@pytest.mark.req("REQ-ARCH-001")
def test_core_has_no_ros_imports() -> None:
    assert _imported_modules(CORE).isdisjoint(ROS_ONLY_MODULES)


@pytest.mark.req("REQ-CTL-001", "REQ-CTL-002", "REQ-CTL-006", "REQ-ARCH-003")
def test_cmd_vel_is_published_only_through_the_clamp() -> None:
    """``self._cmd_pub.publish`` appears exactly once, inside ``_publish_cmd``,
    after a call to ``clamp_velocity``, so no code path can skip the clamp."""
    tree = ast.parse(NODE.read_text())
    publish_sites: list[tuple[str, int]] = []
    clamp_lines: dict[str, list[int]] = {}

    for func in ast.walk(tree):
        if not isinstance(func, ast.FunctionDef):
            continue
        for node in ast.walk(func):
            if not isinstance(node, ast.Call):
                continue
            target = node.func
            if (
                isinstance(target, ast.Attribute)
                and target.attr == "publish"
                and isinstance(target.value, ast.Attribute)
                and target.value.attr == "_cmd_pub"
            ):
                publish_sites.append((func.name, node.lineno))
            if isinstance(target, ast.Name) and target.id == "clamp_velocity":
                clamp_lines.setdefault(func.name, []).append(node.lineno)

    assert len(publish_sites) == 1, publish_sites
    func_name, publish_line = publish_sites[0]
    assert func_name == "_publish_cmd"
    assert any(line < publish_line for line in clamp_lines.get(func_name, []))


@pytest.mark.req("REQ-IF-007")
def test_node_never_targets_platform_cmd_vel() -> None:
    """No string literal in the node names the post-mux topic (comments are allowed)."""
    tree = ast.parse(NODE.read_text())
    literals = [n.value for n in ast.walk(tree) if isinstance(n, ast.Constant) and isinstance(n.value, str)]
    assert not [text for text in literals if "platform/cmd_vel" in text]


@pytest.mark.req("REQ-IF-006", "REQ-SAFE-006")
def test_params_yaml_defines_every_core_parameter_with_valid_values() -> None:
    yaml = pytest.importorskip("yaml")
    values = yaml.safe_load(PARAMS_YAML.read_text())["waypoint_controller"]["ros__parameters"]
    core_names = [f.name for f in fields(ControllerParams)]
    missing = [name for name in core_names if name not in values]
    assert not missing
    assert validate_params(ControllerParams(**{name: values[name] for name in core_names})) == []
    for name in ("namespace", "odom_topic", "cmd_vel_stamped", "base_frame_id"):
        assert name in values
