"""Enforces the project rule that every test carries a requirement ID.

Usage: ``@pytest.mark.req("REQ-NAV-001")`` (one or more IDs per marker).
Lives in the package's test dir so it also applies under ``colcon test``.
"""

from __future__ import annotations

import re

import pytest

REQ_ID = re.compile(r"^REQ-[A-Z]+-\d{3}$")


def pytest_configure(config: pytest.Config) -> None:
    config.addinivalue_line("markers", "req(*ids): requirement ID(s) this test verifies, e.g. REQ-NAV-001")
    config.addinivalue_line("markers", "integration: needs ROS 2 and/or Gazebo; skipped by `make unit`")


def pytest_collection_modifyitems(config: pytest.Config, items: list[pytest.Item]) -> None:
    problems: list[str] = []
    for item in items:
        ids = [req_id for marker in item.iter_markers("req") for req_id in marker.args]
        if not ids:
            problems.append(f"{item.nodeid}: missing @pytest.mark.req(...)")
        problems.extend(f"{item.nodeid}: bad requirement ID {i!r}" for i in ids if not REQ_ID.match(str(i)))
    if problems:
        raise pytest.UsageError("Requirement ID check failed:\n  " + "\n  ".join(problems))
