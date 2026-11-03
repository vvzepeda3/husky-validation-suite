# Contributing

Thanks for helping out. This project validates a Husky waypoint controller against written requirements, so every change should be traceable to a requirement ID.

## Branch naming

```
<type>/<req-id>-<short-description>
```

- `type` matches the Conventional Commit types: `feat`, `fix`, `test`, `docs`, `refactor`, `perf`, `build`, `ci`, `chore`.
- Include the requirement ID when the work implements or verifies one. Leave it out for repo chores.
- Use lowercase and hyphens, and keep it short.

Examples:

```
feat/req-nav-001-single-waypoint
test/req-safe-002-estop-timeout
fix/req-nav-004-heading-wraparound
ci/add-unit-workflow
docs/architecture-diagram
```

Branch from `main` and keep branches short-lived.

## Commit style

We use [Conventional Commits](https://www.conventionalcommits.org/).

```
<type>(<scope>): <summary>

<body: what and why, wrapped at 72 chars>

Refs: REQ-NAV-001
```

- Write the summary in the imperative mood, lowercase, with no trailing period, in 72 characters or fewer.
- Scope is the package or area, e.g. `waypoint_controller`, `sim`, `ci` or `docs`.
- List the requirement IDs touched in a `Refs:` footer.
- Breaking changes: `feat(waypoint_controller)!: ...` plus a `BREAKING CHANGE:` footer.

Examples:

```
feat(waypoint_controller): stop within tolerance of final waypoint

Refs: REQ-NAV-001
```

```
test(waypoint_controller): cover heading wraparound at ±pi

Refs: REQ-NAV-004
```

## Before you open a PR

```bash
make format
make lint
make unit
```

`make unit` must pass. Nothing is done until it does.

## PR checklist

Copy this into your PR description:

```markdown
## Summary
<!-- What changed and why -->

## Requirements
<!-- e.g. Implements REQ-NAV-001, verifies REQ-NAV-002 -->

## Checklist
- [ ] Branch name follows `<type>/<req-id>-<short-description>`
- [ ] Commits follow Conventional Commits, with `Refs:` footers for requirement IDs
- [ ] `make unit` passes
- [ ] `make lint` is clean (ruff + black)
- [ ] New or changed behavior has tests
- [ ] Every new test has a `@pytest.mark.req("REQ-...")` marker with an ID from the requirements doc
- [ ] Type hints on all new functions
- [ ] Traceability updated if requirements or tests changed
- [ ] `CHANGELOG.md` updated under `[Unreleased]` (if user-visible)
- [ ] Integration/simulation tests run locally if the change affects runtime behavior (or explain why not)
```

## Reviews

- At least one approving review before merging.
- Squash-merge, using a Conventional Commit title for the squashed commit.
- Delete the branch after merging.
