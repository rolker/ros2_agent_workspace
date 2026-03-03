# ADR-0007: Retain Make with Dependency Tracking

## Status

Accepted

## Context

The workspace uses a GNU Makefile with 19+ targets as a task runner. Most targets are
thin wrappers around shell scripts — no dependency graph, no incremental build logic.
A fresh-clone workflow requires the user to know and run three commands in order
(`make bootstrap`, `make setup-all`, `make build`), and re-running setup when nothing
changed wastes time.

Issue [#249](https://github.com/rolker/ros2_agent_workspace/issues/249) research
(§1–2) evaluated three alternatives:

- **`just`** — Rust-based command runner with clean syntax, built-in `--list`, no
  `.PHONY` needed. Strong fit as a command runner, but no dependency model.
- **`task`** — Go-based task runner using YAML. Checksum-based dependencies, but
  verbose syntax for what are mostly simple shell commands.
- **`mise`** — Polyglot tool combining version management, environment variables, and
  task running. Overkill for this workspace's needs.

The research framed the question as "Makefile vs command runner." But the real question
turned out to be different: the workspace doesn't need a better command runner — it
needs to use Make's dependency model, which is the one thing command runners don't have.

The current Makefile treats every target as independent. If the dependency graph were
wired up correctly, `make build` would handle bootstrap, manifest setup, and layer
setup automatically on a fresh clone — and skip already-completed steps on subsequent
runs.

## Decision

Retain GNU Make as the workspace task runner. Adopt the **stamp-file pattern** to
express dependencies between setup stages, so that Make's dependency model enforces
correct ordering and enables incremental re-runs.

The stamp-file pattern uses empty marker files (e.g., `.make/bootstrap.done`) as build
targets. Make checks whether the stamp exists and is newer than its prerequisites. This
is a standard pattern used by the Linux kernel, ROS's own build tools, and many other
projects.

Key behaviors after implementation:

- **Fresh clone**: `make build` runs bootstrap → manifest → layer setups → build
- **Already set up**: `make build` skips straight to the build step
- **A `.repos` file changes**: `make build` re-runs only that layer's setup, then builds
- **`make test`**: depends on `build`, handling setup transitively

What stays the same:

- `make help` and the self-documenting target pattern
- `generate-skills` and the `.claude/skills/make_*/SKILL.md` slash command pattern
- No new tool dependency — Make is already installed on every development system

## Consequences

**Positive:**

- `make build` works correctly from a fresh clone — no need to know the setup order
- Incremental re-runs skip completed steps, saving time on subsequent runs
- The dependency graph *enforces* correct ordering rather than documenting it
- No new tool to install, learn, or maintain
- Slash command generation (`generate-skills`) continues to work unchanged

**Negative:**

- Stamp files add a hidden state directory (`.make/`) that can go stale if the
  filesystem is modified outside Make (e.g., manually deleting `configs/manifest`)
- Make syntax remains less readable than `just` for simple command definitions
- Stamp invalidation strategy (simple touch vs hash-based) adds implementation
  complexity — details tracked in [#332](https://github.com/rolker/ros2_agent_workspace/issues/332)

**Follow-up work:**

- [#332](https://github.com/rolker/ros2_agent_workspace/issues/332) — Refactor Makefile
  with stamp-file dependencies (implementation of this decision)
- [#330](https://github.com/rolker/ros2_agent_workspace/issues/330) — Improve first-run
  initialization UX with interactive manifest setup
