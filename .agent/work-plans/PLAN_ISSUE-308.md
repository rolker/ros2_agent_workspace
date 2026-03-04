# Plan: Improve repo onboarding

## Issue

**Issue**: #308
**Status**: In Progress

https://github.com/rolker/ros2_agent_workspace/issues/308

## Context

Only ~1 of 8+ project repos has `.agents/README.md`, 2 have pre-commit, 2 have
CI. Templates exist (`.agent/templates/`) but are passive — no guided workflow
to apply them. The `audit-project` skill reports gaps but doesn't fix them.

Design decision (from issue discussion): an **interactive audit** skill that
presents each gap and lets the user choose "fix now," "open issue," or "skip."
All "fix now" items go into a single PR on the target project repo.

## Approach

1. **Create CI workflow template** (`.agent/templates/ci_workflow.yml`)
   - Based on `s57_tools` CI pattern: `ros:jazzy-ros-core` container, rosdep
     install, colcon build + test
   - Parameterized: default branch name, package list (discovered from
     `package.xml` files), extra dependency cloning steps left as TODO comments
   - Self-contained — no dependency on workspace scripts

2. **Create pre-commit config template** (`.agent/templates/pre-commit-config.yaml`)
   - Based on `s57_tools` pattern: standard hooks, cmake-lint, yamllint,
     no-commit-to-branch
   - Parameterized: protected branch names (discovered from repo's default branch)

3. **Create the `onboard-project` skill** (`.claude/skills/onboard-project/SKILL.md`)
   - **Input**: repo name (directory under `layers/main/<layer>_ws/src/`)
   - **Audit phase**: run checks (reuse `audit-project` checklist categories):
     - `.pre-commit-config.yaml` exists?
     - CI workflow exists in `.github/workflows/`?
     - `.agents/README.md` exists?
     - GitHub branch protection configured? Copilot auto-review enabled?
     - `pkg:` labels exist (for multi-package repos)?
     - License/copyright headers present per ADR-0008?
   - **Interactive phase**: for each gap, ask the user:
     - **Fix now** — add to the batch
     - **Open issue** — create an issue on the project repo
     - **Skip** — move on
   - **Implementation phase** (if any "fix now" items):
     - Create a layer worktree on the project repo
     - For pre-commit: copy template, adjust protected branches
     - For CI: copy template, fill in package list from `package.xml` discovery,
       fill in default branch
     - For `.agents/README.md`: generate from `.agent/templates/project_agents_guide.md`,
       reusing the same package-discovery logic as `gather-project-knowledge` where helpful
     - For GitHub settings: when credentials have admin/write scopes, use `gh api`
       to configure branch protection and Copilot auto-review; if permissions are
       insufficient, skip and report in the summary
     - For `pkg:` labels: create labels via `gh label create`
     - Commit all changes, open a single PR on the project repo
   - **Summary**: report what was fixed (PR link), deferred (issue links), skipped

4. **Update skill lists in non-Claude adapters**
   - Add `onboard-project` to the skill list in `.github/copilot-instructions.md`,
     `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md`

## Files to Change

| File | Change |
|------|--------|
| `.claude/skills/onboard-project/SKILL.md` | New skill definition |
| `.agent/templates/ci_workflow.yml` | New CI workflow template |
| `.agent/templates/pre-commit-config.yaml` | New pre-commit config template |
| `.github/copilot-instructions.md` | Add skill to list |
| `.agent/instructions/gemini-cli.instructions.md` | Add skill to list |
| `.agent/AGENT_ONBOARDING.md` | Add skill to list |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Interactive per-item choice — user controls scope |
| Enforcement over documentation | Skill brings enforcement artifacts (CI, hooks) to repos |
| Workspace vs. project separation | Templates are generic; generated files are self-contained in project repos |
| Workspace improvements cascade | Directly implements this — workspace patterns flow to projects |
| Only what's needed | Optional items, one repo at a time, skip what's not wanted |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0003 — Project-agnostic | Yes | Templates use placeholders, not hardcoded project names |
| 0004 — Enforcement hierarchy | Yes | Skill sets up CI (authoritative) + pre-commit (feedback) |
| 0005 — Layered enforcement | Yes | CI first, pre-commit as complement |
| 0008 — ROS 2 conventions | Yes | License checks reference conventions knowledge file |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| New framework skill | Skill list in non-Claude adapters | Yes (step 4) |
| New templates | Docs that reference templates | N/A — no existing references |

## Open Questions

None — design decisions resolved in issue discussion.

## Success Criteria

- `.agent/templates/ci_workflow.yml` and `.agent/templates/pre-commit-config.yaml` exist
  with parameterized placeholders (not hard-coded project names or branches).
- `onboard-project` skill interactively detects missing CI/pre-commit/agent guide,
  offers to apply templates, and stages changes into a single PR on the target repo.
- Non-Claude adapters list the new skill so it is discoverable across frameworks.
- Graceful handling of insufficient GitHub permissions (report, don't fail).

## Estimated Scope

Single PR to workspace repo (skill + templates + adapter updates).
