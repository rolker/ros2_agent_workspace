# Plan: v1 /start-deployment skill (deployment-mode gap 1)

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/501

(Design decisions locked in [issue comment](https://github.com/rolker/ros2_agent_workspace/issues/501#issuecomment-4560650816) — the issue title's "+ activation marker" phrase is **stale**; the locked design drops the marker. See Open Questions for rename.)

## Context

[ADR-0014](https://github.com/rolker/ros2_agent_workspace/blob/main/docs/decisions/0014-deployment-mode.md) (Draft, PR [#500](https://github.com/rolker/ros2_agent_workspace/pull/500)) defines deployment mode as a phase-aware behavioral autonomy mode (urgency contract) + lifecycle tooling that runs inside it. This PR ships the *first* lifecycle tool: `/start-deployment`, which activates a *single* agent session into deployment mode and either creates / first-activates / resumes a deployment.

The design comment on #501 is the authoritative spec; this plan is the structural / governance / consequences view.

## Approach

1. **Knowledge doc first** (`.agent/knowledge/deployment_mode.md`) — write the canonical reference: lifecycle phases, log-naming convention (per-host file, deployment-start date, dev/`hostname -s` label rule), what-to-write guidance, the `issue_sync` config schema, the project-config schema, and pointers to ADR-0014 and the skill.
2. **Template** (`.agent/templates/deployment_config.yaml`) — a fully-commented sample project deployment config (`issue_sync` interface with empty values + comments; `platform`, `default_branch`, `layer`, `packages`, `labels`, `log_dir`, `hosts`, optional defaults for tides stations). Adopters copy + fill.
3. **Skill** (`.claude/skills/start-deployment/SKILL.md`) — the load-bearing artifact. Frontmatter (`name` + `description`); embedded urgency contract (11 rules verbatim); procedure with three-state detection (create-new / activate-first-time / resume-ongoing) branching on side (`field_mode.sh`); steps invoke configured `issue_sync` commands rather than hard-coded `git bug`; "ensure-essentials" limited to `## Logs` section; rename title to `Deployment YYYY-MM-DD: <scope>` only if not already in that format; tides/weather/currents into the dev log's per-day `## Pre-flight` section, dev-only.
4. **No marker, no `.gitignore` edit, no hook** — explicitly out of scope per the locked design.

## Files to Change

| File | Change |
|------|--------|
| `.agent/knowledge/deployment_mode.md` | **create** — canonical reference doc |
| `.agent/templates/deployment_config.yaml` | **create** — commented sample project config |
| `.claude/skills/start-deployment/SKILL.md` | **create** — the skill (urgency contract embedded, three-state procedure) |
| `AGENTS.md` | **edit** — add small `### Deployment mode` subsection after Skill Worktree Exception, pointing at `/start-deployment` + ADR-0014 + knowledge doc (open-question 2 resolution, 2026-05-28) |
| `.github/copilot-instructions.md` | **edit** — add `/start-deployment` to the skill inventory (per [`principles_review_guide.md`](.agent/knowledge/principles_review_guide.md) Consequences Map) |
| `.agent/instructions/gemini-cli.instructions.md` | **edit** — add `/start-deployment` to the skill inventory (Consequences Map) |
| `.agent/AGENT_ONBOARDING.md` | **edit** — add `/start-deployment` to the skill inventory (Consequences Map) |
| `.agent/work-plans/issue-501/progress.md` | **edit** — `## Plan Authored`, `## Plan Review`, future `## Implementation` entries per [ADR-0013](https://github.com/rolker/ros2_agent_workspace/blob/main/docs/decisions/0013-progress-md-entry-type-vocabulary.md) |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Workspace vs. project separation | Skill is generic; all platform-specific values come from project `.agents/deployment.yaml`. Hardcodes no repo, label, path, or `git bug` mechanism. |
| Workspace improvements cascade to projects | Skill works for any project that supplies the config; BizzyBoat first, IzzyBoat/dora later. |
| Only what's needed | Marker file + hook + `.gitignore` edit dropped from earlier draft. Three files, no state. |
| Improve incrementally | v1 ships `/start-deployment` only; `/wrap-up-deployment`, `/next-deployment`, hook-based auto-injection are separate sub-issues of #495. |
| Human control / tight-by-default, relaxable | Activation is per-session and operator-driven (slash command). No global behavior change. |
| Capture decisions | ADR-0014 already captures the *what*; this PR is the *implementation*. |
| A change includes its consequences | Fast-follow PRs tracked as tasks #9 (`docs/logs/README.md` shrink) and #10 (echoboats `.agents/deployment.yaml`). Non-Claude adapter skill lists + AGENTS.md pointer landed in this PR per the Consequences Map. |
| Enforcement over documentation ([ADR-0004](https://github.com/rolker/ros2_agent_workspace/blob/main/docs/decisions/0004-enforcement-hierarchy-for-agent-compliance.md)) | v1 lands at the **instruction tier only** (SKILL.md urgency contract + knowledge doc). The behavioral half is operator-set and skill-invoked, not hook-enforced. Hook-based auto-injection was rejected in the locked design (would over-broadcast); tracked as a separate fast-follow under umbrella [#495](https://github.com/rolker/ros2_agent_workspace/issues/495). |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| [ADR-0003](https://github.com/rolker/ros2_agent_workspace/blob/main/docs/decisions/0003-workspace-infrastructure-is-project-agnostic.md) (workspace generic) | **Yes** | Skill is mechanism-agnostic; project config in project repo. |
| [ADR-0011](https://github.com/rolker/ros2_agent_workspace/blob/main/docs/decisions/0011-field-mode-for-non-github-origins.md) (field mode) | **Yes** | Side detection via `field_mode.sh`; field commits direct to default branch, dev uses worktree+PR. No hardcoded assumption. |
| [ADR-0013](https://github.com/rolker/ros2_agent_workspace/blob/main/docs/decisions/0013-progress-md-entry-type-vocabulary.md) (progress.md vocabulary) | **Yes** | `## Plan Authored` entry (this commit) + future `## Implementation` per the standard schema. |
| [ADR-0014](https://github.com/rolker/ros2_agent_workspace/blob/main/docs/decisions/0014-deployment-mode.md) (deployment mode, Draft) | **Yes** | This PR is the first concrete implementation; embeds the urgency contract verbatim in SKILL.md. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Add `/start-deployment` skill | `unh_echoboats_project11/.agents/deployment.yaml` so the skill has a BizzyBoat config to consume | **Follow-up** (task #10, separate project-repo PR) |
| Embed lifecycle ownership in the skill | `unh_echoboats_project11/docs/logs/README.md` (shrink: point at skill + workspace doc, keep only boat-specific overrides) | **Follow-up** (task #9, separate project-repo PR) |
| Ship ADR-0014's first implementation | ADR-0014 Status: Proposed → Accepted (after v1 lands + first deployment uses it) | **Future** (post-deployment) |
| AGENTS.md reference to the skill | Small `### Deployment mode` subsection after Skill Worktree Exception | **In this PR** (resolved 2026-05-28 — see Files to Change) |
| Non-Claude adapter skill lists | Add `/start-deployment` to `.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md` | **In this PR** (per Consequences Map) |

## Open Questions

- **Rename #501 title?** Drop "+ activation marker" since the marker is dropped from the design. (Cosmetic; the design comment supersedes. Plan file H1 renamed 2026-05-28.)
- **~~AGENTS.md / CLAUDE.md pointer to `/start-deployment`~~** — **Resolved 2026-05-28**: add a small `### Deployment mode` subsection to AGENTS.md after Skill Worktree Exception (pointing at `/start-deployment` + ADR-0014 + the knowledge doc) in this PR. CLAUDE.md untouched (Claude auto-discovers skills). Non-Claude adapter skill lists also updated in this PR per the Consequences Map.
- **ADR-0014 marker-language reconciliation** — ADR-0014 draft ([PR #500](https://github.com/rolker/ros2_agent_workspace/pull/500)) still describes an "operator-set marker" in Activation and lists it in the Modes table, but the locked design comment and this implementation drop the marker. Options: (a) amend ADR-0014 in *this* PR before/with the skill landing; (b) push an amendment as part of PR #500 (the ADR draft) so the two land coherently; (c) open a follow-up issue and call out the drift in the PR body. Surfaced 2026-05-28 — decision pending.

## Estimated Scope

**Single workspace PR** (this issue, ~7 files: 3 new + 4 small edits to instruction / adapter files, ~350-450 lines total — SKILL.md is the longest). Two fast-follow project-repo PRs on `unh_echoboats_project11` tracked separately.
