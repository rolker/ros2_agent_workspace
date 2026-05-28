---
issue: 501
---

# Issue #501 — v1 /start-deployment skill (deployment-mode gap 1)

## Plan Authored
**Status**: complete
**When**: 2026-05-27 23:44 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Plan**: `.agent/work-plans/issue-501/plan.md` at `bb9f40e`
**PR**: https://github.com/rolker/ros2_agent_workspace/pull/503 (`[PLAN]` prefix)
**Phases**: single

### Open questions
- [ ] Rename #501 title to drop the stale "+ activation marker" phrase (marker dropped from design)
- [x] ~~Add a one-line AGENTS.md / CLAUDE.md pointer to `/start-deployment`~~ → **Resolved 2026-05-28**: add a small `### Deployment mode` subsection to AGENTS.md after Skill Worktree Exception (pointing at `/start-deployment` + ADR-0014 + the knowledge doc) in this PR. CLAUDE.md untouched. Non-Claude adapter skill lists also updated in this PR per the Consequences Map.

## Plan Review
**Status**: complete
**When**: 2026-05-28 08:37 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context)) (in-context — author self-review, delegated to fresh-context sub-agent)

**Plan**: `.agent/work-plans/issue-501/plan.md` at `bb9f40e`
**PR**: https://github.com/rolker/ros2_agent_workspace/pull/503
**Verdict**: changes-requested

### Findings
- [x] ~~(must-fix) ADR-0014 marker-language drift~~ → **Resolved 2026-05-28**: amend ADR-0014 in PR #500 (the ADR draft) to match the locked markerless design (Activation, Modes table, negatives). PR #503 just lands the skill; no ADR edits in #503; PR body will note PR #500 amendment is the coordinated step.

## Implementation
**Status**: complete (local; not yet pushed at write-time)
**When**: 2026-05-28 09:25 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Plan**: `.agent/work-plans/issue-501/plan.md` at `2fcd2d2`
**PR**: https://github.com/rolker/ros2_agent_workspace/pull/503
**Commits** (this PR's implementation, on `feature/issue-501`):
- `d1582f7` — Add /start-deployment skill, knowledge doc, and project-config template (3 new files, 756 insertions)
- `0b9abce` — Wire /start-deployment into adapter / instruction files (AGENTS.md subsection + 3 skill-list updates)
- `e9a7e72` — /start-deployment: address pre-push review findings (4 procedural gaps in SKILL.md + tides.units cleanup)

**Coordinated commits** (separate PR):
- `05a4a42` on PR [#500](https://github.com/rolker/ros2_agent_workspace/pull/500) — ADR-0014 drops marker language to match the locked markerless design (Activation, Modes table, negatives). Pushed 2026-05-28; merge order is `#500 → #503` so the ADR text is coherent when the skill lands.

### Outcome vs plan
All 7 files from the amended plan landed:
- `.agent/knowledge/deployment_mode.md` — canonical reference (~250 lines)
- `.agent/templates/deployment_config.yaml` — commented sample config
- `.claude/skills/start-deployment/SKILL.md` — the skill (urgency contract embedded; 3-state detection; dev/field branching)
- `AGENTS.md` — `### Deployment mode` subsection after Skill Worktree Exception
- `.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md` — `start-deployment` appended to each skill inventory

### Reviews
- [x] Plan review (`949819c`) — verdict: changes-requested. All 5 findings addressed in plan amendments (commits `b909c6f`, `2fcd2d2`).
- [x] Pre-push fresh-context review (sub-agent, 2026-05-28 09:18) — verdict: 4 should-address procedural gaps + 4 polish suggestions. All 4 should-address items + 2 polish wins folded into `e9a7e72`. Suggestion 7 (adapter ordering) and suggestion 8 (AGENTS.md placement) intentionally left as-is per plan resolution.

### Open follow-ups (not in this PR)
- Fast-follow PRs on `unh_echoboats_project11`:
  - `.agents/deployment.yaml` — BizzyBoat config populating the schema
  - `docs/logs/README.md` — shrink to BizzyBoat-specific overrides; point at workspace docs + skill
- ADR-0014 status flip from Proposed → Accepted after the first deployment uses `/start-deployment` (post-deployment task).

## Integrated Review
**Status**: complete
**When**: 2026-05-28 09:35 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #503 at `7174388`
**Sources**: 4 (Copilot R1 @ `2985882`, Copilot R2 @ `2fcd2d2`, Copilot R3 @ `7174388`, Plan Review @ `bb9f40e`)
**Cross-source confirmations**: 0 at the same head SHA. Substantive overlap (not strict cross-confirm): the Plan Review's H1-rename finding and Copilot R1 C1 + R3 C1 all surface the same stale "+ activation marker" phrase — flagged at different correlation keys (plan SHA vs head SHA vs progress.md H1), but the issue-rename unblocks them all.
**CI**: all-pass (Lint, Validate Documentation, commit identity)

### Findings
- [x] (must-fix, Copilot R2) Broken relative link to `principles_review_guide.md` — `plan.md:30` **(resolved in `b41adbd`)**
- [x] (must-fix, Copilot R3) `progress.md` H1 still has "+ activation marker" — `progress.md:5` **(resolved in `b41adbd` — issue renamed + H1 synced)**
- [x] (must-fix, Copilot R3) Vestigial schema-comment in template — `deployment_config.yaml:115` **(resolved in `b41adbd`)**
- [x] (must-fix, Copilot R3) "(Claude Code)" parenthetical — `AGENTS.md:238` **(addressed in `b41adbd`; new R4 finding C3 introduced by the fix — see R4 entry below)**
- [x] (must-fix, Copilot R3) `<workspace_root>` placeholder undefined — `SKILL.md:143` **(resolved in `b41adbd` — discovery defined)**
- [x] (must-fix, Copilot R3) Step 4b log-init ordering — `SKILL.md:235-246` **(resolved in `b41adbd` — reordered)**
- [ ] (suggestion, Copilot R2) Plan Review entry's `plan.md:<line>` refs stale — `progress.md:67` (intentional skip — archival)

### Addressed by subsequent commits (no action needed)
- (Copilot R1, `plan.md:8`) Plan H1 stale phrase — renamed in `b909c6f`
- (Copilot R1, `plan.md:13`) Plan-only PR vs "ships first lifecycle tool" framing — PR is no longer plan-only; implementation landed in `d1582f7`/`0b9abce`/`e9a7e72`
- (Copilot R2, `progress.md:18`) "Adapter skill-list updates" ambiguous tense — implementation landed; past tense is now accurate

### False positives
None.
- [ ] (must-fix) Non-Claude adapter skill lists missing from Consequences — `principles_review_guide.md` Consequences Map requires updating `.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`, and `.agent/AGENT_ONBOARDING.md` skill inventories when a workflow skill is added. Add to Files to Change for this PR; don't defer. — `plan.md:23-28` (Files to Change), `plan.md:53-58` (Consequences)
- [ ] (suggestion) ADR-0004 / "Enforcement over documentation" not in Principles Self-Check — v1 lands at instruction tier only (SKILL.md + knowledge doc), which is defensible but should be stated. Add a row noting hook-based auto-injection is tracked as fast-follow under #495. — `plan.md:30-39`
- [ ] (suggestion) Plan file H1 still reads "+ activation marker" — rename to drop the stale phrase; the issue rename is the operator's call but the plan file's title is the author's. — `plan.md:1`
- [ ] (suggestion) `progress.md` Plan Authored / Plan Review entries not listed in Files to Change — ADR-0013 makes them required artifacts; minor under-statement of the table. — `plan.md:23-28`
- [ ] (process) Surface the AGENTS.md/CLAUDE.md pointer Open Question to the operator before implementation — per `feedback_surface_ux_decisions.md` ask, don't defer in the plan body.

### Notes
- Scope sizing is right (3 files, ~300–400 lines, SKILL.md as load-bearing artifact).
- Plan matches the locked design comment item-for-item on substance (no marker, three-state detection, side-detection via `field_mode.sh`, pluggable `issue_sync`, dev-only tides in metres + explicit TZ, urgency contract embedded, ensure-essentials limited to `## Logs`).
- Independence caveat: the original plan author and this skill's invoking agent are the same identity (Claude Code Agent / Claude Opus 4.7 (1M context)). Evaluation was delegated to a fresh-context sub-agent to mitigate; downstream consumers should weight accordingly.

## Integrated Review
**Status**: complete
**When**: 2026-05-28 10:00 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #503 at `b41adbd`
**Sources**: 2 (Copilot R4 @ `b41adbd`, prior Integrated Review @ `7174388`)
**Cross-source confirmations**: 0
**CI**: all-pass (Lint, Validate Documentation, commit identity)

### Findings
- [ ] (must-fix, Copilot R4) Worktree scripts use CWD-relative `.agent/scripts/worktree_create.sh` paths, but step 2's workspace-root discovery means CWD may be inside a project repo where `.agent/scripts/` doesn't exist. Prefix consistently with `<workspace_root>/` — `SKILL.md:244-250`
- [ ] (must-fix, Copilot R4) "Claude-Code-native" hyphenated — repo convention is "Claude Code" (with space, no hyphen). Regression from R3 C3 fix. — `AGENTS.md:241`
- [ ] (must-fix, Copilot R4) Field-side three-state detection relies on `issue_sync.field_list_open`, but absent-`issue_sync` warning text only mentions skipping the *sync* step. Without `field_list_open` the skill literally cannot list issues on field side. Make `field_list_open` + `field_show` hard-required on field side; stop with explicit error if absent (dev side can still treat `issue_sync` as fully optional). — `SKILL.md:182-188`

### False positives
- (Copilot R4, `plan.md:30`) Claimed `../../knowledge/principles_review_guide.md` resolves under `.agent/work-plans/knowledge/`, suggested `../../../knowledge/...`. Verified by `ls` from `.agent/work-plans/issue-501/`: `../../knowledge/principles_review_guide.md` resolves to `.agent/knowledge/principles_review_guide.md` (file exists). Copilot's suggested `../../../knowledge/...` would resolve to `<repo-root>/knowledge/` (does not exist). The R3-round fix is correct; this is a directory-level miscalculation by the bot.
