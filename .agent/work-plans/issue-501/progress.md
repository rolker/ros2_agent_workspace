---
issue: 501
---

# Issue #501 — v1 /start-deployment skill + activation marker (deployment-mode gap 1)

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
- [ ] (must-fix) ADR-0014 marker-language drift — ADR-0014 draft (PR #500) still describes "operator-set marker" in Activation and the Modes table, but the locked design (issue #501 design comment) and this plan drop the marker. Plan should either amend ADR-0014 in this PR or open a follow-up + flag it in the PR body. — `plan.md:51` (Consequences / ADR-0014 row)
- [ ] (must-fix) Non-Claude adapter skill lists missing from Consequences — `principles_review_guide.md` Consequences Map requires updating `.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`, and `.agent/AGENT_ONBOARDING.md` skill inventories when a workflow skill is added. Add to Files to Change for this PR; don't defer. — `plan.md:23-28` (Files to Change), `plan.md:53-58` (Consequences)
- [ ] (suggestion) ADR-0004 / "Enforcement over documentation" not in Principles Self-Check — v1 lands at instruction tier only (SKILL.md + knowledge doc), which is defensible but should be stated. Add a row noting hook-based auto-injection is tracked as fast-follow under #495. — `plan.md:30-39`
- [ ] (suggestion) Plan file H1 still reads "+ activation marker" — rename to drop the stale phrase; the issue rename is the operator's call but the plan file's title is the author's. — `plan.md:1`
- [ ] (suggestion) `progress.md` Plan Authored / Plan Review entries not listed in Files to Change — ADR-0013 makes them required artifacts; minor under-statement of the table. — `plan.md:23-28`
- [ ] (process) Surface the AGENTS.md/CLAUDE.md pointer Open Question to the operator before implementation — per `feedback_surface_ux_decisions.md` ask, don't defer in the plan body.

### Notes
- Scope sizing is right (3 files, ~300–400 lines, SKILL.md as load-bearing artifact).
- Plan matches the locked design comment item-for-item on substance (no marker, three-state detection, side-detection via `field_mode.sh`, pluggable `issue_sync`, dev-only tides in metres + explicit TZ, urgency contract embedded, ensure-essentials limited to `## Logs`).
- Independence caveat: the original plan author and this skill's invoking agent are the same identity (Claude Code Agent / Claude Opus 4.7 (1M context)). Evaluation was delegated to a fresh-context sub-agent to mitigate; downstream consumers should weight accordingly.
