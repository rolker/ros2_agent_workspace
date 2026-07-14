---
issue: 563
---

# Issue #563 — Project-repo root AGENTS.md for Copilot code review

## Issue Review
**Status**: complete
**When**: 2026-07-14 10:30 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #563
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Scope Assessment

The issue separates its work into two distinct tracks:

1. **Workspace PR**: template (`.agent/templates/project_agents_md.md`) + skill integration (`onboard-project` offers to create it; `audit-project` checks presence + Quality-Standard currency). Single PR, self-contained.
2. **Pilot rollout**: separate per-repo PRs, stacked on this issue, for five first-party repos. Correctly deferred to follow-up.

Right repo: yes — template and skill changes belong in the workspace repo; the resulting AGENTS.md files live in project repos.

Dependencies: pilot PRs depend on the template being merged first.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Explicit rationale (Copilot reads root AGENTS.md since 2026-06-18); FP baseline quantifies the pain |
| Enforcement over documentation | Watch | Template is documentation; no mechanical enforcement for adoption. Copilot benefit is indirect. `audit-project` check is the nearest thing to enforcement — make it prominent in the skill's output |
| Capture decisions, not just implementations | Action needed | Extending AGENTS.md to project-repo level is a design decision. ADR-0006 covers workspace-level only. Needs an ADR-0006 addendum (ADR-0012 allows cross-reference addendums) or a new ADR capturing: "project repos get a thin AGENTS.md that references workspace rules, never forks them" |
| A change includes its consequences | Watch | Skill integration is planned. If `.claude/skills/` files change, `make generate-skills` must run. The issue doesn't mention this step — plan-task should include it |
| Only what's needed | OK | Driven by concrete FP-baseline data (200 FP classifications across 18 repos); template is purposely thin (~40–60 lines) |
| Improve incrementally | OK | Phased: workspace infra first, then pilot repos |
| Workspace vs. project separation | OK | Template stays in workspace; AGENTS.md files live in project repos |
| Workspace improvements cascade to projects | OK | This is the explicit intent of the issue — exactly what this principle calls for |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0001 — Adopt ADRs | Yes | Decision to extend AGENTS.md pattern to project-repo level should be recorded |
| ADR-0003 — Project-agnostic workspace | OK | Template is generic; issue explicitly excludes third-party forks |
| ADR-0006 — Shared AGENTS.md | Directly relevant | Extension of this pattern. Issue correctly says "reference workspace rules, never fork them" — aligns with ADR-0006's rationale. Needs an addendum or successor |
| ADR-0012 — Cross-reference addendums | Relevant | ADR-0012 allows extending ADR-0006 via addendum instead of a full new ADR — a valid, lighter path |

### Consequences

Per the consequences map (template in `.agent/templates/` → docs that reference it + skills that use it):

- `onboard-project` skill: planned in issue — check for presence, offer to create.
- `audit-project` skill: planned in issue — check presence + Quality-Standard currency.
- If `.claude/skills/` files change: `make generate-skills` must run (CLAUDE.md § Makefile targets).
- No changes to `.agent/scripts/` or worktree scripts are implied.

### Actions
- [ ] Add an ADR (or ADR-0006 addendum via ADR-0012's cross-reference-addendum mechanism) recording the decision to extend AGENTS.md to project-repo level, so the "reference, never fork" invariant is enforceable by future reviewers
- [ ] Ensure plan-task includes `make generate-skills` if any `.claude/skills/` SKILL.md files are modified
- [ ] Consider whether `udp_bridge` (28 FPs) and `marine_tools` (19 FPs) — both above some pilot-list repos — warrant inclusion in the first pilot wave; issue currently stops at 5 repos
- [ ] Template should include standalone context about workspace-in-context vs. standalone use (same as `.agent/templates/project_agents_guide.md` does) so Copilot reviewers on project repos get useful context even without workspace access

## Plan Authored
**Status**: complete
**When**: 2026-07-14 12:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-563/plan.md` at `0216e2a`
**Branch**: feature/issue-563 at `0216e2a`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.

## Plan Review
**Status**: complete
**When**: 2026-07-14 14:47 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-563/plan.md` at `0216e2a`
**PR**: PR-less (reviewed from `feature/issue-563` worktree)
**Verdict**: changes-requested

<!-- Independence: name-match heuristic misfires here — $AGENT_NAME "Claude Code Agent"
is shared across all workspace agents, so it always matches the plan author. This is a
fresh-context, separately-dispatched, different-model (Opus vs the Sonnet author) review;
recorded as independent, not self-review. -->

### Findings
- [ ] (must-fix) ADR-0006 addendum records a *new* "reference, never fork" invariant in Status — exceeds ADR-0012's navigational scope; the project-repo extension is a substantive decision that belongs in a new thin ADR with a genuine cross-reference addendum on ADR-0006. Under-captures per "Capture decisions" (review-issue "Action needed") — `plan.md:34-36`
- [ ] (must-fix) Step 5 `make generate-skills` is a no-op: it regenerates gitignored `/make_*` slash commands from the Makefile `.PHONY` line, not from hand-crafted SKILL.md content. Remove step 5 and the "(generated)" Files-to-Change row — `plan.md:48,59`
- [ ] (suggestion) Adding a root `AGENTS.md` row to `audit-project`'s coverage table should also add it to the reference template `.agent/templates/project_governance.md` so skill and template stay in sync — `plan.md:44-46`
- [ ] (suggestion) Currency check via string-search for "nits" is brittle; have the template emit a stable marker (e.g. a `## Quality Standard` heading) and grep for that — `plan.md:46`

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-14 15:38 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-563 at `e8ffa8e`
**Mode**: pre-push
**Depth**: Deep (reason: new ADR-0017 — Deep promotion trigger; 369 lines / governance-trigger files; guidance-doc calibration #537 applied)
**Must-fix**: 1 | **Suggestions**: 4
**Round**: 1 | **Ship**: continue — one mechanical must-fix (wrong-cwd URL lookup); near-shippable once fixed

### Findings
- [ ] (must-fix) `onboard-project` fix step looks up `{WORKSPACE_REPO_URL}` via `gh repo view --json url` from the project-repo cwd, yielding the project URL not the workspace URL; add "on the workspace repo" qualifier (template line 53 has it) — `.claude/skills/onboard-project/SKILL.md:188-190`
- [ ] (suggestion) New root-`AGENTS.md` template not added to the project-guides index (footer asks to keep current) — `.agent/knowledge/README.md:29-33`
- [ ] (suggestion) "Project-Level Guidance" section could cross-link the new template (Ask First — instruction-file edit) — `AGENTS.md:409-417`
- [ ] (suggestion) Pin the template's standalone `## Quality Standard` excerpt to its workspace source to bound the drift ADR-0017 acknowledges — `.agent/templates/project_agents_md.md:15-24`
- [ ] (suggestion) Currency-check snippet references undefined `$REPO_DIR`; define it or use the skill's `<repo-name>` convention — `.claude/skills/audit-project/SKILL.md:62`
