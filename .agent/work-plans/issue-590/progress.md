---
issue: 590
---

# Issue #590 — Flip local review specialist 5f from default-on to opt-in

## Issue Review
**Status**: complete
**When**: 2026-07-30 18:17 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #590
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Opt-in is more explicit — user must actively choose the local pass; wall-clock cost is no longer invisible |
| Enforcement over documentation | OK | No new rules; changing the default of an existing option in skill docs |
| Capture decisions, not just implementations | Watch | Rationale (hardware speed on 8GB-VRAM hardware) is captured in the issue body; implementor should carry that rationale into the `--local` flag description in SKILL.md so the why survives issue closure |
| A change includes its consequences | Action needed | Three knowledge files need updating alongside SKILL.md — see Actions below |
| Only what's needed | OK | Minimal, targeted change; no new abstraction |
| Improve incrementally | OK | Single PR, scoped to default-flip and stale-reference cleanup |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| 0006 — Shared AGENTS.md | No | No changes to AGENTS.md or adapter files; no `--no-local` references found in `.github/copilot-instructions.md` or other adapters |
| 0013 — progress.md vocabulary | Yes | This review-issue entry; no new entry type needed |

### Consequences

From the consequences map, "A framework skill" → "That framework's adapter file; regenerate skills if needed":
- Adapter files confirmed to have no direct `--no-local` references — no adapter updates needed
- No skill list change (content change only, not a new/removed skill), so no regeneration needed

References to the default-on behavior found in knowledge files (all need updating in the same PR):
- `.agent/knowledge/skill_workflows.md` — 2 references (`default-on`, `--no-local`)
- `.agent/knowledge/review_depth_classification.md` — 4 references (`--no-local`, `default-on` in two tier sections)
- `.agent/knowledge/inspiration_agent_workspace_digest.md` — 1 reference (`opt-out via --no-local`)

### Actions
- [ ] Update `.claude/skills/review-code/SKILL.md`: replace `--no-local` with `--local` in usage lines, flag docs, tier tables, section 5f, and report templates; add rationale note (hardware speed constraint) to the `--local` flag description
- [ ] Update `.agent/knowledge/review_depth_classification.md`: 4 references to `default-on` / `--no-local` need flipping to `default-off` / `--local`
- [ ] Update `.agent/knowledge/skill_workflows.md`: 2 references
- [ ] Update `.agent/knowledge/inspiration_agent_workspace_digest.md`: 1 reference
- [ ] Verify no other files reference `--no-local` in a durable artifact (grep `.claude/` `.agent/` for `no-local`)

## Plan Authored
**Status**: complete
**When**: 2026-07-30 18:20 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-590/plan.md` at `f782fdf`
**Branch**: feature/issue-590 at `f782fdf`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.

## Plan Review
**Status**: complete
**When**: 2026-07-30 18:24 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-590/plan.md` at `f782fdf`
**PR**: PR-less
**Verdict**: approve-with-suggestions

### Findings
- [ ] (must-fix) `SKILL.md:799` rationale paragraph ("The specialist is default-on despite the noise…") not enumerated; bare `default-on` — step-5 `--no-local`-only grep won't catch it — `.claude/skills/review-code/SKILL.md:799`
- [ ] (must-fix) `review_depth_classification.md:114` Deep-tier bullet ("Local Model Adversarial default-on") missing from the file's enumerated locations — file has 5 refs, not 4 — `.agent/knowledge/review_depth_classification.md:114`
- [ ] (suggestion) Broaden step-5 straggler grep to also match `default-on`/`default on` (not just `--no-local`); also covers `SKILL.md:958` bare-wording comment — `.agent/work-plans/issue-590/plan.md:55`

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-30 19:01 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-590 at `e630177`
**Mode**: pre-push
**Depth**: Standard (reason: governance/workflow-defining files — SKILL.md + .agent/knowledge/)
**Must-fix**: 0 | **Suggestions**: 0
**Round**: 1 | **Ship**: recommended — docs-only flip, fully consistent, no must-fix

### Findings
- [ ] No issues found. LGTM.

## Integrated Review
**Status**: complete
**When**: 2026-07-30 15:52 -04:00
**By**: Claude Code Agent (Claude Opus)

**PR**: #591 at `8060ea0`
**Sources**: 3 (Copilot R1 @ `8060ea0`, Local Review (Pre-Push) @ `e630177`, Plan Review @ `f782fdf`, CI rollup)
**Cross-source confirmations**: 1
**CI**: all-pass

### Findings
- [x] (cross-confirmed) Plan's Files-to-Change table still says `review_depth_classification.md` — "4 occurrences flipped", while step 2 (and the implementation) covers 5. Raised by Copilot @ `8060ea0` and, earlier, by Plan Review must-fix @ `f782fdf` (body enumeration was fixed; the table row was missed). Implementation itself is correct — all 5 references flipped. Fix: change `4` → `5` in the table row — `.agent/work-plans/issue-590/plan.md:71`
- [ ] (suggestion, Copilot) "No findings format" template's `**Local Adversarial**` line omits the `--local` token, unlike the adjacent Copilot line and unlike the standard/Light templates which use `run (<model>, --local) | skipped (<reason>, --local)`. Fix: add the flag token to both branches — `.claude/skills/review-code/SKILL.md:993`

### False positives
- (none) Both Copilot comments were verified against local code and hold.

### Verification notes
- Straggler grep over `.claude/` + `.agent/` for `no-local` / `NO_LOCAL` / `default-on`: remaining hits are all intentional — SKILL.md:56,155 document `--no-local` as a deprecated no-op; SKILL.md:536 is the Copilot specialist's own rationale; `inspiration_agent_workspace_digest.md:82,96` are historical notes; other hits are prior-issue work plans/progress files.
- `review_depth_classification.md` confirmed at 5 flipped references (lines 73, 80–81, 99, 116–117, 134–136).
- CI: 9 checks, 8 success + 1 `skipped` (duplicate "Validate commit identity" job on a second workflow run); no failures.
