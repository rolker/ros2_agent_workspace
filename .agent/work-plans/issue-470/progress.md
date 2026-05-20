---
issue: 470
---

# Issue #470 — Workflow skills as a composable timeline: progress.md as universal log, triage-reviews as integrator, handoff via fresh-context sub-agent

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-19 01:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))
**Verdict**: approved

**Branch**: feature/issue-470 at `c89cdfe` (fix commit incoming)
**Mode**: pre-push
**Depth**: Standard (reason: 6 files, governance-touching — new ADR + principles guide + 3 governance skills, markdown-only)
**Must-fix**: 1 (resolved in fix commit) | **Suggestions**: 2 (resolved in fix commit)

### Findings
- [x] (must-fix) Broken relative link to ADR-0013 — 3 SKILL.md files use `../../docs/...` but are 3 levels deep, need `../../../docs/...` — `review-issue/SKILL.md:147`, `plan-task/SKILL.md:191`, `review-plan/SKILL.md:249`
- [x] (suggestion) plan-task step 8 commits but doesn't push; "shows up on the draft PR" claim was wrong — added explicit `git push` after the commit — `plan-task/SKILL.md` step 8
- [x] (suggestion) review-plan `--in-context` flag referenced but undefined; replaced with behavioral detection (compare `$AGENT_NAME` to existing `## Plan Authored` entry's `**By**` field) — no new flag surface — `review-plan/SKILL.md` step 6

### Notes
- This is the first `## Local Review (Pre-Push)` entry written by review-code on a PR whose implementation directly motivated ADR-0013. The entry follows the schema the ADR canonicalizes — modest dogfooding.
- All findings came from the fresh-context Adversarial Specialist sub-agent. Governance + plan-drift specialists (run in-context by the lead reviewer) surfaced nothing additional — markdown-only governance change with no consequence gaps.
- Triage with Copilot's review left for a follow-up session per user pause request.

## External Review
**Status**: complete
**When**: 2026-05-19 01:30
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #473 — 3 review(s) from `copilot-pull-request-reviewer[bot]`, 12 valid (3 must-fix, 5 should-fix, 4 nice-to-fix), 5 false positives (addressed by pre-push fix commit `44343da`)
**CI**: all-pass

### Actions
- [x] (must-fix, #1) Add `-c user.name/user.email` overrides to the 3 new commit commands in `review-issue/SKILL.md:211`, `plan-task/SKILL.md:232`, `review-plan/SKILL.md:287` — CI-blocking via `check_pr_authors.py` (mechanism C from #468); 3-source confirmation across all reviews → landed in `2787d23` (also extended to `review-code` for consistency)
- [x] (must-fix, #2) Remove "ADR-0012-style cross-reference" option from ADR-0013 Consequences (lines 116-119); require supersession only. Update `principles_review_guide.md:49` to match — ADR-0012 §52-58 forbids substantive addendums; 2-source confirmation → landed in `4effba5`
- [x] (must-fix, #3) Resolve owning repo before `$WORKTREE_ISSUE` check in `review-issue/SKILL.md:153-157`; fix worktree-list regex to handle layer worktrees (`issue-<repo>-<N>`) + numeric boundary → landed in `8fff314`
- [x] (should-fix, #4) Rename `### Action items` → `### Actions` in `review-issue/SKILL.md:199` to match ADR-0013 §70-71 canonical vocab; 2-source confirmation → landed in `5525c42`
- [x] (should-fix, #5) Fix `progress.md:9` timestamp `2026-05-18 EOD` → `2026-05-19 01:00` per ADR-0013 schema → landed in `5525c42`
- [x] (should-fix, #6) Add locate-or-create flow to `review-plan/SKILL.md` step 6 for PR-number / file-path modes; 2-source confirmation → landed in `3833d6a`
- [x] (should-fix, #7) Fix invalid HTML comment marker on `review-plan/SKILL.md:269` (`<--` → `<!--`) → landed in `5525c42`
- [x] (should-fix, #8) Scope ADR-0013 "head SHA filter" rule to PR-correlated entry types (issue/plan entries use issue number + plan-file SHA) → landed in `5525c42`
- [x] (nice-to-fix, #9) Fix `review-issue` "no action items" condition to not drop Recommendations when Action-needed findings absent → landed in `5525c42` (folded into the `### Actions` rename — Recommendations now preserved under the same heading)
- [x] (nice-to-fix, #10) Update plan.md Estimated Scope (line 134-139) — actual landed diff is +505/-1 across 7 files → landed in `4126cb1`
- [x] (nice-to-fix, #11) Add SHA-citation bullet to `plan-task` step 9 (report should cite progress.md commit SHA per step 8) → landed in `4126cb1`
- [x] (nice-to-fix, #12) Add `## External Review` transitional row to `principles_review_guide.md:36` → landed in `4126cb1`

### False positives (addressed in `44343da`)
- ADR-0013 relative-link depth in 3 SKILL.md files — fixed
- plan-task step 8 missing `git push` — fixed
- review-plan `--in-context` flag undefined — replaced with behavioral detection
- review-issue fallback making persistence optional — addressed by worktree-on-demand pivot in `c89cdfe`
- Plan-only commit findings on `a1d67cd` — resolved by subsequent implementation

## External Review
**Status**: complete
**When**: 2026-05-19 02:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #473 — 4 review(s) (2 fresh comments on HEAD `8a4687f`), 2 valid, 0 false positives
**CI**: all-pass (incl. Mechanism C — field-validated the must-fix #1 fix from commit `2787d23`)

### Actions
- [x] (small, #1) Add `.agent/scripts/` path prefix to `worktree_enter.sh` invocation in `review-issue/SKILL.md:180` — as written it requires the script on PATH → landed in `ce1e85a`
- [x] (small, #2) Label review-issue's step 8a bullets as `**8a.1.**` through `**8a.5.**` so the cross-references from `review-plan/SKILL.md:269, 278` (introduced in commit `3833d6a`) resolve for readers → landed in `ce1e85a`

## External Review
**Status**: complete
**When**: 2026-05-19 17:30
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #473 — 5 review(s) from `copilot-pull-request-reviewer[bot]` (4 fresh comments on HEAD `5d57910`), 4 valid (2 must-fix, 2 should-fix), 0 false positives. Earlier-round comments addressed in the prior two `## External Review` entries.
**CI**: all-pass on Workspace Validation (Lint, Validate Documentation, Mechanism C). `Copilot code review / Cleanup artifacts` step failure is in Copilot's own artifact-deletion job — unrelated to PR code, not blocking.

### Actions
- [x] (must-fix, #1) Resolve ADR-0013 normative contradiction at `docs/decisions/0013-progress-md-entry-type-vocabulary.md:44-46` — added `## External Review` as a transitional row in the Decision table marked as predecessor of `## Integrated Review`, retired by phase B; Predecessor recognition section now cross-referenced from the table itself.
- [x] (must-fix, #2) Added `-c user.name="$AGENT_NAME" -c user.email="$AGENT_EMAIL"` overrides to `plan-task/SKILL.md` step 6 plan-commit example + a paragraph linking to AGENTS.md § Agent Commit Identity and noting the `check_pr_authors.py` (Mechanism C) failure mode.
- [x] (should-fix, #3) `review-issue/SKILL.md` step 8a.1 rewritten — dropped circular `--repo <owner/repo>`, now describes a 3-step probe (workspace first via cwd-based `gh issue view <N>`, then project repos under `layers/main/*/src/*`, then error). Corrected the "mirrors plan-task step 4" claim.
- [x] (should-fix, #4) `review-plan/SKILL.md` step 6 sub-step 1 rewritten with mode-split — PR-number mode reuses step-1 metadata, `--issue`/file-path mode uses the same cwd-based probe as #3. Mirror relationship now actually holds.

## External Review
**Status**: complete
**When**: 2026-05-19 18:45
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #473 — 6 review(s) from `copilot-pull-request-reviewer[bot]` (3 fresh comments on HEAD `c2cd4a6`), 3 valid (1 must-fix, 2 should-fix), 0 false positives.
**CI**: all-pass on Workspace Validation. `Copilot code review / Cleanup artifacts` continues to fail in the artifact-deletion infra job — unrelated, not blocking.

### Actions
- [x] (must-fix, #1) Added `## External Review` to ADR-0013 correlation-key table (PR/branch-head-SHA row, alongside `## Integrated Review`). Corrects oversight from commit `c2cd4a6`.
- [x] (should-fix, #2) Extended ADR-0013 schema to canonicalize `### Open questions` alongside `### Findings` / `### Actions`, with a sentence noting `### Open questions` is reserved for `## Plan Authored`'s pre-implementation decisions.
- [x] (should-fix, #3) Reworded `review-plan` self-review detection to compare `$AGENT_NAME` to the agent-name portion of `**By**` (prefix before ` (`), with explicit note about the `<agent name> (<model>)` format from ADR-0013.

### Notes
- Finding #1 is a self-inflicted regression — the previous round's ADR-0013 fix updated the Decision table but missed the correlation-key table directly below it. Cross-checked the rest of ADR-0013 for similar misses: no further omissions.
- Finding #2 is borderline — Open Questions is an established Plan-Authored pattern with different semantics than Findings/Actions. The fix extends the canonical set rather than collapsing semantics.
- Findings #3 and #4 (from the R5 round) are the same underlying defect across two skill files — fix together to maintain the "mirror" relationship the docs claim.
- All 40 inline comments on earlier commits (a1d67cd / c89cdfe / 44343da / 8a4687f) were classified in the prior two `## External Review` rounds. Cross-checked: none reopened by HEAD changes.

## Local Review
**Status**: complete
**When**: 2026-05-19 23:36
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))
**Verdict**: changes-requested

**PR**: #473 at `c0aa9d0`
**Mode**: post-PR
**Depth**: Deep (reason: substantive ADR addition + 757 lines + 5 governance-touching files)
**Must-fix**: 3 | **Suggestions**: 9

### Findings
- [x] (must-fix) `review-plan/SKILL.md` self-review detection assumes `## Plan Authored` entry exists — added explicit "if no `## Plan Authored` entry exists, omit the annotation" fallback for PR-less / pre-plan-task invocations.
- [x] (must-fix) `<plan-worktree-path>` undefined when sub-step 2's "skip to Append" branch is taken — sub-step 2 now records `<plan-worktree-path>` as `$PWD` before skipping.
- [x] (must-fix) `worktree_remove.sh` invocation missing `.agent/scripts/` prefix — added.
- [x] (suggestion) Duplicate `### Notes` headings within the R5 `## External Review` entry — merged into one.
- [x] (suggestion) Plan drift: `plan.md:89` "No changes to `review-code`" claim updated to reflect the consistency change landed in commit `2787d23` and the `-c` override extension to `triage-reviews` in this commit. Per `plan-task` "During implementation" rule 1.
- [x] (suggestion) Forward-pointer added: `triage-reviews/SKILL.md` step 7 now references ADR-0013 in the commit-pattern paragraph (also notes the phase-B rename to `## Integrated Review`).
- [ ] (suggestion) ADR-0013 schema is silent on the canonical field that carries the PR-head-SHA correlation key. Existing `## External Review` entries put SHAs in prose; a consumer filtering by entry-type + SHA can't rely on a stable field. Recommend canonicalizing `**PR**: #<N> at \`<sha>\`` in the schema — `docs/decisions/0013-progress-md-entry-type-vocabulary.md:69-77`
- [ ] (suggestion) ADR-0013 cites "§52-58 of ADR-0012" by line range — fragile to ADR-0012 edits. Cite by section heading ("Still requires a new/superseding ADR" section) instead — `docs/decisions/0013-progress-md-entry-type-vocabulary.md:134`
- [ ] (suggestion) `review-plan/SKILL.md` step 6's "create one on demand" sub-step says "mirroring `review-issue` step 8a.5" but doesn't restate (or cross-reference) `plan-task` step 4's "infer the layer and package" guidance. Agent with only an issue number has no inline path to layer/package determination — `review-plan/SKILL.md:292-297`
- [ ] (suggestion) `plan-task` step 8 push says "step 7 already pushed and opened the draft PR" — true only in the PR-create branch of step 7. The `gh pr edit` (update) branch may not push new commits; the second push is then the first push of the new commit, not "another push alongside it" — `.claude/skills/plan-task/SKILL.md:251-254`
- [ ] (suggestion) `plan-task` step 8 `Phases` field in the schema is prescribed but no example entry in `progress.md` demonstrates it — schema-vs-usage gap — `.claude/skills/plan-task/SKILL.md:227`
- [ ] (suggestion) ADR-0013 predecessor-recognition clause doesn't address in-progress PRs with existing `## External Review` entries: phase B's `triage-reviews` should update in-place or append. Resolve at phase B planning time — `docs/decisions/0013-progress-md-entry-type-vocabulary.md:99-109`

### Notes
- Independent fresh-context review per user request; did not pre-read prior `## External Review` entries in this `progress.md` before forming findings.
- 3-source convergence on the `review-plan` self-review-detection defect (Copilot R7 + Copilot Adversarial 5e + Claude Adversarial 5d) is exactly the cross-source-confirmation signal ADR-0013 is designed to surface — modest meta-validation.
- All three must-fix items were on the latest Copilot review (R7) at HEAD `c0aa9d0`. Suggestions #5-#11 are net-new findings from this review.
- CI: all-pass (Lint, Validate Documentation, Mechanism C). No blocking CI signal.
- Static analysis: no profile for `.md` (no findings reported, as documented in `review-code/SKILL.md` step 5a).

## External Review
**Status**: complete
**When**: 2026-05-20 00:30
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #473 — 8 review(s) from `copilot-pull-request-reviewer[bot]` (5 fresh comments on HEAD `bd19a5a`), 5 valid (0 must-fix, 5 should-fix), 0 false positives.
**CI**: all-pass on Workspace Validation. `Copilot code review / Cleanup artifacts` artifact-deletion infra failure unchanged, not blocking.

### Actions
- [x] (should-fix, #1) Added a required-fields table to ADR-0013 schema mapping each entry type to its canonical correlation-key field (`**Issue**: #<N>`, `**Plan**: \`<path>\` at \`<sha>\``, `**PR**: #<N> at \`<sha>\``). Consumers can now filter by entry type + correlation-key without parsing prose.
- [x] (should-fix, #2) Replaced `§52-58 of ADR-0012` with a section-heading anchor link to ADR-0012's "Still requires a new/superseding ADR" section — stable across ADR-0012 edits.
- [x] (should-fix, #3) Reworked `review-plan` `--issue`/file-path mode resolution to mirror `review-issue` 8a.1 exactly — explicit "Workspace root first → project repos → no match" probe order, with a note that agent's `cwd` can't be relied on.
- [x] (should-fix, #4) Documented `<layer>` / `<project_repo>` derivation in `review-plan` "create on demand" branch: parse from the local path that matched in step 1's probe; PR-number mode re-runs the probe to discover the path. Mirrors `plan-task` step 4's inference.
- [x] (should-fix, #5) Reworded `plan-task` step 8 push rationale: "step 7 already pushed the branch (whether it created a new draft PR or updated an existing one)" — covers both branches of step 7.
- [x] **Skill env-var subshell caveat** (sub-agent's "ergonomic gap"): added a paragraph + literal-value example to `AGENTS.md § Agent Commit Identity`. The five SKILL.md files reference this section, so the single edit propagates without per-file drift. Landed in commit `75cc3ea`.

### Notes
- **Pattern validated**: 4 of 5 R8 findings cross-source-confirm sub-agent suggestions deferred from the previous round. Exactly the cross-source-confirmation signal ADR-0013 / phase-B `triage-reviews` is designed to surface; the deferred list collapses naturally.
- Finding #3 is net-new from Copilot — the sub-agent verified my R5 review-plan fix but didn't catch the asymmetry with review-issue 8a.1. A reminder that fresh-context review and external review surface different defects.

## External Review
**Status**: complete
**When**: 2026-05-20 01:45
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #473 at `6c8cfc5`
**Reviews**: 9 total from `copilot-pull-request-reviewer[bot]`, 3 fresh comments at HEAD, 3 valid (0 must-fix, 3 should-fix), 0 false positives.
**CI**: all-pass on Workspace Validation. `Copilot code review / Cleanup artifacts` infra failure unchanged.

### Actions
- [x] (should-fix, #1) ADR-0013 Consume-by-filter table: replaced "plan-file SHA" with "plan-commit SHA — the SHA of the commit that last updated `plan.md` (not the PR head, not the file's blob SHA)". Required-fields table now uses `<plan-commit-sha>` and links to the Consume section.
- [x] (should-fix, #2) Added `**Issue**: #<N>` immediately after `**By**` in the `## Issue Review` template (`review-issue/SKILL.md`).
- [x] (should-fix, #3) `## External Review` template in `triage-reviews/SKILL.md` now uses `**PR**: #<N> at \`<short-sha>\`` on its own line; review-count summary moved to a `**Reviews**:` line below.

### Notes
- All 3 findings are cascade consequences of R8 fix #1 (added correlation-key-fields requirement to ADR-0013) — I updated the ADR's required-fields table but didn't update the 2 templates that needed new fields. Audit of all 5 templates confirms only review-issue and triage-reviews are missing fields; plan-task, review-plan, and review-code already match.
- This is the kind of cascade defect the ADR-0013 / phase-B integrator is designed to surface and prevent — a "change requires consequences" principle violation caught one round later by external review instead of pre-push.

## External Review
**Status**: complete
**When**: 2026-05-20 02:20
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #473 at `e3d4ba8`
**Reviews**: 10 total from `copilot-pull-request-reviewer[bot]`, 5 fresh at HEAD, 5 valid (0 must-fix, 5 should-fix), 0 false positives.
**CI**: all-pass on Workspace Validation. `Copilot code review / Cleanup artifacts` infra failure unchanged.

### Actions
- [x] (should-fix, #1) ADR-0013 Decision table row for `## Plan Authored`: "References the plan file SHA" → "References the plan-commit SHA". Closes the cascade left by R9 fix #1 (Consume-by-filter table only).
- [x] (should-fix, #2) Relaxed "immediately after the standard header" to "in the entry's header section (typically near the top, before any `### Findings` / `### Actions` / `### Notes` sub-sections)". Explicitly noted exact position isn't constrained — consumers locate by field name. Closes the contradiction with `## Local Review` template's `**Verdict**`-before-`**PR**` layout.
- [x] (should-fix, #3) `review-issue/SKILL.md` step 8a.2: dropped `$WORKTREE_REPO` reference; added explicit `basename "$WORKTREE_ROOT"` parsing block to derive the repo slug (`issue-workspace-<N>` → `workspace`; `issue-<repo>-<N>` → `<repo>`). Bug had been sitting since R5.
- [x] (should-fix, #4) Same `$WORKTREE_REPO` fix in `review-plan/SKILL.md` step 6.2 (cross-file consistency with #3 — derivation block mirrored).
- [x] (cleanup, #5) Restored R8 entry's `### Notes` section with the two still-relevant observations (pattern-validated cross-source confirmation; sub-agent vs Copilot complementarity). Outdated lines ("Skill env-var docs guardrail still deferred", "Still-deferred sub-agent suggestions" — all now closed) trimmed.

### Notes
- **Pattern recognition**: 3 of 5 findings are direct consequences of edits I made in earlier rounds (R8 fix #1 over-tight rule, R9 fix #1 partial cascade, R5 introduction of `$WORKTREE_REPO` that never existed). ADR/template changes have wider blast radius than I'm catching pre-push; review-code's plan-drift specialist would have caught #3+#4 if I'd ever re-run it on the recent commits.
- Still-deferred (not raised this round): `Phases` schema-vs-usage gap (defer naturally), ADR-0013 silent on in-progress-PR `## External Review` consumption (phase-B planning question).
