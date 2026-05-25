---
issue: 470
---

# Issue #470 — Workflow skills as a composable timeline: progress.md as universal log, triage-reviews as integrator, handoff via fresh-context sub-agent

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-19 01:00 +00:00
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
**When**: 2026-05-19 01:30 +00:00
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
**When**: 2026-05-19 02:00 +00:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #473 — 4 review(s) (2 fresh comments on HEAD `8a4687f`), 2 valid, 0 false positives
**CI**: all-pass (incl. Mechanism C — field-validated the must-fix #1 fix from commit `2787d23`)

### Actions
- [x] (small, #1) Add `.agent/scripts/` path prefix to `worktree_enter.sh` invocation in `review-issue/SKILL.md:180` — as written it requires the script on PATH → landed in `ce1e85a`
- [x] (small, #2) Label review-issue's step 8a bullets as `**8a.1.**` through `**8a.5.**` so the cross-references from `review-plan/SKILL.md:269, 278` (introduced in commit `3833d6a`) resolve for readers → landed in `ce1e85a`

## External Review
**Status**: complete
**When**: 2026-05-19 17:30 -04:00
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
**When**: 2026-05-19 18:45 -04:00
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
**When**: 2026-05-19 23:36 +00:00
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
**When**: 2026-05-20 00:30 +00:00
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
**When**: 2026-05-20 01:45 +00:00
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
**When**: 2026-05-20 02:20 +00:00
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

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-20 02:40 +00:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))
**Verdict**: changes-requested

**Branch**: feature/issue-470 at `b7f174e`
**Mode**: pre-push
**Depth**: Standard (reason: governance-touching files — `docs/decisions/` ADR + two `.claude/skills/*/SKILL.md`)
**Must-fix**: 1 | **Suggestions**: 2

### Findings
- [x] (must-fix) ADR-0013 line 110 "plan-file SHA" → "plan-commit SHA". Closes the cascade-blast-radius leak the R10 narrative identified — exactly what pre-push review was added to catch.
- [x] (suggestion) Documented the comparison mechanism in both snippets: workspace issue matches iff `WORKTREE_SLUG == "workspace"`; project-repo issue matches iff `WORKTREE_SLUG` equals the repo-name portion of step-1's `owner/repo`. — `.claude/skills/review-issue/SKILL.md` step 8a.2, `.claude/skills/review-plan/SKILL.md` step 6.2.
- [x] (suggestion) Added explicit `if [ -z "${WORKTREE_ROOT:-}" ]` guard and `*)` fallthrough arm setting `WORKTREE_SLUG=""` — fail-closed is now explicit, not silent.

### Notes
- Specialist coverage: governance + plan-drift + adversarial run in this thread. Static analysis skipped (markdown only, no profile). Copilot Adversarial unavailable on this runtime — root cause: nvm-managed copilot binary lives at `~/.nvm/versions/node/*/bin/copilot`, PATH-injected by `~/.bashrc`, but `.bashrc` early-returns for non-interactive shells, so Agent-tool sub-shells never load nvm. Fixed in follow-up commit: `review-code`'s probe now falls back to sourcing `nvm.sh` directly + a `~/.nvm` glob before declaring copilot unavailable. Same root-cause family as the `$AGENT_NAME` subshell caveat already documented in AGENTS.md.
- Plan adherence: no drift. R10 fixes are all within Phase A scope (Commit 1 ADR + Commit 2 SKILL.md persistence steps).
- Cross-file consistency check: `$WORKTREE_REPO` grep confirms no remaining doc references outside historical progress.md notes. `worktree_enter.sh` exports the three env vars the snippet claims it exports (verified lines 221-230).
- Cross-source pattern: the must-fix is exactly the cascade-blast-radius pattern the R10 narrative identifies. Catching it pre-push is the value the review was meant to add.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-20 03:10 +00:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))
**Verdict**: approved-with-suggestions

**Branch**: feature/issue-470 at `3b415a0`
**Mode**: pre-push
**Depth**: Standard (reason: governance-touching — ADR-0013 + 3 `.claude/skills/*/SKILL.md` files)
**Must-fix**: 0 | **Suggestions**: 1

### Findings
- [x] (suggestion) Copilot probe glob fallback now uses `printf … | sort -V | tail -1` to pick the newest installed node version. Confirmed on this machine: 3 node versions installed (v18.12.1, v24.12.0, v25.2.1); old for-loop would have picked v18 (first lexicographic), now picks v25 (newest by version sort). Active-nvm version (v24, what PATH probe returns) is unchanged; the fix only affects the fallback-arm behaviour when PATH and nvm.sh probes both fail.

### Notes
- **Copilot Adversarial activated this run** via the PATH probe arm (`COPILOT_BIN=/home/roland/.nvm/versions/node/v24.12.0/bin/copilot`). The previous round's "copilot CLI not installed" diagnosis turns out to be environment-specific — this Claude Code Bash-tool subshell already inherits the parent shell's nvm-injected PATH, so the first probe arm wins without needing the `nvm.sh` source or glob fallback. The hardening still pays off for the harness configurations where PATH doesn't carry forward (which is what the previous sub-agent hit). Verified the probe order works in this environment.
- Cross-model signal: Copilot also flagged the glob ordering issue (only finding worth keeping after silence filter). Its other two findings (`command -v` returning non-absolute paths; `WORKTREE_SLUG="workspace"` colliding with a project named `workspace`) dropped — first is moot for real binaries, second is prevented by the basename-discriminator pattern in worktree naming.
- Plan adherence: no drift. Both commits remain within Phase A scope (cascade-fix + probe hardening).
- This is the first pre-push review to actually exercise Copilot Adversarial end-to-end on #470. The previous Pre-Push entry's "Copilot Adversarial unavailable" note is what the `3b415a0` fix targets — recursion is intentional.

## External Review
**Status**: complete
**When**: 2026-05-20 03:35 +00:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #473 at `ceab6b1`
**Reviews**: 12 total from `copilot-pull-request-reviewer[bot]` (4 fresh at R11 `1d883d8`, 1 fresh at R12 `ceab6b1`), 5 valid (0 must-fix, 5 should-fix), 0 false positives.
**CI**: all-pass on Workspace Validation. `Copilot code review / Cleanup artifacts` infra failure unchanged.

### Actions
- [x] (should-fix, #1) ADR-0013:107-113 — generalized cross-source-confirmation rule. Now reads "cross-source confirmation is keyed by entry type + the entry's correlation key (per the table above)" with explicit list of each entry type's key. 3rd-order cascade from the correlation-key chain finally closed.
- [x] (should-fix, #2) AGENTS.md subshell-caveat example: literal `"Claude Code Agent"` / `"roland+claude-code@ccom.unh.edu"` replaced with `<agent name>` / `<agent email>` placeholders, followed by a sentence pointing at `set_git_identity_env.sh` and warning against copy-pasting another agent's identity.
- [x] (should-fix, #3) review-issue/SKILL.md step 8a.5: added explicit layer/package derivation block for project-repo issues, mirroring `review-plan` step 6.4 (parse `<layer>` from dir before `_ws/src/`, `<project_repo>` from leaf dir of the path step 8a.1's probe matched).
- [x] (should-fix, #4) plan.md ADR-0006 row updated: "Watch (phase C, not A)" → "Yes (small)", with explanation of the `75cc3ea` AGENTS.md edit and its single-source-of-truth propagation via SKILL.md cross-references.
- [x] (cleanup, #5) Sub-agent Pre-Push entry timestamps corrected at progress.md:199 (`2026-05-19 12:00` → `2026-05-20 02:40`) and :214 (`2026-05-19 23:10` → `2026-05-20 03:10`). Now chronologically consistent with the surrounding External Review entries.

### Notes
- **Severity trend**: must-fixes per round are R5=2, R6=1, R7=3, R8=0, R9=0, R10=0, R11=0, R12=0. The cascade-bounding strategy (option-2 pre-push review) is paying off — bug velocity has fallen to small consistency / cosmetic issues. PR functionally complete; remaining work is tidiness.
- Finding #1 is the **third** explicit reference to "plan-file SHA" / "issue number" semantics that needed updating after the R8 correlation-key-fields addition — same root issue, three locations. ADR-0013 has been the recurring source of cascades because the schema rules I added in R8 had implications I didn't trace through the whole document.
- Finding #3 is a missed-symmetric-edit: R8 fix #4 added the layer/package derivation block to `review-plan` step 6 (project-repo "create on demand" branch); the parallel block in `review-issue` step 8a.5 was never updated. The skills' parallel structure is desirable but breaks when fixes don't traverse both files.

## External Review
**Status**: complete
**When**: 2026-05-20 04:15 +00:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #473 at `3f18142`
**Reviews**: 13 total from `copilot-pull-request-reviewer[bot]` (3 fresh at HEAD), 3 valid (0 must-fix, 2 should-fix, 1 needs-discussion), 0 false positives.
**CI**: all-pass on Workspace Validation. `Copilot code review / Cleanup artifacts` infra failure unchanged.

### Actions
- [x] (should-fix, #1) ADR-0013 spelling pass complete: `generalises` → `generalizes`, `Canonicalise` → `Canonicalize`, `recognised` → `recognized`. Also swept `review-plan/SKILL.md:337` (`Initialise` → `Initialize`) and `review-code/SKILL.md:425` (`behaviour` → `behavior`) for consistency with the rest of `docs/decisions/` and the SKILL files. Historical progress.md entries left as-is — timeline artifacts shouldn't be rewritten.
- [x] (should-fix, #3) `review-code/SKILL.md` Copilot probe PATH arm: `[ -n "$COPILOT_BIN" ]` → `[ -x "$COPILOT_BIN" ]` on the first two probe arms (current PATH + nvm.sh). Filters out alias / function / builtin returns from `command -v`; only an actual executable path passes. Glob fallback already uses `[ -x ]`.
- [x] (needs-discussion → resolved, #2) Applied **Option A** (doc-only precondition): added a "Precondition: invoke from the owning repo's cwd" paragraph to `review-issue/SKILL.md` Overview, documenting the historical implicit contract and noting that cwd-independent owning-repo resolution is tracked as a follow-up. Filed [#478](https://github.com/rolker/ros2_agent_workspace/issues/478) for the cwd-independence refactor (Option C territory) to be addressed in phase B of #470 or a dedicated follow-up PR.

### Notes
- Finding #2 is the deepest yet — accumulated defensive edits across R5/R10/R11 introduced inconsistency that wasn't visible from any single edit. Bigger than R10's `$WORKTREE_REPO` bug (single var, single fix) because the right fix depends on a design decision (skill precondition vs self-resolution).
- Severity trend continues: R5=2/R6=1/R7=3 must-fixes, then R8 onward 0 must-fixes. R13 adds 1 needs-discussion finding to the trend — first design question rather than implementation bug.

## External Review
**Status**: complete
**When**: 2026-05-20 05:25 +00:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #473 at `33322cb`
**Reviews**: 14 total from `copilot-pull-request-reviewer[bot]` (5 fresh at HEAD), 5 valid (0 must-fix, 5 should-fix), 0 false positives.
**CI**: all-pass on Workspace Validation. `Copilot code review / Cleanup artifacts` infra failure unchanged.

### Actions
- [x] (should-fix, #1) `principles_review_guide.md:36`: "recognised" → "recognized". Caught R13's spelling-pass leak in the principles guide.
- [x] (should-fix, #2) `principles_review_guide.md:36`: added `## External Review` (transitional, until #470 phase B retires it) to the ADR-0013 row's enumerated entry-type list, between `## Integrated Review` and `## Implementation`.
- [x] (should-fix, #3+#4+#5) Three-skill empty-case checkbox conversion:
  - `plan-task/SKILL.md:233-234`: `No open questions — plan is review-plan-ready.` → `- [ ] No open questions — plan is review-plan-ready.`
  - `review-issue/SKILL.md:302-306`: `No actions — issue is plan-task-ready.` → `- [ ] No actions — issue is plan-task-ready.`
  - `review-plan/SKILL.md:383-384`: `Plan looks solid. Ready for implementation.` → `- [ ] Plan looks solid. Ready for implementation.`
  - Each surrounded by a sentence linking to ADR-0013's checkbox-list schema requirement. Empty sections now parse the same way as populated ones — consumers can use one code path.

### Notes
- All 5 findings are cleanup-level — no must-fix, no functional risk. #1 is a missed-replace from R13's sweep; #2 is a leftover propagation from #470's predecessor-recognition work that didn't reach the principles guide; #3/#4/#5 are a single pattern (empty-case prose instead of checkbox) across the three skills that gained progress.md persistence in this PR.
- Cross-source pattern: #3+#4+#5 is the same defect class across three sibling skill files — exactly the symmetric-edit class of bug that R8 fix #4 / R11 finding #3 already demonstrated. The fix needs to propagate across all three.
- Severity trend continues to flatten: R5=2 must-fixes → ... → R8=0 → R9=0 → R10=0 → R11=0 → R12=0 → R13=0 → R14=0. Bug velocity is now zero must-fixes and decreasing should-fix volume.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-20 01:45 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))
**Verdict**: changes-requested

**Branch**: feature/issue-470 at `bc9266f`
**Mode**: pre-push
**Depth**: Light (reason: tiny doc-only diff, no code, no ADR; cascade-pattern follow-up)
**Must-fix**: 1 | **Suggestions**: 1

### Findings
- [x] (must-fix, Claude+Copilot converged) `review-code/SKILL.md:813-814` empty-case prose converted to `- [ ] No issues found. LGTM.` checkbox + ADR-0013 schema link, matching the pattern landed in the three sibling skills. The cascade is now complete across all four skills that write progress.md entries.
- [x] (suggestion) Overclaim in the R14 External Review note ("consumers can use one code path") is now accurate after fixing must-fix #1 — review-code joins plan-task / review-issue / review-plan in the uniform checkbox-list shape. No wording change needed.

### Notes
- Cascade pattern bounded: this pre-push pass caught exactly what the user predicted — the empty-case checkbox conversion in `bc9266f` correctly propagated across `plan-task` / `review-issue` / `review-plan` but missed `review-code/SKILL.md` itself, the skill that *is* the load-bearing consumer of the schema. One more propagation hop closes it.
- Negative findings: ADR-0013 enumeration at `principles_review_guide.md:36` now matches the canonical ADR table at `0013-progress-md-entry-type-vocabulary.md:82` — no other stale enumerations found in normative docs.
- `triage-reviews/SKILL.md` `### Actions` template has no empty-case at all; pre-existing gap naturally subsumed by #470 phase B's `External Review` → `Integrated Review` migration. Out of scope here.
- Copilot Adversarial activated cleanly (PATH-discovered via nvm-installed binary, no auth issue, no glob issue). Cross-model convergence on the same finding strengthens signal.
- Minor regex observation (not a review finding, just a heads-up): the `sed -i '/^Changes$/,$d'` strip in review-code's step 5e didn't trim Copilot v1.0.49's trailing `Changes / Requests / Tokens` block in this run — block had leading blank lines + indented columns. The SKILL.md acknowledges this as a self-revealing no-op; flagging here for awareness.

## External Review
**Status**: complete
**When**: 2026-05-20 09:25 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #473 at `3adeda2`
**Reviews**: 15 total from `copilot-pull-request-reviewer[bot]` (1 fresh comment at HEAD), 1 valid (0 must-fix, 0 should-fix, 1 cleanup), 0 false positives. R1–R14 (77 comments) all addressed in prior cycles.
**CI**: all-pass on Workspace Validation. Copilot code review pipeline all-pass.

### Actions
- [x] (cleanup, #1) `progress.md:298` `**When**` reads `2026-05-20 01:45` while the preceding entry at `progress.md:275` reads `2026-05-20 05:25` — naïve append-order readers see the timestamps go backward. Root cause is mixed TZ conventions across this file: line 275 is UTC (matches Copilot's `submitted_at`), line 298 is EDT (local). Resolved via schema change rather than one-off fix — ADR-0013 `**When**` schema now requires an explicit numeric UTC offset (`<YYYY-MM-DD HH:MM ±HH:MM>`); the 5 SKILL.md templates were updated to match; all 16 `**When**` lines in this file were backfilled with the offset that was in use at write time (12× `+00:00`, 3× `-04:00`, plus my own R15 entry corrected to its real write time). Chronology now reads forward in UTC.

### Notes
- R15 has a single low-severity finding; no must-fix, no functional risk. The "near-zero must-fix volume" trend continues — R8 onward each had 0 must-fix findings.
- R1–R14 verified addressed by spot-checking: 8 prior `## External Review` entries in this file, 61 checked action items, every review's `commit_id` is reachable in linear history (no force-push complications). No reopened concerns at HEAD.
- Fix decision recorded for future similar choices: when a Copilot finding surfaces a schema-shaped bug AND the load-bearing ADR is being landed in the same PR, prefer the schema fix in the ADR over the one-off cleanup. Costs ~30 lines of edit across ADR + templates + backfill; benefits ship with the ADR itself rather than as a follow-up.
- Backfill caveat: lines 9/30/59 offsets are **inferred** rather than evidence-backed. Both UTC and EDT interpretations produce forward chronology (early-morning EDT and late-evening-prior-day UTC have the same numeric `HH:MM` values for this session), so the choice between them is ambiguous from the data alone. The other 13 `**When**` values match commit-time within a reasonable window in one interpretation but not the other, so their offsets are evidence-backed.
- Sibling `progress.md` files in other work-plans (`.agent/work-plans/issue-{452,460,461,468}/`) are not backfilled. Decision (with user, 2026-05-20): leave as historical artifacts; the ADR-0013 offset schema applies prospectively to new writes only. No follow-up issue.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-20 09:55 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))
**Verdict**: changes-requested

**Branch**: feature/issue-470 at `845ddef`
**Mode**: pre-push
**Depth**: Standard (reason: ADR-touching, doc-only, 7 files, governance-relevant)
**Must-fix**: 1 | **Suggestions**: 5

### Findings
- [x] (must-fix, Claude Adv. + Copilot Adv. cross-source) R15 Notes ¶2 overclaims certainty: "backfilled with the offset that was in use at write time" is unverifiable for lines 9/30/59 — both UTC and EDT interpretations produce forward chronology, the choice is genuinely ambiguous — `progress.md:321-322` — reworded; the original ¶2/¶3 split into three notes: (1) fix-decision rationale, (2) explicit backfill caveat naming 9/30/59 as inferred, (3) explicit sibling-scope decision with user.
- [x] (suggestion, Claude Adv.) ADR rationale para frames numeric-offset rule as prohibition of TZ abbreviations, but prior schema didn't permit abbreviations either; reword as establishing the rule — `0013-progress-md-entry-type-vocabulary.md:75-79` — landed in `2dbd49e`.
- [x] (suggestion, Claude Adv.) Sibling-`progress.md` deferral in R15 Notes ¶3 should surface as an open question to user (per `feedback_surface_scope_deferrals` pattern), not silently parked in Notes — `progress.md:333` — surfaced via `AskUserQuestion`; user answered "leave as historical, no follow-up"; decision now in Notes ¶3 with date stamp.
- [ ] (suggestion, Copilot Adv.) ADR schema silent on backfill / inferred-offset guidance — future migrations could encode uncertain timestamps as authoritative — `0013-progress-md-entry-type-vocabulary.md:65-81` — deferred (user opted not to apply in-PR; could file follow-up if pattern recurs).
- [x] (suggestion, Claude Adv.) ADR schema silent on `Z` as UTC shorthand; one-line include/exclude clarification — `0013-progress-md-entry-type-vocabulary.md:65` — landed in `2dbd49e` (accepted `Z` as `+00:00` synonym per RFC 3339).
- [x] (suggestion, Plan Drift) plan.md Estimated Scope says `+560/-15 across 7 files` but actual landed is `+1208/-16 across 10 files` after the offset refinement — `plan.md:140-149`

### Notes
- Cross-source convergence: Claude Adv. and Copilot Adv. — running with no shared context — both flagged the lines-9/30/59 backfill ambiguity. Per ADR-0013, cross-source confirmation is the strongest signal class; promoting from "suggestion" to "must-fix" on the strength of two independent reviewers reading the same diff cold.
- Plan Drift specialist confirmed no architectural drift — the 4-commit batch fits within Phase A.1 + A.2 scope. Only stale claim is the Estimated Scope line counts.
- Governance: ADR-0012 supersession rule does NOT apply — ADR-0013 is unmerged in this PR; in-PR refinement is the iteration loop until merge.
- Static analysis skipped (markdown-only diff, no linter profile applies).

## External Review
**Status**: complete
**When**: 2026-05-20 10:25 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #473 at `34033de`
**Reviews**: 16 total from `copilot-pull-request-reviewer[bot]` (2 fresh comments at HEAD, both flagging the same defect class in different files). 2 valid (0 must-fix, 2 should-fix), 0 false positives. R1–R15 (78 comments) all addressed in prior cycles.
**CI**: all-pass on Workspace Validation. Copilot code review pipeline all-pass.

### Actions
- [x] (should-fix, #1+#2) Five-skill `mkdir -p` cascade: SKILL.md files that write a new `progress.md` need `mkdir -p .agent/work-plans/issue-<N>` before the frontmatter write. Copilot flagged review-issue:278 and review-plan:339 specifically; the same gap exists in plan-task, review-code, and triage-reviews (same "Frontmatter for new files" pattern). Real defect for review-issue (runs before plan-task creates the dir); defensive-best-practice for the other 4. Propagated identically across all 5 SKILL.md files in one atomic commit — review-issue:274-282, plan-task:203-211, review-plan:337-346, review-code:777-785, triage-reviews:233-241. Pattern matches the R14 + R15 cascade shape.

### Notes
- Single-pattern finding across two files; promoting to 5-file cascade fix because the same template lives in all 5 SKILL.md and consistency beats per-file defensive-only patching.
- Bug-velocity trend continues to flatten: must-fix volume zero across R8 → R16; should-fix volume now 2 (down from 5 last round).
- Copilot is reviewing the implementation files directly now (not plan.md) — sign that the PR's last-mile review focus is converging on operational details rather than design.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-21 00:35 +00:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context) [sandbox])
**Verdict**: changes-requested

**Branch**: feature/issue-470 at `2ef0d71` (pre-fix); fix landed at `5e6035d`
**Mode**: pre-push
**Depth**: Standard (reason: ADR-touching, governance-relevant, cascade-prone — 5 SKILL.md sibling files in diff)
**Must-fix**: 1 | **Suggestions**: 0

### Findings
- [x] (must-fix) Cascade-pattern instance in progress.md itself: commit `5a69f27` (External Review for R16) spliced the new entry between the prior Pre-Push entry's `### Findings` and its `### Notes`, marooning the Pre-Push specialist remarks (Cross-source convergence, Plan Drift, Governance, Static analysis) under the wrong `## External Review` heading. Violates ADR-0013's "consume by entry-type filter" rule — a phase-B integrator filtering `## External Review` notes would pull misattributed Pre-Push specialist content, and the Pre-Push entry itself would surface as missing its Notes section. `progress.md:354-376` (pre-fix) — fixed in `5e6035d` by moving the orphan Notes block back inside the Pre-Push entry.

### Notes
- **Sandbox run** — first end-to-end test of the Docker-based agent sandbox infrastructure for #470. No SSH keys, no gh token, no `~/.claude/projects` memory mount; git push deferred to host-side `push_gateway.sh`. Cascade-pattern spot-check was the explicit task focus.
- **Cascade-pattern verification across 5 SKILL.md files** — all symmetric: `mkdir -p` block (R16 fix, all 5), `**When**` schema with `±HH:MM` (R15 fix, all 5), empty-case checkbox templates (R14 fix, 4/5 — `triage-reviews` `### Actions` empty-case explicitly deferred to phase B per existing R14 notes), entry-type headers per ADR-0013 (all 5), `-c user.name="$AGENT_NAME"` commit override pattern (all 5), lifecycle-position cross-references (all 5).
- **Must-fix is the same cascade-pattern this PR has been tracking** — symmetric edits across sibling structures (here, sibling sub-sections within a single file), missed one boundary. The bug crossed from SKILL.md sibling files to progress.md sibling entries; root cause unchanged. R8-onward zero-must-fix trend on SKILL.md cascade survives because that surface is now well-instrumented; the new surface (progress.md append placement) was not.
- **Static analysis skipped** — markdown-only diff (10 files, all `.md`), no linter profile applies. Per step 4 table: markdown is "no linter — content review only".
- **Claude Adversarial / Copilot Adversarial not dispatched** — sandbox has no `copilot` CLI binary and no GitHub credentials; Claude sub-agent dispatch deferred as a budget-conscious choice given the highly converged state of the PR (R16 zero must-fix, the explicit task framing as "spot-check the cascade pattern"). Cross-model signal unavailable; sole-source must-fix is the cascade-pattern finding above.
- **Plan Drift** — no structural drift. The orphan-notes fix is a doc-only correction within Phase A; no new files, no new entry types, no schema change. (The original ¶5 added a post-fix arithmetic claim — `+1214/-22 across 10 files` — that was fabricated: actual `git diff --shortstat origin/main` at 5e6035d is `+1285/-16` and at 408083c is `+1308/-16`, neither matches the claim. Within-file moves don't alter the cumulative-vs-base diff; the arithmetic conflated 5e6035d's in-commit `+6/-6` with the cumulative delta. Retracted by R18 inline-fix; plan.md's `+1208/-16` baseline was last accurate at R15 commit `845ddef` and has been stale since.)
- **Sandbox observations** (for the host operator): pre-commit hooks ran cleanly via the mounted `/home/roland/project11/.venv/bin/pre-commit`, but had to install hook envs on first commit (~30s extra wall time on `pre-commit-hooks`, `shellcheck-py`, `black`, `flake8`, `pylint`, `yamllint`). Cache lived only for the duration of this commit since the container's `~/.cache/pre-commit/` is not mounted from the host; second commit in this same session would re-pay the cost only if hooks change. No permission errors; no host-coupling leaks observed; the `.git`-as-file → main-tree-`.git/hooks` indirection worked correctly for the worktree.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-21 00:54 +00:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context) [sandbox])
**Verdict**: changes-requested

**Branch**: feature/issue-470 at `408083c` (pre-fix); fix landed at `9399ca4`
**Mode**: pre-push
**Depth**: Standard (reason: full PR is governance-touching — ADR + 5 SKILL.md sibling files; incremental delta is doc-only but cumulative review tier carries)

### Findings
- [x] (must-fix, Claude Adversarial) Fabricated stat claim in R17 Pre-Push entry Notes ¶5: `+1214/-22 across 10 files` arithmetic doesn't match any reachable commit — actual cumulative at 5e6035d is `+1285/-16`, at 408083c is `+1308/-16`. The agent conflated 5e6035d's in-commit `+6/-6` with the cumulative-vs-base delta; within-file moves don't change cumulative. `progress.md:399` — inline-retracted in `9399ca4`, mirroring the R16-fixes-R15 inline-correction precedent at `progress.md:349`.
- [x] (suggestion, Claude Adversarial) plan.md Estimated Scope `+1208/-16 across 10 files` last accurate at R15 commit `845ddef` (verified); has drifted by ~100 lines across 4 commits since (R15 ADR edit, R16 mkdir cascade, R16 progress entry, R16 Copilot fix). `plan.md:140` — rather than bumping again, retired the point-in-time form in `9399ca4`: rubric now says `~1.3k inserts` and directs readers to `git diff --shortstat origin/main..HEAD`. Breaks the per-round bump cascade structurally.

**Must-fix**: 1 | **Suggestions**: 1

### Notes
- **Sandbox round 2** — second end-to-end test of the Docker-based agent sandbox. Memory dir not mounted, so this agent has no recall of prior rounds; everything reconstructed from `plan.md`, `progress.md`, and the diff. Verification of R17 commits (5e6035d, 408083c) found one defect (fabricated arithmetic in ¶5) and one cascade artifact (plan.md staleness) — both addressed in `9399ca4`.
- **Prior R17 commits sound at the load-bearing level** — 5e6035d's orphan-Notes fix was correct (Notes block re-attached to the right Pre-Push entry, four specialist bullets match R15 finding labels). 408083c's entry header schema (entry-type heading, numeric UTC offset, correlation key `**Branch**`, checkbox findings) all conform to ADR-0013. The single defect was confined to ¶5's arithmetic — a fresh-context Claude Adversarial caught it on the first pass.
- **Cross-source signal unavailable this round** — Copilot CLI not installed in sandbox (probed PATH, nvm.sh, `~/.nvm` glob — same finding the R17 agent recorded). Sole adversarial signal came from a Claude general-purpose subagent dispatched via the `Agent` tool. The must-fix was sole-source; confirmation review (also Claude sub-agent) found the R18 fix clean.
- **Cascade-pattern this round** — different surface from prior rounds (SKILL.md siblings → progress.md sibling sub-sections in R17 → arithmetic claim *within* a progress.md note in R18). Root cause unchanged: claims about diffs that don't survive cold reading. The R18 cascade-break (retire plan.md's point-in-time form) addresses the *kind* of claim, not just this instance.
- **Plan Drift** — no structural drift. The fixes are doc-only corrections to existing prose within Phase A; no new files, no new entry types, no schema change. Cumulative-vs-`origin/main` at HEAD of this entry's writing is `+1313/-16 across 10 files` (will tick up slightly by this commit; refer to live `git diff --shortstat` rather than encoding the post-write number here, per the R18 cascade-break).
- **Static analysis skipped** — markdown-only diff, no linter profile applies. Per `review-code` step 4 table.
- **Sandbox observations** (round 2 update for the host operator): pre-commit hook envs from R17 commits did NOT survive into this round's container (separate `~/.cache/pre-commit/` since the host cache isn't bind-mounted — confirms R17's prediction). First commit in this round (`9399ca4`) re-paid the ~30s install cost. No SSH / gh failures because no operations attempted them — all GitHub queries were avoided in favor of local `git` commands.

## External Review
**Status**: complete
**When**: 2026-05-25 12:38 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #473 — 18 review(s) (17 stale, 1 fresh against head `58c9822`), 1 valid, 0 false positives
**CI**: all-pass

### Actions
- [x] Fix (should-fix, Copilot @ `58c9822`): `review-code/SKILL.md:485` glob-fallback uses `sort -V` (GNU coreutils extension; absent on BSD/macOS `sort` and some BusyBox builds). If this arm runs on such a host, `sort` errors → empty `candidate` → `SKIP_COPILOT=1` with reason "copilot CLI not installed" even though the binary exists — a wrongful skip of Copilot Adversarial with a misleading diagnostic. Reaches only the narrow path where copilot is absent from PATH, nvm.sh sourcing failed, but a binary exists under `~/.nvm/versions/node/*/bin/` AND `sort` lacks `-V` (plausible: macOS dev running Claude Code with an nvm-managed copilot). Fixed in `c9f2abf`: replaced `sort -V | tail -1` with a portable glob loop keeping the last executable match (no `sort` dependency); verified empty-on-no-match and last-version-on-match. The code's own comment notes version choice is "mostly theoretical" since all same-platform shims symlink to one npm-loader.js.

### Notes
- The 17 prior reviews (commits `a1d67cd2`…`2ef0d716`, 2026-05-19/20) are stale — superseded by today's (2026-05-25) full Copilot re-review against the current head. The R8→R18 must-fix chain documented above reached ~0 outstanding; the fresh re-review surfacing only this one shell-portability nit corroborates convergence.
