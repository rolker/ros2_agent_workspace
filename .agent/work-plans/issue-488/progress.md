---
issue: 488
---

# Issue #488 — Add make merge-pr: merge + clean up worktree/branches + make sync (adapt from agent_workspace)

## Plan Authored
**Status**: complete
**When**: 2026-05-25 18:57 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Plan**: `.agent/work-plans/issue-488/plan.md` at `4634f0b`
**PR**: https://github.com/rolker/ros2_agent_workspace/pull/494 (`[PLAN]` prefix)
**Phases**: single

### Open questions
- [x] Project-repo PR resolution UX: default workspace + worktree-narrowed scan of layer repos with `--repo-slug`/`REPO=` override (proposed), vs. requiring `REPO=` explicitly for any non-workspace PR (simpler, less magic)? — **resolved (2026-05-25, with user)**: went further — reframed to worktree/issue-keyed resolution (cwd → `--issue`/`--repo-slug` → `--pr` escape hatch); PR# is derived from (repo, branch). See Plan Review > Notes.
- [x] Should `merge-pr` write a terminal `progress.md` entry, or stay mechanical (lifecycle entries owned by #481-C / the `## Implementation` marker)? — **resolved (2026-05-25)**: stay mechanical.

## Plan Review
**Status**: complete
**When**: 2026-05-25 19:14 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context)) — fresh-context sub-agent (dispatched by plan author; independent context)

**Plan**: `.agent/work-plans/issue-488/plan.md` at `4634f0b`
**PR**: https://github.com/rolker/ros2_agent_workspace/pull/494
**Verdict**: approve-with-suggestions

### Findings
- [x] (must-fix) Worktree-narrowing is a chicken-and-egg bug: it greps `worktree_list.sh` for `issue-<slug>-<N>` using N=**PR** number, but worktree dirs are named by **issue** number, and the issue number isn't known until after the repo is resolved + `gh pr view`. The narrowing can't work for layer PRs. — `plan.md:26-28`
- [x] (must-fix) Resolution also breaks if the issue's worktree was already removed. Both dissolve by **requiring `REPO=`/`--repo-slug` for non-workspace PRs** (resolves Open Question 1), or remote-scanning candidates. — `plan.md:26-30`
- [x] (must-fix) `/make_merge-pr` would run `make merge-pr` with no `PR=`: `generate_make_skills.sh` only parameterizes `revert-feature|agent-run|agent-shell`; others get a bare `make <target>`. Add `merge-pr` to the parameterized case (`PR=$ARGUMENTS`) + update `test_generate_make_skills.sh`. The plan treated generate-skills as a no-op. — `generate_make_skills.sh`
- [x] (suggestion) Move the field-mode guard **before** the first `gh pr view` (resolve repo locally by origin host first), else a field repo fails with a confusing "PR not found" before the clear guard message. — `plan.md:31-33`
- [x] (suggestion) `cd "$ROOT_DIR"` in the script's own shell before `worktree_remove` AND before `make sync` (worktree_remove refuses if cwd is inside the target; sync must run from root). — `plan.md:37`
- [x] (suggestion) `worktree_remove.sh` does NOT delete layer branches (only prunes refs + prints a note), so `merge_pr.sh` owns local+remote branch deletion in the **project** repo. Step 7 is load-bearing — implement it pointed at `layers/main/.../src/<slug>`. — `plan.md:40-41`
- [x] (suggestion) Note upstream deps intentionally NOT carried over: `_issue_helpers.sh` and `update_roadmap.sh` don't exist here — don't copy the `source`/call lines. — `plan.md`
- [x] (suggestion) Human-control note: `/make_merge-pr` exposes merge+branch-delete as one-shot, but the generator's default template sets `disable-model-invocation: true` (human-only). Record this posture + preserve it in the parameterized case. — `plan.md:72`

### Notes
- Reviewer's recommended resolution: answer Open Question 1 with "require `REPO=`/`--repo-slug` for non-workspace PRs" — dissolves the two must-fix resolution findings with less magic. Flow shape (resolve → field-guard → CI-wait → merge → remove worktree → delete branches → make sync) and the adapt-don't-port divergences are otherwise sound.
- **Resolution (2026-05-25, with user)**: went further than "require `REPO=`" — reframed to **worktree/issue-keyed** resolution (cwd zero-arg → `ISSUE=`/`REPO=` → `PR=` escape hatch), since most PRs in a mature workspace are project PRs and the PR number is the ambiguous thing; PR# is now derived from (repo, branch). All 8 findings addressed in the plan amendment this round (field-guard reordered before `gh`; `generate_make_skills.sh` + test added with `ISSUE=$ARGUMENTS`/`disable-model-invocation` preserved; cd-to-root, layer-branch-deletion ownership, dropped upstream deps, human-control posture). OQ2: merge-pr stays mechanical.

## Implementation
**Status**: complete
**When**: 2026-05-25 23:43 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Branch**: feature/issue-488 at `9d84297` (local, pre-push; PR #494)
**Commits**: `9d84297` (merge_pr.sh + tests + Makefile target + generator + AGENTS.md)

### Findings
- [x] Plan implemented: `merge_pr.sh` (worktree/issue-keyed: cwd → `--issue`/`--repo-slug` → `--pr`; field-mode guard before `gh`; CI-wait/`--no-wait`; `--merge`; `cd $ROOT` then `worktree_remove`; branch deletion on the **main checkout** via `BRANCH_REPO`, not the removed worktree; `make sync`).
- [x] Makefile `merge-pr` target (+ `.PHONY` + help); `generate_make_skills.sh` parameterized case (`/make_merge-pr <N>` → `make merge-pr ISSUE=<N>`, `disable-model-invocation: true`); `test_generate_make_skills.sh` covers it; AGENTS.md script-table row.
- [x] `test_merge_pr.sh` (6 tests, resolution/guard dispatch) + `test_generate_make_skills.sh` pass; shellcheck clean; generated `/make_merge-pr` verified.
- [x] Self-caught during review: branch deletion targeted the worktree (gone after removal) — fixed to use `BRANCH_REPO` (main checkout). shellcheck SC2066 single-item-for fixed.
- [ ] Next: review-code (pre-push) before pushing, per local-first.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-26 00:09 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context)) — fresh-context sub-agent (dispatched by author; independent context)
**Verdict**: approve-with-suggestions

**Branch**: feature/issue-488 at `5fdb48e` (local, pre-push)
**Sources**: 1 (fresh-context review-code sub-agent)
**Cross-source confirmations**: 0
**CI**: n/a (pre-push)

### Findings
- [x] (fixed `5fdb48e`, must-fix) `find … | head -n1` command-subs (escape-hatch REPO_PATH, BRANCH_REPO, repo_path_in_worktree) could abort under `set -eo pipefail` (unreadable subdir → find rc=1, or SIGPIPE) mid-resolution for layer PRs — a destructive tool aborting half-done. Switched to `find … -print -quit … || true`. — `merge_pr.sh`
- [x] (fixed `5fdb48e`, suggestion) cwd-mode error from a layer-worktree *root* was misleading; added a hint to cd into `<layer>_ws/src/<repo>`. — `merge_pr.sh`
- [x] (fixed `5fdb48e`, suggestion) added a test for the non-feature-branch cwd path (7 tests). — `test_merge_pr.sh`

### Notes
- Reviewer held a high bar (destructive tool): resolution across all 3 modes, destructive-sequence ordering, partial-failure recovery, gh usage, and the generator change all verified correct; both suites pass; shellcheck clean. The one must-fix was a latent set-e/find robustness gap, now closed.

## Integrated Review
**Status**: complete
**When**: 2026-05-26 00:21 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #494 at `ea9b7dd`
**Sources**: 2 (Copilot @ `ea9b7dd`; prior `## Local Review (Pre-Push)` @ `5fdb48e`, all findings resolved)
**Cross-source confirmations**: 0
**CI**: all-pass

### Findings
- [ ] (must-fix/safety, Copilot @ `ea9b7dd`) Always passes `--force` to `worktree_remove.sh` — bypasses the uncommitted-changes guard and can destroy unpushed work. Drop `--force`; if removal fails (dirty worktree), **abort before branch-deletion/sync** (don't leave partial cleanup). worktree_remove without `--force` is non-interactive (refuses-on-dirty, no prompt) — verified. — `merge_pr.sh:213`
- [ ] (must-fix/safety, Copilot @ `ea9b7dd`) `--pr` mode silently falls back to the workspace repo when the `--repo-slug` dir isn't found under `layers/main` — a misspelled slug could merge/delete in the WRONG repo. Treat unknown slug as a usage error; require explicit `--repo-slug workspace` for the workspace. — `merge_pr.sh:101`
- [ ] (valid, Copilot @ `ea9b7dd`) `--issue <N>` without `--repo-slug` only checks `.workspace-worktrees/issue-workspace-<N>`; layer worktrees (`layers/worktrees/issue-<slug>-<N>`) aren't scanned, so the documented "optional --repo-slug" doesn't work for layer issues. Scan both, resolve when unambiguous, error on collision (mirror worktree_remove). — `merge_pr.sh:114`
- [ ] (valid, Copilot @ `ea9b7dd`) Arg parsing assumes `$2` exists for `--issue`/`--repo-slug`/`--pr`; a trailing `--issue` makes `shift 2` error confusingly under set -e. Add `[[ $# -lt 2 ]]` missing-value validation (exit 2), like worktree_remove. — `merge_pr.sh:58`

### False positives
- (none)

### Notes
- First review of the implementation (the 2 earlier Copilot reviews were on plan-only commits). The pre-push fresh-context review-code (@ `5fdb48e`) caught a different must-fix (set-e/find) and missed ALL FOUR of these — confirming again that fresh-context review and Copilot have non-overlapping blind spots. **Lesson refinement: for a destructive tool (merges + branch/worktree deletion), the Copilot round genuinely earns its keep** — unlike the doc-grade churn on #486, two of these are data-safety/wrong-repo bugs. Fix all four before merge.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-26 00:49 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context)) — fresh-context sub-agent (dispatched by author; independent context)
**Verdict**: approve-with-suggestions

**Branch**: feature/issue-488 at `d210109` (local, pre-push)
**Sources**: 1 (fresh-context review-code sub-agent, reviewing the round's 4 Copilot fixes)
**Cross-source confirmations**: 0

### Findings
- [x] (fixed `af3997b`) The 4 Copilot Integrated-Review findings (no-force/abort, --pr unknown-slug error, --issue scan-both, arg-value validation) — verified correct by the sub-agent.
- [x] (fixed `d210109`, latent) WT_DIR existence-gate reconstructed the worktree dir from the local slug, but worktree_create/remove **sanitize** it (`my-pkg → issue-my_pkg-N`); a hyphenated repo dir would skip removal while branches still got deleted. Replaced with a HAVE_WORKTREE flag (no reconstruction). Not triggerable today (no hyphenated project-repo dirs) but closed proactively for a destructive tool.
- [x] (fixed `d210109`, cosmetic) Quoted the slug in the cleanup-guidance echo.

### Notes
- This round's review specifically guarded the *destructive* surface: removal-vs-skip gating, abort-before-branch-delete on dirty, no wrong-repo merge. worktree_remove confirmed non-interactive + exits non-zero on dirty (so the abort fires). Pushing now and **waiting for Copilot** — not merging; for a tool that merges + deletes, every review layer stays in the loop.

## Integrated Review
**Status**: complete
**When**: 2026-05-27 23:41 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #494 at `9acd658`
**Sources**: 2 (Copilot R4 @ `9acd658`, prior local timeline @ `5fdb48e`/`ea9b7dd`/`d210109`)
**Cross-source confirmations**: 0 (R3's 4 cross-confirmed findings already fixed in `af3997b`/`d210109`)
**CI**: all-pass

### Findings
- [ ] (must-fix/safety, Copilot R4 @ `9acd658`) `worktree_remove.sh --issue $N` without `--repo-slug` searches `layers/worktrees` **first**; when a workspace worktree resolves with `REPO_SLUG=""`, the `${REPO_SLUG:+--repo-slug "$REPO_SLUG"}` expansion drops the slug, so a colliding layer worktree at the same issue number gets removed instead of the workspace one. Real risk — issue-number collisions across workspace/layer trees are routine in this workspace. Fix: replace with `--repo-slug "${REPO_SLUG:-workspace}"` (and mirror in the `slug_hint=` cleanup string). `worktree_remove.sh` already handles `--repo-slug workspace` correctly. — `merge_pr.sh:252`
- [ ] (cosmetic, Copilot R2 @ `5a3da9a`) `### Open questions` checkboxes left `[ ]` even though both are resolved in the Plan Review > Notes immediately below; ADR-0013 treats this section as a pre-implementation decision gate, so unchecked boxes can mislead future readers/skills. Fix: tick both checkboxes (optionally inline the resolution). — `progress.md:17-18`

### False positives
- (Copilot R1 @ `4634f0b`, on `plan.md:30`) Branch-name regex too narrow — implementation's `issue_from_branch()` already uses case-insensitive `[iI][sS][sS][uU][eE]-([0-9]+).*` and matches `feature/ISSUE-<N>-<desc>` too.
- (Copilot R1 @ `4634f0b`, on `plan.md:41`) Cleanup should use `gh pr view --json headRefName` — implementation uses the actual `BRANCH` from `git branch --show-current` or `gh pr view ... headRefName`, never reconstructs `feature/issue-<N>`.
- (Copilot R3 @ `ea9b7dd`, 4 findings on `merge_pr.sh`) Arg-value validation / unknown-slug fallback / `--issue` scan-both / always `--force` — all fixed in `af3997b` and verified in the Pre-Push review at `d210109`.

## Integrated Review
**Status**: complete
**When**: 2026-05-28 22:44 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**PR**: #494 at `64d4b57`
**Sources**: 2 (Copilot R5 @ `64d4b57`; prior local timeline @ `5fdb48e`/`ea9b7dd`/`d210109`/`9acd658`)
**Cross-source confirmations**: 0
**CI**: all-pass

### Findings
- [x] (valid/data-safety, Copilot R5 @ `64d4b57`) `repo_path_in_worktree()` picks the *first* `.git` via `find … -print -quit`; a multi-repo layer worktree makes the inner-repo pick non-deterministic → could merge/delete a branch in the wrong project repo. Does NOT affect this PR (workspace worktree returns early). **Fixed `87d8e30`** — error deterministically on >1 inner repo and point at the `--pr <N> --repo-slug <repo-dir>` escape hatch. (Slug-based inner selection rejected: `--issue/--repo-slug` keys the worktree-dir slug, which differs from the inner `src/<dir>` name — see commit body / pre-push review below.) — `merge_pr.sh:83`
- [x] (RECLASSIFIED → false positive, Copilot R5 @ `64d4b57`) `gh pr merge … --merge` lacks `--yes` → headless hang. **False positive, confirmed against gh 2.92.0**: `gh pr merge` has no `--yes` flag (errors as unknown), and `--merge` already makes it non-interactive (gh only prompts when no method flag is given). Initially mis-triaged as valid; caught at implementation by the pre-push review when the added `--yes` broke the merge. — `merge_pr.sh:236`

### False positives
- (Copilot R5 @ `64d4b57`, on `merge_pr.sh:236`) Add `--yes` for the headless path — no such flag on `gh pr merge`; `--merge` is already non-interactive. (Mis-triaged as valid above; corrected after verifying `gh pr merge --help`.)

### Notes
- Round 5. The two R4 findings are confirmed FIXED at head `64d4b57`: workspace-worktree slug-drop (`merge_pr.sh:250-257` now passes `--repo-slug "${REPO_SLUG:-workspace}"` always) and the `### Open questions` checkboxes (both now `[x]`). Of the 10 Copilot inline comments across R1–R5, 8 were on earlier commits and already triaged/fixed in prior rounds; only these 2 are live at head. Neither blocks merging *this* PR (it's a workspace worktree, so the first-`.git` pick can't misfire here), but the data-safety one should be fixed before merge since the tool is destructive and meant for broad use — consistent with the "every review layer stays in the loop" posture from R3.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-28 23:08 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context)) — fresh-context adversarial sub-agent (general-purpose), reviewing this round's R5 fix
**Verdict**: changes-requested (both fixed before push)

**Branch**: feature/issue-488 at `87d8e30` (local, pre-push)
**Mode**: pre-push
**Depth**: Light (reason: small, low-risk shell change; shellcheck-clean; tests pass)
**Must-fix**: 2 | **Suggestions**: 0

### Findings
- [x] (must-fix, fixed `87d8e30`) The R5 fix had added `--yes` to `gh pr merge`; verified against gh 2.92.0 that `--yes` is an unknown flag → the `|| { exit 1; }` guard would fire and break **every** merge in all three modes. Dropped `--yes`; `--merge` is already non-interactive. — `merge_pr.sh:262`
- [x] (must-fix, fixed `87d8e30`) The "slug disambiguates inner repo" regression test passed spuriously: `--issue 88888 --repo-slug repo_a` constructs worktree-dir name `issue-repo_a-88888`, which didn't match the fabricated `issue-faketest-88888`, so it died at "no worktree found" *before* reaching `repo_path_in_worktree` — the assert was satisfied by the wrong error. Removed it along with the slug branch (the inner-repo-selection-by-slug design was unsound: worktree-dir slug ≠ inner `src/<dir>` name). — `test_merge_pr.sh`

### Notes
- The fresh-context adversarial sub-agent caught both — neither would have been caught by re-reading my own diff (I'd written `--yes` believing the Copilot R5 finding; the sub-agent verified the flag doesn't exist). Reinforces `feedback_internal_review_before_copilot`. Net result: the data-safety fix (deterministic error on multi-repo) is sound and tested; the `--yes` finding was a false positive corrected above. Suite 9 → 10, shellcheck clean, `bash -n` clean.

## Integrated Review
**Status**: complete
**When**: 2026-05-28 23:51 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**PR**: #494 at `08e24d6`
**Sources**: 2 (Copilot R6 @ `08e24d6`; prior local timeline)
**Cross-source confirmations**: 0
**CI**: all-pass (lint, docs, commit-identity green on `08e24d6`)

### Findings
- [x] (valid, Copilot R6 @ `08e24d6`) `--issue` mode built the `issue-<slug>-<N>` worktree-dir lookup from the RAW `--repo-slug`, but `worktree_create`/`worktree_remove` name the dir with the sanitized slug (`sed 's/[^A-Za-z0-9_]/_/g'`), so `--repo-slug my-pkg` (dir `issue-my_pkg-N`) was never found. **Fixed `e23df7d`** — sanitize before lookup, mirroring `worktree_remove.sh:100-102`; scoped to the `--issue` path (the `--pr` branch matches a real `src/<dir>` name and stays raw). Latent today (no hyphenated project-repo dirs) but fixed for a destructive tool. — `merge_pr.sh:142`

### False positives
- (none this round)

### Notes
- R6 raised exactly one comment — the previous R5 data-safety fix held. Confirmed the regex is byte-identical across `merge_pr.sh`, `worktree_create.sh:613/637`, `worktree_remove.sh:102`. CI green on the head being reviewed.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-28 23:51 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context)) — fresh-context adversarial sub-agent (general-purpose), reviewing the R6 fix
**Verdict**: approved

**Branch**: feature/issue-488 at `e23df7d` (local, pre-push)
**Mode**: pre-push
**Depth**: Light (reason: one-line sanitization mirroring an audited canonical pattern + one test)
**Must-fix**: 0 | **Suggestions**: 0

### Findings
- [ ] No issues found. LGTM — verified regex identical to worktree_create/worktree_remove; `--pr` correctly stays raw; no raw-`$ARG_REPO_SLUG` reuse downstream (REPO_SLUG/remove_slug derive from the resolved repo basename); the new test is non-spurious (fails under raw lookup).

### Notes
- Ran the adversarial pass again given the R5 round caught a real bug. This round the change was clean — the prior lesson (verify against the canonical pattern) held: I checked the sed regex against worktree_create/remove before writing it. Suite 10 → 11, shellcheck clean, `bash -n` clean.

## Integrated Review
**Status**: complete
**When**: 2026-05-29 00:16 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**PR**: #494 at `6f36740`
**Sources**: 1 (Copilot R7 @ `6f36740`)
**Cross-source confirmations**: 0
**CI**: all-pass

### Findings
- [x] (valid/safety + UX, Copilot R7 @ `6f36740`, 3 comments = 1 finding + 2 doc consequences) `--pr` mode silently defaulted to the workspace repo when `--repo-slug` was omitted; since PR numbers are per-repo, a bare `--pr <N>` (or `make merge-pr PR=<N>`) for a project repo could merge/delete the wrong repo's PR #N. Surfaced to Roland as a UX/interface choice (per `feedback_surface_ux_decisions`); **decision: require `--repo-slug` in `--pr` mode** (use `workspace` literal). **Fixed `108d9bc`** — bare `--pr` exits 2; usage text (header + Usage line), `Makefile` help, and a regression test updated. Aligns with the script's stated design and `plan.md:36` (`PR=<N> REPO=<slug>`). — `merge_pr.sh:122`, `Makefile:76`

### False positives
- (none this round)

### Notes
- The 3 R7 comments (`merge_pr.sh:135`, `merge_pr.sh:62`, `Makefile:76`) collapse to one finding plus its two doc-consequences — fixed together. R7 reviewed a progress-only commit (`6f36740`), so it re-scanned the whole diff and surfaced a pre-existing `--pr` default rather than a new regression.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-29 00:16 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context)) — fresh-context adversarial sub-agent (general-purpose), reviewing the R7 fix
**Verdict**: approved

**Branch**: feature/issue-488 at `108d9bc` (local, pre-push)
**Mode**: pre-push
**Depth**: Light (reason: scoped resolution-logic change + usage/help + one test)
**Must-fix**: 0 | **Suggestions**: 0

### Findings
- [ ] No issues found. LGTM — all three `--pr` cases correct (missing→exit 2, workspace→ROOT_DIR, project→find); `make merge-pr PR=<n>` surfaces the error cleanly (no Make swallowing); cwd/`--issue` modes untouched; usage/help consistent across all four locations; new test non-spurious (would fail if the requirement were removed; exits before any `gh` call).

### Notes
- Suite 11 → 12, shellcheck clean, `bash -n` clean, `make help` line renders correctly.
