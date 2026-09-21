---
issue: 635
---

# Issue #635 — Sweep split by scope + commit-via-PR publish + run-over-run diff + finding tiers

## Issue Review
**Status**: complete
**When**: 2026-09-21 12:20 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #635
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Actions
- [ ] **Reconcile the Scope section against the 2026-09-18 operator comment on this issue before plan-task locks scope.** The issue body's Scope bullets ("Split the sweep report by scope: check 1 = workspace, check 2 = per project" / "Publish = commit: each part is committed... via PR") read as if both scopes get the commit-and-PR treatment now. The operator's later comment on this same issue narrows that: publish-by-commit ships for the **workspace** check only in this slice; **project-repo checks stay report-only** until the rollup shape is decided (three candidate shapes are on the table — per-repo PR, one document per project, or health-follows-the-roadmap — and the operator explicitly says "not decided"). plan-task should implement the report-side split for both scopes (workspace vs. project findings distinguished in the sweep's report), but gate the commit/PR publish path to the workspace check only; committing a `docs/health.md` into any project repo now would be unauthorized scope.
- [ ] **The "Provisional decisions with no review scheduled" row has no data source yet.** It's specified to read `## Decisions made this deployment` sections that `/wrap-up-deployment` is supposed to write, but that change is filed as [#642](https://github.com/rolker/ros2_agent_workspace/issues/642) and has not been implemented (no `## Decisions made this deployment` heading found anywhere in the repo, including the skill file). The design draft ([`docs/design/planning_document_vocabulary.md`](../../../docs/design/planning_document_vocabulary.md), Consequences section) already accepts this ordering explicitly ("a deployment wrapped up before it lands leaves no decision list behind"), so this is not a blocker — but the row's implementation must treat "no such section found in any dev log" as a graceful empty result (consistent with the workspace's "absence is never a finding" pattern used elsewhere in this same draft), never a `FAILED` state. Because #642 hasn't shipped, the exact section format is only loosely specified (decision, date made, where recorded, review owed) — a defensive/tolerant parser is warranted, and the row may need a follow-up once #642's actual output is seen.
- [ ] No other Action-needed findings — see Recommendations below for two non-blocking follow-ups.

### Recommendations
- Once #635 merges, the design draft explicitly gates filing the "roadmap-timed review pass" issue on this issue landing ("deliberately not filed until the health row it reads from exists (#635)") — worth filing that follow-up issue as part of closing out #635, since the draft names it as the next piece.
- The automated-PR commit identity (`Janitor Sweep Agent`) is #636's job (wiring the weekly cloud Routine under that identity), not #635's — #635 only builds the commit/PR *mechanism*. The plan should use the implementing agent's normal per-invocation git identity for any hand-run testing in #635's own PR, rather than attempting to stand up the `Janitor Sweep Agent` identity early.

## Plan Authored
**Status**: complete
**When**: 2026-09-21 12:19 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-635/plan.md` at `74f712c`
**Branch**: feature/issue-635 at `74f712c`
**Phases**: single

Both prior review Actions are carried into the plan: the report-side split
covers both scopes, but the commit/PR publish path (`docs/health.md`) is
gated to the workspace check only; the provisional-decisions scan is
specified as a tolerant, always-non-`FAILED`-on-absence parser pending #642.

### Open questions
- [ ] The provisional-decisions parser is built against #642's *proposed* shape with no real instance to verify against yet — needs a follow-up once #642 ships.
- [ ] Whether an aged, unscheduled provisional decision should ever escalate is explicitly open in the design draft itself — not decided by this plan.
- [ ] The project-repo health-rollup shape (per-repo PR / one doc per project / health-follows-roadmap) is the operator's open decision — project scope stays report-only pending it.

## Plan Review
**Status**: complete
**When**: 2026-09-21 12:21 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)

**Plan**: `.agent/work-plans/issue-635/plan.md` at `74f712c`
**PR**: PR-less
**Verdict**: approve-with-suggestions

### Findings
- [ ] (must-fix) **Files to Change table omits `AGENTS.md`.** The Consequences table (row: "worktree_create.sh's ALLOWED_SKILLS") explicitly says the `AGENTS.md` § Skill Worktree Exception "Allowed skills" line must be updated ("Yes — follow-up commit alongside step 6"), and the Documentation & Instruction Impact section repeats this as a required stale-doc fix. But the Files to Change table lists only `SKILL.md`, `worktree_create.sh`, and `principles_review_guide.md` — `AGENTS.md` is missing. Verified current text still reads "`research`, `inspiration-tracker`" (`AGENTS.md:263`). This is a plan-internal inconsistency that could cause the implementer to skip the one-line edit the plan itself commits to making. — `plan.md:157` (Consequences) vs. `plan.md:143-149` (Files to Change)
- [ ] (suggestion) **Step 4's "replace any existing open PR from a prior `skill/janitor-*` branch rather than stacking a new one per run" names no mechanism.** `AGENTS.md`'s skill-worktree branch convention is timestamped per invocation (`skill/{name}-{YYYYMMDD-HHMMSS-NNNNNNNNN}`), and the only existing precedent (`research`) always opens a fresh PR (`git push -u origin HEAD && gh pr create --fill`, `.claude/skills/research/SKILL.md:108-109`) — it never closes or reuses a prior one. Since this path is meant to run unattended weekly (#636), an unspecified "replace" step risks exactly the kind of PR pile-up the sweep itself would flag as drift. Worth a concrete step (e.g. `gh pr list --head 'skill/janitor-*' --state open --json number` and close the stale one, or a fixed non-timestamped branch name for this skill only) before implementation. — `plan.md:73-74`
- [ ] (suggestion) **Run-over-run diff (step 3) doesn't call out the first-run case.** `docs/health.md` has no history yet (verified: file and its git log are both absent), so the plan's `git show HEAD:docs/health.md` will fail on the very first publish this PR exercises. The intended behavior (every finding renders as "New") is inferable but not stated, and the plan should say explicitly that a missing/failed `git show` means "no prior report" rather than an error condition. — `plan.md:66-71`

### Summary
The plan correctly reconciles the issue's Scope text against the operator's 2026-09-18 narrowing (publish-by-commit for the workspace check only; project checks stay report-only; project health-rollup shape left open) — this was the round-1 review's central flag and the plan carries it through consistently across Approach, Consequences, and Open Questions. The #626 redaction gate is correctly verified as merged (PR #646) before treating the publish path as unblocked. The provisional-decisions scan is appropriately defensive against #642 not having shipped yet (tolerant parser, empty result is `OK` not `FAILED`, matches the design draft's own framing). ADR/principle self-checks are accurate against the actual files (`worktree_create.sh:387`, `principles_review_guide.md:50`). The two must-fix/suggestion items above are implementation-detail gaps (a missing file in the change table, an unspecified PR-replacement mechanism, an unstated first-run edge case) rather than scope or approach problems — the plan is ready for implementation once the `AGENTS.md` omission is fixed; the other two can reasonably be resolved during implementation.

### Recommended Actions
- [ ] Add `AGENTS.md` to the Files to Change table before implementation starts (trivial one-line fix, already scoped by the plan's own Consequences row).
- [ ] During implementation, specify the "replace existing open PR" mechanism concretely in `SKILL.md` rather than leaving it as prose intent.
- [ ] During implementation, state the first-run (`git show` failure) behavior explicitly in `SKILL.md` step 3's diff logic.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-21 12:45 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)
**Verdict**: changes-requested

**Branch**: feature/issue-635 at `9c838a0`
**Mode**: pre-push
**Depth**: Deep (reason: 650 insertions across 7 files, including two governance-trigger files — `.claude/skills/janitor-sweep/SKILL.md` and `AGENTS.md` — well over the 200-line/10-file Deep threshold)
**Must-fix**: 2 | **Suggestions**: 6
**Round**: 1 | **Ship**: continue — 2 must-fix findings on round 1 (convergence rule only recommends shipping past must-fixes at round ≥2)

### Findings
- [x] (must-fix) `SKILL.md` step 6 ("minus the run-over-run-diff subsections, which have no meaning outside this report") directly contradicts step 7 sub-step 3 ("with the `## Workspace` section's content from step 6 (tiers, `New`/`Resolved`/`Unchanged`...)") on whether New/Resolved/Unchanged subsections are committed into `docs/health.md` — `.claude/skills/janitor-sweep/SKILL.md:748-750` vs `:780-783` — fixed: decided to INCLUDE the diff subsections (they're the run-over-run record the issue asked for, and `docs/health.md`'s git history is where an operator reads it); step 6's text now says so explicitly, step 7 sub-step 3 (now 7c) already agreed
- [x] (must-fix) Step 7 sub-step 5's PR-replacement text explicitly permits closing/deleting the prior `skill/janitor-sweep-*` PR and branch *before* the new push/`gh pr create` is confirmed to succeed ("either order is fine") — if push or PR-create then fails, the previously-open published record is destroyed with no PR left open and no recovery path stated; must mandate push-and-create-new-PR-first, close-old-PR-second — `.claude/skills/janitor-sweep/SKILL.md:811-814` — fixed: reordered to push-and-open-new-PR-first (7e), close/comment/delete-old-second (7f); 7e now states the recovery path on failure (old PR stays open, local report already landed, publish reported `FAILED(workspace publish: <reason>)`)
- [x] (suggestion) The literal `gh pr comment` code block posts a placeholder URL (`<new PR URL, filled in after step 6 opens it>`) with no enforced substitution step before the call — an agent running the block verbatim posts a broken comment to a public PR; make the two-pass nature (comment now with placeholder → edit after new PR opens, or defer the whole comment) an explicit numbered sub-step rather than prose-only — `.claude/skills/janitor-sweep/SKILL.md:806, 811-814` — fixed: the reorder above eliminates the two-pass substitution entirely — `$NEW_PR_URL` is captured directly from `gh pr create`'s own stdout in 7e, and 7f's comment uses that real value with no placeholder ever written
- [x] (suggestion) `git push origin --delete "$OLD_BRANCH" || true` swallows any failure (including a real auth/permission error) under "may already be gone" — worth surfacing the actual failure reason in the report rather than fully discarding it — `.claude/skills/janitor-sweep/SKILL.md:808` — fixed: 7f now captures the failure reason into `$DELETE_ERR` and prints a note, rather than a bare `|| true`; deletion failure still doesn't abort the publish
- [x] (suggestion) No stated concurrency invariant for overlapping sweep runs once #636 wires the weekly trigger — a scheduled run and a hand-run could race on step 7 sub-step 5's list/close/delete against each other's GitHub state — worth a one-line stated assumption ("only one sweep publishes at a time") — `.claude/skills/janitor-sweep/SKILL.md` § 7 (whole section) — fixed: added a "Concurrency" paragraph at the top of § 7 stating the one-publisher-at-a-time invariant and that #636's trigger wiring must serialise runs
- [x] (suggestion) No cleanup guidance for an orphaned skill worktree/branch left behind by a failed workspace publish (`FAILED(workspace publish: ...)`) — could accumulate stale `skill/janitor-sweep-*` worktrees over repeated runs — `.claude/skills/janitor-sweep/SKILL.md:833-839` — fixed: added an "Orphaned skill worktree/branch" note after the Failure paragraph, pointing at `worktree_remove.sh --skill janitor-sweep` (verified against `_worktree_helpers.sh`'s `find_worktree_by_skill` glob pattern) plus a manual fallback
- [x] (suggestion) `$BODY_FILE` used in `gh pr create --body-file "$BODY_FILE"` with no local `mktemp`/heredoc construction shown, unlike every other command in this doc shown verbatim — `.claude/skills/janitor-sweep/SKILL.md:823` — fixed: 7e now shows the `mktemp` + heredoc construction inline before the `gh pr create` call
- [x] (suggestion) Step 7's own local sub-step numbering ("step 6" inside its 1-7 list, meaning its own sub-step 6) visually collides with the document's top-level step 6 ("Write the report") — disambiguated only by "below" in prose; worth a distinct label to survive future reordering — `.claude/skills/janitor-sweep/SKILL.md:812` (and step 7's sub-list generally) — fixed: step 7's sub-steps renamed 1-7 → 7a-7g throughout

### Specialist summary
- **Static Analysis**: no linter profile for changed `.md` files (content review only, per table); `shellcheck` unavailable on this host for the one-line `worktree_create.sh` array-addition edit — not independently re-verified beyond manual read (trivial, correct array syntax).
- **Governance**: no must-fix. All 4 Consequences-Map rows this diff touches (Skill Worktree Exception allowlist, `skill_workflows.md` table row, `principles_review_guide.md` Consequences sentence, `worktree_create.sh` `ALLOWED_SKILLS`) confirmed Done. Bot-identity/trigger out-of-scope discipline and the #609 four-state contract (kept separate from the new sixth publish state) both confirmed intact. 2 suggestions (folded above).
- **Plan Drift**: none. All 7 Approach steps implemented; the plan's own "Implementation notes" claims (AGENTS.md landed alongside `worktree_create.sh` in commit `bc5cbf0`; PR-replacement mechanism; first-run diff behavior) verified true against the actual diff and commit log. `skill_workflows.md` addition beyond the original Files-to-Change table is disclosed and justified in the plan.
- **Claude Adversarial / Lens A** (logic & correctness, Deep horizon): found the step-6/step-7 `docs/health.md` content contradiction (must-fix above) plus 3 suggestions (folded above). All other cross-references, the 8-step renumbering, and edge-case handling (first run, empty manifest, no deployment.yaml, zero prior PRs) verified consistent.
- **Claude Adversarial / Lens B** (security/concurrency/lifecycle, Deep horizon): found the unsafe PR-replacement ordering (must-fix above) plus 3 suggestions (folded above, cross-model-confirmed on the placeholder-URL point with Governance's independent finding). Redaction-scrubbing claim for `docs/health.md` verified real (not just asserted) — `redact.sh`/`REDACT_PATH_PREFIXES` wired at step 1, content lifted verbatim from the already-redacted local report.
- **Copilot Adversarial**: not run (`--copilot` not requested for this review).
- **Local Model Adversarial**: not run (`--local` not requested for this review).

### Notes
- Plan Review's round-1 must-fix (AGENTS.md missing from Files to Change) and both suggestions (concrete PR-replacement mechanism; explicit first-run diff behavior) were verified folded into the implementation — confirmed independently by Plan Drift and Governance specialists, not just by the plan's own "Implementation notes" self-report.
- Both must-fix findings here are self-contained prose/ordering fixes in `SKILL.md` — no code, no other file affected. Low likelihood of a second full round of adversarial passes surfacing new findings once these two are fixed; a lighter round-2 pass (or --skip-static self-check) would likely suffice.

## Implementation
**Status**: complete
**When**: 2026-09-21 12:50 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)

**Branch**: feature/issue-635 at `dee1c6a`
**Addressed**: Local Review (Pre-Push), round 1, changes-requested — When 2026-09-21 12:45 -04:00, SHA `9c838a0`
**Commits**: 88a446e, 84ee4d7, dee1c6a

### Actions
- [x] (must-fix) step 6/step 7 contradiction on whether New/Resolved/Unchanged is committed into `docs/health.md` — `.claude/skills/janitor-sweep/SKILL.md:748-750` — resolved: INCLUDE the diff subsections (git history is the run-over-run record); commit `88a446e`
- [x] (must-fix) PR-replacement allowed closing/deleting the old PR before the new one was confirmed — `.claude/skills/janitor-sweep/SKILL.md:811-814` — resolved: reordered to push-and-open-new-PR-first (7e), close-old-second (7f), with a stated recovery path on 7e failure; commit `84ee4d7`
- [x] (suggestion) placeholder PR-URL substitution not an explicit sub-step — resolved: eliminated by capturing `$NEW_PR_URL` from `gh pr create`'s own stdout in 7e; commit `84ee4d7`
- [x] (suggestion) `git push origin --delete ... || true` swallows failures — resolved: failure reason captured and noted, run continues; commit `84ee4d7`
- [x] (suggestion) no concurrency invariant stated — resolved: added a "Concurrency" paragraph (one-publisher-at-a-time, #636 must serialise); commit `84ee4d7`
- [x] (suggestion) no cleanup guidance for an orphaned skill worktree/branch — resolved: added guidance using verified `worktree_remove.sh --skill janitor-sweep` plus a manual fallback; commit `84ee4d7`
- [x] (suggestion) `$BODY_FILE` construction not shown — resolved: `mktemp` + heredoc shown inline in 7e; commit `84ee4d7`
- [x] (suggestion) step 7's internal sub-step numbers collide with top-level step numbers — resolved: renamed 1-7 → 7a-7g; commit `84ee4d7`

Plan sync: `.agent/work-plans/issue-635/plan.md` step 4 (Approach) and its "Implementation notes" referenced the old sub-step numbering and the unordered PR-replacement description; updated in commit `dee1c6a` to match the implemented push-new-PR-first ordering and the 7a-7g sub-step labels.

### Next step
Lifecycle: **Implementation** → **review-code** (re-review the fixes). Not dispatched by this pass per the address-findings skill's no-auto-chain rule — the host orchestrator drives the next phase.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-21 13:20 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)
**Verdict**: changes-requested

**Branch**: feature/issue-635 at `ecfea9d`
**Mode**: pre-push
**Depth**: Deep (reason: 792 insertions across 7 files, including two governance-trigger files — `.claude/skills/janitor-sweep/SKILL.md` and `AGENTS.md` — well over the Deep threshold; unchanged from round 1)
**Must-fix**: 4 | **Suggestions**: 2
**Round**: 2 | **Ship**: continue — 4 new must-fix findings, all genuine (a private-memory-filename reference, an unchecked-failure-then-destructive-delete ordering bug, an asserted-but-unenforced redaction gap, and a step-5/step-6 template inconsistency); none are mechanical one-liners, so round 2 does not meet the ≤2-and-mechanical bar for recommending ship past them

### Round-1 verification (all 8 independently confirmed fixed)
- [x] step 6/7 `docs/health.md` diff-subsection contradiction — resolved, step 5/6/7 now agree (`SKILL.md:541-544,748-754`)
- [x] unsafe PR-replacement ordering — resolved, 7e (push+create) strictly precedes 7f (close+delete), with a stated recovery path on 7e failure (`SKILL.md:819-883`)
- [x] placeholder PR-URL substitution — resolved, `$NEW_PR_URL` captured directly from `gh pr create` stdout (`SKILL.md:833-841`)
- [x] swallowed branch-delete failure — resolved, `$DELETE_ERR` captured and reported (`SKILL.md:868-874`)
- [x] no concurrency invariant — resolved, stated at top of § 7, correctly scoped to #636 as the enforcement owner (`SKILL.md:765-773`)
- [x] no orphan-worktree cleanup guidance — resolved, and the mechanism was verified to actually work: `worktree_remove.sh --skill janitor-sweep`'s `skill-*-janitor-sweep-*` glob (`_worktree_helpers.sh:141-187`) genuinely matches the `skill/janitor-sweep-<ts>-<nano>` branch 7a produces (`SKILL.md:896-914`)
- [x] `$BODY_FILE` construction not shown — resolved, `mktemp` + heredoc shown inline (`SKILL.md:826-830`)
- [x] step 7's sub-step numbers colliding with top-level steps — resolved, renamed 7a-7g with an explicit rationale (`SKILL.md:775-777`)

Also verified independently: `worktree_create.sh`'s `ALLOWED_SKILLS` and `AGENTS.md`'s Skill Worktree Exception table both now list `janitor-sweep`, consistently with each other and with `worktree_remove.sh`/`_worktree_helpers.sh`'s actual `--skill` support.

### Findings
- [x] (must-fix) `SKILL.md:820` and `plan.md:100` cite `reference_copilot_skips_draft_prs.md` as if it were a repo doc — it is a **private agent-memory filename** (matches the operator's own memory sub-file naming convention) and does not exist anywhere in this repo (verified by search). This is a direct AGENTS.md § Documentation Accuracy violation ("Never reference private agent-memory filenames from repo-tracked files... inline the relevant content instead"); a future reader (human or agent) cannot resolve the reference. Fix: replace the citation with the inlined fact (Copilot does not review draft PRs) — `.claude/skills/janitor-sweep/SKILL.md:820`, `.agent/work-plans/issue-635/plan.md:100`
- [x] (must-fix) Step 7f's `gh pr comment "$OLD_PR"` and `gh pr close "$OLD_PR"` exit statuses are unchecked before the loop proceeds to `git push origin --delete "$OLD_BRANCH"`. If `gh pr close` fails (auth hiccup, rate limit, permission), the branch is deleted anyway — leaving an **open PR pointing at a deleted head branch**, effectively stuck (GitHub does not auto-close it, and it becomes awkward to close/merge by hand). The doc surfaces the delete-failure case but not the close-failure case that should gate whether deleting is safe at all — `.claude/skills/janitor-sweep/SKILL.md:862-876`
- [x] (must-fix) The "keep the report free of host identity / absolute local paths" instruction (step 6, restated in Known limitations) is **asserted, not enforced**: the real `redact_text`/`redact_url` machinery set up in step 1 is wired only into this skill's *own* diagnostic/failure strings (`mkdir` failure, manifest errors) and is never applied to the findings text itself, which is synthesized from `audit-workspace`/`audit-project` output (e.g. `audit-workspace`'s stale-worktree check, built on unredacted `worktree_list.sh` output). Since `docs/health.md` is committed and PR'd to a **public** repo and this will eventually run **unattended** under a bot identity (#636), there is a real, unguarded path for an absolute `/home/<user>/...` path to land in a public commit — the doc tells a future unattended agent to "remember to scrub," with no grep/verification gate before `git commit`/`git push` — `.claude/skills/janitor-sweep/SKILL.md:606-614`, `927-990`
- [x] (must-fix) Step 5's diff-rendering mandate ("For **each scope** … render `New`/`Resolved`/`Unchanged` … under each tier that has any diff activity," `SKILL.md:541-544`) is not followed by the step-6 report template for the `## Projects` section: only tier 1 ("Work that can be lost") carries a diff-state tag, in a different shape (`[New/Resolved/Unchanged, or "no prior report…"]` inline per finding) than the Workspace section's subsection form. Tiers 2-5 under `## Projects` (lines 680-697) show bare `- **<repo>**: ...` with **no diff-state indication and no `(same shape)` cross-reference** — an agent following the template literally renders zero run-over-run information for 4 of 5 project-scope tiers, directly contradicting step 5. Same class of step-N/step-N+1 contradiction round 1 already found once (there, steps 6/7); this is a fresh, still-present instance between steps 5 and 6 — `.claude/skills/janitor-sweep/SKILL.md:541-544` vs `:676-697`
- [x] (suggestion) Step 5's prose says the previous `docs/health.md` state is read "from the janitor-sweep worktree **before** the new commit," but per § 7's own preamble the worktree isn't created until 7a, which runs after steps 5 and 6 — the `git show HEAD:docs/health.md` snippet at step 5 actually runs from `$ROOT` (main checkout), not the not-yet-created worktree. Functionally harmless (same answer either way, and correctly restated at 7b where the worktree does exist) but describes an out-of-sequence environment that could prompt premature worktree creation — `.claude/skills/janitor-sweep/SKILL.md:545-551` vs `:762-763`
- [x] (suggestion) The 7e `$BODY_FILE` heredoc uses unquoted `cat << EOF`, diverging from AGENTS.md's own canonical `--body-file` pattern (`cat << 'EOF'`) that this doc itself cites — no variables appear in the placeholder body today so it's not exploitable as written, but a literal copy carries the drift forward once real `$`-containing values are interpolated — `.claude/skills/janitor-sweep/SKILL.md:826-827`

### Specialist summary
- **Static Analysis**: no linter profile for changed `.md` files (content review only). `shellcheck` unavailable on this host for `worktree_create.sh`'s one-line `ALLOWED_SKILLS` array addition — manually verified correct (trivial array syntax, matches existing entries).
- **Governance**: no must-fix beyond the private-memory-filename finding above (found directly by the lead reviewer, not a specialist dispatch this round). All Consequences-Map rows this diff touches (`principles_review_guide.md`'s durable-output sentence, `skill_workflows.md`'s janitor-sweep row, `AGENTS.md`'s allowed-skills line, `worktree_create.sh`'s `ALLOWED_SKILLS`) confirmed Done and mutually consistent.
- **Plan Drift**: none. All 7 Approach steps implemented; plan's "Implementation notes" claims verified true against the diff and commit log (`88a446e`, `84ee4d7`, `dee1c6a`); plan.md itself synced in `dee1c6a` to match the round-1-fixed 7a-7g ordering.
- **Claude Adversarial / Lens A** (logic & correctness, Deep horizon, fresh subagent): independently re-verified all 8 round-1 fixes as genuinely present (not just asserted); found the step-5/step-6 Projects-tier diff-rendering inconsistency (must-fix above) plus 2 suggestions (worktree-sequencing wording, heredoc quoting style). Cross-checked script/flag/path claims (`resolve_repo_checkout.sh`, `manifest_fallback.sh`, `list_overlay_repos.py`, `get_optional_layers()`, `planning_doc_probe.sh`, `is_field_url`, `issue-triage --stale-days`, `deployment.yaml`'s `log_dir`) against real source — all matched.
- **Claude Adversarial / Lens B** (security/concurrency/lifecycle, Deep horizon, fresh subagent): independently re-verified all 8 round-1 fixes; found the 7f unchecked-`gh pr close`-before-delete ordering bug and the asserted-vs-enforced redaction gap (both must-fix above). Independently confirmed `worktree_remove.sh --skill janitor-sweep`'s glob genuinely matches the branch-naming convention `worktree_create.sh` produces.
- **Copilot Adversarial**: not run (`--copilot` not requested).
- **Local Model Adversarial**: not run (`--local` not requested).

### Notes
- The redaction-enforcement gap (must-fix 3) and the tier-rendering gap (must-fix 4) are design-shape gaps, not one-line fixes — they warrant a genuine implementation change (either wiring `redact_text` over the synthesized findings text before it's written to `docs/health.md`, or explicitly scoping what "keep it free of host identity" actually covers; and either extending the Workspace-style subsection form to all five Projects tiers or explicitly stating why tier 1 alone carries it). The private-memory-filename fix and the 7f ordering fix are small and mechanical. Given the mix, another full Deep round after these land is warranted rather than a lighter pass — recommend addressing all four, then a round-3 pre-push review before push.

### Next step
Lifecycle: **Local Review (Pre-Push)** round 2, changes-requested → **address-findings** (work the 4 must-fix + 2 suggestions above) → **review-code** round 3. Not dispatched by this pass — the host orchestrator drives the next phase. Per this skill's convergence rule, the diff should not be pushed / no PR opened until a pre-push review comes back approved (or ships past a low, non-rising, mechanical must-fix count at round ≥2 — not met here).

## Implementation
**Status**: complete
**When**: 2026-09-21 13:02 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)

**Branch**: feature/issue-635 at `62f18a3`
**Addressed**: Local Review (Pre-Push) round 2, changes-requested, When 2026-09-21 13:20 -04:00, SHA `ecfea9d`
**Commits**: 997c62c, 00628f5, e08fba9, 38cc612, acc51b6, a4efd30

### Actions
- [x] (must-fix) Private memory filename (`reference_copilot_skips_draft_prs.md`) cited from `SKILL.md:820` and `plan.md:100` — replaced both citations with the inlined fact ("Copilot code review does not review draft PRs; open it non-draft so the review fires"); verified no other occurrence of the filename remains anywhere in the repo — `997c62c`
- [x] (must-fix) Step 7f deleted the old branch unconditionally after `gh pr comment`/`gh pr close`, without checking their exit status — now `gh pr close`'s own exit status gates the delete: on failure the reason is printed and the loop `continue`s, leaving the old PR and branch untouched rather than risking an open PR pointing at a deleted head — `00628f5`
- [x] (must-fix) `redact_text` was asserted ("keep the report free of host identity") but never actually invoked on the findings text before it reached `docs/health.md` — added a mandatory sub-step inside 7c that runs `redact_text` (using the `$REDACT_PATH_PREFIXES`/`redact.sh` already sourced in step 1) over the rendered `## Workspace` section immediately before the write, with the real invocation verified against `redact.sh`'s actual signature — `e08fba9`
- [x] (must-fix) Step 6's `## Projects` template gave tiers 2-5 no diff-state indication, contradicting step 5's "for each scope" mandate (only tier 1 carried a `[New/Resolved/Unchanged]` tag) — gave tiers 2-5 the same inline diff-state tag as tier 1, added an explanatory paragraph in step 6 on why the Projects scope's rendering *shape* (inline tag) legitimately differs from the Workspace scope's (subsections) while still diffing every tier, and restated the same rationale in step 5 so both steps agree; the one deliberately-undiffed exception (the "provisional decisions" sub-list) is now stated explicitly in both places — `38cc612`
- [x] (suggestion) Step 5 said the prior `docs/health.md` state was read "from the janitor-sweep worktree" although the worktree isn't created until 7a (after steps 5/6) — reworded to say the read runs from the main checkout at this point in the run, restated correctly at 7b — `acc51b6`
- [x] (suggestion) The 7e `$BODY_FILE` heredoc used an unquoted `cat << EOF` delimiter, diverging from AGENTS.md's own `cat << 'EOF'` convention — quoted the delimiter — `acc51b6`

### Plan sync
`plan.md` updated in its own commit (`a4efd30`): dropped the private-memory-filename citation, added the mandatory `redact_text` sub-step to the publish-by-commit approach item, and restated the "every tier of every scope carries diff-state, rendering shape may differ by scope" rule (matching the `SKILL.md` fix) in the run-over-run-diff approach item.

### Next step
Lifecycle: **Implementation** → **review-code** (re-review the fixes, round 3). Not dispatched by this pass — the host orchestrator drives the next phase.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-21 13:07 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)
**Verdict**: changes-requested

**Branch**: feature/issue-635 at `5bd1e50`
**Mode**: pre-push
**Depth**: Deep (reason: 960 insertions across 7 files, including two governance-trigger files — `.claude/skills/janitor-sweep/SKILL.md` and `AGENTS.md` — well over the Deep threshold; unchanged from rounds 1-2)
**Must-fix**: 3 | **Suggestions**: 0
**Round**: 3 | **Ship**: continue — 3 new must-fix findings, all genuine design/correctness gaps (an undefined variable, a chicken-and-egg PR-URL field, a wrong-worktree-deletion risk in the documented cleanup command); none are one-line mechanical fixes, so round 3 does not meet the low-and-mechanical bar for shipping past them

### Round-2 verification (all 6 independently confirmed fixed)
- [x] private-memory-filename citation (`reference_copilot_skips_draft_prs.md`) — resolved, replaced with inlined fact at `SKILL.md:876-877` and `plan.md:122`; confirmed zero remaining occurrences of the filename anywhere in the repo
- [x] step 7f unchecked `gh pr close` before branch delete — resolved, delete is gated on `gh pr close`'s own exit status (`SKILL.md:920-935`); a close failure leaves the old PR/branch untouched
- [x] `redact_text` asserted but never invoked before the `docs/health.md` write — resolved, `WORKSPACE_SECTION=$(redact_text "$WORKSPACE_SECTION")` runs in 7c immediately before the write (`SKILL.md:850-855`); signature verified against `redact.sh`'s real `redact_text <text>` contract
- [x] Projects tiers 2-5 missing diff-state tags — resolved, all five tiers under `## Projects` now carry the `[New/Resolved/Unchanged, or "no prior report..."]` inline tag, with an explanatory paragraph on why the rendering shape legitimately differs from Workspace's subsection form (`SKILL.md:682-731`)
- [x] step 5 wrongly said the pre-publish read runs "from the janitor-sweep worktree" — resolved, reworded to say it runs from the main checkout at that point in the run (`SKILL.md:556-561`)
- [x] 7e `$BODY_FILE` heredoc used unquoted `cat << EOF` — resolved, quoted (`cat << 'EOF'`, `SKILL.md:885`)

### Findings
- [x] (must-fix) **`$WORKSPACE_SECTION`, used in 7c's `redact_text` call, is never assigned anywhere.** Step 6 only ever produces `$REPORT_BODY` (`SKILL.md:610`) — no sub-step extracts a `## Workspace`-only section into a separate variable. 7c's own comment claims it "was rendered (step 6)" but that rendering step does not exist. An agent following 7c literally hits an undefined/empty variable — `.claude/skills/janitor-sweep/SKILL.md:850-855` vs `:588-610` — **fixed in fb2d29b**: step 6 now renders `$WORKSPACE_SECTION` and composes `$HEALTH_BODY` from it; 7b writes `$HEALTH_BODY` verbatim
- [x] (must-fix) **The step-6 report template's `**Publish**: committed to `docs/health.md`, PR <url> / not published: <reason>` line (`SKILL.md:653`) cannot be filled at the time step 6 runs.** Step 6 (report authoring) completes before step 7 exists at all, and the PR URL is only known after 7e's `gh pr create` — yet step 6's closing paragraph says the whole `## Workspace` section, "including its `New`/`Resolved`/`Unchanged`... subsections," "gets lifted verbatim into `docs/health.md`" in 7c, which itself runs *before* 7e. The doc gives no second-pass/backfill instruction (rewrite the line after 7e succeeds, defer it to an amend, or drop it from the verbatim-lifted content) — an agent following the steps as written must either stall on an unfillable field or invent a differing resolution each run — `.claude/skills/janitor-sweep/SKILL.md:653` vs `:783-789` vs `:833-916` (7c precedes 7e) — **fixed in fb2d29b**: the Publish line is gone from the `## Workspace` section and from `docs/health.md`; the outcome is appended to the local report by 7h once known
- [x] (must-fix) **The orphan-cleanup command can silently delete the wrong, live worktree.** `worktree_remove.sh --skill janitor-sweep` resolves via `find_worktree_by_skill()` (`.agent/scripts/_worktree_helpers.sh:141-183`), which globs `skill-*-janitor-sweep-*` and, on more than one match, silently picks the **most recent by timestamp** (only a stderr `Warning:`, not surfaced to the caller). SKILL.md's own text says a failed run's worktree is "left in place for inspection or retry" and that "repeated failed runs would accumulate these" — so in the exact scenario the doc describes (orphan T1 left in place, agent retries, creating T2 with a later timestamp), running the one documented cleanup command deletes the **live retry T2**, not the intended orphan T1, with no warning in the doc and no branch-targeted removal path offered despite `$NEW_BRANCH` having been captured in 7a — `.claude/skills/janitor-sweep/SKILL.md:972-990` — **fixed in fb2d29b**: cleanup keys on `$WT_PATH`/`$NEW_BRANCH` captured in 7a; 7g removes the run's own worktree; 7a removes leftovers by exact path; `worktree_remove.sh --skill` is named as the wrong tool

### Specialist summary
- **Round-2-item re-verification**: done directly by the lead reviewer (not a specialist dispatch) — grep + source-read against each of the 6 fixed files/lines, including reading `redact.sh`'s actual function signature and `AGENTS.md`/`worktree_create.sh`/`skill_workflows.md`/`principles_review_guide.md` for consistency. All confirmed.
- **Claude Adversarial / Lens A** (logic & correctness, Deep horizon, fresh subagent): found the undefined `$WORKSPACE_SECTION` and the Publish-line-ordering chicken-and-egg (both must-fix above); independently re-verified all 6 round-2 fixes hold; verified `worktree_remove.sh --skill` claims and all named script/flag/path references against actual source. No suggestions.
- **Claude Adversarial / Lens B** (security/concurrency/lifecycle, Deep horizon, fresh subagent): found the wrong-worktree-deletion risk in the orphan-cleanup path (must-fix above, independently confirmed by the lead reviewer against `_worktree_helpers.sh:141-183`); traced redaction coverage of every text path reaching `docs/health.md`/PR title/body/comment and found no gap; confirmed 7e→7f ordering and close-gated delete are sound; confirmed the concurrency section is honest about being unenforced, not falsely safe. No suggestions.
- **Governance / Plan Drift**: not separately dispatched this round (unchanged consequences-map rows already confirmed Done in round 2; spot-checked again directly — `AGENTS.md`, `worktree_create.sh`, `skill_workflows.md`, `principles_review_guide.md` all still consistent with each other and with `janitor-sweep` being allowlisted).
- **Copilot Adversarial**: not run (`--copilot` not requested).
- **Local Model Adversarial**: not run (`--local` not requested).

### Notes
- All 3 new findings are in the newly-added `docs/health.md` publish machinery (step 6/§7), the same area rounds 1-2 already fixed twice — this is the fourth distinct class of bug found there across three rounds (contradiction → ordering → this round's undefined-variable/unfillable-field/wrong-deletion trio), which is a real signal that the publish section's design needs a slower pass rather than another quick patch-and-rereview cycle.
- Findings 1 and 2 are closely related (both stem from step 6 rendering `## Workspace` content, including a field it cannot yet know, before step 7 exists) but are reported separately since fixing one does not fix the other: extracting `$WORKSPACE_SECTION` correctly still leaves the Publish-line value unfillable at render time.
- Per the convergence rule, round 3 does not qualify for "recommended" — 3 must-fix, none mechanical, none a design question already settled by prior rounds.

### Next step
Lifecycle: **Local Review (Pre-Push)** round 3, changes-requested → **address-findings** (work the 3 must-fix above; likely requires restructuring so the Publish line and the redacted-section extraction happen after 7e, or deferring the Publish line's population to a follow-up amend/commit) → **review-code** round 4. Not dispatched by this pass — the host orchestrator drives the next phase.

## Implementation
**Status**: complete
**When**: 2026-09-21 13:15 -04:00
**By**: Claude Code Agent (Claude Fable 5.1)

**Branch**: feature/issue-635 at `cf766f3`
**Addressed**: Local Review (Pre-Push) round 3, changes-requested, SHA `f72b5b7`
**Commits**: fb2d29b, 67525f3, cf766f3
**Mode**: host-inline (operator chose "redesign steps 6–7" at the round-3 checkpoint over another patch pass, on the round-3 reviewer's recommendation)

### Actions
- [x] (must-fix) `$WORKSPACE_SECTION` used in 7c but never assigned — step 6 is now "Render, redact, and write the report": it renders `$WORKSPACE_SECTION` and `$PROJECTS_SECTION` once and composes two named artifacts from them, `$HEALTH_BODY` (the exact bytes committed to `docs/health.md`) and `$REPORT_BODY` (the local report), with a table stating what each later sub-step consumes — `fb2d29b`
- [x] (must-fix) the template's `**Publish**: … PR <url>` line could not be filled when written — removed from the `## Workspace` section; `docs/health.md` deliberately carries no PR URL or outcome (the commit and PR are its provenance); the outcome is appended to the local report as `## Publish outcome` by new sub-step 7h on every exit path, and step 8 reports that same line — `fb2d29b`
- [x] (must-fix) `worktree_remove.sh --skill janitor-sweep` could delete the live worktree — cleanup keys on `$WT_PATH` (from `worktree_enter.sh`'s exported `WORKTREE_ROOT`) and `$NEW_BRANCH`, captured in 7a; 7g removes the run's own worktree after the PR is open; 7a removes any leftover `skill-*-janitor-sweep-*` worktree by exact path before creating the new one (sound because a completed run always removes its own); the newest-match tool is named as the wrong one — `fb2d29b`

### Also in the redesign
- Redaction moved from 7c to step 6 and applied once to both artifacts before either is written; nothing is appended to `$HEALTH_BODY` afterwards. Reason strings that reach the local report later (7e notes, 7h outcome) pass through `redact_text` themselves.
- The redundant 7b re-read of the previous `docs/health.md` is gone; step 5 reads it once from `$ROOT` via `git -C`.
- 7e gained a second loop deleting stray remote `skill/janitor-sweep-*` branches with no open PR (a 7d push that succeeded before `gh pr create` failed), so that failure strands nothing on origin.
- Sub-steps relabeled 7a–7h; step 7 opens with an explicit inputs/captured-values contract.

### Plan sync
`plan.md` Approach steps 3 and 4 and the implementation notes updated in `67525f3` to describe the named-artifact data flow, the no-PR-URL-in-health-doc decision, 7a–7h, and the exact-path cleanup.

### Next step
Lifecycle: **Implementation** → **review-code** (round 4, pre-push). Not dispatched by this pass.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-21 13:20 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)
**Verdict**: changes-requested

**Branch**: feature/issue-635 at `74e33b7`
**Mode**: pre-push
**Depth**: Deep (reason: governance-trigger files (`SKILL.md`, `AGENTS.md`) + a full redesign of the publish steps; unchanged from rounds 1-3)
**Must-fix**: 1 | **Suggestions**: 4
**Round**: 4 | **Ship**: recommended — must-fix count dropped from 3 (round 3) to 1, the one remaining defect is mechanical (add missing variable assignments at 4 named points), no design question

### Round-3 verification (all 3 must-fix items confirmed fixed by the redesign)
- [x] `$WORKSPACE_SECTION` undefined at point of use — resolved: step 6's variable table (`SKILL.md:598-603`) names it, renders it once, and composes both `$HEALTH_BODY`/`$REPORT_BODY` from it (`:605-616`); consistently referenced everywhere it's used
- [x] Unfillable `Publish: ... PR <url>` field rendered into `docs/health.md` before the PR exists — resolved: the field is gone from the `## Workspace` section/template (`:690-836` has no such line); the committed file deliberately carries no PR URL (`:641-646`); the outcome is appended to the *local* report only, by 7h, after it's known
- [x] `worktree_remove.sh --skill janitor-sweep` newest-match risk deleting the live retry worktree — resolved and independently re-verified against source: 7a's cleanup glob (`.workspace-worktrees/skill-*-janitor-sweep-*`) is byte-identical to `find_worktree_by_skill`'s glob (`_worktree_helpers.sh:152-154`), confirmed `janitor-sweep` is in `ALLOWED_SKILLS` (`worktree_create.sh:387`) and `--type workspace` always lands in `.workspace-worktrees/` (`worktree_create.sh:783`); after 7a's removal loop at most one match remains before `worktree_enter.sh --skill janitor-sweep` runs, so the newest-match tie-break is moot by construction; cleanup itself now keys on `$WT_PATH`/`$NEW_BRANCH` captured in 7a, never on a fresh lookup

### Earlier-round fixes verified still intact
New-PR-first/old-PR-second ordering (7d before 7e, with an explicit "never the reverse" rationale, `SKILL.md:1031-1038`); branch delete gated on `gh pr close`'s own exit status (7e, `:1003-1006`); 7e's heredoc quoted (`cat << 'EOF'`); no private memory filename anywhere in the repo; Projects tiers 2-5 carry the same `[New/Resolved/Unchanged]` diff-state shape as tier 1; the provisional-decisions sub-list states its no-diff exception explicitly in steps 5 and 6.

### Findings
- [ ] (must-fix) **`$PUBLISH_LINE`, read at 7h (`SKILL.md:1069`, `redact_text "$PUBLISH_LINE"`), is never assigned anywhere in the document.** Step 6's variable table (`:598-603`) — written specifically to close out "a variable read but never assigned," the round-3 defect class — covers `$WORKSPACE_SECTION`/`$PROJECTS_SECTION`/`$HEALTH_BODY`/`$REPORT_BODY` but omits `$PUBLISH_LINE`. Every failure exit in step 7 (7a `:911`, 7c `:948`, 7d `:979`) only echoes `FAILED(workspace publish: ...)` to the conversation — none sets `PUBLISH_LINE=`; there is also no assignment on the success path after 7g (`:1043-1057`). On literal execution, 7h's `## Publish outcome` section renders blank instead of the PR URL or the failure reason, contradicting step 8's own claim (`:1091`) that the operator report states "the same `$PUBLISH_LINE` 7h appended." Same defect class as round 3's `$WORKSPACE_SECTION` bug, relocated outside step 6's table. Fix: add explicit `PUBLISH_LINE="..."` assignments at each of 7a/7c/7d's FAILED points and after 7g's success, or fold `$PUBLISH_LINE` into step 6's table/data-flow contract the same way `$NEW_PR_URL` is captured via command substitution in 7d.
- [ ] (suggestion) `SKILL.md:925-933` (7b, write `docs/health.md`) has no stated failure handling, unlike 7a/7c/7d, and the failure-state summary (`:1075-1082`) lists only 7a/7c/7d — a write failure here would most likely surface, mislabeled, as 7c's commit failure rather than its true cause.
- [ ] (suggestion) `SKILL.md:909-915` (7a) — the `worktree_create.sh ... || { echo "FAILED(...)"; }` snippet doesn't `exit`/`return`, so on literal execution it falls through to `source worktree_enter.sh` after a reported creation failure; the prose right after says "skip to 7h" so this is low-risk (7a's own orphan removal already cleared prior stale worktrees), but a guard would be more robust.
- [ ] (suggestion) `worktree_create.sh` does not itself reject `--type layer` for a workspace-only allowlisted skill (`janitor-sweep`); nothing in this diff exercises that path (the skill always passes `--type workspace`), but a future hand-edit or misuse could park a worktree under `layers/worktrees/`, which 7a's cleanup glob wouldn't reach.
- [ ] (suggestion) `SKILL.md` 7a's `git branch -D "$br"` after a successful `worktree remove` has no failure check or log — cosmetic.

### Specialist summary
- **Round 1-3 fix re-verification**: done directly by the lead reviewer — grep + source-read against `_worktree_helpers.sh`, `worktree_create.sh`, `worktree_enter.sh`, `redact.sh`, and the full text of `SKILL.md` steps 1, 3-8. All confirmed.
- **Claude Adversarial / Lens A** (logic & correctness, Deep horizon, fresh subagent): found the `$PUBLISH_LINE` must-fix independently; also flagged the two suggestions above (7b's missing failure handling, 7a's non-exiting fallthrough) under the guidance-doc calibration; confirmed `$WORKSPACE_SECTION`/`$PROJECTS_SECTION`/`$HEALTH_BODY`/`$REPORT_BODY`/`$TS`/`$WT_PATH`/`$NEW_BRANCH`/`$NEW_PR_URL`/`$PREV_HEALTH` all consistently assigned and consumed; confirmed `docs/health.md` never carries an unknowable-at-render-time field.
- **Claude Adversarial / Lens B** (security/concurrency/lifecycle, Deep horizon, fresh subagent): found no must-fix; independently re-verified the 7a glob-match soundness against `_worktree_helpers.sh`/`worktree_create.sh` source, confirmed the concurrency disclosure is honest (not false-safe), traced full redaction coverage of `$HEALTH_BODY`/`$REPORT_BODY`/7e's PR comment/7h's publish line with no gap, confirmed lifecycle half-states are handled or noted rather than silent, and confirmed the 7d→7e ordering is adequately and explicitly documented. Flagged the `--type layer` allowlist gap and the unchecked `git branch -D` as non-blocking suggestions.
- **Governance / Plan Drift**: spot-checked directly — `AGENTS.md`, `worktree_create.sh`, `skill_workflows.md`, `principles_review_guide.md` all remain consistent with each other and with the redesign; `plan.md`'s Approach steps 3-4 accurately describe the named-artifact data flow, the no-PR-URL-in-health-doc decision, and 7a-7h (spot-checked against `SKILL.md`, not separately dispatched — unchanged consequences-map rows already confirmed Done in round 2).
- **Copilot Adversarial**: not run (`--copilot` not requested).
- **Local Model Adversarial**: not run (`--local` not requested).

### Notes
- The round-3→round-4 pattern held: the redesign closed all 3 named defects cleanly with no regression, and turned up one new, narrower instance of the *same* defect class (a referenced-but-unassigned variable) that step 6's own anti-pattern table didn't cover because `$PUBLISH_LINE` lives in step 7, not step 6. This is a single mechanical omission, not evidence the publish section needs another structural pass — the fix is additive (name the assignment points), not a further redesign.
- Must-fix count: round 1 unknown/fixed, round 2: 6, round 3: 3, round 4: 1 — monotonically decreasing and now at the "low and mechanical" bar the convergence rule names.

### Next step
Lifecycle: **Local Review (Pre-Push)** round 4, changes-requested (ship recommended after the one mechanical fix) → **address-findings** (add the 4 `PUBLISH_LINE` assignment points) → push / open PR → **triage-reviews**. Not dispatched by this pass — the host orchestrator drives the next phase.
