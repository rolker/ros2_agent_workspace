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
- [ ] (must-fix) `SKILL.md` step 6 ("minus the run-over-run-diff subsections, which have no meaning outside this report") directly contradicts step 7 sub-step 3 ("with the `## Workspace` section's content from step 6 (tiers, `New`/`Resolved`/`Unchanged`...)") on whether New/Resolved/Unchanged subsections are committed into `docs/health.md` — `.claude/skills/janitor-sweep/SKILL.md:748-750` vs `:780-783`
- [ ] (must-fix) Step 7 sub-step 5's PR-replacement text explicitly permits closing/deleting the prior `skill/janitor-sweep-*` PR and branch *before* the new push/`gh pr create` is confirmed to succeed ("either order is fine") — if push or PR-create then fails, the previously-open published record is destroyed with no PR left open and no recovery path stated; must mandate push-and-create-new-PR-first, close-old-PR-second — `.claude/skills/janitor-sweep/SKILL.md:811-814`
- [ ] (suggestion) The literal `gh pr comment` code block posts a placeholder URL (`<new PR URL, filled in after step 6 opens it>`) with no enforced substitution step before the call — an agent running the block verbatim posts a broken comment to a public PR; make the two-pass nature (comment now with placeholder → edit after new PR opens, or defer the whole comment) an explicit numbered sub-step rather than prose-only — `.claude/skills/janitor-sweep/SKILL.md:806, 811-814`
- [ ] (suggestion) `git push origin --delete "$OLD_BRANCH" || true` swallows any failure (including a real auth/permission error) under "may already be gone" — worth surfacing the actual failure reason in the report rather than fully discarding it — `.claude/skills/janitor-sweep/SKILL.md:808`
- [ ] (suggestion) No stated concurrency invariant for overlapping sweep runs once #636 wires the weekly trigger — a scheduled run and a hand-run could race on step 7 sub-step 5's list/close/delete against each other's GitHub state — worth a one-line stated assumption ("only one sweep publishes at a time") — `.claude/skills/janitor-sweep/SKILL.md` § 7 (whole section)
- [ ] (suggestion) No cleanup guidance for an orphaned skill worktree/branch left behind by a failed workspace publish (`FAILED(workspace publish: ...)`) — could accumulate stale `skill/janitor-sweep-*` worktrees over repeated runs — `.claude/skills/janitor-sweep/SKILL.md:833-839`
- [ ] (suggestion) `$BODY_FILE` used in `gh pr create --body-file "$BODY_FILE"` with no local `mktemp`/heredoc construction shown, unlike every other command in this doc shown verbatim — `.claude/skills/janitor-sweep/SKILL.md:823`
- [ ] (suggestion) Step 7's own local sub-step numbering ("step 6" inside its 1-7 list, meaning its own sub-step 6) visually collides with the document's top-level step 6 ("Write the report") — disambiguated only by "below" in prose; worth a distinct label to survive future reordering — `.claude/skills/janitor-sweep/SKILL.md:812` (and step 7's sub-list generally)

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
