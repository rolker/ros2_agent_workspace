---
issue: 628
---

# Issue #628 — Planning-document vocabulary + two-root rule: vision/roadmap/decisions/health per scope, with the janitor publish + trigger decision

## Issue Review
**Status**: complete
**When**: 2026-09-14 13:43 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)

**Issue**: #628
**Comment**: best-effort post follows this entry; not recorded inline
**Scope verdict**: needs-splitting

### Actions
- [ ] Sequence the "Deliverables (this repo)" checklist into separate, independently-mergeable issues/PRs (ADR first, then `docs/roadmap.md`, then sweep/health split, then discovery schema + skill changes, then templates, then consequences-map/AGENTS.md rows) rather than one implementation pass — the issue itself cites #249/draft-PR-#257's stall as the cautionary precedent this supersedes; repeating a single large umbrella effort risks the same fate. `plan-task` should propose the sequencing and open the sub-issues (parent-referenced, `Part of #628`) before implementation starts on any one piece.
- [ ] The ADR must explicitly resolve **commit identity for the automated publish path**: point 4 reopens the exact question #569's operator comment (2026-09-11) deliberately deferred out of this slice ("No PRs opened by the sweep, so the commit-identity question — a cloud Routine or cron container acting as the operator's GitHub user — stays out of this slice"). A PR opened by an unattended trigger (cron container / cloud Routine) with no interactive agent session needs a stated identity and auth path (AGENTS.md § Agent Commit Identity assumes a live agent session setting `$AGENT_NAME`/`$AGENT_EMAIL`); the ADR should say who/what commits and how that satisfies `check-commit-identity.py` / `check_pr_authors.py`.
- [ ] The ADR should state plainly (even though AGENTS.md already requires it generally) that the committed health-document PR is **not exempt from human content review before merge** — "green CI is not review" (AGENTS.md § Merging) applies to an automated health-doc PR exactly as it does to any other; worth stating explicitly since this is the first case of a fully-unattended trigger opening a PR.
- [ ] Update `.agent/knowledge/principles_review_guide.md`'s Consequences Map exception clause for `janitor-sweep` once its durable output changes from a local scratchpad report to a committed health document — the clause's current wording ("Only `janitor-sweep` has a durable output today: a local report file under `.agent/scratchpad/janitor/`") goes stale under this proposal and isn't listed among the issue's deliverables.
- [ ] Confirm the discovery mechanism degrades gracefully (not an error) for the large majority of project repos that will not have the discovery file for some time — mirror ADR-0017's incremental-rollout stance ("repos without the file simply behave as before"). Not stated in the issue; should be an explicit test case in the plan.
- [ ] The trigger-mechanism decision (Claude Code Routine vs. anacron vs. GH Actions cron) has real constraints already on record in #569 (laptop often off rules out plain cron; a cloud Routine "cannot reach gitcloud or `layers/`", which matters for project-repo health docs living on gitcloud-mirrored repos) — the ADR needs to actually settle this, not leave it open, per ADR-0001's "capture decisions" bar.
- [ ] If discovery-schema resolution needs a new shared script (parallel to `field_mode.sh` / `manifest_fallback.sh`), add it to `AGENTS.md`'s Script Reference table per the consequences map row for `.agent/scripts/` changes — not currently in the deliverables list.

## Plan Authored
**Status**: complete
**When**: 2026-09-14 13:53 -04:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-628/plan.md` at `864294d`
**Branch**: feature/issue-628 at `864294d`
**Phases**: 4 sub-issues (Part of #628); this worktree delivers (1) — ADR + workspace `docs/roadmap.md` + roadmap template

### Open questions
- [ ] Trigger mechanism: GH Actions weekly cron / Claude Code cloud Routine / anacron on the laptop (or an Actions+anacron split) — the plan's comparison table is the ADR's input; the operator's answer becomes the ADR's Decision.
- [ ] Is `.agent/templates/roadmap.md` in sub-issue (1) or a later one? Plan puts it in (1) so the loop section is not written twice; reversible at review.
- [ ] `AGENTS.md` is Ask First — approving the plan approves exactly two additions (a planning-documents pointer and `docs/roadmap.md` in References).
- [ ] Confirm this PR closes #249 (the workspace roadmap is the direction that issue asked for) — it is a closing keyword in the PR body. Note: PR #257 actually MERGED 2026-02-26; it is #249 that has been idle, not a stalled draft PR as the issue body states.

## Plan Review
**Status**: complete
**When**: 2026-09-14 13:56 -04:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-628/plan.md` at `864294d`
**PR**: PR-less (`--issue` mode, worktree `feature/issue-628`)
**Verdict**: changes-requested

Independent review — dispatched as a fresh-context sub-agent (handoff header present); not the plan author.

Every factual claim in the plan was checked against the tree and GitHub and all
of them hold: 0019 is the highest ADR and `docs/roadmap.md` does not exist; there
is no `.agent/templates/roadmap.md`; [PR #257](https://github.com/rolker/ros2_agent_workspace/pull/257)
**merged** 2026-02-26 (created 02-25) while
[#249](https://github.com/rolker/ros2_agent_workspace/issues/249) is still open —
the plan's "factual correction for the record" is correct and the issue body is
the thing that was wrong; the BizzyBoat roadmap's closing loop section exists as
`## How this roadmap stays useful` (with `## What's not on this roadmap` above
it); ADR-0015 really is titled "container-produces / host-publishes" and
ADR-0019 really does say the no-GitHub-write-auth line is a property of the
launcher's token configuration. Scope, approach and principle alignment are
sound; the findings below are additive amendments, not a rework.

### Evaluation
| Dimension | Verdict | Notes |
|---|---|---|
| Scope | Good | Three new documents + three small edits + filing (2)-(4) is one PR. The umbrella split is exactly what review-issue asked for. |
| Issue alignment | Needs work | The issue's *project-repo* deliverables are never filed, yet the plan closes #628 at (4). See finding 1. |
| File targeting | Needs work | `CLAUDE.md`'s own References list drifts and is not in the plan. See finding 2. |
| Consequences | Needs work | Findings 2 and 6. |
| Documentation & Instruction impact | Good | Present, non-silent, instruction items framed as operator-decided candidates. |
| Principle alignment | Needs work | ADR-0004 (enforcement hierarchy) is triggered and unlisted; the roadmap's declared loop does not exist until (4). Findings 5, 7. |
| ADR compliance | Needs work | ADR-0020 introduces a third publisher that ADR-0015's Decision does not contemplate; the addendum is left conditional. Finding 4. |
| ROS conventions | N/A | Workspace documentation plan. |

### Findings
- [ ] (must-fix) The issue's **project-repo deliverables are dropped**: #628's body routes a `unh_marine_autonomy` roadmap + kind markers and a `unh_echoboats_project11` parent line / health doc to separate issues in those repos, "linked from here". The plan files only (2), (3), (4) — all workspace-repo — and step 8 closes #628 with (4). Those two deliverables would be lost at close. Add them (as sub-issues filed in the project repos in step 7, or as named, linked entries in `docs/roadmap.md`) before #628 is allowed to close — `plan.md` Sub-issue sequence table, step 7, step 8
- [ ] (must-fix) **The trigger comparison is not a fair decision input**, and the operator decides from it at this checkpoint. Three defects: (a) the Cost column is not apples-to-apples — all three mechanisms run the same Claude Code agent, so the Actions cron also draws subscription usage and also needs `CLAUDE_CODE_OAUTH_TOKEN` forwarded, but only the Routine row says so and only Actions is labelled "Free"; (b) the Routine's identity objection conflates the **GitHub PR author** with the **git commit identity** — `check_pr_authors.py` checks commit authors, and a Routine can set the bot identity per-commit with `-c` exactly as the anacron row says anacron must, so the same solvable chore is scored as a near-disqualifying "direct tension" in one row and a footnote in another; (c) the **credential-surface axis is missing** — Actions needs a standing write-scoped PAT stored as a secret on a *public* repo, anacron reuses credentials already on the host and mints nothing. Fix all three before the table is put in front of the operator — `plan.md` Trigger comparison table
- [ ] (must-fix) **Publishing the sweep report by commit is gated on unfixed redaction work.** [#626](https://github.com/rolker/ros2_agent_workspace/issues/626) carries open, unfixed items in exactly one class — absolute host paths and credentials reaching stderr that the sweep report transcribes (resolver refusals, failed `exec 9>`, the `mkdir -p` arm). The ADR decides that report gets committed to a public repo. State the gate in the ADR's Consequences — the commit-publish path does not go live until that class is closed — so the decision does not authorise publishing unredacted host paths — `plan.md` Approach step 1 ("Publish means commit, not post")
- [ ] (must-fix) `CLAUDE.md` **drifts** with the `AGENTS.md` References addition. `CLAUDE.md:26-32` carries its own References list (README § Vision, ARCHITECTURE.md, `docs/decisions/`, …) that parallels `AGENTS.md`'s. The plan's consequences row dismisses this as "adapters carry no roadmap rows to drift" — true but beside the point: the adapter's list is the one that *should* gain the roadmap row, per the Consequences Map's own "`AGENTS.md` → framework adapters if affected". Add `CLAUDE.md` to Files to Change (it is also **Ask First**, so name it in step 6's approval scope) — `plan.md` Files to Change, Consequences table row 3, step 6
- [ ] (must-fix) **ADR-0015 needs the cross-reference addendum decided here, not conditionally.** ADR-0015's Decision states the host publishes and "nothing publishes from inside the sandbox"; its host/container dichotomy has no third actor. An unattended trigger has **no host session** — it *is* the publisher. That is not a gap ADR-0020 can cite 0015 for, it is a case 0015 does not cover. Commit to the ADR-0012 addendum on 0015 in this PR rather than the plan's "if ... it lands as an addendum", and have ADR-0020 name the unattended publisher explicitly as a third actor — `plan.md` ADR Compliance table, ADR-0012 row
- [ ] (suggestion) **ADR-0004 / ADR-0005 (enforcement hierarchy) are triggered and absent from the ADR Compliance table.** ADR-0020 states new compliance rules ("a roadmap names the roadmaps beneath it"; "read the roadmap and the health document together before choosing work") with no hook, CI check or guardrail. The Principles Self-Check is honest about this ("the enforcement is the sweep ... lands in (3)/(4)") — carry that same honesty into the ADR Compliance table as an explicit ADR-0004 row with the deferral named, rather than leaving the ADR silent on it — `plan.md` ADR Compliance table
- [ ] (suggestion) **The workspace roadmap declares a forcing function that will not exist for three more PRs.** Step 3 names "the periodic sweep" as the workspace's loop, but the sweep is not scheduled until (4). A roadmap whose loop section describes a cadence nobody runs is the exact failure mode the issue diagnoses in the framework's `VISION.md`. Either name a forcing function that exists today (plan review / the next hand-run sweep) or mark the section as pending with a link to (4) — `plan.md` Approach step 3
- [ ] (suggestion) **Consider whether the discovery declaration needs a new per-repo file at all.** Project repos already carry `.agents/README.md`, a root `AGENTS.md`, `.agents/deployment.yaml`, `.agents/review-context.yaml` and `.agents/ci_local_upstream_extra.sh`; ADR-0017's own Negative consequence is "one more per-repo file to keep current". Two fields (roadmap path, health path) may belong in an existing `.agents/` file. The schema is (2)'s, but the ADR states the principle — have it leave that open rather than mandating a separate file — `plan.md` Approach step 1 ("Discovery, in principle")
- [ ] (suggestion) **Before using `Closes #249`, check what else #249 asked for.** #249 is titled "Workspace rework: establish direction and improve incrementally" and PR #257 merged against it; the roadmap answers "establish direction" but #249 may carry unclosed items beyond it. Confirm each is either in `docs/roadmap.md` or explicitly dropped, then close. The plan already flags the keyword for operator confirmation — this is the check that should precede that answer — `plan.md` step 8, Open Questions
- [ ] (suggestion) **Consider splitting ADR-0020 in two.** It bundles a documentation taxonomy (kinds, two-root rule, discovery) with a mechanism decision (publish=commit, bot identity, trigger, human-review gate). When the trigger changes, a superseding ADR would have to churn the taxonomy too. Two ADRs in the same PR still satisfies the operator's "the ADR in (1) weighs the trigger and decides it" — `plan.md` Approach step 1
- [ ] (suggestion) **Bound the workspace roadmap's first cut.** The BizzyBoat roadmap it is modelled on is ~700 lines; the plan sets no size target. A first roadmap that is a survey rather than a short list of next moves is hard to keep current — `plan.md` Approach step 3

### Note for the operator checkpoint
The plan correctly does **not** pre-decide the trigger — it presents the
comparison and defers to plan review. But the comparison as written tilts
against the Claude Code Routine on a ground that is solvable (finding 2b) and
understates the Actions option's cost and credential surface (2a, 2c). The
trigger question should not be put to the operator until that table is corrected.
