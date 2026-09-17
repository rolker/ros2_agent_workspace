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
- [x] (must-fix) The issue's **project-repo deliverables are dropped**: #628's body routes a `unh_marine_autonomy` roadmap + kind markers and a `unh_echoboats_project11` parent line / health doc to separate issues in those repos, "linked from here". The plan files only (2), (3), (4) — all workspace-repo — and step 8 closes #628 with (4). Those two deliverables would be lost at close. Add them (as sub-issues filed in the project repos in step 7, or as named, linked entries in `docs/roadmap.md`) before #628 is allowed to close — `plan.md` Sub-issue sequence table, step 7, step 8
- [x] (must-fix) **The trigger comparison is not a fair decision input**, and the operator decides from it at this checkpoint. Three defects: (a) the Cost column is not apples-to-apples — all three mechanisms run the same Claude Code agent, so the Actions cron also draws subscription usage and also needs `CLAUDE_CODE_OAUTH_TOKEN` forwarded, but only the Routine row says so and only Actions is labelled "Free"; (b) the Routine's identity objection conflates the **GitHub PR author** with the **git commit identity** — `check_pr_authors.py` checks commit authors, and a Routine can set the bot identity per-commit with `-c` exactly as the anacron row says anacron must, so the same solvable chore is scored as a near-disqualifying "direct tension" in one row and a footnote in another; (c) the **credential-surface axis is missing** — Actions needs a standing write-scoped PAT stored as a secret on a *public* repo, anacron reuses credentials already on the host and mints nothing. Fix all three before the table is put in front of the operator — `plan.md` Trigger comparison table
- [x] (must-fix) **Publishing the sweep report by commit is gated on unfixed redaction work.** [#626](https://github.com/rolker/ros2_agent_workspace/issues/626) carries open, unfixed items in exactly one class — absolute host paths and credentials reaching stderr that the sweep report transcribes (resolver refusals, failed `exec 9>`, the `mkdir -p` arm). The ADR decides that report gets committed to a public repo. State the gate in the ADR's Consequences — the commit-publish path does not go live until that class is closed — so the decision does not authorise publishing unredacted host paths — `plan.md` Approach step 1 ("Publish means commit, not post")
- [x] (must-fix) `CLAUDE.md` **drifts** with the `AGENTS.md` References addition. `CLAUDE.md:26-32` carries its own References list (README § Vision, ARCHITECTURE.md, `docs/decisions/`, …) that parallels `AGENTS.md`'s. The plan's consequences row dismisses this as "adapters carry no roadmap rows to drift" — true but beside the point: the adapter's list is the one that *should* gain the roadmap row, per the Consequences Map's own "`AGENTS.md` → framework adapters if affected". Add `CLAUDE.md` to Files to Change (it is also **Ask First**, so name it in step 6's approval scope) — `plan.md` Files to Change, Consequences table row 3, step 6
- [x] (must-fix) **ADR-0015 needs the cross-reference addendum decided here, not conditionally.** ADR-0015's Decision states the host publishes and "nothing publishes from inside the sandbox"; its host/container dichotomy has no third actor. An unattended trigger has **no host session** — it *is* the publisher. That is not a gap ADR-0020 can cite 0015 for, it is a case 0015 does not cover. Commit to the ADR-0012 addendum on 0015 in this PR rather than the plan's "if ... it lands as an addendum", and have ADR-0020 name the unattended publisher explicitly as a third actor — `plan.md` ADR Compliance table, ADR-0012 row
- [x] (suggestion) **ADR-0004 / ADR-0005 (enforcement hierarchy) are triggered and absent from the ADR Compliance table.** ADR-0020 states new compliance rules ("a roadmap names the roadmaps beneath it"; "read the roadmap and the health document together before choosing work") with no hook, CI check or guardrail. The Principles Self-Check is honest about this ("the enforcement is the sweep ... lands in (3)/(4)") — carry that same honesty into the ADR Compliance table as an explicit ADR-0004 row with the deferral named, rather than leaving the ADR silent on it — `plan.md` ADR Compliance table
- [x] (suggestion) **The workspace roadmap declares a forcing function that will not exist for three more PRs.** Step 3 names "the periodic sweep" as the workspace's loop, but the sweep is not scheduled until (4). A roadmap whose loop section describes a cadence nobody runs is the exact failure mode the issue diagnoses in the framework's `VISION.md`. Either name a forcing function that exists today (plan review / the next hand-run sweep) or mark the section as pending with a link to (4) — `plan.md` Approach step 3
- [x] (deferred: not in the operator's approved scope for this revision pass — the go-ahead enumerated the other three suggestions and gave this one no disposition; raised for the operator at the next plan review) (suggestion) **Consider whether the discovery declaration needs a new per-repo file at all.** Project repos already carry `.agents/README.md`, a root `AGENTS.md`, `.agents/deployment.yaml`, `.agents/review-context.yaml` and `.agents/ci_local_upstream_extra.sh`; ADR-0017's own Negative consequence is "one more per-repo file to keep current". Two fields (roadmap path, health path) may belong in an existing `.agents/` file. The schema is (2)'s, but the ADR states the principle — have it leave that open rather than mandating a separate file — `plan.md` Approach step 1 ("Discovery, in principle")
- [x] (suggestion) **Before using `Closes #249`, check what else #249 asked for.** #249 is titled "Workspace rework: establish direction and improve incrementally" and PR #257 merged against it; the roadmap answers "establish direction" but #249 may carry unclosed items beyond it. Confirm each is either in `docs/roadmap.md` or explicitly dropped, then close. The plan already flags the keyword for operator confirmation — this is the check that should precede that answer — `plan.md` step 8, Open Questions
- [x] (deferred: operator declined the split 2026-09-14; recorded as a named option in Approach step 1, with the seam identified, so a future supersede can take it) (suggestion) **Consider splitting ADR-0020 in two.** It bundles a documentation taxonomy (kinds, two-root rule, discovery) with a mechanism decision (publish=commit, bot identity, trigger, human-review gate). When the trigger changes, a superseding ADR would have to churn the taxonomy too. Two ADRs in the same PR still satisfies the operator's "the ADR in (1) weighs the trigger and decides it" — `plan.md` Approach step 1
- [x] (suggestion) **Bound the workspace roadmap's first cut.** The BizzyBoat roadmap it is modelled on is ~700 lines; the plan sets no size target. A first roadmap that is a survey rather than a short list of next moves is hard to keep current — `plan.md` Approach step 3

### Note for the operator checkpoint
The plan correctly does **not** pre-decide the trigger — it presents the
comparison and defers to plan review. But the comparison as written tilts
against the Claude Code Routine on a ground that is solvable (finding 2b) and
understates the Actions option's cost and credential surface (2a, 2c). The
trigger question should not be put to the operator until that table is corrected.

## Implementation
**Status**: complete
**When**: 2026-09-14 14:15 -04:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-628 at `368c56f`
**Addressed**: `## Plan Review` (2026-09-14 13:56 -04:00, plan at `864294d`, verdict changes-requested)
**Commits**: 3c2c78a (plan revision), 368c56f (finding checkboxes)

Plan-revision pass — the findings were against `.agent/work-plans/issue-628/plan.md`,
not against code. No source file changed. Every factual claim added was verified
against the tree or `gh` (both project repos are `git@github.com:rolker/…`;
`unh_marine_autonomy` has no `docs/roadmap.md`; the BizzyBoat roadmap is 703 lines
with no parent line; #626's open items include the host-path/credential-to-stderr
class; `check_pr_authors.py` inspects each commit's primary author, not the PR
opener; `GITHUB_TOKEN` is scoped to the repo running the workflow; ADR-0012 permits
Status-line + References addendums; `CLAUDE.md:23-34` carries its own References list).

### Actions
- [x] (must-fix) Project-repo deliverables restored as sub-issues (5) and (6), filed in `rolker/unh_marine_autonomy` and `rolker/unh_echoboats_project11` and linked from #628; step 8 files them cross-repo, step 9 states #628 does not close until they exist — `plan.md` Sub-issue sequence table, steps 8-9
- [x] (must-fix) Trigger comparison rewritten as a fair decision input: commit-identity and credential-surface columns added, all three rows scored on the same basis, a "what is *not* a discriminator" paragraph separating the PR-author question from commit identity, and the three real discriminators named. Still not pre-decided — `plan.md` Trigger comparison
- [x] (must-fix) #626 redaction gate stated as an ADR-0020 Consequence, with the specific open sites named; (3)/(4) must not ship the publish path before it closes — `plan.md` Approach step 1
- [x] (must-fix) `CLAUDE.md` References added to Files to Change, to the Ask-First approval scope (step 7), to the Consequences row and to Documentation & Instruction Impact — `plan.md` step 7, Files to Change, Consequences row 3
- [x] (must-fix) ADR-0012 addendum on ADR-0015 decided here, not conditionally: new Approach step 2 quotes the addendum text (unattended trigger = a third actor with no host session), ADR Compliance row rewritten, ADR-0015 added to Files to Change — `plan.md` Approach step 2, ADR Compliance
- [x] (suggestion) ADR-0004 / ADR-0005 row added to the ADR Compliance table, naming the deferral and its closers ((2) and (4)) — `plan.md` ADR Compliance
- [x] (suggestion) Roadmap first cut bounded at 150 lines, against the measured 703-line BizzyBoat roadmap — `plan.md` Approach step 4
- [x] (suggestion) #249's remaining items checked before the closing keyword: four body properties plus three items from its operator comment, each dispositioned in step 9; two carried forward explicitly, with a drop-to-`Part of` fallback in Open Questions — `plan.md` step 9, Open Questions
- [x] (suggestion) Forcing function addressed by wording: the loop is declared ahead of its trigger, the pending state is written into the roadmap itself and linked to (4), and why declaring it early is deliberate — `plan.md` Approach step 4
- [x] (suggestion) Splitting ADR-0020 in two — recorded as a named option in Approach step 1 with the seam identified (deferred: operator declined the split 2026-09-14; note it, do not do it)
- [x] (suggestion) Whether the discovery declaration needs a new per-repo file — (deferred: not in the operator's approved scope for this pass; the go-ahead enumerated the other three suggestions and gave this one no disposition. **Open for the operator at the next plan review** — the change would be one clause in Approach step 1 making the ADR state the principle without mandating a separate file)

### Notes
- Nothing pushed, per the sub-agent handoff contract.
- Next: `review-plan` re-review of the revised plan, then the operator's trigger decision at the plan-review checkpoint with the corrected table.

## Plan Review
**Status**: complete
**When**: 2026-09-14 14:20 -04:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-628/plan.md` at `3c2c78a`
**PR**: PR-less (`--issue` mode, worktree `feature/issue-628`)
**Verdict**: changes-requested

Independent review — dispatched as a fresh-context sub-agent (handoff header
present); not the plan author. Round 2, against the revision at `3c2c78a`.

**All five round-1 must-fixes are genuinely closed in the plan text.** (1)
project-repo deliverables restored as sub-issues (5)/(6) with the close
condition on #628 stated (both repos verified `git@github.com:rolker/…`;
`unh_marine_autonomy` has no `docs/roadmap.md` and carries exactly the eight
flat `docs/*.md` pages the plan lists; the BizzyBoat roadmap is 703 lines with
no parent line). (2) the trigger table gained commit-identity and
credential-surface columns and a "what is *not* a discriminator" paragraph —
`check_pr_authors.py` does inspect each commit's **primary author email**
(`check_pr_authors.py:115-127`), so the PR-author/commit-identity separation is
correct. (3) the #626 gate is stated with the right class (its items are open
and unchecked). (4) `CLAUDE.md` added — its References list is at
`CLAUDE.md:23-34` as claimed. (5) the ADR-0015 addendum is decided in this PR.
Four of the six should-fixes are applied; the ADR-split decline and the
per-repo-discovery-file deferral are recorded.

Every other checkable claim holds: repo is PUBLIC; 19 ADRs with 0019 highest and
no `docs/roadmap.md` / `.agent/templates/roadmap.md`; PR #257 MERGED 2026-02-26
while #249 is OPEN; the sweep does resolve repos via `resolve_repo_checkout.sh`
+ `manifest_fallback.sh` and works with no `layers/` (SKILL.md:124-127, 197);
`GITHUB_TOKEN` repo-scoping and Actions-free-on-public are correct; the
Routine's cloud/GitHub-scoping and Pro 5 / Max 15 / Team 25 caps (with the
inconsistent tier floor) match `.agent/knowledge/research_digest.md:124,132`;
`~/.config/ros2-agent/claude-oauth-token` exists.

**One trigger-table cell does not survive checking** (finding 1) and the
operator decides from that table at this checkpoint.

### Evaluation
| Dimension | Verdict | Notes |
|---|---|---|
| Scope | Good | Rev 2 adds two cross-repo issue filings and one ADR addendum; no scope creep beyond what the issue body already routes. |
| Issue alignment | Good | Project-repo deliverables restored; umbrella close condition stated. |
| File targeting | Good | `CLAUDE.md` and ADR-0015 now listed; both Ask-First files named in the approval scope. |
| Consequences | Needs work | Findings 3, 8. |
| Documentation & Instruction impact | Good | Present, non-silent, instruction items framed as operator-decided candidates. |
| Principle alignment | Needs work | Documentation accuracy: findings 1, 3, 4, 6. |
| ADR compliance | Needs work | The ADR-0015 addendum wording sits at ADR-0012's boundary — finding 5. |
| ROS conventions | N/A | Workspace documentation plan. |

### Findings
- [x] (must-fix) **Trigger table, Reach: the gitcloud half of anacron's unique-reach claim is wrong, and reach is named the first of the three real discriminators.** `janitor-sweep` deliberately **excludes** non-GitHub-origin repos (`is_field_url` rule, `.claude/skills/janitor-sweep/SKILL.md:271-287`), and all 44 repos in `configs/manifest/repos/*.repos` are `github.com` — so no mechanism sweeps gitcloud repos today and none would gain them. Anacron's unique reach is `layers/` alone, and the sweep is explicitly built not to assume `layers/` exists (SKILL.md:38, 197). Correct the cell and state what `layers/` actually buys a sweep before the operator chooses — `plan.md` Trigger comparison (anacron row, "The real discriminators" paragraph)
- [x] (must-fix) **The round-1 suggestion deferred *to the operator at this review* — whether the discovery declaration needs a new per-repo file — is not in the plan.** It lives only in progress.md; the operator reads `plan.md` at the checkpoint, so as written it will not be decided. Add it to Open Questions (the change itself is one clause in Approach step 1: have the ADR state the principle without mandating a separate file, given `.agents/README.md`, root `AGENTS.md`, `deployment.yaml`, `review-context.yaml` already exist and ADR-0017's own Negative is "one more per-repo file") — `plan.md` Open Questions, Approach step 1
- [x] (suggestion) **#249's disposition omits its live children.** #249's final comment (2026-02-26) records Phase 2 as *not started* and names [#263](https://github.com/rolker/ros2_agent_workspace/issues/263) (simplify scripts/Makefile — the same "not 42 scripts" body property the plan maps to a roadmap entry) and [#264](https://github.com/rolker/ros2_agent_workspace/issues/264) as where to resume, with #265/#266 spun off; **all four are OPEN**. Also, "its one operator comment" understates — #249 carries nine comments. Name #263/#264 (and #265/#266) in the roadmap's #249 disposition so `Closes #249` does not orphan them — `plan.md` step 9
- [x] (suggestion) **Stale line refs in the #626 gate.** The plan copies #626's file:line references, which #626 itself stamps "as of `d7b8baf`". `resolve_repo_checkout.sh:131,141` and `janitor-sweep/SKILL.md:108` still land correctly, but the lock-open sites are now `resolve_repo_checkout.sh:466` and `manifest_fallback.sh:205`, not `:436` / `:181`. Cite #626 rather than re-stating line numbers, or re-stamp them — `plan.md` Approach step 1
- [x] (suggestion) **The ADR-0015 addendum wording sits at ADR-0012's boundary.** ADR-0012 permits a Status-line note and a References entry; the quoted block is a three-sentence paragraph that asserts what ADR-0015's Decision "does not contemplate" — a scope qualification a later reader could contest, where ADR-0012's own examples are pure pointers. Keep the Status line to the pointer plus a References entry and carry the third-actor reasoning in ADR-0020 itself; say in the plan which sentence lands where — `plan.md` Approach step 2
- [x] (suggestion) **Attribution:** Context says "Per the operator's decisions comment (2026-09-14) … sequenced into **six** sub-issues". That comment enumerates four ((1)–(4)); (5) and (6) come from the issue body's project-repo deliverables and the round-1 review. Reword so no decision is attributed to the operator that was not stated (AGENTS.md § Documentation Accuracy) — `plan.md` Context
- [x] (suggestion) **Credential surface is ranked unevenly.** The Routine is "Smallest" while anacron — which mints nothing and keeps every credential on the host — is unranked; a cloud Routine holds a standing GitHub connection on Anthropic-managed infrastructure, which is not obviously smaller. Drop the superlatives and state the axis (new stored secrets: 2 / 0 / 0, plus where each existing credential lives) — `plan.md` Trigger comparison
- [x] (suggestion) **`principles_review_guide.md`'s ADR Applicability table stops at ADR-0013** — 0011, 0012 and 0014–0019 have no row. The Consequences table answers "Add an ADR → the ADR table … | Yes", so an ADR-0020 row would be the first in six; decide that deliberately, and have Files to Change name both edits (it currently names only the Consequences Map row) — `plan.md` Files to Change, Consequences table row 1

### Note for the operator checkpoint
The corrected table is now fair on cost, commit identity and credential surface —
the three round-1 defects are genuinely fixed. **Do not decide the trigger from
the Reach row as written**: it credits anacron with seeing gitcloud repos, and
the sweep excludes non-GitHub origins by design while every manifest repo is on
GitHub. On the evidence checked, the honest discriminators are **cadence
exactness** (Actions only), **credential surface** (Actions adds two standing
secrets; the other two add none), and **whether a sweep should see the local
`layers/` tree at all** (anacron only) — that last one is a design question the
plan has not answered. Separately, the per-repo-discovery-file question was
deferred to you at this review but never made it into the plan text (finding 2).

## Implementation
**Status**: complete
**When**: 2026-09-17 10:57 -04:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-628 at `ee41296` (plan rev 3 head; this entry's own commit follows)
**Addressed**: `## Plan Review` (round 2, 2026-09-14 14:20 -04:00, against plan sha `3c2c78a`) — 2 must-fix + 6 suggestions, all eight actioned
**Commits**: `5576551` `0ff3e13` `aee1383` `040cb38` `4bb491f` `faca3f3` `bbf7c60` `4510548` `ce8b8cc` `040f8fd` `ee41296`

Plan rev 3. Beyond the eight review findings, this pass folds in the operator
decisions recorded on #628 (2026-09-14 18:56Z and 2026-09-17), which settle the
two questions the round-2 review left at the checkpoint.

### Actions
- [x] (must-fix) Trigger table reach cell — the anacron row no longer claims gitcloud reach. Verified: `janitor-sweep` excludes non-GitHub origins by url via `is_field_url` before the rotation is built, and all 44 `url:` entries in `configs/manifest/repos/*.repos` are `github.com` (2026-09-17). Anacron's unique reach is the local `layers/` tree alone, now stated as what it actually buys: the two layer-dependent `audit-project` checks that are SKIPPED in `clone` mode (optional `colcon test`, "correct layer") and grading the operator's working tree rather than the manifest-pinned ref — `plan.md` § Trigger comparison
- [x] (must-fix) The discovery question is now in `plan.md`, recorded as decided rather than open — `plan.md` Open Questions + Approach step 1 + sub-issue (2) row
- [x] (suggestion) #249's disposition names #263/#264/#265/#266 (all OPEN, verified 2026-09-17) and the "its one operator comment" claim is corrected — #249 carries nine — `plan.md` Approach step 9
- [x] (suggestion) The #626 gate cites #626 instead of restating its `file:line` refs, with the reason (#626 stamps them "as of `d7b8baf`"; two have already moved) — `plan.md` Approach step 1
- [x] (suggestion) The ADR-0015 addendum is cut to a one-sentence Status pointer plus a References entry; the third-actor argument moves into ADR-0020, and step 2 names which sentence lands where — `plan.md` Approach step 2, Files to Change, ADR Compliance
- [x] (suggestion) Context attribution fixed: the operator's 2026-09-14 comment enumerates four sub-issues; (5) and (6) come from the issue body and the round-1 review — `plan.md` Context
- [x] (suggestion) Credential-surface superlatives dropped for the axis — new stored secrets 2 / 0 / 0, plus where each existing credential already lives (`~/.config/gh/hosts.yml`, `~/.config/ros2-agent/claude-oauth-token`, Anthropic-managed for the Routine) — `plan.md` § Trigger comparison
- [x] (suggestion) The `principles_review_guide.md` ADR-table row is decided: ADR-0020 gets a row (the guide's own Consequences Map requires it); the eight pre-existing gaps (0011, 0012, 0014–0019, verified 2026-09-17) go on `docs/roadmap.md` as a backlog entry, not into this PR. Files to Change now names both edits to that file — `plan.md` Approach step 6, Files to Change, Consequences

### Operator decisions folded in (not review findings)
- [x] Trigger = **weekly Claude Code cloud Routine**, recorded as ADR-0020's Decision with the table retained as its Context; sub-issue (4)'s scope updated (no credential provisioning — the Routine stores no new secret)
- [x] **#249 stays open**; the PR body says `Part of #249` and carries **no** closing keyword for any issue
- [x] Discovery = **expected locations following common conventions**, published in ADR-0020 as an explicit kind → expected-location table framed as a **recommendation that sets expectations, not a requirement** — a project storing a document elsewhere is not in violation and produces no error or finding. No per-repo file, no schema; the synthetic-fixture and sibling-manifest-comparison items are dropped
- [x] **Project-agnosticism (ADR-0003)**: the table is to be justified as generic ROS 2 convention and the two roots named abstractly ("the repo the manifest points at"); `unh_marine_autonomy` / `unh_echoboats_project11` are examples and first instances, never the derivation. (2)'s graceful-absence test is named as what protects a differently-laid-out project
- [x] **Design-collection consistency review** recorded as a discussion item on the umbrella — what was observed, the `unh_marine_autonomy` `VISION.md` objective 2 "Reliable Seafloor Mapping (\"Safety First\")" vs world-store draft #391 case (verified 2026-09-17), and the proposed verdict shape — explicitly **not** a deliverable here, **not** specified in ADR-0020, and no sub-issue filed
- [x] Sibling `rolker/agent_workspace` named as a template source: `docs/ROADMAP.md` and `.agent/scripts/update_roadmap.sh` (verified 2026-09-17, including what the script does), adopted for shape and an explicit-`#N` item format rather than ported

### Notes
- Nothing deferred; all eight findings actioned.
- Not pushed (host performs pushes). The plan is rev 3; the round-2 review's own closing note asked for a third plan review before implementation, and the operator's 2026-09-14 comment says the same.
