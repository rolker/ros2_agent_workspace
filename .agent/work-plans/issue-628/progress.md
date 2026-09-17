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

**Branch**: feature/issue-628 at `407d923` (plan rev 3 head; this entry's own commit follows)
**Addressed**: `## Plan Review` (round 2, 2026-09-14 14:20 -04:00, against plan sha `3c2c78a`) — 2 must-fix + 6 suggestions, all eight actioned
**Commits**: `5576551` `53015cc` `9949fc1` `7e08c77` `ee54bf9` `7fb955c` `cdd9101` `e24a6d7` `4683cfd` `e2b4ae3` `407d923`

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

## Plan Review
**Status**: complete
**When**: 2026-09-17 11:02 -04:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-628/plan.md` at `407d923`
**PR**: PR-less (`--issue` mode, worktree `feature/issue-628`)
**Verdict**: changes-requested

Independent review — dispatched as a fresh-context sub-agent, read the plan
cold; not the plan author. Round 3, against plan rev 3 at `407d923`.

**All eight round-2 findings are genuinely closed in the plan text**, each
re-verified against source rather than taken from the `## Implementation`
entry: the anacron reach cell no longer claims gitcloud reach and now states
what `layers/` actually buys (the two layer-dependent `audit-project` checks,
`audit-project/SKILL.md:152-156,254`, and grading the working tree vs the
manifest-pinned ref); `janitor-sweep` does exclude non-GitHub origins by url
before the rotation (`SKILL.md:273`) and all 44 `url:` entries under
`configs/manifest/repos/` are `github.com`; the discovery question is now in
`plan.md` (Open Questions + Approach step 1 + the (2) row), recorded as
decided; #249's disposition names #263/#264/#265/#266 (**all four OPEN**, as is
#249, verified) and the nine-comment correction is in; the #626 gate cites the
issue instead of restating its `file:line` refs and every item it describes is
open and unchecked in #626; the ADR-0015 addendum is cut to a Status pointer
plus a References entry with the third-actor argument moved into ADR-0020; the
Context attribution now credits the operator with four sub-issues and the issue
body/round-1 review with (5)/(6); the credential column states new-stored-secret
counts (2/0/0) with no superlatives; the `principles_review_guide.md` ADR row is
decided and both edits are named in Files to Change.

Every other checkable claim re-verified and holding: 19 ADRs with 0019 highest
so 0020 is free; no `docs/roadmap.md` and no `.agent/templates/roadmap.md`; the
ADR Applicability table runs 0001–0010 then 0013, so exactly eight rows are
missing (0011, 0012, 0014–0019); `CLAUDE.md`'s References list is at
`CLAUDE.md:23-34`; `check_pr_authors.py:115-127` reads `authors[0]`, the
primary commit author; `research_digest.md:124` carries the Pro 5 / Max 15 /
Team 25 caps with the "floor reported inconsistently" hedge and `:132` the
cloud-hosted/GitHub-scoped line; both workspace and `rolker/agent_workspace` are
PUBLIC, the sibling's default branch is `main`, and `update_roadmap.sh` does
exactly what the plan says (grep for a literal `#<N>` in `ROADMAP.md` /
`docs/ROADMAP.md`, table Status column → `done` or `- [ ]` → `- [x]`, `trap
'exit 0' EXIT`); `unh_marine_autonomy` has no `docs/roadmap.md` and exactly the
eight flat `docs/*.md` pages listed, and its `VISION.md` objective 2 is
`Reliable Seafloor Mapping ("Safety First")`; the BizzyBoat roadmap is 703 lines
with no parent line; #391 and #393 are OPEN; PR #257 MERGED 2026-02-26.

**The six operator decisions named at this checkpoint are all stated correctly
in the plan** and none is re-opened here: umbrella/(1)-only, weekly Claude Code
cloud Routine, `Janitor Sweep Agent`, #249 open with `Part of #249` and no
closing keyword, conventional expected locations published as a table that is a
recommendation and not a requirement with no per-repo file or schema and absence
never an error, project-agnosticism with the two projects as examples only, and
the design-collection consistency review as a discussion item with no sub-issue.
Findings 1 and 2 below are about *how the plan sources* that last pair, not
about the decisions themselves.

### Evaluation
| Dimension | Verdict | Notes |
|---|---|---|
| Scope | Good | Three new documents, five small edits, five issue filings — one PR. Sub-issue boundaries are clean and (3)/(4) work is consistently pushed out of (1). |
| Issue alignment | Good | Every "Deliverables (this repo)" bullet is assigned to a numbered sub-issue; the project-repo bullets survive as (5)/(6) with the umbrella close condition stated. |
| File targeting | Needs work | Finding 3 — the two other framework adapters carry the same References list the plan's own step-7 rationale is built on. |
| Consequences | Needs work | Finding 3; finding 7 (an unstated sub-issue dependency). |
| Documentation & instruction impact | Good | Present and non-silent; instruction items framed as operator-decided candidates, correctly deferred to (3). |
| Principle alignment | Concern | Documentation accuracy: findings 1, 2, 6, 5. |
| ADR compliance | Needs work | ADR-0003 is handled well in substance; ADR-0008 does not support the justification the plan requires (finding 2). ADR-0012 test paraphrased (finding 6). |
| ROS conventions | N/A | Workspace documentation plan. |

### Findings
- [x] (must-fix) **An operator quote in the plan exists nowhere in the record.** Approach step 1 attributes to the operator, dated 2026-09-17: *"Keep in mind that project11 is the project a lot of this is being developed against, but this workspace is meant to be agnostic to the actual project..."*. It is in none of #628's six comments, in no other issue (`gh search issues`), and nowhere in the repo except `plan.md` itself. The **substance is right and is not in question** — it restates ADR-0003 — but the plan tells the ADR author to carry this quote into an Accepted ADR as the justification for the table. Per AGENTS.md § Documentation Accuracy ("never attribute … to a person who didn't state them"), either cite where it was said or replace it with the verifiable 14:54Z #628 comment already quoted two sentences later, plus ADR-0003 — `plan.md` Approach step 1
- [x] (must-fix) **"Generic ROS 2 project conventions" is not a claim any ROS 2 source supports, and the plan requires the ADR to make it.** Approach step 1: the table "is stated as **generic ROS 2 project conventions** … and **must be justified as such in the ADR**". Nothing in the ROS 2 documentation or the REPs locates a `VISION.md`, a `docs/roadmap.md` or an ADR directory; ADR-0008's Decision scopes "ROS 2 official conventions" to naming, packaging, licensing, message design and launch structure from docs.ros.org and the REPs. The honest and equally sufficient justification is *common open-source documentation practice, plus the paths this workspace already uses* — which is what the parenthetical in that same sentence already says. Requiring a ROS-2-convention justification invites a fabricated citation in a durable document — `plan.md` Approach step 1, Principles Self-Check ("Workspace vs. project separation"), ADR Compliance (ADR-0003 row)
- [x] (must-fix) **The framework-adapter fan-out stops one file short — and the rationale given does not distinguish the ones left out.** Step 7 adds the roadmap to `CLAUDE.md`'s References because it "would otherwise be the one References list in the repo that does not name the roadmap". `.github/copilot-instructions.md:112-121` and `.agent/instructions/gemini-cli.instructions.md:81-90` carry the same list, with the same entries, and the Consequences Map row the plan invokes names `.github/copilot-instructions.md` **explicitly** ("`AGENTS.md` → Framework adapters if affected (`.github/copilot-instructions.md`, etc.)"). Either add both rows or state why only the Claude adapter — and note both are instruction files, so the Ask-First approval sentence that currently scopes approval to "exactly these three additions" would have to widen to five — `plan.md` Approach step 7, Files to Change, Consequences table row 3
- [x] (must-fix) **The health row of the kind → expected-location table is not a path, but sub-issue (2) is told to probe "exactly the paths in" that table.** The cell reads "beside the roadmap (`docs/`), named by the sweep" — no filename. The table is defined as the single reader spec so that (2) cannot drift from it; with this row underspecified, (2) must invent the name, which is the drift the single-source design exists to prevent. (1) writes the table and (3) writes the document, so the name has to be settled in (1) or the row has to say explicitly that the name is (3)'s to fix and (2) globs — `plan.md` Approach step 1 (table), sub-issue (2) row
- [x] (suggestion) **#609 is CLOSED** (verified), but Approach step 4 lists it among the "open direction-setting threads" the roadmap draws from. The wording is "#609 follow-ups", which may be intended, but as printed it will read as an open thread. #569, #626, #627 and #610 are all OPEN as claimed — `plan.md` Approach step 4
- [x] (suggestion) **ADR-0012's test is paraphrased inside quotation marks.** Approach step 2 cites *"does this change what was decided?"*. ADR-0012's actual test is "if someone reads only the edited ADR without knowing about the change, will they get a misleading picture of what was originally decided?" The conclusion (addendum, no supersede) is correct under either phrasing — quote the real one or drop the quotation marks — `plan.md` Approach step 2
- [x] (suggestion) **Sub-issue (5) depends on (2) and does not say so.** (5) puts a one-line `kind:` marker on eight pages, but the marker convention and its template ship in (2) — the (6) row states its gate on (3) explicitly, so the asymmetry reads as an omission rather than a decision — `plan.md` Sub-issue sequence table, rows (5) and (2)
- [x] (suggestion) **The new ADR is not said to come from `.agent/templates/adr_template.md`**, which exists and which every prior ADR follows. One clause in step 1 — `plan.md` Approach step 1, Files to Change
- [x] (suggestion) **"Superseding #249" is used unqualified where the operator's decision is that #249 stays open.** Approach step 4 and the Files to Change row both say `docs/roadmap.md` supersedes #249; step 9 correctly explains that it absorbs the direction while #249 remains the Phase-2 tracker. The two readings are reconcilable but the short form is the one that gets pasted into a PR body — say "absorbs #249's direction" where the long explanation is not adjacent — `plan.md` Approach step 4, Files to Change
- [x] (suggestion) **"all 44 `url:` entries" counts a manifest the sweep does not read.** `underlay.repos` holds 9 of the 44; `get_overlay_repos` excludes it, so the set the rotation sees is 35. Every one of both sets is `github.com`, so the conclusion is unaffected — the number is just not the one the claim is about — `plan.md` Trigger comparison ("No mechanism reaches the gitcloud repos")

### Note for the operator checkpoint
Rev 3 is close. The four must-fixes are all one- or two-sentence edits and none
re-opens a decision you have made: two are sourcing problems in the paragraph
that argues project-agnosticism (an operator quote that is not in the record,
and a "ROS 2 convention" justification no ROS 2 source supports — the *point* is
right, ADR-0003 already says it), one is a consequence the plan's own rationale
implies but stops short of (the Copilot and Gemini adapters carry the identical
References list, and the Consequences Map names the Copilot one by name — note
this would widen the Ask-First approval from three additions to five), and one
is a gap in the table that sub-issue (2) is contracted to read literally (the
health row names no file). Nothing in the six decisions you recorded on #628 is
misstated in the plan.


## Implementation
**Status**: complete
**When**: 2026-09-17 11:12 -04:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-628 at `e42cc1e`
**Addressed**: `## Plan Review` round 3 (2026-09-17 11:02 -04:00, plan rev 3 at `407d923`, review entry `5766238`) — verdict changes-requested, 4 must-fix + 6 suggestions
**Commits**: `ee5277c`, `fccae92`, `41d62d6`, `9f4428b`, `405e14d`, `0ff6377`, `d949b75`, `e42cc1e`
**Work product**: `.agent/work-plans/issue-628/plan.md` rev 4, plan head `d949b75`

Plan-only revision per the operator's round-3 decision (2026-09-17 comment on
#628: fix all ten, then a fourth plan review). Nothing in the plan was
implemented. Every claim added was verified against source or against the
GitHub record before it was written; the four suggestions that are one-clause
record corrections were grouped into one commit (`405e14d`) because splitting
four one-line fixes in the same paragraph-set would have produced commits that
cannot be reasoned about independently anyway.

### Actions
- [x] (must-fix 1) Operator quote now cited to its source — the verbatim #628 comment of 2026-09-17 ("Operator note recorded late", issuecomment-5716595167); quote retained, ADR-0003 named as the standing authority — `plan.md` Approach step 1
- [x] (must-fix 2) "Generic ROS 2 project conventions" replaced with "common open-source documentation practice, plus the paths this workspace already uses", in all three places, with an explicit note that it is deliberately *not* a ROS 2 convention (ADR-0008 scopes those to naming/packaging/licensing/messages/launch); new ADR-0008 row in ADR Compliance records the check — `plan.md` Approach step 1, Principles Self-Check, ADR Compliance
- [x] (must-fix 3) Adapter fan-out widened to all three adapters (`CLAUDE.md:23-34`, `.github/copilot-instructions.md:112-121`, `.agent/instructions/gemini-cli.instructions.md:81-90` — same list, verified); Ask-First scope restated as the **five** pointer additions the operator approved in the round-3 comment — `plan.md` Approach step 7, Files to Change (2 new rows), Consequences row 3, Documentation Impact, Open Questions, Estimated Scope
- [x] (must-fix 4) Health row given a literal path, `docs/health.md`, so every row of the kind → expected-location table is a path and sub-issue (2)'s "exactly the table's paths" contract holds; (2) probes it, (3) writes it and cannot rename it without amending the table. No per-repo declaration file added (operator ruled that out) — `plan.md` Approach step 1 table, sub-issue (2) and (3) rows
- [x] (suggestion 5) #609 removed from the open direction-setting threads — CLOSED 2026-08-24 (verified); #569/#626/#627/#610 re-verified OPEN — `plan.md` Approach step 4
- [x] (suggestion 6) ADR-0012's test now quoted as written ("if someone reads only the edited ADR without knowing about the change, will they get a misleading picture of what was originally decided?", ADR-0012 § "How to tell the difference") — `plan.md` Approach step 2
- [x] (suggestion 7) Sub-issue (5)'s gate on (2) for the `kind:` marker format stated explicitly, matching how (6) states its gate on (3) — `plan.md` sub-issue table row (5)
- [x] (suggestion 8) `.agent/templates/adr_template.md` § "Blank ADR template" named as ADR-0020's source structure, extended with Alternatives considered / References (both present in ADR-0019) — `plan.md` Approach step 1, Files to Change
- [x] (suggestion 9) "Superseding #249" replaced with "absorbs #249's direction", consistent with "#249 stays open, `Part of #249`, no closing keyword" — `plan.md` Approach step 4, Files to Change
- [x] (suggestion 10) Overlay set corrected to **35** github.com entries, not 44: `get_overlay_repos` ignores `underlay.repos` (`.agent/scripts/lib/workspace.py:86,106`), which holds 9 of the 44. Conclusion unchanged — `plan.md` Trigger comparison

### Deferred
None — all ten findings were actioned.

### Next
Fourth plan review (`review-plan`) against plan rev 4 at `d949b75`, per the operator's round-3 decision. Not dispatched from here.

## Plan Review
**Status**: complete
**When**: 2026-09-17 11:17 -04:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-628/plan.md` at `d949b75`
**PR**: PR-less (`--issue` mode, worktree `feature/issue-628`, branch head `8a0ee14`)
**Verdict**: approve-with-suggestions

Independent review — dispatched as a fresh-context sub-agent, read the plan
cold; not the plan author. Round 4, against plan rev 4 at `d949b75`.

**All ten round-3 findings are genuinely closed in the plan text**, each
re-verified against source rather than taken from the `## Implementation`
entry:

1. (must-fix) The operator quote is now cited to
   [issuecomment-5716595167](https://github.com/rolker/ros2_agent_workspace/issues/628#issuecomment-5716595167)
   — that comment exists on #628, dated 2026-09-17T15:04:26Z, and carries the
   quoted sentence verbatim; ADR-0003 is named as the standing authority.
2. (must-fix) "Generic ROS 2 project conventions" is gone from all three sites
   (Approach step 1, Principles Self-Check, ADR Compliance), replaced by
   "common open-source documentation practice, plus the paths this workspace
   already uses", with an explicit *not* a ROS 2 convention statement. The
   ADR-0008 scoping claim checks out: 0008's Decision §1 names naming,
   packaging, licensing, message design and launch file structure
   (`0008-follow-ros2-official-conventions.md:29-34`). A new ADR-0008 row in
   ADR Compliance records the check as deliberately not claimed.
3. (must-fix) The adapter fan-out now covers all three named adapters, and the
   cited line ranges are right: `CLAUDE.md:23-34`, `.github/copilot-instructions.md:112-121`,
   `.agent/instructions/gemini-cli.instructions.md:81-90` are each that file's
   `## References` list. The stated pre-existing difference is real — the two
   non-Claude lists omit `README.md` § Vision and `docs/decisions/`. Ask-First
   scope is restated as the five additions the operator approved in
   [issuecomment-5716624829](https://github.com/rolker/ros2_agent_workspace/issues/628#issuecomment-5716624829),
   quoted correctly.
4. (must-fix) Every row of the kind → expected-location table is now a literal
   path, health included (`docs/health.md`), and the (2) and (3) rows are
   consistent with it — (2) probes it, (3) writes it and cannot rename it
   without amending the table.
5. (suggestion) #609 is CLOSED (verified) and the plan now says so explicitly
   instead of listing it as an open thread; #569, #626, #627, #610 re-verified
   OPEN.
6. (suggestion) ADR-0012's test is now quoted as written — matches
   `0012-permit-cross-reference-addendums-in-adrs.md:62-64` word for word.
7. (suggestion) Sub-issue (5)'s gate on (2) is stated, symmetric with (6)'s
   gate on (3).
8. (suggestion) `.agent/templates/adr_template.md` § "Blank ADR template"
   (line 62) is named as the source structure; its four sections are
   Status/Context/Decision/Consequences as the plan says, and ADR-0019 does
   carry both `## Alternatives considered` (:119) and `## References` (:136).
9. (suggestion) "Superseding #249" is gone; the plan says "absorbs #249's
   direction" in both places, with the long explanation at step 9. No
   closing-keyword token appears anywhere in the plan text (grepped).
10. (suggestion) The count is now 35, and it is correct: the seven files under
    `configs/manifest/repos/` hold 44 `url:` entries, `underlay.repos` holds 9,
    and `get_overlay_repos` skips it unless `include_underlay`
    (`.agent/scripts/lib/workspace.py:86,106`). All 44 are `github.com`, so the
    conclusion holds either way.

**Other claims re-verified this round and holding**: 19 ADRs, 0019 highest, so
0020 is free; exactly three are still `Proposed` (0005, 0008, 0009); no
`docs/roadmap.md`, no `docs/health.md`, no `.agent/templates/roadmap.md`;
`README.md:3` is `## Vision`; the ADR Applicability table runs 0001–0010 then
0013, so exactly eight rows are missing (0011, 0012, 0014–0019); the
Consequences Map row invoked by step 7 is `principles_review_guide.md:44` and
does name the Copilot adapter explicitly; `check_pr_authors.py:112-127` reads
`authors[0]`, the primary commit author; `research_digest.md:124` carries the
Pro 5 / Max 15 / Team 25 caps with the "floor reported inconsistently" hedge
and `:132` the cloud-hosted / GitHub-repo-scoped line; `audit-project/SKILL.md:152-156`
and `:252-255` are the two SKIPPED-not-OK layer-dependent checks;
`janitor-sweep/SKILL.md:273` excludes non-GitHub origins via `is_field_url`
before the rotation, and `:38` carries the no-`layers/` assumption;
`worktree_create.sh:387` has `ALLOWED_SKILLS=("research" "inspiration-tracker")`,
so (3)'s addition is needed; ADR-0015 has both a `## Status` section (already
carrying an ADR-0019 qualifier, so the addendum matches existing practice) and
a `## References` section, and its "nothing publishes from inside the sandbox"
line is at `:19`; #626 is OPEN with every redaction-class item unchecked and
stamped "as of `d7b8baf`", exactly as the plan describes; PR #257 MERGED
2026-02-26 (created 2026-02-25); #249 OPEN with 9 comments and #263/#264/#265/#266
all OPEN; `unh_marine_autonomy` has `VISION.md` last touched 2026-01-15, no
`docs/roadmap.md`, and exactly the eight flat `docs/*.md` pages listed;
`unh_echoboats_project11`'s `docs/roadmap.md` is 703 lines with "How this
roadmap stays useful" at :683 and no parent line; `rolker/agent_workspace` is
public, default branch `main`, and carries `docs/ROADMAP.md` and
`.agent/scripts/update_roadmap.sh`; uma #391 and #393 are OPEN.

**All the operator decisions named for this round are stated correctly in the
plan and none is re-opened here**: umbrella with this worktree = (1) only;
weekly Claude Code cloud Routine; `Janitor Sweep Agent`; #249 open with
`Part of #249` and no closing keyword; discovery = conventional expected
locations published as a kind → location table that is an expectation and not
a requirement, no per-repo file, no schema, absence never an error, health =
`docs/health.md`; framed as common open-source documentation practice and not
as a ROS 2 convention, project-agnostic per ADR-0003 with the two project
repos as examples; the design-collection consistency review as a discussion
item with no sub-issue; Ask-First = the five approved pointer additions.

### Evaluation
| Dimension | Verdict | Notes |
|---|---|---|
| Scope | Good | Three new documents, eight small edits, five issue filings — one PR. (3)/(4) work is consistently pushed out of (1). |
| Issue alignment | Good | Every "Deliverables (this repo)" bullet maps to a numbered sub-issue; the project-repo bullets survive as (5)/(6) with a stated umbrella-close condition. |
| File targeting | Good | All ten rows in Files to Change verified to exist with the cited line ranges. Finding 1 is a completeness note about a fourth adapter, not a wrong target. |
| Consequences | Good | Consequences Map rows all traced to `principles_review_guide.md`; cross-sub-issue consequences ((2)'s script row, (3)'s exception-clause rewrite) are recorded where they land. |
| Documentation & instruction impact | Good | Present and non-silent; the one instruction-level rule is framed as a candidate and correctly deferred to (3). |
| Principle alignment | Good | The documentation-accuracy problems that drove round 3 are fixed at source; every quote in rev 4 is now traceable. |
| ADR compliance | Good | 0001/0003/0004/0005/0006/0008/0012/0013/0014/0015/0017/0018/0019 all addressed; the 0004/0005 deferral is stated rather than silent. |
| ROS conventions | N/A | Workspace documentation plan. |

### Findings
- [x] (suggestion) **A fourth framework adapter exists and carries its own References list.** `AGENTS.md:11` lists `.agent/AGENT_ONBOARDING.md` as the "Other" adapter, and it has a `## References` list at `:109`. Step 7 says "all three framework adapters" and "each adapter that carries one" as if that enumerated the set, which is the same gap round 3 flagged one adapter further out. The operator's approval caps the work at five additions, so the fix is one clause naming AGENT_ONBOARDING.md as knowingly out — and there is a real distinguishing reason to give: its list is a different, shorter one (it also omits `docs/decisions/` and `.agent/knowledge/`), unlike the three that are identical entry-for-entry — `plan.md` Approach step 7, Files to Change, Documentation & Instruction Impact
- [x] (suggestion) **The chosen Routine row does not answer the cross-repo *write* question the Actions row is charged for.** Sub-issue (3) commits a health document into each repo it grades, and the Actions row is scored 2 new secrets precisely because "the built-in `GITHUB_TOKEN` is scoped to the repo running the workflow". The Routine row answers only that no *new* secret is stored, with no statement of whether the operator's existing GitHub connection reaches the 35 repos a health document would be committed to — and `research_digest.md:132` calls Routines "GitHub-repo-scoped". The decision is the operator's and is not re-opened; this is the same "confirm before relying on it" caveat the (4) row already carries for the daily run cap, applied to the axis the choice was actually made on — `plan.md` Trigger comparison (Routine row, credential-surface cell), sub-issue (4) row

### Note for the operator checkpoint
Rev 4 converges. All four must-fixes and all six suggestions from round 3 are
closed in the plan text, verified against source rather than against the
implementation entry, and nothing you decided on #628 is misstated. The two
items above are both suggestions and neither blocks implementation: one is a
one-clause completeness note (a fourth adapter file exists — `AGENT_ONBOARDING.md` —
and the plan should say it is deliberately out rather than imply three is the
whole set; no sixth edit is proposed), and one asks the trigger table to say
whether the cloud Routine's GitHub connection actually reaches the 35 repos it
will have to commit health documents into, which is the axis the trigger was
chosen on. Either can be folded in during implementation or waived.

## Implementation
**Status**: complete
**When**: 2026-09-17 11:35 -04:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: `feature/issue-628` at `74699fd`
**Addressed**: the `## Plan Review` entry of 2026-09-17 11:17 -04:00 (round 4,
against plan rev 4 at `d949b75`, verdict approve-with-suggestions), plus the
operator's decision comment "Operator decision at the plan-review checkpoint,
round 4" on [#628](https://github.com/rolker/ros2_agent_workspace/issues/628)
(2026-09-17).
**Commits**: `bcbf666`, `ccf9cbb`, `74699fd`

Plan rev 5 at `74699fd`. Plan only — nothing implemented. The operator's
round-4 instruction was "reframe deliverable 1 as a design draft, then
implement", with no further plan-review round, so this revision is the last
plan pass before implementation.

### What rev 5 changes

1. **Deliverable (1) is a design draft, not ADR-0020** (`bcbf666`). The page is
   `docs/design/planning_document_vocabulary.md` with `Status: Draft`, creating
   `docs/design/` — `docs/` today holds only `PRINCIPLES.md` and `decisions/`
   (verified 2026-09-17). The content plan is what ADR-0020 was going to say;
   the document kind, its status, and the record it lands in are what changed.
   Deliberately **not** structured from the blank ADR template, which would
   produce an ADR in everything but the directory. *Design draft* is a
   supporting kind in the vocabulary, so the expected-location table prescribes
   no path for it and `docs/design/` is stated as a local choice, not a claim
   the table makes — the page is its own first worked example of the kind it
   defines.
2. **New sub-issue (7): promote the draft to an ADR**, gated on (2), (3) and at
   least one ROS 2 project other than `unh_marine_autonomy` having exercised
   the two-root rule. Numbered last rather than inserted, because (5) and (6)
   are referred to by number in the operator's comments and in the round-3/4
   review entries; (7) runs last anyway. It carries the three items deferred
   out of (1): the ADR-0012 addendum on ADR-0015, the `principles_review_guide.md`
   ADR Applicability row, and the resolution of the draft's two open table rows.
3. **The kind → expected-location table is provisional** (`ccf9cbb`), cited to
   the 2026-09-17 documentation-conventions survey. The draft carries a
   condensed evidence section with the survey's **source URLs**, because the
   survey file is under `.agent/scratchpad/`, which is gitignored
   (`.gitignore:40`, verified) and cannot be cited by path. The table gains a
   per-row status column: **vision open** (a `README.md` § Vision section has
   more evidence behind it — no surveyed ROS 2 project has a `VISION.md`, and
   this workspace itself uses one); **roadmap open three ways**
   (`docs/roadmap.md` / root `ROADMAP.md` / an external board or docs site);
   **decisions settled** (MADR literally recommends `docs/decisions/`);
   **health = this workspace's own choice**, with REP-2004's per-package
   quality declaration named as the nearest ROS 2 analogue and explicitly not a
   precedent. The survey also supplies the *positive* evidence for rev 4's
   "deliberately not a ROS 2 convention" statement, which rev 4 could only
   assert: REP-2004 is the only REP touching documentation and never mentions
   roadmaps, visions or ADRs.
4. **Sub-issue (2) gains discovery fallbacks for externally hosted roadmaps** —
   org-level GitHub Projects (`gh api orgs/<org>/projects`), the repo
   `homepage` field, and README links matching "roadmap"/"governance". Stated
   as extensions of *discovery* only: never a requirement, never a check, and
   finding nothing still behaves exactly as today.
5. **Dropped from this slice**: the ADR-0015 Status pointer + References entry,
   and the `principles_review_guide.md` ADR Applicability row. The addendum
   drops for a reason of its own — ADR-0012's permitted class is an ADR
   pointing at another **ADR**, and pointing an accepted ADR's Status line at a
   `Draft` page would put a provisional document into the binding record. The
   third-actor argument (a scheduled trigger has no host session and *is* the
   publisher) stays in the draft, which is the document making it. ADR-0012 is
   therefore recorded as **not triggered** in this slice — this PR edits no ADR
   text at all.
6. **Consequences checked against the guide's own table, not assumed**: the
   Consequences Map (`principles_review_guide.md:42-53`) has **no row for
   adding a prose page under `docs/`** that is neither a principle nor an ADR,
   so the design draft triggers no existing row. What does apply is the
   template row (step 3 adds `.agent/templates/roadmap.md`) and the `AGENTS.md`
   row (step 7's adapter fan-out). Stated in Consequences rather than left as a
   silent absence; the row this PR *adds* closes the part of the gap this work
   creates.
7. **ADR Compliance updated for a plan that adds no ADR**: ADR-0001 is
   triggered and answered with a deliberate deferral of *binding status* (not
   of capture), with (7) as the named closer and its gate stated; ADR-0012
   becomes "not triggered"; ADR-0015/0019 become "cited, not edited". ADR-0003
   (project-agnostic), ADR-0008 ("checked and deliberately not claimed") and
   ADR-0004/0005 (enforcement deferred, one step further now that the rules are
   draft text) are kept and restated for the draft framing.
8. **Pointer rows: still five, pointing at both new planning documents.** Rev
   4's five additions named `docs/roadmap.md` alone because the other new
   document was an ADR and `docs/decisions/` is already in those lists; the
   draft is a new page in a new directory no list names. The plan flags
   explicitly that it reads the operator's "five" as five *pointer additions*
   (one References block per file, two in `AGENTS.md`), not five *lines* —
   because it is the operator's number, not the plan's.
9. **Every prior operator decision left intact**: trigger = weekly Claude Code
   cloud Routine; `Janitor Sweep Agent` on `skill/janitor-*`; #249 stays open
   with `Part of #249`; no closing keywords; discovery by conventional
   locations with no per-repo file and no schema; project-agnostic ADR-0003
   framing with the two project repos as examples only; the design-collection
   consistency review as a discussion item with no sub-issue filed.

### Actions
- [x] (suggestion) A fourth framework adapter exists and carries its own References list — `.agent/AGENT_ONBOARDING.md` named as knowingly left out in Approach step 7, Files to Change and Documentation & Instruction Impact, with the two distinguishing reasons: the five-addition Ask-First cap is fully spoken for, and its References list is a different, shorter one (`AGENTS.md`, `AI_IDENTITY_STRATEGY.md`, `WORKFORCE_PROTOCOL.md`, `WORKTREE_GUIDE.md`, `ARCHITECTURE.md`, project `.agents/README.md` — no `docs/decisions/`, no `.agent/knowledge/`, no `README.md` § Vision), where the other three are identical entry for entry. Verified: `AGENTS.md:11`, `.agent/AGENT_ONBOARDING.md:109` — `74699fd`
- [x] (suggestion) The chosen Routine row does not answer the cross-repo *write* question the Actions row is charged for — the credential-surface cell now carries "confirm the operator's GitHub connection reaches all 35 overlay repos, with write access, before relying on the Routine for cross-repo writes", and sub-issue (4)'s scope carries the same as one of two things to confirm before relying on the mechanism. Verified: `.agent/knowledge/research_digest.md:132` does describe Routines as "cloud-hosted and GitHub-repo-scoped". The decision itself is not re-opened — `74699fd`

### Also fixed, found while revising (not a round-4 finding)
- **A live GitHub closing-keyword token in the plan text.** Rev 4's Open
  Questions opened a bullet with the verb *close* immediately before `#249`,
  which is a live closing token: pasted into a PR body it would have
  auto-closed issue #249 — the exact hazard AGENTS.md § Issue-closing keywords
  names for plan text, and the operator's standing "no closing keywords
  anywhere" decision. The round-4 review's grep reported the plan clean and
  missed it. Reworded in `bcbf666`; the whole plan re-grepped clean for
  `(close|fix|resolve)[sd]? #<N>` forms.

### Not done / open
- **Nothing is implemented.** Plan only, per the task. No draft page, no
  roadmap, no template, no sub-issues filed.
- **The draft's two open table rows (vision, roadmap) are open by design**, not
  an oversight — the survey leaves them at a genuine split and the draft
  publishes them flagged. They resolve at (7).
- **The "five pointer additions = five files-worth of References blocks"
  reading** is the plan's, flagged in step 7 and in Open Questions so the
  operator can correct it cheaply if they meant five lines.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-17 12:05 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-628 at `1545690`
**Mode**: pre-push
**Depth**: Deep (reason: 1609 changed lines, 11 files, governance/instruction files in the diff)
**Must-fix**: 2 | **Suggestions**: 7
**Round**: 1 | **Ship**: continue — both must-fixes are mechanical, but one is a live GitHub auto-close token that must not reach `main`

**Specialists**: Static Analysis (pre-commit, clean; no Markdown linter profile — content review only), Governance, Plan Drift, Claude Adversarial Lens A + Lens B, plus a fact-verification pass over every issue state, ADR status, link and cited URL. Copilot and Local Adversarial: off (default).

### Findings
- [x] (must-fix) HEAD commit body contains a live closing keyword naming #249 while describing the reword that removed it; merged to `main` this closes the issue the operator decided stays open — fix with `git commit --amend` on HEAD (the sha is cited nowhere, so no re-remap) — `commit 1545690` message body, line 3 (fixed by the host before this pass: the reworded commit is `1545690`; the branch carries no closing-keyword token)
- [x] (must-fix) As-built says "seven rows in `ROADMAP.md`" carry `#TBD — filed at publish`; there are six (`ROADMAP.md:58-63`), so the total is 9, not 10 — `.agent/work-plans/issue-628/plan.md:351`
- [x] (suggestion) The template's Health-document line shows a live `docs/health.md` link while its own parenthetical says to omit the link until one exists — copy-paste propagates a dangling link — `.agent/templates/roadmap.md:36`
- [x] (suggestion) The "five pointer additions" accounting is self-inconsistent — `AGENTS.md` counted as two *lines*, each adapter as one *block* though each landed two lines; the shipped diff is 2 lines × 4 files — `.agent/work-plans/issue-628/plan.md:129,290`
- [x] (suggestion) Kinds table says the roadmap is edited "On its own loop (see below)", but no section on this page describes a roadmap's own cadence — it lives in the template — `docs/design/planning_document_vocabulary.md:46`
- [x] (suggestion) "the same convention the `research` skill already uses" reads as available today; `janitor-sweep` is not in `worktree_create.sh`'s `ALLOWED_SKILLS` (tracked in sub-issue (3), not flagged inline) — `docs/design/planning_document_vocabulary.md:268`
- [x] (suggestion) Two-root rule uses binding phrasing ("permitted to presume") while the header says nothing here binds; one hedge would settle the tension — `docs/design/planning_document_vocabulary.md:60-68`
- [x] (suggestion) "Roadmaps beneath this one: none in this repo" is followed by forward-looking narrative; the template wants this as a short structural field — `ROADMAP.md:10-15`
- [x] (suggestion) README § Documentation "For All Users" lists `ARCHITECTURE.md` and `AGENTS.md` but not `ROADMAP.md` — outside the operator-approved Vision-pointer scope, so raise rather than edit — `README.md:77-80`

### Verified clean
- Ask-First scope: exactly two References lines added in each of `AGENTS.md`, `CLAUDE.md`, `.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`; no other hunk in those files.
- Attribution: all five operator quotes verbatim against the #628 comments; the cited permalink is the comment carrying the quoted text; no decision attributed beyond what the comments say.
- Facts: every issue state (#249 #263 #264 #265 #266 #569 #609 #610 #626 #627 #628), the "nineteen ADRs" count, the three still-`Proposed` ADRs (0005/0008/0009), the ADR-applicability gap (0011, 0012, 0014–0019 absent), the 35-repo count, and every cited external URL against the host survey — all match source.
- Links: every relative link and anchor in the three new documents resolves from its own file's location; `ROADMAP.md` is 138 lines (inside the 150-line bound).
- SHAs: all 28 commit shas cited in `plan.md`/`progress.md` resolve; the post-rebase remap is complete.
- Placeholders: 9 `#TBD — filed at publish` occurrences, none malformed.
- Project-agnosticism (ADR-0003): `unh_marine_autonomy` / `unh_echoboats_project11` appear only as worked examples.
- The design-collection consistency review appears once, as a pointer to the #628 discussion, unspecified.
- "Published expectation, not a requirement; absence is never a finding" is held consistently, including in the new Consequences Map row.

## Implementation
**Status**: complete
**When**: 2026-09-17 12:14 -04:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-628 at `73fd775`
**Addressed**: `## Local Review (Pre-Push)` round 1, 2026-09-17 12:05 -04:00, verdict changes-requested (2 must-fix + 7 suggestions), reviewed at `1545690`
**Commits**: 818640c, 52ca2ec, f873977, 00fc7d7, bb1e85d, 419e2a9, 2a47d14, 7aba3d7, 73fd775

All nine open actions are fixed in this PR — none filed as a follow-up
issue, per the operator's standing rule that review-found small defects
ride the PR they were found in.

### Actions
- [x] (must-fix) The commit body carrying a live closing keyword for #249 — the
  host reworded that commit before this pass (`9e00462` → `1545690`), and
  `git log main..HEAD` now matches no closing-keyword token. The review entry
  cited the old sha twice — as the branch head reviewed and as the offending
  commit — and neither resolved any more, so both citations were repaired to
  `1545690` and the finding's own text de-tokenised (it quoted the keyword
  verbatim). Correcting a dangling sha citation inside the entry being checked
  off is the established repair on this branch: progress.md is append-only for
  *entries*, not for a reference that has stopped resolving — `818640c`
- [x] (must-fix) As-built placeholder count — six rows in `ROADMAP.md`
  (`ROADMAP.md:58-63`, one per unfiled sub-issue) plus three mentions in the
  design draft, nine in all, not seven plus three — `plan.md:350-355`, `52ca2ec`
- [x] (suggestion) Roadmap template shipped a live `docs/health.md` link while
  telling the author to omit it; the line is now a `<...>` placeholder in the
  same form as the two fields above it — `.agent/templates/roadmap.md:36-40`,
  `f873977`
- [x] (suggestion) Five-pointer accounting made consistent: both passages now
  state the shipped shape, two lines in each of four files, eight in all —
  `plan.md:129,290`, `00fc7d7`
- [x] (suggestion) Kinds table's "On its own loop (see below)" now cross-refers
  to the roadmap template's *How this roadmap stays useful*, the section that
  actually describes the cadence — `planning_document_vocabulary.md:46`,
  `bb1e85d`
- [x] (suggestion) The `research`-skill convention sentence now states that
  `worktree_create.sh`'s `ALLOWED_SKILLS` holds `research` and
  `inspiration-tracker` only (verified 2026-09-17) and that adding
  `janitor-sweep` belongs to the sweep-split sub-issue — worded without a new
  `#TBD` token, so the placeholder count stays at nine —
  `planning_document_vocabulary.md:268-275`, `419e2a9`
- [x] (suggestion) Two-root rule softened from "is permitted to presume" to "a
  skill *may* assume these two roots — and, on this draft's proposal, only
  these two", keeping the limiting sense while matching the header's
  non-binding framing — `planning_document_vocabulary.md:66-69`, `2a47d14`
- [x] (suggestion) "Roadmaps beneath this one" is now the answer alone, with the
  forward-looking narrative as its own paragraph below and a `<br>` on the
  parent line so the two render as a field pair; `ROADMAP.md` is 139 lines,
  inside the 150-line bound — `ROADMAP.md:9-16`, `7aba3d7`
- [x] (suggestion) `README.md` § Documentation "For All Users" now names
  `ROADMAP.md` beside `ARCHITECTURE.md` and `AGENTS.md`. Fixed rather than
  raised: `README.md` is not an Ask First file, and the operator's rule is to
  fix review-found defects in the PR. The Ask-First scope across `AGENTS.md`
  and the three adapters is untouched, and the plan's as-built notes record the
  second `README.md` pointer — `README.md:79`, `73fd775`

### Checks
- `pre-commit run --files <changed>` from `/home/roland/project11/.venv` on every
  commit: all hooks pass, none skipped for cause. No code changed, so no package
  tests apply.
- Re-verified after the last fix: nine `#TBD — filed at publish` placeholders
  intact (six in `ROADMAP.md`, three in the draft); `ROADMAP.md` 139 lines;
  `git log main..HEAD` carries no closing-keyword token; the new
  `../../.agent/templates/roadmap.md` link resolves from `docs/design/`.

### Noted, not actioned
- Four **older** progress.md entries (the issue-review and plan-review rounds,
  lines 38, 68, 85, 185) quote closing-keyword phrasing verbatim in their own
  finding text. They pre-date this pass, they are historical appended entries,
  and progress.md is not a surface GitHub's parser reads — but plan or progress
  text pasted into a PR body inherits the hazard, so the publish step should
  scrub from the plan/PR body, not copy these lines.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-17 12:19 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-628 at `acf89a3`
**Mode**: pre-push
**Depth**: Deep (reason: 1740 changed lines, 11 files, governance/instruction files in the diff)
**Must-fix**: 2 | **Suggestions**: 0
**Round**: 2 | **Ship**: recommended — both must-fixes are one-token record corrections in the plan's as-built notes, produced by the last round-1 fix shifting `ROADMAP.md` by a line; fix them and ship rather than spend another round

**Specialists**: Static Analysis (pre-commit over the changed files, clean; no Markdown linter profile — content review only), Governance + Scope + Attribution, Claude Adversarial Lens A (facts, links, internal consistency) — both sub-specialists returned **no findings**. Copilot and Local Adversarial: off (default).

### Findings
- [x] (must-fix) As-built size check still says `ROADMAP.md` is 138 lines; it is 139 after the round-1 structural fix (`7aba3d7`) — still inside the 150-line bound, but the recorded measurement is wrong — `.agent/work-plans/issue-628/plan.md:356`
- [x] (must-fix) As-built cites the six placeholder rows as `ROADMAP.md:58-63`; the same fix shifted them to `ROADMAP.md:59-64` — `.agent/work-plans/issue-628/plan.md:360`

### Round-1 findings re-verified against source
All nine round-1 items are genuinely resolved, each checked against the file rather than the checkbox:
- No commit message in `main..HEAD` puts a closing keyword before an issue number (re-grepped over full commit bodies).
- Placeholder accounting corrected: nine `#TBD — filed at publish` occurrences, six in `ROADMAP.md` and three in the draft.
- `.agent/templates/roadmap.md:36-40` — Health-document line is now a `<...>` placeholder, no live link.
- `plan.md:129,290` — pointer accounting states one uniform shipped shape, two lines in each of four files.
- `planning_document_vocabulary.md:46` — the kinds table's roadmap loop cross-refers to the template's *How this roadmap stays useful*.
- `planning_document_vocabulary.md:268-276` — states plainly that `ALLOWED_SKILLS` holds `research` and `inspiration-tracker` only and that adding the sweep belongs to the sweep-split sub-issue.
- `planning_document_vocabulary.md:66-69` — two-root rule now hedged to match the non-binding header.
- `ROADMAP.md:9-16` — parent/children render as a short field pair, narrative moved below.
- `README.md:79` — `ROADMAP.md` now named in § Documentation "For All Users".

### Verified clean
- **Scope**: exactly two References lines added in each of `AGENTS.md`, `CLAUDE.md`, `.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`; no other hunk in any of the four. `README.md` carries the two approved pointer lines; `principles_review_guide.md` the single Consequences Map row.
- **Attribution**: every operator quote in the draft and the plan matches the #628 comments verbatim, with the right dates; the cited permalink resolves to the comment carrying its quoted text; nothing attributed beyond what was said.
- **Facts**: nineteen ADRs, the three still-`Proposed` ADRs (0005/0008/0009), the ADR-applicability gap (0011, 0012, 0014–0019), `ALLOWED_SKILLS`, and every issue state cited (#249 #263 #264 #265 #266 #569 #609 #610 #626 #627 #628, plus the two cross-repo `unh_marine_autonomy` issues) all match source.
- **Links**: every relative link and in-page anchor in the three new documents resolves from its own file's location; `README.md#vision` exists.
- **SHAs**: every 7-hex commit sha cited in `plan.md`/`progress.md` resolves; the only other hex strings are GitHub comment IDs inside permalinks.
- **Project-agnosticism (ADR-0003)**: the kinds table, the two-root rule and the template are stated generically; the two project repos appear only as labelled worked examples.
- **"Published expectation, not a requirement; absence is never a finding"** holds in the draft, the roadmap, the template and the new Consequences Map row.
- `ROADMAP.md` is 139 lines, inside the 150-line bound. Pre-commit over all changed files: clean.

## Integrated Review
**Status**: complete
**When**: 2026-09-17 13:38 -04:00
**By**: Claude Code Agent (Claude Opus)

**PR**: #638 at `0829d73`
**Sources**: 3 (Copilot R1 @ `92526cf`, Copilot R2 @ `0829d73` = head, Local Review (Pre-Push) @ 2026-09-17 12:19; CI rollup)
**Cross-source confirmations**: 1
**CI**: all-pass (9 checks green at head; `copilot-pull-request-reviewer`, Lint, Script tests, Validate Documentation, commit-identity Mechanism C)

Copilot R1 was submitted against `92526cf`, one commit behind head; every R1 item kept below was re-raised at head in R2, so nothing here rests on a stale read. Two R1 items were superseded by `2156bbb` and are not carried.

### Findings
- [x] (cross-confirmed, must-fix) As-built size check records `ROADMAP.md` as 139 lines "inside step 4's 150-line first-cut bound"; it is **150** — at the bound, not inside. Third recurrence of the same defect: the Pre-Push review already flagged it as must-fix (138 → 139) after `7aba3d7`, and `2156bbb` staled it again. Fix the number **and** the form — AGENTS.md § Documentation Accuracy forbids hand-typed measured values in durable artifacts, so state the bound and that the count is re-measured at publish rather than freezing a new one — `.agent/work-plans/issue-628/plan.md:356`
- [x] (must-fix, Copilot R2 inline) The final as-built note still says the six `ROADMAP.md` rows and three draft mentions carry `#TBD — filed at publish`, that the host has not filed the sub-issues, and that these are "the only `#TBD` tokens in the tree". `92526cf` replaced all nine with real links (#634–#637, `unh_marine_autonomy#394`, `unh_echoboats_project11#492`); no `#TBD` survives outside this progress timeline. The plan-first workflow requires the as-built record to match the branch. Rewrite in the past tense as a publish-time staging note — `.agent/work-plans/issue-628/plan.md:377-383`
- [x] (must-fix, Copilot R1+R2, 3 inline comments) The one existing writer of workspace roadmap items is pointed at a path that does not exist: `inspiration-tracker` appends accepted findings to the "To Consider" section of `docs/ROADMAP.md`. Verified: `docs/ROADMAP.md` is absent from `origin/main` and is not created here, and root `ROADMAP.md` has no "To Consider" section. The dangling target **pre-dates this PR** — nothing ever created that file — so this is not a regression, but this PR is what makes the canonical target exist, and leaving the writer aimed elsewhere contradicts the PR's own Consequences Map row and the draft's claim that the kinds table is the single source. `.claude/skills/` is not an Ask-First file, so it rides this PR. Repoint all three lines at root `ROADMAP.md` and name an existing section — **Deferred** is the semantic match ("Leftovers land here — things that came up, are not bounded enough for an issue, and are not being done next") — `.claude/skills/inspiration-tracker/SKILL.md:227,250,258`
- [x] (low, Copilot R1+R2) Kind label mismatch: the vocabulary table names the core kind `decision` (singular, line 47), the expected-location table labels the row `decisions` (line 114). #634 is contracted to derive the kind marker from this table, so the canonical value is ambiguous. Keep the kind singular; the **path** stays `docs/decisions/` — `docs/design/planning_document_vocabulary.md:114`
- [x] (low, Copilot R1+R2) "No surveyed ROS 2 project has a `VISION.md` … the alternative rested on a single instance" sits 100 lines below the draft's own statement that `unh_marine_autonomy` states its purpose in a `VISION.md` today. The survey (lines 160-199) is entirely **external** projects, so the sentence is defensible on its own terms, but as written it reads as self-contradiction and undercounts: two in-house instances exist (`rolker/agent_workspace`, untouched since January; `unh_marine_autonomy`, which unh_marine_autonomy#394 folds into README § Vision). Scope the claim to surveyed external projects and name both instances. Same sentence copied into the plan — `docs/design/planning_document_vocabulary.md:204-208` and `.agent/work-plans/issue-628/plan.md:103`
- [x] (low, Copilot R2) The read-both-documents rule is stated unconditionally in the draft and in the template, while the draft makes a missing health document normal and **no `docs/health.md` exists anywhere yet**. The workspace instance already qualifies it ("once one exists", `ROADMAP.md:135`); the template does not, so every project root instantiated from it points agents at a file that is not there. The Pre-Push review flagged the adjacent instance of this same class (the template's live `docs/health.md` link against its own omit-until-it-exists parenthetical). Qualify both to match the instance — `docs/design/planning_document_vocabulary.md:330-333` and `.agent/templates/roadmap.md:131-133`
- [x] (low, Copilot R1) The #634 external-discovery fallback specifies `gh api orgs/<org>/projects` — the retired Projects (classic) REST endpoint. Verified against the live API with host auth: `gh api orgs/ros2/projects` returns HTTP 404. Projects v2 has no REST listing, so the fallback as written would discover nothing. Use `gh project list --owner <org>` (GraphQL) and drop the parenthetical implying both REST forms are worth probing — `.agent/work-plans/issue-628/plan.md:56`
- [x] (low, Copilot R2) The identity form is rendered as inline code `git -c user.name/user.email`, which is not runnable git syntax — `git -c` requires `key=value`. This is established shorthand elsewhere in the repo (issue-470's plan and progress), so it is not wrong as prose, but on a **published** design page inline-code formatting reads as a command. Render it `git -c user.name=… -c user.email=…` or drop the backticks; same string in the plan's trigger table, fixed in the same pass — `docs/design/planning_document_vocabulary.md:293` and `.agent/work-plans/issue-628/plan.md:154`
- [x] (low, Copilot R1) "The conventional-path reader (#634) makes the two-root rule checkable" overstates the planned work: #634 discovers documents and validates nothing — its own plan row makes absence never a finding and adds no checks. Discovery is the precondition for a check, not the check. Restate as making the documents the rule talks about discoverable, leaving enforcement to a scoped follow-up — `docs/design/planning_document_vocabulary.md:344`
- [x] (low, Copilot R1+R2) "The skills simply do not find it and behave exactly as they do today" is narrower than #634's own planned behaviour, which adds discovery-only fallbacks (org board, `homepage`, README links) that may surface a document outside the expected paths. The guarantee that matters — **absence is never a finding** — is untouched by the fallbacks, so one clause separating the canonical in-repo probe from optional non-requirement discovery settles it before #634 lands and stales the sentence — `docs/design/planning_document_vocabulary.md:120-127`

### False positives
- (Copilot R2) "The final progress entry repeats the stale 139-line measurement; correct this verification record" — `progress.md` is an append-only dated timeline (ADR-0013). That entry was accurate against its own head SHA (`92526cf`); `2156bbb` changed the file afterwards. Editing a past entry to match a later commit falsifies the record rather than correcting it — the current measurement belongs in the next entry, which is this one. Structural, not a priority judgement. — `.agent/work-plans/issue-628/progress.md:766`
- (Copilot R2) "The health/roadmap consequence forces a parent-roadmap update on every routine `docs/health.md` refresh" — the failure mode cannot occur: the row states its own trigger ("a **new or moved child roadmap** leaves the parent's list wrong"), and the template's structural field (`.agent/templates/roadmap.md:33-34,66`) admits only parent and child **roadmaps**. A health document never appears in any parent's list, so a wholesale health rewrite cannot reach the consequence. A one-clause tightening is optional if the row is touched anyway. — `.agent/knowledge/principles_review_guide.md:47`

### Notes
- Nothing in this round touches the operator decisions recorded on #628 (design draft not an ADR; conventional paths with no per-repo file; root `ROADMAP.md`; README § Vision; `docs/health.md`; project-agnostic per ADR-0003; two References lines per instruction file; the consistency review as a discussion item only). No finding was accepted that would revisit one.
- All ten open findings are documentation-accuracy or consumer-consistency fixes confined to files already in this PR, plus one three-line change to `.claude/skills/inspiration-tracker/SKILL.md`. Per the operator's standing rule, review-found small defects are fixed in this PR rather than filed.

## Implementation
**Status**: complete
**When**: 2026-09-17 13:53 -04:00
**By**: Claude Code Agent (Claude Opus)

**PR**: #638 at `883b7f3`
**Addressed**: `## Integrated Review` (2026-09-17 13:38 -04:00, at `23b874f`) — all ten open findings, none deferred
**Commits**: `b1b3882` `82e74ce` `cd1210e` `371c891` `c7f58d6` `919b0f2` `534cd38` `0e8f5b4` `3e08c76` `883b7f3`

### Actions
- [x] (cross-confirmed, must-fix) As-built size check: both hand-typed counts replaced with the bound plus a statement that the count is re-measured (`wc -l`) at publish, per AGENTS.md § Documentation Accuracy — `.agent/work-plans/issue-628/plan.md:356,372` (`b1b3882`)
- [x] (must-fix) Placeholder note rewritten in the past tense as a publish-time staging record, naming `92526cf` and the six real links that replaced the nine `#TBD` tokens — `.agent/work-plans/issue-628/plan.md:376-391` (`82e74ce`)
- [x] (must-fix) `inspiration-tracker` repointed from the non-existent `docs/ROADMAP.md` "To Consider" to the root `ROADMAP.md` **Deferred** section, with the append format changed from a bullet list to the Deferred table's row shape (verified against `.agent/templates/roadmap.md` and the workspace instance) and two links to the vocabulary draft and the template. Three prose mentions fixed (steps 7, 8 and Guidelines); the digest-template line already read `ROADMAP.md` and needed no change — `.claude/skills/inspiration-tracker/SKILL.md:250-272,352` (`cd1210e`)
- [x] (low) Kind label made singular `decision` in the expected-location table and its row-by-row prose, in the draft and the plan; the path `docs/decisions/` is unchanged and the row now says so — `docs/design/planning_document_vocabulary.md:114,232` (`371c891`)
- [x] (low) `VISION.md` evidence scoped to the **external** projects surveyed, and both in-house instances named (`rolker/agent_workspace`, and `unh_marine_autonomy` whose `VISION.md` unh_marine_autonomy#394 folds into README § Vision); same sentence fixed in the plan — `docs/design/planning_document_vocabulary.md:204-213` and `.agent/work-plans/issue-628/plan.md:103` (`c7f58d6`)
- [x] (low) Read-both rule qualified "once a health document exists" in the draft's canonical statement and in the template, each with one clause saying why (no `docs/health.md` exists yet; a missing one is never a finding) — `docs/design/planning_document_vocabulary.md:334-345` and `.agent/templates/roadmap.md:130-135` (`919b0f2`)
- [x] (low) Org-board discovery fallback now specifies `gh project list --owner <org>`; the retired REST endpoint is named only as retired (HTTP 404, verified 2026-09-17) and the "both are worth a look" parenthetical is gone — `.agent/work-plans/issue-628/plan.md:56` (`534cd38`)
- [x] (low) Identity form rendered as runnable syntax `git -c user.name=… -c user.email=…` in the draft and the plan's trigger table — `docs/design/planning_document_vocabulary.md:297` and `.agent/work-plans/issue-628/plan.md:154` (`0e8f5b4`)
- [x] (low) "#634 makes the two-root rule checkable" restated as making the documents *discoverable*, with discovery named as the precondition for a check and enforcement left to a scoped follow-up — `docs/design/planning_document_vocabulary.md:344` (`3e08c76`)
- [x] (low) One clause added separating the canonical in-repo probe from #634's optional discovery fallbacks (org board, `homepage`, README links), keeping "absence is never a finding" intact — `docs/design/planning_document_vocabulary.md:120-131` (`883b7f3`)

### Verification
- Pre-commit over every file changed in this pass: clean.
- The two false positives in the Integrated Review were left as dismissals; no past progress entry was edited.
- `ROADMAP.md` was not touched in this pass and stays at its pre-existing size, inside the 150-line bound. Per the first finding's fix, the count is measured at publish rather than recorded here.
- New relative links in `.claude/skills/inspiration-tracker/SKILL.md` resolve from that file's location.

## Local Review
**Status**: complete
**When**: 2026-09-17 13:58 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**PR**: #638 at `aa05f55`
**Mode**: post-PR (re-review after address-findings)
**Depth**: Standard (reason: governance-touching files; docs-only, 12 files)
**Scope**: verification of the ten `## Integrated Review` findings (entry `23b874f`) against source in `b1b3882`..`883b7f3`, regression check on those commits, plus a light whole-diff sanity pass
**Must-fix**: 1 | **Suggestions**: 1
**Round**: 2 (post-PR) | **Ship**: recommended — must-fix fell 3 → 1; the one open item is a two-file factual sentence rewrite with the correct wording already determined, not a design question

### Findings
- [x] (must-fix) Regression from `c7f58d6`: the `VISION.md` evidence sentence now calls `rolker/agent_workspace` "the framework repo ... untouched since January". Verified: that repo has **no `VISION.md`** at root or under `docs/` (its root holds `README.md`/`AGENTS.md`/`ARCHITECTURE.md`; `docs/` holds `PRINCIPLES.md`, `ROADMAP.md`, `decisions/` — checked 2026-09-17), and the draft itself uses "the framework repo" for `unh_marine_autonomy` at lines 14-15 and 31-32 and calls `rolker/agent_workspace` "the sibling" at lines 221 and 233. The stale instance is `unh_marine_autonomy`'s `VISION.md`, last committed 2026-01-15 ("Establish Framework Vision"), which is what the pre-fix sentence and the Context section both said. As written the sentence contradicts its own document and cites a file that does not exist. Name `unh_marine_autonomy` as the sole in-house `VISION.md` instance, untouched since January, folded into README § Vision by unh_marine_autonomy#394; drop `rolker/agent_workspace` from this sentence (it is cited correctly elsewhere as the sibling, for its `docs/ROADMAP.md`) — `docs/design/planning_document_vocabulary.md:210-216` and the same sentence at `.agent/work-plans/issue-628/plan.md:103`
- [x] (suggestion) The as-built size record still reads "still **inside** step 4's 150-line first-cut bound" while `ROADMAP.md` measures exactly 150 — the inside/at distinction the round-1 finding named. The sibling record twelve lines below already says "which it still meets", which is the accurate form; match it — `.agent/work-plans/issue-628/plan.md:356`

### Verified resolved
All ten Integrated Review findings check out against source, one atomic commit each:
- Size check: both hand-typed counts replaced with the bound plus a `wc -l`-at-publish statement (`b1b3882`).
- Placeholder note rewritten in the past tense, naming `92526cf` and the six real links; no `#TBD` survives outside that record and the progress timeline — grep-confirmed (`82e74ce`).
- `inspiration-tracker` repointed to root `ROADMAP.md` **Deferred** at all three prose mentions plus the Guidelines line; the append snippet is a 3-cell row matching the `Item | Issue | Deferred because` header in both the template and the instance; both new relative links resolve (`cd1210e`).
- Kind singular `decision` in the expected-location table and its row-by-row prose; no `| decisions` row remains (`371c891`).
- Read-both rule qualified "once one exists" in both the draft and the template, each with its reason (`919b0f2`).
- Org-board fallback now `gh project list --owner <org>`; the retired REST form named only as retired (`534cd38`).
- Identity form rendered as runnable `git -c user.name=… -c user.email=…` in draft and plan (`0e8f5b4`).
- #634 restated as making documents *discoverable*, enforcement left to a follow-up (`3e08c76`).
- One clause separating the canonical in-repo probe from #634's discovery fallbacks, "absence is never a finding" intact (`883b7f3`).
- The `VISION.md` scoping fix (`c7f58d6`) addressed the finding as stated but introduced the must-fix above.

### Regression checks
- No closing keyword precedes an issue number anywhere in `main..HEAD` commit messages.
- `wc -l ROADMAP.md` = 150, at step 4's 150-line bound.
- Every relative markdown link introduced by this PR resolves on disk; the only unresolved targets are the four pre-existing `.agent/project_knowledge/` pointers, a gitignored optional symlink documented in place as "may not exist".
- Ask-First files (`AGENTS.md`, `CLAUDE.md`, `.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`) each carry exactly two added References lines and nothing else.
- Only checkbox ticks were made to the prior Integrated Review entry — no past progress text edited.
- All ten fix commits are authored by the agent identity and carry the runtime-model `Co-Authored-By` trailer; tree is clean; no trailing whitespace in changed files.
- Operator decisions recorded on #628 were not re-opened by any finding above.
