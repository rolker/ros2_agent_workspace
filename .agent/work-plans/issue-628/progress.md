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
