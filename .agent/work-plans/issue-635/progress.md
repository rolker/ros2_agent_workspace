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
