---
issue: 652
---

# Issue #652 — janitor-sweep: local-first — --publish opt-in, local diff source, optional per-project health when the project root has a ROADMAP.md

## Issue Review
**Status**: complete
**When**: 2026-09-22 11:37 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #652
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Actions
- [ ] Plan-task must pin down how the sweep locates "the project root" checkout on disk — the issue's phrase "the project root (the manifest repo checkout)" matches the two-root rule's definition (`docs/design/planning_document_vocabulary.md`: "the repo a manifest entry points at"), and that repo (`unh_marine_autonomy`) is already checked out in the normal rotation (`layers/main/core_ws/src/unh_marine_autonomy`) — but the issue doesn't say how the sweep is meant to identify *which* rotation repo is the project root generically (e.g. deriving `owner/repo` from `configs/project_bootstrap.url` the way `manifest_fallback.sh` already does, vs. some new config). Resolve this with existing tooling, not a new declaration, to stay consistent with decision 3's "no new declaration file, no schema" and ADR-0003 (project-agnostic workspace) — this workspace must not hardcode `unh_marine_autonomy`.
- [ ] Plan-task must define the run-over-run diff source and state (New/Resolved/Unchanged/Not re-examined, per the current § 5 coverage-gated mechanism from #651) for the new per-project local health report. The issue specifies diff sources for the two existing scopes (decision 2) but is silent on this third report type; § 5 is detailed and load-bearing (coverage-gated resolution, `(since <date>)` stamps) and a silent exemption for the new report would be an inconsistency, not a simplification.
- [ ] Plan-task must specify how § 7's `## Publish outcome` section and step 6's `$HEALTH_BODY` documentation (currently: "exactly the bytes committed to `docs/health.md`") read on a default, non-publishing run. The current SKILL.md text assumes step 7 (and therefore a commit) always happens; the default run under this issue never reaches step 7, so both need their own non-publishing wording rather than describing a write that didn't occur.
- [ ] Confirm the new per-project report file's write failure is named using the same "the report write is itself a state" convention the workspace report already uses (§ status contract), since it is a second durable local output this skill now produces.

## Findings detail

### Scope Assessment
**Well-scoped?** Yes — single coherent PR: an opt-in `--publish` flag gating the existing 7a–7h commit-and-PR flow (already cleanly separable, keyed only on `$HEALTH_BODY`/`$REPORT`/`$ROOT`), a diff-source switch for workspace scope, and one new optional local report. Issue explicitly excludes #651 (already landed, PR #655) and the what-next port (own issue) from scope.
**Right repo?** Yes — workspace repo; this is workspace tooling (`janitor-sweep` skill + `ROADMAP.md`).
**Dependencies**: Part of #628 (planning-document vocabulary / two-root rule). Follows #635 (merged, PR #647) and PR #648 (first live run). Main now includes #651's merged changes (PR #655) to SKILL.md § 5/§ 6 — the coverage-gated `Not re-examined` diff state. Checked the issue's three decisions against the current (post-#651) skill text: decision 2 (workspace-scope diff source switch) is compatible with § 5's post-#651 coverage-gated logic — the coverage gate is orthogonal to where the "previous run" data comes from — but the switch itself is not yet implemented (`--publish` does not exist in SKILL.md today; confirmed via grep).

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Workspace vs. project separation | OK | Project-root concept is defined generically via the two-root rule; no project-specific content proposed for the workspace repo. |
| Only what's needed | OK | Decision 3 explicitly avoids a new declaration file/schema, reusing the existing `ROADMAP.md`-presence convention. |
| Human control and transparency | Watch | See Action items on `## Publish outcome` / `$HEALTH_BODY` wording under non-publish runs — a stale or misleading description of what was/wasn't committed would undercut this. |
| A change includes its consequences | Watch | The new per-project report is a second durable local output; its diff behavior and write-failure naming aren't specified in the issue (see Actions). |
| Capture decisions, not just implementations | OK | Issue records the operator's verbatim direction (2026-09-21) and three explicit, rationale-bearing decisions. |
| Enforcement over documentation | OK | `--publish` off by default is itself the safety property; no enforcement gap. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| 0003 — Project-agnostic workspace | Yes | The project-root probe must stay generic (see Action item 1) — issue wording is already generic, but the resolution mechanism isn't specified. |
| 0013 — progress.md entry-type vocabulary | No | `janitor-sweep` is the documented exception (periodic, non-issue-scoped skill; `principles_review_guide.md` Consequences Map) — no new entry type needed. |
| 0017 — Extend AGENTS.md to project repos | No | Not implicated; this issue doesn't touch project-repo agent guides. |

### Consequences

- `principles_review_guide.md` Consequences Map: "Where a skill does have a durable output it must name the state in which that write failed" — the new per-project report file needs this named, same as the existing report (see Actions).
- `ROADMAP.md`: issue's own "Also in this PR" section already commits to updating the #636 row (currently still `planned`, not yet `deferred`) and adding rows for #652 and the what-next port — verified current `ROADMAP.md` state matches what the issue describes as pending (no #651/#652/#653 rows exist yet), so no conflict with #651's landed PR.

### Recommendations

- Resolve "the project root" checkout via existing tooling (derive `owner/repo` from `configs/project_bootstrap.url`, as `manifest_fallback.sh` already does) rather than introducing new state.
- Extend the existing § 5 coverage-gated diff mechanism to the new per-project report rather than leaving it undiffed, unless plan-task records an explicit reason not to (e.g. "first run, no prior local report" is already a first-class case § 5 handles for project scope).
- Update the `## Publish outcome` / `$HEALTH_BODY` prose for the non-publish default path in the same PR — this is exactly the kind of stale-description gap the "change includes its consequences" principle exists to catch.

---
**Authored-By**: `Claude Code Agent`
**Model**: `Claude Sonnet`

## Plan Authored
**Status**: complete
**When**: 2026-09-22 11:40 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-652/plan.md` at `c858ecd`
**Branch**: feature/issue-652 at `c858ecd`
**Phases**: single

### Open questions
- [ ] Confirm step-1a (lettered top-level insertion, not a full renumber) is acceptable style for this 1500-line skill doc.
- [ ] Confirm whether `manifest_bootstrap_identity()`'s exit codes get a second documented block or fold into `manifest_config_dir`'s existing table.
- [ ] Confirm per-project report filenames use the repo name (not a distinct "project" label — none exists yet in the vocabulary draft).

---
**Authored-By**: `Claude Code Agent`
**Model**: `Claude Sonnet`
