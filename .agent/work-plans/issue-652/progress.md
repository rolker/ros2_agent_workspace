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

## Plan Review
**Status**: complete
**When**: 2026-09-22 11:45 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-652/plan.md` at `c858ecd`
**PR**: PR-less
**Verdict**: changes-requested

### Findings
- [ ] (must-fix) The extraction of `manifest_bootstrap_identity()` from `manifest_config_dir()` is behavior-preserving as scoped (correctly limited to the `bootstrap_url` resolution + regex match + owner/repo/branch/config_path safety checks at `manifest_fallback.sh:148-181`, excluding the "normal case" fast-path short-circuit at lines 127-146, which stays in `manifest_config_dir`), but the plan names no test to extend for it. `.agent/scripts/tests/test_resolve_repo_checkout.sh` already sources `manifest_fallback.sh` directly and exercises its internals (its source-guard behavior around line 471-513, and a direct call to `_manifest_fallback_url_key` at lines 664-666) — this is the test file the new function needs coverage in (a valid-pointer case asserting the four-field TSV output, and a malformed/missing-pointer case asserting exit 3), but neither the plan's Files to Change table (`plan.md:347-355`) nor its Estimated Scope (`plan.md:428-436`, "no new tests ... the skill has no executable test harness") names it. That justification is about the *skill*'s lack of a test harness; it does not apply to `manifest_fallback.sh`, which does have script-level tests today.
- [ ] (must-fix) Step 1a's resolution of "the project root" checkout (`plan.md:63-92`, "Reuses `resolve_repo_checkout.sh` (works without `layers/`)", `plan.md:99`) has an unstated dependency and a correctness gap specific to the no-`layers/` branch — which is not a corner case here, it is the scenario the design doc names as the primary trigger for this skill (`docs/design/planning_document_vocabulary.md` § The trigger: "Claude Code cloud Routine ... cloud-hosted, so no local `layers/` tree"). Traced through the actual scripts: when a layer checkout exists (`layers/main/*/src/$PROJECT_REPO_NAME`), `resolve_repo_checkout.sh` resolves it directly (case a) and this is moot. When it does not, `resolve_repo_checkout.sh` falls to case (b): it enumerates repos from the manifest's own `.repos` files and looks up `$PROJECT_REPO_NAME` by name — which only succeeds if the project root repo lists *itself* as an overlay entry in its own manifest. Verified this holds today only because `unh_marine_autonomy` happens to self-list in `core.repos` (`grep unh_marine_autonomy: layers/main/core_ws/src/unh_marine_autonomy/config/repos/core.repos`) — an incidental property of this one manifest's authoring, not a contract any script enforces or that ADR-0003 project-agnosticism guarantees for a different project. If a project's manifest doesn't self-list, step 1a silently degrades to `FAILED(project root resolve: ...)` with no per-project report ever produced, on every cloud run. Separately, this path is also wasteful even when it works: `manifest_config_dir`/`manifest_bootstrap_identity`'s own clone step (inside `manifest_fallback.sh`) already clones the project root into `.agent/scratchpad/manifest-repo/<repo>` to read its `config_path`; routing through `resolve_repo_checkout.sh` instead produces a second, independent clone of the identical repo into `.agent/scratchpad/janitor-repos/<repo>`. Plan should either (a) use the manifest-repo clone directory `manifest_config_dir`'s clone step already produces as the project root checkout directly (falling back to a layer checkout when present), avoiding both the self-listing dependency and the duplicate clone, or (b) explicitly document the self-listing dependency as a known limitation and confirm it's acceptable that non-self-listing projects silently get no per-project report on cloud runs.
- [ ] (should-fix) `$PROJECT_REPO_AUDITED_THIS_RUN` is read in step 3's write-gate (`plan.md:188`: `[ -n "${PROJECT_REPO_AUDITED_THIS_RUN:-}" ]`) but is never assigned anywhere in the plan's pseudocode — not in step 1a (`plan.md:63-92`) and not in step 2's "Rotation interaction" prose (`plan.md:122-132`), which describes the condition in words ("when that repo is in this run's chunk") but doesn't say where it becomes this variable. Name the assignment point (presumably alongside the existing repo-rotation loop, step 2/current-step-3) so implementation doesn't have to invent it.

### Confirmed correct (no finding)
- The `--publish` gating of `$HEALTH_BODY` and the local diff-source flip (§5) compose correctly with the #651 coverage gate: step 6 (current SKILL.md) renders `$WORKSPACE_SECTION` once and embeds the identical bytes into both `$HEALTH_BODY` and `$REPORT_BODY` (`.claude/skills/janitor-sweep/SKILL.md:812-819`), so `$REPORT_BODY`'s own `## Workspace` section (written to `.agent/scratchpad/janitor/<ts>-sweep.md`) is structurally identical to `docs/health.md`'s content — the "prior findings set" definition (§5, minus `Resolved`, including `Not re-examined`) reads correctly off either source. The plan's local-diff-source text (`plan.md:239-249`) matches this.
- The per-project report's "which repos belong to the project" question is well-defined and matches the two-root rule (`docs/design/planning_document_vocabulary.md` § The two-root rule: "the project root — the repo a manifest entry points at"). The plan derives `$PROJECT_REPO_NAME` from the tracked `configs/project_bootstrap.url` pointer (the same identity `manifest_config_dir` already derives for its own clone) — that is one repo, the manifest repo itself, not every repo the manifests declare. Verified this matches the concrete instance: `configs/manifest -> ../layers/main/core_ws/src/unh_marine_autonomy/config`.

### Open questions (plan's own) — assessment
- Step-1a lettered insertion vs. renumbering: acceptable — the skill already has top-level-adjacent lettered precedent (7a-7h) and the plan's stated reason (avoiding churn on the many step-5/6/7/8 cross-references in a 1500-line file) is sound.
- `manifest_bootstrap_identity()`'s exit-code documentation placement (fold into the existing table vs. a second block): no strong preference; a second, clearly-labeled block reads less ambiguously given the new function has a narrower contract (no clone, no exit 5/6) than `manifest_config_dir`.
- Per-project report filename using the repo name: correct given the two-root rule's equating of "project" and "project root repo" — no separate "project" label exists in the vocabulary draft today.

### Summary
Plan is well-researched and its four open Issue Review actions are addressed with specific, traceable mechanisms. The coverage-gate composition and the per-project report's scope definition both check out against the actual (post-#651) SKILL.md text and the two-root rule. Two must-fix gaps remain before implementation: the missing test-extension for the `manifest_fallback.sh` refactor, and the unverified/unsafe-for-other-projects dependency `resolve_repo_checkout.sh`'s manifest-lookup path has on the project root self-listing in its own `.repos` files — which silently fails exactly in the cloud-Routine scenario this skill is meant to run in. Both are fixable without changing the plan's overall shape.
