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

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-22 12:24 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-652 at `5c3af09`
**Mode**: pre-push
**Depth**: Deep (reason: 1453 lines changed; skill/knowledge/governance override triggers)
**Must-fix**: 3 | **Suggestions**: 12
**Round**: 1 | **Ship**: continue — three must-fixes at round 1, but all are one-line mechanical fixes; one more round should close it

Tests run by the reviewer: `bash .agent/scripts/tests/test_resolve_repo_checkout.sh` → 70 passed / 0 failed (incl. all 12 new `bootstrap identity` cases); `make test-scripts` → all script tests + 220 pytest passed, exit 0. `bash -n` clean on both shell files; shellcheck not installed in this worktree.

Scrutiny asks: (a) the `manifest_bootstrap_identity()` extraction IS behaviour-preserving for `manifest_config_dir()`'s parse path (identical regex, `.git` strip, safety validation, exit 3 mapped through; no dangling `bootstrap_url`/`pointer_file` references); (c) the diff-source flip does NOT regress the #651 coverage gate — the gate keys on this run's `### Coverage` rows, not on the prior document, and the local report's `## Workspace` section is the same bytes as `docs/health.md` by construction; (d) the plan's Files-to-Change matches the diff exactly.

### Findings
- [x] (must-fix) `$PROJECT_HEALTH_BODY` is redacted above the guard that decides whether it was ever rendered — unbound read under `set -u` on the common path — `.claude/skills/janitor-sweep/SKILL.md:1166`
- [x] (must-fix) the primary-report write-failure block does not stop, and the new per-project block then appends to `$REPORT` and can record `written:` for a run whose report never landed — `.claude/skills/janitor-sweep/SKILL.md:1099-1102` (with 1183)
- [x] (must-fix) new Deferred text claims a cloud run renders checks 2 and 3 as `SKIPPED(no project configured on this checkout)`, contradicting step 2's no-`layers/` row and reusing 1a's per-project status as a check status — `.claude/skills/janitor-sweep/SKILL.md:1823-1826`
- [x] (suggestion) `prior report predates retention` is unreachable on the default workspace diff branch — `.claude/skills/janitor-sweep/SKILL.md:781-797` (vs 1261-1264)
- [x] (suggestion) `$PREV_SOURCE` is `""` rather than one of the template's `none — …` strings, and the `--publish` branch names a source even when `git show` failed — `.claude/skills/janitor-sweep/SKILL.md:766-767,787`
- [x] (suggestion) `SKIPPED(project repo not in this run's chunk)` also fires when the repo was in the chunk but its probe/audit failed — `.claude/skills/janitor-sweep/SKILL.md:1173`
- [x] (suggestion) warn that `PROJECT_REPO_AUDITED_THIS_RUN=1` dies with the subshell if check 2's loop is a `| while read` pipeline — `.claude/skills/janitor-sweep/SKILL.md:639-640`
- [x] (suggestion) `FAILED(project root probe: <reason>)` is absent from the status contract and from step 8's bullet list, though 1a promises all four states are stated — `.claude/skills/janitor-sweep/SKILL.md:75-124,1753-1758`
- [x] (suggestion) "the next run's 7a/7e" is now the next *publishing* run — orphan worktrees and stray branches can survive many default runs — `.claude/skills/janitor-sweep/SKILL.md:1509-1514,1620-1621`
- [x] (suggestion) "no new lookup logic is needed" overclaims — §5's only concrete lookup globs `*-sweep.md` and the per-project file has a different shape — `.claude/skills/janitor-sweep/SKILL.md:1150-1159`
- [x] (suggestion) guard the project-root glob with `[ -n "$PROJECT_REPO_NAME" ]`, and note that two layers holding the same repo name resolve first-glob-wins unrecorded — `.claude/skills/janitor-sweep/SKILL.md:378`
- [x] (suggestion) prose says `<ts>-<repo>-health.md`, the code writes `<ts>-$$-<repo>-health.md` — `.claude/skills/janitor-sweep/SKILL.md:56,1117` vs 1175
- [x] (suggestion) the extracted exit-3 diagnostic dropped the "no `configs/manifest`" half, so the message that sends an operator to `make setup-all` now reads as a pointer problem only — `.agent/scripts/manifest_fallback.sh:384`
- [x] (suggestion) the Script Reference row does not name the new `manifest_bootstrap_identity` entry point — AGENTS.md is Ask-First and the plan deferred it deliberately, so this is an operator call — `AGENTS.md:587` (deferred: AGENTS.md is Ask-First — the operator decides this at the publish gate; not changed on the branch)
- [x] (suggestion) the ROADMAP `#635` row flip (`planned` → `done`) is a ride-along recorded nowhere in the plan — `.agent/work-plans/issue-652/plan.md`

## Implementation
**Status**: complete
**When**: 2026-09-22 12:36 -04:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-652 at `aeef0e0`
**Addressed**: `## Local Review (Pre-Push)` (2026-09-22 12:24 -04:00, branch at `5c3af09`) — round 1, changes-requested: 3 must-fix, 12 suggestions
**Commits**: d7ee385 3a0ce62 a2c373d 8288a04 dda04fd 338880f 833f2b4 752f35b 9f5a136 4358140 0cd86bb 28fb2ff aeef0e0

Tests after the fixes: `bash .agent/scripts/tests/test_resolve_repo_checkout.sh`
→ 71 passed / 0 failed (70 before, plus the new `manifest_config_dir` exit-3
case); `make test-scripts` → all 16 script suites + 220 pytest passed, exit 0.
`bash -n` clean on `manifest_fallback.sh`; pre-commit (incl. shellcheck) ran on
every commit.

### Actions
- [x] (must-fix) `$PROJECT_HEALTH_BODY` redacted above the guard — moved the `redact_text` inside the write branch, immediately before the write, with the `set -u` reason stated — `SKILL.md` § 6 (`3a0ce62`)
- [x] (must-fix) the primary-report write failure does not stop — added `exit 1` and a paragraph saying the stop was inert on `main` and is load-bearing now that the per-project block appends to `$REPORT` — `SKILL.md` § 6 (`d7ee385`)
- [x] (must-fix) the Deferred cloud-run text — rewritten as what #636 must **implement** when unparked (a cloud-mode switch skipping checks 2 and 3 with its own check-level `SKIPPED(cloud run: workspace scope only)`, never cloning manifests to manufacture a project), plus a plain statement that **today** a no-`layers/` host still follows step 2's manifest-clone row and audits the cloned repos. 1a's per-project status string is no longer reused as a check status — `SKILL.md` § Deferred (`a2c373d`)
- [x] (suggestion) `prior report predates retention` unreachable on the workspace branch — dropped from the workspace `**Diffed against**` template, with why it is only distinguishable in project scope (`dda04fd`)
- [x] (suggestion) `$PREV_SOURCE` empty / named on a failed `git show` — both branches now assign one of the template's own strings; the `--publish` branch names the commit only when the read succeeded (`dda04fd`)
- [x] (suggestion) `SKIPPED(project repo not in this run's chunk)` also fired when check 2 never audited it — chunk membership recorded in step 2 where it is decided (`$PROJECT_REPO_IN_CHUNK`); step 6 states which of the two applied; new `SKIPPED(check 2 did not audit <repo>)` added to step 8's list (`8288a04`)
- [x] (suggestion) subshell warning for `PROJECT_REPO_AUDITED_THIS_RUN=1` — check 2's loop must not be a `| while read` pipeline; `for` or here-string named (`338880f`)
- [x] (suggestion) `FAILED(project root probe: <reason>)` missing from the status contract and step 8 — added to both, and distinguished from the seventh state (built-vs-written) (`833f2b4`)
- [x] (suggestion) "the next run's 7a/7e" is now the next *publishing* run — stated at both sites, with the cost (an orphan worktree/branch can outlive many default runs) named rather than discovered (`752f35b`)
- [x] (suggestion) "no new lookup logic is needed" overclaimed — the per-project files are **not** a diff source; § 5 keeps reading the `*-sweep.md` `## Projects` section. The new logic is retention, not lookup (`9f5a136`)
- [x] (suggestion) project-root glob unguarded — `[ -n "$PROJECT_REPO_NAME" ]` added (an empty name would glob every layer's `src/`), and first-glob-wins across two layers holding the same repo name is now recorded (`4358140`)
- [x] (suggestion) filename prose vs code — the sweep report's own code carries the `$$` pid (on `main` too), so the pid stays and the **prose** was fixed: every `<ts>-…` mention now reads `<ts>-<pid>-sweep.md` / `<ts>-<pid>-<repo>-health.md`, and the convention is named once where `$REPORT` is defined (`0cd86bb`)
- [x] (suggestion) the extracted exit-3 diagnostic dropped the "no `configs/manifest`" half — `manifest_config_dir` now adds what it knows on top of what the extracted function knows, with a regression case asserting both halves appear — `manifest_fallback.sh`, `test_resolve_repo_checkout.sh` (`28fb2ff`)
- [x] (suggestion) the AGENTS.md Script Reference row does not name `manifest_bootstrap_identity` — `AGENTS.md:587` (deferred: AGENTS.md is Ask-First; the operator decides this at the publish gate. Not changed on the branch — flagged here so the gate has it in front of it)
- [x] (suggestion) the ROADMAP `#635` row flip recorded nowhere in the plan — recorded in plan § 6 as an explicit ride-along (a status correction to an already-merged issue, kept because leaving it stale would contradict #636's row right beside it), plus the `**Health document**:` paragraph reword and the new test case, both added to Files to Change (`aeef0e0`)

### Deferred
- The `AGENTS.md` Script Reference row for `manifest_bootstrap_identity` — Ask-First per AGENTS.md § Boundaries. Left for the operator at the publish gate; it is a one-line addition to the `manifest_fallback.sh` row, not a behaviour change.
