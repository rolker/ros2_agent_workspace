---
issue: 634
---

# Issue #634 — Conventional-path discovery of planning documents in janitor-sweep and audit-project (sub-issue 2 of #628)

## Issue Review
**Status**: complete
**When**: 2026-09-18 10:04 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)

**Issue**: #634
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: needs-more-detail

### Scope Assessment

**Well-scoped?** Mostly, but one bullet needs resolving before plan-task: see
Action 1 below. Bounded to workspace-repo skill logic (`janitor-sweep`,
`audit-project`), consistent with how sub-issue 1 (PR #638) was scoped and
merged as a single PR.

**Right repo?** Yes — workspace repo; the probed skills live here and the
change is generic ROS 2 workspace infra (ADR-0003), not project-specific.

**Dependencies**: Depends on the merged design draft
(`docs/design/planning_document_vocabulary.md`, PR #638) for the
expected-location table it must probe exactly. No other open sub-issue of
#628 blocks it. The draft's own "Promotion path to an ADR" section
(line ~612) names this issue's reader as one of three gates for promoting the
draft to ADR-0637 — so #637 (ADR promotion) is downstream of #634, not the
reverse.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Workspace vs. project separation | OK | Change is confined to workspace-repo skills; nothing project-specific proposed. |
| Only what's needed | Watch | The externally-hosted-document fallback (org GitHub Project board via `gh api orgs/<org>/projects`, `homepage` field, README link matching) is explicitly "discovery only... no requirement, no check, no finding" per the design draft. Bundling it into the same PR as the core path-probe adds a network/auth dependency (org Projects API scope, rate limits) for a feature that changes no outcome. Worth asking whether it should be a follow-up issue rather than in-scope here, since the issue's own "Out of scope" line ("Any check that treats absence... as a finding") already fences it off from ever mattering to pass/fail. |
| Test what breaks | Watch | The issue requires "an explicit test case" for graceful absence, but `janitor-sweep` and `audit-project` are markdown-procedure skills today (`.claude/skills/janitor-sweep/SKILL.md`, `.claude/skills/audit-project/SKILL.md`) with no backing script and no test harness in the repo. The conditional bullet ("If the probe is worth factoring into a shared script...") is doing load-bearing work here: without factoring the probe into an actual script, there is nothing to attach an automated test to. Recommend treating that factoring as effectively required, not optional, given the test-case requirement two bullets above it. |
| Capture decisions, not just implementations | OK | The design draft already captures the "no per-repo file, no schema" decision (2026-09-17) with rationale; this issue implements it rather than re-deciding it. |
| Enforcement over documentation | OK | Probing becomes code inside existing skills rather than a new markdown-only convention. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0003 — Project-agnostic workspace | Yes | Already explicitly cited in the issue's own "graceful absence" bullet; the change stays generic. |
| ADR-0013 — progress.md entry-type vocabulary | No (exception applies) | `janitor-sweep` and `audit-project` are periodic, non-issue-scoped skills — the review guide's consequences map carves them out of the "must persist a typed progress.md entry" row. No new entry type needed here. |
| ADR-0017 — Extend AGENTS.md to project repos | Watch | Not directly triggered, but the design draft's declared-file alternative was rejected partly citing ADR-0017's own Negative ("one more per-repo file") — worth a passing check during plan-task that the probe doesn't reintroduce a per-repo file through the back door (e.g. a cache/marker file). |

### Consequences

- If the probe is factored into a shared script (conditional bullet), `AGENTS.md`'s Script Reference table needs a new row — the issue already names this.
- If a `kind:` marker convention is actually introduced (see Action 1), the design draft's expected-location table and its "no per-repo declaration file and no schema" statement would need to be reconciled or amended, since a marker is a form of per-document schema the draft currently forecloses.
- `docs/design/planning_document_vocabulary.md` line ~612 names "the conventional-path reader has landed" as gate 1 of 3 for promoting the draft to an ADR (#637) — closing #634 should update that gate's status where #637 tracks it.

### Recommendations

- Split the externally-hosted-document discovery fallbacks (GitHub Project board API, `homepage` field, README link matching) into a follow-up issue, or clearly flag them as optional/stretch within this PR — they are explicitly non-blocking per the design draft and add an auth/network surface unrelated to the core, testable, path-probe requirement.
- Factor the probe into a shared script (not left as skill-markdown prose only) so the "explicit test case" for graceful absence has something to run against.

### Actions
- [ ] **Action needed**: the scope bullet "Ships the `kind:` marker convention and the `.agent/templates/` kind marker" references a mechanism that does not exist anywhere in the merged design draft it is supposed to implement "exactly." The draft (`docs/design/planning_document_vocabulary.md`) explicitly settled on path-only discovery and rejected a per-repo declaration file (Options considered, "Options considered and not taken"); no `kind:` marker syntax is defined in the draft, its templates (`.agent/templates/roadmap.md`, `adr_template.md`, `project_governance.md`), or anywhere else in the tracked tree (verified via `grep -rn "^kind:"`). This "kind:" marker appears to be a holdover from the *pre-draft* proposal in the parent issue (#628's original body, item 3: "a one-line `kind:` marker at the top of each document"), which the settled draft superseded. Before plan-task proceeds, this bullet needs either (a) removal from #634's scope as stale, or (b) an operator decision to add the marker convention to the design draft first (which would also require reconciling it with the "no schema" decision). Recommend surfacing this to the operator rather than plan-task silently inventing a marker syntax or silently dropping the bullet.
- [ ] Consider splitting the externally-hosted-document discovery fallbacks (GitHub org Project board API, `homepage` field, README link matching) into a follow-up issue — they are non-blocking discovery-only per the design draft and add auth/network surface to an otherwise self-contained path-probe change.
- [ ] Confirm during plan-task whether the probe needs factoring into a shared script (vs. skill-markdown prose only) to satisfy the issue's "explicit test case" requirement for graceful absence — the workspace currently has no test harness for markdown-only skill procedures.

## Plan Authored
**Status**: complete
**When**: 2026-09-18 10:10 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)

**Plan**: `.agent/work-plans/issue-634/plan.md` at `3baac19`
**Branch**: feature/issue-634 at `3baac19`
**Phases**: single

### Open questions
- [ ] Confirm the AGENTS.md Script Reference row wording before merge (Ask-First: instruction file edit).
- [ ] Confirm `run_script_tests.sh` auto-discovers `test_*.sh` or needs the new test file added explicitly — resolve during implementation.
- [ ] Confirm `## Vision` heading matching should be a prefix match (`^## Vision`) vs. requiring the exact bare heading — low-stakes, descriptive row only.

## Plan Review
**Status**: complete
**When**: 2026-09-18 10:15 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)

**Plan**: `.agent/work-plans/issue-634/plan.md` at `3baac19`
**PR**: PR-less
**Verdict**: approve-with-suggestions

### Evaluation

| Dimension | Verdict | Notes |
|---|---|---|
| Scope | Good | Single PR, matches sub-issue 1's precedent; external fallbacks correctly excluded per #643 split. |
| Issue alignment | Good | Both 2026-09-18 operator decisions on #634 (kind marker dropped; external fallbacks split to #643) are reflected exactly, verbatim-checked against the issue body and comments. The issue-review checkpoint's carried concern (probe must be a real script; no per-repo cache/marker) is explicitly addressed in step 1. |
| File targeting | Good | The four probed paths match `docs/design/planning_document_vocabulary.md`'s expected-location table exactly (vision/roadmap/decision/health), verified by reading the table directly (lines 112-117). Wiring through `audit-project` step 1's `$ROOT` pattern and `janitor-sweep`'s existing per-repo embed (SKILL.md line ~380/497) is verified accurate — no second call site is created. |
| Consequences | Good | Consequences table is complete; correctly defers the ADR-promotion gate update to #637 (already settled at issue-review) and correctly scopes the `AGENTS.md` row consequence to itself. |
| Documentation & instruction impact | Good | Non-silent; lists both skill-doc edits landing in this PR and flags the `AGENTS.md` row as an Ask-First candidate rather than auto-applying it, per `AGENTS.md § Boundaries`. |
| Principle alignment | Good | "Only what's needed" and "Enforcement over documentation" are both satisfied — no fifth janitor-sweep check invented, probe factored as real code with a dedicated test file. |
| ADR compliance | Good | ADR-0003 (verified: Decision section confirms workspace infra must stay project-agnostic) — the probe checks generic paths only, graceful-absence test instantiates it. ADR-0017 correctly invoked for the no-marker/no-cache constraint (matches the design draft's own rejection of a declaration file). ADR-0013 carve-out for periodic non-issue-scoped skills is consistent with review-issue's own assessment. |
| ROS conventions | N/A | Workspace-repo skill/script change only, no project-repo code touched. |

### Findings

1. **[Open Question 2, resolvable now]** `run_script_tests.sh` (verified by reading it) already auto-discovers `test_*.sh` via `nullglob` over both `"$SCRIPTS_DIR"/test_*.sh` and `"$TESTS_DIR"/test_*.sh` — no explicit registration list exists to add to. The plan's Open Question "Confirm `run_script_tests.sh` auto-discovers `test_*.sh`..." can be answered now rather than left for implementation: yes, it auto-discovers; `test_planning_doc_probe.sh` needs no extra wiring beyond being placed in `.agent/scripts/tests/`.
2. **[File targeting, minor]** `probe_decision`'s "present iff `docs/decisions/` contains at least one entry" doesn't say whether a placeholder file (e.g. a tracked `.gitkeep` some repos use to keep an empty dir in git) counts as an "entry." If it does, a repo that created the directory but hasn't written a decision yet would read as "present," contradicting the plan's own stated intent ("an empty placeholder directory reads as absent, not present"). Suggest the implementation exclude dotfiles when counting entries, and the test file add a `.gitkeep`-only case alongside the already-planned present/empty/absent cases.
3. **[Open Question 3, no action needed pre-implementation]** The `## Vision` heading prefix-match question (exact heading vs. `## Vision and Goals`-style trailing text) is already correctly flagged by the plan itself as low-stakes and resolvable during implementation — noting it here only to confirm review agrees it's not a blocker.

### Summary

Plan is well-aligned with the merged design draft's expected-location table (verified directly against `docs/design/planning_document_vocabulary.md`), ADR-0003, and both operator decisions recorded on the issue (verified against the issue body and comments via `gh`). The proposed script shape matches the workspace's existing `field_mode.sh`/`test_field_mode.sh` pattern, and the `janitor-sweep`/`audit-project` wiring claim (no second call site) is verified accurate against the current SKILL.md. No must-fix findings. Ready for implementation; the two minor suggestions above are cheap to fold in during implementation rather than requiring a plan revision.

### Recommended Actions

- [ ] During implementation, exclude dotfiles (e.g. `.gitkeep`) when counting `docs/decisions/` entries in `probe_decision`, and add that case to the test file.
- [ ] Resolve Open Question 2 as answered above (no registration needed) when updating the plan inline per `plan-task`'s "During implementation" rules.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-18 10:37 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)
**Verdict**: changes-requested

**Branch**: feature/issue-634 at `380bed3`
**Mode**: pre-push
**Depth**: Deep (reason: 798 lines changed across 7 files, ≥200-line Deep-promotion threshold; also carries Standard-tier governance-trigger files — two `.claude/skills/*/SKILL.md` edits and an `AGENTS.md` Script Reference row)
**Static analysis**: relies on pre-commit's `shellcheck-py` v0.9.0.6 (`--severity=warning`) at each commit — `shellcheck` binary unavailable on this review host to independently re-run; full `.agent/scripts/tests/run_script_tests.sh` suite re-run clean (27/27 shell tests incl. the new `test_planning_doc_probe.sh`, 220/220 pytest)
**Claude Adversarial**: 2 passes (Lens A + Lens B), Deep prompt, broadened file horizon
**Copilot Adversarial**: off (default)
**Local Adversarial**: off (default)
**Must-fix**: 1 | **Suggestions**: 8
**Round**: 1 | **Ship**: continue — one must-fix (missing `$ROOT` empty-guard) is a precise, mechanical, single-line fix, but Round 1 with an open must-fix does not meet the "no must-fix" or "round ≥2 + low/not-rising" recommended-ship criteria

### Findings
- [x] (must-fix) `audit-project` step 7's probe invocation has no `$ROOT`-empty guard, unlike step 5's identical-precondition guard a few lines above — `.claude/skills/audit-project/SKILL.md:259`
- [x] (suggestion) Step 7 also has no existence/executable check for `planning_doc_probe.sh` itself (a layer worktree with an unmerged main checkout would hit a raw "No such file" error) — `.claude/skills/audit-project/SKILL.md:259`
- [x] (suggestion) `probe_vision`'s `^## Vision` prefix match has no word-boundary check — `## Visionary Roadmap` would false-positive as present — `.agent/scripts/planning_doc_probe.sh:64`
- [x] (suggestion, deferred: fenced-code/blockquote awareness needs a stateful line scanner or markdown parser for a self-inflicted edge case, and the Planning Documents section is descriptive-only — worst case is a harmless false "present"; documented in the script's header comment instead) `probe_vision` is not markdown-context-aware — a `## Vision` line inside a fenced code block or blockquote would still match — `.agent/scripts/planning_doc_probe.sh:64`
- [x] (suggestion) Permission-denied paths (unreadable README, unlistable `docs/decisions/`) are indistinguishable from genuine absence — no distinct error signal — `.agent/scripts/planning_doc_probe.sh:64,92-101,107`
- [x] (suggestion) `probe_decision`'s dotfile-exclusion relies on bash's default (unset) `dotglob` with no explicit guard or filter — `.agent/scripts/planning_doc_probe.sh:88-101`
- [x] (suggestion) `docs/decisions/` containing only a broken symlink reads as absent (the `[ -e ]` no-match guard also filters broken symlinks) — undocumented side effect — `.agent/scripts/planning_doc_probe.sh:93-97`
- [x] (suggestion) `janitor-sweep`'s embedded Planning Documents table relies solely on an inline HTML comment to stay non-scoring — no explicit tie-back from the roll-up-rule prose — `.claude/skills/janitor-sweep/SKILL.md:500`
- [x] (suggestion) Report table templates show the kind's fixed expected path in Location even for "Not found," but the probe emits an empty path for absent kinds — ambiguous rendering guidance, plus test-coverage gaps (symlink/subdir/permission-denied/prefix-false-positive cases untested) — `.claude/skills/audit-project/SKILL.md:318-325`, `.claude/skills/janitor-sweep/SKILL.md:500-508`, `.agent/scripts/tests/test_planning_doc_probe.sh`

## Implementation
**Status**: complete
**When**: 2026-09-18 10:45 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Branch**: feature/issue-634 at `64f672c`
**Addressed**: Local Review (Pre-Push), 2026-09-18 10:37 -04:00, round 1, at `380bed3` — 1 must-fix, 8 suggestions
**Commits**: 0342378, 6359766, bd2bf79, 64f672c

### Actions
- [x] `audit-project` step 7 gets the same `$ROOT`-empty guard step 5 uses (SKIPPED, not a raw error) — `.claude/skills/audit-project/SKILL.md`
- [x] Step 7 also guards on `planning_doc_probe.sh` actually existing at `$ROOT/.agent/scripts/` before invoking it — `.claude/skills/audit-project/SKILL.md`
- [x] `probe_vision`'s heading match gets a word-boundary check (`^## Vision([[:space:]]|$)`); `## Visionary Roadmap` no longer false-positives — `.agent/scripts/planning_doc_probe.sh`, new test case
- [x] (deferred: markdown-context-aware matching costs more than it's worth — a stateful line scanner or markdown parser for a self-inflicted edge case, and the Planning Documents section is descriptive-only so the worst outcome is a harmless false "present"; documented in the script's header comment) `probe_vision` not markdown-context-aware — `.agent/scripts/planning_doc_probe.sh`
- [x] Permission-denied paths (unreadable README, unlistable `docs/decisions/`) now emit a distinct stderr diagnostic instead of being indistinguishable from genuine absence; TSV contract unchanged — `.agent/scripts/planning_doc_probe.sh`, new chmod-based tests (skipped when running as root)
- [x] `probe_decision`'s dotfile exclusion now uses `find ... ! -name '.*'` instead of relying on the caller's unset-by-default `dotglob` — `.agent/scripts/planning_doc_probe.sh`, new `.gitkeep`-plus-real-entry test
- [x] Broken-symlink-reads-as-absent behavior documented explicitly in `probe_decision`'s header comment, with a dedicated test case — `.agent/scripts/planning_doc_probe.sh`
- [x] `janitor-sweep`'s Check 2 rollup rule now names the Planning Documents table explicitly as non-scoring, tying back to `audit-project`'s own § 7 rule — `.claude/skills/janitor-sweep/SKILL.md`
- [x] Both report-format templates (`audit-project` § Report Format, `janitor-sweep`'s embedded example) now show Location as empty (`—`) on "Not found" rows, matching what the probe actually emits — `.claude/skills/audit-project/SKILL.md`, `.claude/skills/janitor-sweep/SKILL.md`

### Verification
- `.agent/scripts/tests/run_script_tests.sh`: 27/27 shell tests pass (incl. `test_planning_doc_probe.sh`, now 36 assertions, up from 29), 220/220 pytest.
- `shellcheck --severity=warning` on `planning_doc_probe.sh` and `test_planning_doc_probe.sh`: clean.
- `plan.md`'s "Implementation notes (as built)" section updated with an "Address-findings pass" subsection summarizing this round.

### Next step
Lifecycle: **Implementation** → **review-code** (re-review). Not auto-dispatched here per this skill's "no auto-chaining" rule — the host orchestrator drives the next phase.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-18 10:50 -04:00
**By**: Claude Code Agent (Claude Sonnet)
**Verdict**: approved

**Branch**: feature/issue-634 at `f87c963`
**Mode**: pre-push
**Depth**: Deep (reason: 1035 lines changed across 7 files, ≥200-line Deep-promotion threshold; also carries override-trigger files — two `.claude/skills/*/SKILL.md` edits and an `AGENTS.md` Script Reference row)
**Static analysis**: `shellcheck` binary unavailable on this review host; relies on pre-commit's `shellcheck-py` v0.9.0.6 (`--severity=warning`) at each commit as in round 1 — full `.agent/scripts/tests/run_script_tests.sh` suite re-run clean (27/27 shell tests incl. `test_planning_doc_probe.sh` at 36/36 internal assertions, 220/220 pytest)
**Claude Adversarial**: 2 passes (Lens A + Lens B), Deep prompt, broadened file horizon (fresh-context, no history carried in)
**Copilot Adversarial**: off (default)
**Local Adversarial**: off (default)
**Must-fix**: 0 | **Suggestions**: 3
**Round**: 2 | **Ship**: recommended — no must-fix findings this round; round-1's single must-fix and 8 suggestions verified fixed against the diff (not just trusted from the Implementation entry), one deliberate documented skip (markdown-context-aware Vision matching) carried forward unchanged

### Findings
- [ ] (suggestion, Lens A) `## Vision` regex `^## Vision([[:space:]]|$)` requires exactly one space/char boundary; a heading written `##  Vision` (double space) still matches via `[[:space:]]` so this is fine as coded — verified no bug, downgraded from Lens A's raw note: regex is correct, no action needed — `.agent/scripts/planning_doc_probe.sh:93`
- [ ] (suggestion, Lens A) `test_planning_doc_probe.sh` has no broken-symlink case for `ROADMAP.md` / `docs/health.md` at the root, analogous to the existing broken-symlink case for `docs/decisions` entries (behavior is already correct via `[ -f ]`; test-completeness gap only) — `.agent/scripts/tests/test_planning_doc_probe.sh`
- [ ] (suggestion, Lens A) No test for `docs/decisions` (or `docs`) itself being a symlink to a directory (behavior is already correct via `[ -d ]` following symlinks; test-completeness gap only) — `.agent/scripts/tests/test_planning_doc_probe.sh`

### Verification notes (this round)
- All 9 round-1 items re-checked against the current diff, not trusted from the Implementation entry: `$ROOT`-empty guard (`.claude/skills/audit-project/SKILL.md:259-268`), probe-script existence guard (same block), word-boundary Vision regex (`.agent/scripts/planning_doc_probe.sh:93`), permission-denied diagnostics for README/decisions (lines 88-101), `find ... ! -name '.*'` dotfile exclusion (lines 118-126), documented broken-symlink-as-absent (header comment + line 124), janitor-sweep non-scoring tie-back (`.claude/skills/janitor-sweep/SKILL.md:404-410`), Location-column empty-on-absent in both report templates (`audit-project` SKILL.md:334-350, `janitor-sweep` SKILL.md:504-521) — all confirmed present and correct.
- Cross-checked step renumbering consistency (old audit-project §7 → §8, new "Check planning documents" inserted as §7): every remaining "step 7"/"step 8" reference in both SKILL.md files points at the correct section (Lens B, via grep across `.claude`/`AGENTS.md`).
- Security/lifecycle (Lens B): probe functions never return file contents, only present/absent + fixed relative path; no shell injection (repo_path always a quoted positional param); script is fully stateless, safe for parallel invocation across a janitor-sweep chunk; TSV contract (4 lines, 3 fields each) cannot be corrupted by a permission-denied path (degrades to "absent" + separate stderr diagnostic).
- Two prior-round deliberate deferrals stand unchanged and are correctly out of scope for this round: markdown-context-aware Vision matching (documented in script header, harmless-false-positive worst case) and externally-hosted-document discovery fallbacks (split to #643).

### Next step
Lifecycle: **Local Review (Pre-Push)** → push / open PR → **triage-reviews**. Ship recommended — no must-fix findings; the three suggestions above are test-completeness/cosmetic and can ride the PR or a fast follow-up at the operator's discretion.
