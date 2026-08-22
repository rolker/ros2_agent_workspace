---
issue: 602
---

# Issue #602 — docker_run_agent.sh: shield worktree build artifacts from host contamination

## Issue Review
**Status**: complete
**When**: 2026-08-22 19:43 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #602
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Scope Assessment

The issue targets a single, well-identified gap in `docker_run_agent.sh`: the anonymous-volume "hole-punch" loop shields `layers/main/*_ws/{build,install,log}` but not the dispatched worktree's `*_ws` equivalents. The proposed fix is a small, targeted addition to the existing mount-assembly block. Fits in a single PR with no sub-issue splitting required.

**Right repo?** Yes — `docker_run_agent.sh` is workspace infrastructure under `.agent/scripts/`. No project-repo content involved.

**Dependencies**: None blocking. Issues referenced (#492, #532, #552, #570) are historical context only.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Fix is minimal and commented; the issue documents the mechanism and acceptance criteria clearly |
| Enforcement over documentation | OK | Existing comment stated the intent; this fix makes enforcement match the comment |
| Capture decisions, not just implementations | Watch | The `-L` symlink guard and the "only dispatch-target worktree, not all worktrees" rationale should appear as code comments; issue body is thorough but won't survive a diff |
| A change includes its consequences | Action needed | No automated test for mount-arg generation; acceptance criteria are behavioral (manual reproduction). A regression test (similar to `test_layer_sourcing.sh`) checking the new loop's output would prevent silent future breakage |
| Only what's needed | OK | ~3 extra mounts per launch (only the dispatched worktree), not 334 |
| Improve incrementally | OK | Small, targeted change to a known-bad path |
| Test what breaks | Watch | The symlink-guard logic (`[ ! -L "$ws_dir" ]`) is subtle; a unit-style test for the mount-argument builder would catch regressions without requiring a full Docker launch |
| Workspace vs. project separation | OK | Pure workspace infrastructure change |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| 0002 — Worktree isolation | Yes | Fix is directly about worktree/container boundary — aligns with the intent |
| 0003 — Project-agnostic workspace | Yes (any workspace-repo change) | Fix is generic ROS 2 infrastructure, no project coupling |
| 0005 — Layered enforcement | Watch | Fix provides runtime enforcement; adding an automated test would add a local-feedback layer |

### Consequences

Per the consequences map:
- `docker_run_agent.sh` description in `AGENTS.md` script table does not need updating (the entry description "Launch the sandboxed agent container for a worktree" is still accurate). No new entry needed.
- The host/container build-artifact contamination pattern is a non-obvious pitfall — implementation may surface a `.agent/knowledge/` note candidate (operator approval required before any edit lands).
- The image staleness check mentioned in the issue is explicitly called out as separable; a follow-up issue should be filed after this PR merges.

### Actions
- [ ] Add an automated test (akin to `test_layer_sourcing.sh`) that verifies the new worktree-path loop adds anonymous-volume mounts for real `*_ws` directories and skips symlinked ones.
- [ ] Ensure code comments in `docker_run_agent.sh` capture the symlink-guard rationale and the "only dispatched worktree, not all worktrees" design decision so it survives beyond the issue.
- [ ] File a follow-up issue for the image staleness check (launcher warning when image predates host ROS packages) after this PR merges.

## Plan Authored
**Status**: complete
**When**: 2026-08-22 19:47 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-602/plan.md` at `46fa709`
**Branch**: feature/issue-602 at `46fa709`
**Phases**: single

### Open questions
- [ ] Should `--print-mounts` be the dry-run API, or is it cleaner to extract the mount-generation block into a sourced helper and test it directly?

## Plan Review
**Status**: complete
**When**: 2026-08-22 19:55 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-602/plan.md` at `46fa709`
**PR**: PR-less (`--issue 602`)
**Verdict**: approve-with-suggestions

Verified against `docker_run_agent.sh` (lines 322–393) and a real layer worktree
(`layers/worktrees/issue-camp-199`: `ui_ws` is a real dir, siblings are symlinks
into `layers/main` — confirms the plan's symlink-guard analysis). The mount
mechanics, no-op behavior for workspace worktrees, and section-4 mirroring are
all sound. All three review-issue actions (test, code comments, follow-up issue)
are carried into the plan.

### Findings
- [ ] (must-fix) Test harness can't control the tree as written — plan.md:44–50 has the test call `docker_run_agent.sh --issue X --print-mounts` "with `ROOT_DIR` pointed at a temp tree", but `ROOT_DIR` is computed from the script's own path (script line 18) with no env override, and `WORKTREE_PATH` derives from it (lines 244–264). Implementation must add an injection hook (e.g. `: "${ROOT_DIR:=<computed>}"` so an env value wins, or a `WORKTREE_PATH` override) or the `--print-mounts` test cannot fabricate a fake worktree. Resolve the open question in favor of whichever approach makes this injection clean.
- [ ] (suggestion) `--print-mounts` must short-circuit before any docker call — to stay Docker-free in CI it must print `MOUNT_ARGS[@]` and exit before the image build/stage step (script lines 286–320) and the `docker run` at line 527. Call this out in the plan so the flag is placed correctly.
- [ ] (suggestion) Consequence not captured: the new `--print-mounts` flag should be added to `show_usage()` (script lines 43–81) and the `# Usage:` header comment (lines 8–10); the Consequences table omits this doc-drift.
- [ ] (suggestion) Positive/no-action: placing the test at `.agent/scripts/tests/test_docker_run_mount_args.sh` is correct — `run_script_tests.sh` auto-globs `tests/test_*.sh`, so `make test-scripts` / `make validate` will pick it up with no wiring.

## Implementation
**Status**: complete
**When**: 2026-08-22 20:53 +00:00
**By**: Claude Code Agent (Claude Opus)

**Issue**: #602
**Branch**: feature/issue-602
**Commits**: `7596d74`, `113ce69`, `a7b147b`

### What was implemented

Core fix — shield the dispatched worktree's `*_ws` build artifacts so the
container no longer shares `build/install/log` with the host at the same
absolute path, plus the four approved Plan Review findings.

### Actions

- [x] **Core change** — added a section-4b anonymous-volume shield loop over
  the resolved `$WORKTREE_PATH/*_ws`, guarded on `[ -d ]` **and** `[ ! -L ]`
  (symlinked siblings into `layers/main` are already shielded by section 4;
  `[ -d ]` follows symlinks so `[ ! -L ]` is what filters them). Mirrors
  section 4's `mkdir -p` ownership precaution and fail-loud handling (#566).
  Scoped to the one resolved worktree, not a glob of all worktrees. `7596d74`
- [x] **(must-fix)** Test-injection hook — `ROOT_DIR` now honors a pre-set env
  value via `: "${ROOT_DIR:=…}"`, so a test can point worktree resolution and
  both shield loops at a fabricated temp tree. Open question resolved in favor
  of `--print-mounts` + this hook. `113ce69`
- [x] **(suggestion)** `--print-mounts` short-circuits before all Docker
  interaction — the image inspect/build block and the auth-required check are
  skipped in this mode, and the print-and-exit sits after section 8 (mounts
  assembled) but before `docker run`. Docker- and credential-free. `113ce69`
- [x] **(suggestion)** Flag documented in `show_usage()` and the `# Usage:`
  header comment; Consequences table in plan.md extended to capture the
  doc-drift. `113ce69`
- [x] **(suggestion, no action)** Test placed at
  `.agent/scripts/tests/test_docker_run_mount_args.sh`; auto-globbed by
  `run_script_tests.sh`. `a7b147b`
- [x] Plan kept in sync — open question resolved, injection/Docker-free
  placement recorded, Files-to-Change + Consequences updated. `113ce69`

### Verification actually run

- New test `test_docker_run_mount_args.sh`: **6/6 pass** — real worktree
  `ui_ws` shielded (build/install/log), symlinked `nav_ws` skipped by
  `[ ! -L ]`, `layers/main` target shielded exactly once (no duplication),
  workspace worktree a clean no-op. Both `--print-mounts` runs exit 0.
- `shellcheck` on the new test: clean.
- Manual `--print-mounts` smoke test against a fabricated tree: correct mount
  plan (real `*_ws` mounted, symlink skipped).
- `run_script_tests.sh` (full suite): my test passes; all shell + 73 pytest
  tests pass **except** the pre-existing `test_check_commit_identity.sh`
  "no propagation rejection" case, which fails identically on base commit
  `9cfa05a` (unrelated `git -c` propagation/env issue, not touched by this
  change — verified by checkout).
- Pre-commit hooks ran on every commit (shellcheck included); none bypassed.

### Not done (out of scope, as planned)

- Follow-up issue for the image-staleness launcher warning — plan step 4,
  explicitly deferred until after this PR merges.

### Next step

review-code re-review (pre-push) before the host opens the PR.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-08-22 21:02 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-602 at `67c4ca3`
**Mode**: pre-push
**Depth**: Deep (reason: container-sandbox mount/isolation change + ROOT_DIR env-injection hook on a user-input script that shells out to docker)
**Must-fix**: 0 | **Suggestions**: 3
**Round**: 1 | **Ship**: recommended — no must-fix; core shield correct across static + governance + plan-drift + 2 adversarial lenses; 3 suggestions applyable or trackable

### Findings
- [ ] (suggestion) `--print-mounts` "dry run" still runs `mkdir -p` (sections 3/4/4b) before the short-circuit, creating build/install/log + scratchpad on the host — cross-pass confirmed (Lens A+B); gate mkdir behind `[ "$PRINT_MOUNTS" = false ]` — `.agent/scripts/docker_run_agent.sh:349,365,398`
- [ ] (suggestion) Test assertion #4 ("no symlink duplication") is redundant/misleading — a removed `[ ! -L ]` guard is caught by assertion #3, not #4 — `.agent/scripts/tests/test_docker_run_mount_args.sh:82`
- [ ] (suggestion) `ROOT_DIR` is a generic env-overridable name; low risk today (no caller exports it) but a future `export ROOT_DIR` would silently repoint mounts — consider `DRA_ROOT_DIR_OVERRIDE` — `.agent/scripts/docker_run_agent.sh:23`

### Notes
- Static analysis: shellcheck clean on both changed files (pre-existing SC2016 at line 278 is on an untouched line — not reported).
- Local Model Adversarial skipped: no Ollama server at http://localhost:11434.
- Copilot Adversarial: off (default; not opted in).
- Reviewed against local `origin/main` (fetch offline — base may be stale); `gh issue view 602` unavailable offline, context taken from plan.md/progress.md.
