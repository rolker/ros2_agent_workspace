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

## Integrated Review
**Status**: complete
**When**: 2026-08-22 17:28 -04:00
**By**: Claude Code Agent (Claude Opus 5 (1M context))

**PR**: #603 at `578b3d5`
**Sources**: 2 (Copilot R1 @ `578b3d5`, CI rollup). The `Local Review (Pre-Push)` sits at `67c4ca3` — an older head — so nothing correlates at the current SHA.
**Cross-source confirmations**: 0
**CI**: all-pass (Lint (pre-commit), Validate Documentation, Script tests, Validate commit identity — all SUCCESS)

1 unresolved thread, current (not outdated). Verified empirically rather than by argument.

### Findings
- [ ] (valid, Copilot) **My `set -e` comment is factually wrong** — `.agent/scripts/docker_run_agent.sh:352`. The comment claims `[ "$PRINT_MOUNTS" = false ] && mkdir -p ...` "exits non-zero the moment the guard is false, aborting the dry run at that line". Copilot says `set -e` does not abort on a failing command that is part of an `&&` list when it is not the last command. **Tested, and Copilot is right for this position:**

  | case | result |
  |---|---|
  | mid-script (the actual site, line 352) | `line after` printed, **exit 0 — no abort** |
  | last line of a script | script **exits 1** |
  | last line of a function under `set -e` | caller **aborts, exit 1** |

  So the hazard is real, but only when the AND-OR list is the final command of a script or function — not here. The `if` form is still defensible (uniform with the two `if ! mkdir` sites, and safe if the block is ever relocated), but the stated justification is incorrect and would mislead the next reader into believing a rule that does not hold. Fix: correct the comment to state the real rule and why the `if` is kept, or drop the claim.
- [ ] (suggestion, carried from `Local Review (Pre-Push)` @ `67c4ca3`, still open) `ROOT_DIR` is a generic env-overridable name; no caller exports it today, but a future `export ROOT_DIR` in a wrapper would silently repoint every mount. Consider `DRA_ROOT_DIR_OVERRIDE`. Left as a judgement call at the publish gate; unaddressed.

### False positives
- None. The single Copilot comment is correct, and correct about something I asserted confidently and wrongly in a code comment.

### Notes
- The other two `Local Review (Pre-Push)` suggestions are fixed at this head: the `mkdir` side-effect gating (`ffb66b2`) and the reworded duplication assertion (`578b3d5`).
- CI is fully green including `Script tests`, which exercises the new `test_docker_run_mount_args.sh` (6/6) via `run_script_tests.sh`'s glob.

## Integrated Review
**Status**: complete
**When**: 2026-08-22 17:46 -04:00
**By**: Claude Code Agent (Claude Opus 5 (1M context))

**PR**: #603 at `c7c469c` (round 2)
**Sources**: 2 (Copilot R2 @ `c7c469c`, CI rollup). Prior `Local Review (Pre-Push)` is at `67c4ca3`; the round-1 `Integrated Review` at `578b3d5`. Neither correlates at the current head.
**Cross-source confirmations**: 0
**CI**: all-pass (Lint (pre-commit), Validate Documentation, Script tests, commit identity — all SUCCESS)

1 unresolved thread, current. Verified empirically; Copilot is right for the second consecutive round about the same comment.

### Findings
- [ ] (valid, Copilot R2) **The corrected `set -e` comment is still wrong** — `.agent/scripts/docker_run_agent.sh:358`. Round 1 fixed the claim that `set -e` aborts mid-script. The replacement said the `&&` form "only bites when such a list is the FINAL command of a script or function, where its non-zero status ... does propagate under `set -e`" — which lumps two different mechanisms under `set -e`. Copilot: a script whose final command is a failing `&&` list exits non-zero **regardless of `set -e`**. Tested, and correct:

  | case | `set -e` | result |
  |---|---|---|
  | script, last line | **off** | **exit 1** — plain exit-status propagation, no `set -e` involved |
  | function, last line | off | caller continues, exit 0 |
  | function, last line | on | caller **aborts**, exit 1 — genuinely a `set -e` effect |

  So only the *function* case is a `set -e` behaviour; the *script* case is ordinary shell exit-status semantics. Fix: reword to name each mechanism correctly, and shorten — a guard comment should not be a shell-semantics tutorial, and this one has now been wrong twice.

### False positives
- None.

### Notes
- Round-1's finding (the original, opposite error) was fixed in `c7c469c`; this is a fresh defect introduced by that fix, not a re-raise.
- Standing pattern worth recording: two consecutive rounds of a bot correcting confidently-worded shell semantics in a *comment*. The code was never wrong — only the prose explaining it. The durable lesson is to keep explanatory comments minimal and verifiable rather than authoritative-sounding.

## Integrated Review
**Status**: complete
**When**: 2026-08-22 17:55 -04:00
**By**: Claude Code Agent (Claude Opus 5 (1M context))

**PR**: #603 at `af1f5cd` (round 3)
**Sources**: 3 (Copilot R3 @ `af1f5cd`, `Local Review (Pre-Push)` @ `67c4ca3`, CI rollup)
**Cross-source confirmations**: 1
**CI**: all-pass (Lint (pre-commit), Validate Documentation, Script tests, commit identity — all SUCCESS)

Copilot's round-3 verdict is "🔵 Needs a closer look". Zero unresolved threads — the concern is in its summary and suppressed comments, not a thread. The round-2 comment finding is fixed and its thread resolved.

### Findings
- [ ] (cross-confirmed — `Local Review (Pre-Push)` @ `67c4ca3` + Copilot R3 @ `af1f5cd`) **`ROOT_DIR` is an ambient env override on a script that builds docker bind mounts** — `.agent/scripts/docker_run_agent.sh:23`. Both reviewers reached this independently. Copilot frames it harder than the local review did: an ambient `ROOT_DIR` "can accidentally repoint bind mounts and expand host exposure". The SHAs differ, but Copilot itself labels it "previously missed — in code that hasn't changed since the last review", so the two sources describe identical code; recording as cross-confirmed rather than letting a SHA technicality understate the strongest signal available.

  **The justification for deferring it was false.** Both the earlier triage and PR #603's body state "no caller exports `ROOT_DIR` today". Verified now — **`.agent/scripts/agent:19-20` does exactly that**:
  ```
  ROOT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"
  export ROOT_DIR
  ```
  and `dispatch_subagent.sh:562` invokes `"$SCRIPT_DIR/docker_run_agent.sh"` as a child process, so an exported value is inherited on that path. Today the two computations agree (both are `dirname` twice from `.agent/scripts`), so behaviour is unchanged — but the hook is **live, not dormant**, and the claim that nothing exports it is simply wrong. I repeated it from the round-1 review without checking.

  Fix: scope the override to a purpose-specific name (e.g. `DRA_ROOT_DIR_OVERRIDE`) so an ambient `ROOT_DIR` cannot reach the mount logic; update `tests/test_docker_run_mount_args.sh` (three call sites) and the comment at :19-23. Also correct the "no caller exports it" claim in the PR body and in issue #602.

### False positives
- None.

### Notes
- Round-2's finding (the `set -e` comment) is fixed at `af1f5cd` and its thread resolved; Copilot did not re-raise it.
- Three rounds, and the *code* has been correct throughout — rounds 1 and 2 were both about prose in one comment. This round is the first substantive one since the pre-push review, and it is substantive because a shared assumption ("nothing exports ROOT_DIR") went unverified by me across the issue body, the PR body, and two triage reports.
