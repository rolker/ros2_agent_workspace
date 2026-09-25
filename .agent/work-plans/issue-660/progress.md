---
issue: 660
---

# Issue #660 — Port the external cross-model reviewer (Gemini via agy + Codex) from agent_workspace into review-code

## Issue Review
**Status**: complete
**When**: 2026-09-23 14:32 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)

**Issue**: #660
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Actions
- [ ] Pick a fresh ADR number for the parallel-sync-only decision (next free is 0020 as of this review) — `0015` is already taken here by `docs/decisions/0015-dispatch-handoff-context-contract.md` (unrelated topic); do not port the source's ADR file verbatim under its original number.
- [ ] Add `AGENTS.md` Script Reference rows for `cross_model_review.sh`, `_agy_review.sh`, `_cli_review.sh`, `_resolve_work_plans_dir.sh`, `_resolve_default_branch.sh`.
- [ ] Update `.agent/knowledge/inspiration_agent_workspace_digest.md`'s "Partially adopted" note (references #206/#212) once this lands.
- [ ] Treat `_resolve_work_plans_dir.sh` / `_resolve_default_branch.sh` as net-new shared helpers in this workspace (no existing equivalent — `check_branch_updates.sh` resolves the default branch inline today); scope the port to what the cross-model scripts need, and do not fold in a repo-wide refactor of existing inline resolvers into the new helpers.
- [ ] Re-verify the port table's script contents against `~/agent_workspace` `main` at plan/implementation time rather than this review's snapshot (`48b0d82`, "Merge pull request #341 from rolker/feature/issue-320") — the issue's own follow-up comment notes the source was still moving as of 2026-09-22.
- [ ] Decide design point 3 (new step 5g vs. replacing 5e's body) — recommend **5g** (new step): 5e's Copilot-specific machinery (skip-reasons, untrusted-PR safety gate, `--allow-all-tools` security note) doesn't generalize cleanly to a `--agents gemini,codex,copilot` call, and keeping them separate avoids destabilizing the working Copilot path.

## Plan Authored
**Status**: complete
**When**: 2026-09-23 14:42 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)

**Plan**: `.agent/work-plans/issue-660/plan.md` at `9ceaf11`
**Branch**: feature/issue-660 at `9ceaf11`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.

## Plan Review
**Status**: complete
**When**: 2026-09-23 14:46 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)

**Plan**: `.agent/work-plans/issue-660/plan.md` at `9ceaf11`
**PR**: PR-less
**Verdict**: changes-requested

### Findings
- [ ] (must-fix) Plan's "no tool access... by design" claim for codex is unverified/inaccurate — codex's `_cli_review.sh` invocation (`codex exec -o <file>`) passes no `-s`/`-a` flag; its restriction (`sandbox: read-only`, `approval: never`, confirmed by live run on this host) is upstream's current default, not a script-enforced guarantee like claude's `--permission-prompts none` or copilot's `--available-tools=''`. A future codex-cli default change (or a host config change) would silently grant write/full access with no code here to catch it, and even read-only codex can still read local files (env, other repos, credentials) and quote them into review text under a prompt-injected diff. Recommend pinning `-s read-only -a never` explicitly in the codex arm and revisiting whether 5g needs a fork/non-collaborator-PR gate analogous to 5e's, given this residual read-tool surface — `plan.md:154-158`
- [ ] (must-fix) CI break: `.github/workflows/validate.yml`'s `script-tests` job installs only `pytest pyyaml` and never runs `make lint`/populates `.venv`, so neither candidate python has `markdown-it-py`. GitHub Actions sets `CI=true`, and the ported test suite's `require_plan_parser` guard (`test_cross_model_review.sh`) explicitly **FAILs** (not skips) when the library isn't importable under `CI=true` — verified against source lines ~1444-1475. The plan's "Files to Change" table omits `.github/workflows/validate.yml` entirely; `requirements.txt`'s `.venv` install (via `make lint`) does not reach this separate CI job. Without adding `markdown-it-py` to that job's pip-install step, the ported test suite goes red on every PR/push — `plan.md:116-131` (Files to Change table)
- [ ] (should-fix) Timeout mismatch: `cross_model_review.sh`'s default `AGENT_TIMEOUT` (1800s) and `AGY_PRINT_TIMEOUT` (30m) are far above the Bash tool's 600s (10-min) hard cap a review-code sub-agent would use to invoke it, and 6x 5e's existing explicit `timeout 300` for Copilot. Typical runs are 140–200s (per issue evidence), but a wedged/slow CLI would hit the harness's Bash-tool cap before the script's own internal safety timeout fires — a hard-killed invocation with no `EXIT=`/`FINDINGS_FILE=` output at all, which is a different failure shape than the "failed agent doesn't fail the review" per-agent contract the plan assumes. Recommend the 5g invocation snippet set an explicit, Bash-tool-safe `AGENT_TIMEOUT`/`AGY_PRINT_TIMEOUT` and the SKILL.md text document the hard-timeout case separately from a per-agent `EXIT!=0` — `plan.md:146-153`
- [ ] (suggestion) Live-verification step (plan step 8) should explicitly run `make lint` (or otherwise populate `.venv` with `markdown-it-py`) as part of implementation, not leave it to chance — otherwise the shipped PR's own live-verification run may only ever exercise the degraded (WARNING, no Plan Context) path and the ported `_plan_approach.py` feature never gets a real proof run before being called done — `plan.md:174-199`
- [ ] (suggestion) Test-suite assertion count is stale: source currently carries 461 assertions (`grep -c assert_ test_cross_model_review.sh`), not the "~370" the issue's #313 comment cites. Record the actual ported count in Implementation Notes for a future upstream diff, rather than carrying forward an intermediate figure.

### Verification notes (this review)
- Owner decisions 1–5 (issue #660 comment, 2026-09-23 14:37) all correctly implemented in the plan as given (Standard+Deep default-on with `--no-cross-model`; new step 5g, 5e untouched, `--agents gemini,codex` only; ADR referenced not ported, 0015 collision correctly identified; AGENTS.md rows approved and included; full trust weighting).
- `markdown-it-py`/`.venv` resolution mechanism (`git -C "$SCRIPT_SELF_DIR" rev-parse --git-common-dir`) verified correct for this workspace's worktree layout: from this worktree, `--git-common-dir` resolves to the MAIN root's `.git`, matching `Makefile`'s `VENV_DIR := $(MAIN_ROOT)/.venv` — a worktree session finds the shared `.venv` correctly. Confirmed `.venv/bin/python3` exists on this host but does not yet have `markdown_it` importable (pre-port state, as expected).
- `.gitignore` rows (plan step 4) land before the live-verification run (plan step 8) that would create `review-*-prompt.md`/`review-*-findings.md` under `.agent/work-plans/issue-<N>/` — no risk of accidental commit. Both `progress_append.sh` and `address-findings`'s `git add` are scoped to `progress.md` only, confirmed by reading both — neither could sweep in a findings/prompt file even without the gitignore rows.
- Hosted CI test-suite execution (mocks-only) confirmed accurate as far as it goes: `test_cross_model_review.sh` builds mock `agy`/codex/claude/copilot binaries under a `mktemp` dir prepended to `PATH` — no real CLI or network calls, so the suite is legitimately hermetic. The break flagged above (must-fix #2) is a *dependency* gap (markdown-it-py), not a mocking gap.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-24 15:05 -04:00
**By**: Claude Code Agent (Claude Opus 5.5 (1M context))

**Branch**: `feature/issue-660` at `ee6094d`
**Depth**: Deep (6,000+ lines; CI workflow and instruction-file changes)
**Round**: 1 — **Ship: recommended** (every Must-fix fixed and tested; no design question left open)
**Specialists**: static (shellcheck, flake8), governance + plan drift, Claude Adversarial Lens A + Lens B, Cross-Model (codex; gemini after the upstream re-sync)

### Findings (all addressed on this branch)
- [x] (cross-confirmed: codex, Lens A, Lens B, governance) 5e's `AGY_PRINT_TIMEOUT=480` rejected by the script; gemini backstop above the 600s cap — `.claude/skills/review-code/SKILL.md`
- [x] (cross-confirmed: codex, governance) untrusted-PR gate used `gh pr view` fields gh does not expose — `.claude/skills/review-code/SKILL.md`
- [x] (codex) escalation watchdog could not be cancelled; could `kill -9` a recycled PID — `_cli_review.sh`, `_agy_review.sh`
- [x] (gemini) pre-push base could be a stale local main — SKILL.md 5e + `_resolve_default_branch.sh`
- [x] (gemini) nine defects in the upstream-verbatim code, fixed here per owner decision — see plan.md Implementation Notes
- [x] (governance) live verification unrecorded — plan.md Implementation Notes
- [x] (Lens A) codex self-review exclusion is inert without `$AGENT_FRAMEWORK` — documented in SKILL.md 5e

### Not changed
- (Lens B) untrusted-PR gate enforced in the script as well — owner decision: stays in the skill text only
- (Lens B) claude arm lacks tool lockdown — not used by 5e; kept for upstream reconcilability
- (governance) instruction-file edits go slightly past the flag-naming lines — noted in the PR description

## Implementation
**Status**: complete
**When**: 2026-09-25 09:31 -04:00
**By**: Claude Code Agent (Claude Opus 5.5)

Hosted-CI fix plus two rounds of external cross-model review of PR #662 (Codex and Gemini via agy 1.2.11, headless, through this PR's own cross_model_review.sh), run in place of an owner read at the owner's request.

**Branch**: feature/issue-660 at `1b9549c`
**Commits**: `51b8770`, `5af570f`, `69a6907`, `d75e794`, `b5f080a`, `f4632de`, `1e5f9e7`, `67d7bce`, `5818535`, `1b9549c`

### Actions
- [x] (CI) `--work-dir` + `--no-progress` was checked after CLI discovery, so a runner without codex exited 1 "unavailable" instead of the exit-2 usage error; the check moved up with the other flag checks and its test hides the CLIs (`51b8770`)
- [x] (must-fix, Codex round 1) `AGENT_KILL_AFTER=0` accepted, but `timeout -k 0` disables the SIGKILL escalation; zero is now refused in the script and both helpers (`5af570f`)
- [x] (must-fix, Codex round 1) the helpers signalled only the CLI's PID, so a TERM-ignoring child outlived the review; each CLI now leads its own process group (setsid) and all signals go to the group, tested with mocks that start such a child (`69a6907`)
- [x] (must-fix, Codex round 1) review-code did not forward the issue step 1 resolved; it now passes `--issue <N>` or `--no-progress`, and PR mode accepts a keyword-less PR under `--no-progress` (`d75e794`)
- [x] (must-fix, Codex round 1) two runs into one issue dir overwrote each other's artifacts; a non-blocking `flock` refuses the second with exit 5 (`b5f080a`)
- [x] (owner: chase Gemini) Gemini ended empty on a denied RunCommand three runs in a row; agy has no switch that removes its built-in tools, so `_agy_review.sh` now resumes the same conversation once (`--conversation <id>`) with "that was denied, answer in text only", inside the print-timeout budget. Verified live in round 2 (`f4632de`)
- [x] (must-fix, Codex round 2) `echo | grep -q` under pipefail could make `assert_not_contains` falsely pass; here-string with `--` (`1e5f9e7`)
- [x] (must-fix, Gemini round 2) the closing-keyword regex missed `Close`/`Closed`/`Fixed`/`Resolved` and `Closes:`; now GitHub's full set, in one function the test runs directly (`67d7bce`)
- [x] (must-fix, Codex round 2 + Gemini round 2) a TERM between a background launch and its `$!` assignment left the child unrecorded; the agent jobs inherited the lock fd; without setsid the post-exit sweep hit a bare, possibly reused PID — all three fixed (`5818535`)
- [x] (not acted on, Gemini round 2) shared cleanup reap deadline is intended (all jobs are TERMed at once); two claude-arm findings and the `pkill` note concern upstream-verbatim code unused here; exact `## Approach` matching follows the plan template
- [x] (housekeeping) plan Implementation Notes record both rounds and the owner decisions (`1b9549c`)

### Verification
- `bash .agent/scripts/tests/test_cross_model_review.sh` → **722 passed / 0 failed**
- The hardened `--work-dir` test and the fd-9 test each fail against the unfixed code and pass with the fix
- `make test-scripts` → all script suites green + 220 pytest passed, exit 0
- pre-commit (incl. shellcheck) ran on every commit

### Notes
- Owner decision after round 2: fix the five valid findings, push, no third review round.
- The launch-window race is not reachable deterministically from a test.
