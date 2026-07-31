---
issue: 594
---

# Issue #594 — progress_append.sh: eliminate finish-phase permission prompts

## Issue Review
**Status**: complete
**When**: 2026-07-31 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #594
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Scope Assessment

The issue proposes two deliverables: (1) a new `.agent/scripts/progress_append.sh` script that reads progress entry text from stdin and performs a scoped `git add` + commit, and (2) guidance updates in three skills (run-issue, triage-reviews, review-code step 8) to use `jq`/`progress_read.py` for parsing and `progress_append.sh` for writes. Both fit in a single PR and address a concrete pain (250 heredoc occurrences, 120 `cat >>` occurrences triggering permission prompts). The scope boundary is well-defined: push, PR creation, and merge are explicitly kept as human-gated checkpoints.

The issue is a workspace infrastructure change (scripts, settings, skill guidance) — correctly placed in the workspace repo.

**Dependencies**: Related to #592 (checkpoint re-orientation context); the two are complementary and independent. #594 is not blocked by #592.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Automation scope is narrow and explicit; human gates (push, PR, merge) are preserved; allowlist entry makes the new automation visible |
| Enforcement over documentation | OK | Fix is a script + allowlist entry, not just docs; guidance is behavioral (agent instructions), not a new code correctness rule |
| Capture decisions, not just implementations | Watch | Narrow-script pattern follows established dlog.sh precedent; no new ADR proposed. Acceptable given the precedent, but a brief ADR addendum noting the pattern extension would be valuable |
| A change includes its consequences | Action needed | Consequences map: a new script in `.agent/scripts/` requires an update to AGENTS.md's script reference table. The issue body does not mention this. Implementation PR must include it. |
| Only what's needed | OK | Script is minimal by design; the scope discipline (no arbitrary paths, no arbitrary commit messages beyond entry-type slot) is explicitly stated |
| Improve incrementally | OK | Small, scoped change following an established precedent |
| Test what breaks | Watch | No tests mentioned for `progress_append.sh`. dlog.sh has no dedicated test either — consistent with precedent — but given that this script commits to git, a smoke test (dry-run mode or test-repo fixture) would reduce regression risk |
| Workspace vs. project separation | OK | Pure workspace infrastructure |
| Workspace improvements cascade to projects | OK | Pattern is reusable; other repos could adopt the same approach |
| Primary framework first, portability where free | OK | Allowlist entry is Claude Code-specific (`.claude/settings.json`), which is appropriate here |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0004 — Enforcement hierarchy | Partial | Skill guidance update is the documentation layer; no hook or CI check proposed to catch regressions to `cat >>` or ad-hoc heredoc patterns. Acceptable for agent behavioral guidance, but worth noting the absence of a mechanical enforcement layer |
| ADR-0006 — Shared AGENTS.md | Yes | New script requires an entry in AGENTS.md's script reference table (see "A change includes its consequences" above) |
| ADR-0013 — progress.md vocabulary | Yes | `progress_append.sh` must append entries conforming to ADR-0013 types. The "fixed `progress: <entry type> for #<N>`" commit message pattern is correct; implementation should document or validate which entry types the script accepts |

### Consequences

- New script → **AGENTS.md script reference table** must include `progress_append.sh` (not mentioned in issue; must be added to PR scope per consequences map).
- Allowlist entry format should follow the existing pattern: `Bash(.agent/scripts/progress_append.sh:*)` — confirm consistency with existing entries in `.claude/settings.json`.
- If the skill guidance is added to AGENTS.md (rather than only inside `.claude/skills/`), then framework adapters (`.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`) may need a note — but since this is a Claude Code allowlist mechanism, a Claude-skill-only guidance update is sufficient.

### Actions
- [ ] Add `progress_append.sh` entry to AGENTS.md script reference table in the same PR.
- [ ] Confirm allowlist entry uses the anchored `Bash(.agent/scripts/progress_append.sh:*)` form consistent with existing `.claude/settings.json` entries.
- [ ] Consider a smoke test for `progress_append.sh` (at minimum, verify it fails gracefully on a bad issue number or non-existent progress.md parent dir).

## Plan Authored
**Status**: complete
**When**: 2026-07-31 16:19 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-594/plan.md` at `af42e6f`
**Branch**: feature/issue-594 at `af42e6f`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.

## Plan Review
**Status**: complete
**When**: 2026-07-31 16:23 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-594/plan.md` at `af42e6f`
**PR**: PR-less (--issue mode; gh unauthenticated in this container)
**Verdict**: changes-requested

<!-- Independence: plan authored by "Claude Code Agent (Claude Sonnet)"; this review
     is a separate fresh-context dispatch on a different model (Opus). Genuinely
     independent — no author self-review annotation. -->

### Findings
- [x] (must-fix) Step 2 targets `.claude/settings.json` as a committed "Files to Change" item, but that file is **not git-tracked** (never in history; only exists untracked in the developer's main tree). The "existing entries (dispatch_subagent.sh, dlog.sh, dashboard.sh)" it claims consistency with live in that untracked local file (dlog is actually in `settings.local.json`). As written, the allowlist edit cannot land in the PR and won't propagate to other agents/machines — defeating the issue's "prompt-free thereafter" goal. Decide the mechanism: follow the dlog precedent (document a manual "allowlist once" step adding `Bash(<workspace_root>/.agent/scripts/progress_append.sh:*)` to `.claude/settings.local.json` in the skill guidance), or deliberately introduce a *new committed* shared `.claude/settings.json` and call that out (it's a first — a tracked shared-settings file — and interacts with the untracked local one; may warrant an Ask-First note). — `plan.md:39-42`, `plan.md:69`
- [x] (suggestion) Allowlist anchoring: the proposed relative form `Bash(.agent/scripts/progress_append.sh:*)` only matches when invoked exactly that way from the workspace root; the dlog precedent uses the absolute `Bash(<workspace_root>/.agent/scripts/dlog.sh:*)` form for robustness across cwd. Since progress writes happen from both the host (workspace root) and worktrees, prefer the absolute form or document the invocation contract. — `plan.md:39`
- [x] (suggestion) Commit identity in `progress_append.sh`: the script commits, so it must carry agent identity robustly. `$AGENT_NAME`/`$AGENT_EMAIL` are frequently lost across fresh subshells (per AGENTS.md), which would fall back to human git config and trip `check_pr_authors.py`. Have the script fail loud when identity is unset, or accept identity as args — note this in the plan's script contract (dlog has no precedent here since it doesn't commit). — `plan.md:31`
- [x] (suggestion) Test placement: `.agent/scripts/test_progress_append.sh` is picked up by `make test-scripts` (run_script_tests.sh globs sibling `test_*.sh`), so it works — but the dlog precedent lives at `.agent/scripts/tests/test_dlog.sh`. Consider `.agent/scripts/tests/` for consistency. Also reconcile the two names in the plan: Approach step 4 says `scripts/test_progress_append.sh`, the Files table says `.agent/scripts/test_progress_append.sh`. — `plan.md:51`, `plan.md:71`

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-31 17:03 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-594 at `a460099`
**Mode**: pre-push
**Depth**: Deep (reason: security-relevant — tracked permission allowlist + script that shells out to git)
**Must-fix**: 2 | **Suggestions**: 5
**Round**: 1 | **Ship**: continue — one cross-pass-confirmed root-cause defect (identity/test); address then re-review

Static analysis clean (shellcheck both scripts, settings.json valid JSON). Both fresh-context Claude Adversarial lenses (A logic, B systemic) independently flagged the same two must-fixes → cross-pass confirmed. Copilot off (default); local off (--no-local, #590). Plan adherence strong (7/7 files, no scope creep, all 3 plan-review suggestions applied).

### Findings
- [x] (must-fix) Identity guarantee defeated by env precedence: script commits via `git -c user.name/email`, but ambient `GIT_AUTHOR_*`/`GIT_COMMITTER_*` (exported by set_git_identity_env.sh) outrank `-c` — silently disables the `--name`/`--email` override and can commit under a stale/human identity (the check_pr_authors.py trip it claims to prevent). Fix: set the four GIT_* vars to the validated identity. — `.agent/scripts/progress_append.sh:100-101`
- [x] (must-fix) Shipped smoke test is non-hermetic — fails 2/10 (cases 1, 8) in the standard agent shell; picked up by run_script_tests.sh so `make test-scripts`/`make validate` go red. Same root cause as above; the deterministic-authorship fix turns it green without masking. — `.agent/scripts/tests/test_progress_append.sh` (cases 1, 8)
- [x] (suggestion) Machine-specific absolute paths (`/home/roland/project11/...`) baked into the now-shared tracked settings.json, redundant with the relative forms — drop the absolute duplicates. — `.claude/settings.json:5,6,30,32`
- [x] (suggestion) First tracked *shared* allowlist also shares git fetch/gh search/journalctl/etc.; confirm each entry is intentionally shareable + note settings.local.json still layers on top. — `.claude/settings.json:3-33`
- [x] (suggestion) CRLF/trailing-space heading leaks `\r`/double-space into the commit subject; trim trailing whitespace from ENTRY_TYPE. — `.agent/scripts/progress_append.sh:81`
- [x] (suggestion) Non-idempotent on commit failure: entry appended before commit, so a naive re-run after exit-3 double-appends. — `.agent/scripts/progress_append.sh:97-104`
- [x] (suggestion) `git commit -- <pathspec>` would desync tree vs HEAD if a hook mutates progress.md and exits 0 (latent — no such hook today); add a note. — `.agent/scripts/progress_append.sh:101`

**Dogfood note**: this entry was written and committed by the shipped `progress_append.sh` — it worked (appended + scoped commit succeeded). Commit landed correctly under the agent identity here only because ambient `GIT_AUTHOR_*` equals the agent identity in this session — which is exactly why must-fix #1 stays latent on the happy path.

**Host note for address-findings**: the shared-settings direction is an operator decision made at the plan checkpoint — do not revert it. For the shareability suggestion (#4 in the list): apply the mechanical half (drop the absolute-path duplicate entries, add a one-line layering comment/doc note); the entry-by-entry "intentionally shareable" confirmation is deferred to the operator at PR review — list the entries needing eyes in your Implementation entry instead of deciding for them. For the idempotency suggestion: a re-run guard (skip append if the file's last entry heading+When matches stdin, or check for an identical uncommitted tail) is in scope; full transactionality is not.

## Implementation
**Status**: complete
**When**: 2026-07-31 17:15 +00:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-594 at `7540f57`
**Addressed**: `## Local Review (Pre-Push)` (2026-07-31 17:03 +00:00, branch `a460099`) — 2 must-fix + 5 suggestions, all 7 actioned
**Commits**: `a4d59ee` `e603702` `09973b7` `de62a8a` `7540f57`

### Actions
- [x] (must-fix) Identity guarantee defeated by env precedence — pin author+committer via `GIT_AUTHOR_*`/`GIT_COMMITTER_*` in the commit invocation so ambient exports can't override the validated identity; `-c` kept as fallback — `.agent/scripts/progress_append.sh:94-107` (`a4d59ee`)
- [x] (must-fix) Non-hermetic smoke test (cases 1, 8) — resolved by the identity fix above; suite now 10/10 → 12/12 green in the standard agent shell (ambient `GIT_*` set) — `.agent/scripts/tests/test_progress_append.sh` (`a4d59ee`)
- [x] (suggestion) Machine-specific absolute-path allowlist duplicates — dropped the three `/home/roland/project11/...` entries redundant with their relative forms (progress_read.py, setup.bash, set_git_identity_env.sh); kept `/opt/ros/jazzy/setup.bash` (not machine-specific, no relative twin) — `.claude/settings.json` (`de62a8a`)
- [x] (suggestion) Shared-allowlist shareability — mechanical half done: layering note added (see below) + absolute dupes dropped. Entry-by-entry "intentionally shareable" confirmation is **deferred to the operator at PR review** per the review host note (partial: operator confirmation pending) — `.claude/settings.json`, `AGENTS.md:558` (`de62a8a`, `7540f57`)
- [x] (suggestion) CRLF/trailing-space heading leaking into the commit subject — trim the trailing whitespace run from `ENTRY_TYPE`; added regression case 11 — `.agent/scripts/progress_append.sh:75-78` (`e603702`)
- [x] (suggestion) Non-idempotent on commit-failure replay — guard skips the re-append when the file tail already equals the entry (append-then-commit-failed replay); added regression case 12; full transactionality intentionally out of scope — `.agent/scripts/progress_append.sh:91-101` (`09973b7`)
- [x] (suggestion) `git commit -- <pathspec>` tree/HEAD desync — added an in-code note documenting the latent hazard (no such hook today) alongside the identity fix — `.agent/scripts/progress_append.sh:100-102` (`a4d59ee`)

### Deferred to operator (shareability confirmation — finding #4)
The tracked `.claude/settings.json` is the first *shared* allowlist. These entries do more than run local scripts and warrant an operator eyeball before merge for "intentionally shareable across all machines/agents":
- `Bash(git fetch:*)`
- `Bash(gh run watch:*)`, `Bash(gh search:*)`, `Bash(gh label list:*)`
- `Bash(journalctl:*)`
- `Bash(source /opt/ros/jazzy/setup.bash)` (assumes ROS 2 Jazzy at the standard prefix)
Per-machine `.claude/settings.local.json` still layers on top, so a machine can always add (not remove) beyond this baseline — noted in `AGENTS.md:558`.

### Verification
- `bash .agent/scripts/tests/test_progress_append.sh` → 12 passed, 0 failed
- `shellcheck .agent/scripts/progress_append.sh` clean; `.claude/settings.json` valid JSON
- pre-commit hooks green on every fix commit (identity, shellcheck included)

### Next step
Lifecycle: **Implementation** → **review-code** (re-review the fixes). Hand off to a fresh-context sub-agent:
`.agent/scripts/dispatch_subagent.sh --mode in-process --issue 594 --skill review-code`

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-31 17:23 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-594 at `b4c01c5`
**Mode**: pre-push
**Depth**: Deep (reason: security-relevant — first tracked permission allowlist + script that shells out to git commit)
**Must-fix**: 1 | **Suggestions**: 1
**Round**: 2 | **Ship**: recommended — must-fix falling 2→1; the one remaining is a precise, mechanical file:line fix (idempotency exit-code), address then ship rather than loop a full round 3

Static analysis clean (shellcheck both scripts; settings.json valid JSON; test-file SC2015 are info-level, below the --severity=warning profile). Both fresh-context Claude Adversarial lenses ran (A logic, B systemic); Copilot off (default), local off (--no-local, #590). Round-1's two must-fixes (identity env-precedence, non-hermetic test) are correctly resolved — suite 12/12 green. Plan adherence strong (7/7 files, no scope creep, all 3 plan-review suggestions applied). Test verified discovered by tests/run_script_tests.sh (make test-scripts). Lens B: no new must-fix — the :* wildcard pairs with a trusted-operator model and grants no capability beyond plain bash/git; the one propagating item (git fetch:* on the shared tracked settings.json) is already on the operator-deferred shareability list.

### Findings
- [x] (must-fix) Idempotency guard reports false failure on the success path: re-running a byte-identical entry after a successful commit skips the re-append, then git finds a clean tree and the script exits 3 with "commit failed (pre-commit hook?)" though the entry IS committed — violates the exit-3=git-failure contract in the prompt-free host path (verified empirically). Fix: exit 0 when tail matches and nothing to commit. — `.agent/scripts/progress_append.sh:99-104,116-122`
- [x] (suggestion) Idempotency test case 12 asserts only entry count, not exit code — passes despite the spurious exit-3; add an rc assertion to pin exit semantics. — `.agent/scripts/tests/test_progress_append.sh:129-140`

### Next step
Lifecycle: **Local Review** → address-findings (one mechanical must-fix) → re-review, then push / open PR. Verdict is changes-requested, so the host dispatches **address-findings** on the open finding above; Ship: recommended means a full round-3 re-review is optional once the mechanical fix + test assertion land.

## Implementation
**Status**: complete
**When**: 2026-07-31 17:27 +00:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-594 at `25fcc77`
**Addressed**: `## Local Review (Pre-Push)` (2026-07-31 17:23 +00:00, branch `b4c01c5`, Round 2) — 1 must-fix + 1 suggestion, both actioned
**Commits**: `25fcc77`

### Actions
- [x] (must-fix) Idempotency guard false-failure on the success path — after `git add`, exit 0 when nothing is staged for the file (the entry is already committed: a prior run appended AND committed it, so the append guard skipped the re-append). Previously fell through to an empty `git commit` that failed into exit 3, which the prompt-free host path reads as a real git failure. Verified empirically: identical re-run now exits 0. — `.agent/scripts/progress_append.sh:106-116` (`25fcc77`)
- [x] (suggestion) Test case 12 now captures and asserts `rc == 0` on the identical re-run (in addition to the entry-count invariant), pinning the exit semantics the fix guarantees — without it the count-only check passed despite the spurious exit-3. — `.agent/scripts/tests/test_progress_append.sh:129-142` (`25fcc77`)

### Verification
- Repro: identical re-run against a fresh repo now exits 0 (was 3)
- `bash .agent/scripts/tests/test_progress_append.sh` → 12 passed, 0 failed
- `shellcheck .agent/scripts/progress_append.sh` clean
- pre-commit hooks green on the fix commit (shellcheck, identity, no-commit-to-branch included)

### Next step
Lifecycle: **Implementation** → **review-code** (re-review the fixes). Hand off to a fresh-context sub-agent:
`.agent/scripts/dispatch_subagent.sh --mode in-process --issue 594 --skill review-code`
Round-2 Ship was "recommended" (must-fix 2→1, mechanical), so a full round-3 re-review is optional once this mechanical fix + rc assertion land.
