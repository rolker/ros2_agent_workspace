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
- [ ] (must-fix) Step 2 targets `.claude/settings.json` as a committed "Files to Change" item, but that file is **not git-tracked** (never in history; only exists untracked in the developer's main tree). The "existing entries (dispatch_subagent.sh, dlog.sh, dashboard.sh)" it claims consistency with live in that untracked local file (dlog is actually in `settings.local.json`). As written, the allowlist edit cannot land in the PR and won't propagate to other agents/machines — defeating the issue's "prompt-free thereafter" goal. Decide the mechanism: follow the dlog precedent (document a manual "allowlist once" step adding `Bash(<workspace_root>/.agent/scripts/progress_append.sh:*)` to `.claude/settings.local.json` in the skill guidance), or deliberately introduce a *new committed* shared `.claude/settings.json` and call that out (it's a first — a tracked shared-settings file — and interacts with the untracked local one; may warrant an Ask-First note). — `plan.md:39-42`, `plan.md:69`
- [ ] (suggestion) Allowlist anchoring: the proposed relative form `Bash(.agent/scripts/progress_append.sh:*)` only matches when invoked exactly that way from the workspace root; the dlog precedent uses the absolute `Bash(<workspace_root>/.agent/scripts/dlog.sh:*)` form for robustness across cwd. Since progress writes happen from both the host (workspace root) and worktrees, prefer the absolute form or document the invocation contract. — `plan.md:39`
- [ ] (suggestion) Commit identity in `progress_append.sh`: the script commits, so it must carry agent identity robustly. `$AGENT_NAME`/`$AGENT_EMAIL` are frequently lost across fresh subshells (per AGENTS.md), which would fall back to human git config and trip `check_pr_authors.py`. Have the script fail loud when identity is unset, or accept identity as args — note this in the plan's script contract (dlog has no precedent here since it doesn't commit). — `plan.md:31`
- [ ] (suggestion) Test placement: `.agent/scripts/test_progress_append.sh` is picked up by `make test-scripts` (run_script_tests.sh globs sibling `test_*.sh`), so it works — but the dlog precedent lives at `.agent/scripts/tests/test_dlog.sh`. Consider `.agent/scripts/tests/` for consistency. Also reconcile the two names in the plan: Approach step 4 says `scripts/test_progress_append.sh`, the Files table says `.agent/scripts/test_progress_append.sh`. — `plan.md:51`, `plan.md:71`
