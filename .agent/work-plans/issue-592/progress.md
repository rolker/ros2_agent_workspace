---
issue: 592
---

# Issue #592 — run-issue checkpoints: require repo-qualified re-orientation context in every AskUserQuestion

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-31 11:39 -04:00
**By**: Claude Code Agent (Claude Fable 5)
**Verdict**: approved

**Branch**: feature/issue-592 at `718498a`
**Mode**: pre-push
**Depth**: Standard (reason: `.claude/skills/*/SKILL.md` governance override trigger)
**Must-fix**: 0 | **Suggestions**: 3
**Round**: 1 | **Ship**: recommended — no must-fix; suggestions are mechanical and applied in-branch

### Findings
- [x] (suggestion) Pin "repo slug" to the GitHub repo name, not the local directory name (dir `project11` ≠ repo `ros2_agent_workspace`) — `.claude/skills/run-issue/SKILL.md:272` (fixed in-branch)
- [x] (suggestion) triage-reviews inline header template paraphrases the canonical format (`<phase>` vs `phase X of Y (<phase name>)`) — drift seed, cross-confirmed by all 3 specialists — `.claude/skills/triage-reviews/SKILL.md:388` (fixed in-branch: matches canonical verbatim)
- [ ] (suggestion) No mechanical enforcement layer for the header (ADR-0004/0005 Watch): a PreToolUse hook could assert the `<repo>#<N>` prefix — backlog note, not this PR — `.claude/skills/run-issue/SKILL.md:271`

## Issue Review
**Status**: complete
**When**: 2026-07-31 19:51 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #592
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Summary

Issue #592 was originally about `AskUserQuestion` checkpoint dialogs lacking
context (repo, issue title, PR number). PR #593 addressed that surface. The
issue was then reopened because the same "stand on its own" failure recurred
on two other surfaces within hours of the merge:

1. **Bash `description` fields** — permission-prompt descriptions carried no
   repo/issue/phase context (the description is the only context channel for a
   permission dialog).
2. **Between-phase status messages** — transition reports emitted bare
   issue/PR numbers with no statement of what the work is.

The reopened scope adds three items:

1. Generalize the rule in `run-issue/SKILL.md` § Checkpoints to cover all
   three operator-facing surfaces (AskUserQuestion — already done; Bash
   description fields; transition/status reports).
2. Evaluate a PreToolUse hook that warns/rejects `AskUserQuestion` whose
   question lacks a `<repo>#<N>` prefix (and whether the same is feasible for
   Bash descriptions). If not feasible, document why in the issue before
   closing.
3. Fold in the prompt-free tweak: task-output files under `/tmp` should be
   read with the Read tool / `progress_read.py`, not Bash grep (which prompts
   outside the project dir, defeating #594's prompt-free goal).

All three items target `run-issue/SKILL.md` (primary) and `triage-reviews/SKILL.md`
(secondary — one-line reminder about the full header on the fix-vs-defer
checkpoint). The `triage-reviews` handoff section already carries a reminder
from #593; the issue asks to verify it covers hand-driven runs and update if
needed.

**Right repo**: Workspace repo ✓ — all changes are skill files under `.claude/skills/`.

**Dependencies**: PR #593 is merged and is context only. No blocking deps.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Fix directly serves this principle — the three surfaces are the exact points where human situational awareness breaks down across concurrent sessions |
| Enforcement over documentation | Watch | Item 2 requires a documented conclusion (hook implemented or infeasibility recorded). The #593 pre-push review already flagged this as a backlog note; the issue explicitly says this must not slip again — "record why in the issue before closing again" |
| A change includes its consequences | Watch | Changing `run-issue/SKILL.md` — check `.github/copilot-instructions.md` and `.agent/instructions/gemini-cli.instructions.md` for references that may need updating. If a PreToolUse hook is implemented, it needs entries in `.claude/settings.json` and `AGENTS.md` |
| Only what's needed | OK | Targeted to two skill files; no abstraction beyond what's required |
| Improve incrementally | OK | Small, docs-only change (unless the hook evaluation produces code) |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0004 — Enforcement hierarchy | Yes | Item 2 requires evaluation of a mechanical enforcement layer (hook or CI check) — the issue explicitly invokes ADR-0004/0005 |
| ADR-0005 — Layered enforcement | Yes | Same as ADR-0004 — both are triggered whenever a new compliance rule is proposed |
| ADR-0006 — Shared AGENTS.md | Watch | If the behavior visible in adapter files (copilot-instructions, gemini-cli) references the Checkpoints section, those may need a touch |
| ADR-0013 — progress.md entry-type vocabulary | No | No new entry types; this is a skill-prose change only |

### Consequences

From the consequences map:
- Changing a framework skill → check adapter files (`.github/copilot-instructions.md`,
  `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md`) for
  skill-list or behavior references that reference the surfaces being generalized.
- If a PreToolUse hook is implemented → add it to `.claude/settings.json` (hooks
  section) and document it in the AGENTS.md script reference table.

### Actions
- [ ] Mechanical enforcement evaluation must reach a documented conclusion (hook or explicit infeasibility note) — not another "backlog note, not this PR"; the issue requires it before closing.
- [ ] After implementing skill-file changes, check adapter files (`.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`) for stale references to the Checkpoints section or surfaces being generalized.
- [ ] If a PreToolUse hook is created, add it to `.claude/settings.json` and document in AGENTS.md.

## Plan Authored
**Status**: complete
**When**: 2026-07-31 20:45 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-592/plan.md` at `386a5eb`
**Branch**: feature/issue-592 at `386a5eb`
**Phases**: single

### Open questions
- [ ] Does the Claude Code hook mechanism reliably receive `AskUserQuestion` parameters in the format assumed (JSON with `question` key)? Verify before implementing the hook, or stub and document if verification fails.

## Plan Review
**Status**: complete
**When**: 2026-07-31 20:48 +00:00
**By**: Claude Code Agent (Claude Opus)  <!-- independent: fresh-context dispatch on a different model than the Sonnet plan author; shared workspace $AGENT_NAME is not self-review -->

**Plan**: `.agent/work-plans/issue-592/plan.md` at `386a5eb`
**PR**: PR-less (--issue / worktree mode; no GitHub auth in container)
**Verdict**: approve-with-suggestions

### Findings
- [x] (suggestion) AskUserQuestion PreToolUse hook over-fires: it gates on `$WORKTREE_ISSUE`, which is set in *any* worktree session, not only run-issue orchestration — the same "can't distinguish orchestration" failure mode the plan uses to reject the Bash-`description` hook. Reconcile: scope more tightly or document the accepted false-positive rate — `plan.md:33-37` (resolved by operator decision 2026-07-31: hook is warn-only + always-on; `$WORKTREE_ISSUE` gate dropped — over-firing costs only a nudge)
- [x] (suggestion) If implemented, the hook check belongs in a versioned `.agent/hooks/*.py` script (matching `check-commit-identity.py` / `check_pr_authors.py`), not inline in settings.json; add that file to Files-to-Change — `plan.md:51` (done: logic in `.agent/hooks/check_question_context.py`; settings.json only wires it)
- [x] (suggestion) Consequence gap: Issue Review requires documenting a new hook in the AGENTS.md script-reference table (existing `.agent/hooks/*.py` are listed there); plan's Consequences table lists only `.claude/settings.json` — `plan.md:75-78` (done: AGENTS.md row added to Files-to-Change + Consequences table)
- [x] (suggestion) Verified — adapter files (`.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`) have no Checkpoints/AskUserQuestion references; step 4 resolves to "no change." Mark those rows verify-only in Files-to-Change — `plan.md:53-54` (done: rows marked verify-only; re-verified in implementation — still no references)

## Implementation
**Status**: complete
**When**: (stamped by progress_append.sh)
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-592
**Scope**: reopened #592 — generalize "stand on its own" to all operator-facing surfaces + warn-only enforcement hook

### What was done (atomic commits)
1. **Plan-sync** (`9b99aac`) — folded in the operator decision (AskUserQuestion
   hook is **warn-only + always-on**; the `$WORKTREE_ISSUE` gate is **dropped**
   because it over-fires on any worktree session — a false positive costs only a
   nudge) and the four Plan Review amendments; checked off all four Plan Review
   findings in progress.md.
2. **Hook mechanism** (`4100d97`) — new `.agent/hooks/check_question_context.py`
   (`PreToolUse`, matcher `AskUserQuestion`): nudges when no `questions[].question`
   opens with a `\S+#\d+` repo-qualified token in its header window. **Warn-only**
   (never a deny/block decision) and **fail-safe** (any parse error / unexpected
   schema / exception → exit 0 silent). Nudge rides `hookSpecificOutput
   .additionalContext` + `systemMessage`. `.claude/settings.json` only wires it.
   7-case smoke test at `.agent/scripts/tests/test_check_question_context.sh`
   (conforming silent / non-conforming warn+exit 0 / no deny-block / mixed /
   malformed / empty / no-questions) — runs via `run_script_tests.sh` glob, all
   pass. AGENTS.md hooks-table row added. Bash-description-hook **infeasibility**
   recorded in the script header comment (no reliable orchestration signal in the
   tool-call JSON to distinguish orchestration Bash from ordinary Bash).
3. **run-issue prose** (`82aa7cc`) — § Checkpoints reframed around **three**
   surfaces (AskUserQuestion dialogs; Bash `description` fields → open with
   `<repo>#<N> <phase>: …`; transition/status reports → open with repo#issue +
   plain-words work statement, never bare numbers; the between-checkpoints
   narration sentence now requires the repo-qualified form). Prompt-free section
   extended: task-output files under `/tmp` are read with the `Read` tool /
   `progress_read.py`, never Bash grep (an out-of-project Bash read prompts).

### Verified, no change needed
- **triage-reviews/SKILL.md** — the Checkpoint-context paragraph (`:375-381`)
  already names "the `/run-issue` host, or a hand-driven session" and points at
  run-issue § Checkpoints as the single format source. Covers hand-driven runs;
  no edit.
- **Adapter files** — re-verified `.github/copilot-instructions.md` and
  `.agent/instructions/gemini-cli.instructions.md`: no Checkpoints/AskUserQuestion
  references (grep clean). Verify-only, as the Plan Review amendment expected.

### Notes for the host / next step
- **Bash-description infeasibility issue comment** is NOT posted here (this
  container dispatch has no `gh` auth). The host should post the infeasibility
  conclusion (mirrored from the hook header comment) on #592 at close time.
- **Pre-existing, unrelated test failure**: `run_script_tests.sh` reports
  `test_check_commit_identity.sh` failing ("strict: should have rejected on-disk
  human email…"). It is **env-driven** (on-disk git config + `AGENT_NAME` set in
  this worktree) and touches identity files this branch does **not** modify — not
  introduced by this work. Flagging, not fixing (would need its own investigation).
- Next: host review-code → publish checkpoint → push + open PR (`Closes #592`).
