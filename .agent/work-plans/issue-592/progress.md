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
