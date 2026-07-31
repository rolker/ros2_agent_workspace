# Plan: run-issue checkpoints — generalize "stand on its own" to all operator-facing surfaces

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/592

## Context

PR #593 added the re-orientation header to `AskUserQuestion` checkpoint dialogs in
`run-issue/SKILL.md`. Within hours of merge, the same context-failure recurred on two
other surfaces: (1) Bash `description` fields in permission prompts, and (2) transition/
status reports between phases that emitted bare issue/PR numbers. The issue was reopened
with three required additions.

## Approach

1. **Extend § Checkpoints in `run-issue/SKILL.md`** — add rules for the two uncovered surfaces:
   - **Bash `description` fields**: every orchestration Bash command must open its
     `description` with `<repo>#<N> <phase>: …` (the description is the only context
     in a permission dialog).
   - **Transition/status reports**: each between-phase report must open with
     `<repo>#<N>` + a plain-words statement of what the work is (not bare numbers).
   - Update the "Between checkpoints" sentence (currently line 313) to embed the format
     requirement.

2. **Extend the prompt-free section** — add: task-output files written by sub-agents to
   `/tmp` must be read with the `Read` tool or `progress_read.py`, not Bash grep
   (Bash reads outside the project dir prompt; defeats #594's prompt-free goal).

3. **Evaluate mechanical enforcement (PreToolUse hook)** — ADR-0004/0005 require a
   documented conclusion, not another "backlog note":
   - **`AskUserQuestion` hook**: technically feasible — hook reads the `question` field
     from the tool-call JSON, checks for `<repo>#<N>` prefix, gates on `$WORKTREE_ISSUE`
     to limit to lifecycle sessions. Implement in `.claude/settings.json` `hooks` section.
   - **Bash `description` hook**: not feasible to scope without pervasive false positives —
     all Bash calls would need the format, including non-orchestration commands; the hook
     cannot reliably distinguish. Record infeasibility in a code comment and in the issue
     comment before closing.

4. **Check adapter files** — verify `.github/copilot-instructions.md` and
   `.agent/instructions/gemini-cli.instructions.md` for references to the Checkpoints
   section or to the surfaces being generalized; update if stale.

5. **`triage-reviews/SKILL.md`** — the handoff section (lines 375-381) already covers
   both `/run-issue` and hand-driven sessions; no change needed (verified).

## Files to Change

| File | Change |
|------|--------|
| `.claude/skills/run-issue/SKILL.md` | Add Bash-description + transition rules to § Checkpoints; extend prompt-free section with `/tmp` read-tool note |
| `.claude/settings.json` | Add `PreToolUse` hook for `AskUserQuestion` prefix check |
| `.github/copilot-instructions.md` | Verify/update if references to Checkpoints are stale |
| `.agent/instructions/gemini-cli.instructions.md` | Same check |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Directly serves it — the three surfaces are exact points where situational awareness breaks down |
| Enforcement over documentation | Hook evaluation is required by the issue and ADR-0004/0005; must reach a conclusion, not another backlog note |
| A change includes its consequences | Check adapter files for stale references |
| Only what's needed | Two skill-file edits + one hook; no new abstractions |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0004 — Enforcement hierarchy | Yes | PreToolUse hook for AskUserQuestion evaluated and implemented (or infeasibility documented) |
| ADR-0005 — Layered enforcement | Yes | Same — hook is the mechanical layer above instruction prose |
| ADR-0006 — Shared AGENTS.md | Watch | Adapter files checked for Checkpoints references |

## Consequences

| If we change... | Also update... | Included? |
|---|---|---|
| `run-issue/SKILL.md` § Checkpoints | adapter files (copilot, gemini) | Yes — step 4 |
| PreToolUse hook | `.claude/settings.json` hooks section | Yes — step 3 |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): adapter files if they reference the Checkpoints
  section with outdated wording; prompt-free section in run-issue itself.
- **Agent-instruction candidates**: None — the surfaces covered are orchestration-only;
  the hook provides machine enforcement, not a new knowledge pattern.

## Open Questions

- [ ] Does the Claude Code hook mechanism reliably receive `AskUserQuestion` parameters
  in the format assumed (JSON with `question` key)? Verify before implementing the hook,
  or stub the hook + document if verification fails.

## Estimated Scope

Single PR — docs-only unless the hook is feasible, in which case one small `.claude/settings.json` edit.
