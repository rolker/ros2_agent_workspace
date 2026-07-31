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
   - **`AskUserQuestion` hook**: implemented as **warn-only and always-on**
     (operator decision, plan checkpoint 2026-07-31). The hook logic lives in a
     versioned script `.agent/hooks/check_question_context.py` (matching
     `check-commit-identity.py` conventions; `.claude/settings.json` only wires it
     as a `PreToolUse` matcher for `AskUserQuestion`). It reads the tool-call JSON
     from stdin and, when **no** `questions[].question` opens with a `<repo>#<N>`
     token, emits a non-blocking nudge to add the re-orientation header. It
     **never blocks or denies** — a false positive costs a nudge, nothing else —
     and **fails SAFE**: any parse error, unexpected schema, or internal exception
     exits 0 silently so the tool call is never broken. The `$WORKTREE_ISSUE` gate
     from the original plan is **dropped** (Plan Review finding: it fires in *any*
     worktree session, not only run-issue orchestration — the same
     can't-distinguish failure mode used to reject the Bash-`description` hook).
     Warn-only + always-on sidesteps that: over-firing costs only a nudge.
   - **Bash `description` hook**: not feasible to scope without pervasive false positives —
     all Bash calls would need the format, including non-orchestration commands; the hook
     cannot reliably distinguish. Record infeasibility in a **comment block in the hook
     script header** (`check_question_context.py`); the matching **issue comment is posted
     by the host at close time** (no `gh` in this container dispatch — do not attempt it).

4. **Check adapter files** — verify `.github/copilot-instructions.md` and
   `.agent/instructions/gemini-cli.instructions.md` for references to the Checkpoints
   section or to the surfaces being generalized; update if stale.

5. **`triage-reviews/SKILL.md`** — the handoff section (lines 375-381) already covers
   both `/run-issue` and hand-driven sessions; no change needed (verified).

## Files to Change

| File | Change |
|------|--------|
| `.claude/skills/run-issue/SKILL.md` | Add Bash-description + transition rules to § Checkpoints; extend prompt-free section with `/tmp` read-tool note |
| `.agent/hooks/check_question_context.py` | **New** — warn-only, always-on hook logic (reads tool-call JSON, checks `<repo>#<N>` token, fails safe); Bash-description infeasibility recorded in header comment |
| `.claude/settings.json` | Wire the `PreToolUse` `AskUserQuestion` matcher to the script (no inline logic) |
| `.agent/scripts/tests/test_check_question_context.sh` | **New** — smoke test (conforming → silent; non-conforming → warn + exit 0; malformed/empty → silent exit 0); runs via `run_script_tests.sh` glob |
| `AGENTS.md` | Add `check_question_context.py` row to the script-reference hooks table |
| `.github/copilot-instructions.md` | **Verify-only** — reviewer confirmed no Checkpoints/AskUserQuestion references; re-verify, expect no change |
| `.agent/instructions/gemini-cli.instructions.md` | **Verify-only** — same |

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
| `run-issue/SKILL.md` § Checkpoints | adapter files (copilot, gemini) | Yes — step 4 (verify-only; no refs found) |
| PreToolUse hook | `.claude/settings.json` hooks section | Yes — step 3 |
| Add `.agent/hooks/*.py` hook | AGENTS.md script-reference table | Yes — step 3 (Plan Review consequence gap) |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): adapter files if they reference the Checkpoints
  section with outdated wording; prompt-free section in run-issue itself.
- **Agent-instruction candidates**: None — the surfaces covered are orchestration-only;
  the hook provides machine enforcement, not a new knowledge pattern.

## Open Questions

- [x] Does the Claude Code hook mechanism reliably receive `AskUserQuestion` parameters
  in the format assumed (JSON with `question` key)? **Resolved by design:** the hook
  fails SAFE — if the stdin JSON is absent, malformed, or shaped differently than
  assumed, it exits 0 silently and the tool call proceeds. Warn-only + fail-safe means
  a wrong schema assumption degrades to "no nudge," never a broken checkpoint.

## Estimated Scope

Single PR — skill-prose edits plus a small hook script (`check_question_context.py`),
its `.claude/settings.json` wiring, a smoke test, and the AGENTS.md table row.
