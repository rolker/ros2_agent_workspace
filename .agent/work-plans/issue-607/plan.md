# Plan: Flip dispatch-mode default from container to in-process (auto mode retires #545's premise)

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/607

## Context

`.claude/skills/run-issue/SKILL.md` and `.agent/knowledge/skill_workflows.md`
currently tell agents to **lean toward `container`** dispatch, on the stated
grounds that it eliminates permission prompts (traced to #545). Claude Code's
**auto mode** now auto-approves the routine tool calls that made in-process
dispatch prompt-heavy, and in-process `Agent`-tool sub-agents inherit the
session's permission mode — so the fan-out case the knowledge file singles
out goes quiet too, when auto mode is active.

The Issue Review entry (`.agent/work-plans/issue-607/progress.md`, commit
`70be42f`) verified the issue's quotations and container-limitation claims
against source and found the issue well-scoped, with one substantive content
gap: the replacement text must not state "in-process is the default"
unconditionally, because that would reintroduce the prompt-flood problem
#545 solved for sessions *not* running Claude Code auto mode (other
permission modes, and non-Claude host runtimes, which have no `Agent` tool
at all and are already called out separately in the file).

**Note on the #606 citation**: the issue's "Root-owned volumes fix" section
cites "until #604 / #606". The Issue Review flagged #606 as absent from git
history and treated it as a citation error. That flag is superseded: #606 is
an open PR (fixing #604) created by the host after the review ran. Leave
that citation as-is in both target files — do not "correct" it to #604-only.

## Approach

1. **Rewrite the "Choosing a mode" section in `run-issue/SKILL.md`
   (~L92-110).** Replace the "container is the biggest practical reason"
   framing and the "Lean toward `container`" heading with:
   - **in-process is the default** for phases run in this same session,
     *conditioned explicitly*: state that the prompt-elimination advantage of
     container dispatch is only decisive when the session is **not** already
     running under a mode that auto-approves routine tool calls (Claude
     Code's auto mode, referenced the way the surrounding prose already
     references host-runtime capabilities — no new detection mechanism, just
     accurate advisory text). Under auto mode, in-process `Agent`-tool
     sub-agents inherit the auto-approval and run prompt-free too, so the
     prompt-volume argument for containers doesn't apply.
   - For a session **not** in auto mode (default Claude Code permission
     mode, or a non-Claude host runtime without an `Agent` tool), keep the
     existing guidance: lean toward `container` for phases that do many
     tool calls, or expect per-call prompts from an in-process fan-out.
   - Reframe container's remaining case as **isolation or dependency
     environment**, not prompt volume: untrusted input (the sandbox
     boundary is what contains a dispatched agent — keep this paragraph,
     content unchanged per the issue's "Non-goals"/explicit instruction),
     and a clean OS/dependency set for the review-code fan-out or
     implementation.
   - Update the section heading's inline citation from "(#545)" to
     "(#607)" — record **auto mode** as the reason the advice moved, so a
     future reader sees why, not that it drifted.
   - Do **not** touch the `--context-file` limitation paragraphs (~L64-74),
     the in-process/container bullet definitions above the heading
     (~L76-91) except where the "biggest practical reason" prompt-volume
     claim needs the same conditioning, or the background-dispatch /
     freshness-gate paragraphs below (~L112-138) — all orthogonal per the
     issue's non-goals.

2. **Rewrite the "Fan-out goes to containers" bullet in
   `skill_workflows.md` § Dispatch Practices (~L131-133).** Replace the
   prompt-volume framing with one keyed on isolation need, conditioned the
   same way as step 1: under auto mode, in-process fan-out is prompt-free
   and preferred (lower launch cost, same context root); reach for container
   fan-out when the work needs OS-level isolation (untrusted input) or a
   clean dependency environment, or when the session is not running auto
   mode and prompt volume would otherwise be a problem. Keep the three
   neighbouring bullets (background dispatch, no filesystem-wide search
   scope, the exit-137 free-RAM gate) unchanged — orthogonal per the issue's
   scope.

3. **No other files.** Confirmed in the Issue Review (`Consequences`
   section) that no adapter file (`.github/copilot-instructions.md`,
   `.agent/instructions/gemini-cli.instructions.md`) duplicates this advice,
   so no cascade edit is needed. `dispatch_subagent.sh` itself is explicitly
   out of scope per the issue's Non-goals (no `--mode` default change, no
   tooling change) — this is a documentation-only PR.

## Files to Change

| File | Change |
|------|--------|
| `.claude/skills/run-issue/SKILL.md` | Rewrite "Choosing a mode" (~L92-110): in-process default conditioned on auto mode; container reframed to isolation/dependency-environment; citation `(#545)` → `(#607)`. |
| `.agent/knowledge/skill_workflows.md` | Rewrite the "Fan-out goes to containers" bullet (~L131-133) in § Dispatch Practices: same auto-mode-conditioned isolation framing. |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | This is itself a governance-advice change to instruction files, which AGENTS.md marks Ask-First; the operator has approved this specific change in principle (per dispatch instructions). The PR still goes through the normal review lifecycle (review-plan → review-code) before merge. |
| Enforcement over documentation | Advisory prose in both files, same as the original #545 text — not a regression; no new mechanical enforcement is proposed or implied. |
| Capture decisions, not just implementations | The rewrite itself records *why* the advice moved (auto mode) inline in the prose, at the same fidelity #545 was captured (also prose-only, no ADR). Consistent with precedent. |
| A change includes its consequences | Addressed directly: the `(#545)` → `(#607)` citation update in `run-issue/SKILL.md` is step 1's last bullet, closing the Issue Review's flagged gap. |
| Primary framework first, portability where free | This is the core content fix from the Issue Review: the new default is explicitly conditioned on Claude Code auto mode being active, not stated as an unconditional default. Non-auto-mode sessions and non-Claude host runtimes (already called out in the existing text as lacking an `Agent` tool) keep the container-leaning guidance instead of being silently exposed to prompt floods. |
| Only what's needed | Two files, two targeted sections, no touching `dispatch_subagent.sh`, no deprecating containers — matches the issue's Non-goals exactly. |
| Improve incrementally | Single PR, prose-only, scoped edits. |
| Test what breaks | N/A — advisory prose, no enforced logic; nothing to unit test. Verification is a read-through in review-plan / review-code confirming the conditioning language is unambiguous. |
| Workspace vs. project separation | Both target files are workspace infra (`.claude/skills/`, `.agent/knowledge/`) — correct repo, no project-repo crossover. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0001 — Adopt ADRs | Watch, not required | Same as the Issue Review's assessment: this reverses operational guidance in the same way #545 established it, and #545 was never captured as an ADR. Not requiring one now keeps the bar consistent; not blocking. |
| 0003 — Project-agnostic workspace | OK | Both files are generic workspace dispatch tooling. |
| 0015 — Dispatch handoff context contract | Not triggered | This issue doesn't touch the `--context-file` read/write mechanism, only the mode-selection advice around it (confirmed in Issue Review). |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| The "Choosing a mode" heading text and citation in `run-issue/SKILL.md` | Any other place that cites "(#545)" as the mode-choice rationale | Yes — checked: `skill_workflows.md`'s bullet doesn't carry an inline issue citation, so no second citation update is needed there. |
| The "Fan-out goes to containers" rule in `skill_workflows.md` | `run-issue/SKILL.md`'s own container/in-process definitions (~L76-91), which restate the prompt-elimination claim in the bullet describing `container` | Yes — step 1 explicitly conditions that bullet's "biggest practical reason" language too, so the two files stay consistent with each other. |
| Removing "prompt volume" as the primary in-process-vs-container driver | Downstream advice that assumed the old default (e.g. any skill telling agents to reach for `--mode container` "for prompt-free work" without an isolation reason) | No further instances found — `grep` across `.claude/` and `.agent/knowledge/` for "prompt-free" / "permission prompt" also hits `start-deployment/SKILL.md`, `deployment_mode.md`, `triage-reviews/SKILL.md`, and `review-code/SKILL.md`, but those all discuss unrelated prompt sources (deployment-log git commits, `progress_append.sh` usage) — none restate the container-vs-in-process dispatch-mode rationale this issue changes (verified during exploration). |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): `.claude/skills/run-issue/SKILL.md`
  and `.agent/knowledge/skill_workflows.md` themselves — these ARE the
  documentation being corrected; both edits land in this PR (see Files to
  Change).
- **Agent-instruction candidates** (proposals only — operator decides):
  None — this issue's entire content *is* an instruction-file update; there
  is no further candidate to propose beyond what's already in scope.

## Open Questions

- None. The one substantive gap flagged by the Issue Review (conditioning
  the new default on auto mode) is addressed in Approach step 1; the
  citation-update gap is addressed in Approach step 1's last bullet; the
  #606 citation is confirmed correct and left untouched per explicit
  instruction.

## Estimated Scope

Single PR, two files, prose-only edits.


## Implementation notes (plan sync)

Written after `review-plan` returned `changes-requested` (5 must-fix, 3
suggestions) and the operator ruled at the plan checkpoint. What actually
shipped differs from the plan above in four ways, all review-driven:

1. **The do-not-touch fence was redrawn.** The plan fenced off the
   in-process/container bullet definitions (~L76-91). The review showed that
   fence protected three statements the change falsifies — the `for quick /
   cheap phases` scoping label, the unconditional prompt-flood Caveat, and the
   `for isolation *and* prompt-free dispatch` bullet heading (with the "biggest
   practical reason" claim spanning L90-92, partly outside the named range).
   Both bullets were rewritten; the genuinely orthogonal `--context-file` and
   background/freshness paragraphs were left alone.

2. **The auto-mode condition names its tell and fails safe.** The plan
   conditioned the new default on auto mode while declining to say how a reader
   determines which case they are in — but the reader *is* the agent choosing.
   The text now names the observable (the Claude Code session's
   permission-mode indicator) and states the fallback explicitly: *cannot
   confirm auto mode → the container-leaning guidance is in force*. The
   asymmetry is deliberate, since an uncertain reader must not land on the
   prompt-flooding branch #545 existed to prevent.

3. **The safety reasoning was rewritten for both paths** (operator decision at
   the plan checkpoint). The plan said keep the sandbox-boundary paragraph
   "content unchanged", but that paragraph was a *caution against* reaching for
   container — incoherent once the surrounding advice says to choose container
   for untrusted input. It is now affirmative, every clause of the reasoning
   preserved, and it covers the mirror case the plan missed entirely: what
   contains an in-process phase under auto mode (host permission policy and
   allowlist, worktree confinement, and the checkpoints that survive regardless
   of mode). Auto mode removed the prompts, not the need to know what is
   holding.

4. **The claim is grounded in observation, not asserted mechanism.** Rather
   than stating that in-process `Agent` sub-agents inherit the session
   permission mode — a claim about Claude Code internals this workspace cannot
   verify from source — the text cites the observed #604 lifecycle: nine typed
   `progress.md` entries (`review-issue`, `plan-task`, `review-plan`, two
   `review-code` rounds, `triage-reviews`, three `address-findings` passes) plus
   an unentried implementation pass, all in-process under auto mode with no
   approvals for the dispatched work.

Scope additions:

- **Third stale surface** (review finding 1): the `# Container (isolation; use
  for implementation-heavy phases):` code-block comment in
  `skill_workflows.md` ~L102-107 — the retired advice in miniature, ~25 lines
  above the bullet the plan did rewrite, and invisible to the plan's
  consequences grep because it contains neither "prompt-free" nor "permission
  prompt". The widened search (`implementation-heavy`, bare `--mode container`,
  "lean toward", "prefer container") found no further surfaces;
  `review-plan/SKILL.md:435` already reads "isolation-worthy" and stays.
- **`skill_workflows.md` now carries the #545 → #607 citation** so a reader
  arriving at the knowledge file first learns the rule was reversed
  deliberately (review suggestion 7 / issue proposal item 3).
- **`.claude/skills/review-plan/SKILL.md` self-review heuristic** (operator
  decision to fold in): it compared `$AGENT_NAME` against the plan author, but
  every dispatched agent shares one `$AGENT_NAME`, so it matched on every
  review and destroyed the signal it carries. Now keyed on whether the plan was
  authored in the same context, with the shared-identity trap stated so it is
  not reintroduced.
