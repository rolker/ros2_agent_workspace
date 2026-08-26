# Plan: Flip dispatch-mode default from container to in-process (auto mode retires #545's premise)

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/607

## Context

`.claude/skills/run-issue/SKILL.md` and `.agent/knowledge/skill_workflows.md`
currently tell agents to **lean toward `container`** dispatch, on the stated
grounds that it eliminates permission prompts (traced to #545). Claude Code's
**auto mode** now auto-approves the routine tool calls that made in-process
dispatch prompt-heavy — observed across the whole #604 lifecycle, which ran
in-process under auto mode with no operator approvals for the dispatched
work, `triage-reviews` fan-out included. (The shipped text cites that
observation rather than asserting that in-process `Agent` sub-agents inherit
the session's permission mode: that is a claim about Claude Code internals
this workspace cannot verify from source.)

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
history and treated it as a citation error. That flag was superseded: #606 was
the PR fixing #604, created by the host after the review ran, and it has since
merged as `a00193c`. Leave that citation as-is in both target files — do not
"correct" it to #604-only.

## Approach

1. **Rewrite the "Choosing a mode" section in `run-issue/SKILL.md`
   (~L92-110).** Replace the "container is the biggest practical reason"
   framing and the "Lean toward `container`" heading with:
   - **in-process is the default** for phases run in this same session,
     *conditioned explicitly*: state that the prompt-elimination advantage of
     container dispatch is only decisive when the session is **not** already
     running under a mode that auto-approves routine tool calls (Claude
     Code's auto mode). Under auto mode, in-process `Agent`-tool sub-agents
     run prompt-free too, so the prompt-volume argument for containers
     doesn't apply. *(The plan said "no new detection mechanism"; the shipped
     text does name the tell the deciding agent reads — see Implementation Notes
     — because
     an unstated condition is one no reader can evaluate.)*
   - For a session **not** in auto mode (default Claude Code permission
     mode, or a non-Claude host runtime without an `Agent` tool), keep the
     existing guidance: lean toward `container` for phases that do many
     tool calls, or expect per-call prompts from an in-process fan-out.
   - Reframe container's remaining case as **isolation or dependency
     environment**, not prompt volume: untrusted input, and a clean
     OS/dependency set for implementation.
     *(Superseded during implementation — see Implementation Notes: the
     sandbox-boundary paragraph was rewritten, not kept "content unchanged",
     and now covers both dispatch paths. The `review-code` fan-out was also
     removed from the container list — it wants the `Agent` tool and the host
     Ollama endpoint, neither of which the sandbox has, so a container run of it
     is **degraded** (specialists evaluate sequentially; specialist 5f cannot run
     at all) rather than impossible. The landed guidance therefore prefers
     in-process wherever an `Agent` tool is available, with the degraded
     container run as the fallback on runtimes that have none.)*
   - Update the section heading's inline citation from "(#545)" to
     "(#607)" — record **auto mode** as the reason the advice moved, so a
     future reader sees why, not that it drifted.
   - Do **not** touch the `--context-file` limitation paragraphs or the
     background-dispatch / freshness-gate paragraphs below the section — both
     orthogonal per the issue's non-goals, and both left alone.
     *(Fence redrawn during implementation — see Implementation Notes: the
     in-process/container bullet definitions were originally fenced off too,
     but they carried three statements this change falsifies, so both bullets
     were rewritten.)*

2. **Rewrite the "Fan-out goes to containers" bullet in
   `skill_workflows.md` § Dispatch Practices (~L131-133).** Replace the
   prompt-volume framing with one keyed on isolation need, conditioned the
   same way as step 1: under auto mode, in-process fan-out is prompt-free
   and preferred (lower launch cost, same context root); reach for container
   fan-out when the work needs OS-level isolation (untrusted input) or a
   clean dependency environment, or when auto mode cannot be confirmed and
   prompt volume would otherwise be a problem. Keep the three neighbouring
   bullets (background dispatch, no filesystem-wide search scope, the
   exit-137 free-RAM gate) unchanged — orthogonal per the issue's scope.
   *(Two additions beyond the named bullet, both landed: the `# Container (…)`
   code-block comment ~25 lines above it — the same retired advice in
   miniature, invisible to the plan's consequences grep because it contains
   neither "prompt-free" nor "permission prompt" — and the file's own
   `#545` → `#607` citation, so a reader arriving here first sees the rule was
   reversed deliberately. A widened search — `implementation-heavy`, bare
   `--mode container`, "lean toward", "prefer container" — found no further
   surfaces.)*

3. **Third file: `.claude/skills/review-plan/SKILL.md`.** The plan originally
   said "no other files"; two additions landed there, both operator-approved:
   the `--mode container` recommendation for implementation work (L~451) now
   points at the new default, and the self-review-detection heuristic was
   fixed to key on *how the reviewer was invoked* rather than on the shared
   `**By**` identity string (rationale in Implementation Notes).

   Still out of scope and untouched: the adapter files
   (`.github/copilot-instructions.md`,
   `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md`,
   `CLAUDE.md`) carry no mode-choice rationale — verified, no cascade edit
   needed. `dispatch_subagent.sh` itself stays out per the issue's Non-goals
   (no `--mode` default change, no tooling change) — this remains a
   documentation-only PR.

## Files to Change

| File | Change |
|------|--------|
| `.claude/skills/run-issue/SKILL.md` | Rewrite "Choosing a mode": in-process default conditioned on auto mode (tell = the injected `While auto mode is active:` system reminder, which the agent can actually read); container reframed to isolation/dependency-environment; the in-process/container bullet definitions above it re-conditioned; the containment paragraph rewritten to cover both dispatch paths accurately; citation `(#545)` → `(#607)`. |
| `.agent/knowledge/skill_workflows.md` | Rewrite the "Fan-out goes to containers" bullet in § Dispatch Practices with the same auto-mode-conditioned isolation framing, plus the `# Container (…)` code-block comment ~25 lines above it; carries its own `#545` → `#607` citation. |
| `.claude/skills/review-plan/SKILL.md` | Added during implementation (operator-approved): align the `--mode container` recommendation for implementation work with the new default, and fix the self-review-detection heuristic to key on invocation rather than the shared `**By**` identity string. Extended in the round-5 fix pass: the Next-step block's container reason is now the clean OS/dependency environment, with untrusted input pointed at the data fence. |
| `.devcontainer/agent/README.md` | Added during the round-4 fix pass: this PR's containment paragraph cites the README as the source of the "no GitHub credentials enter the container" framing, so the README had to stop asserting it. Opening and § Security Model now state the real boundary (forwarded host Claude credentials, unvalidated `GH_TOKEN` scopes, workspace bind-mounted rw); Prerequisites and the "Container won't start" checklist corrected to the three auth sources the launcher actually accepts, and (round 5) the auth check made a non-printing presence test. |
| `.claude/skills/review-code/SKILL.md` | Added during the round-2 fix pass: the convergence guidance costed each review round as "a container cycle", an assumption this issue retires. One-line correction. |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | This is itself a governance-advice change to instruction files, which AGENTS.md marks Ask-First; the operator has approved this specific change in principle (per dispatch instructions). The PR still goes through the normal review lifecycle (review-plan → review-code) before merge. |
| Enforcement over documentation | Advisory prose in both files, same as the original #545 text — not a regression; no new mechanical enforcement is proposed or implied. |
| Capture decisions, not just implementations | The rewrite itself records *why* the advice moved (auto mode) inline in the prose, at the same fidelity #545 was captured (also prose-only, no ADR). Consistent with precedent. |
| A change includes its consequences | Addressed directly: the `(#545)` → `(#607)` citation update in `run-issue/SKILL.md` is step 1's last bullet, closing the Issue Review's flagged gap. |
| Primary framework first, portability where free | This is the core content fix from the Issue Review: the new default is explicitly conditioned on Claude Code auto mode being active, not stated as an unconditional default. Non-auto-mode sessions and non-Claude host runtimes (already called out in the existing text as lacking an `Agent` tool) keep the container-leaning guidance instead of being silently exposed to prompt floods. |
| Only what's needed | Three files (two target files plus one operator-approved folded-in fix), no touching `dispatch_subagent.sh`, no deprecating containers — matches the issue's Non-goals exactly. |
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
| The "Choosing a mode" heading text and citation in `run-issue/SKILL.md` | Any other place that cites "(#545)" as the mode-choice rationale | Yes. *(Revised during implementation: the plan had recorded that `skill_workflows.md`'s bullet carried no inline issue citation and so needed none. The landed edit added one — the fail-safe paragraph now closes with `#545 → #607` — so both files carry the rationale trail.)* |
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

Single PR, five files, prose-only edits. Two were planned;
`.claude/skills/review-plan/SKILL.md` was added during implementation by
operator decision (see Approach step 3 and the Implementation Notes), and
`.devcontainer/agent/README.md` and `.claude/skills/review-code/SKILL.md`
were added by review rounds as consequences of the claims this PR corrects
(see Files to Change).


## Implementation Notes

Rationale for the design decisions that are not obvious from the diff (the
*what* is synced inline above; per `plan-task` § During implementation rule 2,
this section carries only the *why*).

- **The plan's do-not-touch fence around the in-process/container bullet
  definitions was redrawn** because that fence protected three statements this
  change falsifies — the "for quick / cheap phases" scoping label, the
  unconditional prompt-flood caveat, and the "for isolation *and* prompt-free
  dispatch" bullet heading. A fence that preserves false text is not a scope
  boundary worth keeping. The genuinely orthogonal `--context-file` and
  background/freshness paragraphs were left alone.

- **The new default names an observable tell** even though the plan said "no new
  detection mechanism", because the reader who applies the condition *is* the
  agent choosing the mode: an unstated condition is one nobody can evaluate. The
  tell is the `While auto mode is active:` `system-reminder` injected into the
  session's own context (once, early — not per turn), not the operator's
  permission-mode indicator, which is terminal UI the agent cannot see. The
  fallback direction is asymmetric on purpose: an uncertain reader must land on
  the container-leaning branch #545 exists to protect, not on the
  prompt-flooding one.

- **The sandbox-boundary paragraph became affirmative and two-sided** (operator
  decision at the plan checkpoint) because as a *caution against* reaching for
  container it was incoherent beside advice that says to choose container for
  untrusted input. Rewriting it forced both halves to be verified against
  source, and the verified answer narrows *both* modes: auto mode stands down
  routine approvals in-process, the tracked settings carry no `deny`, the
  worktree scoping is handoff prose with nothing enforcing it — and on the
  container side, `docker_run_agent.sh` bind-mounts the whole workspace root
  read-write and forwards the host's Claude credentials, so the sandbox's real
  contribution is a clean machine plus the absence of GitHub write auth. Stated
  that plainly, the case for containers on untrusted input rests on where it
  actually holds.

- **`review-plan/SKILL.md` was folded in** (operator decision) rather than split
  out: its self-review heuristic compared `$AGENT_NAME` against the plan author,
  but every dispatched agent shares one `$AGENT_NAME`, so it matched on every
  review and destroyed the signal it carried. Keying it on *how the reviewer was
  invoked* is the same correction this PR makes elsewhere — read your own
  context, not a string.
