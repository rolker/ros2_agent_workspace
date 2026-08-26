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

**Note on the #606 citation**: the *issue's* "Root-owned volumes fix" section
cites "until #604 / #606". The Issue Review flagged #606 as absent from git
history and treated it as a citation error. That flag was superseded: #606 was
the PR fixing #604, created by the host after the review ran, and it has since
merged as `a00193c`. So the citation needs no "correction" to #604-only.
*(Scope correction, round 6: this only ever concerned the issue text. Neither
target file contains `#606` at all — the earlier wording "leave that citation
as-is in both target files" described an edit that had no subject.)*

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
   - Reframe container's remaining case as the **clean OS/dependency
     environment**, not prompt volume.
     *(Corrected during implementation: the plan originally listed "untrusted
     input" here too. That is the claim this PR removes everywhere — a
     container bind-mounts the workspace read-write and forwards the host
     Claude credential, so it is not containment for untrusted input. The
     countermeasure that landed is the **data fence**, held by the phase in
     either mode.)*
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
   - Do **not** touch the background-dispatch / freshness-gate paragraphs
     below the section — orthogonal per the issue's non-goals, and left alone.
     *(Fence redrawn twice during implementation — see Implementation Notes.
     First: the in-process/container bullet definitions were originally fenced
     off too, but they carried three statements this change falsifies, so both
     bullets were rewritten. Second: the `--context-file` paragraphs were also
     fenced off and were **not** left alone — the container-auth sentence was
     re-qualified (`run-issue/SKILL.md:45-49`, commit `42cde4f`) and the
     orthogonality of `--context-file` to `--mode` was stated there in the
     round-6 pass. Same reason: the fence protected text this change
     falsifies.)*

2. **Rewrite the "Fan-out goes to containers" bullet in
   `skill_workflows.md` § Dispatch Practices (~L131-133).** Replace the
   prompt-volume framing with one keyed on isolation need, conditioned the
   same way as step 1: under auto mode, in-process fan-out is prompt-free
   and preferred (lower launch cost, same context root); reach for container
   fan-out when the work needs a clean OS/dependency environment, or when auto
   mode cannot be confirmed and prompt volume would otherwise be a problem.
   *(Corrected during implementation, as in step 1: "OS-level isolation
   (untrusted input)" was dropped — a container is not containment for
   untrusted input, and the landed text names the data fence instead.)*
   Keep the three neighbouring bullets (background dispatch, no filesystem-wide search scope, the
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

   *(Round-6 additions: the self-review-detection section now also names the
   one mechanical marker that does exist — `dispatch_subagent.sh`'s
   "You are a fresh-context sub-agent dispatched for issue #N" handoff header,
   a positive test for independence — and says where the "what left you unsure"
   line goes in the entry.)*

4. **Fourth file: `.claude/skills/review-code/SKILL.md`** (added by the round-2
   fix pass). Its convergence guidance costed each review round as "a container
   cycle" — an assumption this issue retires. One-line correction.

5. **Fifth file: `.devcontainer/agent/README.md`** (added by the round-4 fix
   pass). This PR's containment paragraph cited the README as the source of the
   "no GitHub credentials enter the container" framing, so the README had to
   stop asserting it: the opening, § Security Model and § Read-only GitHub
   Access now state the real boundary, and the Prerequisites / "Container won't
   start" checklists name the three auth sources the launcher actually accepts
   (round 5: as a non-printing presence test; round 6: bounded to non-`--shell`
   launches, which the guard exempts).

6. **Sixth through eighth files — the data fence's actual home** (added by the
   round-6 fix pass, operator decision): `review-issue/SKILL.md`,
   `plan-task/SKILL.md` and `triage-reviews/SKILL.md`. Both target files name
   the data fence as *the* countermeasure for untrusted input in either mode and
   assign that duty to these three phases — but the fence existed in none of
   them, and a dispatched sub-agent loads its own SKILL.md, not the
   orchestrator's. Each now states it at the step that fetches third-party text.
   `triage-reviews`'s `address-findings` hop was also un-hard-coded from
   `--mode in-process` to the mode choice this PR defines.

   Still out of scope and untouched: the adapter files
   (`.github/copilot-instructions.md`,
   `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md`,
   `CLAUDE.md`) carry no mode-choice rationale — verified, no cascade edit
   needed. *(Round 6 refines this: nothing in them is falsified, but the new
   non-Claude-runtime `--mode container` branch is missing from all three
   adapters, and `AGENTS.md:560` still carries the "no-GitHub-auth container
   phase" phrasing this PR spent three rounds qualifying. All four are
   **Ask-First** instruction files, so they are flagged to the operator rather
   than edited here.)* `dispatch_subagent.sh` itself stays out per the issue's Non-goals
   (no `--mode` default change, no tooling change) — this remains a
   documentation-only PR.

## Files to Change

| File | Change |
|------|--------|
| `.claude/skills/run-issue/SKILL.md` | Rewrite "Choosing a mode": in-process default conditioned on auto mode (tell = the injected `While auto mode is active:` system reminder, which the agent can actually read); container reframed to the clean OS/dependency environment; the in-process/container bullet definitions above it re-conditioned; the containment paragraph rewritten to cover both dispatch paths accurately; citation `(#545)` → `(#607)`. Also, outside the section: the `--context-file` paragraphs' container-auth sentence re-qualified and its orthogonality to `--mode` stated (`:45-49`, `:58-67`), and a stale `review-code/SKILL.md` line-range citation refreshed (`:411`). Round 6: both remaining GitHub-write absolutes qualified, the checkpoints' backing stated per mode, the quick phases routed on the cannot-confirm branch, and the fence's home named. |
| `.agent/knowledge/skill_workflows.md` | Rewrite the "Fan-out goes to containers" bullet in § Dispatch Practices with the same auto-mode-conditioned isolation framing, plus the `# Container (…)` code-block comment ~25 lines above it; carries its own `#545` → `#607` citation. Also the `# In-process (…)` code-block comment, rewritten to name the new default. Round 6: the prompt-volume clause reconciled with the cannot-confirm branch, the quick phases routed there, the container-auth fallback's antecedent fixed, and the fence's home named. |
| `.claude/skills/review-plan/SKILL.md` | Added during implementation (operator-approved): align the `--mode container` recommendation for implementation work with the new default, and fix the self-review-detection heuristic to key on invocation rather than the shared `**By**` identity string. Extended in the round-5 fix pass: the Next-step block's container reason is now the clean OS/dependency environment, with untrusted input pointed at the data fence. Round 6: the mechanical dispatch-header marker named as a positive independence test, and the "what left you unsure" line given a destination. |
| `.devcontainer/agent/README.md` | Added during the round-4 fix pass: this PR's containment paragraph cites the README as the source of the "no GitHub credentials enter the container" framing, so the README had to stop asserting it. Opening and § Security Model now state the real boundary (forwarded host Claude credentials, unvalidated `GH_TOKEN` scopes, workspace bind-mounted rw); Prerequisites and the "Container won't start" checklist corrected to the three auth sources the launcher actually accepts, and (round 5) the auth check made a non-printing presence test. § Read-only GitHub Access was corrected in the same pass. Round 6: the auth guard bounded to non-`--shell` launches. |
| `.claude/skills/review-code/SKILL.md` | Added during the round-2 fix pass: the convergence guidance costed each review round as "a container cycle", an assumption this issue retires. One-line correction. |
| `.claude/skills/review-issue/SKILL.md` | Added in the round-6 fix pass (operator decision): step 1 now states the **data fence** this PR names as the countermeasure for untrusted input, and its `--context-file` note no longer asserts the dispatch has no GitHub read auth. |
| `.claude/skills/plan-task/SKILL.md` | Added in the round-6 fix pass: step 1 states the data fence over the issue body and comments it fetches. |
| `.claude/skills/triage-reviews/SKILL.md` | Added in the round-6 fix pass: step 5 states the data fence over the PR review comments it fetches, and the `address-findings` hop no longer hard-codes `--mode in-process`. |
| `docs/decisions/0019-what-contains-a-dispatched-agent.md` (new) | Records the corrected containment model this PR establishes. |
| `docs/decisions/0004-*`, `docs/decisions/0015-*` | ADR-0012 cross-reference addendums pointing at ADR-0019; neither ADR's existing text is rewritten. |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | This is itself a governance-advice change to instruction files, which AGENTS.md marks Ask-First; the operator has approved this specific change in principle (per dispatch instructions). The PR still goes through the normal review lifecycle (review-plan → review-code) before merge. |
| Enforcement over documentation | Advisory prose in both files, same as the original #545 text — not a regression; no new mechanical enforcement is proposed or implied. |
| Capture decisions, not just implementations | The rewrite itself records *why* the advice moved (auto mode) inline in the prose, at the same fidelity #545 was captured (also prose-only, no ADR). Consistent with precedent. |
| A change includes its consequences | Addressed directly: the `(#545)` → `(#607)` citation update in `run-issue/SKILL.md` is step 1's last bullet, closing the Issue Review's flagged gap. |
| Primary framework first, portability where free | This is the core content fix from the Issue Review: the new default is explicitly conditioned on Claude Code auto mode being active, not stated as an unconditional default. Non-auto-mode sessions and non-Claude host runtimes (already called out in the existing text as lacking an `Agent` tool) keep the container-leaning guidance instead of being silently exposed to prompt floods. |
| Only what's needed | Eight files: the two target files, plus six additions that are each a consequence of a claim this PR corrects (`review-plan`, `review-code`, `.devcontainer/agent/README.md`, and the three skills that now carry the data fence) — every one review-driven or operator-approved and recorded as such above. Plus ADR-0019 and its two addendums. Still no `dispatch_subagent.sh` change, no deprecating containers — matches the issue's Non-goals. |
| Improve incrementally | Single PR, prose-only, scoped edits. |
| Test what breaks | N/A — advisory prose, no enforced logic; nothing to unit test. Verification is a read-through in review-plan / review-code confirming the conditioning language is unambiguous. |
| Workspace vs. project separation | Both target files are workspace infra (`.claude/skills/`, `.agent/knowledge/`) — correct repo, no project-repo crossover. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0001 — Adopt ADRs | **Yes** (revised) | Originally "watch, not required", on parity with #545's prose-only precedent. Revised after round 6: the durable part of this change is not the default flip but the *corrected containment model*, which took six rounds to get right and qualifies two accepted ADRs. **ADR-0019** now records it, by operator decision, in this PR. |
| 0003 — Project-agnostic workspace | OK | Both files are generic workspace dispatch tooling. |
| 0012 — Cross-reference addendums | Yes | Used as intended: ADR-0004 and ADR-0015 receive addendums pointing at the new ADR-0019, with no rewriting of their accepted text. |
| 0015 — Dispatch handoff context contract | **Yes** (revised) | Originally assessed "not triggered" — the issue does not touch the `--context-file` mechanism. But the corrected containment model qualifies ADR-0015's credential-boundary statements (`:30`, `:41`, `:93`, `:103`), so it receives an ADR-0012 addendum pointing at ADR-0019. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| The "Choosing a mode" heading text and citation in `run-issue/SKILL.md` | Any other place that cites "(#545)" as the mode-choice rationale | Yes. *(Revised during implementation: the plan had recorded that `skill_workflows.md`'s bullet carried no inline issue citation and so needed none. The landed edit added one — the fail-safe paragraph now closes with `#545 → #607` — so both files carry the rationale trail.)* |
| The "Fan-out goes to containers" rule in `skill_workflows.md` | `run-issue/SKILL.md`'s own container/in-process definitions (~L76-91), which restate the prompt-elimination claim in the bullet describing `container` | Yes — step 1 explicitly conditions that bullet's "biggest practical reason" language too, so the two files stay consistent with each other. |
| Removing "prompt volume" as the primary in-process-vs-container driver | Downstream advice that assumed the old default (e.g. any skill telling agents to reach for `--mode container` "for prompt-free work" without an isolation reason) | **Three instances found and changed, in later rounds**: `review-code/SKILL.md:878-883` (a review round costed as "a container cycle"), `review-plan/SKILL.md:453-470` (`--mode container` recommended for implementation work), and `triage-reviews/SKILL.md:360` (the `address-findings` hop hard-coded to `--mode in-process`, contradicting this PR's own fail-safe branch). The original "no further instances" claim came from a `grep` for "prompt-free" / "permission prompt" — too narrow to reach downstream advice that names a mode without naming prompts. Beyond those three, three independent sweeps over `.claude/`, `.agent/`, `.devcontainer/`, `.github/`, `docs/`, `AGENTS.md`, `CLAUDE.md`, `Makefile`, `README.md` and `presentations/` found no surviving retired advice. |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): `.claude/skills/run-issue/SKILL.md`
  and `.agent/knowledge/skill_workflows.md` themselves — these ARE the
  documentation being corrected; both edits land in this PR (see Files to
  Change).
- **Stale docs added by review rounds** (all landed in this PR):
  `.claude/skills/review-plan/SKILL.md`, `.claude/skills/review-code/SKILL.md`,
  `.devcontainer/agent/README.md`, and — for the data fence —
  `.claude/skills/review-issue/SKILL.md`, `.claude/skills/plan-task/SKILL.md`,
  `.claude/skills/triage-reviews/SKILL.md`. See Approach steps 3-6.
- **Decision record**: ADR-0019 records the corrected containment model, with
  ADR-0012 cross-reference addendums on ADR-0004 (enforcement hierarchy) and
  ADR-0015 (dispatch handoff contract), both of which state invariants this PR
  qualifies.
- **Agent-instruction candidates** (proposals only — operator decides):
  Four **Ask-First** files are flagged, not edited: `AGENTS.md:560`'s
  "no-GitHub-auth container phase" phrasing, and the missing non-Claude-runtime
  `--mode container` branch in `.github/copilot-instructions.md`,
  `.agent/instructions/gemini-cli.instructions.md` and
  `.agent/AGENT_ONBOARDING.md`.

## Open Questions

- None. The one substantive gap flagged by the Issue Review (conditioning
  the new default on auto mode) is addressed in Approach step 1; the
  citation-update gap is addressed in Approach step 1's last bullet; the
  #606 citation concerns the issue text only — neither target file contains it
  — so nothing was edited there.

## Estimated Scope

Single PR, prose-only edits, plus one new ADR. Eight files carry the prose:
two were planned; `.claude/skills/review-plan/SKILL.md` was added during
implementation by operator decision (see Approach step 3);
`.devcontainer/agent/README.md` and `.claude/skills/review-code/SKILL.md` were
added by review rounds as consequences of the claims this PR corrects; and
`review-issue`, `plan-task` and `triage-reviews` were added in the round-6 fix
pass to give the data fence a home in the phases assigned it. The round-6 pass
also adds **ADR-0019** (what actually contains a dispatched agent) with
ADR-0012 cross-reference addendums on ADR-0004 and ADR-0015 — an operator
decision to record the containment model in this PR rather than defer it.


## Implementation Notes

Rationale for the design decisions that are not obvious from the diff (the
*what* is synced inline above; per `plan-task` § During implementation rule 2,
this section carries only the *why*).

- **The plan's do-not-touch fence around the in-process/container bullet
  definitions was redrawn** because that fence protected three statements this
  change falsifies — the "for quick / cheap phases" scoping label, the
  unconditional prompt-flood caveat, and the "for isolation *and* prompt-free
  dispatch" bullet heading. A fence that preserves false text is not a scope
  boundary worth keeping. The background/freshness paragraphs were genuinely
  orthogonal and were left alone. The `--context-file` paragraphs were not: the
  fence came down there too (rounds 3 and 6), for the same reason — they stated
  a container-auth absolute and left `--context-file`'s orthogonality to
  `--mode` unsaid.

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
  contribution is a clean machine plus the absence of GitHub write auth *as
  configured*. Stated that plainly, the case for containers on untrusted input
  does not hold at all — which is why the landed text points at the data fence
  instead, and why round 6 records the model as **ADR-0019** with ADR-0012
  addendums on ADR-0004 and ADR-0015.

- **`review-plan/SKILL.md` was folded in** (operator decision) rather than split
  out: its self-review heuristic compared `$AGENT_NAME` against the plan author,
  but every dispatched agent shares one `$AGENT_NAME`, so it matched on every
  review and destroyed the signal it carried. Keying it on *how the reviewer was
  invoked* is the same correction this PR makes elsewhere — read your own
  context, not a string.
