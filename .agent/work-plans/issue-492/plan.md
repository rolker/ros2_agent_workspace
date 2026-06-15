# Plan: /run-issue host orchestration skill (#492)

## Issue
https://github.com/rolker/ros2_agent_workspace/issues/492

## Context
Phase C's keystone. The lifecycle skills (review-issue → plan-task → review-plan
→ implement → review-code → triage-reviews → address-findings) already each
write a typed `progress.md` entry and emit a handoff; `dispatch_subagent.sh`
already dispatches a phase and verifies the sub-agent wrote the expected entry
(PRE/POST entry-count delta → reports the new entry or `FAILED`). What's missing
is the **host driver**: a skill that runs the phases in order, reads each
entry's verdict/findings/open-questions to pick the next action, and pauses at
user checkpoints. This session's hand-driven #491 run *is* the manual version of
exactly this.

## Approach
A new `.claude/skills/run-issue/SKILL.md` — host-agent orchestration logic, no
new script (reuses `dispatch_subagent.sh` + `progress_read.py`). Its spine is a
**phase-transition decision table**: given the last `progress.md` entry
(type + verdict), pick the next dispatch and whether to checkpoint.

1. **Decision table** (the core):

   | Last entry | Condition | Next action | Checkpoint |
   |---|---|---|---|
   | (none) | — | dispatch `review-issue` | — |
   | `## Issue Review` | open-question actions | ask the open Qs, then `plan-task` | **yes** (iff open Qs) |
   | `## Issue Review` | none | dispatch `plan-task` | — |
   | `## Plan Authored` | — | dispatch `review-plan` | — |
   | `## Plan Review` | any | run implementation, then `review-code` | **always** |
   | `## Local Review (Pre-Push)` | approved | push → PR (or field-push) | **yes** (before publish) |
   | `## Local Review (Pre-Push)` | changes-requested | address inline, re-run `review-code` | maybe |
   | `## Integrated Review` | open findings | dispatch `address-findings` | **yes** (non-trivial findings) |
   | `## Integrated Review` | none | merge | **yes** (before merge) |
   | `## Implementation` **preceded by** `## Integrated Review` | — | dispatch `review-code` (re-review) | — |

   The last row is the **shared-routing-key disambiguation**: `## Implementation`
   is written by both `address-findings` and a future `implement` skill, so route
   on the *preceding* entry (Integrated Review before ⇒ address-findings pass ⇒
   re-review).
2. **Checkpoints** via `AskUserQuestion`, never silent: after `## Issue Review`
   iff open questions; **always** after `## Plan Review`; after each
   `## Integrated Review` with non-trivial findings; and **before any
   push / PR creation / merge** (local-first).
3. **Implementation phase**: there is no `implement` skill yet — the host runs
   implementation inline (then dispatches `review-code`). Documented as such, not
   stubbed.
4. **Local-first PR-at-end**: orchestrated `plan-task` must **not** open its
   early `[PLAN]` draft PR (step 7); the orchestrator pushes + opens the PR at
   the end. Mechanism = open question 1.
5. **Field mode** (ADR-0011): at the publish step, branch on `field_mode.sh` —
   gitcloud-origin repos push to the field remote with **no PR and no Copilot**.
6. **Consequences**: adapter skill lists + `skill_workflows.md`.

## Files to Change
| File | Change |
|------|--------|
| `.claude/skills/run-issue/SKILL.md` | **New** — orchestrator (decision table + checkpoints + publish step) |
| `.claude/skills/plan-task/SKILL.md` | Make step-7 early draft PR suppressible when orchestrated (open question 1) |
| `.agent/knowledge/skill_workflows.md` | Document `/run-issue` as the lifecycle driver |
| `.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md` | Add `run-issue` to skill lists |

## Principles Self-Check
| Principle | Consideration |
|---|---|
| Human control & transparency | Checkpoints at every publish/irreversible step; never auto-push/merge; the decision table is explicit and inspectable |
| Only what's needed | No new script — reuse dispatch + progress_read; don't orchestrate phases that don't exist (implement stays inline) |
| Capture decisions | Shared-routing-key disambiguation documented in the table |
| A change includes its consequences | plan-task tweak + adapters + lifecycle doc in scope |

## ADR Compliance
| ADR | Triggered | How addressed |
|---|---|---|
| 0013 — progress.md vocabulary | Yes (consumer) | Routes by entry type; handles shared `## Implementation` via preceding-entry rule |
| 0011 — Field mode | Yes | Publish step branches on `field_mode.sh` (push, no PR/Copilot) |
| 0004/0005, 0006 | Yes | Convention-only skill; adapters updated |

## Consequences
| If we change... | Also update... | In plan? |
|---|---|---|
| Add a workflow skill | adapter skill lists + `skill_workflows.md` + `dispatch_subagent.sh` maps if dispatchable | Yes (run-issue is the driver, not itself dispatched — no skill_entry_type row needed) |
| plan-task PR behavior | plan-task SKILL.md + its "During implementation" notes | Yes (open question 1) |

## Open Questions
- [ ] **plan-task PR-deferral mechanism** — (a) add a suppress signal (env/flag) so *orchestrated* plan-task skips the early PR, standalone default unchanged [recommended, least disruptive]; or (b) flip plan-task's default to no-early-PR with publish-early opt-in (changes standalone behavior). And: ship the plan-task tweak **in this PR** or a stacked follow-up?
- [ ] **Copilot at publish** — default off (per the quota opt-in decision), `--copilot` opt-in surfaced as a checkpoint option? Confirm.

## Estimated Scope
Single PR (orchestrator + small plan-task tweak + adapters), unless we split the plan-task change to a stacked follow-up (open question 1).
