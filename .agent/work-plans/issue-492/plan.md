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
4. **Local-first PR-at-end** (decided): **flip `plan-task`'s default** — step 7
   no longer opens an early `[PLAN]` draft PR; the plan is committed to the
   branch and "publish early" becomes an **opt-in flag** (e.g. `--draft-pr`).
   The orchestrator pushes + opens the PR at the end. This ships **in this PR**.
   Downstream `review-plan` already accepts a plan **file / `--issue <N>`** (not
   only a PR), so it still works with no early PR — verify and adjust its default
   resolution to prefer the local plan file when no PR exists.
5. **Field mode** (ADR-0011): at the publish step, branch on `field_mode.sh` —
   gitcloud-origin repos push to the field remote with **no PR and no Copilot**.
6. **Consequences**: adapter skill lists + `skill_workflows.md`.

## Files to Change
| File | Change |
|------|--------|
| `.claude/skills/run-issue/SKILL.md` | **New** — orchestrator (decision table + checkpoints + publish step incl. field-mode + opt-in Copilot) |
| `.claude/skills/plan-task/SKILL.md` | **Flip default**: step 7 no longer opens a draft PR; add `--draft-pr` opt-in; update the "Next step" + "During implementation" prose that assumes an early PR |
| `.claude/skills/review-plan/SKILL.md` | Confirm/adjust no-PR resolution: prefer the local plan file via `--issue <N>` when no PR exists |
| `.agent/knowledge/skill_workflows.md` | Document `/run-issue` as the lifecycle driver; note plan-task no longer auto-publishes |
| `AGENTS.md` | Post-Task-Verification "Plan-first workflow" wording mentions "if the PR opened with a work plan" — already conditional, but verify it reads correctly now that no early PR is the default (**instruction file — confirm with user before edit**) |
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
_Resolved with the user (2026-06-15):_
- [x] **plan-task PR behavior** → **flip the default to no-early-PR**; publish-early becomes an opt-in flag. Ships **in this PR** (not a stacked follow-up).
- [x] **Copilot at publish** → **off by default**; the publish checkpoint offers `--copilot` as a per-run opt-in.
- [ ] **AGENTS.md edit** — the "Plan-first workflow" wording may need a light tweak; AGENTS.md is an "ask first" instruction file → confirm the exact change with the user during implementation before editing.

## Estimated Scope
Single PR: orchestrator + plan-task default flip (+ `--draft-pr` opt-in) +
review-plan no-PR resolution + skill_workflows + adapters. Larger than a typical
new-skill PR because of the plan-task default flip and its ripple.
