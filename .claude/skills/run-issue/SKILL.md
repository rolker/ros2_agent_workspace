---
name: run-issue
description: Host orchestrator that drives an issue through the full per-issue lifecycle — dispatches each phase (review-issue → plan-task → review-plan → implement → review-code → triage-reviews → address-findings) as a fresh-context sub-agent, reads each phase's progress.md entry to choose the next action, and pauses at user checkpoints. Local-first: the PR is created at the end, never auto-pushes or auto-merges without confirmation.
---

# Run Issue

## Usage

```
/run-issue <issue-number>
```

## Overview

`run-issue` is the **host orchestrator** for the composable-timeline lifecycle
(#470 / #481 phase C). It does not do review or implementation work itself —
it *drives*: dispatch a phase via `dispatch_subagent.sh`, read the typed
`progress.md` entry that phase wrote, decide the next action from the decision
table below, and pause at `AskUserQuestion` checkpoints at every juncture where
a human should weigh in or where work would otherwise publish.

It is the automation of the hand-driven lifecycle — the same sequence an
operator runs by issuing `/review-issue`, `/plan-task`, … one at a time.

**This skill is the driver, not a dispatched phase.** It is never the target of
`dispatch_subagent.sh` and has no `skill_entry_type` row; it *calls* the
dispatcher for the other skills.

**Principles this skill exists to honor:**
- **Human control** — checkpoints gate every push, PR creation, and merge.
  Nothing leaves the local machine without an explicit confirmation.
- **Transparency** — the decision table is explicit; each step says what entry
  it read and what it will dispatch next.
- **Local-first** — the PR is created **at the end**, after a clean local
  review (not early). `plan-task` no longer opens an early PR by default.

## How phases are dispatched

Each phase runs in a **fresh-context sub-agent** via the dispatcher:

```bash
.agent/scripts/dispatch_subagent.sh --mode <in-process|container> --issue <N> --skill <phase>
```

- **in-process** (default) for reviews and short phases — fast, same context root.
- **container** when isolation is wanted (e.g. running the full review-code
  specialist fan-out, or implementation that benefits from a clean OS).

The dispatcher already verifies the sub-agent wrote the expected entry type
(PRE/POST entry-count delta — `dispatch_subagent.sh:216-282`); treat a `FAILED`
report as a stop-and-surface, not a silent retry. After each dispatch, read the
**newest** matching entry:

```bash
python3 .agent/scripts/progress_read.py .agent/work-plans/issue-<N>/progress.md --type "<Entry Type>"
```

**Entry-type matching is by base type / prefix, not exact string.** `review-code`
pre-push writes `## Local Review (Pre-Push)`; `skill_workflows.md`'s handoff
table abbreviates it `## Local Review`. Match the same way the dispatcher does
(`dispatch_subagent.sh:217-235`) so the variant doesn't slip past.

## Decision table (the spine)

Read the **last** progress.md entry; act per its type + verdict:

| Last entry | Condition | Next action | Checkpoint |
|---|---|---|---|
| (none) | — | dispatch `review-issue` | — |
| `## Issue Review` | open-question actions present | surface the open Qs, then `plan-task` | **yes** (iff open Qs) |
| `## Issue Review` | none | dispatch `plan-task` | — |
| `## Plan Authored` | — | dispatch `review-plan` | — |
| `## Plan Review` | any verdict | run implementation, then `review-code` | **always** |
| `## Local Review (Pre-Push)` | `approved` | push → open PR (or field-push) | **yes** (before publish) |
| `## Local Review (Pre-Push)` | `changes-requested` | address inline, re-dispatch `review-code` | maybe |
| (PR published) | review comments landed | dispatch `triage-reviews` | **yes** (async wait) |
| `## Integrated Review` | open findings remain | dispatch `address-findings` | **yes** (non-trivial findings) |
| `## Integrated Review` | none | merge | **yes** (before merge) |
| `## Implementation` **preceded by** `## Integrated Review` | — | dispatch `review-code` (re-review) | — |

**Shared `## Implementation` routing key**: `## Implementation` is written by
both `address-findings` and a future `implement` skill. Disambiguate by the
**preceding** entry — a `## Integrated Review` immediately before means this was
an address-findings pass, so the next action is a re-review.

**Implementation phase**: there is no `implement` skill yet. After
`## Plan Review`, the host runs implementation **inline** (edits, commits,
plan-sync), then dispatches `review-code`. When an `implement` skill lands,
swap the inline step for a dispatch — the table is unchanged.

## Checkpoints

Use `AskUserQuestion` — **never block silently**. Mandatory checkpoints:

1. **After `## Issue Review`** — only if it left open-question actions; surface
   them and get answers before `plan-task`.
2. **After `## Plan Review`** — always; the plan + its verdict are the last
   cheap place to redirect.
3. **After each `## Integrated Review` with non-trivial findings** — confirm the
   fix approach before dispatching `address-findings`.
4. **Before any push, PR creation, or merge** — local-first: the user confirms
   that work publishes.

Between checkpoints the orchestrator proceeds automatically, reporting each
transition (entry read → next dispatch).

## Publish step (local-first, field-mode aware)

Reached when a `## Local Review (Pre-Push)` is `approved` and the user confirms
at the publish checkpoint. Branch on origin via
[`field_mode.sh`](../../.agent/scripts/field_mode.sh):

- **GitHub-origin (dev mode)**: `git push`, then `gh pr create` (drop any
  `[PLAN]` framing — this is the real PR). **Copilot is off by default**; the
  publish checkpoint offers `--copilot` as a per-run opt-in (the standing quota
  decision — see review-code). After the PR accrues review comments, resume at
  the `triage-reviews` row.
- **Field mode (gitcloud / non-GitHub origin)**: push to the field remote with
  **no PR and no Copilot** (the field workflow — see AGENTS.md § Field Mode).
  The lifecycle ends at the push; reconciliation to GitHub is a later dev-side
  `/import-field-changes`.

Never `git push`, open a PR, or merge without having passed the corresponding
checkpoint.

## No auto-chaining beyond the operator (Scope E)

`run-issue` is the *single* orchestrator. The phase skills never dispatch each
other — they emit a handoff prompt + progress.md entry, and `run-issue` reads
the entry and drives the next step. Sub-agents do not spawn sub-agents. This
keeps user-checkpoint control at one layer.

## Guidelines

- **Read the entry, don't assume the outcome** — always re-read the newest
  progress.md entry after a dispatch; a phase may have failed, deferred, or
  surfaced open questions.
- **Surface, don't swallow** — a `FAILED` dispatch report, an unexpected entry
  type, or a missing entry is a stop-and-ask, never a silent retry.
- **One issue, linear** — single-issue pipeline; concurrent multi-issue
  orchestration is a non-goal (per #481).
- **Stay local until told otherwise** — the default posture is "keep it on this
  machine"; publishing is always a confirmed, explicit step.
