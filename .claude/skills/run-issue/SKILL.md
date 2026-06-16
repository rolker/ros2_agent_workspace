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

- **in-process** (default) for reviews and short phases — fast, same context
  root. In-process dispatch spawns the phase via Claude Code's **`Agent` tool**,
  which exists in the **host** session but **not** inside a headless container.
  This is precisely why `run-issue` is a *host* orchestrator (see Scope E): run
  it on the host, where it can fan phases out in-process; it is never itself
  dispatched into a container. (Non-Claude host runtimes lack the `Agent` tool
  too — there, drive the phases manually or use `--mode container`.)
- **container** when isolation is wanted (e.g. running the full review-code
  specialist fan-out, or implementation that benefits from a clean OS). Container
  mode is headless and self-contained, so it needs no host `Agent` tool.

The dispatcher already verifies the sub-agent wrote the expected entry type
(PRE/POST entry-count delta — `dispatch_subagent.sh:216-282`); treat a `FAILED`
report as a stop-and-surface, not a silent retry. After each dispatch, read the
timeline and act on the **last** entry — that is the one the phase just wrote:

```bash
# Whole timeline; `.entries[-1]` is the just-written entry, whatever its type.
python3 .agent/scripts/progress_read.py .agent/work-plans/issue-<N>/progress.md
```

To pull a *specific* phase's entry later (e.g. an Integrated Review's findings),
filter with `--type`, **passing the full canonical entry type verbatim** — not
an abbreviation:

```bash
python3 .agent/scripts/progress_read.py .agent/work-plans/issue-<N>/progress.md \
    --type "Local Review (Pre-Push)"
```

**`Local Review (Pre-Push)` is its own canonical type, distinct from `Local
Review`.** The parenthetical is part of the name (`progress_read.py`
`CANONICAL_TYPES`), and `--type` matches on the full heading / `base_type`
*exactly* — so `--type "Local Review"` will **not** match the `(Pre-Push)`
variant. (The dispatcher's PRE/POST gate, by contrast, adds a `startswith`
prefix match in its own one-liner — `dispatch_subagent.sh:225-235` — which is
why a pre-push dispatch resolved as `Local Review` still counts the
`(Pre-Push)` entry. The `--type` filter does not have that fallback; pass the
real type.) Pre-push `review-code` always writes `## Local Review (Pre-Push)`;
`skill_workflows.md`'s handoff table abbreviates it `## Local Review`, but the
orchestrator routes on the real heading. When you need either Local Review
variant, pass both (`--type` is repeatable).

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

**`## Local Review (Pre-Push)` is the exact heading** every pre-push
`review-code` writes, and **the orchestrator only ever dispatches `review-code`
in pre-push mode** — so within this flow that is the only Local-Review heading
produced, and the table keys on it verbatim. `## Local Review` (no parenthetical)
is review-code's real **post-PR** heading (`review-code/SKILL.md:864-866, 872`),
*not* a mere abbreviation — but the orchestrator never drives post-PR
`review-code`: post-PR review feedback is consumed by `triage-reviews`, which
writes `## Integrated Review`. (`skill_workflows.md`'s handoff table does
abbreviate the pre-push entry as `## Local Review`; that is a doc shorthand, not
the routed heading.) **Fallback**: if a bare `## Local Review` (post-PR) is
nonetheless the last entry — e.g. someone ran `/review-code <PR>` by hand —
route it like the `## Integrated Review` rows (open findings ⇒ `address-findings`;
none ⇒ merge checkpoint).

**Shared `## Implementation` routing key**: `## Implementation` is written by
both `address-findings` and a future `implement` skill. Disambiguate by the
**preceding** entry — a `## Integrated Review` immediately before means this was
an address-findings pass, so the next action is a re-review.

**Implementation phase**: there is no `implement` skill yet. After
`## Plan Review`, the host runs implementation **inline** (edits, commits,
plan-sync), then dispatches `review-code`. When an `implement` skill lands,
swap the inline step for a dispatch — **and add a table row** for a bare
`## Implementation` *not* preceded by `## Integrated Review` (a fresh
implementation pass ⇒ dispatch `review-code`). That state is unreachable today
(inline implementation writes no such entry before the host itself runs
`review-code`), so the table has no row for it yet.

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

**Idempotency guard (publish writes no progress.md entry).** The publish step
does **not** append an entry, so after a successful publish the last entry is
*still* `## Local Review (Pre-Push)` (`approved`) — the same state that triggered
publish. A naive re-run would double-publish. Before publishing, check for an
already-open real PR on the branch and, if found, treat the work as published —
skip to the `triage-reviews` wait instead of pushing/opening again:

```bash
# A non-[PLAN] open PR on this branch ⇒ already published.
gh pr list --head "$(git branch --show-current)" --state open \
    --json url,title --jq '.[] | select(.title | startswith("[PLAN]") | not) | .url'
```

- **GitHub-origin (dev mode)**: `git push`, then `gh pr create` (drop any
  `[PLAN]` framing — this is the real PR). After the PR accrues review comments,
  resume at the `triage-reviews` row.
  - **Copilot opt-in scope**: `--copilot` is a **`review-code` flag** governing
    its Copilot Adversarial specialist — it is *not* consumed by `git push` or
    `gh pr create`. **Off by default** (the standing quota decision — see
    review-code). The publish checkpoint surfaces it as a per-run choice because
    that is where the user decides whether to spend Premium quota on cross-model
    coverage for this PR; if opted in, pass `--copilot` to the `review-code`
    re-runs (pre-push and any post-triage re-review), not to the publish
    commands.
- **Field mode (gitcloud / non-GitHub origin)**: push to the field remote with
  **no PR and no Copilot** (the field workflow — see AGENTS.md § Field Mode).
  The lifecycle ends at the push; reconciliation to GitHub is a later dev-side
  `/import-field-changes`. **Hook caveat**: if the field repo lists its default
  branch in a `no-commit-to-branch` pre-commit hook, commits fail there — fix the
  project's hook config (drop its default branch from the list), **never
  `--no-verify`** (AGENTS.md § Field Mode).

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
