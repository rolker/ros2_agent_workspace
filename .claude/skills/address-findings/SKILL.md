---
name: address-findings
description: Work through the open action items from the latest review entry in progress.md — a ## Integrated Review (post-PR) or a ## Local Review (Pre-Push) (pre-push) — make each fix, commit atomically with pre-commit hooks, check the box, and record a ## Implementation entry. The close-the-loop phase between a review and its re-review.
---

# Address Findings

## Usage

```
/address-findings [--issue <N>]
```

Run from the issue's worktree (or pass `--issue <N>`). Operates on the
**latest review entry** — a `## Integrated Review` (post-PR triage) or a
`## Local Review (Pre-Push)` (pre-push `review-code`) — in that issue's
`.agent/work-plans/issue-<N>/progress.md`.

## Overview

**Lifecycle position**:
- post-PR: triage-reviews → **address-findings** → review-code (re-review)
- pre-push: review-code (changes-requested) → **address-findings** → review-code (re-review)

A review phase produces the *source review entry* — `triage-reviews` a
`## Integrated Review` (post-PR), or pre-push `review-code` a `## Local Review
(Pre-Push)` (changes-requested). Either is a fix plan of `- [ ]` checkbox
actions (must-fix and suggestions), plus plain-bullet false positives that are
*dismissals, not actions*. This skill consumes that entry — it works each open
action, commits the fix atomically, checks the box, and writes one closing
`## Implementation` entry so the timeline shows what was addressed.

It is deliberately **thin**: it does not re-classify findings (that was the
review's job — `triage-reviews` post-PR, `review-code` pre-push) and does not
re-review its own work (that is the next `review-code` pass). It only *acts on
an agreed fix plan*.

**Entry type** — writes `## Implementation` (an existing ADR-0013 type;
no new type is minted). The orchestrator (`/run-issue`, #492) reads the
last entry: `## Implementation` → dispatch a re-review.

> **Note for #492:** `## Implementation` is shared — both this skill and a
> future `implement` skill write it — so the orchestrator can't route on
> entry type alone (a fresh implement pass and a post-triage fix pass both
> land as `## Implementation`). It should disambiguate by what *precedes*
> the entry (a `## Integrated Review` **or `## Local Review (Pre-Push)`**
> immediately before ⇒ this was an address-findings pass ⇒ next is re-review).
> Flagged here for #492's design.

## Steps

### 1. Locate the issue and its progress.md

Resolve `<N>` from `--issue` or `$WORKTREE_ISSUE`. The file is
`.agent/work-plans/issue-<N>/progress.md` in the issue's worktree. If it
doesn't exist, stop with an error — there's no triage to address.

### 2. Read the latest review entry

This skill addresses whichever review came last — a `## Integrated Review`
(post-PR, from `triage-reviews`) **or** a `## Local Review (Pre-Push)`
(pre-push, from `review-code`). Use the shared parser rather than re-parsing
markdown (`--type` is repeatable):

```bash
python3 .agent/scripts/progress_read.py \
    .agent/work-plans/issue-<N>/progress.md \
    --type "Integrated Review" --type "Local Review (Pre-Push)"
```

`progress_read.py` emits JSON. Consider only entries whose `base_type` is
**`Integrated Review`** or **`Local Review (Pre-Push)`** — *not* the raw
`--type` match, which also surfaces legacy `## External Review` (Integrated
Review's recognized predecessor) and the post-PR `## Local Review`, neither of
which this skill acts on. Take the **single latest** such entry — the *source
review entry* — and act only on it; never merge findings across entries or fall
back to an older one (so a stale pre-push review is ignored once a later
Integrated Review exists, and vice-versa). Then:

- **No qualifying entry at all** (the file exists but no `Integrated Review` /
  `Local Review (Pre-Push)` is present — no review has run yet): report "no
  review entry to address — run `review-code` (pre-push) or `triage-reviews`
  (post-PR) first" and exit without a commit. Do **not** fall back to other
  entry types.
- Otherwise take the source review entry's `findings[]` array. Each finding is
  `{section, checked, source_hint, text}`. The action items are the entries with
  `checked == false`. (False positives are plain bullets, so they never appear
  in `findings[]` — no special handling needed.)
- **No unchecked findings** in the source review entry: report "nothing to
  address — the latest review has no open actions" and exit without a commit.

### 3. Address each open finding

For each unchecked finding, in listed order (cross-confirmed first):

1. Make the code/doc change the finding calls for. Read the cited
   `file:line` and the surrounding code first — verify the finding
   against the current source before acting (a finding can be stale if
   an earlier round already touched the area).
2. If, on inspection, the finding is **not** actionable (already fixed,
   or a genuine false negative on re-read), do **not** fake a change.
   Record it as a **deferred** item: check its box in the source review
   entry with a `(deferred: <reason>)` annotation and list it in
   the `## Implementation` deferred actions (step 5). Checking it (rather
   than leaving it open) is deliberate: a finding's checkbox should reflect
   whether it still needs attention, and a deferred one does not. It also
   keeps the skill idempotent — if `address-findings` is ever re-run against
   the same source review entry, it acts only on `checked == false`
   items and so won't re-attempt a deferral. "Checked" here means
   *consciously handled*, not *code changed*; the `(deferred: …)` annotation
   records which.
3. From the issue worktree, stage the files for this finding and commit
   atomically with pre-commit hooks and per-invocation identity (AGENTS.md
   § Agent Commit Identity):

   ```bash
   git -c user.name="$AGENT_NAME" -c user.email="$AGENT_EMAIL" \
       commit -m "<area>: <what was fixed> (#<N>)"
   ```

   Never `--no-verify`. One logical fix per commit. The source review
   entry's box-check (next step) is the one permitted addition to a
   finding's commit beyond the finding's own files.
4. Check the box for that finding in the source review entry
   (`- [ ]` → `- [x]`) so the timeline tracks resolution — in the same
   commit as the fix, or a trailing progress commit.

### 4. Re-run hooks / quick local checks

After the last fix, run the relevant quick checks (the linters
pre-commit already ran, plus any package test touched by the changes).
This is not a full `review-code` — it's a sanity pass before handing to
the re-review.

### 5. Write the `## Implementation` entry

Append one entry (per ADR-0013; reuse `## Implementation`). ADR-0013 lists
`## Implementation` among the PR/Branch-correlated types, so the entry **must**
carry a `**Branch**:`/`**PR**:` line — without it `progress_read.py` parses the
entry's `correlation` as `null`:

```markdown

## Implementation
**Status**: complete
**When**: <YYYY-MM-DD HH:MM ±HH:MM>
**By**: <agent name> (<model>)

**Branch**: <branch-name> at `<short-sha>`   <!-- or **PR**: #<N> at `<sha>` if a PR exists -->
**Addressed**: <the source review entry's type + When/SHA it consumed>
**Commits**: <short-shas of the fix commits>

### Actions
- [x] <finding addressed> — `file:line`
- [x] <finding consciously deferred — checked, so it reads as handled> — `file:line` (deferred: <reason>)
```

Deferred / not-actionable findings are written **checked** with a `(deferred:
…)` annotation — checked means "consciously handled," so the checkbox reflects
that the finding no longer needs attention (and a re-run against the same entry,
which acts only on `checked == false` items, won't re-attempt it). Never leave a
deliberately-skipped finding unchecked.

Commit (do **not** push — when dispatched as a sub-agent the host performs
pushes; AGENTS.md § Agent Commit Identity). Run from the issue worktree:

```bash
git add .agent/work-plans/issue-<N>/progress.md
git -c user.name="$AGENT_NAME" -c user.email="$AGENT_EMAIL" \
    commit -m "progress: addressed review findings for #<N>"
```

### Next step

Lifecycle: **Implementation** → **review-code** (re-review the fixes)

Hand off to a **fresh-context sub-agent** — independence is what makes the
re-review trustworthy. Use the dispatcher:

    .agent/scripts/dispatch_subagent.sh --mode in-process --issue <N> --skill review-code

The re-review reads the diff cold and confirms the findings are genuinely
resolved. **No auto-chaining (Scope E):** this skill never dispatches the next
phase itself — the host orchestrator (`/run-issue`, #492) drives, pausing at
user checkpoints. This step only emits this prompt and its `progress.md` entry.

## Guidelines

- **Act on the agreed plan, don't re-litigate it** — the review
  (`triage-reviews` post-PR, or `review-code` pre-push) already classified each
  finding. If you disagree with a finding on inspection, defer it with a reason;
  don't silently drop it or argue it in the commit.
- **Never fake resolution** — a checked box on an *actioned* finding must
  correspond to a real commit. A finding you consciously **defer** is also
  checked, but annotated `(deferred: <reason>)` so it reads as "handled, not
  changed" (see Step 3.2 / Step 5). What you must never do is leave a finding
  *silently* untouched — every finding ends either fixed-and-checked or
  deferred-checked-with-reason.
- **Atomic commits** — one finding per commit where practical, so a re-review
  (and a revert) can reason about each fix independently.
- **Stay thin** — no re-classification, no self-review. The next `review-code`
  pass is the quality gate on this work.
