---
name: address-findings
description: Work through the open action items from the latest ## Integrated Review entry in progress.md — make each fix, commit atomically with pre-commit hooks, check the box, and record a ## Implementation entry. The close-the-loop phase between triage-reviews and a re-review.
---

# Address Findings

## Usage

```
/address-findings [--issue <N>]
```

Run from the issue's worktree (or pass `--issue <N>`). Operates on the
**latest** `## Integrated Review` entry in that issue's
`.agent/work-plans/issue-<N>/progress.md`.

## Overview

**Lifecycle position**: triage-reviews → **address-findings** → review-code (re-review)

`triage-reviews` produces a `## Integrated Review` entry: a fix plan of
`- [ ]` checkbox actions (must-fix, cross-confirmed, and single-source
findings), plus plain-bullet false positives that are *dismissals, not
actions*. This skill consumes that entry — it works each open action,
commits the fix atomically, checks the box, and writes one closing
`## Implementation` entry so the timeline shows what was addressed.

It is deliberately **thin**: it does not re-classify findings (that was
`triage-reviews`' job) and does not re-review its own work (that is the
next `review-code` pass). It only *acts on an agreed fix plan*.

**Entry type** — writes `## Implementation` (an existing ADR-0013 type;
no new type is minted). The orchestrator (`/run-issue`, #492) reads the
last entry: `## Implementation` → dispatch a re-review.

## Steps

### 1. Locate the issue and its progress.md

Resolve `<N>` from `--issue` or `$WORKTREE_ISSUE`. The file is
`.agent/work-plans/issue-<N>/progress.md` in the issue's worktree. If it
doesn't exist, stop with an error — there's no triage to address.

### 2. Read the latest Integrated Review

Use the shared parser rather than re-parsing markdown:

```bash
python3 .agent/scripts/progress_read.py \
    .agent/work-plans/issue-<N>/progress.md --type "Integrated Review"
```

`progress_read.py` emits JSON; take the **last** `Integrated Review`
entry's `findings[]` array. Each finding is
`{section, checked, source_hint, text}`. The action items are the
entries with `checked == false`. (False positives are plain bullets, so
they never appear in `findings[]` — no special handling needed.)

If there are no unchecked findings, report "nothing to address — the
latest Integrated Review has no open actions" and exit without a commit.

### 3. Address each open finding

For each unchecked finding, in listed order (cross-confirmed first):

1. Make the code/doc change the finding calls for. Read the cited
   `file:line` and the surrounding code first — verify the finding
   against the current source before acting (a finding can be stale if
   an earlier round already touched the area).
2. If, on inspection, the finding is **not** actionable (already fixed,
   or a genuine false negative on re-read), do **not** fake a change —
   leave the box unchecked and record why in the `## Implementation`
   entry's deferred list (step 5). Never check a box you didn't act on.
3. Stage only the files for this finding and commit atomically with
   pre-commit hooks and per-invocation identity (AGENTS.md § Agent
   Commit Identity):

   ```bash
   git -c user.name="$AGENT_NAME" -c user.email="$AGENT_EMAIL" \
       commit -m "<area>: <what was fixed> (#<N>)"
   ```

   Never `--no-verify`. One logical fix per commit.
4. Check the box for that finding in the `## Integrated Review` entry
   (`- [ ]` → `- [x]`) so the timeline tracks resolution. This edit can
   ride in the same commit as the fix, or a trailing progress commit.

### 4. Re-run hooks / quick local checks

After the last fix, run the relevant quick checks (the linters
pre-commit already ran, plus any package test touched by the changes).
This is not a full `review-code` — it's a sanity pass before handing to
the re-review.

### 5. Write the `## Implementation` entry

Append one entry (per ADR-0013; reuse `## Implementation`):

```markdown

## Implementation
**Status**: complete
**When**: <YYYY-MM-DD HH:MM ±HH:MM>
**By**: <agent name> (<model>)

**Addressed**: <integrated-review entry timestamp/sha it consumed>
**Commits**: <short-shas of the fix commits>

### Actions
- [x] <finding addressed> — `file:line`
- [ ] <finding deferred / not actionable> — `file:line` (reason)
```

Commit and push:

```bash
git -C <worktree> add .agent/work-plans/issue-<N>/progress.md
git -C <worktree> -c user.name="$AGENT_NAME" -c user.email="$AGENT_EMAIL" \
    commit -m "progress: addressed integrated-review findings for #<N>"
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

- **Act on the agreed plan, don't re-litigate it** — `triage-reviews` already
  classified each finding. If you disagree with a finding on inspection, defer
  it with a reason; don't silently drop it or argue it in the commit.
- **Never fake resolution** — a checked box must correspond to a real commit.
  Deferred/non-actionable items stay unchecked with a recorded reason.
- **Atomic commits** — one finding per commit where practical, so a re-review
  (and a revert) can reason about each fix independently.
- **Stay thin** — no re-classification, no self-review. The next `review-code`
  pass is the quality gate on this work.
