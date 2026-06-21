---
issue: 466
skill: plan-task
---

# Work Plan — Issue #466: Local-First Posting for review-issue

**Issue**: #466 — Persist review-issue and review-plan output to progress.md
**Scope**: POST-SIDE ONLY — skill-side posting behavior in `review-issue` SKILL.md.
**Out of scope** (tracked in #552): host injecting read-context; false-FAILED on
re-dispatch replacing a typed entry (count 1→1).

## Context

The persistence half of issue #466 is already shipped: both `review-issue` and
`review-plan` write typed `## Issue Review` / `## Plan Review` entries to
`progress.md` (observed driving #550). What remains is a posting-order and
reporting bug in `review-issue`:

1. **Wrong order**: step 7 (GitHub comment) runs before step 8 (progress.md
   commit). If posting fails, progress.md may not yet have been written.
2. **Wrong status**: when posting fails due to missing gh auth (the normal
   container case), the skill self-marks `**Status**: partial`, even though it
   produced a complete progress.md entry. `dispatch_subagent.sh` maps
   `partial → exit 1`, so a fully-successful container phase pages the host as
   "FAILED".

`review-plan` has no posting step at all — no change needed there.

`dispatch_subagent.sh` already maps `complete → exit 0` (lines 481–482, 492–493).
No code change is needed in that script; the fix is entirely in `review-issue`
SKILL.md.

## Changes

### 1. Reorder steps in `review-issue` SKILL.md

**File**: `.claude/skills/review-issue/SKILL.md`

Current order:
- Step 7 — Post findings as a GitHub comment
- Step 8 — Persist to progress.md + commit

New order (swap):
- Step 7 — Persist to progress.md + commit (**canonical record — always runs**)
- Step 8 — Post findings as a GitHub comment (**best-effort / optional**)

The commit in step 7 becomes the primary record. Step 8 is a publication
convenience that may be skipped without affecting the lifecycle state.

### 2. Gate the posting step on gh auth

In the new step 8, add a pre-flight check before calling `gh issue comment`:

```bash
# Best-effort only — skip silently if gh auth is absent
if gh auth status --active 2>/dev/null; then
    gh issue comment <N> --body-file "$BODY_FILE"
else
    echo "[review-issue] No GitHub auth — skipping comment post (progress.md is the record)."
fi
```

Do not set `**Status**: partial` based on this branch. The status is `complete`
regardless of whether the post succeeded.

### 3. Update `**Comment**:` field in the progress.md template

The `**Comment**:` line in the `## Issue Review` entry currently expects a
GitHub comment URL. With best-effort posting it may not have one. Use:

- When posting succeeded: `**Comment**: <URL>`
- When posting was skipped: `**Comment**: (not posted — no GitHub auth)`

This is already the pattern used by the review performed while driving #550
(see the existing `## Issue Review` entry in this file).

### 4. Document the decision in SKILL.md

Add a "Posting behavior" note to the `## Overview` section of
`review-issue/SKILL.md` making the best-effort contract explicit:

> **GitHub posting is best-effort.** The `progress.md` entry (step 7) is the
> canonical record. Step 8 posts a comment for human visibility on GitHub, but
> skips silently when no gh auth is present (e.g., container dispatch per
> #532). A skipped post does **not** change the phase status — the skill
> reports `complete` as long as the progress.md entry was committed.

### 5. Verify dispatch_subagent.sh (no code change expected)

Confirm that `dispatch_subagent.sh` maps `complete → exit 0` (already true at
lines 481–482, 492–493 in the current file). If confirmation finds a gap,
add a clarifying comment — but do not change the mapping logic. Update
AGENTS.md script table only if `dispatch_subagent.sh` is actually modified.

## Files Touched

| File | Change |
|------|--------|
| `.claude/skills/review-issue/SKILL.md` | Reorder steps 7↔8; add gh-auth gate; document best-effort decision in Overview |
| `.agent/scripts/dispatch_subagent.sh` | Read-only verification; comment only if modified |
| `AGENTS.md` | Only if `dispatch_subagent.sh` changes (not expected) |

## Acceptance Criteria (from issue)

- [ ] `review-issue` reports `**Status**: complete` when the only unmet step is
  the optional GitHub post
- [ ] No gh auth ⇒ skip the post silently; `progress.md` entry is committed
  before the posting attempt
- [ ] `dispatch_subagent.sh` does not surface a locally-complete phase as FAILED
  / exit 1 because the optional GitHub post was skipped
- [ ] Skill docs updated with the best-effort-posting decision

## Step Sequence

1. Edit `review-issue/SKILL.md`: swap steps 7 and 8, add gh-auth gate, add
   Overview note on best-effort posting, update `**Comment**:` field template.
2. Read `dispatch_subagent.sh` to confirm `complete → exit 0` mapping; add
   comment if helpful (no logic change).
3. Run pre-commit, commit changes with agent identity.
4. Run `/review-code` pre-push against the diff.
5. Address any review findings; re-commit if needed.

## Consequences

- All container-dispatched `review-issue` phases (and future peers using the
  same pattern) will now correctly report `complete` when the optional post is
  skipped.
- The `## Issue Review` entry is always present in `progress.md` before any
  posting attempt — the timeline is correct even if posting fails mid-way.
- No change to the human-facing review content or the governance evaluation
  logic.
