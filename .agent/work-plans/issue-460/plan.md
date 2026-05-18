# Plan: review-code flag additions + distinct pre-push progress header (#460)

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/460

## Context

PR #453 ported the dual-mode `/review-code` from `rolker/agent_workspace`.
Upstream PR [rolker/agent_workspace#185](https://github.com/rolker/agent_workspace/pull/185)
landed afterwards and refined the same skill with four small additions.
The inspiration-tracker run that produced PR #459 captured these as
Pending Review in `.agent/knowledge/inspiration_agent_workspace_digest.md`;
this PR lands them.

All four additions are additive on top of the dual-mode shape we shipped.
No design unknowns — the upstream implementation answers each. The work
is small enough that one PR with four atomic commits is the right shape
(per the workspace's bundle-related-cleanups preference).

## Scope

All edits are in `.claude/skills/review-code/SKILL.md`. No other files
need to change.

### 1. `--skip-static` flag (both modes)

Suppress the Static Analysis Specialist when set. Useful when pre-commit
was already clean or the user has run linters separately. Surfaces in
the report header as `**Static analysis**: skipped (--skip-static)`.

Touch points:
- **Usage block** — add to both mode lines
- **Step 1 argument parsing** — recognize and consume the flag, export
  `SKIP_STATIC=true`
- **Step 5a** — early-return when `SKIP_STATIC=true`; emit a one-line
  "skipped (--skip-static)" note for the silence filter and report
- **Step 7 report headers** — show the skip status when set
- **Step 7 silence-filter wording** — clarify that at Light tier with
  `--skip-static`, zero specialists run and the "No findings" path
  fires automatically

### 2. `--no-progress` flag (pre-push mode)

Explicit opt-out for `progress.md` persistence. Today the skill skips
persistence silently when no issue can be resolved from the branch
name; `--no-progress` makes the opt-out user-visible and decouples it
from the issue-resolution failure path.

Touch points:
- **Usage block** — add to pre-push line
- **Step 1 pre-push subsection** — argument parsing; export
  `NO_PROGRESS=true`
- **Step 8 entry point** — skip the entire persistence step when
  `NO_PROGRESS=true`; note "skipped (--no-progress)" in report Summary

This flag does **not** apply to post-PR mode (a PR with no closing
references is the post-PR equivalent and already handled by step 8's
fallthrough). Don't add it to the post-PR Usage line.

### 3. `--issue <N>` override (pre-push mode)

Allow explicit issue arg to override the branch-name extraction at
Step 1 line 113–114. Useful for skill worktrees and one-off branches
whose names don't carry `issue-<N>`.

Touch points:
- **Usage block** — add to pre-push line
- **Step 1 pre-push subsection** — argument parsing; when present,
  set `ISSUE_NUM=<arg>` and skip the branch-name extraction

The override is mutually exclusive with `--no-progress` only in spirit
(they both relate to progress.md handling). Don't enforce an error if
both are passed — `--no-progress` wins (no persistence regardless of
issue number).

### 4. Distinct `## Local Review (Pre-Push)` vs `## Local Review` headers

Allow pre-push and post-PR entries for the same issue to coexist on
the timeline.

Touch points:
- **Step 8 template** — switch header between `## Local Review` and
  `## Local Review (Pre-Push)` based on mode
- The `**PR**` / `**Branch**` placeholder split landed in PR #453
  triage already covers the body; only the header needs updating

## Out of scope

- **Cross-model script `--branch` mode** (upstream PR #185 also touches
  `cross_model_review.sh`). Not applicable here — we explicitly didn't
  port cross-model. Recorded under "Not adopted" in the digest.
- **`_resolve_default_branch.sh` helper**. Equivalent to our existing
  in-skill resolution (with the pipefail fix from PR #453 triage R5-1).
  No porting needed.

## Implementation order

Four atomic commits, each independently revertable:

1. `--skip-static` (largest — affects step 1 + step 5a + step 7)
2. `--no-progress` (step 1 + step 8)
3. `--issue <N>` override (step 1 + interaction with step 8)
4. Distinct pre-push header (step 8 template)

## Test plan

The skill is markdown — there's no executable test. Verification is
manual against the documented behavior:

- [ ] Read each touched section after each commit; confirm the flag
      is mentioned in Usage and handled in the relevant Step.
- [ ] After commit 1, confirm `--skip-static` references match across
      Usage, Step 1, Step 5a, Step 7.
- [ ] After commit 2, confirm `--no-progress` is only on the pre-push
      Usage line (not post-PR).
- [ ] After commit 3, confirm `--issue <N>` argument parsing overrides
      branch-name extraction in a clear way.
- [ ] After commit 4, confirm `## Local Review (Pre-Push)` header is
      conditional on mode, with an example template entry.
- [ ] Run `/review-code` pre-push on the resulting diff before opening
      the PR (per AGENTS.md Post-Task Verification step 5).

## Open questions

None. All four items have clear upstream precedent.

## Principles self-check

| Principle | Compliance |
|---|---|
| Only what's needed | Yes — four narrow additions, no scope creep. |
| Human control and transparency | Yes — all four flags are user-facing opt-ins; defaults preserve current behavior. |
| Test what breaks | Limited — skill is markdown, no automated tests. Manual verification + pre-push self-review covers this. |
| Capture decisions, not just implementations | Plan + atomic commit messages reference each upstream source. |
| Bundle related cleanups; don't split | One PR, four atomic commits — matches the bundle preference for this kind of additive port. |

## Consequences

- `.agent/knowledge/inspiration_agent_workspace_digest.md` Pending
  Review section will need its 4 items moved to "Ported" after merge —
  the inspiration-tracker skill handles this automatically on the next
  run, no manual edit needed in this PR.
- No external surfaces affected. No project repos touched.
