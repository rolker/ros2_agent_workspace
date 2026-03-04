# Plan: Fix fetch_copilot_reviews.sh "Argument list too long" crash

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/347

## Context

`fetch_copilot_reviews.sh` crashes on line 143 when processing PRs with large
review histories. The `jq` invocation uses `--argjson comments "$ALL_COMMENTS"`
which places the entire paginated API response on the command line, exceeding
the OS `ARG_MAX` limit. The script was added in #343 and first failed on PR #338.

## Approach

1. **Replace `--argjson` with stdin-based input** — Instead of passing
   `$ALL_COMMENTS` as a `--argjson` argument, use `jq`'s `--slurpfile` with a
   temp file or restructure the pipeline to pass both inputs via a combined
   JSON object piped to stdin. The cleanest approach: write `$ALL_COMMENTS` to
   a temp file and use `--slurpfile comments <tmpfile>`, which reads from disk
   instead of the command line. Adjust the filter to use `$comments[0][]`
   since `--slurpfile` wraps contents in an array.

2. **Clean up the temp file** — Use a trap or explicit `rm` to ensure the
   temp file is removed on exit (normal or error).

3. **Verify with reproduction steps** — Run the fixed script against PR #338
   from the issue-332 worktree to confirm the fix works.

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/fetch_copilot_reviews.sh` | Replace `--argjson comments "$ALL_COMMENTS"` with `--slurpfile` from a temp file; update jq filter to use `$comments[0][]` |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| A change includes its consequences | Bug fix with no interface change — no doc/test/reference updates needed |
| Only what's needed | Single targeted fix, no scope creep |
| Improve incrementally | Small, reviewable change |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0002 — Worktree isolation | Yes | Working in worktree `issue-workspace-347` |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `.agent/scripts/fetch_copilot_reviews.sh` | `AGENTS.md` script table / `Makefile` | No — not needed (bug fix, no interface change) |

## Open Questions

None — the approach is straightforward.

## Estimated Scope

Single PR.
