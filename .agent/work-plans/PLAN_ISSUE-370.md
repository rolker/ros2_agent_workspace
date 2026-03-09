# Plan: triage-reviews: fetch all reviews and include CI status

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/370

## Context

`fetch_pr_reviews.sh` filters reviews by HEAD's committer date, which breaks
when unrelated commits land after reviews are submitted. The cutoff jumps past
pending reviews, silently hiding them. The `triage-reviews` skill depends on
this script and inherits the blind spot.

The fix removes timestamp filtering from the script (return all reviews) and
adds CI check-run data. The skill takes over timeline reasoning using commit
IDs already present in the review data.

## Approach

1. **Update `fetch_pr_reviews.sh` — remove timestamp filter, add CI status**
   - Remove the `HEAD_TIMESTAMP` / cutoff logic entirely.
   - Return all reviews unconditionally (keep `user_login`, `user_type`,
     `submitted_at`, `commit_id` on each review for skill-side reasoning).
   - Add a new `ci_status` array to the output by querying
     `repos/{slug}/commits/{sha}/check-runs` for the PR's head SHA.
     Each entry: `{name, conclusion, html_url}`.
   - Update the script's header comment (lines 2–4), help text (line 51),
     and `--help` output to reflect the new behavior.
   - Remove the "Limitations" block (lines 11–15) since the timestamp
     approach is gone.

2. **Update `SKILL.md` — classify using full review history + CI section**
   - **Step 3** (fetch): Remove references to "cutoff timestamp" and
     "comments submitted after the most recent commit." The script now
     returns all reviews; the stop condition changes to: if the result
     contains no reviews at all, report "No reviews on this PR" and stop.
   - **Step 5** (evaluate): Add guidance on using `commit_id` from each
     review to determine if subsequent commits may have addressed a comment.
     If the file at the referenced path+line has changed between the
     review's `commit_id` and HEAD, classify as "Potentially addressed —
     verify." For force-pushed branches where `commit_id` is unreachable,
     fall back to reading the current code and noting the uncertainty.
   - **Step 6** (report template): Replace `**Cutoff**` with
     `**Reviews**: <total> review(s), <total> inline comment(s)`.
     Add a `### CI Status` section after the summary:
     ```
     ### CI Status
     | Check | Result | Link |
     |-------|--------|------|
     | Lint  | pass   | URL  |
     ```
   - **Frontmatter**: Update description to mention CI status.

3. **Update `AGENTS.md` script reference table**
   - Change the `fetch_pr_reviews.sh` entry from
     "Fetch PR review comments after HEAD" to
     "Fetch all PR reviews and CI status".

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/fetch_pr_reviews.sh` | Remove timestamp filter; add CI check-runs to output; update header/help |
| `.claude/skills/triage-reviews/SKILL.md` | Update steps 3/5/6 for full-history mode + CI section; update frontmatter |
| `AGENTS.md` | Update script reference table entry for `fetch_pr_reviews.sh` |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Returning all reviews gives the agent and user full visibility instead of silently filtering. CI status adds another transparency layer. |
| A change includes its consequences | Plan includes AGENTS.md update and SKILL.md frontmatter — all downstream references covered. |
| Only what's needed | Three files, focused changes. No new scripts or abstractions. |
| Improve incrementally | Enhancement to existing tool; no redesign. |
| Test what breaks | No automated tests added — the script is a thin API wrapper and the jq pipeline is straightforward. Shellcheck runs in CI via pre-commit. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0002 — Worktree isolation | Yes | Work done in worktree `issue-workspace-370`. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Script in `.agent/scripts/` | AGENTS.md script reference table | Yes — step 3 |
| Framework skill `.claude/skills/` | Skill frontmatter description | Yes — step 2 |

## Open Questions

- **Force-pushed branches**: When a review's `commit_id` is no longer reachable
  in the branch history, we can't do a reliable file-diff check. The plan
  proposes falling back to reading current code + noting uncertainty. Is this
  acceptable, or should we try harder (e.g., `git cat-file -e`)?

## Estimated Scope

Single PR.
