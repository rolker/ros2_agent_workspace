# Plan: triage-copilot skill misses human and non-bot Copilot comments

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/366

## Context

The `triage-copilot` skill uses `fetch_copilot_reviews.sh` to retrieve PR
review comments. The script filters reviews by `user.login` matching
"copilot" (case-insensitive) or `github-actions[bot]`, which catches the
`copilot-pull-request-reviewer[bot]` reviews but misses:

1. **Human comments** (e.g., `rolker`) — review objects with `user.login`
   that doesn't match the copilot pattern
2. **`Copilot` user comments** — inline comments attributed to the `Copilot`
   user (type: Bot) that happen to be attached to bot reviews (so currently
   included by accident, but not by design)

The timestamp filter (only show comments after HEAD) is valuable and should
be retained — the owner noted that agents previously couldn't distinguish
addressed vs. unaddressed reviews. The fix is to remove the user-login
filter while keeping the timestamp filter, and add source attribution so the
skill can apply context-appropriate triage.

Owner guidance: "Probably a good idea to show all human comments, but make
sure to understand the context of the comment to decide if it still applies.
Actually all comments could be considered with that caveat."

## Approach

1. **Rename script** — `fetch_copilot_reviews.sh` → `fetch_pr_reviews.sh`
   to reflect broader scope. Update all references.

2. **Remove user-login filter** — keep only the timestamp filter
   (`submitted_at > HEAD committer date`). All reviews after HEAD are
   included regardless of author.

3. **Add user attribution to JSON output** — include `user_login` and
   `user_type` fields in both review and comment objects so the skill can
   distinguish human vs. bot sources.

4. **Update skill instructions** — the triage-copilot skill should:
   - Process all review comments (not just Copilot)
   - Group output by source (human reviewers vs. Copilot)
   - For human comments: check if already addressed by recent commits
   - For Copilot comments: evaluate as before (valid issue vs. false positive)
   - Rename report header to "PR Review Triage" (not "Copilot Triage")

5. **Update AGENTS.md script reference** — rename the script entry.

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/fetch_copilot_reviews.sh` | Rename to `fetch_pr_reviews.sh`; remove user-login filter from jq; add `user_login`/`user_type` to output JSON |
| `.agent/scripts/fetch_pr_reviews.sh` | (new name) See above |
| `.claude/skills/triage-copilot/SKILL.md` | Update step 3 script reference; update step 5/6 to handle all comment sources; update report template |
| `AGENTS.md` | Rename script in reference table |
| `.github/copilot-instructions.md` | Update script reference if present |
| `.agent/instructions/gemini-cli.instructions.md` | Update script reference if present |
| `.agent/AGENT_ONBOARDING.md` | Update script reference if present |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Fix directly serves this — human comments were invisible to the triage workflow |
| A change includes its consequences | Script rename requires updating all references (AGENTS.md, skill, adapters) |
| Only what's needed | Minimal change: remove one jq filter, add two fields, update references |
| Improve incrementally | Single PR, focused scope |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0006 Shared AGENTS.md | Yes — script table change | Update AGENTS.md script reference table |
| ADR-0004 Enforcement hierarchy | No | N/A |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Script name in `.agent/scripts/` | `AGENTS.md` script reference table | Yes |
| Skill behavior | Skill file in `.claude/skills/` | Yes |
| Script referenced in framework adapters | Non-Claude adapters | Yes |

## Open Questions

- Should the skill be renamed from `triage-copilot` to something broader
  (e.g., `triage-reviews`)? The current name is misleading since it will
  now triage all review comments. Renaming involves updating the skill
  directory name, all adapter skill lists, and user muscle memory.

## Estimated Scope

Single PR — script change + skill update + reference updates.
