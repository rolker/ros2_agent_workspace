# Plan: Add triage-copilot skill for evaluating Copilot review feedback

**Issue**: [#343](https://github.com/rolker/ros2_agent_workspace/issues/343)
**Status**: In Progress

## Context

After Copilot reviews a PR, an agent must manually fetch review comments, read
each one, evaluate it against local code, and decide which are valid vs false
positives. This is a recurring pattern that can be structured into a skill.

The issue proposes a read-only triage skill (`/triage-copilot <PR>`) that:
- Fetches Copilot review comments newer than HEAD
- Evaluates each against the local worktree code, principles, and ADRs
- Classifies comments as valid issues or false positives
- Presents a structured plan — does not auto-fix

A [design note comment](https://github.com/rolker/ros2_agent_workspace/issues/343#issuecomment-3998184737)
proposes a helper script to consolidate `gh api` calls into one permission prompt.

A [review comment](https://github.com/rolker/ros2_agent_workspace/issues/343#issuecomment-3998397086)
confirms the scope and flags consequences (adapter skill lists, script reference table).

## Approach

1. **Create helper script** — `.agent/scripts/fetch_copilot_reviews.sh --pr <N>`
   - Accepts `--pr <N>` (required)
   - Gets HEAD commit timestamp via `git log`
   - Fetches Copilot review IDs and timestamps via `gh api`
   - Filters to reviews submitted after HEAD's committer date
   - Fetches comments for matching reviews
   - Outputs structured JSON to stdout
   - Handles edge cases: no commits, zero Copilot reviews, API errors

2. **Create skill file** — `.claude/skills/triage-copilot/SKILL.md`
   - YAML header with name and description
   - Steps: confirm worktree, sync, run helper script, evaluate each comment
     against local code + governance, classify and present
   - Output format: structured markdown in conversation (no files written)
   - Guidelines section matching review-pr style

3. **Update adapter skill lists** — add `triage-copilot` to:
   - `.github/copilot-instructions.md`
   - `.agent/instructions/gemini-cli.instructions.md`
   - `.agent/AGENT_ONBOARDING.md`

4. **Update script reference table** — add `fetch_copilot_reviews.sh` to
   `AGENTS.md` script reference table

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/fetch_copilot_reviews.sh` | New helper script — fetch + filter Copilot reviews as JSON |
| `.claude/skills/triage-copilot/SKILL.md` | New skill file — triage workflow |
| `.github/copilot-instructions.md` | Add `triage-copilot` to workflow skill list |
| `.agent/instructions/gemini-cli.instructions.md` | Add `triage-copilot` to workflow skill list |
| `.agent/AGENT_ONBOARDING.md` | Add `triage-copilot` to workflow skill list |
| `AGENTS.md` | Add `fetch_copilot_reviews.sh` to script reference table |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Plan-only output; user decides what to fix. No auto-fix, no comments posted. |
| A change includes its consequences | Adapter skill lists and script reference table updated in same PR. |
| Only what's needed | Solves a concrete recurring task. Helper script consolidates permission prompts — pragmatic. |
| Improve incrementally | Standalone skill; no changes to existing skills or workflows. |
| Workspace vs. project separation | Generic workspace tooling — works with any PR in any repo. |
| Primary framework first, portability where free | Claude Code skill (framework-specific). Helper script is plain bash + gh (portable). Skill docs in `.claude/skills/` are plain markdown readable by other frameworks. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0003 — Project-agnostic workspace | Yes | Skill is generic — not coupled to any project repo |
| 0006 — Shared AGENTS.md | Yes | Script reference table updated in AGENTS.md |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| A framework skill (`.claude/skills/`) | Adapter files | Yes — step 3 |
| Workflow skill list (add a skill) | Copilot, Gemini, Onboarding adapter files | Yes — step 3 |
| A script in `.agent/scripts/` | Script reference table in `AGENTS.md` | Yes — step 4 |

## Open Questions

- **Committer date vs author date**: The issue says "comments newer than HEAD."
  Rebases can make author and committer dates diverge. Plan uses **committer date**
  (most recent action on the branch) — is that correct? (Review comment flagged this.)
- **Force-push handling**: If the PR has been force-pushed, HEAD may not match
  what Copilot reviewed. The helper script should document this limitation.
  Acceptable for v1?
- **Cross-repo PRs**: Should the helper script auto-detect the repo slug from
  the current directory, or require it as an argument? Plan: auto-detect via
  `gh repo view --json nameWithOwner`.

## Estimated Scope

Single PR — all 6 files are small, self-contained changes.
