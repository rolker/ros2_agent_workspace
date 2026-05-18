# Plan: Scope a Copilot-only cross-model adversarial review

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/461

## Context

PR #453 ported the review-skill from `rolker/agent_workspace` but
deliberately did not adopt its `cross_model_review.sh` (tmux dispatch +
Gemini/Codex/Copilot). The 2026-05-15 scoping comment on #461 settled
the design for a slimmed-down, Copilot-only revisit. The four
acceptance-criteria questions were resolved 2026-05-18:

1. Activate on **all tiers** (Light + Standard + Deep), `--no-copilot`
   opt-out per invocation.
2. **Graceful skip with notice** when `copilot` binary missing or
   unauthenticated (covers field hosts gabby/salmon).
3. **Default Copilot context** for v1 (~25.5k token floor accepted);
   evaluate after a few real runs and tighten via `--add-dir` or
   isolated `-C` if Premium burn is unacceptable.
4. **File an issue on `rolker/agent_workspace`** about its likely-broken
   `copilot -p < prompt` invocation; decoupled from this PR.

## Approach

1. **Add Copilot Adversarial as a new specialist (5e) in `review-code` SKILL.md.**
   Synchronous call, no tmux. Invocation:
   `copilot -p "" --allow-all-tools < prompt > findings 2>&1`. Strip
   the trailing `Changes / Requests / Tokens` metadata block.
   Activates on Light + Standard + Deep. Prompt mirrors Claude
   Adversarial's tier-appropriate focus areas (Light reuses the
   Standard prompt body since Claude Adversarial doesn't run on Light).
2. **Implement availability detection.** Before invoking, probe with
   `command -v copilot` and a quick auth check
   (`copilot --version` succeeds without prompting). On failure, emit a
   one-line `Copilot Adversarial skipped: <reason>` finding and
   continue. No flag needed.
3. **Add `--no-copilot` flag** to the `review-code` Usage block and
   argument parser. When set, skip dispatch entirely without the
   "skipped" notice.
4. **Update the Specialists overview** in step 5 to list 5a–5e and
   remove the "Cross-model adversarial is intentionally not wired here"
   note from 5d (replace with a pointer to 5e).
5. **Update `inspiration_agent_workspace_digest.md`.** Move the
   Cross-Model entry from "Not adopted" to a new "Partially adopted"
   section: Copilot-only synchronous dispatch ported; tmux machinery
   and Gemini/Codex remain non-adopted with the existing rationale.
6. **File the upstream invocation report** on
   `rolker/agent_workspace` as a discrete step. Title: "cross_model_review.sh:
   Copilot invocation likely missing `-p \"\"` and `--allow-all-tools`".
   Body: our verified local invocation + smoke-test repro. Not a
   blocker for this PR.
7. **Open a follow-up issue** for the v1 context-strategy evaluation
   ("Evaluate Copilot Adversarial context cost; tighten if needed").
   Link it from the digest entry and from `review-code` SKILL.md so the
   v1 choice doesn't silently calcify.

## Files to Change

| File | Change |
|------|--------|
| `.claude/skills/review-code/SKILL.md` | Add 5e Copilot Adversarial subsection; add `--no-copilot` to Usage + arg parser; update step-5 tier table to dispatch 5e on all tiers; replace 5d cross-model-not-wired note with pointer to 5e |
| `.agent/knowledge/inspiration_agent_workspace_digest.md` | Move Cross-Model entry from "Not adopted" → new "Partially adopted" entry; preserve non-Copilot non-adoption rationale |

No new script file. The invocation is small enough (3–4 lines: probe,
call, strip trailing block) to live inline in SKILL.md as part of step
5e, matching how 5a–5d are documented. If the inline shell grows past
~15 lines during implementation, factor into
`.agent/scripts/run_copilot_adversarial.sh`.

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | `--no-copilot` flag + "skipped" notice in report + tier table in SKILL.md make it visible when Copilot runs, when it doesn't, and how to suppress it. Default-on is "tight by default, relaxable as confidence grows." |
| Only what's needed | Copilot-only, synchronous, no tmux, no `--add-dir` plumbing. Inline shell in SKILL.md rather than a new script unless complexity demands it. |
| A change includes its consequences | Digest entry updated; follow-up evaluation issue filed so the v1 context choice has an owner; upstream invocation report filed; no schema changes to progress.md / review-context.yaml. |
| Capture decisions, not just implementations | Acceptance-criteria answers recorded on #461; rationale repeated in SKILL.md prose under 5e; "Partially adopted" entry captures the cross-model design split. |
| Improve incrementally | v1 ships default context + all-tier activation; tier-scoping and context-tightening deferred to the follow-up issue once we have cost data. |
| Workspace vs project separation | Workspace tooling; no project-repo coupling. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0002 worktree isolation | Yes | Plan + implementation land on `feature/issue-461` workspace worktree |
| 0003 workspace infra is project-agnostic | Yes | `review-code` and the new specialist are generic ROS 2 / workspace tooling; no project-specific assumptions |
| 0004 enforcement hierarchy | Yes | `--no-copilot` opt-out is a user-facing escape hatch; default-on enforces the cross-model second-read by default |
| 0011 field mode | Yes | Graceful skip when Copilot unavailable covers field-mode hosts (gabby/salmon) without a separate code path |

## Consequences

| If we change… | Also update… | Included in plan? |
|---|---|---|
| `review-code` specialist set (add 5e) | `inspiration_agent_workspace_digest.md` "Not adopted" → "Partially adopted" | Yes — Step 5 |
| `review-code` Usage (`--no-copilot`) | None — `review-code` is invoked as a skill; no other consumers of its CLI surface | Verify via workspace grep before commit |
| `review-code` specialist set | `AGENTS.md` / `CLAUDE.md` references to review-code | Workspace grep planned; spot-check during implementation |
| Default-on Premium-request consumption | Cost-evaluation follow-up issue | Yes — Step 7 |
| Upstream `cross_model_review.sh` invocation suspected broken | Issue filed on `rolker/agent_workspace` | Yes — Step 6 |

## Open Questions

None — acceptance-criteria answers (#461 comments) cover all four.

## Estimated Scope

Single PR. Two file edits + two new issues filed (one upstream, one
follow-up). No new scripts unless inline shell exceeds ~15 lines
during implementation.
