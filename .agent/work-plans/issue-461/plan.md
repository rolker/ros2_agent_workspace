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
   Activates on Light + Standard + Deep. **Light tier reuses the
   Standard prompt body** (settled 2026-05-18 after review-plan
   flagged this as an unsurfaced design choice); Deep tier adds the
   existing Deep-prompt focus areas (security / concurrency /
   lifecycle / cross-cutting effects). Update the Light condensed
   report format (review-code SKILL.md lines 444–459) to include a
   Copilot Adversarial findings slot.
2. **Implement availability detection + post-invocation guard.**
   Before invoking, probe with `command -v copilot` and a sanity check
   `copilot --version` (presence-only — explicitly not an auth check;
   real auth detection happens after the call). After invoking, route
   timeout (exit 124), non-zero exit, empty output, or auth-error
   text in the output to the skipped-with-notice path so
   unauthenticated hosts never surface as silent zero-finding reviews.
   Wrap the invocation in `timeout 300` so a hung CLI cannot block
   the whole review. (Post-review hardening; see Implementation Notes.)
2a. **Implement untrusted-PR safety gate** (post-PR mode only). Before
    dispatching 5e, check whether the PR head is from a fork or the
    author lacks OWNER/MEMBER/COLLABORATOR association. If so, route to
    the skipped-with-notice path unless `--allow-untrusted-copilot` is
    set. Rationale: `--allow-all-tools` exposes file/shell access to
    Copilot, and an external contributor's diff is attacker-controlled
    prompt content. (Post-review hardening; see Implementation Notes.)
3. **Add `--no-copilot` flag** to the `review-code` Usage block and
   argument parser. When set, skip dispatch entirely without the
   "skipped" notice. Also add `--allow-untrusted-copilot` (post-PR only)
   as the explicit bypass for step 2a's gate.
4. **Update the Specialists overview** in step 5 to list 5a–5e and
   remove the "Cross-model adversarial is intentionally not wired here"
   note from 5d (replace with a pointer to 5e).
5. **Update `inspiration_agent_workspace_digest.md`.** Move the
   Cross-Model entry from "Not adopted" to a new "Partially adopted"
   section: Copilot-only synchronous dispatch ported; tmux machinery
   and Gemini/Codex remain non-adopted with the existing rationale.
   Also edit the "Ported → Adversarial Specialist" entry (line ~62)
   which currently states "Cross-model variant deliberately not
   ported" — that sentence directly contradicts the new partial
   adoption.

5a. **Update `.agent/knowledge/review_depth_classification.md`.** Three
    sites go stale on merge:
    - Light / Standard / Deep specialist lists (~lines 68–69, 81–86,
      98–102) — add Copilot Adversarial to all three.
    - "Note on cross-model adversarial" block (~lines 106–111) — its
      "not adopted here" framing inverts after this PR.
    - Light condensed report description (~lines 71–72) — needs a
      slot or pointer for Copilot findings to match the SKILL.md
      Light report shape.

5b. **Update `.agent/knowledge/skill_workflows.md` line 18.** The
    "Adversarial-Claude-only caveat" reference (applied to non-Claude
    framework adapters) becomes stale once Copilot Adversarial is
    in-tree on all tiers. Reword or remove.
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
| `.claude/skills/review-code/SKILL.md` | Add 5e Copilot Adversarial subsection (Standard prompt reused on Light + Deep adds existing Deep focus areas); add `--no-copilot` and `--allow-untrusted-copilot` to Usage + arg parser; update step-5 tier dispatch to fire 5e on all tiers; update Light condensed report format to include Copilot findings slot; rename 5d to "Claude Adversarial Specialist"; replace 5d cross-model-not-wired note with pointer to 5e. Post-review hardening: `timeout 300` wrapper, post-invocation guard (empty / auth-error / non-zero exit / timeout → skipped path), untrusted-PR safety gate, `**Copilot Adversarial**:` parallel header line in both standard and Light condensed report templates, qualified "Light + --skip-static = zero specialists" predicate to require Copilot also suppressed. |
| `.agent/knowledge/inspiration_agent_workspace_digest.md` | Move Cross-Model entry from "Not adopted" → new "Partially adopted" entry; preserve non-Copilot non-adoption rationale; fix the "Ported → Adversarial Specialist" stale "Cross-model variant deliberately not ported" sentence |
| `.agent/knowledge/review_depth_classification.md` | Add Copilot Adversarial to Light/Standard/Deep specialist lists; invert the "Note on cross-model adversarial" block; add Copilot slot to Light condensed report description |
| `.agent/knowledge/skill_workflows.md` | Reword the "Adversarial-Claude-only caveat" reference to "the Claude Adversarial Specialist still requires Claude Code's `Agent` tool, but the Copilot Adversarial Specialist runs from any runtime that has the `copilot` CLI"; qualify the pre-push coverage claim by tier (Light = SA+Copilot only; Standard/Deep add Governance + Plan Drift + Claude Adversarial). |
| `.github/copilot-instructions.md` | Rename "Adversarial Specialist is Claude-only" to "Claude Adversarial Specialist is Claude-only"; add Copilot Adversarial to framework-agnostic list; update the pre-push caveat (Copilot's pre-push pass cannot catch *Claude*-side adversarial findings — Copilot Adversarial runs natively); qualify the pre-push coverage claim by tier. |
| `.agent/instructions/gemini-cli.instructions.md` | Same rename + Copilot Adversarial addition pattern as the Copilot adapter |
| `.agent/AGENT_ONBOARDING.md` | Same rename + Copilot Adversarial addition pattern as the Copilot adapter |

No new script file. The inline shell in SKILL.md grew past the
original ~15-line threshold during post-review hardening (now ~60
lines across probe + gate + invocation + post-call guard), but the
content is kept inline rather than factored to
`.agent/scripts/run_copilot_adversarial.sh` because the security model
(`--allow-all-tools` + the untrusted-PR gate) needs to be visible to
the next agent reading 5e — not buried behind a script reference.
Revisit the factor-out decision if 5e accumulates non-security
elaboration in a future change.

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | `--no-copilot` flag + "skipped" notice in report + tier table in SKILL.md make it visible when Copilot runs, when it doesn't, and how to suppress it. Default-on is "tight by default, relaxable as confidence grows." |
| Only what's needed | Copilot-only, synchronous, no tmux, no `--add-dir` plumbing. Inline shell in SKILL.md rather than a new script unless complexity demands it. **Tradeoff acknowledged**: all-tier activation produces a Light-tier resource inversion — a trivial PR's Light review now consumes ~25k Copilot tokens + 1 Premium request, more *external* cost than yesterday's Standard. The cost-evaluation follow-up issue (Step 7) is the data path to revisit, but the asymmetry is named here rather than papered over. |
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
| `review-code` specialist set | Framework adapters that reference "Adversarial Specialist is Claude-only" | Yes — Files-to-Change row added during implementation for `.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md`. The original plan said "no specialist-list enumeration found"; that was wrong — three adapters carried the stale phrasing |
| Cross-model "not adopted" / "Claude-only" prose in knowledge docs | `review_depth_classification.md`, `skill_workflows.md`, `inspiration_agent_workspace_digest.md` (line ~62 in "Ported" section) | Yes — listed in Files to Change + Step 5a/5b |
| Default-on Premium-request consumption | Cost-evaluation follow-up issue | Yes — Step 7 |
| Upstream `cross_model_review.sh` invocation suspected broken | Issue filed on `rolker/agent_workspace` | Yes — Step 6 |

## Open Questions

None remaining. The four issue-comment acceptance answers settled the
top-level decisions; review-plan (PR #464, 2026-05-18) surfaced two
derived questions that the original four didn't directly address:

- **Light-tier prompt body** — resolved 2026-05-18: reuse the Standard
  prompt body (simplest one-prompt-per-tier-pair shape). Documented in
  Step 1.
- **Light-tier Premium-request asymmetry** — acknowledged as an
  intentional tradeoff in the "Only what's needed" self-check row;
  data-driven revisit deferred to the Step 7 follow-up issue.

## Estimated Scope

Single PR. Four-plus knowledge/skill file edits, three framework-adapter
edits, and two new issues filed (one upstream, one follow-up). Inline
shell grew past the original ~15-line threshold during post-review
hardening (timeout + post-invocation guard + untrusted-PR gate) but
is kept inline; see the note under "Files to Change" and Implementation
Notes for rationale.

## Implementation Notes

- **Framework adapters carried stale "Claude-only" phrasing.** Plan-time
  workspace grep was scoped narrowly; in-flight grep
  (`grep -rln -E "cross-model|cross_model|Claude.{0,5}only|..."`) found
  three additional consumers: `.github/copilot-instructions.md`,
  `.agent/instructions/gemini-cli.instructions.md`, and
  `.agent/AGENT_ONBOARDING.md`. All three said "Adversarial Specialist
  is Claude-only" — actively wrong after this PR. Folded into the same
  knowledge-doc commit since the prose change is identical
  (rename to "Claude Adversarial Specialist", add Copilot Adversarial
  as framework-agnostic).

- **Untrusted-PR safety gate added during post-PR review pass.**
  The original Step 2 only covered availability detection (binary
  missing / auth failure). Post-PR review of PR #464 surfaced that
  `--allow-all-tools` + post-PR mode on contributor diffs grants
  Copilot file/shell access to prompt content that originated outside
  the repo's trust boundary. User chose the "explicit confirmation
  gate" option: external PRs (fork head OR author association below
  COLLABORATOR) route to the skipped path unless
  `--allow-untrusted-copilot` is passed. Implemented as a new step 2a
  in 5e between the availability probe and the invocation. The flag
  is post-PR only (an error if passed in pre-push, where the gate
  doesn't apply).

- **Post-invocation guard formalized.** Step 2's original prose said
  "post-call empty findings or auth-text routes to skipped-notice
  path" but the snippet didn't actually enforce that. Post-PR review
  flagged this as a discrepancy between the documented contract and
  the shown code. The hardened snippet now includes explicit checks
  for timeout (exit 124), non-zero exit, empty findings file, and
  auth-error grep patterns — each routes to the skipped path with a
  specific reason. Plus a `timeout 300` wrapper so a hung CLI cannot
  block the whole review (field-mode-relevant when connectivity is
  flaky).

- **Light-tier zero-specialist predicate corrected.** With 5e firing
  at Light, the prior "Light + --skip-static = zero specialists"
  predicate became conditional on Copilot also being suppressed. Both
  the Step 5a comment and the Step 7 report-format guidance updated
  to reflect the new precondition.
