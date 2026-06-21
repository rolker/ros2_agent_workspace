---
issue: 553
---

# Issue #553 — Self-contained checkpoints in run-issue

## Issue Review
**Status**: complete
**When**: 2026-06-21 14:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #553
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | This change directly strengthens operator transparency — checkpoints become self-contained dialogs that don't require scrollback to adjudicate |
| Only what's needed | OK | Single targeted edit to the Checkpoints section of run-issue/SKILL.md; no new mechanism, just improved guidance text |
| Improve incrementally | OK | Small, focused doc change; constraint in the issue explicitly forbids scope creep into the decision table or checkpoint set |
| A change includes its consequences | Watch | Pure guidance doc — no automated tests apply. The acceptance criteria ("dialog readable in isolation") is a human-observable property; verify with a live run-issue trace if available |
| Enforcement over documentation | OK | This is a doc refinement, not a new rule requiring enforcement |
| Capture decisions | OK | No new design decision warranting an ADR; the change refines existing behavior |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0001 (Adopt ADRs) | No | Behavioral refinement to existing skill, not a new design decision |
| ADR-0002 (Worktree isolation) | No | Already in worktree |
| ADR-0006 (Shared AGENTS.md) | Watch | Consequences map: changing a framework skill → check adapter files. The checkpoint-format change doesn't add/remove skills or change the skill list, so no adapter update is needed; the other adapters don't describe checkpoint format. Still worth a quick scan. |
| ADR-0013 (progress.md vocabulary) | No | No new entry type; change is to checkpoint prose in SKILL.md |

### Consequences

- **No cross-file updates required.** The Checkpoints section change is self-contained. The skill description in AGENTS.md and other adapter files reference `run-issue` only at a high level; they don't describe checkpoint format.
- **Acceptance is self-verifying**: a host following the updated guidance produces a checkpoint dialog that reads correctly in isolation — no new tests to write.

### Recommendations

- The two concrete requirements (re-orientation header; finding embedded in the dialog) are clear enough that implementation can proceed directly — no further scoping needed.
- The "no change to decision table or checkpoint set" constraint in the issue is a useful guardrail; plan-task should explicitly note it to prevent scope creep during implementation.

### Actions
- [ ] No actions — issue is plan-task-ready.

## Plan Authored
**Status**: complete
**When**: 2026-06-21 17:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-553/plan.md` at `78f4c63`
**Branch**: feature/issue-553 at `78f4c63`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.

## Plan Review
**Status**: complete
**When**: 2026-06-21 16:04 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-553/plan.md` at `78f4c63`
**PR**: PR-less (--issue mode)
**Verdict**: approve-with-suggestions

### Findings
- [ ] (suggestion) Stale line reference: Checkpoints section is at lines 242–256, not 244–253 (Publish step begins at 258) — `plan.md:22`
- [ ] (suggestion) Carry into SKILL.md prose that the re-orientation header rides in the `AskUserQuestion` `question` field, not the 12-char `header` chip — `plan.md:30`

<!-- Independence: Plan Authored by "Claude Code Agent (Claude Sonnet)"; this review is a fresh-context dispatch on Claude Opus, not an in-context author self-review. Agent-name match is a false positive of the generic shared name, so no self-review annotation applied. -->

## Implementation
**Status**: complete
**When**: 2026-06-21 18:00 +00:00
**By**: Claude Opus

**Branch**: feature/issue-553 at `c2973e1`
**Files**: `.claude/skills/run-issue/SKILL.md`

Expanded the Checkpoints section of `run-issue/SKILL.md` (verified live at lines
242–256, Publish step at 258 — matching the review-plan correction) with two new
requirements, inserted after the intro line and the 4-item checkpoint list:

- **Re-orientation header** — every `AskUserQuestion` opens its `question`-field
  text with `Issue #N: <title> — phase X of Y; <one-line state>`. Guidance states
  explicitly that this rides in the `question` field, **not** the ~12-char
  `header` chip.
- **Finding-embedding** — when a checkpoint is triggered by a `## Plan Review`,
  `## Local Review (Pre-Push)`, or `## Integrated Review` finding, the finding's
  severity + condensed text must appear verbatim in the `question` field or an
  option `description`; labels stay short, the *why* rides in the description.

Added a worked example (sample `question` string + two labelled options) making
both concrete, and stated both apply to all four mandatory checkpoints.

Collateral-change check passed: `git diff` shows only a 33-line insertion within
the Checkpoints section — the decision table, the 4-item checkpoint list, the
Publish step, and all other sections are untouched. Pre-commit passed (no
`--no-verify`). Committed as `c2973e1`. Per task scope: no push, no PR.
