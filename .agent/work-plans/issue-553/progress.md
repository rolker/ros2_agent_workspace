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
