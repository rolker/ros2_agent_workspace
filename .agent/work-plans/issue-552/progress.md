---
issue: 552
---

# Issue #552 — dispatch_subagent.sh: container review lifecycle depends on in-sandbox GitHub access (read-side) + exit-contract false-FAILED on entry replace

## Issue Review
**Status**: complete
**When**: 2026-06-21 14:30 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #552
**Comment**: (best-effort post skipped — no GitHub auth in container dispatch)
**Scope verdict**: well-scoped

### Scope Assessment

Two tightly related gaps in `dispatch_subagent.sh` (workspace infra, correct repo):

- **Gap 1 (read-side)**: Container phases whose first step is `gh issue view <N>` (e.g., `review-issue`) fail outright with no GitHub read auth. Fix is host-side injection of issue/PR context into the dispatch handoff prompt — the workaround used live on #466 should become first-class in `dispatch_subagent.sh` (or `run-issue`).
- **Gap 2 (exit-contract)**: Exit contract gates on PRE→POST `entry_count` delta. A re-dispatch that *replaces* an existing typed entry (1→1) reports false-FAILED even when the sub-agent wrote a `complete` entry. Fix is gating on last matching entry's `**Status**`/`**When**`, not raw count delta.

Both fixes land in the same script area and belong in one PR. No splitting needed.

**Dependencies**: #490 (dispatch plumbing parent), #481 (run-issue orchestrator), #532 (container-produces/host-publishes contract), #466 (skill-side posting sibling), #550 (sibling). These are informational — no blocking dependency for implementation.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Both fixes make pipeline outcomes more predictable and debuggable; host-injection should be logged so the operator can see what context was injected |
| Enforcement over documentation | OK | Gap 2 fix strengthens a mechanical enforcement gate (exit contract) rather than adding docs |
| Capture decisions, not just implementations | Action needed | The "host-injects-context" dispatch contract is a new architectural decision; it should be recorded — either as a cross-reference addendum to ADR-0013 or a new ADR covering the dispatch handoff contract |
| A change includes its consequences | Action needed | `review-issue` SKILL.md step 1 documents the `gh issue view` read-path auth dependency (lines 38–44) and must be updated to describe the host-injection bypass; AGENTS.md script reference row for `dispatch_subagent.sh` must reflect any new flag (e.g. `--context-file`) |
| Only what's needed | OK | Both fixes solve observed, concrete failures from the #466 live run; no speculative scope |
| Improve incrementally | OK | Localized changes to one script + doc updates; no architectural redesign |
| Test what breaks | Action needed | Gap 2's false-FAILED scenario (re-dispatch overwrites same-type entry, count stays flat) needs a regression test alongside the fix |
| Workspace vs. project separation | OK | `dispatch_subagent.sh` is workspace infra; fix belongs here |
| Workspace improvements cascade to projects | OK | Robust dispatch benefits all container-dispatched phases in any project |
| Primary framework first, portability where free | OK | Claude Code is the primary runtime; fix is framework-agnostic dispatch plumbing |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0001 — Adopt ADRs | Yes | "Host-injects-context" is a design decision future agents need to understand; record it |
| ADR-0002 — Worktree isolation | No | No new feature work pattern introduced |
| ADR-0004 — Enforcement hierarchy | Watch | Exit-contract fix is enforcement hardening; ensure it is applied at the right layer (script gate, not just docs) |
| ADR-0005 — Layered enforcement | Watch | Same as ADR-0004 — strengthening the mechanical gate at dispatch level is the right layer |
| ADR-0013 — progress.md entry-type vocabulary | Yes | Gap 2 fix changes how `dispatch_subagent.sh` reads typed entries — it must consume `**Status**` and `**When**` per ADR-0013's schema rather than raw count; the fix must remain aligned with the consume-by-entry-type-filter rule |

### Consequences

- `dispatch_subagent.sh` change: update AGENTS.md script reference table row for this script (description, any new flags)
- If a `--context-file` / `--issue-body` parameter is added to `dispatch_subagent.sh`: update `run-issue` SKILL.md (it drives dispatch) and `WORKTREE_GUIDE.md` if it mentions dispatch options
- `review-issue` SKILL.md step 1 and "Posting behavior" section describe the read-path auth dependency; update to note the host-injection path that satisfies it without a read token
- Decision on "host-injects-context" needs a record (ADR addendum or new ADR); #532's issue body already owns the "container-produces, host-publishes" framing — this extends it to the read side
- Regression test for Gap 2: a test that runs `dispatch_subagent.sh` against a pre-populated progress.md (1 entry already present) and verifies the exit contract reports `OK` on a same-count replace with a `complete` status

### Actions
- [ ] Add host-side context injection to `dispatch_subagent.sh` (or `run-issue`) as a first-class option, replacing the hand-rolled `--prompt-file` workaround used on #466
- [ ] Strengthen exit-contract in `dispatch_subagent.sh` to gate on last matching entry's `**Status**`/`**When**` instead of (or in addition to) raw count delta
- [ ] Add regression test for false-FAILED on re-dispatch replace (Gap 2 scenario)
- [ ] Update `review-issue` SKILL.md step 1 / posting-behavior section to reflect the host-injection bypass of the read-path auth dependency
- [ ] Update AGENTS.md script reference table for `dispatch_subagent.sh`
- [ ] Record the "host-injects-context" dispatch contract decision (ADR addendum or new ADR)
