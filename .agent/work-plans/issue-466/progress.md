---
issue: 466
---

# Issue #466 — Persist review-issue and review-plan output to progress.md

## Issue Review
**Status**: complete
**When**: 2026-06-21 05:08 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #466
**Comment**: (not posted — GitHub posting is optional/best-effort per local-first contract #532)
**Scope verdict**: well-scoped

### Scope Assessment

**Well-scoped?** Yes — the persistence half is already shipped (both `review-issue` and
`review-plan` write typed `progress.md` entries). The remaining work is narrowly defined:
(1) make `review-issue` report `complete` (not `partial`) when the only unmet step is the
optional GitHub post, and (2) verify `dispatch_subagent.sh` does not surface a locally-
complete phase as FAILED solely because the post was skipped. Both fit a single PR.

**Right repo?** Yes — workspace repo. Changes are to `.claude/skills/review-issue/SKILL.md`
and `.agent/scripts/dispatch_subagent.sh` (verification/documentation; may not need a code
change). No project-repo changes required.

**Dependencies**: The issue body confirms the persistence half is shipped (observed driving #550).
No blocking open issues identified. Issue #532 (local-first posting contract) is referenced
as context, not a blocker.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Silently skipping an optional post and recording the skip in `progress.md` keeps the record honest; no hidden state change |
| Enforcement over documentation | OK | Skill instructions are the appropriate enforcement layer here; behavior fix in SKILL.md is sufficient |
| Capture decisions, not just implementations | Watch | The "posting is best-effort/optional" decision should be explicit in SKILL.md — not just implied by the fix. Issue text calls for skill docs update, which covers this. |
| A change includes its consequences | Action needed | SKILL.md must be updated; `dispatch_subagent.sh` behavior should be verified and documented; AGENTS.md/CLAUDE.md need no change (issue text confirms they don't reference per-skill persistence detail) |
| Only what's needed | OK | Minimal change: update the complete/partial/failed logic in the skill and SKILL.md; no scope creep needed |
| Improve incrementally | OK | Persistence half already shipped; this is a clean targeted follow-up |
| Workspace vs. project separation | OK | Workspace skills and scripts only |
| Workspace improvements cascade to projects | OK | All container-dispatched phases benefit from the corrected posting behavior |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0002 — Worktree isolation | Yes | Work proceeds in the existing `issue-workspace-466` worktree |
| ADR-0004 — Enforcement hierarchy | Watch | The optional-posting rule lives only in skill instructions; no hook/CI enforcement possible for skill behavior, so docs-only is acceptable here |
| ADR-0006 — Shared AGENTS.md | No | Issue text explicitly confirms AGENTS.md/CLAUDE.md references are unchanged |
| ADR-0013 — progress.md vocabulary | Yes | Entry type `## Issue Review` with `complete` status is per ADR-0013. The fix must ensure the entry is committed *before* the optional posting attempt, so the timeline is correct even if posting is skipped |

### Consequences

Per the consequences map:
- Changing a framework skill → framework adapter file may need updating if it references posting behavior; issue text says no AGENTS.md/CLAUDE.md change needed
- If `dispatch_subagent.sh` changes → update AGENTS.md script reference table (§ Script Reference)
- The posting-as-optional change must be recorded in SKILL.md so the intent survives future reads

### Actions
- [ ] Update `review-issue` SKILL.md: document GitHub posting as best-effort/optional; when no gh auth, skip silently and report `complete`
- [ ] Ensure the `progress.md` write + commit happens *before* the optional posting attempt (so entry is always present regardless of posting outcome)
- [ ] Verify `dispatch_subagent.sh`: confirm that a `complete` progress.md entry from the skill results in exit 0 (no code change likely needed — the current logic already maps `complete → exit 0`; add a comment if helpful)
- [ ] If `dispatch_subagent.sh` is changed, update AGENTS.md script reference table entry for `dispatch_subagent.sh`
- [ ] Skill docs updated per acceptance criteria

## Plan Authored
**Status**: complete
**When**: 2026-06-21 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-466/plan.md`
**Scope**: Post-side only — `review-issue` SKILL.md posting behavior; `review-plan` has no posting step (no change needed); `dispatch_subagent.sh` already maps `complete → exit 0` (no code change needed).

### Findings
- [ ] Plan looks solid. Ready for implementation.

## Plan Review
**Status**: complete
**When**: 2026-06-21 05:25 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-466/plan.md` at `abfc8bc`
**PR**: PR-less (`--issue` mode; no gh auth in container)
**Verdict**: approve-with-suggestions

### Findings
- [ ] (must-fix) Reorder makes persistence run first, so the GitHub comment URL is unknown when the `## Issue Review` entry is written — change #3's "When posting succeeded: `**Comment**: <URL>`" is unobtainable at write time. Specify placeholder-then-amend+recommit, or drop the on-success-URL case. — `plan.md:70`
- [ ] (suggestion) "Swap steps 7↔8" must also fix stale internal cross-refs that assume posting precedes persistence: "After posting the comment, append…" (`SKILL.md:161`) and `**Comment**: <URL … from step 7>` (`SKILL.md:305`). — `plan.md:36`
- [ ] (suggestion) Bug-#2 framing: current SKILL.md never instructs self-marking `partial`; step 8 hardcodes `complete`. Real failure mode is abort-at-step-7-before-persist (see commit `a38b523`). Align wording with the actual mechanism. — `plan.md:23`

## Implementation
**Status**: complete
**When**: 2026-06-21 05:37 +00:00
**By**: Claude Code Agent (Claude Opus)

**Issue**: #466 (post-side only; read-side + exit-contract gaps tracked in #552)

Implemented the approved revised plan (commit 87ac99e), post-side only.

### What changed
- `.claude/skills/review-issue/SKILL.md`:
  - Reordered steps: **step 7 = Persist to progress.md** (canonical record,
    written + committed first); **step 8 = Post findings as a comment**
    (best-effort, last). Renumbered sub-steps `8a`/`8b` → `7a`/`7b`.
  - Gated the post on `gh auth status --active`: skips silently with a
    `[review-issue]` note when auth is absent; phase status stays `complete`
    regardless of post outcome.
  - Dropped the inline comment-URL case from `**Comment**:` (URL unknown at
    write time now) → `(best-effort post follows this entry; not recorded
    inline)`. Single-write entry, no amend+recommit.
  - Reconciled all stale cross-refs with the new order: precondition para
    ("Steps 1 and 8", "Step 7's owning-repo probe", "step 7a.1's probe"),
    former `:161` ("After posting…" → persist-first intro), former `:305`
    (no inline URL).
  - Added a "Posting behavior" note to the Overview making the best-effort
    contract explicit (plan change #4).
- `.agent/scripts/dispatch_subagent.sh`: verified `complete → exit 0` —
  `complete` falls through the status `case` with no `exit` (lines ~483/494),
  while `failed`/`partial` map to `exit 1`. No code change (and none to
  AGENTS.md, per plan).

### Verification
- Re-grepped SKILL.md for `step N` / `8a` / `8b` / "after posting" /
  "from step 7" references — all reconciled; step numbering coherent 1–8
  with 7a–7b sub-steps.
- Pre-commit passed (no `--no-verify`). Committed as `3b70c73` with agent
  identity.

### Out of scope (not touched)
- Read-side host-injected context and false-FAILED-on-redispatch (count 1→1)
  — tracked in #552.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-21 05:48 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-466 at `d430610`
**Mode**: pre-push
**Depth**: Standard (reason: governance-touching skill doc; lifecycle-procedure change)
**Must-fix**: 2 | **Suggestions**: 2
**Round**: 1 | **Ship**: continue — two must-fix, one a genuine correctness concern in the gh-auth gate

### Findings
- [ ] (must-fix) Step rename `8a.x`→`7a.x` left 4 dangling cross-refs in sibling skill — `.claude/skills/review-plan/SKILL.md:269,291,310,320` (→ 7a.1/7a.2/7a.3/7a.5)
- [ ] (must-fix) gh-auth gate handles only no-token; read-only container `GH_TOKEN` makes `gh auth status --active` pass, then unguarded `gh issue comment` fails loudly (403) — reintroduces bad status in the #532 target env — `.claude/skills/review-issue/SKILL.md:359`
- [ ] (suggestion) `description:`/Overview still flatly say "Posts findings as a comment"; posting is now best-effort — `.claude/skills/review-issue/SKILL.md:3`
- [ ] (suggestion) Read path (step 1 `gh issue view`, 7a.1 probe) remains ungated auth dependency; "persist-first complete" guarantee only holds with a read token — `.claude/skills/review-issue/SKILL.md:26`
