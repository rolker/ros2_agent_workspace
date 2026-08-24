---
issue: 607
---

# Issue #607 — Flip dispatch-mode default from container to in-process (auto mode retires #545's premise)

## Issue Review
**Status**: complete
**When**: 2026-08-24 09:26 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #607
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Fact-check against source

Verified the issue's quotations and claims against the actual files rather
than trusting them:

- **Quoted passages TRUE, verbatim.** `.claude/skills/run-issue/SKILL.md`
  (~L98-100) reads "**Lean toward `container`**… whenever keeping the
  operator **out of the approval loop** matters" and (~L92) "That prompt
  elimination is the biggest practical reason to use it." `.agent/knowledge/skill_workflows.md`
  (~L131-133) reads "**Fan-out goes to containers, not in-process agents.**
  Review/exploration fan-out via in-process Agent-tool sub-agents floods the
  operator with permission prompts; container dispatch runs sandboxed and
  prompt-free."
- **Section 4 masking `layers/main/*_ws/install`: TRUE.** `docker_run_agent.sh:369`,
  "4. Anonymous volumes for build/install/log in each layer workspace."
- **`--context-file` covers `review-issue` not `triage-reviews`: TRUE.**
  `triage-reviews/SKILL.md` calls `gh pr view` / `fetch_pr_reviews.sh` /
  `gh issue view` directly with no context-file path — matches run-issue's own
  documented limitation.
- **No host Agent tool in container: TRUE**, consistent with run-issue/SKILL.md's
  own text.
- **#581 / #558 / #585: UNVERIFIABLE locally** — no local trace (git log,
  `.agent/knowledge/`) of these issue numbers; this dispatch has no GitHub
  read auth to check them directly. Not a reason to doubt the issue, just a
  gap this review can't close.
- **Root-owned volumes fix: TRUE but mis-cited.** Only **#604** appears in
  git log for the worktree-volume-chown fix (`8789f1e fix: chown the
  dispatched worktree's anonymous volumes`, `7596d74 Shield dispatched
  worktree build artifacts from host (#602)`); **`#606` appears nowhere** in
  history. The root-owned symptom itself traces to `#566` in code comments,
  not `#604`/`#606`. Minor citation error, doesn't affect the substance of
  the claim.
- **Auto mode's existence/effect: corroborated by this very session** — the
  dispatching orchestrator's own session carries a system reminder
  "Auto Mode Active" that biases toward not stopping for approvals, matching
  the issue's premise.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Issue is explicitly framed as a governance proposal "raised for the operator rather than made unilaterally"; non-goals section keeps scope honest. |
| Enforcement over documentation | Watch | Purely advisory prose in both target files — nothing mechanically enforces the mode choice, matching how the original #545 preference was also advisory-only. Consistent, not a regression. |
| Capture decisions, not just implementations | Watch | #545 wasn't ADR'd, so no precedent requires one here either; the proposed change already plans to record "auto mode" as the reason the advice moved, which satisfies the spirit at doc level. |
| A change includes its consequences | Action needed | See below — the `run-issue/SKILL.md` heading currently reads "**Choosing a mode (#545).**"; the rewrite should update that inline citation to the new decision (#607) so a future reader doesn't find a dangling reference to the superseded issue. Also verified: no other adapter file (`.github/copilot-instructions.md`, `.agent/instructions/`) echoes this advice, so no additional cascade needed. |
| Only what's needed | OK | Non-goals section explicitly excludes touching `dispatch_subagent.sh` and doesn't propose deprecating containers. |
| Improve incrementally | OK | Single PR, two files, scoped edits. |
| Test what breaks | OK | N/A — advisory prose, no enforced logic to test. |
| Workspace vs. project separation | OK | Both target files are workspace infra (skills/knowledge), correct repo. |
| Primary framework first, portability where free | Action needed | The issue's premise (auto mode eliminates prompts) is Claude-Code-specific and further conditional on the **operator's session actually running in auto mode** — not all Claude Code sessions do. The proposed replacement text ("in-process becomes the default") should be conditioned on auto mode being active, not stated as an unconditional default; otherwise an operator in normal (non-auto) permission mode gets misled back into the exact prompt-flood problem #545 was written to solve. This is the one substantive content gap for plan-task to close. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| 0001 — Adopt ADRs | Watch, not required | This reverses operational guidance future agents rely on, which is ADR territory in spirit — but the original #545 decision was never captured as an ADR either, so requiring one now would be an inconsistent bar. Not blocking. |
| 0003 — Project-agnostic workspace | OK | Both files are generic workspace dispatch tooling, not project-specific. |
| 0015 — Dispatch handoff context contract | Not triggered | Issue doesn't touch the `--context-file` read/write mechanism itself, only the mode-selection advice around it. |

### Consequences

- Update the `(#545)` citation in `run-issue/SKILL.md`'s "Choosing a mode" heading to reference this issue once the text is replaced.
- No other files need updates — confirmed no adapter file duplicates the advice being changed.

### Actions
- [ ] Condition the new "in-process is the default" guidance on the operator's session actually running in Claude Code auto mode — state the fallback (container, or "expect prompts") for non-auto-mode sessions and non-Claude host runtimes, so the rewrite doesn't silently reintroduce the prompt-flood problem #545 solved.
- [ ] Update the inline `(#545)` issue citation in `run-issue/SKILL.md`'s "Choosing a mode" heading to point at this issue.
- [ ] Correct the citation in the issue body itself (or in the resulting doc text) from "#604 / #606" to "#604" for the root-owned-volume fix — `#606` does not appear in git history.

**Correction (from plan-task, 2026-08-24)**: the #606 action item above is
superseded — #606 is an open PR (fixing #604) created by the host after this
review ran; it is not a citation error. The plan leaves that citation as-is.

## Plan Authored
**Status**: complete
**When**: 2026-08-24 09:31 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-607/plan.md` at `8dd6e01`
**Branch**: feature/issue-607 at `8dd6e01`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.
