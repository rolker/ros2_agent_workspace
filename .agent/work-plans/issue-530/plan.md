# Plan: v1 `/wrap-up-deployment` skill — phase [3] orchestrator (deployment-mode gap)

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/530

## Context

`/start-deployment` (phase [0]) shipped in #501. Phase [3] (wrap-up) has no
tooling — operators have been executing the wrap-up workflow manually, with 4
validated end-to-end runs accumulating a clear 7-step procedure. This skill
codifies that procedure as a workspace-level SKILL.md, keeping project-specific
details (log paths, issue-sync commands, bag paths) in `.agents/deployment.yaml`
per ADR-0003.

The issue review resolved one pre-implementation decision: the skill does **NOT**
write a `progress.md` entry — it is a deployment-lifecycle skill (writes the
deployment dev log + closes the deployment GitHub issue), not a per-issue
development-pipeline step. ADR-0013 governs the development pipeline only.

## Approach

1. **Create `.claude/skills/wrap-up-deployment/SKILL.md`** — the main
   deliverable. Follows the same structure as `start-deployment/SKILL.md`:
   frontmatter → overview → lifecycle position → steps → guidelines → references.

   Steps the skill documents (phase [3] from `deployment_mode.md`):
   1. Read project config from `.agents/deployment.yaml`
   2. Verify dev-side (wrap-up must run on dev — `gh` access required for closing the issue)
   3. Find the open deployment issue via `gh issue list --label deployment`
   4. Collect all per-host log files for this deployment (match on `<start-date>_*_logs.md`)
   5. Import field changes via `/import-field-changes` (each field host's repo)
   6. Review and merge field-import PRs (standard review-loop flow)
   7. Consolidate — add a `## Wrap-up` summary to the dev log; stamp the
      deployment issue body with a summary (bag references, outcomes, follow-up
      issue links); `dlog.sh` for all entries
   8. File deferred analysis sub-issues from deployment logs (items marked
      "deferred to wrap-up" during live ops)
   9. Close the deployment issue: `gh issue close <N> --comment "Wrap-up complete."`

   ADR-0003 compliance: SKILL.md must not embed platform-specific paths, host
   names, or bag-extraction commands — these come from `.agents/deployment.yaml`.
   The skill reads the `log_dir` key and `issue_sync.dev_push` as needed;
   bag-extraction specifics are out of scope for v1.

2. **Update `AGENTS.md` lines 254-255** — replace the placeholder "The wrap-up
   half (`/wrap-up-deployment`) and recovery checklist are tracked under umbrella
   #495" with a sentence noting the skill location and that recovery checklist
   (#496) remains a follow-up.

3. **Update skill lists in all three non-Claude adapter files** (ADR-0006):
   - `.github/copilot-instructions.md` — append `wrap-up-deployment` to the
     `Available workflow skills:` line (currently ends with `start-deployment`)
   - `.agent/instructions/gemini-cli.instructions.md` — same edit
   - `.agent/AGENT_ONBOARDING.md` — same edit

4. **Run `make generate-skills`** to register `/wrap-up-deployment` as a Claude
   Code slash command (CLAUDE.md § Makefile targets).

5. **Run `/review-code` (pre-push)** before pushing — catches ADR-0003 drift,
   plan alignment, and static issues while they're cheap to fix locally.

## Files to Change

| File | Change |
|------|--------|
| `.claude/skills/wrap-up-deployment/SKILL.md` | **Create** — new skill file |
| `AGENTS.md` | Update "Deployment mode" section — replace `#495` placeholder with skill pointer |
| `.github/copilot-instructions.md` | Add `wrap-up-deployment` to skill list |
| `.agent/instructions/gemini-cli.instructions.md` | Add `wrap-up-deployment` to skill list |
| `.agent/AGENT_ONBOARDING.md` | Add `wrap-up-deployment` to skill list |

Plus: run `make generate-skills` (modifies auto-generated skill files; exact
output determined at runtime).

## Principles Self-Check

| Principle | Consideration |
|---|---|
| A change includes its consequences | All 4 adapter files and `make generate-skills` are in scope — no stale skill lists left behind |
| Workspace vs. project separation | SKILL.md must reference `.agents/deployment.yaml` for every project-specific value (ADR-0003); the review check explicitly covers this |
| Verify documentation claims against source code | SKILL.md step descriptions verified against `deployment_mode.md` phase [3] and the `start-deployment` precedent |
| Never document from assumptions | `log_dir` and `issue_sync` keys verified against `deployment_config.yaml` template before referencing them |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0002 — Worktree isolation | Yes | Already in worktree `feature/issue-530` |
| ADR-0003 — Project-agnostic workspace | Yes (key) | SKILL.md reads `log_dir`, `packages`, `layer`, `issue_sync` from `.agents/deployment.yaml`; no platform-specific content embedded |
| ADR-0006 — Shared AGENTS.md | Yes | AGENTS.md update + all 3 non-Claude adapters updated in the same PR |
| ADR-0013 — `progress.md` vocabulary | Watch | Skill does NOT write a `progress.md` entry (resolved; see Context above) |
| ADR-0014 — Deployment mode | Yes (key) | Skill is explicitly phase [3]; urgency-contract relaxation at wrap-up is stated in the skill's overview; wording consistent with SKILL.md's own contract in `start-deployment` |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Add new skill directory | `make generate-skills` to register slash command | Yes — step 4 |
| Add new skill | All non-Claude adapter skill lists | Yes — step 3 |
| AGENTS.md "Deployment mode" section | Same section in `deployment_mode.md` knowledge doc (if text duplicates the same claim) | No — knowledge doc accurately says wrap-up is a follow-up; no drift after this change |
| Wrap-up ships | ADR-0014 Status "Proposed" → "Accepted" candidate | No — leave as separate follow-up issue (the recovery checklist #496 and `/next-deployment` are still outstanding) |

## Open Questions

- Should `.agents/deployment.yaml` gain a `wrap_up:` section for project-specific
  steps (e.g., bag extraction commands, analysis script paths)? V1 may reasonably
  defer bag extraction to a manual operator step (out of scope per ADR-0003 unless
  there is a generic hook). Decision: treat bag extraction as out-of-scope for v1;
  skill documents that project-specific post-wrap steps (analysis, bag DB) are
  operator-driven from the debrief checklist (#435). This avoids speculative config
  schema before any project has configured it.

## Estimated Scope

Single PR — all changes are in workspace infrastructure files, no ROS package
code. Estimated 5–7 files (SKILL.md + AGENTS.md + 3 adapters + generated
skill files from `make generate-skills`).
