# Inspiration Digest: agent_workspace

Type: fork
Last checked: 2026-04-30
Repo: rolker/agent_workspace @ c02887e00eecfac31cb442fea47427dc08a02b5c

## Activity Snapshot

- ~14 open issues (mostly enhancements from inspiration runs), 0 open PRs
- Active development since the previous digest (2026-03-23): the fork landed
  the depth-tier review pipeline and the cross-model adversarial dispatch in
  the meantime.
- Notable since last refresh: `review_depth_classification.md` knowledge doc,
  Claude + Gemini Adversarial Specialists in `review-code`, `progress.md`
  persistence in both `review-code` and `triage-reviews`, and a flexible
  input set on `review-plan` (PR / file / `--issue <N>`).

## Pending Review

_None._ Items previously listed here have either been ported or moved to
the appropriate section below.

## Roadmapped

_None outstanding._

## Skipped

_None outstanding._

## Ported

Pieces imported into this workspace via PR #453 (issue #452). All adapted
for the layered/multi-repo workspace and for Claude-only operation. The
fork's items remain the upstream source if we later decide to pull more
of the surrounding tooling.

- **Review depth classification** (`review_depth_classification.md`).
  Knowledge doc with risk signals, override-trigger files, Light /
  Standard / Deep tier criteria, and user-override syntax. Adapted to
  workspace-and-project repo paths and framed **experimental** until
  thresholds are validated against real PR data.
- **Adversarial Specialist (Claude-only)** in `review-code`. Fresh-context
  subagent dispatched at Standard and Deep tiers via `Agent`, with no
  context from other specialists. Cross-model variant deliberately not
  ported (see "Not adopted" below).
- **`review-code` dual-mode + depth dispatch.** Pre-push mode (no arg)
  diffs against the current repo's default branch; post-PR mode
  (`<N>` or URL) diffs against the PR base. Specialists dispatched per
  tier from `review_depth_classification.md`.
- **`progress.md` persistence** in `review-code` and `triage-reviews`.
  Both skills append a step entry to
  `.agent/work-plans/issue-<N>/progress.md` in the issue's owning repo
  so findings survive across sessions.
- **`review-plan` flexible inputs.** Accepts `<pr-number>`, a path to a
  plan file, or `--issue <N>` (resolved to
  `.agent/work-plans/issue-<N>/plan.md`).

## Not adopted

- **Cross-Model Adversarial Specialist** (`.agent/scripts/cross_model_review.sh`,
  Gemini/Codex/Copilot tmux dispatch). The workspace doesn't standardize
  on multi-CLI review yet, and the tmux session machinery adds complexity
  beyond the highest-leverage subset (Claude-only adversarial). Revisit
  if multi-CLI review becomes a workflow we actually exercise.

## Deferred

- `setup_project.sh` — generic project bootstrapping script, upstream-only (2026-03-23)
- `document-project` skill — generic documentation skill (upstream equivalent of document-package) (2026-03-23)
- 5 upstream inspiration digests — cross-pollination research from gstack, superpowers, microsoft-skills, gastown, ros2_agent_workspace (2026-03-23)
- `inspiration_registry.yml` upstream entries — upstream tracks 5 projects we could learn from (2026-03-23)
- `gh_create_issue.sh` 4-line diff — possible bug fix or improvement (2026-03-23)
- `discover_governance.sh` 18-line diff — minor improvements (2026-03-23)
- `configure_git_identity.sh` 30-line diff — minor divergence (2026-03-23)
- `skill_workflows.md` diff — may have structural improvements (2026-03-23)
- ADR-0010 title divergence — different wording for git-bug ADR (2026-03-23)
