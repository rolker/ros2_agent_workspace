# Inspiration Digest: agent_workspace

Type: fork
Last checked: 2026-05-15
Repo: rolker/agent_workspace @ 837c24ee64d76a4906140fd262ae9413bc262636

## Activity Snapshot

- 5 open issues touch the review/skills surface (notably #190 — progress.md
  convention rework + new `/implement` skill); 0 open PRs.
- 113 commits since previous snapshot (`c02887e`, 2026-04-26); 3 files in
  the review-skill area changed.
- Notable since last refresh: `/review-code` gains an explicit `--branch`
  mode with `--skip-static`, `--no-progress`, and `--issue <N>` flags
  (PR #185), plus a `_resolve_default_branch.sh` helper. Cross-model script
  extended to `--branch` mode (#185) — not applicable here, see "Not
  adopted".

## Pending Review

Stacked on PR #453 (issue #452) — the items below are upstream refinements
to the dual-mode `/review-code` we already ported in #453. All four are
small and additive on top of our existing implementation.

- **`--skip-static` flag** (both modes) — suppress the static-analysis
  specialist when pre-commit was already clean. Source: upstream
  `.claude/skills/review-code/SKILL.md` step 5a. (2026-05-15)
- **`--no-progress` flag** (branch mode) — explicit opt-out for
  progress.md persistence on skill worktrees / one-off branches that
  don't have an issue to track against. Source: upstream review-code
  SKILL steps 1b + 8. (2026-05-15)
- **`--issue <N>` override** (branch mode) — fallback when the branch
  name doesn't carry the issue number. Source: upstream review-code
  SKILL step 1b. (2026-05-15)
- **Distinct `## Local Review (Pre-Push)` vs `## Local Review`
  progress.md headers** — allows pre-push and post-PR entries for the
  same issue to coexist on the timeline. Source: upstream review-code
  SKILL step 8. (2026-05-15)

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
  - **Update 2026-05-15**: upstream extended the script with `--branch`
    mode and a `_resolve_default_branch.sh` helper (PR #185). Still not
    applicable here for the same reason — the script itself isn't ported.

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
