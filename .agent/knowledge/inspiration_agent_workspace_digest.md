# Inspiration Digest: agent_workspace

Type: fork
Last checked: 2026-07-14
Repo: rolker/agent_workspace @ 837c24ee64d76a4906140fd262ae9413bc262636

## Activity Snapshot

- **No code changes since the 2026-05-15 check** — HEAD is unchanged
  (`837c24e`, last commit 2026-05-12). No merged PRs, no closed issues.
  File-level diff skipped this run (would be identical to last run's).
- Upstream has been quiet since 2026-05-18. Late-May activity was all
  planning: 21 open issues, 3 open (stale) PRs.
- Workspace redesign push: #172 umbrella (multi-tenant + project-type
  adapters + per-project manifests) refined into #210 (10-verb adapter
  contract + single_project adapter) with plan PR #211 — open, unmerged
  since 2026-05-17.
- Reverse porting: upstream filed #208/#209 to port **this workspace's**
  field mode (ADR-0011, `field_mode.sh`, hotfix workflow + non-GitHub
  remote reconciliation) — confirmation of the pattern's value, nothing
  to pull back.
- #207 "Documentation enforcement: from aspiration to mechanism" — idea
  worth watching, no implementation yet.
- #206 + #212 continue the cross_model_review.sh thread (tmux-default
  reconsidered; Copilot invocation likely broken) — that script remains
  unadopted here, see "Partially adopted".

## Pending Review

_None outstanding._

## Roadmapped

_None outstanding._

## Skipped

_None outstanding._

## Ported

- **`/review-code` flag refinements** (pending since 2026-05-15) — all
  four items landed locally in the subsequent review-skill work and are
  live in `.claude/skills/review-code/SKILL.md` as of 2026-07-14:
  `--skip-static` (both modes), `--no-progress` (pre-push), `--issue <N>`
  override, and the distinct `## Local Review (Pre-Push)` progress.md
  header (also in `address-findings` and `progress_read.py`).
  (verified 2026-07-14)

Pieces imported into this workspace via PR #453 (issue #452). All adapted
for the layered/multi-repo workspace and for Claude-only operation. The
fork's items remain the upstream source if we later decide to pull more
of the surrounding tooling.

- **Review depth classification** (`review_depth_classification.md`).
  Knowledge doc with risk signals, override-trigger files, Light /
  Standard / Deep tier criteria, and user-override syntax. Adapted to
  workspace-and-project repo paths and framed **experimental** until
  thresholds are validated against real PR data.
- **Claude Adversarial Specialist** in `review-code`. Fresh-context
  subagent dispatched at Standard and Deep tiers via `Agent`, with no
  context from other specialists. A Copilot-only slice of the upstream
  cross-model variant is also ported — see "Partially adopted" below.
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

## Partially adopted

- **Copilot Adversarial Specialist** (`review-code` step 5e). Ported in
  PR #464 (issue #461). Synchronous Copilot CLI dispatch (`copilot -p ""
  --allow-all-tools < prompt`), no tmux, graceful skip when the CLI is
  missing or unauthenticated (covers field hosts gabby/salmon).
  Originally default-on at Light + Standard + Deep (opt-out via
  `--no-copilot`), but **now opt-in via `--copilot`, off by default**
  ([#467](https://github.com/rolker/ros2_agent_workspace/issues/467)
  resolved): a GitHub Copilot Premium-request billing change exhausted
  the team's monthly quota, so the per-run cost (~25.5k token floor, one
  Premium request) is no longer paid on every review. The cross-model
  second-read signal it used to provide by default is now covered by two
  in-house disjoint-lens Claude Adversarial passes (5d); `--copilot`
  adds a true second-vendor read on top when a reviewer judges it worth
  the Premium request. The old Light-tier "resource inversion" no longer
  applies, since Light no longer auto-runs Copilot.
- **What remains unadopted**: the tmux-orchestrated multi-CLI dispatch
  in `cross_model_review.sh` and the Gemini/Codex specialists. The
  tmux session machinery adds complexity beyond the highest-leverage
  subset and the workspace doesn't standardize on Gemini or Codex.
  Revisit if multi-CLI parallel review (beyond Claude + Copilot)
  becomes a workflow we actually exercise.
  - **Update 2026-05-15**: upstream extended the script with `--branch`
    mode and a `_resolve_default_branch.sh` helper (PR #185). Still not
    applicable to the unadopted slice — the script itself isn't ported,
    only the Copilot single-CLI invocation pattern was lifted.
  - **Upstream invocation report**:
    [rolker/agent_workspace#212](https://github.com/rolker/agent_workspace/issues/212)
    flags that upstream's `copilot -p < prompt` likely misses
    `-p ""` and `--allow-all-tools`; the Copilot dispatch may be
    silently broken or version-dependent. Decoupled from this
    workspace's adoption.

## Not adopted

_None outstanding._

## Deferred

All items below re-deferred 2026-07-14 (upstream unchanged since they
were filed; nothing new to weigh).

- `setup_project.sh` — generic project bootstrapping script, upstream-only (2026-03-23)
- `document-project` skill — generic documentation skill (upstream equivalent of document-package) (2026-03-23)
- 5 upstream inspiration digests — cross-pollination research from gstack, superpowers, microsoft-skills, gastown, ros2_agent_workspace (2026-03-23)
- `inspiration_registry.yml` upstream entries — upstream tracks 5 projects we could learn from (2026-03-23)
- `gh_create_issue.sh` 4-line diff — possible bug fix or improvement (2026-03-23)
- `discover_governance.sh` 18-line diff — minor improvements (2026-03-23)
- `configure_git_identity.sh` 30-line diff — minor divergence (2026-03-23)
- `skill_workflows.md` diff — may have structural improvements (2026-03-23)
- ADR-0010 title divergence — different wording for git-bug ADR (2026-03-23)
