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
  context from other specialists. The upstream cross-model variant (Gemini +
  Codex) is ported as step 5e — see "Partially adopted" below.
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

- **Cross-model adversarial review** (`review-code` step 5e) —
  **Gemini + Codex adopted 2026-09-24**
  ([#660](https://github.com/rolker/ros2_agent_workspace/issues/660)).
  `cross_model_review.sh` and its helpers (`_agy_review.sh`,
  `_cli_review.sh`, `_resolve_work_plans_dir.sh`,
  `_resolve_default_branch.sh`, `_plan_approach.py`) and test suite are
  ported from upstream `main` @ `48b0d82` and re-synced to `97a87fa`
  (upstream's fix for Gemini's headless tool denial, #336), using upstream's parallel-sync
  execution model (upstream ADR-0015 — the tmux mode that kept this
  unadopted is gone upstream). Default on at Standard + Deep with
  `--agents gemini,codex`, `--no-cross-model` to opt out. Local
  adaptations: codex pinned `-s read-only -a never` (upstream relies on
  CLI defaults — reported upstream), the work-plans resolver's remediation
  text matches this workspace's `worktree_enter.sh`, the header
  comments drop upstream's user-tier promotion notes, and both helpers'
  escalation watchdog resets its inherited ignored signals so it can be
  cancelled (same bug upstream).
  - **Copilot Adversarial Specialist removed** in the same change. It was
    ported in PR #464 (issue #461) as a synchronous `copilot -p ""
    --allow-all-tools` dispatch, default on, then made opt-in via
    `--copilot` ([#467](https://github.com/rolker/ros2_agent_workspace/issues/467))
    after a Premium-request billing change exhausted the monthly quota.
    Copilot CLI runs Claude/GPT models, so it added no vendor the other
    passes lack; Gemini + Codex replace it. The GitHub-side Copilot PR
    reviewer (`gh pr review --copilot`) is unaffected.
  - **Local Model Adversarial** (step 5f, Ollama via
    `.agent/scripts/local_review.sh`,
    [#570](https://github.com/rolker/ros2_agent_workspace/issues/570))
    is this workspace's own addition, originally default on, now opt-in
    via `--local`
    ([#590](https://github.com/rolker/ros2_agent_workspace/issues/590))
    — the run is the review's wall-clock long pole on current hardware.
- **What remains unadopted**: upstream's `copilot` and `claude` agents
  in `cross_model_review.sh`. The `_cli_review.sh` arms are ported
  verbatim but 5e never selects them.

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
