# Script & Makefile Intent Recovery (Issue #252)

Results of a script-by-script walkthrough. Each entry captures the underlying
intent/need, not just what the script does mechanically.

## Functional Clusters

### Environment & Setup

| Script | Intent | Status |
|--------|--------|--------|
| `env.sh` | Single "source this" entry point: ROS 2 base, workspace layers, checkout guard, dev tools, git identity. Everything depends on this. | Active. Name should better reflect ROS `setup.bash` convention, but `setup.sh` is already taken (layer provisioning). Needs a name that avoids collision. |
| `bootstrap.sh` | One-time system setup (install ROS 2 Jazzy + dependencies). | Active |
| `setup.sh` | Provision layers: clone manifest repo, create symlinks, `vcs import` repos into layer directories. Main onboarding mechanism. | Active |
| `health_check.sh` | "Is this workspace ready to use?" Verifies ROS 2, tools, configs, layers, hooks. | Active |

### Build & Test

| Script | Intent | Status |
|--------|--------|--------|
| `build.sh` | Orchestrate layered `colcon build` in correct dependency order. Worktree-aware. | Active |
| `test.sh` | Run `colcon test` across all layers, produce markdown/CSV/JSON reports. Worktree-aware, respects lock. | Active |
| `verify_change.sh` | Quick feedback: test a single package by name without the full suite. | Active |
| `build_report_generator.py` | Internal plumbing: parse colcon `events.log` into markdown build summary. | Active |

### Worktree Isolation (ADR-0002 implementation)

| Script | Intent | Status |
|--------|--------|--------|
| `worktree_create.sh` | Worktree factory. Workspace type (full checkout + symlinked layers) or layer type (hybrid: git worktrees for modified packages, symlinks for rest). Optional `--plan-file` creates draft PR. | Active, core workflow |
| `worktree_enter.sh` | Source to enter a worktree: cd, set env vars, source ROS 2, fetch issue title for verification. | Active, core workflow |
| `worktree_list.sh` | Inventory all active worktrees with issue/type/branch/status. | Active |
| `worktree_remove.sh` | Safe teardown: two-phase checks then cleanup. Refuses if caller is inside worktree. | Active |
| `_worktree_helpers.sh` | Shared functions for layer worktree branch detection and dirty checks. | Active |

### Identity Detection & Configuration

| Script | Intent | Status |
|--------|--------|--------|
| `set_git_identity_env.sh` | **Preferred** method: set ephemeral git identity via env vars. | Active |
| `detect_cli_env.sh` | Detect which agent framework is running via env vars. | Active |
| `detect_agent_identity.sh` | Orchestrator: detect framework then configure identity. | Active |
| `framework_config.sh` | Data: lookup tables mapping frameworks to names, emails, models. | Active |
| `configure_git_identity.sh` | Older method: writes persistent `.git/config` identity. | **Superseded** by `set_git_identity_env.sh` |
| `test_identity_introspection.sh` | Regression tests for the identity system. | Active, lives outside `tests/` inconsistently |

**Cluster note**: Five scripts doing one job (set git identity for the current
agent). Consolidation candidate.

### Container Isolation

| Script | Intent | Status |
|--------|--------|--------|
| `docker_run_agent.sh` | Host-side launcher: build image, validate worktree, generate mounts, launch container, check outbox on exit. | Active, untested in real use |
| `push_request.sh` | Container-side: queue a push+PR signal file for host processing after exit. | Active, untested |
| `issue_request.sh` | Container-side: queue an issue creation signal file for host processing after exit. | Active, untested |
| `push_gateway.sh` | Host-side: interactive processor for queued push and issue requests. Validates paths and branch names. | Active, untested |

**Cluster note**: Coherent set. Recently merged but no real-world use yet. Role
undecided (optional tool vs every workflow).

### Status & Reporting

| Script | Intent | Status |
|--------|--------|--------|
| `status_report.sh` | "What's the state of everything?" Repo sync, git status, PRs, issues, test results. Worktree-aware. | Active |
| `pr_status.sh` | PR triage dashboard: classify review comments as critical/minor, multiple output formats (human dashboard, agent JSON, "next actionable" queries). | Active, lightly tested. More useful once project work generates PRs. |
| `read_feature_status.py` | Parse issue checkboxes into machine-readable progress JSON (phase, status, percent). | Active, origin unclear (manual tracking or conductor pattern) |

### Sync & Validation

| Script | Intent | Status |
|--------|--------|--------|
| `sync_repos.py` | Safe multi-repo sync: pull on default branches, fetch-only on feature branches, skip dirty repos. | Active |
| `validate_repos.py` | Configuration linter: validate `.repos` file syntax, structure, cross-file uniqueness. | Active |
| `validate_workspace.py` | Runtime drift detector: compare what `.repos` says vs what's on disk. Optional `--fix`. | Active |

### GitHub Integration

| Script | Intent | Status |
|--------|--------|--------|
| `gh_create_issue.sh` | Pre-validation wrapper: check labels against `github_metadata.json` before calling `gh issue create`. Fail fast with helpful errors. | Active |

### Utility / Plumbing

| Script | Intent | Status |
|--------|--------|--------|
| `lib/workspace.py` | Shared Python library: parse `.repos` files, list repos, look up versions, extract GitHub slugs. | Active |
| `lib/git_helpers.sh` | Shell functions wrapping git commands that open editors. | Active, partially redundant with `GIT_EDITOR=true` from `env.sh` |
| `get_repo_info.py` | CLI wrapper around `lib/workspace.find_repo_version()`. | Active |
| `list_overlay_repos.py` | CLI wrapper around `lib/workspace.get_overlay_repos()`. | Active |
| `lock.sh` / `unlock.sh` | Advisory workspace lock. Original motivation fuzzy (precursor to worktrees, or concurrent build prevention). | Active, trust-based, not enforced |

### Claude Code Integration

| Script | Intent | Status |
|--------|--------|--------|
| `generate_make_skills.sh` | Sync Makefile targets to Claude Code `/make_*` slash commands. Makefile is the primary interface (works in any terminal); skills add tab-completion and agent interpretation of output in Claude Code. | Active |

### High-Level Entry Points

| Script | Intent | Status |
|--------|--------|--------|
| `agent` | Subcommand wrapper (`agent start-task <N>`). Only one subcommand implemented. | Active, thin wrapper |

### Deprecated / Superseded

| Script | Intent | Status |
|--------|--------|--------|
| `checkout_default_branch.sh` | Switch to default branch. | **Deprecated**: conflicts with ADR-0002 |
| `start_issue_work.sh` | Older workflow entry point with branch-based path (`git checkout -b`). Worktree path grafted on later. | **Superseded** by `worktree_create.sh` + `worktree_enter.sh` |
| `update_issue_plan.sh` | Commit changes to work plan files. Part of older branch workflow. Underlying intents (handoff to other agents, audit trail) still valid but mechanism is superseded. | **Superseded** |
| `generate_knowledge.sh` | Originally populated project knowledge via symlinks. Now a no-op stub. Intent (aggregate project-specific knowledge into a discoverable location) still needs proper design. | **No-op**, intent unresolved |
| `revert_feature.sh` | Revert all commits for an issue by number. Part of conductor pattern ideas. | **Never tested**, manual revert suffices for rare cases |
| `doc_analyzer.py` | Documentation quality scorer. References stale paths. Tested a few times early on before the workspace used its present layout. | **Likely broken**, valuable intent |

### Test Suite

| Location | Intent | Status |
|--------|--------|--------|
| `tests/` (6 files) | Script-level regression tests for: build_report_generator, detect_cli_env, generate_make_skills, pr_status, revert_feature, worktree_create. | Partial coverage. `test_identity_introspection.sh` lives outside this dir inconsistently. |

## Makefile

The Makefile is the **primary human interface** — standard `make <target>` UX
that any developer is used to. Most targets are thin wrappers around the scripts
above. Targets with their own logic:

| Target | Intent |
|--------|--------|
| `help` | Self-documenting: print available targets |
| `clean` | Remove colcon build artifacts across all layers |
| `setup-all` | Orchestrate all layer setup targets in order |
| `$(PRE_COMMIT)` | Auto-provision `.venv` with pip + pre-commit |
| `setup-dev` | Install git hooks + generate Claude Code skills |
| `format` | Run `black` via pre-commit |
| `lint` | Run all pre-commit hooks |

## Cross-Cutting Observations

- **Consolidation candidates**: Identity cluster (5 scripts to 1-2),
  `lib/git_helpers.sh` (partially redundant with `GIT_EDITOR=true`)
- **Deprecated scripts to remove**: `checkout_default_branch.sh`,
  `start_issue_work.sh`, `update_issue_plan.sh`, `configure_git_identity.sh`
- **Unresolved intents**: project knowledge aggregation
  (`generate_knowledge.sh`), documentation quality scoring (`doc_analyzer.py`),
  handoff/audit trail (`update_issue_plan.sh`)
- **Untested but potentially valuable**: container isolation cluster,
  `pr_status.sh`, `revert_feature.sh`, `read_feature_status.py`
- **Naming**: `env.sh` should better reflect ROS `setup.bash` convention, but
  `setup.sh` is already taken by the layer provisioning script — needs a name
  that avoids collision
