# Plan: Refactor Makefile with stamp-file dependencies and consolidate scripts

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/332

## Context

The Makefile has 19+ independent targets with no dependency graph. Setup requires
knowing the correct command order. Several status/info scripts overlap. ADR-0007
decided to keep Make and use stamp-file dependencies. Target taxonomy agreed in #335.

## Open Questions — Resolved

1. **Stamp invalidation**: Simple touch — no hash-based tracking. Users run
   `make clean` to reset if scripts change. Keeps it simple (ADR-0007 principle fit).
2. **Dashboard quick mode**: `make dashboard QUICK=1` translates to
   `dashboard.sh --quick` — single interface, Make variable passed as CLI flag.
3. **Manifest symlink guard**: Yes — if stamp exists but `configs/manifest` symlink
   is missing, invalidate the stamp (one Make conditional).
4. **Unused scripts**: Remove `checkout_default_branch.sh`, `update_issue_plan.sh`,
   `start_issue_work.sh` (all superseded by worktree workflow).
5. **Non-interactive detection**: Treat `CI` or `NONINTERACTIVE` as non-interactive
   when set to any non-empty value. Document `NONINTERACTIVE` in `AGENTS.md` as the
   workspace-wide "no prompts" opt-out.

## Approach

### Commit 1: Stamp-file Makefile + target cleanup

1. **Add `.make/` to `.gitignore`**
2. **Rewrite `Makefile`** with stamp-file dependency chain:
   - `STAMP` resolves to the **main workspace root** (not the current directory),
     so `make build` in a worktree reuses the main tree's stamps rather than
     re-running bootstrap/setup. Use the same root-detection pattern as
     `setup.bash` (check for `.workspace-worktrees/` or `layers/worktrees/`
     in the path and resolve up).
   - `$(STAMP)/bootstrap.done` — runs `bootstrap.sh`, touches stamp
   - `$(STAMP)/setup-dev.done` — depends on bootstrap stamp; runs venv +
     pre-commit install; touches stamp
   - `$(STAMP)/manifest.done` — depends on bootstrap stamp; runs
     `setup_layers.sh` with no layer argument (the script calls
     `bootstrap_manifest_repo` first, then proceeds to set up all layers).
     Since we can't run manifest-only with the current script, this stamp
     represents "manifest bootstrapped + all layers set up from
     `layers.txt`". Guard: if `configs/manifest` symlink is missing, delete
     stamp and re-run.
   - `$(STAMP)/layer-%.done` — depends on manifest stamp +
     `configs/manifest/repos/%.repos`; runs `setup_layers.sh %` to re-import
     only the changed layer; touches stamp
   - `build` — depends on all layer stamps; runs `build.sh`
   - `test` — depends on `build`; runs `test.sh`
   - `lint` — depends on setup-dev stamp; runs `pre-commit run --all-files`
   - `clean` — removes `build/install/log` artifacts AND `.make/` directory
   - `setup-all` — depends on all layer stamps (convenience, no build)
   - `help` — self-documenting (updated target list)
3. **Remove targets**: `setup-core`, `setup-underlay`, `setup-platforms`,
   `setup-sensors`, `setup-simulation`, `setup-ui`, `format`, `status`,
   `status-quick`, `health-check`
4. **Keep unchanged**: `sync`, `validate`, `lock`, `unlock`, `revert-feature`,
   `pr-triage`, `generate-skills`, `agent-build`, `agent-run`, `agent-shell`,
   `push-gateway`
5. **Bootstrap guard**: If bootstrap stamp doesn't exist, print a message
   explaining what's about to happen (system packages install) before
   proceeding. Skip the prompt when running non-interactively: treat the
   environment as non-interactive if either `CI` is set to any non-empty
   value or `NONINTERACTIVE` is set to any non-empty value. Full interactive
   first-run prompt is #330's scope.

### Commit 2: Dashboard script consolidation

1. **Create `dashboard.sh`** — merges functionality from `status_report.sh` and
   `health_check.sh`:
   - Default mode: sync + repo status + worktree status + PR/issue status +
     health checks + latest test results
   - `--quick` flag: skip sync and GitHub API calls
   - Worktree-aware (same detection pattern as existing scripts)
2. **Add `dashboard` target** to Makefile:
   ```makefile
   dashboard:
   ifdef QUICK
   	@./.agent/scripts/dashboard.sh --quick
   else
   	@./.agent/scripts/dashboard.sh
   endif
   ```
3. **Keep `status_report.sh` and `health_check.sh`** for now — mark with
   deprecation comments pointing to `dashboard.sh`. Remove in a follow-up
   once `dashboard` is validated.

### Commit 3: Remove superseded scripts + update all references

1. **Delete** `checkout_default_branch.sh` (conflicts with ADR-0002)
2. **Delete** `update_issue_plan.sh` (superseded by worktree workflow)
3. **Delete** `start_issue_work.sh` (superseded by `worktree_create.sh` +
   `worktree_enter.sh`)
4. **Update all docs referencing deleted scripts**:
   - `.agent/work-plans/README.md` — remove references
   - `.agent/WORKTREE_GUIDE.md` — remove/update references
   - `.agent/WORKFORCE_PROTOCOL.md` — remove/update references
   - `.agent/scripts/README.md` — remove entries
   - `.agent/scripts/worktree_enter.sh` — remove from help text

### Commit 4: Documentation + slash commands

1. **Update `AGENTS.md`**:
   - Script reference table: remove deleted scripts, add `dashboard.sh`,
     note `status_report.sh` deprecation
   - Build & Test section: document that `make build` handles setup
     automatically
   - Document `NONINTERACTIVE` env var in Boundaries or a new Environment
     Variables section
2. **Run `make generate-skills`** — auto-removes orphaned `/make_status`,
   `/make_status_quick`, `/make_format`, `/make_health_check`,
   `/make_setup_core`, etc. Auto-creates `/make_dashboard`.

## Files to Change

| File | Change |
|------|--------|
| `Makefile` | Rewrite with stamp-file dependencies, remove/add targets |
| `.gitignore` | Add `.make/` |
| `.agent/scripts/dashboard.sh` | New — merged status + health check |
| `.agent/scripts/checkout_default_branch.sh` | Delete |
| `.agent/scripts/update_issue_plan.sh` | Delete |
| `.agent/scripts/start_issue_work.sh` | Delete |
| `.agent/scripts/status_report.sh` | Add deprecation comment |
| `.agent/scripts/health_check.sh` | Add deprecation comment |
| `.agent/scripts/worktree_enter.sh` | Remove deleted script references from help |
| `.agent/work-plans/README.md` | Remove deleted script references |
| `.agent/WORKTREE_GUIDE.md` | Remove/update deleted script references |
| `.agent/WORKFORCE_PROTOCOL.md` | Remove/update deleted script references |
| `.agent/scripts/README.md` | Remove deleted script entries |
| `AGENTS.md` | Update script table, Build & Test, add `NONINTERACTIVE` docs |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Enforcement over documentation | Stamp dependencies enforce setup ordering mechanically |
| Only what's needed | Net reduction: 19+ targets → ~14; 3 dead scripts removed |
| Improve incrementally | 4 atomic commits; deprecated scripts kept until dashboard validated |
| A change includes its consequences | All docs referencing deleted scripts updated in same PR |
| Human control and transparency | Bootstrap guard warns before apt install on fresh clone |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0007 — Retain Make | Yes | This is the implementation |
| 0003 — Project-agnostic | Trivially | All changes are generic workspace infra |
| 0002 — Worktree isolation | Yes | Work done in worktree |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Scripts in `.agent/scripts/` | Script reference table in `AGENTS.md` | Yes (commit 4) |
| Scripts in `.agent/scripts/` | `Makefile` targets | Yes (commit 1) |
| Scripts in `.agent/scripts/` | All docs referencing them | Yes (commit 3) |
| Framework skills | Regenerate skills | Yes (commit 4) |

## Estimated Scope

Single PR, 4 atomic commits. The dashboard script is the largest piece of new
code (~200-300 lines merging two existing scripts).
