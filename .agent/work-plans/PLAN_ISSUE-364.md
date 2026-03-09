# Plan: UX polish — reduce root clutter and noisy script output

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/364

## Context

A human walkthrough of the workspace revealed several UX friction points:
stale files cluttering the root, decorative emoji noise in script output,
a footgun where sourcing the wrong script kills the terminal, and awkward
paths to reach scripts. The owner has confirmed: remove all 4 root files
(YAGNI), strip decorative emoji across all scripts, keep markdown links
but improve raw readability, and add a `scripts` symlink at root.

## Approach

Split into two atomic commits for clean review.

### Commit 1: Safety and output cleanup

1. **Add source-detection guards** to all executable scripts that contain
   `exit` calls. Use this pattern at the top (after the shebang/comments):
   ```bash
   if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
       echo "Error: This script should be executed, not sourced."
       echo "  Run: $0 $*"
       return 1
   fi
   ```
   Scripts to guard (25 scripts with `exit` calls — all in `.agent/scripts/`):
   `bootstrap.sh`, `setup_layers.sh`, `worktree_create.sh`,
   `worktree_remove.sh`, `worktree_list.sh`, `build.sh`, `test.sh`,
   `dashboard.sh`, `configure_git_identity.sh`, `detect_agent_identity.sh`,
   `gh_create_issue.sh`, `generate_make_skills.sh`,
   `check_branch_updates.sh`, `issue_request.sh`, `revert_feature.sh`,
   `fetch_copilot_reviews.sh`, `push_gateway.sh`, `push_request.sh`,
   `docker_run_agent.sh`, `discover_governance.sh`, `pr_status.sh`,
   `verify_change.sh`, `lock.sh`, `unlock.sh`,
   `test_identity_introspection.sh`.

   Skip scripts designed to be sourced: `setup.bash`,
   `set_git_identity_env.sh`, `worktree_enter.sh`, `detect_cli_env.sh`,
   `_worktree_helpers.sh`, `framework_config.sh`, `lib/git_helpers.sh`.

2. **Strip decorative emoji** from script output. Keep only functional
   status indicators: ✅ ❌ ⚠️ ℹ️ ⏭️
   Replace decorative emoji with plain text equivalents:
   - 🚀 → (nothing, or "Pulling")
   - 🌿 → (nothing, or "Fetching")
   - 📋 → "Issue"
   - 🔧 → (nothing)
   - 📦 → (nothing)
   - 📁 → (nothing)
   - 🔍 → (nothing)
   - 📊 → (nothing)
   - 💡 → "Tip:"
   - 📝 → (nothing)
   - 🔒/🔓 → "LOCKED"/"UNLOCKED"
   - 🟢🟡🟠🔴 → text labels or ASCII markers
   - 1️⃣ 2️⃣ → "1." "2."
   - ⬆️⬇️↕️🔀 → text ("Ahead", "Behind", etc.)
   - 📂 → (nothing)
   - 🤖 → (nothing)

   Affected files (11): `sync_repos.py`, `check_branch_updates.sh`,
   `worktree_create.sh`, `worktree_list.sh`, `dashboard.sh`,
   `pr_status.sh`, `lock.sh`, `unlock.sh`, `build.sh`, `test.sh`,
   `verify_change.sh`.

   Note: `dashboard.sh` line 402 greps for emoji (🔧📦📁) in
   `worktree_list.sh` output — this grep pattern must be updated to
   match the new plain-text output format.

### Commit 2: Root cleanup and scripts symlink

3. **Remove root files**: `CHANGELOG.md`, `CONTRIBUTING.md`,
   `SECURITY.md`, `QUICKSTART.md`.

4. **Add `scripts` symlink** at repo root → `.agent/scripts/`.
   Track it in git (not gitignored).

5. **Update references**:
   - `README.md`: Remove links to deleted files, update Documentation
     section for terminal-friendly readability (keep links, drop emoji,
     use whitespace alignment, add blank lines). Remove "Getting Help"
     section references to deleted files.
   - `.github/workflows/validate.yml` line 36: Remove
     `CONTRIBUTING.md SECURITY.md` from `required_files` check.
   - `ARCHITECTURE.md` line 228: Remove reference to CONTRIBUTING.md.
   - `.agent/scripts/PR_STATUS_README.md`: Remove CONTRIBUTING.md ref.
   - `AGENTS.md`: Update script reference paths to show shorter
     `scripts/` alternative. Note which scripts are sourced vs executed.
   - `CLAUDE.md`: Update environment setup to use `scripts/` path.

## Files to Change

| File | Change |
|------|--------|
| 25 scripts in `.agent/scripts/` | Add source-detection guard |
| 11 scripts in `.agent/scripts/` | Strip decorative emoji |
| `CHANGELOG.md` | Delete |
| `CONTRIBUTING.md` | Delete |
| `SECURITY.md` | Delete |
| `QUICKSTART.md` | Delete |
| `scripts` (new symlink) | Create → `.agent/scripts/` |
| `README.md` | Remove refs to deleted files, terminal-friendly formatting |
| `.github/workflows/validate.yml` | Remove deleted files from required_files |
| `ARCHITECTURE.md` | Remove CONTRIBUTING.md reference |
| `.agent/scripts/PR_STATUS_README.md` | Remove CONTRIBUTING.md reference |
| `AGENTS.md` | Add `scripts/` shortcut note, source vs execute guidance |
| `CLAUDE.md` | Update setup paths |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | Core motivation — removing YAGNI files and noise |
| Enforcement over documentation | Source guards enforce correct usage mechanically |
| A change includes its consequences | All downstream references updated in same PR |
| Improve incrementally | Two focused commits, each reviewable independently |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0007 — Retain Make | Watch | Symlink complements Make, doesn't replace it |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Scripts in `.agent/scripts/` | Script reference in `AGENTS.md`, Makefile targets | Yes |
| Worktree scripts | `.agent/WORKTREE_GUIDE.md`, `AGENTS.md` worktree section | Yes (source vs execute note) |
| README.md | Verify no other docs link to sections that change | Yes |
| CI workflow | Verify CI still passes after removing required_files entries | Yes |

## Open Questions

- Should `AGENTS.md` and `CLAUDE.md` switch fully to `scripts/` paths,
  or show both? Recommendation: show `scripts/` as primary, note that
  `.agent/scripts/` also works.
- The `dashboard.sh` grep for worktree_list emoji output needs careful
  testing — should verify the replacement pattern still parses correctly.

## Estimated Scope

Single PR, two commits.
