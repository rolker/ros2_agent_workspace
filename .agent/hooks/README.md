# Git Hooks

This directory contains custom git hooks for the ROS2 Agent Workspace.

## Available Hooks

### `post-checkout`

**Purpose**: Automatically symlink `layers/` directory to workspace worktrees for full functionality.

**What it does**:
- Detects when a workspace worktree is created (`.workspace-worktrees/issue-N/`)
- Automatically creates symlink: `layers -> ../../layers`
- Enables `make sync`, `make build`, and other layer-dependent commands in worktrees
- Skips if symlink already exists or if not a workspace worktree

**When it runs**: After git checkout, git worktree add, or git clone

**Exit codes**:
- `0` - Success or skipped (not a workspace worktree)
- `1` - Error creating symlink

**Installation**: Symlinked from `.git/hooks/post-checkout` to `.agent/hooks/post-checkout`

**Related**: Issue #142, WORKTREE_GUIDE.md

---

### `check-source-artifacts.py`

**Purpose**: Prevent accidental commits of temporary artifacts in source directories.

**What it does**:
- Scans for suspicious file patterns (`.tmp`, `.temp`, `.bak`, `.log`, etc.)
- Warns if such files are found in `layers/*/src/` directories
- Helps maintain clean source repositories

**When it runs**: Before every commit (via pre-commit framework)

**Exit codes**:
- `0` - No suspicious files found
- `1` - Warning issued (files found but commit not blocked)

**Configuration**: See `.pre-commit-config.yaml`

---

### `check-branch-updates.py`

**Purpose**: Check if the default branch has new commits before committing to a feature branch.

**What it does**:
- Fetches latest commits from default branch (e.g., `main`)
- Compares current feature branch against it
- Shows how many commits behind/ahead
- Provides merge vs rebase recommendations
- Displays recent commits you're missing

**When it runs**: Before every commit (via pre-commit framework)

**Exit codes**:
- `0` - Always (pre-commit hook wrapper is non-blocking; informational only)

The underlying script (`.agent/scripts/check_branch_updates.sh`) provides the actual check and recommendations, but the pre-commit hook wrapper always returns 0 to avoid blocking commits. Use the script directly with `--strict` mode if you want to enforce branch updates.

**Configuration**: See `.pre-commit-config.yaml`

**Related**:
- Script: `.agent/scripts/check_branch_updates.sh`
- Workflow: `.agent/workflows/ops/check-branch-updates.md`
- Can be run manually with `--strict` mode to block commits

**Example output**:
```
🔍 Checking for updates in default branch (main)...
⚠️  Default branch has new commits!
  Current branch:  feature/my-feature
  Default branch:  main
  Commits behind:  3
  Commits ahead:   2

📋 Recommendations:
  1️⃣  MERGE: git merge origin/main
  2️⃣  REBASE: git rebase origin/main
```

---

## Installing Hooks

Pre-commit hooks are automatically configured via the `.pre-commit-config.yaml` file.

### Initial Setup

```bash
# Install pre-commit framework (one-time)
pip install pre-commit

# Install the git hooks (one-time per clone)
pre-commit install
```

### Running Manually

```bash
# Run all hooks on all files
pre-commit run --all-files

# Run a specific hook
pre-commit run check-branch-updates
pre-commit run check-source-artifacts

# Run hooks on staged files only (default before commit)
pre-commit run
```

### Bypassing Hooks

**⚠️ Not recommended**, but if needed:

```bash
# Skip all hooks for a commit
git commit --no-verify -m "message"

# Skip specific hook via SKIP environment variable
SKIP=check-branch-updates git commit -m "message"
```

## Creating New Hooks

To add a new custom hook:

1. **Create the hook script** in this directory (`.agent/hooks/`)
   - Make it executable: `chmod +x your-hook.py`
   - Use appropriate shebang: `#!/usr/bin/env python3`
   - Return appropriate exit codes (0 = success, non-zero = failure)

2. **Add to `.pre-commit-config.yaml`** under the `local` repo:
   ```yaml
   - repo: local
     hooks:
       - id: your-hook-id
         name: Your Hook Name
         entry: python3 .agent/hooks/your-hook.py
         language: system
         pass_filenames: false  # if hook doesn't need file list
         stages: [pre-commit]
         always_run: true  # if hook should run even when no files changed
   ```

3. **Document it** in this README

4. **Test it**:
   ```bash
   pre-commit run your-hook-id --all-files
   ```

## Hook Development Guidelines

- **Keep hooks fast**: Hooks run before every commit
- **Provide clear output**: Users should understand what went wrong
- **Fail gracefully**: Don't crash on unexpected input
- **Return correct exit codes**: 0 for success, non-zero for failure
- **Make hooks optional when possible**: Warn instead of block unless critical
- **Document behavior**: Update this README when adding hooks

## Troubleshooting

### "Hook failed" but I don't know why

```bash
# Run with verbose output
pre-commit run --verbose --all-files
```

### "Hook not found" or "command not found"

```bash
# Reinstall hooks
pre-commit clean
pre-commit install
```

### Hooks are slow

```bash
# Check which hooks are taking time
time pre-commit run --all-files

# Skip expensive hooks during development
SKIP=pylint,black git commit -m "WIP"
```

### Update hook dependencies

```bash
# Update pre-commit hook repos to latest versions
pre-commit autoupdate
```

## See Also

- [`.pre-commit-config.yaml`](../../.pre-commit-config.yaml) - Hook configuration
- [`.agent/workflows/dev/submit-pr.md`](../workflows/dev/submit-pr.md) - PR workflow
- [`.agent/rules/common/git-hygiene.md`](../rules/common/git-hygiene.md) - Git best practices
- [Pre-commit framework documentation](https://pre-commit.com/)
