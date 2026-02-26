# Agent Scripts Reference

Helper scripts in `.agent/scripts/` for workspace management. These scripts can be run directly or referenced from framework instruction files.

## Setup & Initialization

### `setup.sh`

Initialize workspace layers from project configuration.

**Usage:**
```bash
./.agent/scripts/setup.sh              # Auto-setup all layers (recommended)
./.agent/scripts/setup.sh <layer_name> # Setup specific layer
```

**Examples:**
```bash
./.agent/scripts/setup.sh              # Setup all layers (underlay, core, platforms, etc.)
./.agent/scripts/setup.sh core         # Setup only core layer
./.agent/scripts/setup.sh underlay     # Setup only underlay layer
```

**What it does:**
1. Bootstraps the project repository (if needed) using `configs/project_bootstrap.url`
2. For auto-setup: Reads `config/layers.txt` from project to determine which layers to set up
3. Creates `layers/main/<layer>_ws/src` directory for each layer
4. Imports repositories from project's `config/repos/<layer>.repos` using vcs tool

**When to use:**
- First time workspace setup (just run with no arguments)
- Adding a new layer that was added to the project
- Re-importing repositories after cleaning

**Dependencies:**
- `vcstool` (`pip install vcstool` or `apt install python3-vcstool`)
- `git`
- Internet connection to clone repositories

**Outputs:**
- `layers/main/<layer>_ws/src/` directories with cloned repositories

---

### `bootstrap.sh`

Install ROS 2 Jazzy and system dependencies on Ubuntu 24.04.

**Usage:**
```bash
./.agent/scripts/bootstrap.sh
```

**What it does:**
1. Adds ROS 2 Jazzy apt repository
2. Installs ROS 2 base packages
3. Installs colcon build tools
4. Installs vcstool and other dependencies

**When to use:**
- First time setup on Ubuntu 24.04
- If ROS 2 commands are not found
- Before building workspace layers

**Dependencies:**
- Ubuntu 24.04
- sudo access

---

## Status & Reporting

### `status_report.sh` ⭐ Recommended

**Comprehensive workspace status** — repository sync, git status, GitHub PRs/issues, and test results.

**Usage:**
```bash
./.agent/scripts/status_report.sh                    # Full status (sync + GitHub)
./.agent/scripts/status_report.sh --quick            # Fast local-only (no sync, no GitHub)
./.agent/scripts/status_report.sh --skip-sync        # Skip fetch, keep GitHub queries
./.agent/scripts/status_report.sh --skip-github      # Local status only (offline)
./.agent/scripts/status_report.sh --help             # Show all flags
```

**Makefile shortcuts:**
```bash
make status        # Full status
make status-quick  # Equivalent to --quick
```

**Flags:**

| Flag | Effect |
|------|--------|
| *(none)* | Full: sync + repos + GitHub PRs/issues + tests |
| `--quick` | Alias for `--skip-sync --skip-github` |
| `--skip-sync` | Skip git fetch |
| `--skip-github` | Skip GitHub API calls |
| `--help` | Usage |

**What it shows:**
- Root repository git status
- All sub-repository statuses per workspace layer (clean vs. attention)
- Branch tracking (ahead/behind, wrong branch)
- Open GitHub Pull Requests and Issues (unless `--skip-github`)
- Latest test results (if available)

**When to use:**
- Daily morning status check (`make status`)
- Quick check before committing (`make status-quick`)

**Dependencies:**
- Required: `vcs`, `git`, `python3`, `jq`
- Optional: `gh` CLI authenticated (`gh auth login`) — for GitHub PR/issue features

For principles-aware PR review, use the `review-pr` skill instead.

---

## Building & Compilation

### `build.sh`

Build ROS 2 workspace layers with colcon.

**Usage:**
```bash
./.agent/scripts/build.sh [layer_names...]
```

**Examples:**
```bash
./.agent/scripts/build.sh              # Build all layers
./.agent/scripts/build.sh core         # Build only core layer
./.agent/scripts/build.sh core ui      # Build core and ui layers
```

**What it does:**
1. Sources the ROS 2 environment
2. Runs `colcon build --symlink-install` on each layer
3. Generates build report in `.agent/scratchpad/build_report.md`

**Dependencies:**
- ROS 2 Jazzy installed
- All workspace layers initialized (see `setup.sh`)
- System packages installed (see `bootstrap.sh`)

**Outputs:**
- Build artifacts in each `layers/<layer>_ws/build/`
- `.agent/scratchpad/build_report.md` with build summary

---

## Environment Management

### `env.sh`

Source the ROS 2 environment for all workspace layers in correct order.

**Usage:**
```bash
source ./.agent/scripts/env.sh
```

**What it does:**
1. Sources ROS 2 Jazzy system installation
2. Sources each workspace layer in order (underlay → overlay)
3. Sets up environment variables for ROS tools

**When to use:**
- Before running ROS commands (rostopic, rosservice, colcon build)
- In new terminal/shell session
- After building a new layer

**Note:** This must be sourced (not executed) to affect current shell.

---

## Git & Configuration

### `set_git_identity_env.sh` (Ephemeral - Recommended for Host-Based Agents)

Set git identity using environment variables (session-only, doesn't modify `.git/config`).

**Usage:**
```bash
source .agent/scripts/set_git_identity_env.sh "<Name>" "<email>"
```

**Examples:**
```bash
source .agent/scripts/set_git_identity_env.sh "Copilot CLI Agent" "roland+copilot-cli@ccom.unh.edu"
source .agent/scripts/set_git_identity_env.sh "Gemini CLI Agent" "roland+gemini-cli@ccom.unh.edu"
```

**What it does:**
1. Exports `GIT_AUTHOR_NAME`, `GIT_AUTHOR_EMAIL`, `GIT_COMMITTER_NAME`, `GIT_COMMITTER_EMAIL`
2. These environment variables take precedence over `.git/config`
3. Identity applies ONLY to current shell session

**Why use this:**
- ✅ Does NOT modify `.git/config` (user's identity remains intact)
- ✅ Perfect for host-based agents (Copilot CLI, Gemini CLI) that share workspace with user
- ✅ When session ends, user can commit as themselves without reverting config
- ✅ No disruption to shared workspace workflow

**Important:** 
- Must be **sourced** (not executed) to affect current shell
- Identity disappears when shell session ends
- Ideal for Copilot CLI, Gemini CLI, and other host-based agents

---

### `configure_git_identity.sh` (Persistent - For Containerized Agents)

Configure git identity persistently across all repositories by modifying `.git/config`.

**Usage:**
```bash
./.agent/scripts/configure_git_identity.sh "<Name>" "<email>"
```

**Example:**
```bash
./.agent/scripts/configure_git_identity.sh "Antigravity Agent" "roland+antigravity@ccom.unh.edu"
```

**What it does:**
1. Configures git in workspace repository
2. Configures git in all repositories under `layers/*/src/`
3. Sets git user.name and user.email in each `.git/config`

**Why use this:**
- ✅ Persists across all shell sessions
- ✅ Ideal for containerized agents (Antigravity) or dedicated agent-only checkouts
- ✅ Automatically configures all repositories in one command

**Trade-offs:**
- ⚠️ Modifies `.git/config` persistently
- ⚠️ Not suitable for shared workspaces (disrupts user workflow)

**Important:** 
- Best for isolated environments (containers, dedicated checkouts)
- User must manually revert `.git/config` if sharing workspace

**Decision Tree:**
```
Are you in a container or isolated environment?
├─ YES → Use this script (configure_git_identity.sh)
└─ NO → Do you share this checkout with the user?
         ├─ YES → Use set_git_identity_env.sh instead
         └─ NO → Use this script (configure_git_identity.sh)
```

**Dependencies:**
- git
- Write access to all repository `.git/config` files

---

### `lock.sh` / `unlock.sh`

Lock/unlock the workspace to prevent accidental modifications.

**Usage:**
```bash
./.agent/scripts/lock.sh              # Lock the workspace
./.agent/scripts/unlock.sh            # Unlock the workspace
```

**What it does:**
- Creates/removes a `.workspace.lock` file
- Workflows can check for this file to prevent operations on locked workspaces

**When to use:**
- `lock.sh`: Before long operations or to pause work
- `unlock.sh`: When resuming work or if locked unexpectedly

---

### `checkout_default_branch.sh`

Automatically detect and switch to the remote default branch (e.g. `main` or `jazzy`).
 
 **Usage:**
 ```bash
 ./.agent/scripts/checkout_default_branch.sh
 ```
 
 **What it does:**
 1. Detects `origin/HEAD`
 2. Checks for uncommitted changes (Safeguard)
 3. Switches to default branch
 4. Pulls latest changes
 
 **Safety:**
 - Will **EXIT with error** if you have uncommitted changes. This prevents accidental loss of work or "hiding" work in stashes.
 
 ---

### `check_branch_updates.sh`

Check if the default branch has new commits and provide merge/rebase recommendations.

**Usage:**
```bash
./.agent/scripts/check_branch_updates.sh          # Informational check
./.agent/scripts/check_branch_updates.sh --strict # Block if behind
```

**What it does:**
1. Fetches latest commits from default branch (e.g., `main`)
2. Compares your feature branch against it
3. Shows how many commits you're behind/ahead
4. Detects if branches have diverged
5. Provides specific recommendations (merge vs rebase)
6. Displays recent commits you're missing

**When to use:**
- Before committing changes
- Before creating a pull request
- After returning to an old feature branch
- During long-running development work
- When coordinating with team updates

**Exit codes:**
- `0` - Branch is up-to-date, on default branch, or informational check completed
- `1` - Updates required (strict mode only)
- `2` - Error occurred

**Examples:**

```bash
# Check status (non-blocking)
$ ./.agent/scripts/check_branch_updates.sh
⚠️  Default branch has new commits!
  Current branch:  feature/my-feature
  Default branch:  main
  Commits behind:  3
  Commits ahead:   2

📋 Recommendations:
  1️⃣  MERGE: git merge origin/main
  2️⃣  REBASE: git rebase origin/main
```

**Integration:**
- Runs automatically via pre-commit hook (when `make setup-dev` has been run)
- Recommended before creating a pull request

**See also:**
- `.agent/hooks/check-branch-updates.py` - Pre-commit hook implementation

---

## Testing & Validation

### `test.sh`

Run tests on ROS 2 workspace layers.

**Usage:**
```bash
./.agent/scripts/test.sh [layer_names...]
```

**Examples:**
```bash
./.agent/scripts/test.sh               # Test all layers
./.agent/scripts/test.sh core          # Test only core layer
```

**What it does:**
1. Sources ROS 2 environment
2. Runs `colcon test` on specified layers
3. Generates test report

**Dependencies:**
- ROS 2 Jazzy installed
- Workspace built (see `build.sh`)

**Outputs:**
- `.agent/scratchpad/test_report.md` (Human readable)
- `.agent/scratchpad/test_summary.json` (Machine readable)
- `.agent/scratchpad/test_history.csv` (Historical data)

---

### `verify_change.sh`

Run targeted verification on a specific package.

**Usage:**
```bash
./.agent/scripts/verify_change.sh --package <package_name> [--type <unit|lint|all>]
```

**Examples:**
```bash
./.agent/scripts/verify_change.sh --package unh_marine_autonomy --type lint
./.agent/scripts/verify_change.sh --package marine_interfaces --type unit
```

**What it does:**
1. Auto-detects the workspace layer containing the package
2. Runs `colcon test` with specific filters (e.g. `-L unit`)
3. Returns 0 on success, 1 on failure

**When to use:**
- Before committing changes to a specific package
- When an agent is verifying a fix

---

### `validate_repos.py`

Validate `.repos` configuration files for syntax errors.

**Usage:**
```bash
python3 ./.agent/scripts/validate_repos.py
```

**What it does:**
1. Checks all `.repos` files in `configs/`
2. Validates YAML syntax
3. Reports any errors or warnings

**When to use:**
- Before running `setup.sh`
- When modifying `.repos` files
- Troubleshooting import failures

---

## Documentation & Knowledge

### `discover_governance.sh`

Scan workspace and project repos for governance documents (principles, ADRs,
agent guides, architecture docs). Outputs a machine-readable inventory.

**Usage:**
```bash
./.agent/scripts/discover_governance.sh          # TSV output
./.agent/scripts/discover_governance.sh --json   # JSON lines output
```

**Output columns:** `path`, `type`, `size`, `scope`

---

## Worktree Management (Task Isolation)

These scripts enable parallel task development using git worktrees.

### `worktree_create.sh`

Create an isolated worktree for a specific issue/task.

**Usage:**
```bash
./.agent/scripts/worktree_create.sh --issue <number> [--type layer|workspace] [--plan-file <path>]
```

**Examples:**
```bash
./.agent/scripts/worktree_create.sh --issue 123                    # Layer worktree (default)
./.agent/scripts/worktree_create.sh --issue 123 --type workspace   # Workspace worktree
./.agent/scripts/worktree_create.sh --issue 123 --type workspace --plan-file /path/to/plan.md  # With draft PR from plan
./.agent/scripts/worktree_create.sh --issue 123 --branch feature/custom-name
```

**Worktree Types:**
- **layer** (default): For ROS package development. Created in `layers/worktrees/issue-<N>/`
- **workspace**: For infrastructure work (.agent/, configs/, docs). Created in `.workspace-worktrees/issue-<N>/`

**`--plan-file <path>` flag:**
When set, pushes the branch and creates a draft PR immediately after worktree creation,
making work visible on GitHub. The plan file is used to generate a structured PR summary
body (with `Closes #N`) and the full plan is posted as a PR comment. No template file is
committed to the repository. All draft PR operations are non-fatal.

**What it does:**
1. Creates a new git worktree at the specified location
2. Creates a new branch `feature/issue-<N>` (or uses existing)
3. Sets up task-specific scratchpad directory
4. (With `--plan-file`) Pushes branch and creates draft PR with plan

---

### `worktree_list.sh`

List all active git worktrees.

**Usage:**
```bash
./.agent/scripts/worktree_list.sh [--verbose]
```

**Output example:**
```
📁 Main Workspace
   Path:   /path/to/ros2_agent_workspace
   Branch: main
   Status: clean

📦 Issue #123 (layer)
   Path:   /path/to/ros2_agent_workspace/layers/worktrees/issue-123
   Branch: feature/issue-123
   Status: dirty
```

---

### `worktree_enter.sh`

Enter a worktree and set up the environment.

**Usage:**
```bash
source ./.agent/scripts/worktree_enter.sh --issue <number>
```

**Note:** Must be **sourced** (not executed) to affect the current shell.

**What it does:**
1. Changes to the worktree directory
2. Sources ROS 2 environment (for layer worktrees)
3. Sets environment variables: `WORKTREE_ISSUE`, `WORKTREE_TYPE`, `WORKTREE_ROOT`

---

### `worktree_remove.sh`

Remove a worktree and clean up.

**Usage:**
```bash
./.agent/scripts/worktree_remove.sh --issue <number> [--force]
```

**What it does:**
1. Checks for uncommitted changes (unless `--force`)
2. Removes the worktree directory
3. Prunes git worktree references
4. Shows how to delete the branch if desired

---

## Utility Scripts

### `build_report_generator.py`

Generate detailed build reports.

**Usage:**
```bash
python3 ./.agent/scripts/build_report_generator.py
```

**Outputs:**
- Detailed build log summary
- Failed packages list
- Compilation warnings and errors
- Build performance metrics

---

### `list_overlay_repos.py`

List all repositories in overlay workspace layers.

**Usage:**
```bash
python3 ./.agent/scripts/list_overlay_repos.py
python3 ./.agent/scripts/list_overlay_repos.py --format names
python3 ./.agent/scripts/list_overlay_repos.py --include-underlay
```

**Outputs:**
- JSON array of repository information (default)
- Repository names only (with `--format names`)
- Includes underlay repos (with `--include-underlay`)

**Note:** Now uses the shared `lib/workspace.py` module for consistency.

---

### `get_repo_info.py`

Get version/branch information for a specific repository.

**Usage:**
```bash
python3 ./.agent/scripts/get_repo_info.py <repo_name>
```

**Example:**
```bash
python3 ./.agent/scripts/get_repo_info.py project11
# Output: jazzy
```

**Note:** Now uses the shared `lib/workspace.py` module for consistency.

---

### `lib/workspace.py` (Library Module)

Shared library module providing common workspace management functions.

**Functions:**
- `get_workspace_root()` - Get absolute path to workspace root
- `get_overlay_repos(include_underlay=False)` - Get list of all repositories from .repos files
- `find_repo_version(target_repo)` - Find version/branch for a specific repository
- `extract_github_owner_repo(url)` - Extract owner and repo name from GitHub URL

**Usage:**
```python
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'lib'))
from workspace import get_overlay_repos, extract_github_owner_repo

repos = get_overlay_repos()
for repo in repos:
    owner, name = extract_github_owner_repo(repo['url'])
    print(f"{owner}/{name}")
```

**When to use:**
- Creating new scripts that need to discover repositories
- Avoiding code duplication
- Ensuring consistent repository parsing logic

---

## Issue Management

### `read_feature_status.py`

Parse Feature Track issue status and return machine-readable JSON output.

**Usage:**
```bash
python3 ./.agent/scripts/read_feature_status.py --issue <number>
python3 ./.agent/scripts/read_feature_status.py --issue <number> --pretty
```

**Examples:**
```bash
python3 ./.agent/scripts/read_feature_status.py --issue 146
python3 ./.agent/scripts/read_feature_status.py --issue 139 --pretty
```

**What it does:**
1. Fetches issue body from GitHub using `gh` CLI
2. Parses markdown checkboxes from issue body
3. Detects phase numbers from section headers (e.g., "### Phase 2: Implementation")
4. Calculates completion percentage based on checked/total tasks
5. Identifies current unchecked task
6. Returns JSON with status: 'in_progress', 'done', or 'blocked'

**Output format:**
```json
{
  "phase": 2,
  "current_task": "Write implementation code",
  "status": "in_progress",
  "percent_complete": 65
}
```

**When to use:**
- Automated tracking of Feature Track issue progress
- Integration with CI/CD workflows
- Agent scripts that need to determine task status
- Generating progress reports

**Dependencies:**
- `python3`
- `gh` CLI authenticated (`gh auth login`)
- Issue must follow Feature Track format with markdown checkboxes

**Exit codes:**
- `0` - Success
- `1` - Error (issue not found, authentication failure, etc.)

---

## Common Workflows

### Daily status check
```bash
make status                             # Full morning report (sync + GitHub)
make status-quick                       # Fast local-only check
```

### Initialize workspace (first time)
```bash
./.agent/scripts/bootstrap.sh          # Install ROS 2 and tools
./.agent/scripts/setup.sh               # Auto-setup all layers
source ./.agent/scripts/env.sh          # Source environment
./.agent/scripts/build.sh               # Build all layers
```

### Daily development
```bash
source ./.agent/scripts/env.sh         # Each new terminal
./.agent/scripts/status_report.sh --quick  # Quick workspace state check
./.agent/scripts/build.sh core         # Build changes
./.agent/scripts/test.sh core          # Run tests
```

### Before committing
```bash
./.agent/scripts/status_report.sh --quick  # Verify clean state
# Commit changes to feature branch
```

### Troubleshooting
```bash
./.agent/scripts/status_report.sh      # See what's wrong
python3 ./.agent/scripts/validate_repos.py  # Check configs
./.agent/scripts/build_report_generator.py  # Detailed errors
```

---

## Error Messages & Solutions

### "vcs command not found"
```bash
pip install vcstool
# or
sudo apt install python3-vcstool
```

### "ROS 2 Jazzy not found"
```bash
./.agent/scripts/bootstrap.sh
source ./.agent/scripts/env.sh
```

### "Workspace is LOCKED"
```bash
./.agent/scripts/unlock.sh
```

### Script not executable
```bash
chmod +x ./.agent/scripts/*.sh
```

---

**Location**: `.agent/scripts/`
