# CLI Commands Reference

**Purpose**: Quick reference for AI CLI agents to discover and use workspace workflows.

This document maps:
- **Workflow names** → Actual scripts/markdown files
- **Slash commands** (how CLI agents invoke them) → What they do
- **CLI-native alternatives** (when to use direct ROS/git commands instead)

---

## Quick Command Index

| Command | Category | What It Does | When To Use |
|---------|----------|--------------|-------------|
| `/check-status` | Status | Full workspace status (git, builds, GitHub) | Start of session, before/after major changes |
| `/check-local-status` | Status | Quick git status only | Fast check, no GitHub API needed |
| `/check-github-status` | Status | GitHub PRs and issues only | Looking for tasks, checking PR status |
| `/check-branch-updates` | Status | Check if default branch has new commits | Before committing, before PR, during long work |
| `/build` | Build | Build specific workspace or package | Working on single package |
| `/build-all` | Build | Build all workspaces (underlay + overlays) | After vcstool import, major changes |
| `/rebuild-all` | Build | Clean + build all workspaces | Dependency issues, fresh start |
| `/clean` | Cleanup | Remove build artifacts | Disk space, build corruption |
| `/test-all` | Testing | Run all tests across workspaces | Pre-PR validation |
| `/add-repo` | Development | Add new ROS package to workspace | Starting new package |
| `/start-feature` | Development | Create feature branch from clean state | Beginning new task |
| `/finish-feature` | Development | Finalize feature (tests, docs, clean commits) | Task complete, ready for review |
| `/submit-pr` | Development | Create GitHub pull request | Ready to merge |
| `/create-issue` | Development | Create GitHub issue with label validation | Creating new issues programmatically |
| `/setup-environment` | Setup | One-command initial setup | First time in workspace |

---

## Status & Inspection Commands

### `/check-status`

**File**: `.agent/workflows/ops/check-status.md`  
**Script**: `.agent/scripts/status_report.sh`

**What it does**:
- Git status across all repos (branch, uncommitted changes)
- Build status (which workspaces built successfully)
- GitHub status (open PRs, assigned issues)
- Workspace health (missing dependencies, broken packages)

**Example**:
```bash
.agent/scripts/status_report.sh
```

**CLI-native alternative**:
```bash
# Just git status:
git status

# Verbose with vcstool:
vcs status workspaces/ros2_ws/src
```

**When to use**:
- ✅ Start of every session
- ✅ Before committing/pushing
- ✅ When debugging "why isn't this working?"
- ❌ Don't use in tight loops (slow, hits GitHub API)

---

### `/check-local-status`

**File**: `.agent/workflows/ops/check-local-status.md`  
**Script**: Custom (git-only check)

**What it does**:
- Quick git status (no GitHub API calls)
- Shows branch, uncommitted files
- Faster than full `/check-status`

**Example**:
```bash
git status
vcs status workspaces/ros2_ws/src
```

**When to use**:
- ✅ Quick checks during development
- ✅ Avoiding GitHub API rate limits
- ❌ When you need PR/issue information

---

### `/check-github-status`

**File**: `.agent/workflows/ops/check-github-status.md`

**What it does**:
- Lists open PRs
- Shows assigned issues
- Identifies available tasks

**Example** (using GitHub CLI):
```bash
gh pr list
gh issue list --assignee @me
gh issue list --label "good first issue"
```

**When to use**:
- ✅ Looking for tasks to work on
- ✅ Checking PR review status
- ✅ Coordinating with other agents

---

### `/check-branch-updates`

**File**: `.agent/workflows/ops/check-branch-updates.md`  
**Script**: `.agent/scripts/check_branch_updates.sh`

**What it does**:
- Fetches latest commits from default branch
- Compares your feature branch against it
- Shows commits behind/ahead counts
- Provides merge or rebase recommendations
- Displays recent commits you're missing

**Example**:
```bash
.agent/scripts/check_branch_updates.sh
```

**CLI-native alternative**:
```bash
# Manual check
git fetch origin main
git log HEAD..origin/main  # See what's new
git log --oneline --left-right HEAD...origin/main  # Divergence check
```

**When to use**:
- ✅ Before committing changes
- ✅ Before creating a PR
- ✅ After returning to old feature branch
- ✅ During long-running development
- ❌ On default branch (automatically skips)

**Strict mode** (blocks if behind):
```bash
.agent/scripts/check_branch_updates.sh --strict
```

---

## Build Commands

### `/build`

**File**: `.agent/workflows/ops/build.md`

**What it does**:
- Builds specific workspace or package
- Uses `colcon build` with symlink-install
- Handles dependency installation if needed

**Example**:
```bash
cd workspaces/ros2_ws
colcon build --packages-select my_package
```

**CLI-native alternative**:
```bash
cd workspaces/ros2_ws
colcon build --symlink-install
# Or for specific package:
colcon build --packages-select <package_name>
```

**When to use**:
- ✅ After code changes to single package
- ✅ Testing package-specific changes
- ❌ After vcstool import (use `/build-all` instead)

---

### `/build-all`

**File**: `.agent/workflows/ops/build-all.md`  
**Script**: `.agent/scripts/build_all.sh`

**What it does**:
- Builds underlay workspace first
- Then builds each overlay in correct order
- Sources intermediate installs for dependencies

**Example**:
```bash
.agent/scripts/build_all.sh
```

**When to use**:
- ✅ After `vcs import` (new repos added)
- ✅ Fresh clone of workspace
- ✅ Major dependency changes
- ❌ Small changes to single package (use `/build` instead)

---

### `/rebuild-all`

**File**: `.agent/workflows/ops/rebuild-all.md`

**What it does**:
- Cleans all build artifacts (build/, install/, log/)
- Runs `/build-all` from scratch
- Useful for resolving dependency issues

**Example**:
```bash
rm -rf workspaces/*/build workspaces/*/install workspaces/*/log
.agent/scripts/build_all.sh
```

**When to use**:
- ✅ "It worked before, now it doesn't" (stale builds)
- ✅ Dependency hell / linker errors
- ✅ Corrupted build cache
- ❌ Normal development (slow, unnecessary)

---

### `/clean`

**File**: `.agent/workflows/ops/clean.md`

**What it does**:
- Removes build/, install/, log/ directories
- Optionally prunes removed repositories

**Example**:
```bash
rm -rf workspaces/*_ws/build workspaces/*_ws/install workspaces/*_ws/log
```

**When to use**:
- ✅ Freeing disk space
- ✅ Before `/rebuild-all`
- ✅ Cleaning up after experiments
- ❌ During normal development

---

## Development Commands

### `/start-feature`

**File**: `.agent/workflows/dev/start-feature.md`

**What it does**:
- Verifies workspace is clean
- Checks out default branch (usually `main`)
- Pulls latest changes
- Creates new feature branch

**Example**:
```bash
git status --porcelain  # Verify clean
git checkout main
git pull
git checkout -b feature/ISSUE-46-cli-integration
```

**When to use**:
- ✅ Starting new task
- ✅ After completing previous task
- ❌ If already on correct feature branch

---

### `/finish-feature`

**File**: `.agent/workflows/dev/finish-feature.md`

**What it does**:
- Runs tests to ensure quality
- Updates documentation if needed
- Verifies commits are clean and atomic
- Prepares for PR submission

**Checklist**:
- [ ] All tests pass
- [ ] Documentation updated
- [ ] Commits are atomic and well-described
- [ ] No uncommitted changes
- [ ] Ready for code review

**When to use**:
- ✅ Task complete, ready for review
- ✅ Before `/submit-pr`

---

### `/submit-pr`

**File**: `.agent/workflows/dev/submit-pr.md`

**What it does**:
- Pushes feature branch to origin
- Creates GitHub pull request
- Links to related issue
- Adds AI signature

**Example**:
```bash
git push -u origin feature/ISSUE-46-cli-integration
gh pr create --title "Add CLI integration support" --body "Closes #46

**🤖 Authored-By**: Copilot CLI Agent"
```

**When to use**:
- ✅ After `/finish-feature`
- ✅ Task complete and tested
- ❌ Work in progress (use draft PR instead)

---

### `/create-issue`

**Script**: `.agent/scripts/gh_create_issue.sh`

**What it does**:
- Creates GitHub issues with automatic label validation
- Validates labels against `.agent/github_metadata.json` before submission
- Prevents failures from invalid/non-existent labels
- Provides helpful error messages with valid label suggestions
- Supports all standard `gh issue create` options

**Example**:
```bash
# Create issue with validated labels
.agent/scripts/gh_create_issue.sh \
  --title "Fix navigation bug" \
  --body "Description of the bug" \
  --label "bug" \
  --label "enhancement"

# Create with body from file
.agent/scripts/gh_create_issue.sh \
  --title "Add feature" \
  --body-file /tmp/description.md \
  --label "enhancement"

# Interactive mode (no validation)
.agent/scripts/gh_create_issue.sh
```

**Valid labels** (see `.agent/github_metadata.json`):
- `bug` - Something isn't working
- `documentation` - Improvements or additions to documentation
- `duplicate` - This issue or pull request already exists
- `enhancement` - New feature or request
- `good first issue` - Good for newcomers
- `help wanted` - Extra attention is needed
- `invalid` - This doesn't seem right
- `question` - Further information is requested
- `wontfix` - This will not be worked on

**Adding new labels**:
1. Create label in GitHub: `gh label create "my-label" --description "Description" --color "hex-color"`
2. Update `.agent/github_metadata.json` labels array
3. Commit both changes together

**CLI-native alternative**:
```bash
# Direct gh CLI (no validation)
gh issue create --title "Title" --body "Body" --label "bug"
```

**When to use**:
- ✅ Creating issues programmatically with labels
- ✅ Avoiding trial-and-error with label names
- ✅ Batch issue creation with consistent labels
- ❌ Interactive issue creation (use `gh issue create` directly)

---

### `/add-repo`

**File**: `.agent/workflows/dev/add-repo.md`

**What it does**:
- Adds new ROS package repository to workspace
- Updates `.repos` file
- Runs `vcs import`
- Installs dependencies
- Builds new package

**Example**:
```bash
# Edit configs/<workspace>.repos
vcs import workspaces/ros2_ws/src < configs/ros2.repos
rosdep install --from-paths workspaces/ros2_ws/src --ignore-src -r -y
cd workspaces/ros2_ws && colcon build
```

**When to use**:
- ✅ Adding external ROS package
- ✅ Integrating third-party dependency
- ❌ Creating new package from scratch (use `ros2 pkg create` instead)

---

### `/test-all`

**File**: `.agent/workflows/dev/test_all.md`

**What it does**:
- Runs `colcon test` across all workspaces
- Generates test results
- Reports failures

**Example**:
```bash
cd workspaces/ros2_ws
colcon test
colcon test-result --verbose
```

**When to use**:
- ✅ Before submitting PR
- ✅ After major refactoring
- ✅ Validating CI will pass
- ❌ During rapid iteration (slow)

---

## Setup Commands

### `/setup-environment`

**File**: `.agent/workflows/ops/setup-environment.md`

**What it does**:
- Sources ROS environment
- Configures git identity (framework-aware)
- Runs initial status check
- One-command setup for first-time users

**Example**:
```bash
source .agent/scripts/env.sh
source .agent/scripts/set_git_identity_env.sh "Copilot CLI Agent" "roland+copilot-cli@ccom.unh.edu"
.agent/scripts/status_report.sh
```

**When to use**:
- ✅ First time in workspace
- ✅ After shell restart
- ✅ Forgot to source environment
- ❌ Already set up (idempotent but unnecessary)

---

## CLI-Native Alternatives

Sometimes using ROS/git commands directly is more efficient than workflows:

### Git Operations

```bash
# Quick status
git status

# Create branch
git checkout -b feature/my-task

# Commit
git add .
git commit -m "My changes"

# Push
git push -u origin feature/my-task
```

**When to use**: Small, focused git operations where workflows add overhead.

---

### ROS Build Operations

```bash
# Build single package (fast)
cd workspaces/ros2_ws
colcon build --packages-select my_package

# Build with dependencies
colcon build --packages-up-to my_package

# Build parallel
colcon build --parallel-workers 4
```

**When to use**: Iterative development on single package.

---

### ROS Testing

```bash
# Test single package
cd workspaces/ros2_ws
colcon test --packages-select my_package
colcon test-result --verbose

# Run specific test
colcon test --packages-select my_package --pytest-args -k test_my_function
```

**When to use**: Debugging specific test failures.

---

### VCS Operations

```bash
# Import repositories
vcs import workspaces/ros2_ws/src < configs/ros2.repos

# Update repositories
vcs pull workspaces/ros2_ws/src

# Status across all repos
vcs status workspaces/ros2_ws/src
```

**When to use**: Managing multiple repositories efficiently.

---

## Framework-Specific Commands

### GitHub Copilot CLI

If you have access to GitHub Copilot CLI extensions:

```bash
# GitHub operations (faster than web UI)
gh pr list
gh pr view 123
gh issue list
gh issue view 46

# Workspace analysis (if supported)
# Use CLI-native tools for repo scanning
```

### Gemini CLI

Check `.agent/instructions/gemini-cli.instructions.md` for Gemini-specific patterns.

---

## Decision Matrix: Workflow vs. CLI-Native

| Situation | Use Workflow | Use CLI-Native |
|-----------|--------------|----------------|
| **First time setup** | ✅ `/setup-environment` | ❌ Too many steps |
| **Build single package** | ❌ Overhead | ✅ `colcon build --packages-select` |
| **Build after vcs import** | ✅ `/build-all` | ❌ Order matters |
| **Quick git status** | ❌ Overhead | ✅ `git status` |
| **Full workspace status** | ✅ `/check-status` | ❌ Too many commands |
| **Create PR** | ✅ `/submit-pr` | ⚠️ Either (depends on complexity) |
| **Run specific test** | ❌ Overhead | ✅ `colcon test --packages-select` |
| **Clean workspace** | ✅ `/clean` | ⚠️ Either (safety) |

---

## Discovering New Workflows

All workflows are documented in:

```
.agent/workflows/
├── dev/          # Development workflows
│   ├── add-repo.md
│   ├── start-feature.md
│   ├── finish-feature.md
│   ├── submit-pr.md
│   └── ...
└── ops/          # Operations workflows
    ├── build.md
    ├── build-all.md
    ├── check-status.md
    ├── clean.md
    └── ...
```

Browse these directories to find specialized workflows.

---

## Creating Custom Commands

If you find yourself repeating a sequence of commands:

1. **Document it** in `.agent/workflows/dev/` or `.agent/workflows/ops/`
2. **Create a script** in `.agent/scripts/` if complex
3. **Update this file** with the new command mapping
4. **Open a PR** to share with other agents

---

**Last Updated**: 2026-01-27  
**Maintained By**: Framework Engineering Team  
**Related**: [AI_CLI_QUICKSTART.md](AI_CLI_QUICKSTART.md), [AI_RULES.md](AI_RULES.md), [AGENT_INDEX.md](AGENT_INDEX.md)
