# Changelog

All notable changes to the ROS 2 Agent Workspace will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- **Draft PR on Worktree Creation** (Issue #205):
  - `--draft-pr` flag for `worktree_create.sh`
  - Workspace worktrees: generates work plan from template, commits, pushes, creates draft PR
  - Layer worktrees: empty commit + push + draft PR in each package repo
  - Cross-repo issue references for layer draft PRs
  - All operations non-fatal — worktree creation succeeds even if `gh` is unavailable
- **Cross-Repo PR Triage Dashboard** (Issue #207):
  - `--all-repos` flag for `pr_status.sh` — queries all workspace repos in one view
  - `discover_repos()` function reusing `list_overlay_repos.py --include-underlay`
  - Repo name shown in dashboard, simple, and JSON output when using `--all-repos`
  - `make pr-triage` convenience target (`pr_status.sh --all-repos --simple`)
- **PR Status Dashboard** (Issue #164):
  - `.agent/scripts/pr_status.sh` - Interactive PR pipeline visibility tool
  - Automatic categorization: Needs Review, Critical Issues, Minor Issues, Ready to Merge
  - Comment severity classification (critical vs minor keywords)
  - Interactive mode for workflow actions (review, fix, merge)
  - `.agent/scripts/PR_STATUS_README.md` - Usage documentation
- **Conductor-Inspired Workflow Patterns** (Issue #139):
  - `.github/ISSUE_TEMPLATE/feature_track.md` - Structured planning template for features/bugs
  - Plan-Before-Code rule in `.agent/AI_RULES.md` - Enforces planning before implementation
  - Phase Verification Protocol in `.agent/AI_RULES.md` - User checkpoints between major phases
  - `.agent/scripts/revert_feature.sh` - Revert all commits for a specific issue
  - `/revert-feature` command added to CLI_COMMANDS.md and Makefile
  - Updated CONTRIBUTING.md with planning workflow and feature reversal guidance
  - Updated AI_CLI_QUICKSTART.md with Feature Track template reference
- **AI CLI Integration** (Issue #46):
  - `.agent/AI_RULES.md` - Universal agent rules (single source of truth)
  - `.agent/AI_CLI_QUICKSTART.md` - 5-minute setup guide for CLI agents
  - `.agent/CLI_COMMANDS.md` - Workflow discovery and command mapping
  - `.agent/instructions/gemini-cli.instructions.md` - Gemini CLI support
  - `.agent/workflows/ops/setup-environment.md` - One-command environment setup
  - Enhanced `.agent/scripts/configure_git_identity.sh` with `--agent` and `--detect` flags
  - New `.agent/scripts/detect_cli_env.sh` for framework auto-detection
  - Enhanced `.agent/scripts/set_git_identity_env.sh` with framework presets
  - Basic test suite for framework detection
- **GitHub API Integration** (Issue #46):
  - Enhanced `/check-status` workflow with CLI-aware GitHub API usage
  - Caching for GitHub data to avoid rate limits
  - Fallback to web API when CLI tools unavailable
- **Documentation Consolidation** (Issue #46):
  - Single source of truth for universal agent rules (`.agent/AI_RULES.md`)
  - Framework-specific overlay pattern (Copilot CLI, Gemini CLI)
  - Updated README.md with CLI-specific quick start section
  - Updated AGENT_INDEX.md with CLI agent navigation
  - Updated CONTRIBUTING.md with documentation maintenance guidelines
- **Temporary File Management** (Issue #44):
  - `.agent/scratchpad/` directory for persistent temporary artifacts
  - Custom pre-commit hook `check-source-artifacts.py` to warn about temp files in source directories
  - Comprehensive README for scratchpad usage
- Comprehensive documentation suite:
  - ARCHITECTURE.md - Detailed system architecture and design
  - CONTRIBUTING.md - Development guidelines and workflows
  - SECURITY.md - Security policy and best practices
  - QUICKSTART.md - Step-by-step setup guide for new users
  - CHANGELOG.md - Version history tracking
- Development tooling:
  - Makefile with common operations (build, test, clean, etc.)
  - .editorconfig for consistent code formatting
  - .pre-commit-config.yaml for automated code quality checks
  - requirements.txt for Python dependencies
- CI/CD infrastructure:
  - GitHub Actions workflow for validation (validate.yml)
  - Dependabot configuration for dependency updates
  - Markdown link checking configuration
- New utility scripts:
  - health_check.sh - Comprehensive workspace health validation
  - validate_repos.py - Syntax and structure validation for .repos files
- Enhanced script documentation:
  - Added detailed headers to setup.sh, build.sh, test.sh

### Changed
- **Status Script Consolidation** (Issue #203):
  - Consolidated `status_report.sh`, `status_full.sh`, and `check_full_status.py` into a single `status_report.sh`
  - Added `--quick` flag (alias for `--skip-sync --skip-github`) for fast local-only checks
  - Added `--pr-triage` flag for PR comment classification (critical/minor) across all workspace repos
  - Removed `check_full_status.py` (functionality merged into `status_report.sh`)
  - Updated `Makefile` targets: `make status` and `make status-quick` both use `status_report.sh`
  - Added deprecation note to `pr_status.sh` pointing to `--pr-triage`
- **Git Hygiene** (Issue #44):
  - Updated `.agent/rules/common/clean-root.md` with detailed scratchpad policy
  - Added `.agent/scratchpad/` to `.gitignore`
  - Standardized temporary file location from ambiguous `./ai_workspace` to `.agent/scratchpad/`
- Pre-commit configuration now includes custom source artifact detection hook
  - Added module docstrings to Python scripts
- README improvements:
  - Quick start section with links to guides
  - Troubleshooting section with common issues
  - Development tools section
  - Links to all documentation
- Fixed shellcheck warnings in scripts:
  - bootstrap.sh - Separated variable declaration and assignment (SC2155)
  - build.sh - Used grouped redirects for efficiency (SC2129)
  - env.sh - Changed `! -z` to `-n` (SC2236)
- Added shebang to env.sh for proper shell detection
- Improved code quality and maintainability across all scripts

### Security
- Documented eval() usage in build_report_generator.py with security note
- Added security policy (SECURITY.md)
- Configured pre-commit hooks for security checks

## [Previous Versions]

Historical changes before this changelog are documented in git commit history.
See: https://github.com/rolker/ros2_agent_workspace/commits/main
