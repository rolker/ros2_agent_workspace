# Changelog

All notable changes to the ROS2 Agent Workspace will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
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

### Changed
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
