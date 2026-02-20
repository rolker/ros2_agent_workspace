# Technology Research for Issue #249: Workspace Simplification

**Issue**: [#249 — Simplify workspace: audit scripts, streamline Makefile, consolidate docs into README](https://github.com/rolker/ros2_agent_workspace/issues/249)
**Date**: 2026-02-20
**Researcher**: Claude Code Agent (Claude Opus 4.6)

---

## Executive Summary

Issue #249 identifies accumulated complexity across three dimensions: 42 scripts, 19
Makefile targets, and 24 documentation files. This research evaluates technologies and
patterns relevant to each simplification phase. The key finding is that an emerging
cross-platform standard (**AGENTS.md**) directly addresses the multi-framework instruction
file duplication problem, while modern command runners (**just**, **Task**, **mise**) offer
compelling alternatives to the current Makefile + shell script architecture.

---

## 1. Command Runner Alternatives to Make

### Current State
The workspace uses a GNU Makefile with 19 targets, most of which are thin wrappers around
shell scripts in `.agent/scripts/`. This two-layer indirection (Makefile → script → colcon)
adds cognitive overhead without providing Make's core value proposition (dependency-based
incremental builds).

### Evaluated Alternatives

#### 1a. `just` (Justfile) — **Recommended for Evaluation**

A Rust-based command runner with syntax close to Make but designed for task execution
rather than builds.

| Aspect | Detail |
|--------|--------|
| **Repository** | https://github.com/casey/just |
| **Language** | Rust (single binary, no deps) |
| **Format** | `justfile` — looks like Makefile but simpler |
| **Key advantage** | Built-in `just --list` with descriptions; no `.PHONY` needed |
| **Tab sensitivity** | Accepts spaces or tabs (unlike Make) |
| **Directory traversal** | Searches parent directories for justfile |
| **Multi-language recipes** | Recipes can be bash, python, etc. |
| **AI integration** | `just-mcp` provides MCP adapter for LLM tooling |
| **Migration path** | Closest syntax to Makefile; migration is straightforward |
| **Stability** | No planned breaking changes; frequent minor releases |
| **Editor support** | LSP via `just-lsp`; syntax highlighting in Helix, VS Code, etc. |

**Fit for this workspace**: Strong. The workspace uses Make purely as a command runner
(not for file-based dependency tracking). `just` would eliminate `.PHONY` declarations,
provide self-documenting task listing out of the box, and support multi-language recipes
for the Python validation scripts. The `just-mcp` integration is interesting for future
agent tooling.

#### 1b. `task` (Taskfile.dev)

A Go-based task runner using YAML configuration.

| Aspect | Detail |
|--------|--------|
| **Repository** | https://github.com/go-task/task |
| **Language** | Go (single binary) |
| **Format** | YAML (`Taskfile.yml`) |
| **Key advantage** | Checksum-based dependency detection; includes with namespaces |
| **Cross-platform** | Linux, macOS, Windows |
| **Verbosity** | More verbose than Make/just (4 lines for single command) |
| **Preconditions** | Built-in conditional execution |
| **Global taskfile** | Supports `~/Taskfile.yml` for cross-project tasks |

**Fit for this workspace**: Moderate. YAML syntax is more verbose than needed for
what are mostly simple shell commands. The namespace includes feature is attractive
for the layered workspace structure, but the verbosity cost may not justify switching.

#### 1c. `mise` (mise-en-place)

A Rust-based polyglot tool combining version management, environment variables, and
task running.

| Aspect | Detail |
|--------|--------|
| **Repository** | https://github.com/jdx/mise |
| **Docs** | https://mise.jdx.dev/ |
| **Language** | Rust |
| **Format** | TOML (`mise.toml`) |
| **Scope** | Dev tools + env vars + task runner |
| **Maturity** | Tasks out of experimental; stable as of 2025 |
| **Unique value** | Can manage tool versions AND run tasks |

**Fit for this workspace**: Lower priority. The workspace already has ROS 2 managing
its toolchain via `apt` and `rosdep`. mise's tool management value is diminished in a
ROS environment where system packages dominate. However, its task runner could be useful
if the team wants a single tool for environment + tasks.

### Recommendation

**`just` is the strongest candidate** for replacing the Makefile. It has the closest
syntax to Make (lowering migration cost), provides self-documentation natively, and
has emerging AI/LLM integration. However, **the current Makefile with self-documenting
patterns** (see Section 2) may be sufficient if the team prefers zero new dependencies.

---

## 2. Self-Documenting Makefile Patterns

If the team decides to keep Make rather than adopting a new tool, the Makefile should
be made self-documenting.

### The Standard Pattern

Annotate targets with `## comment` after the target declaration:

```makefile
.DEFAULT_GOAL := help

setup: ## Initialize workspace layers from project configuration
	.agent/scripts/setup.sh

build: ## Build all workspace layers with colcon
	.agent/scripts/build.sh

help: ## Show this help message
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-20s\033[0m %s\n", $$1, $$2}'
```

### Grouped Targets Variant

For the workspace's 7 proposed core targets plus internal ones, use `##@` section headers:

```makefile
##@ Core Workflow
setup: ## Initialize workspace
build: ## Build all layers
test:  ## Run all tests

##@ Maintenance
status: ## Show workspace status
lint:   ## Run pre-commit hooks
clean:  ## Remove build artifacts

##@ Help
help:   ## Display this help
```

### Variable Documentation (Julio Merino, 2025)

A newer pattern documents user-settable variables alongside targets. Useful for the
workspace's configurable options (e.g., `ISSUE=<N>` for `revert-feature`).

**Sources**:
- [Self-Documented Makefile (Marmelab)](https://marmelab.com/blog/2016/02/29/auto-documented-makefile.html)
- [Self-documenting Makefiles (Julio Merino, 2025)](https://blogsystem5.substack.com/p/make-help)
- [Well-documented Makefiles (Suvash Thapaliya)](https://www.thapaliya.com/en/writings/well-documented-makefiles/)
- [GitHub Gist — prwhite](https://gist.github.com/prwhite/8168133)

---

## 3. AGENTS.md — Cross-Platform Agent Instruction Standard

### Overview

AGENTS.md is an open, vendor-neutral specification for providing AI coding agents with
project-specific instructions. It is stewarded by the **Agentic AI Foundation** under the
Linux Foundation, with contributions from OpenAI, Anthropic (via MCP), and others.

| Aspect | Detail |
|--------|--------|
| **Specification** | https://agents.md/ |
| **Repository** | https://github.com/agentsmd/agents.md |
| **Adoption** | 60,000+ open-source projects (as of late 2025) |
| **Supported tools** | Codex, Cursor, Devin, Factory, Gemini CLI, GitHub Copilot, Jules, VS Code, and more |
| **Measured impact** | 28.6% median runtime reduction, 16.6% token reduction |

### How It Relates to This Workspace

The workspace currently maintains **four separate instruction files**:
- `CLAUDE.md` (Claude Code)
- `.github/copilot-instructions.md` (GitHub Copilot)
- `.agent/instructions/gemini-cli.instructions.md` (Gemini CLI)
- `.agent/AI_RULES.md` (universal fallback)

AGENTS.md could serve as the **single canonical source** for shared workspace instructions
(build commands, testing procedures, coding conventions, architecture), with
framework-specific files containing only agent-specific behavioral rules.

### Key Design Properties

- **Placement**: Root of repository; subdirectories can have their own `AGENTS.md`
- **Precedence**: Closest `AGENTS.md` to the edited file wins; user prompts override all
- **Format**: Standard Markdown, no special syntax required
- **Size guideline**: ≤ 150 lines recommended for performance
- **Complement, not replace**: Works alongside `README.md` (for humans) and framework
  files (for agent-specific behavior)

### Relevance to Issue #249

AGENTS.md directly addresses the maintainer's concern about instruction duplication. The
current approach duplicates workspace instructions across four files. AGENTS.md would:

1. **Consolidate shared instructions** into one file read by all agents
2. **Preserve framework-specific rules** in their respective files (CLAUDE.md, etc.)
3. **Align with industry standard** adopted by 60K+ projects

However, the maintainer's comment notes that duplication was **intentional** to prevent
compromise. AGENTS.md's design addresses this: framework-specific files can override or
extend AGENTS.md, so security-sensitive rules can remain in per-framework files.

### Relationship to Anthropic's Agent Skills

Anthropic introduced **Agent Skills** (October 2025) as a complementary standard — modular,
reusable AI agent capabilities as self-contained directories. This is related but separate
from AGENTS.md and could be relevant for the workspace's script organization.

**Sources**:
- [AGENTS.md Official Site](https://agents.md/)
- [AGENTS.md GitHub Repository](https://github.com/agentsmd/agents.md)
- [OpenAI Codex Guide — AGENTS.md](https://developers.openai.com/codex/guides/agents-md/)
- [Agentic AI Foundation (OpenAI announcement)](https://openai.com/index/agentic-ai-foundation/)
- [AGENTS.md Deep Dive (PRPM)](https://prpm.dev/blog/agents-md-deep-dive)

---

## 4. Script Consolidation Patterns

### Current State

The workspace has 42 scripts in `.agent/scripts/`. The audit in issue #249 should classify
each as keep/deprecate/consolidate.

### Patterns for Reducing Script Sprawl

#### 4a. Subcommand Pattern (CLI-style)

The workspace already has an `agent` script with subcommands (`agent start-task <N>`).
Expanding this pattern could replace many standalone scripts:

```
agent setup          → setup.sh
agent build          → build.sh
agent status         → status_report.sh
agent status --quick → status_report.sh --quick
agent worktree create → worktree_create.sh
agent worktree list   → worktree_list.sh
```

This reduces the number of "entry points" a user must know about from 42 to 1 (`agent`),
while keeping the underlying scripts as implementation details.

#### 4b. Library Extraction

The workspace already has `.agent/scripts/lib/` with shared functions. Moving more
common logic into libraries reduces duplication across scripts. The key candidates:
- Git operations (branch detection, remote queries)
- GitHub API interactions
- ROS 2 workspace path resolution
- Output formatting and logging

#### 4c. Script Categories (Keep / Consolidate / Deprecate)

Based on the workspace exploration:

**Keep as-is** (clear purpose, no overlap):
- `worktree_create.sh`, `worktree_enter.sh`, `worktree_list.sh`, `worktree_remove.sh`
- `build.sh`, `test.sh`
- `env.sh`
- `sync_repos.py`, `validate_workspace.py`

**Consolidate candidates**:
- `status_report.sh` + `pr_status.sh` + `health_check.sh` → unified status with flags
- `set_git_identity_env.sh` + `configure_git_identity.sh` → single script with mode flag
- `validate_repos.py` + `validate_workspace.py` → already complementary, but could merge

**Audit for deprecation**:
- Scripts listed in the issue as "dead or undocumented" — requires individual review

---

## 5. Documentation Consolidation

### Current State

24 markdown files across root and `.agent/` directories, with documented overlaps between
CLAUDE.md, AI_RULES.md, README.md, ARCHITECTURE.md, and scripts/README.md.

### Patterns and Approaches

#### 5a. README as Single Source of Truth (Issue #249 Proposal)

The issue proposes making README.md the canonical reference. This is the simplest approach
and avoids introducing new tooling. The proposed structure:

1. What this is
2. Quick start
3. Key workflows
4. For AI agents
5. Architecture
6. Customization
7. Development tools
8. Contributing

**Pros**: No new tools, universally understood, renders on GitHub
**Cons**: Long READMEs become hard to navigate; mixing human + agent docs may dilute both

#### 5b. Tiered Documentation (Recommended)

Instead of one mega-README, use a tiered approach:

| Document | Audience | Content |
|----------|----------|---------|
| `README.md` | Humans | Quick start, architecture overview, contributing |
| `AGENTS.md` | All AI agents | Build/test commands, coding conventions, workspace patterns |
| `CLAUDE.md` | Claude Code only | Claude-specific behavioral rules, env setup |
| `ARCHITECTURE.md` | Both | System design (referenced, not duplicated) |

This approach:
- Follows the AGENTS.md standard for agent instructions
- Keeps README.md focused on human developers
- Eliminates redundancy between AI_RULES.md and framework files
- Preserves the maintainer's intent for framework-specific isolation

#### 5c. Documentation Tools (Lower Priority)

For this workspace, dedicated documentation tooling (mdBook, Docusaurus) is overkill.
The content is operational instructions, not user-facing documentation. Plain Markdown
files in the repository are the right format.

---

## 6. ROS 2 Workspace Best Practices

### Relevant Patterns for Simplification

- **Selective builds**: `colcon build --packages-select <pkg>` and
  `colcon build --packages-up-to <pkg>` are well-supported and should be documented
  prominently
- **COLCON_IGNORE**: Place empty file to exclude packages from builds
- **Symlink install**: `--symlink-install` for faster iteration on Python packages
- **Colcon mixins**: Preconfigured command-line shortcuts; could replace some script logic
- **rosdep**: `rosdep install --from-paths src --ignore-src -r` for dependency resolution

### Workspace Layering

The current layered architecture (underlay → overlay chain) is standard ROS 2 practice.
No simplification needed at the architecture level. The complexity is in the tooling
around it, not the pattern itself.

**Sources**:
- [ROS 2 Colcon Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
- [Henki Robotics — ROS 2 Best Practices (2025)](https://henkirobotics.com/ros-2-best-practices/)
- [leggedrobotics/ros_best_practices](https://github.com/leggedrobotics/ros_best_practices)
- [Package Organization for ROS (Robotics Back-End)](https://roboticsbackend.com/package-organization-for-a-ros-stack-best-practices/)

---

## 7. Recommendations Summary

### For Phase 1 (Audit)
- Use the script categorization in Section 4c as a starting framework
- Evaluate each script against the question: "Is this called by users, by other scripts,
  or by nobody?"

### For Phase 2 (Simplify Interface)
- **Option A (conservative)**: Keep Make, add self-documenting `##` pattern, reduce to
  7 core targets as proposed. Lowest risk, no new dependencies.
- **Option B (moderate)**: Replace Makefile with `justfile`. Better UX, self-documenting
  by default, low migration cost. Requires installing `just`.
- Expand the `agent` subcommand pattern to serve as the primary human/agent interface.

### For Phase 3 (Documentation)
- Adopt **AGENTS.md** as the cross-platform agent instruction file
- Keep `README.md` human-focused and concise
- Keep `CLAUDE.md` for Claude-specific behavioral rules only (env setup, worktree rules)
- Keep `ARCHITECTURE.md` as the referenced architecture document
- Deprecate `AI_RULES.md` (replaced by AGENTS.md)
- Deprecate `.agent/scripts/README.md` (fold into `make help` or `just --list`)

### For Phase 4 (Align Agent Instructions)
- CLAUDE.md references AGENTS.md for shared instructions, contains only:
  - Environment setup commands
  - Claude-specific behavioral rules
  - Worktree workflow rules (if Claude-specific)
- Same pattern for copilot-instructions.md and gemini-cli.instructions.md

---

**Authored-By**: `Claude Code Agent`
**Model**: `Claude Opus 4.6`
