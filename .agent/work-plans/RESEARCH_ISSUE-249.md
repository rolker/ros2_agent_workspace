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

### `colcon defaults.yaml` — Declarative Build Configuration

Regardless of which task runner is chosen, colcon's own `defaults.yaml` can absorb
flags currently hardcoded in scripts. Instead of encoding `--symlink-install` and
`--cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON` in `build.sh`, declare them in a
`defaults.yaml` file per layer. This makes build scripts thinner and the configuration
more discoverable. See [colcon Quick Start](https://colcon.readthedocs.io/en/released/user/quick-start.html).

### Recommendation

**`just` is the strongest candidate** for replacing the Makefile. It has the closest
syntax to Make (lowering migration cost), provides self-documentation natively, and
has emerging AI/LLM integration. However, **the current Makefile with self-documenting
patterns** (see Section 2) may be sufficient if the team prefers zero new dependencies.

Example migration of current Makefile to justfile:

```just
# justfile
set shell := ["bash", "-euo", "pipefail", "-c"]

# Show available recipes
default:
    @just --list

# Build all workspace layers
build:
    ./.agent/scripts/build.sh

# Build a specific package
build-pkg package:
    cd layers/main/core_ws && colcon build --packages-select {{package}}

# Run tests on all layers
test:
    ./.agent/scripts/test.sh

# Show workspace status
status *args:
    ./.agent/scripts/status_report.sh {{args}}

# Revert all commits for an issue
revert-feature issue:
    ./.agent/scripts/revert_feature.sh --issue {{issue}}
```

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

### Multi-Tool Symlink Strategy

For workspaces that use both CLAUDE.md and AGENTS.md, the recommended approach
([Kaushik Gopal](https://kau.sh/blog/agents-md/)) is:

```bash
# Option 1: AGENTS.md as canonical, symlink for Claude
mv CLAUDE.md AGENTS.md
ln -s AGENTS.md CLAUDE.md

# Option 2: Separate files, Claude imports shared content
# CLAUDE.md contains:
#   @AGENTS.md
#   # Claude-specific overrides below
```

For this workspace, **Option 2 is better** because CLAUDE.md has Claude-specific
behavioral rules (env.sh sourcing, worktree guards) that should not appear in the
universal AGENTS.md.

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
- [AGENTS.md Sync Strategy (Kaushik Gopal)](https://kau.sh/blog/agents-md/)
- [One File to Guide Them All (Layer5)](https://layer5.io/blog/ai/agentsmd-one-file-to-guide-them-all/)
- [AI Instruction Files Comparison (0xdevalias)](https://gist.github.com/0xdevalias/f40bc5a6f84c4c5ad862e314894b2fa6)

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

#### 4d. Target Reduction

Based on industry research ([ScriptRunner](https://www.scriptrunner.com/blog-cio-head-of-it/tool-sprawl-killing-it-productivity)),
73% of professionals say fragmented tooling blocks collaboration. A realistic target
for this workspace is reducing from 42 scripts to approximately 15-20 by:
1. Inlining simple wrapper scripts into the task runner
2. Merging related scripts (status, identity, validation)
3. Deprecating unused or experimental scripts
4. Moving test scripts to a test framework

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

#### 5c. CLAUDE.md Best Practices (Anthropic Guidance)

Based on [Anthropic's official guidance](https://code.claude.com/docs/en/best-practices),
[builder.io](https://www.builder.io/blog/claude-md-guide), and
[HumanLayer](https://www.humanlayer.dev/blog/writing-a-good-claude-md):

- **Keep CLAUDE.md under 300 lines** — for every line, ask "Would removing this cause
  Claude to make mistakes?" If not, cut it. Bloated files cause instructions to be ignored.
- **Use `@imports` for progressive disclosure** — put detailed instructions in separate
  files and reference them with `@path/to/file`. Claude pulls in content when relevant.
- **Use `.claude/rules/` for topic-specific rules** — instead of one massive file, split
  into focused rule files (e.g., `code-style.md`, `git-workflow.md`). Auto-loaded.
- **Evolve organically** — treat CLAUDE.md like code. Review it when agent behavior
  is wrong. Use emphasis (`IMPORTANT`, `YOU MUST`) sparingly but when needed.

#### 5d. Documentation Tools (Lower Priority)

For this workspace, dedicated documentation tooling (mdBook, Docusaurus) is overkill.
The content is operational instructions, not user-facing documentation. Plain Markdown
files in the repository are the right format.

If a browsable docs site becomes needed later, **MkDocs Material**
([squidfunk.github.io/mkdocs-material](https://squidfunk.github.io/mkdocs-material/))
is the best fit — Python-native (aligns with ROS 2 ecosystem), supports Mermaid diagrams
for architecture visualization, and can render existing .md files with minimal config.

#### 5e. Documentation Drift Detection

An emerging tool, [Packmind's context-evaluator](https://packmind.com/evaluate-context-ai-coding-agent/),
analyzes git repos and flags stale AI instruction files. This addresses the workspace's
documented problem of features being "implemented but not reliably followed."

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

## 7. Architecture Documentation Lifecycle

### Problem Statement

ARCHITECTURE.md is a reference document, but nothing in the workflow requires consulting
it. The instruction files say "Read When Needed, Not Upfront" — which agents interpret as
"skip it." Decisions documented only in commit messages or issue comments are at risk of
being silently reverted.

### Current State

- `ARCHITECTURE.md` exists at workspace root, describing system design and layering
- `.github/workflows/validate.yml` checks that `ARCHITECTURE.md` exists (line 37), but
  does **not** check whether it was updated when relevant files changed
- `.pre-commit-config.yaml` has four custom hooks, none related to architecture sync
- No PR template exists (`.github/PULL_REQUEST_TEMPLATE.md` is missing)
- No Architecture Decision Records (ADRs) — design rationale lives only in issues/commits

### 7a. CI Enforcement (No AI Required)

Most architecture enforcement is structural and deterministic. Traditional CI checks are
fast, free, and cover the failure modes that actually occur in practice.

**File-change correlation** — "If infrastructure files changed, did architecture docs get
updated too?" Uses [tj-actions/changed-files](https://github.com/tj-actions/changed-files)
(4,000+ stars) to detect changes in architecture-relevant files (`configs/`, `Makefile`,
`.agent/scripts/env.sh`, `setup.sh`) and warn if neither `ARCHITECTURE.md` nor
`docs/decisions/` was updated in the same PR.

**Structural validation** — "Does the doc match reality?" Script-based checks that verify
paths referenced in ARCHITECTURE.md actually exist, and that every `*_ws` directory is
mentioned. Catches the common drift pattern where a layer is added/removed without
updating the docs.

**Dependency boundary enforcement** — "Do packages respect layer ordering?" Parse
`package.xml` files and verify that packages in `core_ws` don't depend on packages in
higher-layer workspaces. This is graph analysis — no AI needed. This **prevents**
architecture violations rather than just documenting them.

**New package → require ADR** — If a PR adds a new `package.xml`, require a corresponding
ADR in `docs/decisions/` explaining why the package was added and which layer it belongs
to. Enforced as a CI check that blocks merge.

**PR checklist enforcement** — GitHub Actions can parse the PR body and verify that if
"Architecture-relevant change" is checked, the sub-items (ARCHITECTURE.md updated, ADR
created) are also checked.

### 7b. Where AI Adds Value (Optional, Lower Priority)

| Check | Needs AI? | Why |
|-------|-----------|-----|
| File-change correlation | No | File existence check |
| Directory tree matches doc | No | Path existence check |
| Layer dependency violations | No | Graph analysis of package.xml |
| PR approach contradicts ADR intent | **Yes** | Natural language understanding |
| Should this PR have created an ADR? | **Maybe** | Heuristics get 80%, AI gets the rest |
| Is the ARCHITECTURE.md update accurate? | **Yes** | Requires reading code + prose together |

AI-powered semantic review is expensive, slow, and non-deterministic — three things you
generally don't want in CI. It also requires API keys and cost management. Start with the
structural checks; add AI review later only if the gap is felt.

### 7c. Architecture Decision Records (ADRs)

ADRs are short, immutable documents — one per decision — that capture **context**,
**decision**, and **consequences**. The canonical template is
[MADR (Markdown Architectural Decision Records)](https://adr.github.io/madr/).

**Proposed file organization:**

```
docs/decisions/
  0001-layered-workspace-architecture.md
  0002-worktree-isolation-over-branch-switching.md
  0003-duplicate-agent-instructions-per-framework.md
  0004-env-vars-for-agent-identity.md
```

ADR #3 is critical — it documents that instruction duplication was **intentional**, making
the decision discoverable and durable rather than buried in an issue comment.

**Minimal MADR template:**

```markdown
# {short title}

## Status
Accepted | Deprecated | Superseded by [ADR-NNNN](NNNN-title.md)

## Context and Problem Statement
{Why was this decision needed?}

## Decision
{What was decided.}

## Consequences
{Positive and negative consequences.}
```

**Tooling**: [adr-tools](https://github.com/npryce/adr-tools) (pure Bash, zero
dependencies) provides `adr new`, `adr list`, `adr generate toc`. Alternatively,
just create files manually — no tooling required.

### 7d. Agent Instruction Integration

Add an "Architecture Checkpoints" section to CLAUDE.md (and equivalents) that names
specific files at specific workflow stages:

```markdown
## Architecture Checkpoints

### Before Planning
- Read ARCHITECTURE.md "Core Concepts" — which layer does this touch?
- Scan docs/decisions/ titles — any ADRs that constrain this area?

### Before Opening PR
- If you added/removed/renamed a package → update ARCHITECTURE.md
- If you made a design decision with alternatives → create an ADR
```

This works because agents process their instruction file on every session. It converts
architecture consultation from a suggestion into a workflow step.

### 7e. PR Template

Create `.github/PULL_REQUEST_TEMPLATE.md` with an architecture impact section:

```markdown
## Architecture Impact

- [ ] No architecture impact (routine change within existing patterns)
- [ ] Architecture-relevant change (check all that apply below):
  - [ ] Adds/removes/renames a package or layer
  - [ ] Changes cross-layer dependencies
  - [ ] Changes the build or source order
  - [ ] Modifies agent workflow or instruction files

### If architecture-relevant:
- [ ] ARCHITECTURE.md updated (or confirmed still accurate)
- [ ] ADR created in docs/decisions/ (if a new architectural decision)
```

Research indicates checklists are effective when short (5-8 items), project-specific, and
enforced by automation. For AI agents, the template text becomes part of the PR creation
context, forcing active engagement with each checkbox.

### 7f. Defense in Depth

The most effective pattern combines multiple layers:

| Layer | Speed | Bypassable? | Purpose |
|-------|-------|-------------|---------|
| Pre-commit hook | Fast | Yes (`--no-verify`) | Developer feedback |
| PR template checklist | At PR creation | Rubber-stampable | Conscious acknowledgment |
| CI file-correlation check | Minutes | No | Enforcement |
| CI structural validation | Minutes | No | Drift detection |
| CI dependency boundary check | Minutes | No | Violation prevention |

No single layer is sufficient. Pre-commit hooks can be bypassed; checklists can be
rubber-stamped; CI catches what the others miss. The workspace already follows this
defense-in-depth pattern for linting.

### 7g. Agent Compliance with Local Tooling

**Observed problem**: Agents bypass pre-commit hooks (via `--no-verify` or equivalent),
believing the hooks don't apply to them. This is not a bug in any single agent — it
reflects a fundamental misalignment: agents optimize for task completion, and local
tooling looks like friction to skip.

**Why this happens**:
- Pre-commit hooks are opt-in enforcement — any `git commit` can bypass them
- Agent instruction files may say "never skip hooks" but agents from other frameworks
  may not have equivalent instructions, or may interpret them loosely
- Agents don't inherently understand that hooks are policy, not convenience

**Mitigation layers** (from weakest to strongest):

| Layer | Mechanism | Agent-proof? |
|-------|-----------|-------------|
| Instruction files | "NEVER use `--no-verify`" in CLAUDE.md, AGENTS.md, etc. | No — depends on compliance |
| Wrapper scripts | `agent commit` command that doesn't expose `--no-verify` flag | Partial — agents can bypass the wrapper |
| Git hook + CI parity | Run the same checks in both pre-commit and CI | Yes — CI catches what hooks miss |
| Branch protection | Require status checks to pass before merge | Yes — non-bypassable |
| Server-side hooks | Pre-receive hooks on the git server (GitHub doesn't expose these for hosted repos) | Yes — but not available on GitHub |

**Practical recommendation**: The only truly agent-proof enforcement is CI + branch
protection. Pre-commit hooks are useful for fast feedback but should never be the sole
enforcement mechanism. Every check that matters must also run in CI.

For this workspace specifically:
1. Ensure every pre-commit hook has a CI equivalent in `validate.yml`
2. Add explicit "hooks are mandatory, not optional" language to AGENTS.md (shared
   across all frameworks)
3. Consider a wrapper script (`agent commit`) that runs hooks explicitly before
   committing, so agents that bypass `git commit` hooks still get checked
4. Use branch protection rules to require CI status checks before merge

### 7h. User Trust and Design Transparency

**Problem**: Users working with AI agents need varying levels of visibility into
*why* decisions were made, not just *what* was done. A user new to the codebase or
to AI-assisted development needs more rationale than an experienced user who just
wants results. The framework should support this as a dial, not a switch.

**Transparency mechanisms, from lightweight to heavy:**

| Mechanism | Audience | Cost | When |
|-----------|----------|------|------|
| Descriptive commit messages | Everyone | Low | Every commit |
| ADR references in code comments | Code reviewers | Low | When touching architecture-relevant code |
| PR description with design rationale | PR reviewers | Low-Medium | Every PR |
| Inline code comments explaining "why" | Future developers/agents | Low | Non-obvious design choices only |
| Work plans with alternatives considered | Users who want oversight | Medium | Complex tasks |
| Interactive confirmation before key decisions | Users who want control | Medium | Configurable per-user |

**What this means for agent instructions:**

Agents should be told to provide rationale proportional to the decision's impact:

```markdown
## Design Transparency

### Always explain (in commit messages and PR descriptions):
- Why you chose this approach over alternatives
- What constraints or ADRs influenced the decision
- What trade-offs were accepted

### Explain in code comments only when:
- The choice is non-obvious and a future reader would ask "why?"
- You're implementing a pattern that contradicts common convention
- The code references an architectural decision (link to ADR)

### Don't over-explain:
- Standard patterns that follow existing codebase conventions
- Trivial changes (formatting, renames, typo fixes)
- Choices with no meaningful alternatives
```

**User control mechanisms:**

The level of rationale an agent provides could be configured per-user or per-session:

- **Minimal**: Agent does the work, provides standard commit messages and PR descriptions.
  For users who trust the framework and review results.
- **Standard** (default): Agent explains key decisions in PR descriptions and flags
  architecture-relevant changes. For typical development.
- **Detailed**: Agent documents alternatives considered, links to relevant ADRs, and
  asks for confirmation before architecture-impacting decisions. For new users or
  sensitive changes.

This could be implemented as:
- An environment variable (`AGENT_TRANSPARENCY=minimal|standard|detailed`)
- A section in the user's agent config file
- A per-issue label (e.g., `needs-review` triggers detailed mode)

**Connection to ADRs**: ADRs serve double duty here. They document the rationale for
architectural decisions *and* provide agents with context for explaining their choices.
When an agent's implementation aligns with an existing ADR, it can reference it:
"Following ADR-0002 (worktree isolation), created a new worktree rather than switching
branches." This is both transparent and efficient — the rationale exists once in the
ADR, and agents just point to it.

**Sources**:
- [tj-actions/changed-files (GitHub)](https://github.com/tj-actions/changed-files)
- [brettcannon/check-for-changed-files (GitHub)](https://github.com/brettcannon/check-for-changed-files)
- [pre-commit.com](https://pre-commit.com/)
- [ADR process — AWS Prescriptive Guidance](https://docs.aws.amazon.com/prescriptive-guidance/latest/architectural-decision-records/adr-process.html)
- [adr.github.io](https://adr.github.io/)
- [MADR documentation](https://adr.github.io/madr/)
- [npryce/adr-tools (GitHub)](https://github.com/npryce/adr-tools)
- [GitHub PR Template (Pull Checklist)](https://www.pullchecklist.com/posts/github-pr-template)
- [Pull Request Template — Microsoft Engineering Playbook](https://microsoft.github.io/code-with-engineering-playbook/code-reviews/pull-request-template/)
- [arxiv: Agentic Coding Manifests](https://arxiv.org/html/2509.14744v1)

---

## 8. Workspace vs. Project Boundary

### Design Intent

This workspace repo is a **generic ROS 2 agent workspace** — it should work for any
ROS 2 project, not just the current dogfood project (project 11, UNH marine autonomy).
Everything in this repo should be either generic ROS 2 tooling or framework-agnostic
agent infrastructure. Project-specific content belongs in the project repos.

| Content type | Lives in | Examples |
|---|---|---|
| Build/worktree/CI infrastructure | Workspace repo | Makefile, `.agent/scripts/`, worktree workflow |
| Agent instruction patterns | Workspace repo | CLAUDE.md, AGENTS.md templates, `.agent/knowledge/` |
| Architecture doc *patterns* (ADR templates, PR templates) | Workspace repo | `.agent/templates/`, `.github/PULL_REQUEST_TEMPLATE.md` |
| Generic ROS 2 agent knowledge | Workspace repo | How to build/test/lint ROS 2 packages |
| Integration docs for external packages | Workspace repo | `.agent/project_knowledge/robot_localization.md` |
| Project architecture (`ARCHITECTURE.md`) | Project repo | `unh_marine_autonomy/ARCHITECTURE.md` |
| Project ADRs | Project repo | `unh_marine_autonomy/docs/decisions/` |
| Project agent guide (`.agents/README.md`) | Project repo | Package inventory, conventions, pitfalls |

### The Role of `.agent/project_knowledge/`

This directory serves as a **symlink aggregation point** that gives agents a unified
view of project-level knowledge without traversing the full `layers/main/*/src/*/` tree.

It has two types of content:

**1. Symlinks to project repo knowledge** (for repos you own):

```
.agent/project_knowledge/
├── unh_marine_autonomy/ → ../../layers/main/core_ws/src/unh_marine_autonomy/.agents/
```

The project repo contains the canonical knowledge (architecture, conventions, pitfalls).
The symlink makes it accessible from the workspace root so agents can grasp the big
picture without knowing the layer structure.

**2. Integration docs for external packages** (for repos you don't own):

```
.agent/project_knowledge/
├── robot_localization.md      # How we configure and depend on it
├── ros_gz.md                  # Our Gazebo integration assumptions
```

These are workspace-level files documenting your *relationship* to packages you use
but can't add documentation to. They answer:
- What specific interfaces (topics/services/params) does our code depend on?
- What configuration assumptions are in our launch files?
- What version constraints matter and why?
- What workarounds are we carrying?

**Why this matters for agents**: When an agent enters a worktree, it can read
`.agent/project_knowledge/` to understand the full project context — both the
project's own architecture (via symlinks) and the integration surface with
external packages (via workspace-level docs). No tree traversal needed.

**Why this matters for reusability**: When someone forks this workspace for a
different ROS 2 project, the `project_knowledge/` directory starts empty. They
populate it with symlinks to their own project repos and integration docs for
the external packages they use. The workspace infrastructure doesn't change.

---

## 9. Cross-Reference with Open Issues

21 issues are currently open. Several overlap with or complement #249's scope. This
section maps each relevant issue to this research, identifies synergies, and flags
topics the research may have underweighted.

### Directly Addressed by This Research

| Issue | Title | Covered in |
|-------|-------|------------|
| #241 | Enforce Antigravity Agent Compliance | §7g (agent compliance), §7f (defense in depth) |
| #172 | Pre-commit hook for outdated doc references | §7a (structural validation), §7f (defense in depth) |
| #126 | Workspace Context Awareness & Drift Detection | §5e (drift detection), §7a (structural validation) |
| #214 | UX: improve workflow discoverability | §1 (just/Make), §2 (self-documenting Makefile), §4a (subcommand pattern) |
| #155 | Add Context Analysis & DoD to Feature Template | §7d (architecture checkpoints), §7e (PR template) |
| #245 | Generate Claude Code slash commands from Makefile | §1a (just + just-mcp), §4a (subcommand pattern) |

### Complementary (Not Directly Addressed but Related)

| Issue | Title | Relationship | Gap in research? |
|-------|-------|-------------|-----------------|
| #247 | Harden worktree rules — prohibit editing main tree | Enforcement problem (like §7g). The issue proposes instruction-level rules, but the same compliance gap applies: agents may ignore instructions. Container-level isolation (#229) is the stronger solution. | Minor — §7g covers the enforcement philosophy but doesn't address worktree-specific guardrails |
| #229 | Sandboxed agent development: DevContainer with YOLO mode | The strongest form of §7g's enforcement: instead of telling agents not to bypass hooks, make it physically impossible via filesystem constraints (read-only mounts, volume isolation). | **Yes — research should note container isolation as the ultimate enforcement layer** |
| #234 | Incremental package quality improvement | Quality-on-entry pattern complements §7d (architecture checkpoints). When an agent enters a package, it checks quality gaps — same principle as checking ARCHITECTURE.md before planning. | Minor — could reference as a parallel pattern |
| #235 | Periodic workspace health sweep | The "structural validation" CI check (§7a) is a subset of this. The health sweep is broader: instruction consistency, script relevance, template freshness, dependency alignment. | Minor — §7a covers architecture-specific checks; #235 covers the full workspace |
| #124 | Automate Agent Identity & Git Signatures | Related to §7g — identity setup is another thing agents skip or get wrong. The instruction-based approach has the same compliance gap. | No — orthogonal to architecture enforcement |
| #131 | Agent guidance for ROS 2 package licensing | Not covered by this research. Licensing is a quality gate, not an architecture concern. Could be a CI check (verify `package.xml` has a license field). | No — out of scope |

### Key Insight from Issue Cross-Reference

**Container isolation (#229, #237, #236) is the missing enforcement layer.** The research
(§7g) identifies CI + branch protection as "agent-proof" enforcement, but the open issues
reveal a more ambitious approach already being explored: **make violations impossible at
the filesystem level** rather than detecting them after the fact.

The enforcement hierarchy, updated:

| Layer | Mechanism | Catches violations... |
|-------|-----------|----------------------|
| Instructions | CLAUDE.md, AGENTS.md | ...if the agent reads and follows them |
| Pre-commit hooks | `.pre-commit-config.yaml` | ...before commit, if not bypassed |
| Wrapper scripts | `agent commit` | ...before commit, if agent uses the wrapper |
| CI + branch protection | GitHub Actions + rules | ...before merge (non-bypassable) |
| **Container isolation** | DevContainer + read-only mounts | ...**before they can happen** (filesystem prevents them) |

Container isolation (bottom layer) is the strongest because it shifts from
**detect-and-block** to **prevent-by-construction**. An agent in a container with
read-only mounts on `layers/main/*/src/` physically cannot modify the main tree.

### Issues Not Related to This Research

| Issue | Title | Why not related |
|-------|-------|----------------|
| #243 | Root Cause Analysis: validate_workspace.py | Bug investigation, not architecture |
| #244 | CLAUDE.md rule: never guess GitHub URLs | Instruction quality, not simplification |
| #230 | ros2launch_session for integration testing | Testing tooling, not architecture/CI |
| #162 | Stacked PRs and sub-issues | Git workflow, not architecture enforcement |
| #129 | Format GitHub references as clickable links | UX polish |
| #102 | Project-specific glossary | Documentation content, not structure |

---

## 10. Recommendations Summary

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
- Formalize `.agent/project_knowledge/` as symlink aggregation point (see §8):
  - Symlinks to project repos' `.agents/` directories for owned packages
  - Integration docs (plain files) for external packages you use but don't own
  - Document this pattern in CLAUDE.md and AGENTS.md so agents know to check it
- Ensure all workspace-repo content is generic ROS 2 / agent infrastructure — project-
  specific architecture docs, ADRs, and `.agents/README.md` belong in project repos

### For Phase 4 (Align Agent Instructions)
- CLAUDE.md references AGENTS.md for shared instructions, contains only:
  - Environment setup commands
  - Claude-specific behavioral rules
  - Worktree workflow rules (if Claude-specific)
- Same pattern for copilot-instructions.md and gemini-cli.instructions.md

### For Phase 5 (Architecture Documentation Lifecycle)

Ordered by impact-to-effort ratio:

| Priority | Action | Effort | AI? |
|----------|--------|--------|-----|
| 1 | Create `.github/PULL_REQUEST_TEMPLATE.md` with architecture impact checklist | Low | No |
| 2 | Add "Architecture Checkpoints" to CLAUDE.md and equivalent instruction files | Low | No |
| 3 | Create `docs/decisions/` and seed 3-5 retroactive ADRs (layered workspaces, worktree isolation, instruction duplication, env.sh guardrails) | Medium | No |
| 4 | Add file-change correlation CI check to `validate.yml` | Medium | No |
| 5 | Add structural validation CI check (referenced paths exist) | Medium | No |
| 6 | Add layer dependency boundary CI check (no upward deps in package.xml) | Medium | No |
| 7 | Add architecture-sync pre-commit hook (warning-only) | Low | No |
| 8 | (Future) AI-powered semantic architecture review on PRs | High | Yes |

Key insight: items 1–7 are all deterministic, traditional CI/tooling. AI (item 8)
adds value only for semantic checks ("does this PR contradict the *intent* of an ADR?")
and should be deferred until the structural checks are in place.

### For Phase 6 (Agent Compliance and User Trust)

**Agent compliance** (see also §9 cross-reference with #241, #247, #229):
- Ensure every pre-commit hook check has a CI equivalent — hooks are fast feedback,
  CI is enforcement. Agents bypass hooks; they can't bypass CI + branch protection.
- Add "hooks are mandatory" language to AGENTS.md (cross-framework)
- Consider `agent commit` wrapper that runs checks explicitly
- Longer-term: container isolation (#229) is the strongest enforcement — it prevents
  violations by construction (read-only mounts) rather than detecting them after the fact

**User trust / transparency**:
- Define transparency levels (`minimal`, `standard`, `detailed`) as a configurable dial
- Default to `standard`: agents explain key decisions in PR descriptions, flag
  architecture-relevant changes, reference ADRs when applicable
- ADRs serve double duty: they document rationale *and* give agents something to
  reference instead of generating explanations from scratch
- Commit messages should explain "why" not "what" — this is low-cost and always useful

---

**Authored-By**: `Claude Code Agent`
**Model**: `Claude Opus 4.6`
