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

A late addition (Appendix B) analyzes Addy Osmani's "[How to Write a Good Spec for AI
Agents](https://www.oreilly.com/radar/how-to-write-a-good-spec-for-ai-agents/)" (O'Reilly,
January 2026), which draws on GitHub's analysis of 2,500+ agent configuration files. Five
actionable ideas are identified: the **Always/Ask/Never rule taxonomy**, **self-audit
instructions**, **SPEC.md as a persistent session anchor**, **conformance suites as
stepping stones** to full instruction testing, and **formalizing the two-phase task
pattern** (draft spec, then execute incrementally). These are integrated into the
recommendations in §10.

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

**Bootstrapping from project history ("workspace archaeology")**:

Most projects that need ARCHITECTURE.md and ADRs already have a rich history of
architectural decisions — they're just buried in closed issues, PR descriptions,
commit messages, and deleted files. The archaeology method systematically recovers
this intent:

1. **Closed issues**: Scan for design proposals, RFCs, and "architecture" labels.
   Issues often contain the *context* and *alternatives considered* that ADRs need.
2. **Unmerged PRs**: PRs closed without merging represent approaches that were tried
   and rejected — exactly the "alternatives considered" section of an ADR.
3. **Commit history**: Look for `revert`, `refactor`, phase-numbered commits, and
   commit bodies containing "TODO", "future work", or "deferred." These signal
   decisions that evolved over time.
4. **Deleted files**: Use `git log --diff-filter=D --summary` to find files that
   were built and later removed. The build-then-delete pattern often represents
   an architectural decision (e.g., "we tried a skill framework and decided it
   was over-engineered").
5. **Recurring themes**: Group findings by theme. If the same problem was addressed
   3+ times (e.g., "agents don't follow rules" → documentation → hooks → guardrails
   → container isolation), that progression *is* the architectural rationale and
   deserves an ADR.

The output of this process is:
- A set of retroactive ADRs capturing decisions that were made implicitly
- An ARCHITECTURE.md grounded in actual history, not aspirational design
- A list of lost or underweight intent that may warrant new issues

This method is especially valuable for **project repos that have been under
development without formal architecture documentation**. An agent can perform
the archaeology as a dedicated task (with its own issue), producing ADRs and an
ARCHITECTURE.md that reflects how the project actually evolved — not how someone
imagines it was designed.

For this workspace, the archaeology in Appendix A recovered five themes, identified
three items of lost intent worth restoring, and surfaced the most significant
undelivered capability (instruction regression testing). The same technique applied
to a project repo like `unh_marine_autonomy` would recover design decisions about
sensor driver patterns, mission planning architecture, and simulation integration
that currently live only in git history.

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

**The diff-approval gap — real-time rationale at the point of decision:**

The mechanisms above (commit messages, PR descriptions, ADRs) all provide rationale
*after* the work is done. But the most critical transparency moment is earlier: when
the agent presents a diff for approval and the user must decide yes/no *right now*.

At that moment, the user typically sees:
- Which file is being modified (gives context about the area)
- The raw diff (shows *what* changed)

What's missing:
- **Why this approach** — what pattern is being applied, what alternative was rejected
- **How this edit fits the big picture** — is this 1 of 12 related changes? The
  foundation for something coming next? A prerequisite?
- **What design decision is embedded** — is this introducing a new pattern, or
  following an existing one?

This matters because a diff that looks wrong in isolation might be correct in context,
and a diff that looks fine might be subtly introducing architectural drift. The user
can't evaluate either without knowing the agent's reasoning at that moment.

**Possible approaches (from lightweight to heavy):**

| Approach | Mechanism | Cost | Trade-off |
|----------|-----------|------|-----------|
| Pre-edit narration | Agent states intent *before* presenting the diff: "I'm applying the observer pattern here because..." | Low | Depends on agent discipline; easy to skip |
| Edit annotations | Structured comment above each tool call explaining rationale, pattern, and big-picture fit | Low-Medium | Adds tokens to every edit; may be noise for experienced users |
| Session plan visibility | Agent publishes a numbered plan at task start; each edit references its step: "Edit 3/7: implementing the subscriber (step 2 of plan)" | Medium | Requires planning discipline; plan may drift |
| Claude Code hooks (PostToolUse) | A hook that captures the edit context and presents a summary alongside the diff | Medium | Technical; requires hook development |
| Diff annotation metadata | Tool-level support where the Edit tool accepts optional `rationale` and `pattern` fields displayed in the approval UI | Low (if supported) | Requires upstream Claude Code changes |

The lightest viable approach is **pre-edit narration as agent instruction** — adding
to `CLAUDE.md`:

```markdown
## Edit Rationale

Before presenting a diff for approval, briefly state:
1. What pattern or approach this edit implements
2. How it fits into the current task (e.g., "3 of 5 changes for the new subscriber")
3. Any non-obvious design choice and why you made it

Skip rationale for trivial edits (formatting, typos, renames).
```

This costs nothing to implement — it's just an instruction. The question is whether
agents follow it consistently, which connects back to the instruction compliance
problem (§9). The heavier approaches (hooks, tool metadata) enforce it mechanically
but require development effort.

**Connection to workspace archaeology**: The rationale that's lost when a session ends
is exactly the kind of context that workspace archaeology must later reconstruct from
commit messages, PR descriptions, and issue comments. If the rationale is captured at
edit time and flows into commits, the archaeology becomes easier — or unnecessary.

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

### Agent Instruction Portability: The package.xml Principle

**Problem**: Project repos will contain agent instructions (`AGENTS.md`),
architecture docs, ADRs, and `.agents/README.md`. These files must make sense
in two very different contexts:

1. **In-workspace**: An agent has the full picture — workspace CLAUDE.md,
   worktree workflow, `.agent/project_knowledge/`, layered build infrastructure.
2. **Standalone on GitHub**: A PR reviewer (GitHub Copilot, a human, a CI bot)
   sees only the project repo. No workspace context exists.

If project repo instructions reference workspace concepts (worktrees, `make build`,
`.agent/project_knowledge/`), they become confusing or broken in the standalone
context.

**The principle**: Packages don't know about workspaces. A `package.xml` declares
its own dependencies — it never says "I'm in core_ws" or "build me with `make build`."
That's a workspace concern. **Agent instructions should follow the same pattern.**

**Project repo AGENTS.md must be fully standalone.** It assumes the reader (human
or AI) has only this repo. No references to worktrees, layered builds, `make`
targets, or `.agent/project_knowledge/`. Standard ROS 2 commands only:

```markdown
# AGENTS.md (in a project repo)

## Build & Test
colcon build --packages-select <package>
colcon test --packages-select <package>

## Architecture
See [ARCHITECTURE.md](ARCHITECTURE.md) for system design.
See [docs/decisions/](docs/decisions/) for architectural decision records.

## Packages
- pkg_a: Sensor drivers and hardware interfaces
- pkg_b: Mission planning and execution

## Conventions
- ...
```

GitHub Copilot reviewing a PR sees this, understands the project on its own terms,
and gives useful reviews. No dangling references to workspace infrastructure it
can't see.

**The workspace adds context on top, never the other way around.** The workspace's
CLAUDE.md / AGENTS.md says "when working in a project repo, also check
`.agent/project_knowledge/`" and "use worktrees, not branch checkout" and "build
via `make build`." Those are workspace-level overlays. The project repo is oblivious.

**Instruction layering** (mirrors the ROS 2 overlay pattern):

```
┌─────────────────────────────────────┐
│ Workspace CLAUDE.md / AGENTS.md     │  Workspace-specific: worktrees,
│ .agent/project_knowledge/           │  layered builds, integration docs
├─────────────────────────────────────┤
│ Project repo AGENTS.md              │  Standalone: standard ROS 2 commands,
│ Project repo ARCHITECTURE.md        │  project architecture, conventions
│ Project repo .agents/README.md      │
│ Project repo docs/decisions/        │
└─────────────────────────────────────┘
```

An agent in the workspace sees both layers. An agent on GitHub (Copilot reviewing
a PR, a contributor cloning just the repo) sees only the bottom layer. Both get
coherent, non-contradictory instructions.

**The discipline this requires**: when writing project repo docs, never use
workspace-specific commands or reference workspace-level paths. Always use
standard ROS 2 commands (`colcon build`, `colcon test`, `rosdep install`). If
you catch yourself writing workspace-specific instructions in a project repo,
that content belongs in the workspace layer instead.

**Test for correctness**: clone the project repo into a fresh, standalone
workspace with no agent infrastructure. Do the instructions in AGENTS.md still
work? If not, they've leaked workspace assumptions.

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
- **SPEC.md worktree anchor** (from Appendix B-ext, Idea 3): When entering a worktree,
  surface the work plan so agents always re-read it. Minor script change to
  `worktree_enter.sh` + instruction update in CLAUDE.md.

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
- **Restructure rules using Always/Ask/Never taxonomy** (from Appendix B-ext, Idea 1):
  Reorganize CLAUDE.md and AGENTS.md rules into three tiers (Always do / Ask first /
  Never do) based on GitHub's analysis of 2,500+ agent configuration files. This is
  cleaner and more parseable than the current flat list with inconsistent emphasis markers.
  See §B-ext.3 for concrete example

### For Phase 4 (Align Agent Instructions)
- **Workspace-level** instruction files (CLAUDE.md, copilot-instructions.md, etc.)
  reference a shared AGENTS.md and contain only framework-specific overrides:
  - Environment setup commands
  - Framework-specific behavioral rules
  - Worktree workflow rules
- **Project-level** AGENTS.md files must be fully standalone (see §8, "package.xml
  principle"): standard ROS 2 commands only, no workspace references. These must
  work for GitHub Copilot reviewing PRs, contributors cloning just the repo, or
  any context outside the workspace.
- **Portability test**: clone the project repo into a bare workspace. If the
  instructions break, they've leaked workspace assumptions.
- The workspace layer adds context on top (worktrees, `make build`, integration
  docs) — project repos are oblivious to it, just as packages are oblivious to
  which workspace they're built in.
- **Add self-audit instruction** (from Appendix B-ext, Idea 2): Add a "Post-Task
  Verification" section to CLAUDE.md and AGENTS.md requiring agents to compare
  their work against the issue/spec before declaring a task complete. Zero
  implementation cost, addresses the intent-level gap between CI checks and
  human review. See §B-ext.3 for concrete wording.
- **Formalize the two-phase task pattern** (from Appendix B-ext, Idea 5): Explicitly
  separate "Draft the Spec" (plan mode, read architecture docs, generate acceptance
  criteria) from "Execute Incrementally" (one section at a time, self-audit after
  each). This mostly formalizes existing `--plan-file` practice but adds explicit
  self-audit loops and spec-update steps.
- **Optional requirements spec for complex issues** (from Appendix D, §D6): For issues
  tagged `architecture-relevant` or similarly complex, add an optional "Requirements &
  Acceptance Criteria" section to the plan template. This captures goals, constraints,
  and testable criteria *before* the plan captures approach. Not a mandatory gate — avoid
  the Spec Kit waterfall overhead (see D3.5).
- **Do not adopt Spec Kit wholesale** (from Appendix D, §D6): The workspace's existing
  workflow (issue → plan → draft PR → implement) already covers ~70% of Spec Kit's value.
  The full four-phase gated workflow adds ceremony that conflicts with the "radical
  simplicity" principle. Cherry-pick useful concepts (three-tier boundaries, `/analyze`
  validator, constitution pattern) rather than importing the framework.

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
| 8 | Create lightweight instruction conformance file (`.agent/conformance/`) with deterministic checks as a stepping stone to full Promptfoo pipeline (Appendix B-ext, Idea 4) | Low-Medium | No |
| 9 | Add `/analyze`-style consistency checker (from Appendix D, §D3): validate that instruction files, plans, and code are internally consistent — referenced paths exist, spec terms match code names, no naming drift across documents. GitHub Spec Kit demonstrates the concept; we can implement a lightweight version as a script. | Medium | No |
| 10 | (Future) AI-powered semantic architecture review on PRs | High | Yes |

Key insight: items 1–9 are all deterministic, traditional CI/tooling. AI (item 10)
adds value only for semantic checks ("does this PR contradict the *intent* of an ADR?")
and should be deferred until the structural checks are in place.

**Bootstrapping ADRs and ARCHITECTURE.md for project repos**: Use the workspace
archaeology method (§7c) as a repeatable pattern. For each project repo that needs
architecture docs, an agent can perform the archaeology as a dedicated task —
scanning closed issues, unmerged PRs, reverted commits, and deleted files to
recover implicit decisions. This produces retroactive ADRs grounded in actual
history rather than aspirational design. See Appendix A for the method applied
to this workspace.

### For Phase 6 (Agent Compliance and User Trust)

**Agent compliance** (see also §9 cross-reference with #241, #247, #229):
- Ensure every pre-commit hook check has a CI equivalent — hooks are fast feedback,
  CI is enforcement. Agents bypass hooks; they can't bypass CI + branch protection.
- Add "hooks are mandatory" language to AGENTS.md (cross-framework)
- Consider `agent commit` wrapper that runs checks explicitly
- Longer-term: container isolation (#229) is the strongest enforcement — it prevents
  violations by construction (read-only mounts) rather than detecting them after the fact
- **File-change-triggered enforcement** (from Appendix D, §D4): Amazon Kiro validates
  the pattern of triggering validation prompts on specific file changes. Claude Code's
  PostToolUse hooks can implement a similar pattern — e.g., after edits to `package.xml`,
  `ARCHITECTURE.md`, or `CLAUDE.md`, automatically run a validation check. This bridges
  the gap between pre-commit hooks (too late) and real-time monitoring (too expensive).

**User trust / transparency**:
- Define transparency levels (`minimal`, `standard`, `detailed`) as a configurable dial
- Default to `standard`: agents explain key decisions in PR descriptions, flag
  architecture-relevant changes, reference ADRs when applicable
- ADRs serve double duty: they document rationale *and* give agents something to
  reference instead of generating explanations from scratch
- Commit messages should explain "why" not "what" — this is low-cost and always useful
- **Diff-time rationale** (§7h): Add "Edit Rationale" instruction to CLAUDE.md requiring
  agents to state the pattern, big-picture fit, and non-obvious choices *before* each
  diff. This is the cheapest intervention with the highest trust impact — it closes the
  gap between "I can see what changed" and "I understand why it changed this way." Longer
  term, Claude Code tool-level metadata (rationale field on Edit) would make this
  mechanical rather than instruction-dependent.

---

## Appendix A: Workspace Archaeology — Lost and Underweight Intent

Analysis of 95 closed issues, 129 closed PRs (4 unmerged), and 236 commits to identify
ideas and intentions that were implemented then removed, partially implemented, or
proposed but never delivered.

### A1. Lost Intent (Implemented Then Removed)

These features were built, merged, and later deleted during the doc consolidation (#181).
The question for each: was the *intent* worth preserving even if the *implementation*
was wrong?

#### Continuous Improvement Workflow (#110)
**What it was**: A structured process for agents to report infrastructure friction at
session end. Agents would review their session for documentation gaps, workflow problems,
environment issues, and discoverability problems, then file well-formed improvement issues.

**Why it was removed**: Deleted as part of the 51-file cleanup in #181. The workflow
file format (`.agent/workflows/`) was deemed unnecessary — agents don't need step-by-step
workflow files to use `gh issue create`.

**Is the intent still valuable?** Yes. The *mechanism* was over-engineered, but the
*practice* of agents reporting friction is how the workspace improves. Currently nothing
prompts agents to do this. A single line in AGENTS.md ("At session end, file an issue
for any friction you encountered") would preserve the intent without the overhead.

**Triage**: Add to Phase 3 recommendations — one line in AGENTS.md, not a workflow file.

#### Workspace Glossary (#100)
**What it was**: A comprehensive terminology reference disambiguating "project" vs
"workspace", "underlay" vs "overlay", "repository" vs "package", etc. Included an
"Ambiguity Resolution Examples" section and a "Terms to Avoid" section.

**Why it was removed**: Deleted in #181 phase 6 with the rationale "agents understand
these terms." Also contained Project11-specific content (UNH Marine Autonomy definitions).

**Is the intent still valuable?** Partially. Agents *don't* reliably understand the
workspace/project/package/repository distinctions — this is exactly the kind of ambiguity
that causes wrong-worktree and wrong-repo bugs. The generic ROS 2 terminology section
belongs in AGENTS.md or `.agent/knowledge/`. The Project11-specific section belongs in
the project repo (consistent with §8's workspace/project boundary).

**Triage**: Extract generic terms (workspace, overlay, underlay, package, repository,
layer) into AGENTS.md. Project-specific terms go in project repo's AGENTS.md.

#### Guiding Principles / Agent Optimizer Guide (#90)
**What it was**: Five pillars for workspace design decisions: quality artifacts,
continuous self-improvement, multi-agent scalability, radical simplicity, and
industry alignment.

**Why it was removed**: Deleted in #181 — content was either inlined into framework
files or deemed unnecessary.

**Is the intent still valuable?** The principles themselves are sound and align with
this research's findings. "Radical Simplicity" is literally what #249 is about.
"Multi-Agent Scalability" motivates the worktree isolation pattern. These principles
would be useful in ARCHITECTURE.md as the "why" behind design decisions — not as a
separate file, but as a section that gives future agents (and humans) the design
philosophy.

**Triage**: Seed as part of the ADR effort (Phase 5). ADR-0001: "Workspace Design
Principles" capturing the five pillars as architectural context.

#### Skills System (test-engineering, doc-coauthoring, project-management, etc.)
**What it was**: 18 skill files providing structured procedures for tasks like
writing ROS 2 tests, triaging issues, creating documentation, etc. Each had templates,
step-by-step procedures, and scope definitions.

**Why it was removed**: The custom skill framework was unused by any agent framework.
Test templates were preserved in `.agent/templates/testing/`.

**Is the intent still valuable?** The test engineering templates survive. The rest
was genuinely over-engineered — agents don't need a "skill framework" to write docs
or triage issues. However, the *knowledge* in the test engineering skill (ROS 2 test
patterns, launch_testing examples, GTest/PyTest templates) is valuable reference
material. It currently lives in `.agent/templates/testing/` which is adequate.

**Triage**: No action needed — valuable content was preserved, framework was
rightfully removed.

#### Start-Feature Workflow & Submit-PR Workflow
**What it was**: Step-by-step workflows for starting feature work (check status,
create branch/worktree, create work plan, open draft PR) and submitting PRs
(verification checklist, PR description template, post-submit steps).

**Why it was removed**: Content inlined into framework instruction files (CLAUDE.md
etc.) which now contain the worktree workflow directly.

**Is the intent still valuable?** The content is preserved — CLAUDE.md contains the
worktree workflow. But the *PR submission checklist* may have been lost in the
inlining. Worth verifying that the PR template recommendation (Phase 5, item 1)
would recapture this.

**Triage**: Verify PR template covers what submit-pr.md had. If not, incorporate
into `.github/PULL_REQUEST_TEMPLATE.md`.

### A2. Undelivered Intent (Proposed But Never Implemented)

#### Agent Instruction Regression Testing (#154, PR #159)
**What it was**: Promptfoo-based framework to automatically test that agent
instruction files produce correct agent behavior. PR #159 implemented it but
was closed without merging due to Gemini API timeouts in CI.

**Why it failed**: Technical — API provider timeouts, not a bad idea. The concept
(regression testing for agent instructions) remains unimplemented.

**Is the intent still valuable?** Yes — this is the most significant undelivered
capability. As instruction files evolve (AGENTS.md, CLAUDE.md), there's no
automated way to verify they still produce correct behavior. Manual testing is
the only option today.

**Triage**: Add as a future-phase recommendation. The Promptfoo approach may not
be the right tool (CI-time API calls are fragile), but the *concept* of instruction
regression testing deserves exploration. Alternatives: static analysis of instruction
files (do referenced paths exist? are command examples valid?), or periodic
human-in-the-loop review.

#### Track Lifecycle Management (#150)
**What it was**: Ability to archive and restore feature tracks — marking completed
work as archived so it doesn't clutter active views, with the ability to restore
if needed.

**Status**: Issue was closed as completed, but the feature tracks themselves were
later removed in the #181 consolidation. The archive/restore concept is moot without
the track system.

**Triage**: No action needed — the underlying system was removed.

#### Git Notes for Agent Handoff Audit Trail (#151)
**What it was**: Using `git notes` to attach metadata to commits for agent handoff
— recording which agent worked on what, session boundaries, and decision context.

**Status**: Issue was closed as completed, but the notes system was removed in the
consolidation.

**Is the intent still valuable?** The handoff problem is real in multi-agent
environments. Git notes are a clever low-overhead approach, but they don't transfer
across forks/clones by default. The draft PR workflow (create PR with work plan)
is the current solution and is arguably better — PR descriptions are visible,
searchable, and survive forks.

**Triage**: No action needed — draft PR workflow serves this purpose.

#### Review-Against-Rules Workflow (#149)
**What it was**: Automated checking of code changes against workspace rules
before submission.

**Status**: Closed as completed, but the workflow was later deleted. The intent
(automated rule compliance checking) is partially covered by pre-commit hooks
and CI.

**Triage**: Covered by Phase 6 (CI parity with pre-commit hooks). No separate
action needed.

### A3. Partially Delivered Intent

#### Multi-Agent Workflow (PR #38, Issue #35)
**What it was**: RFC proposing phased multi-agent coordination using worktrees,
Docker isolation, and TDD. PR #38 delivered Phases 1-2 (testing/reporting and
agent behavior codification). The proposal was set to "Implementing" status,
suggesting further phases.

**What was delivered**: Worktree isolation, identity management, draft PR
visibility, and the WORKFORCE_PROTOCOL.md.

**What wasn't**: Docker-based isolation (now tracked as #229), TDD integration
as a first-class workflow, and formalized agent-to-agent handoff.

**Triage**: Docker isolation is tracked (#229). TDD as a workflow is partially
covered by test templates. Agent handoff is handled by draft PRs. The original
RFC's vision is mostly realized through different mechanisms.

#### Workspace/Project Config Separation (#132)
**What was delivered**: `bootstrap.yaml` pattern, `setup.sh` integration,
`.agent/project_knowledge` symlink, ARCHITECTURE.md documentation.

**What the owner intended but may be underweight**: rolker's design discussion
formalized two patterns (standalone config repo vs. key-repo-is-also-a-package)
and detailed the `agent_context/` directory structure. The symlink aggregation
pattern we discussed in §8 is exactly what was designed here, but the current
implementation may not fully reflect the detailed design from the comment.

**Triage**: Verify current implementation matches the design in rolker's Feb 12
comment on #132. The §8 section of this research should align with that intent.

#### Enforce Workflow Protocol Compliance (#103)
**What was delivered**: Pre-commit hook blocking commits to main, git checkout
guardrail, commit identity validation, GitHub issue template.

**What was deferred**: "Remaining UX improvements tracked separately" per rolker's
closing comment. These became #214 (still open): Makefile target for issue
workflow, additional issue templates, main workspace warning.

**Triage**: Already tracked as open issue #214.

### A4. Recurring Themes from the Archaeological Record

**1. Enforcement over documentation** (threads: #103 → #125 → #178 → #241)
The workspace repeatedly learned that agents don't follow written rules. The
evolution: documentation → prominent warnings → pre-commit hooks → checkout
guardrails → worktree enforcement → container isolation. Each layer was added
after the previous one proved insufficient. This arc validates the enforcement
hierarchy in §9 (cross-reference section).

**2. Build-then-delete cycle** (threads: #90 → #181, skills → #181, workflows → #181)
Complex systems were built (51 workflow/rule/skill files, agent optimizer guide,
glossary) and later deleted when they proved to be unused overhead. The lesson:
start with the minimum viable instruction and add complexity only when agents
demonstrably need it. This validates the "radical simplicity" principle and the
CLAUDE.md best practice of keeping instruction files under 300 lines.

**3. Identity is hard** (threads: #49 → #51 → #77 → #84 → #92 → #124 → #144 → #195)
Eight issues across the workspace's history deal with agents getting their own
identity wrong — copying example model names, attributing commits to the wrong
author, reporting stale model names. The problem keeps recurring because identity
detection is environment-dependent and fragile. Open issue #124 still tracks this.

**4. Pre-commit fragility** (threads: #198 → #212 → #213 → #172)
Pre-commit hooks are the workspace's primary fast-feedback mechanism, but they're
fragile: not installed by default (#198), not on PATH for agents (#212), bypassed
by `--no-verify` (#241), and no CI equivalent for some checks (#172). The defense-
in-depth recommendation (§7f) directly addresses this pattern.

**5. Workspace reusability as a design goal** (threads: #132 → §8 → this research)
The intent to make this workspace generic (not Project11-specific) has been explicit
since #132 and is now formalized in §8. But it requires ongoing discipline —
project-specific content keeps creeping into workspace-level files (the glossary
had UNH Marine Autonomy terms, for example).

---

## Appendix B: External Analysis — Osmani's "How to Write a Good Spec for AI Agents"

**Source**: Addy Osmani, "How to Write a Good Spec for AI Agents" (January 2026).
Published on [O'Reilly Radar](https://www.oreilly.com/radar/how-to-write-a-good-spec-for-ai-agents/),
[addyosmani.com](https://addyosmani.com/blog/good-spec/), and
[Substack](https://addyo.substack.com/p/how-to-write-a-good-spec-for-ai-agents).

This appendix compares Osmani's framework (targeted at individual developers using AI
coding agents) against our workspace infrastructure (targeted at multi-agent teams in
an established ROS 2 workspace). The goal: identify ideas we can borrow to improve our
system.

> **See also**: [Appendix D](RESEARCH_ISSUE-249_APPENDIX-D.md) provides a deeper dive
> into the broader spec-driven development movement — GitHub Spec Kit architecture,
> Amazon Kiro hooks, the Scott Logic critique, and strategic adoption recommendations.

### B-ext.1 Article Summary

Osmani's core framework, informed by **GitHub's analysis of 2,500+ agent configuration
files**, proposes five principles for writing effective AI agent specs:

1. **Start with a high-level vision, let the AI draft the details** — provide a "product
   brief" and let the agent elaborate into a full spec. Human controls direction, AI
   handles expansion.

2. **Structure the spec like a professional PRD** — cover six core areas: Commands,
   Testing, Project Structure, Code Style, Git Workflow, and Boundaries.

3. **Break tasks into modular prompts (divide & conquer)** — feed only the relevant
   section of the spec for each task. Refresh context per major task to avoid stale or
   irrelevant information.

4. **Use Plan Mode and spec-driven development** — start in read-only mode, draft and
   refine the spec until no ambiguity remains, save as `SPEC.md`, then execute
   incrementally. The spec persists between sessions as a context anchor.

5. **Keep it goal-oriented and make it a living document** — focus on *what* and *why*
   (user stories, acceptance criteria), not detailed *how*. Update the spec as decisions
   are made.

Additional patterns from the article:

- **Three-tier boundary system** (from the GitHub analysis): categorize rules as
  "Always do" (autonomous), "Ask first" (human approval), or "Never do" (hard stops).
  More effective than flat rule lists.
- **Self-audit instruction**: "After implementing, compare the result with the spec and
  confirm all requirements are met. List any spec items not addressed."
- **LLM-as-Judge**: Use a second agent to review the first agent's output against the
  spec's quality guidelines. Effective for subjective criteria (code style, readability,
  architectural coherence).
- **Conformance suites**: Language-independent tests (often YAML-based) that any
  implementation must pass. Include in the spec's "Success" section.

### B-ext.2 Overlap Analysis

| Topic | Osmani's Framing | Our Research | Difference |
|-------|-----------------|--------------|------------|
| Structured instruction files | "6 core areas" (commands, testing, project structure, code style, git workflow, boundaries) from 2,500+ config analysis | §3 (AGENTS.md), §5 (CLAUDE.md best practices) | Nearly identical conclusions. Our "keep CLAUDE.md under 300 lines" matches his "attention budget" concern. His empirical base (2,500 configs) validates our approach |
| Living document | Update spec as decisions are made; version-control it | §7c (ADRs), §7h (transparency), §5c ("evolve organically") | We formalize this as ADRs + architecture checkpoints rather than a monolithic spec |
| Plan before execute | Plan Mode (read-only), draft spec, then implement incrementally | Already in practice — `worktree_create.sh --plan-file`, draft PRs with plans | Same idea, different mechanism |
| LLM-as-Judge | Second agent reviews against spec quality guidelines | §C5 (Promptfoo, DeepEval, AgentIF, constraint decomposition) | We go much deeper — full testing architecture, academic papers, cost analysis |
| Modular context feeding | Feed only the relevant spec section per task | §5c (`@imports` for progressive disclosure, `.claude/rules/` for topic-specific rules) | Same principle, different implementation |
| Version-control specs | Commit the spec to the repo | Implicit throughout | Aligned |

### B-ext.3 Borrowable Ideas (Actionable for This Workspace)

#### Idea 1: Always / Ask First / Never Taxonomy for CLAUDE.md

**What Osmani proposes**: A three-tier boundary system derived from GitHub's analysis of
2,500+ agent configuration files. Rules are categorized as:

- **Always do** (agent proceeds autonomously): "Always run tests before commits."
- **Ask first** (agent pauses for human): "Ask before modifying database schemas."
- **Never do** (hard stop): "Never commit secrets or API keys."

**Why this matters for us**: CLAUDE.md is currently a flat list of rules with inconsistent
emphasis (`IMPORTANT`, `NEVER`, `YOU MUST`). The three-tier system provides a cleaner
structure that maps directly to our enforcement hierarchy (§7g):

| Tier | Maps to enforcement layer |
|------|--------------------------|
| Always | Hooks enforce, CI validates |
| Ask First | Agent pauses, wrapper scripts confirm |
| Never | CI blocks, branch protection prevents |

**Concrete application**:

```markdown
## Boundaries

### Always (proceed autonomously)
- Run pre-commit hooks before committing
- Use worktrees for all feature work
- Include AI signature on all GitHub artifacts
- Reference issue numbers in branch names and PRs
- Read AGENTS.md / .agents/README.md before modifying a project repo
- Use `--body-file` for multiline GitHub CLI content

### Ask First (get human approval)
- Before modifying CLAUDE.md, AGENTS.md, or other instruction files
- Before adding/removing a workspace layer
- Before changing CI pipeline configuration
- Before adding new dependencies to package.xml
- Before modifying branch protection rules

### Never (hard stops)
- Never commit to main (branch is protected)
- Never use `git checkout <branch>` (env.sh blocks it)
- Never skip hooks with `--no-verify`
- Never push to a branch you didn't create
- Never commit secrets, API keys, or credentials
- Never document from assumptions — verify against source code
```

**Priority**: High — simple restructuring with immediate clarity benefit.

#### Idea 2: Self-Audit Instruction

**What Osmani proposes**: Append to agent instructions: "After implementing, compare the
result with the spec and confirm all requirements are met. List any spec items not
addressed." The agent outputs a short checklist after each implementation, catching
omissions before tests even run.

**Why this matters for us**: Agents currently declare tasks "done" without systematic
self-checking. Our CI catches structural problems, but intent-level gaps (did I actually
address all the requirements in the issue?) go undetected until human review.

**Concrete application** (add to CLAUDE.md):

```markdown
## Post-Task Verification

Before marking a task complete or opening a PR for review:
1. Re-read the issue description and any linked work plan
2. Compare your changes against each stated requirement
3. List any requirements not yet addressed
4. If items remain, either complete them or explain in the PR why they were deferred
```

**Priority**: High — zero implementation cost, addresses a real gap.

#### Idea 3: SPEC.md as Persistent Worktree Anchor

**What Osmani proposes**: Save the refined spec as `SPEC.md` in the repo. It persists
between sessions, anchoring the agent whenever work resumes. This mitigates the
"forgetfulness" problem where long conversations or agent restarts lose context.

**Why this matters for us**: Our `worktree_create.sh --plan-file` already creates a
draft PR with the plan attached, and work plans live in `.agent/work-plans/`. But agents
don't always re-read the plan when resuming work. The plan is "out there" but not
prominently in the agent's path.

**Concrete application**: When `worktree_enter.sh` is sourced, check for a plan file
associated with the worktree's issue and remind the agent:

```bash
# In worktree_enter.sh
if [ -f ".agent/work-plans/PLAN_ISSUE-${WORKTREE_ISSUE}.md" ]; then
    echo "📋 Work plan available: .agent/work-plans/PLAN_ISSUE-${WORKTREE_ISSUE}.md"
    echo "   Read it before starting work."
fi
```

And add to CLAUDE.md:

```markdown
## Entering a Worktree

After sourcing worktree_enter.sh, check for a work plan or SPEC file:
- `.agent/work-plans/PLAN_ISSUE-${WORKTREE_ISSUE}.md`
- `SPEC.md` in the worktree root
If present, read it before starting work — it contains the scope, plan, and
acceptance criteria for this task.
```

**Priority**: Medium — requires minor script change + instruction update.

#### Idea 4: Conformance Framing for Instruction Testing

**What Osmani proposes**: Include conformance criteria directly in the spec's "Success"
section. Language-independent tests (often YAML-based) that any implementation must pass.

**How this differs from our approach**: Our §C5 evaluates full testing frameworks
(Promptfoo, DeepEval) as separate infrastructure. Osmani frames conformance as part of
the spec itself — simpler and more immediate.

**Concrete application**: For our instruction files, start with a lightweight conformance
file before building the full Promptfoo pipeline:

```yaml
# .agent/conformance/instruction-rules.yaml
# Deterministic checks — run as a CI step or pre-commit hook

rules:
  - name: "Branch names follow pattern"
    type: deterministic
    check: "git branch --show-current | grep -qE '^(feature/(issue-|ISSUE-)|claude/|main$)'"

  - name: "AI signature present in PR body"
    type: deterministic
    check: "gh pr view --json body --jq '.body' | grep -q 'Authored-By'"

  - name: "No commits to main by agents"
    type: deterministic
    check: "! git log main --author='Claude' --oneline | grep -q ."

  - name: "--body-file used instead of --body for multiline"
    type: deterministic
    check: "! git diff HEAD~1 -- '*.sh' | grep -q '\\-\\-body '"

  - name: "Commit message explains why, not just what"
    type: llm-judged
    prompt: "Does this commit message explain WHY the change was made, not just WHAT was changed?"
```

This is a stepping stone to the full Promptfoo architecture (§C5) — start with
deterministic checks that run in CI today, add LLM-judged checks when the framework is
ready.

**Priority**: Medium — bridges the gap between "no testing" and "full eval framework."

#### Idea 5: Product Brief → AI-Generated Spec Pattern

**What Osmani proposes**: Start with a short goal statement (3-5 sentences). Let the AI
elaborate it into a full spec with implementation details, test plans, and edge cases.
Review and refine. Then save and execute incrementally.

**Why this matters for us**: We do something similar (issue descriptions → work plans via
`--plan-file`), but the pattern isn't formalized. The key insight is that the elaboration
step should be explicit and named — "draft the spec" — not just an implied part of
planning.

**Concrete application**: Formalize the two-phase pattern in agent instructions:

```markdown
## Starting a New Task

### Phase 1: Draft the Spec
1. Read the issue description
2. Read relevant architecture docs (ARCHITECTURE.md, ADRs, .agents/README.md)
3. Generate a detailed spec: scope, approach, acceptance criteria, test plan
4. Save as work plan via `--plan-file` (creates draft PR for review)
5. Wait for approval before proceeding

### Phase 2: Execute Incrementally
1. Work through the spec one section at a time
2. Feed only the relevant context for each sub-task
3. Self-audit against the spec after completing each section
4. Update the spec if decisions change during implementation
```

**Priority**: Low-Medium — mostly formalizes existing practice, but the explicit
"self-audit after each section" and "update spec if decisions change" steps add value.

### B-ext.4 What Our Research Covers That Osmani Doesn't

Osmani targets individual developers on greenfield projects. Our research addresses
infrastructure concerns for multi-agent teams in an established workspace:

| Our research topic | Why Osmani doesn't cover it |
|---|---|
| Multi-agent coordination & worktree isolation (§4, §C4) | Single-developer focus |
| Enforcement hierarchy: instructions → hooks → CI → containers (§7f, §7g, §9) | Assumes developer compliance |
| Architecture documentation lifecycle: ADRs, CI-enforced drift detection (§7a-7h) | Not an infrastructure concern for individual developers |
| Workspace archaeology: mining git history for lost intent (§7c, Appendix A) | Assumes greenfield or recent projects |
| Workspace/project boundary: package.xml principle for instruction portability (§8) | Single-repo assumption |
| Technology landscape scan: 60+ tools evaluated (Appendix C) | Article, not reference guide |
| Agent compliance gap analysis (§7g) | Assumes agents follow instructions |
| Diff-time rationale (§7h) | Not an approval-workflow concern |
| ROS 2 / robotics-specific tooling (§6) | Web/app development focus |

### B-ext.5 Recommendations Integration

The five borrowable ideas are incorporated into the main recommendations (§10) as
follows:

| Idea | Incorporated into | Phase |
|------|------------------|-------|
| Always/Ask/Never taxonomy | Phase 3 (Documentation) & Phase 4 (Align Instructions) | 3-4 |
| Self-audit instruction | Phase 4 (Align Instructions) & Phase 6 (Compliance/Trust) | 4, 6 |
| SPEC.md worktree anchor | Phase 2 (Simplify Interface) | 2 |
| Conformance framing | Phase 5 (Architecture Documentation) or standalone | 5 |
| Product brief → AI spec | Phase 4 (Align Instructions) | 4 |

---

## Appendix C: Technology Landscape Scan — Existing Tools and Emerging Methodologies

Systematic scan (February 2026) of tools, frameworks, and research that overlap with
the capabilities this workspace is building. Organized by problem domain. For each tool:
what it does, maturity, and whether we should adopt, monitor, or skip it.

### C1. Git Archaeology & Architecture Recovery from History

**The gap we identified**: No existing tool mines the "rejection history" (closed-without-
merge PRs, reverted commits, deleted files) to reconstruct architectural rationale.

| Tool | What It Does | Maturity | Action |
|------|-------------|----------|--------|
| **CodeScene** | Behavioral code analysis: hotspots, temporal coupling, organizational analysis from git history. Grounded in "Your Code as a Crime Scene" methodology | Production (commercial) | **Monitor** — expensive but the temporal coupling analysis is exactly what you'd want for identifying hidden architectural dependencies |
| **Code-Maat** | Open-source engine behind the Crime Scene methodology. Git log forensics | Stable | **Evaluate** — free, could complement our archaeology method for quantitative analysis |
| **iCODES** | LLM-powered semantic search over commit intent. Builds SQLite DB of commit insights | Early | **Evaluate** — closest tool to our archaeology method; semantic commit search could accelerate the process |
| **Microsoft Code Researcher** | Deep research agent for multi-step reasoning over code + commit history. Tested on Linux kernel | Research prototype (2025) | **Monitor** — methodology paper is valuable; tool not publicly available |
| **RepoAgent** | LLM-powered documentation generation via global structure analysis + git change tracking | Published (EMNLP 2024) | **Evaluate** — could generate per-package docs from code; used on 270k LOC codebases |
| **Git-Native Semantic Memory** | Auto-capture architectural decisions as git notes with semantic embeddings. Zero infrastructure | Prototype (Dec 2025) | **High interest** — stores decisions in git itself; sub-10ms retrieval; directly addresses our "lost rationale" problem |
| **MCP ADR Analysis Server** | AI-powered ADR analysis exposed as MCP tools for coding agents. Detects tech stacks, generates ADRs from requirements | Active OSS | **Evaluate** — could give Claude Code native ADR generation capabilities via MCP |
| **GitHub Actions + LLM ADR Pipeline** | Methodology: collect PR metadata, call LLM to draft ADR, commit to `/docs/adr/` for review | Pattern (2025) | **Adopt pattern** — implementable with standard GH Actions; every architectural PR ships with a draft ADR |
| **GitEvo** | Python tool combining git + tree-sitter to track how code constructs evolve over time | Early (MSR 2026) | **Monitor** — syntax-tree level evolution tracking; Python/JS/TS/Java only |
| **PyDriller** | Python framework for mining git repos programmatically | Mature | **Adopt as library** — building block for any custom archaeology tooling |

**Key finding**: Our workspace archaeology method (§7c) fills a gap that no existing tool
covers — specifically the systematic mining of *negative space* (what was rejected, removed,
or abandoned). The closest tools focus on what *exists* in the codebase, not what was tried
and discarded. The "Vibe ADR" pattern (ADRs as agent-readable decision graphs) aligns with
our vision of ADRs serving both human and agent audiences.

### C2. AI Agent Guardrails & Compliance Enforcement

**The gap we identified**: The gap is between writing rules and verifying compliance. Many
tools help define rules; very few verify that agents actually followed them.

| Tool | What It Does | Maturity | Action |
|------|-------------|----------|--------|
| **Claude Code Hooks** (PreToolUse/PostToolUse/Stop) | Deterministic lifecycle hooks that can block, modify, or validate agent actions. "Block-at-Submit" recommended pattern | Production | **Already in use** — expand beyond `env.sh` guardrail to cover commit format, issue references, worktree enforcement |
| **Ruler** | CLI that maintains single `.ruler/` source of truth and distributes rules to 30+ agent config formats (Claude, Cursor, Copilot, Aider, etc.) | Active (v0.3.32) | **Adopt if multi-agent** — eliminates rule duplication across tools |
| **AGENTS.md standard** | Community standard for cross-agent instruction files using RFC 2119 keywords. Native in Codex CLI, OpenCode | Early | **Align** — our `.agents/README.md` convention already fits; formalizing as `AGENTS.md` adds portability |
| **Open Policy Agent (OPA)** | General-purpose policy engine (CNCF graduated) using Rego declarative language. Decouples policy from enforcement | Production | **Evaluate** — hook sends tool metadata as JSON to OPA; OPA evaluates against versioned Rego policies. Provides testable, auditable policy layer beyond soft prompt rules |
| **LlamaFirewall CodeShield** | Static analysis of LLM-generated code using Semgrep/regex across 8 languages. 96% precision | Production (Meta) | **Evaluate for CI** — catches insecure AI-generated code patterns |
| **Superagent Safety Agent** | Separate "safety agent" that evaluates prompts, tool calls, and responses against policies | Active OSS | **Monitor** — the pattern of a compliance agent reviewing another agent's work is architecturally interesting |
| **Policy-as-Prompt** | Research: auto-extract enforceable policies from design artifacts into testable classifiers | Research (NeurIPS 2025) | **Monitor** — converting CLAUDE.md rules into testable classifiers would close the rules-verification gap |
| **Guardrails AI** | Python framework for I/O guards with community validator hub | Production (v0.9) | **Skip** — designed for LLM app outputs, not coding agent workspace compliance |
| **NVIDIA NeMo Guardrails** | Colang DSL for dialogue flows and programmable rails | Production | **Skip** — conversational AI focus, wrong abstraction for coding agents |
| **Amazon Kiro Hooks** | File-change-triggered validation prompts (e.g., "when `src/models/` changes, run validation"). Bridges gap between pre-commit hooks (too late) and real-time monitoring (too expensive) | Production (GA Dec 2025) | **Adopt pattern** — Claude Code PostToolUse hooks can implement file-change-triggered validation; validates the reactive enforcement pattern. See Appendix D, §D4 |

**Key findings**:
1. The "rules file" pattern (`CLAUDE.md`, `.cursorrules`, etc.) is now universal — our approach is mainstream
2. Frontier LLMs follow ~150-200 instructions consistently; priority saturates beyond that. Our CLAUDE.md must stay focused
3. Deterministic enforcement (hooks + CI) must complement soft rules — agents bypass hooks, can't bypass CI + branch protection
4. No existing tool validates "did this agent follow its CLAUDE.md correctly?" — this remains an open problem
5. The "block-at-submit" hook pattern (let agent work, validate at commit) is the recommended approach
6. File-change-triggered enforcement (Kiro pattern) is becoming an industry standard alongside lifecycle hooks

### C3. Architecture Drift Detection & Layer Boundary Enforcement

**The gap we identified**: No tool in the ROS 2 ecosystem validates package.xml dependencies
against architectural layer boundaries.

| Tool | What It Does | Maturity | Action |
|------|-------------|----------|--------|
| **import-linter** | Python architecture contracts: Layers, Independence, Forbidden Modules. Builds import graph, checks edges against rules | Production | **Adopt** — directly usable for Python packages in our workspace |
| **PyTestArch** | ArchUnit-inspired architecture testing as pytest tests | Active | **Evaluate** — alternative to import-linter with pytest integration |
| **deptry** | Finds unused/missing/misclassified Python dependencies. Rust-powered parser | Production | **Adopt** — catches dependency hygiene issues in Python packages |
| **IWYU (Include-What-You-Use)** | Enforces that each C++ file directly includes what it uses. CMake integration | Production (Clang 18+) | **Adopt** — include hygiene for C++ packages |
| **CppDepend** | Customizable C++ code queries for architecture and coding standards. Dependency analysis, cycle detection | Production (commercial) | **Evaluate** — most capable C++ architecture tool, but commercial |
| **Sonargraph-Architect** | Architecture checks via DSL for C/C++, Python, Java. Quality gates with baselines | Production (commercial) | **Monitor** — supports our languages but expensive |
| **SonarQube Architecture-as-Code** | Define architecture in YAML; detect drift in CI | New (2025, Java only; C++ "under consideration") | **Monitor** — when C++/Python support ships, this becomes very relevant |
| **Structurizr** | C4 model DSL for version-controlled architecture diagrams | Production | **Adopt for docs** — architecture diagrams as code, rendered from DSL |
| **colcon graph** | ROS 2 native package dependency visualization (DOT/Graphviz) | Production | **Already available** — use as input to custom boundary checker |
| **Nx Module Boundaries** | Tag-based dependency constraints (JS/TS) | Production | **Use as design blueprint** — the `depConstraints` model maps perfectly to ROS 2 workspace layers |
| **Bazel Visibility** | Package-level visibility controls | Production | **Use as design blueprint** — the visibility model informs our layer boundary design |
| **catkin_lint** | Validates package.xml vs CMakeLists.txt consistency (ROS 1 only) | Stable (ROS 1) | **Gap** — no ROS 2 equivalent exists; this is a significant ecosystem gap |

**Key findings**:
1. **Must build**: A custom layer boundary checker that parses `package.xml`, maps packages to layers by workspace location, validates dependency edges against rules in a `layer_architecture.yaml`. No existing tool does this
2. Python tooling is rich (`import-linter`, `PyTestArch`, `deptry`) — adopt directly
3. C++ architecture testing has no open-source ArchUnit equivalent; options are commercial tools or custom scripts
4. The Nx/Bazel conceptual models are the right design blueprints for our custom checker
5. `catkin_lint` for ROS 2 is a missing piece the whole ecosystem needs

### C4. Multi-Agent Coordination & Workspace Orchestration

**The gap we identified**: No tool combines worktree isolation + advisory locks + agent
messaging + task orchestration for a multi-repo workspace.

| Tool | What It Does | Maturity | Action |
|------|-------------|----------|--------|
| **Clash** | Detects merge conflicts between worktree pairs using `git merge-tree` simulation. JSON output, watch mode, pre-edit hooks | Early OSS (Rust) | **Evaluate now** — drop-in conflict detection for our existing worktrees |
| **MCP Agent Mail** | Async agent coordination via MCP: identities, inboxes, advisory file reservations, searchable history. Git + SQLite backed | Active OSS | **Evaluate now** — advisory reservations map well to our worktree model; MCP means native Claude Code integration |
| **Claude Agent SDK** | Programmatic access to Claude Code's tools and agent loop. Subagent parallelization with context isolation | Production (Anthropic) | **Adopt for orchestration** — the native way to build a multi-agent orchestrator that spawns subagents in worktrees |
| **CCManager** | CLI for managing multiple AI coding sessions across worktrees. Session context transfer, hooks | Active OSS | **Evaluate** — context transfer between worktrees could integrate with our scripts |
| **ccswarm** | Specialized agent pools (Frontend, Backend, DevOps, QA) using Claude Code CLI | Active OSS | **Evaluate** — agent pool concept maps to ROS 2 subsystems (navigation, perception, infrastructure) |
| **Agent-MCP** | Multi-agent framework with hard file locks, task dependencies, real-time dashboard | OSS (AGPL) | **Monitor** — hard locks are too restrictive for worktree-isolated agents |
| **Claude-Flow** | Third-party orchestration claiming 64-agent systems, 84.8% SWE-Bench | Active (claims unverified) | **Monitor skeptically** — extraordinary claims need independent verification |
| **Microsoft Agent Framework** | Unified SDK for multi-agent with enterprise governance. Task adherence, PII detection | Public preview | **Monitor** — relevant if heterogeneous agent teams (Claude + Copilot) are needed |
| **GitHub Spec Kit** | Agent-agnostic spec-driven development toolkit. Four-phase gated workflow (Specify → Plan → Tasks → Implement), `/analyze` consistency validator, per-feature spec directories, constitution pattern | Experimental (40k+ stars) | **Evaluate selectively** — full four-phase ceremony is overkill (Scott Logic found 1,600 lines of spec overhead for moderate tasks). Cherry-pick: `/analyze` validator concept, constitution pattern, per-feature spec directories. See Appendix D, §D3 |
| **CrewAI** | Role-based multi-agent with workflows and governance | Production | **Skip** — no git/code/worktree awareness |
| **MetaGPT** | Simulates software company (PM, architect, engineers) from one-line spec | Research-grade | **Skip** — greenfield generation, not incremental development |

**Key architectural lessons from the ecosystem**:

1. **Advisory locks > hard locks**: Multiple projects converged on this. Hard locks cause
   head-of-line blocking; advisory reservations surface intent while allowing progress.
   Soft locks + batching reduced duplicate work from 2.3% to 0.2% (Skywork study)

2. **Cursor's three-role hierarchy**: Equal-status agents with locking failed. Optimistic
   concurrency also failed (agents became risk-averse). The working pattern: **Planners**
   (explore + create tasks) → **Workers** (execute without coordinating) → **Judges**
   (evaluate at cycle end)

3. **Worktree isolation is now industry standard**: Cursor 2.0 (8 parallel agents in
   worktrees), OpenAI Codex (auto-worktree per agent), and our workspace all converged
   independently on the same pattern

4. **Runtime isolation beyond filesystem**: DoltHub discovered that worktrees share ports,
   databases, Docker daemon. For ROS 2 this means shared `ROS_DOMAIN_ID` and DDS discovery.
   Container isolation (#229) addresses this

5. **Cost-aware orchestration**: Anthropic's Plan-and-Execute pattern (expensive model
   plans, cheap model executes) reduces costs 90%. No open-source tool implements this
   with worktree awareness yet

6. **Google DORA 2025 warning**: 90% AI adoption increase correlates with 9% bug rate
   climb, 91% more code review time, 154% larger PRs. Coordination quality matters more
   than parallelism throughput

### C5. Instruction/Prompt Regression Testing

**The gap we identified**: When CLAUDE.md changes, nothing verifies that agents still
follow the rules correctly. Appendix A identified this as the most significant
undelivered capability.

**Critical finding**: No tool is specifically designed for testing CLAUDE.md / .cursorrules
/ AGENTS.md files. However, the general-purpose prompt evaluation ecosystem has matured
significantly, and several frameworks can be adapted.

| Tool | What It Does | Maturity | Action |
|------|-------------|----------|--------|
| **Promptfoo** | Declarative YAML test cases with mixed assertions (exact, regex, LLM-as-judge, semantic similarity). Official GitHub Action with PR score breakdowns. [Evaluate Coding Agents guide](https://www.promptfoo.dev/docs/guides/evaluate-coding-agents/) | Production (17k stars, used by Anthropic internally) | **Adopt** — strongest fit for instruction regression testing. YAML-driven, CI-native, cost caching |
| **DeepEval** | Python/pytest with 50+ metrics including G-Eval (custom NL criteria scored by LLM). Built-in IFEval benchmark. Regression tracking with green/red rows | Production (Apache 2.0) | **Evaluate** — alternative if pytest integration preferred. G-Eval maps directly to rule compliance scoring |
| **Evidently AI** | GitHub Action for LLM testing: run agent, evaluate, fail CI on regression. 100+ built-in evals | Production (Apache 2.0) | **Evaluate** — GH Action workflow fits our CI pipeline well |
| **Arize prompt-learning** | LLM-as-judge evaluates coding agent output, auto-rewrites CLAUDE.md for accuracy. 5-15% SWE-Bench improvement. Has [Claude Code-specific code](https://github.com/Arize-ai/prompt-learning/tree/main/coding_agent_rules_optimization/claude_code) | Published | **Evaluate** — only published work targeting CLAUDE.md optimization with automated eval |
| **Braintrust AutoEvals** | Model-graded evaluation with GitHub Action posting PR score comparisons. Production traces → test cases | Production (commercial + OSS) | **Monitor** — PR integration excellent but OSS component thinner |
| **Inspect AI** (UK AISI) | Sandboxed agent execution (Docker/K8s). 100+ pre-built evals. Most rigorous safety framework | Production (govt-backed) | **Monitor** — right model for verifying agents *actually* use worktrees, but high overhead |
| **Anthropic Bloom** | Auto-generates adversarial scenarios from behavior descriptions. Measures compliance frequency/severity | OSS (Dec 2025) | **Monitor** — designed for safety behaviors, but architecture generalizes to workspace rules |

**Key academic research informing the approach:**

| Paper | Key Contribution | Relevance |
|-------|-----------------|-----------|
| **AgentIF** (NeurIPS 2025, [code](https://github.com/THU-KEG/AgentIF)) | First benchmark for instruction following in agentic scenarios. 707 instructions, avg 11.9 constraints. Even best models follow <30% of instructions perfectly | **Very high** — validates need; methodology (extract constraints → per-constraint eval → CSR/ISR metrics) directly applicable |
| **InFoBench** (ACL 2024) | Decomposed Requirements Following Ratio (DRFR) — breaks complex instructions into yes/no verification questions | **High** — CLAUDE.md is a complex instruction with many sub-requirements; DRFR decomposition maps directly |
| **IFEval** (Google 2023) | "Verifiable instructions" checked deterministically (format, keyword, structure) | **High** — many CLAUDE.md rules are deterministically verifiable: branch names, signatures, `--body-file` usage |
| **LLMBar** (ICLR 2024) | Meta-evaluation: how reliable is LLM-as-judge for scoring instruction compliance? | **Medium** — validates reliability of our proposed evaluation approach |

**Recommended implementation architecture:**

1. **Constraint decomposition** (InFoBench/AgentIF method): Break each CLAUDE.md rule
   into individually testable constraints. Classify each as:
   - **Deterministic** (branch naming pattern, AI signature presence, `--body-file` usage)
   - **LLM-judgable** (commit message quality, design rationale, "why" vs "what")
   - **Behavioral** (actually uses worktrees, refuses to commit to main — requires sandbox)

2. **Promptfoo for test execution**: Define constraints as YAML test cases with mixed
   assertion types. Deterministic assertions where possible, model-graded for semantic
   rules. Run via GitHub Action on every PR modifying instruction files

3. **CI gate**: Post results as PR comment showing per-constraint scores. Block merge
   if compliance drops below threshold. Cache LLM calls to manage costs

4. **Feedback loop**: Real agent compliance failures become new test cases. Re-evaluate
   full suite against new model versions periodically

**Practical challenges:**
- **Cost**: Agentic test cases cost $0.10-$0.30 each, take 30-120 seconds. Full compliance
  suites need careful test selection and caching
- **Non-determinism**: Same instructions produce different behavior across runs. Multiple
  trials per test case are needed, increasing cost
- **LLM-as-judge reliability**: Even frontier models struggle to judge conditional/nested
  constraints (AgentIF finding)

### C6. Cross-Cutting Synthesis: What to Adopt, Build, or Skip

**Adopt now** (existing tools that directly solve our problems):
| Tool | Problem Solved | Effort |
|------|---------------|--------|
| `import-linter` | Python architecture boundary enforcement | Low — pip install + config |
| `deptry` | Python dependency hygiene | Low — pip install + CI step |
| IWYU | C++ include hygiene | Low — CMake integration exists |
| PyDriller | Library for custom archaeology tooling | Low — pip install |
| Promptfoo | Instruction regression testing framework | Medium — YAML config + GH Action |
| `AGENTS.md` alignment | Cross-agent instruction portability | Low — rename/formalize existing files |
| GitHub Actions + LLM ADR pipeline | Auto-draft ADRs from architectural PRs | Medium — GH Action + LLM API |
| Structurizr DSL | Architecture diagrams as code | Low — DSL + Docker for rendering |

**Build custom** (no existing tool covers these):
| Tool | Problem Solved | Effort |
|------|---------------|--------|
| ROS 2 layer boundary checker | Validate package.xml deps against `layer_architecture.yaml` | Medium — Python script parsing XML + graph analysis |
| CLAUDE.md constraint decomposition | Break rules into testable constraints (deterministic + LLM-judged) | Medium — one-time analysis + Promptfoo YAML |
| Archaeology automation | Scripted version of our manual archaeology method | Medium — PyDriller + LLM + GitHub API |

**Evaluate soon** (promising, need hands-on testing):
| Tool | Why Evaluate | Risk |
|------|-------------|------|
| Clash | Drop-in worktree conflict detection | Early-stage, may have rough edges |
| MCP Agent Mail | Advisory file reservations via MCP | Adds infrastructure dependency |
| Claude Agent SDK | Native multi-agent orchestrator | Requires orchestrator development |
| DeepEval / Evidently AI | Alternative eval frameworks for instruction testing | May overlap with Promptfoo |
| Arize prompt-learning | Auto-optimize CLAUDE.md via eval feedback loop | Requires LLM API costs |
| OPA (Rego policies) | Formal, testable policy enforcement via hooks | Rego learning curve |
| iCODES | Semantic search over commit intent | Early-stage |
| Git-native semantic memory | Auto-capture decisions as git notes | Prototype only |
| Ruler | Multi-agent rule distribution | Only needed if we adopt multiple agent tools |

**Monitor** (interesting but not ready or not our problem yet):
- CodeScene (commercial, expensive)
- SonarQube Architecture-as-Code (waiting for C++/Python)
- Microsoft Agent Framework (wrong ecosystem unless we go multi-vendor)
- Policy-as-Prompt research (auto-extract rules into classifiers)
- Sonargraph-Architect (commercial, expensive)
- Inspect AI / Anthropic Bloom (safety eval frameworks — useful if we need sandboxed testing)
- AEBOP (Agentic Engineering Body of Practices — emerging standard)

**Skip** (wrong problem or wrong ecosystem):
- Guardrails AI / NeMo Guardrails (chatbot guardrails, not coding agent workspace)
- CrewAI / MetaGPT / OpenAI Agents SDK (no git/workspace awareness, or wrong vendor)
- GitButler virtual branches (incompatible with ROS 2 build requirements)
- LangSmith (LangChain ecosystem lock-in)

---

## Appendix D: Spec-Driven Development and AI Agent Specification Practices

See [RESEARCH_ISSUE-249_APPENDIX-D.md](RESEARCH_ISSUE-249_APPENDIX-D.md) — evaluates
the spec-driven development movement (Addy Osmani's O'Reilly article, GitHub Spec Kit,
Amazon Kiro) against the workspace's existing workflow. Key findings: the three-tier
boundary system for agent instruction files, Spec Kit's `/analyze` consistency validator,
and the recommendation to cherry-pick concepts rather than adopt the full SDD framework.

> **Note**: Appendix B covers the Osmani article's direct overlap with this report's
> existing findings and proposes 5 concrete borrowable ideas. This appendix (D) goes
> deeper into the broader SDD movement — Spec Kit architecture, Kiro hooks, the Scott
> Logic critique — and provides strategic adoption recommendations. Read B for "what
> to borrow," D for "what to avoid and why."

---

**Authored-By**: `Claude Code Agent`
**Model**: `Claude Opus 4.6`
