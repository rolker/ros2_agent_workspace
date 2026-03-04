# Research Digest: Workspace

<!-- Last updated: 2026-03-04 -->
<!-- If older than 30 days, consider running /research --refresh; entries older than 90 days should be flagged for review -->

## Command Runner Alternatives to Make

**Added**: 2026-02-27 | **Sources**: [just](https://github.com/casey/just), [Task](https://github.com/go-task/task), [mise](https://github.com/jdx/mise)

Key takeaways:
- `just` is the strongest candidate for replacing Make as a command runner — closest syntax, built-in `just --list`, no `.PHONY` needed, multi-language recipes
- `just-mcp` provides an MCP adapter, enabling any MCP-compatible agent to invoke workspace commands
- `task` (Go/YAML) offers namespace includes but is more verbose; `mise` (Rust/TOML) bundles tool version management but adds little for ROS 2 where `apt`/`rosdep` manage toolchains
- `colcon defaults.yaml` can absorb build flags currently hardcoded in scripts, regardless of which runner is chosen

**Relevance**: The workspace uses Make purely as a command runner, not for dependency-based builds. Switching to `just` would reduce boilerplate and provide a pathway to MCP-accessible commands.

---

## AGENTS.md — Cross-Platform Agent Instruction Standard

**Added**: 2026-02-27 | **Sources**: [agents.md spec](https://agents.md/), [GitHub repo](https://github.com/agentsmd/agents.md), [OpenAI Codex guide](https://developers.openai.com/codex/guides/agents-md/), [Agentic AI Foundation](https://openai.com/index/agentic-ai-foundation/), [deep dive](https://prpm.dev/blog/agents-md-deep-dive), [sync strategy](https://kau.sh/blog/agents-md/)

Key takeaways:
- Vendor-neutral spec stewarded by the Agentic AI Foundation (Linux Foundation), with contributions from OpenAI, Anthropic, and others
- 60,000+ open-source projects adopted; measured 28.6% median runtime reduction, 16.6% token reduction
- Supported by Codex, Cursor, Devin, Gemini CLI, GitHub Copilot, Jules, VS Code, and more
- Recommended size: ≤150 lines; closest `AGENTS.md` to the edited file wins; user prompts override all
- For multi-tool workspaces, the `@AGENTS.md` import pattern in framework-specific files (e.g., CLAUDE.md) is preferred over symlinking

**Relevance**: Already adopted in this workspace. AGENTS.md consolidates shared rules; framework-specific files contain only agent-specific behavioral rules. This eliminated the previous four-file duplication problem.

---

## AI Agent Spec Writing Best Practices

**Added**: 2026-02-27 | **Sources**: [Osmani, "How to Write a Good Spec for AI Agents" (O'Reilly, Jan 2026)](https://www.oreilly.com/radar/how-to-write-a-good-spec-for-ai-agents/), [addyosmani.com](https://addyosmani.com/blog/good-spec/)

Key takeaways:
- Informed by GitHub's analysis of 2,500+ agent configuration files
- **Three-tier boundary system**: categorize rules as "Always" (autonomous), "Ask first" (human approval), or "Never" (hard stop) — more effective than flat rule lists
- **Self-audit instruction**: tell agents to compare results against the spec and list unmet items
- **SPEC.md as session anchor**: persistent spec file survives context resets across sessions
- **Conformance suites**: language-independent tests that any implementation must pass
- **Two-phase task pattern**: draft spec first (plan mode), then execute incrementally

**Relevance**: The Always/Ask/Never taxonomy was adopted directly into AGENTS.md. The self-audit pattern maps to our post-task verification checklist.

---

## Harness Engineering — Production-Scale Agent-First Development

**Added**: 2026-02-27 | **Sources**: [OpenAI blog (Lopopolo, Feb 2026)](https://openai.com/index/harness-engineering/), [Böckeler analysis (martinfowler.com)](https://martinfowler.com/articles/exploring-gen-ai/harness-engineering.html), [InfoQ coverage](https://www.infoq.com/news/2026/02/openai-harness-engineering-codex/)

Key takeaways:
- Team of 3–7 engineers built ~1M LOC product with zero human-written code over 5 months using Codex; 3.5 PRs/engineer/day
- The **harness** (environment, constraints, feedback loops) matters more than the agent's capabilities — early slowness came from an underspecified environment, not incapable models
- Three harness components: **context engineering** (instruction files), **architectural constraints** (linters, structural tests), **garbage collection** (background agents fighting entropy)
- AGENTS.md should be a ~150-line **map** (table of contents), not an encyclopedia — deep rules live in referenced knowledge files
- **Lint error messages as agent teaching**: custom linters should include remediation instructions so failures become learning context
- **Garbage collection agents**: background tasks that scan for drift and open small fix-up PRs on a recurring cadence
- **"Anything not in-context doesn't exist"**: only repo-local versioned artifacts are visible to agents; knowledge in chat, docs, or heads is invisible
- **"Corrections are cheap, waiting is expensive"**: favor fast feedback loops over blocking gates for agent-generated work

**Relevance**: Validates several workspace patterns (worktree isolation, progressive disclosure, artifact-based communication). The map-vs-encyclopedia principle guides AGENTS.md/CLAUDE.md sizing. Garbage collection agents are a future pattern once multi-agent workflows are adopted.

---

## Multi-Agent Engineering Patterns

**Added**: 2026-02-27 | **Sources**: [GitHub blog, "Multi-agent workflows often fail" (Feb 2026)](https://github.blog/ai-and-ml/generative-ai/multi-agent-workflows-often-fail-heres-how-to-engineer-ones-that-dont/)

Key takeaways:
- Core thesis: most multi-agent failures come from **missing structure**, not model capability — treat agents as distributed system components
- **Typed schemas** for inter-agent data: machine-checkable contracts (Zod, JSON Schema) prevent field name drift and ambiguous types
- **Action schemas** (discriminated unions): constrain what agents can do — prevents contradictory actions like one agent closing an issue another just opened
- **MCP as enforcement layer**: validates tool inputs/outputs before execution, making instruction-level rules mechanically enforceable
- **Distributed systems principles apply**: design for failure, validate at boundaries, constrain before scaling, log intermediate state, design for idempotency
- **Constrain the solution space** to increase trust (Böckeler): restrict what agents can do to increase confidence in what they do

**Relevance**: Not immediately actionable for single-agent workflows, but provides the engineering framework for when Agent Teams or multi-agent orchestration is adopted. The workspace's worktree isolation already maps well to distributed systems best practices.

---

## Claude Code Agent Teams

**Added**: 2026-02-27 | **Sources**: [Claude Code Agent Teams docs](https://docs.anthropic.com/en/docs/agents-and-tools/claude-code/agent-teams)

Key takeaways:
- Multiple Claude Code instances coordinate as a team — leader spawns teammates, each gets its own context window, communication via mailbox
- Coordination patterns: leader/swarm/pipeline/watchdog
- Enable with `CLAUDE_CODE_EXPERIMENTAL_AGENT_TEAMS=1` (still experimental)
- Best suited for parallel work on separate packages where agents don't modify the same files

**Relevance**: The workspace's worktree infrastructure already provides the isolation layer Agent Teams assumes. Each teammate could work in its own worktree. Worth evaluating on non-critical multi-package features once stable.

---

## Model Context Protocol (MCP) — Industry Standard

**Added**: 2026-02-27 | **Sources**: [MCP spec](https://spec.modelcontextprotocol.io/), [Agentic AI Foundation](https://www.linuxfoundation.org/press/linux-foundation-launches-the-agentic-ai-foundation)

Key takeaways:
- Donated to the Linux Foundation's Agentic AI Foundation (Dec 2025) — now the cross-vendor standard adopted by OpenAI, Google, Anthropic, and others
- Thousands of community servers; universal protocol for connecting agents to tools and data
- **ROS 2-specific MCP servers** emerging: ROSBag MCP Server (natural language queries over bag files, [arXiv:2511.03497](https://arxiv.org/abs/2511.03497)), ROS 2 Robot Control MCP Server (agent access to topics/services/actions)
- Beyond tool connection, MCP adds a **validation layer** — can enforce contracts that soft instruction rules cannot

**Relevance**: Strategic for this workspace. `just-mcp` could expose workspace commands to any agent. ROS 2 MCP servers could enable runtime system inspection during development. MCP enforcement is the cross-agent equivalent of Claude Code hooks.

---

## ROS 2 Agent Frameworks

**Added**: 2026-02-27 | **Sources**: [ROSA (NASA JPL)](https://github.com/nasa-jpl/rosa), [RAI (RobotecAI)](https://github.com/RobotecAI/rai), [ROS-LLM (Auromix)](https://github.com/Auromix/ROS-LLM), [EmbodiedAgents (Automatika)](https://automatika-robotics.github.io/embodied-agents/)

Key takeaways:
- **ROSA** (NASA JPL): natural language interaction with ROS 1/2 systems — inspect nodes, query topics, diagnose issues; published at IROS 2024
- **RAI** (RobotecAI): vendor-agnostic agentic framework for Physical AI; integrates with ROS 2 navigation, manipulation, perception
- **ROS-LLM**: LLM-based code generation for robot behaviors (sequences, behavior trees, state machines)
- **EmbodiedAgents**: production-grade deployment of Physical AI on real robots via ROS 2
- These operate at **runtime** (inspect, diagnose, operate robots) vs. our workspace which operates at **development-time** (write code, manage PRs) — complementary, not competing
- MCP bridges the gap: a development agent could invoke a ROSA MCP server to inspect a running system while debugging

**Relevance**: ROSA is the most immediately relevant for diagnosing integration issues by querying a running simulation from within a coding session. Others are worth monitoring.

---

## Agentic Coding Market and Tool Landscape

**Added**: 2026-02-27 | **Sources**: [GitHub blog](https://github.blog/ai-and-ml/generative-ai/multi-agent-workflows-often-fail-heres-how-to-engineer-ones-that-dont/), [Anthropic engineering](https://www.anthropic.com/engineering/effective-harnesses-for-long-running-agents)

Key takeaways:
- Market grew from $550M to $4B in one year; 57% of companies run AI agents in production (Jan 2026)
- Gartner reports 1,445% surge in multi-agent system inquiries (Q1 2024 → Q2 2025)
- Quality concern: 40% quality deficit projected — more code enters pipelines than reviewers can validate; Google DORA 2025 found 90% AI adoption increase correlates with 9% higher bug rates, 91% more review time, 154% larger PRs
- **Worktree isolation is converging as an industry standard**: ccswarm, agentree, git-worktree-runner, Cursor 2.0, OpenAI Codex all independently adopted it
- **mini-swe-agent** (100 lines of Python, 65–74% on SWE-bench Verified) validates radical simplicity — a minimal harness with good context engineering outperforms complex frameworks
- **GitHub Agentic Workflows** (technical preview): define CI automation in Markdown instead of YAML; AI agents figure out execution

**Relevance**: The convergence on worktree isolation validates our architecture. The quality deficit numbers reinforce the importance of the harness (lints, CI, structural enforcement) over raw agent throughput. GitHub Agentic Workflows are worth monitoring for issue triage and CI failure investigation automation.

---

## GitHub Outage Resilience and Offline Development

**Added**: 2026-03-04 | **Sources**: [SecureSlate: GitHub outage resilience](https://getsecureslate.com/blog/what-the-github-outage-taught-us-about-resilience-and-compliance-2026), [GitHub down Feb 2026](https://serenitiesai.com/articles/github-down-ai-coding-tools-dependency-2026), [GitHub June 2025 outage](https://www.webpronews.com/githubs-june-2025-outage-how-a-routine-database-migration-cascaded-into-a-platform-wide-crisis/), [Self-hosted git platforms 2026](https://dasroot.net/posts/2026/01/self-hosted-git-platforms-gitlab-gitea-forgejo-2026/)

Key takeaways:
- GitHub has had multiple major outages in 2025–2026, partly due to its ongoing datacenter-to-Azure migration; Feb 2026 outage blocked pushes, CI, and PR reviews globally
- **Multi-remote strategy** is the most common resilience pattern: keep GitHub as primary, add a secondary remote (Forgejo, GitLab, Gitea) as failover — preserves existing integrations while enabling continued work during outages
- **Forgejo** (community fork of Gitea) is lightweight, self-hostable, and has Forgejo Actions (GitHub Actions–compatible but not identical; some workflow tweaks needed). Supports offline runner registration.
- **Gitea** offers a plugin-based microservices model; Gitea Actions provides closer GitHub Actions compatibility
- Alternative workflows mentioned by developers during outages: `git-send-email`, Radicle (decentralized git protocol)
- Core git operations (commit, branch, merge, diff, log) are fully local and unaffected by outages — the pain points are CI, PRs, issues, and code review

**Relevance**: This workspace depends on GitHub for issues, PRs, CI, and Copilot reviews. During outages, local coding continues but the issue-first workflow, worktree scripts (which use `gh`), and the PR-based review process are blocked. A resilience strategy should address these specific dependencies.

---

## git-bug — Distributed Offline-First Issue Tracker

**Added**: 2026-03-04 | **Sources**: [git-bug repo](https://github.com/git-bug/git-bug), [BrightCoding overview](https://www.blog.brightcoding.dev/2025/06/01/git-bug-a-distributed-offline-first-bug-tracker-embedded-in-git/), [HN discussion](https://news.ycombinator.com/item?id=43971620)

Key takeaways:
- Stores issues as git objects (under `refs/bugs`), not files — no clutter in the working tree, full version history, distributed by default
- **Offline-first**: create, edit, comment on issues without network; syncs via `git push/pull` to any remote
- **Bidirectional bridges**: GitHub, GitLab, Jira, Launchpad — can pull issues from GitHub and push local issues back
- Multiple interfaces: CLI, terminal UI (TUI), and web UI (significantly improved in recent releases)
- Latest release: v0.10.1 (May 2025), active development with 2,400+ commits
- **git-issue** (Spinellis) is a simpler alternative storing issues as files in a dedicated branch

**Relevance**: Could serve as a local issue cache that stays in sync with GitHub when online and continues working offline. The GitHub bridge would allow the workspace's issue-first workflow to continue during outages. The `refs/bugs` storage means no interference with the working tree or worktree isolation.

---

## Local CI with Act (nektos/act)

**Added**: 2026-03-04 | **Sources**: [nektos/act repo](https://github.com/nektos/act), [CICube guide](https://cicube.io/blog/run-github-actions-locally/), [Microsoft guide](https://techcommunity.microsoft.com/blog/azureinfrastructureblog/using-act-to-test-github-workflows-locally-for-azure-deployments-cicd/4414310)

Key takeaways:
- Runs GitHub Actions workflows locally using Docker — reads `.github/workflows/` and spins up containers matching the GitHub environment
- **Works offline** with cached Docker images; no GitHub connectivity needed for execution
- Eliminates the commit-push-wait cycle for CI feedback; saves GitHub Actions minutes
- Can serve as a local task runner alongside or instead of Make
- Limitations: not 100% compatible with all GitHub Actions features (marketplace actions may need adaptation), requires Docker

**Relevance**: The workspace already generates CI workflow templates (`.agent/templates/ci_workflow.yml`) for project repos. `act` could run these locally as a pre-push check or during outages. Combined with pre-commit hooks (already in place), this would provide full local CI coverage. The ROS 2 container images used in CI (`ros:jazzy-ros-core`) work with `act` since they're standard Docker images.

---

## ROS 2 Offline Development Patterns

**Added**: 2026-03-04 | **Sources**: [rosdep offline mirror PR](https://github.com/ros-infrastructure/rosdep/pull/839), [ROS Answers: rosdep offline](https://answers.ros.org/question/276942/can-sudo-rosdep-init-and-rosdep-update-be-done-offline/), [rosdep mirror guide](https://answers.ros.org/question/338122/how-do-i-deploy-a-rosdep-mirror/)

Key takeaways:
- `rosdep` can work offline by setting `ROSDISTRO_INDEX_URL=file:///path/to/local/index-v4.yaml` — clone the `rosdistro` repo locally and point to it
- All ROS 2 binary packages can be cached via a local apt mirror or pre-installed in Docker images
- `colcon build` and `colcon test` are fully local — no network needed once dependencies are installed
- The main online dependencies are: initial `rosdep init/update`, `apt install` for new packages, and `git clone` for source dependencies not yet in rosdep

**Relevance**: ROS 2 development is inherently local-friendly. The workspace's build/test workflow (`make build`, `make test`) works offline once dependencies are installed. The gap is in the governance layer (issues, PRs, reviews), not the build layer.

---

## Lightweight Self-Hosted Git Forges (Forgejo / Gitea)

**Added**: 2026-03-04 | **Sources**: [Self-hosted git platforms 2026](https://dasroot.net/posts/2026/01/self-hosted-git-platforms-gitlab-gitea-forgejo-2026/), [Forgejo comparison](https://forgejo.org/compare/), [Forgejo vs Gitea](https://forgejo.org/compare-to-gitea/), [Gitea comparison](https://docs.gitea.com/installation/comparison)

Key takeaways:
- **Gitea** and **Forgejo** (community fork of Gitea) are functionally near-identical: ~200MB RAM, single binary or Docker one-liner, run on Raspberry Pi
- Both offer: repo hosting, built-in issue tracker, web UI, wiki, Actions-compatible CI runners
- **GitLab CE** requires 4-8GB+ RAM — roughly 40x heavier for the same core git hosting role
- Forgejo is community-governed (no corporate entity); Gitea has Gitea Ltd behind it. Forgejo is the safer long-term bet for governance stability
- Forgejo Actions are GitHub Actions–compatible but not identical — minor workflow tweaks typically needed
- Both support offline runner registration for CI in disconnected environments

**Relevance**: The workspace uses a GitLab instance on the robot network for repo sync. Forgejo/Gitea could replace it at a fraction of the resource cost while adding a built-in issue tracker and web UI. Combined with git-bug for distributed issue sync, this enables a fully local-first development workflow where GitHub becomes a publication channel rather than a dependency. See [#345](https://github.com/rolker/ros2_agent_workspace/issues/345).
