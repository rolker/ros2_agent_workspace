# Research Digest: Workspace

<!-- Last updated: 2026-05-28 -->
<!-- If older than 30 days, consider running /research --refresh; entries older than 90 days should be flagged for review -->
<!-- 2026-05-28 full re-survey: refreshed all Feb–Mar 2026 landscape entries (the prior 2026-05-27 pass had only touched two). Original "Added" dates preserved; "Updated" stamps mark this pass. "Operational-Assistant Behavior Under Live Time Pressure" (2026-05-27) was already fresh and left as-is. Workspace-wide note: Opus 4.6 is no longer the current model — Opus 4.7 (2026-04-16) and Opus 4.8 (2026-05-28) shipped in this window; treat any "Opus 4.6 is current" framing elsewhere as stale. -->

## Command Runner Alternatives to Make

**Added**: 2026-02-27 | **Updated**: 2026-05-28 | **Sources**: [just](https://github.com/casey/just), [just CHANGELOG](https://github.com/casey/just/blob/master/CHANGELOG.md), [just-mcp](https://github.com/promptexecution/just-mcp), [Task](https://taskfile.dev/docs/changelog), [mise](https://github.com/jdx/mise)

Key takeaways:
- `just` (now **v1.51.0**, ~May 2026) remains the strongest Make-replacement candidate — `just --list`, no `.PHONY`, multi-language recipes; recent additions: `[env]` module-level export overrides, graceful recursion failure (was stack overflow)
- **`just-mcp` has matured from "adapter" to a production-ready MCP server**: discover/execute/introspect justfile recipes over MCP (`--stdio` + Docker run modes), explicitly agent-oriented and context-saving (the LLM gets command + params + hints without reading the bash/Python body), small-model (8k–32k) friendly
- `task` (Go/YAML) active on v3: added `if:` task conditions, runtime prompts for required vars, TLS/mTLS for remote Taskfiles, `TASK_`-prefixed env overrides — better CI/container ergonomics, but still more verbose than just
- `mise` (Rust/TOML, CalVer) added monorepo task support; still bundles tool-version management but adds little for ROS 2 where `apt`/`rosdep` manage toolchains
- `colcon defaults.yaml` can absorb build flags currently hardcoded in scripts, regardless of which runner is chosen

**Relevance**: The workspace uses Make purely as a command runner, not for dependency-based builds. Switching to `just` would reduce boilerplate; just-mcp's maturation makes the MCP-accessible-commands pathway more concrete than at last survey.

---

## AGENTS.md — Cross-Platform Agent Instruction Standard

**Added**: 2026-02-27 | **Updated**: 2026-05-28 | **Sources**: [agents.md spec](https://agents.md/), [AAIF adds 43 members (LF, 2026-05-18)](https://www.linuxfoundation.org/press/agentic-ai-foundation-adds-43-new-members-as-enterprise-and-government-adoption-of-open-agent-standards-accelerates), [aaif.io press](https://aaif.io/press/agentic-ai-foundation-adds-43-new-members-as-enterprise-and-government-adoption-of-open-agent-standards-accelerates/), [OpenAI Codex guide](https://developers.openai.com/codex/guides/agents-md/)

Key takeaways:
- Vendor-neutral spec stewarded by the Agentic AI Foundation (Linux Foundation); **membership ~190 organizations** as of 2026-05-18 (43 added: 4 Gold incl. F5/GoDaddy/Stripe/TRON, 27 Silver, 12 Associate) — up from "170+" in April; enterprise + government adoption highlighted
- **60,000+ open-source projects adopted** — still the cited figure in the May release, so treat as a floor that may be stale (it hasn't visibly grown despite member growth); measured 28.6% median runtime reduction, 16.6% token reduction
- Official supported-tool list expanded: **Amp, Codex, Cursor, Devin, Factory, Gemini CLI, GitHub Copilot, Jules, VS Code** (adds Amp + Factory vs. last survey)
- AAIF announced a **2026 global events program (AGNTCon + MCPCon, North America + Europe)**
- Recommended size: ≤150 lines; closest `AGENTS.md` to the edited file wins; user prompts override all
- For multi-tool workspaces, the `@AGENTS.md` import pattern in framework-specific files (e.g., CLAUDE.md) is preferred over symlinking

**Relevance**: Already adopted in this workspace. AGENTS.md consolidates shared rules; framework-specific files contain only agent-specific behavioral rules. This eliminated the previous four-file duplication problem.

---

## AI Agent Spec Writing Best Practices

**Added**: 2026-02-27 | **Updated**: 2026-05-28 | **Sources**: [Osmani, "How to Write a Good Spec for AI Agents" (O'Reilly, Jan 2026)](https://www.oreilly.com/radar/how-to-write-a-good-spec-for-ai-agents/), [Stack Overflow: shared coding guidelines for AI (Mar 2026)](https://stackoverflow.blog/2026/03/26/coding-guidelines-for-ai-agents-and-people-too/), [Augment Code: how to build AGENTS.md](https://www.augmentcode.com/guides/how-to-build-agents-md)

Key takeaways:
- Informed by GitHub's analysis of 2,500+ agent configuration files; "never commit secrets" confirmed as the single most common helpful constraint
- **Three-tier boundary system**: categorize rules as "Always" (autonomous), "Ask first" (human approval), or "Never" (hard stop) — more effective than flat rule lists
- **Self-audit instruction**, **SPEC.md as session anchor**, **conformance suites**, and the **two-phase task pattern** (draft spec in plan mode, then execute) all still hold
- New refinements since last survey: **"Agent Experience (AX)" design** — OpenAPI schemas, `llms.txt`, explicit type defs for agent-consumed surfaces; and **per-role agent personas** (`@docs-agent`, `@test-agent`, `@security-agent` with role-scoped AGENTS.md)
- Line guidance now commonly stated as **150–200 lines, then split into nested/subdirectory files**

**Relevance**: The Always/Ask/Never taxonomy was adopted directly into AGENTS.md. The self-audit pattern maps to our post-task verification checklist. The per-role-persona idea maps to our specialist review sub-agents.

---

## Harness Engineering — Production-Scale Agent-First Development

**Added**: 2026-02-27 | **Updated**: 2026-05-28 | **Sources**: [Latent Space deep-dive interview w/ Lopopolo](https://www.latent.space/p/harness-eng), [OpenAI blog (Lopopolo, Feb 2026)](https://openai.com/index/harness-engineering/), [Fowler, "Harness engineering"](https://martinfowler.com/articles/harness-engineering.html), [Hanchung, "Hidden Technical Debt: Agent Harness"](https://leehanchung.github.io/blogs/2026/05/08/hidden-technical-debt-agent-harness/)

Key takeaways:
- Team of 3–7 engineers built ~1M LOC product via Codex with **zero human-written code AND zero human code review** over 5 months; ~1,500 PRs; throughput scaled from ~¼ engineer-equivalent/person to **3–10 engineers' worth/person** (the earlier "3.5 PRs/eng/day" was a narrow slice of this)
- The **harness** (environment, constraints, feedback loops) matters more than the agent's capabilities — early slowness came from an underspecified environment, not incapable models
- Three harness components: **context engineering** (instruction files), **architectural constraints** (linters, structural tests), **garbage collection** (background agents fighting entropy)
- AGENTS.md should be a ~150-line **map**, not an encyclopedia; **lint errors as agent teaching**; **"anything not in-context doesn't exist"**; **"corrections are cheap, waiting is expensive"**
- Now formalized as the **"fourth paradigm of AI engineering"** (Fowler article + `awesome-harness-engineering` list)
- **Criticism / limitations** (this pass): EU regulatory blind spot (GDPR / AI Act / CLOUD Act data-sovereignty); a documented "hidden technical debt of the agent harness"; **"optimism asymmetry"** — agents predict fixes well but regressions poorly (cited fix precision ~33.7% vs regression precision ~11.8%), which directly stresses the 0%-human-review claim

**Relevance**: Validates several workspace patterns (worktree isolation, progressive disclosure, artifact-based communication). The map-vs-encyclopedia principle guides AGENTS.md/CLAUDE.md sizing. The optimism-asymmetry/regression-blindness finding reinforces our Quality Standard's insistence on tests + adversarial review rather than trusting agent self-assessment — especially for marine-safety code.

---

## Multi-Agent Engineering Patterns

**Added**: 2026-02-27 | **Updated**: 2026-05-28 | **Sources**: [GitHub blog, "Multi-agent workflows often fail" (Feb 2026)](https://github.blog/ai-and-ml/generative-ai/multi-agent-workflows-often-fail-heres-how-to-engineer-ones-that-dont/), [Shah, "The Problem Isn't the Model"](https://medium.com/@malav399/multi-agent-workflows-keep-failing-the-problem-isnt-the-model-1fa032218755), [awesome-harness-engineering](https://github.com/ai-boost/awesome-harness-engineering)

Key takeaways:
- Core thesis: most multi-agent failures come from **missing structure**, not model capability — treat agents as distributed system components
- **Typed schemas** for inter-agent data (Zod, JSON Schema) prevent field-name drift; **action schemas** (discriminated unions) prevent contradictory actions; **MCP as enforcement layer** validates tool I/O before execution
- **Distributed systems principles apply**: design for failure, validate at boundaries, constrain before scaling, log intermediate state, design for idempotency
- All original facts verified unchanged; the post drove substantial secondary coverage since, and these patterns are now bundled under the broader **"harness engineering"** umbrella in the literature (orchestration, MCP, permissions, observability) — the two topics are converging

**Relevance**: Not immediately actionable for single-agent workflows, but provides the engineering framework for when Agent Teams or multi-agent orchestration is adopted. The workspace's worktree isolation already maps well to distributed systems best practices.

---

## Claude Code Agent Teams

**Added**: 2026-02-27 | **Updated**: 2026-05-28 | **Sources**: [Claude Code Agent Teams docs](https://code.claude.com/docs/en/agent-teams), [Anthropic: Claude Opus 4.8](https://www.anthropic.com/news/claude-opus-4-8), [Claude model history](https://en.wikipedia.org/wiki/Claude_(language_model))

Key takeaways:
- Released 2026-02-05 alongside Opus 4.6; **still experimental, still disabled by default** (`CLAUDE_CODE_EXPERIMENTAL_AGENT_TEAMS=1`) as of 2026-05-28 — no GA, no default-on flip (re-verified this pass). Requires **Claude Code v2.1.32+**
- Note the model context has moved on: **Opus 4.7 (2026-04-16) and Opus 4.8 (2026-05-28) are now current** — the "released with Opus 4.6" pairing is history, not the current model
- One session is team lead; teammates are full independent Claude Code instances, each with its own context window and tool access, that **talk peer-to-peer** via a mailbox and **claim work off a shared task list** — unlike subagents, which only report to their parent
- Capabilities matured since release: **plan-approval mode for teammates**, **subagent definitions reusable as teammates**, teammate **hooks** (`TeammateIdle`, `TaskCreated`, `TaskCompleted`) for quality gates, **task dependencies with auto-unblock**, **file-locking on task claims**, and `teammateMode` (in-process vs split-pane)
- **Subagents vs. Teams**: subagents for cleanly decomposable, coordination-free work; teams for dependent work spanning multiple parts
- **Cost is linear in parallelism** — no separate agent pricing; N agents burn quota Nx

**Relevance**: The workspace's worktree infrastructure already provides the isolation Teams assume — each teammate works in its own worktree. The shared-task-list + peer-mailbox model (plus the new task-dependency + file-locking primitives) is directly relevant to the multi-host, multi-agent deployment lifecycle ([#495](https://github.com/rolker/ros2_agent_workspace/issues/495), [#470](https://github.com/rolker/ros2_agent_workspace/issues/470)): a deployment could run as a team (per-host loggers + a lead) rather than uncoordinated parallel sessions. Still experimental, so not yet a dependency to build on.

---

## Model Context Protocol (MCP) — Industry Standard

**Added**: 2026-02-27 | **Updated**: 2026-05-28 | **Sources**: [MCP first-anniversary / 2025-11-25 spec](https://blog.modelcontextprotocol.io/posts/2025-11-25-first-mcp-anniversary/), [MCP 2026-07-28 release candidate](https://blog.modelcontextprotocol.io/posts/2026-07-28-release-candidate/), [ros-mcp-server release (Open Robotics Discourse)](https://discourse.openrobotics.org/t/ros-mcp-server-release/50024)

Key takeaways:
- Governed by the Linux Foundation's **Agentic AI Foundation** (since Dec 2025), a sibling founding project to AGENTS.md and Block's goose — now the cross-vendor standard (OpenAI, Google, Anthropic, ...)
- **Current GA is the 2025-11-25 spec**: formalized async **Tasks**, server identity, Client ID Metadata Documents, client security requirements, elicitation/enhanced sampling, and the **Extensions system** (reverse-DNS extension IDs, capability negotiation)
- **Next major cut: the 2026-07-28 release candidate** (still upcoming) — stateless protocol core, Extensions framework, Tasks demoted to an extension, **MCP Apps** (server-rendered UIs), auth hardening (OAuth/OIDC, DPoP, Workload Identity Federation), formal deprecation policy
- **Registry GA** with enterprise governance/discoverability; gateway+registry patterns maturing
- **ROS 2 MCP servers expanding**: ROSBag MCP Server (NL queries over bag files, [arXiv:2511.03497](https://arxiv.org/abs/2511.03497)); **kakimochi/ros2-robot-control** and **lpigeon/ros-mcp-server** (ROS 1 & 2, `/cmd_vel`, model-agnostic, no robot-code changes) — the **validation layer** is better attributed to these server implementations than to the core protocol

**Relevance**: Strategic for this workspace. `just-mcp` could expose workspace commands to any agent. ROS 2 MCP servers could enable runtime system inspection during development — and the `/cmd_vel`-focused servers map directly onto our cmd_vel-only autonomy stack. MCP enforcement is the cross-agent equivalent of Claude Code hooks.

---

## ROS 2 Agent Frameworks

**Added**: 2026-02-27 | **Updated**: 2026-05-28 | **Sources**: [RAI (RobotecAI) releases](https://github.com/RobotecAI/rai/releases), [RAI paper arXiv:2505.07532](https://arxiv.org/abs/2505.07532), [ROSClaw arXiv:2603.26997](https://arxiv.org/abs/2603.26997), [ROSA (NASA JPL)](https://github.com/nasa-jpl/rosa)

Key takeaways:
- **RAI (RobotecAI) → 2.0**: a near-from-scratch rewrite (new packages, unified architecture, simpler embodied-agent build path); still the strongest "Physical AI" runtime contender (ROS 2 + LLM/VLM + voice + scenario tasks)
- **ROSA (NASA JPL)**: still LangChain-based NL interaction with ROS 1/2 (inspect nodes, query topics, diagnose; param validation/constraint safety); new direction is an **NVIDIA IsaacSim integration** (control robots and IsaacSim itself)
- **New entrant — ROSClaw** (arXiv:2603.26997, Mar 2026): model-agnostic executive layer bridging the OpenClaw runtime to ROS 2 — dynamic capability discovery + affordance injection, multimodal observation normalization, **pre-execution action validation within a configurable safety envelope**, structured audit/provenance logging; model/robot swaps are config-only; quantifies up to 4.8× differences in out-of-policy action rates across foundation models
- **ROS-LLM** (Auromix, now with a Nature Machine Intelligence write-up) and **EmbodiedAgents** (Automatika) facts unchanged
- No framework has become dominant — the field is still fragmented and operates at **runtime** (inspect, diagnose, operate) vs. our **development-time** workspace; MCP bridges the two

**Relevance**: ROSA is the most immediately relevant for diagnosing integration issues by querying a running simulation from a coding session. ROSClaw's safety-envelope + audit-logging design is a convergent pattern with the ROS MCP servers and directly echoes our Quality Standard for the cmd_vel-only stack — worth watching even though it's runtime-side.

---

## Agentic Coding Market and Tool Landscape

**Added**: 2026-02-27 | **Updated**: 2026-05-28 | **Sources**: [Anthropic 2026 Agentic Coding Trends Report](https://resources.anthropic.com/2026-agentic-coding-trends-report), [SevenOlives: $4B market consolidation 2026](https://sevenolives.com/blog/ai-coding-agents-4-billion-market-consolidation-2026), [GitHub Agentic Workflows technical preview](https://github.blog/changelog/2026-02-13-github-agentic-workflows-are-now-in-technical-preview/)

Key takeaways:
- **Market sizing revised sharply upward**: enterprise AI coding-agent market now ~**$9.8B–$11B annualized (Apr 2026)** (the "$550M→$4B" figure is now the prior-year baseline)
- **Consolidation is the new headline**: top three (Cursor, Copilot, Claude Code) hold **70%+ share** ("winner-take-most"); M&A wave — OpenAI 6 acquisitions in early 2026, Google's $2.4B Windsurf/Codeium acqui-hire, Anthropic acquired Vercept
- **Anthropic 2026 Trends Report** stats: 78% of Q1 2026 Claude Code sessions are multi-file edits (up from 34% Q1 2025); avg session 4min→23min; well-maintained context files → 40% fewer errors / 55% faster
- **Adoption-vs-production gap** reframed: ~79% adoption vs ~11% actually in production (the older "57% in prod" framing overstates it); Gartner's 1,445% multi-agent inquiry surge still valid
- Quality concern persists: more code enters pipelines than reviewers can validate; **worktree isolation has converged as an industry standard**; mini-swe-agent validates radical simplicity
- **GitHub Copilot CLI reached GA (Apr 2026)**, but **GitHub Agentic Workflows remains in technical preview, not GA** — don't conflate the two

**Relevance**: The convergence on worktree isolation validates our architecture. The quality-deficit numbers and adoption/production gap reinforce the importance of the harness (lints, CI, structural enforcement) over raw agent throughput. GitHub Agentic Workflows are still worth monitoring for issue triage and CI-failure-investigation automation.

---

## GitHub Outage Resilience and Offline Development

**Added**: 2026-03-04 | **Updated**: 2026-05-28 | **Sources**: [GitHub availability report, April 2026](https://github.blog/news-insights/company-news/github-availability-report-april-2026/), [CNBC: Microsoft/AI coding outages (2026-05-22)](https://www.cnbc.com/2026/05/22/microsoft-was-positioned-to-win-in-ai-coding-outages-got-in-the-way.html), [byteiota: GitHub reliability crisis](https://byteiota.com/github-reliability-crisis-three-nines/)

Key takeaways:
- The outage pattern **continued and worsened** through Mar–May 2026 — characterized now as a sustained reliability crisis, not a one-off. **April 2026 tied February as the worst month (7 major incidents each)**; a dozen-plus 1hr+ incidents since March
- New named incidents: an Apr 1–2 cluster (Copilot backend resource exhaustion → 2.7 hr outage) and the **Apr 23 Merge Queue bug** that produced *incorrect commits* — merge groups with >1 PR silently reverted prior merges (a **data-integrity** failure, not just availability)
- **Official acknowledgment**: GitHub CTO attributed Feb–Mar failures to rapid load growth, architectural coupling allowing localized issues to cascade, and inability to shed load from misbehaving clients; GitHub published an availability report + public apology
- **Azure migration is ongoing, not complete**: only ~12.5% of traffic on Azure (Iowa) as of March, targeting 50% by July 2026; load context ~275M commits/week (~14× YoY)
- **Multi-remote strategy** remains the most common resilience pattern (GitHub primary + Forgejo/GitLab/Gitea failover); `git-send-email` / Radicle as alternative workflows
- Core git operations (commit, branch, merge, diff, log) are local and unaffected by *availability* outages — **but the Apr 23 Merge Queue bug shows GitHub-side automation can still corrupt commit contents**, so "core git ops unaffected" needs that caveat

**Relevance**: This workspace depends on GitHub for issues, PRs, CI, and Copilot reviews. The sustained crisis strengthens the case for the offline-first resilience strategy already in progress ([#345](https://github.com/rolker/ros2_agent_workspace/issues/345)) — local coding continues during outages, but the issue-first workflow, `gh`-based worktree scripts, and PR review are blocked.

---

## git-bug — Distributed Offline-First Issue Tracker

**Added**: 2026-03-04 | **Updated**: 2026-05-28 | **Sources**: [git-bug releases](https://github.com/git-bug/git-bug/releases), [git-bug repo](https://github.com/git-bug/git-bug)

Key takeaways:
- Stores issues as git objects (under `refs/bugs`), not files — no working-tree clutter, full version history, distributed by default
- **Offline-first**: create, edit, comment without network; syncs via `git push/pull` to any remote
- **Bidirectional bridges**: GitHub, GitLab, Jira, Launchpad
- Multiple interfaces: CLI, TUI, web UI
- **v0.10.1 (May 2025) is still the latest release as of 2026-05-28** — no 2026 release; the post-v0.8.1→v0.10.1 burst has gone quiet, so v0.10.1 is now latest-but-aging (repo still live, 2,400+ commits)
- **git-issue** (Spinellis) is a simpler file-based alternative

**Relevance**: Could serve as a local issue cache that stays in sync with GitHub when online and continues working offline — directly relevant given the worsening GitHub outage picture. The quiet release cadence is worth weighing before depending on it. The workspace already uses it (`sync_repos.py` includes git-bug; `gh_create_issue.sh` has a `GITBUG_CREATE=1` offline path).

---

## Local CI with Act (nektos/act)

**Added**: 2026-03-04 | **Updated**: 2026-05-28 | **Sources**: [nektos/act releases](https://github.com/nektos/act/releases), [act runners docs](https://nektosact.com/usage/runners.html), [Dagger vs GitHub Actions](https://computingforgeeks.com/dagger-vs-github-actions-comparison/)

Key takeaways:
- Runs GitHub Actions workflows locally using Docker; **works offline** with cached images; eliminates the commit-push-wait cycle
- Now **v0.2.88** (~May 2026), actively maintained — mostly dependency updates; recent feature: scalar values + template expressions in matrix strategies (`${{ fromJSON(...) }}` dynamic matrices)
- Usability addition: the **GitHub Local Actions VS Code extension** (GUI front-end for act)
- Alternatives unchanged in substance: `gitlab-runner exec` still weak; **Dagger** remains complementary (pipelines-as-code that run identically on laptop and in CI), not a replacement
- Limitations: not 100% compatible with all Actions features (some marketplace actions need adaptation), requires Docker

**Relevance**: The workspace generates CI workflow templates (`.agent/templates/ci_workflow.yml`) for project repos. `act` could run these locally as a pre-push check or during outages (now more compelling given the sustained GitHub reliability crisis). The ROS 2 container images used in CI (`ros:jazzy-ros-core`) work with `act` since they're standard Docker images.

---

## ROS 2 Offline Development Patterns

**Added**: 2026-03-04 | **Updated**: 2026-05-28 | **Sources**: [ROS 2 Lyrical Luth release notes](https://docs.ros.org/en/rolling/Releases/Release-Lyrical-Luth.html), [Lyrical Luth released (Open Robotics Discourse)](https://discourse.openrobotics.org/t/ros-2-lyrical-luth-released/55021), [rosdep offline mirror guide](https://answers.ros.org/question/338122/how-do-i-deploy-a-rosdep-mirror/)

Key takeaways:
- **New distro: ROS 2 Lyrical Luth** (~2026-05-22) — the 12th release and a new **LTS** supported through **May 2031**; **Jazzy (2024 LTS)** remains a supported parallel LTS and **Kilted Kaiju (May 2025)** is the non-LTS in between
- Lyrical highlights (context): Callback Group Events Executor (~10–15% less CPU), `ros2 service info --verbose`, generic `resource_retriever_service`, Rust generator (`rosidl_generator_rs`) shipping as a default
- **Offline mechanics unchanged**: `rosdep` offline via `ROSDISTRO_INDEX_URL=file:///path/to/local/index-v4.yaml`; local apt mirror or pre-installed Docker images; `colcon build`/`test` fully local
- Online dependencies remain: initial `rosdep init/update`, `apt install` for new packages, `git clone` for source deps not yet in rosdep

**Relevance**: ROS 2 development is inherently local-friendly; `make build`/`make test` work offline once dependencies are installed. The gap remains in the governance layer (issues, PRs, reviews), not the build layer. Lyrical Luth is the distro to evaluate for any new LTS work, though this workspace currently targets Jazzy.

---

## Lightweight Self-Hosted Git Forges (Forgejo / Gitea)

**Added**: 2026-03-04 | **Updated**: 2026-05-28 | **Sources**: [Forgejo v15.0 (Apr 2026)](https://forgejo.org/2026-04-release-v15-0/), [Gitea 1.25.5 release](https://blog.gitea.com/release-of-1.25.5/), [Gitea releases](https://github.com/go-gitea/gitea/releases)

Key takeaways:
- Both are lightweight (~200MB RAM), single binary or Docker one-liner; offer repo hosting, built-in issue tracker, web UI, wiki, Actions-compatible CI runners; **GitLab CE is ~40× heavier**
- **Versions have churned hard**: Forgejo shipped **v14.0 (Jan 2026)** and **v15.0 (Apr 2026)** plus v11.0.14 LTS (May 2026) on a rapid quarterly major cadence; Gitea is on the **v1.26.x** line (1.26.0 Apr, 1.26.2 May 2026) — any text citing Forgejo v11/v12 or Gitea v1.24 as current is now wrong
- **Forgejo Actions matured substantially** (v15): OIDC token support (Runner v12.5.0+), `concurrency` blocks, matrix jobs, `runs-on` from earlier jobs, scheduled jobs, email-on-failure — narrows the "GH-compatible but not identical" gap meaningfully
- The two are **increasingly divergent, not "near-identical"**: version schemes have fully decoupled (v15 vs v1.26) and Forgejo's faster cadence + Actions feature lead mean feature sets no longer track 1:1; Forgejo's independent release velocity reinforces the "community-governed = safer long-term" point

**Relevance**: The workspace uses a GitLab instance on the robot network for repo sync. Forgejo/Gitea could replace it at a fraction of the resource cost while adding a built-in issue tracker and web UI. Forgejo Actions' OIDC/concurrency/matrix maturation makes a local CI mirror more viable. Combined with git-bug, this enables a fully local-first workflow where GitHub becomes a publication channel rather than a dependency. See [#345](https://github.com/rolker/ros2_agent_workspace/issues/345).

---

## Agent Orchestrators for Parallel Coding Agents

**Added**: 2026-03-11 | **Updated**: 2026-05-28 | **Sources**: [ComposioHQ/agent-orchestrator](https://github.com/ComposioHQ/agent-orchestrator), [Augment Code: open-source agent orchestrators](https://www.augmentcode.com/tools/open-source-agent-orchestrators), [Deloitte 2026: AI agent orchestration](https://www.deloitte.com/us/en/insights/industry/technology/technology-media-and-telecom-predictions/2026/ai-agent-orchestration.html)

Key takeaways:
- **Agent-orchestrator** (ComposioHQ, MIT): spawns parallel coding agents each in its own git worktree/branch/PR; pluggable adapters (runtime, agent, workspace, tracker, notifier) with a **reaction system** for CI failures / review comments / approved PRs — the key differentiator over manual setups
- **Vibe Kanban is sunsetting / now community-maintained** (last release v0.1.41, 2026-04-03, after Bloop's shutdown) — it was a flagship example at last survey, so this is the key correction
- **Convergent patterns are now treated as industry-standard / table-stakes** rather than emerging: (1) git worktree isolation per agent, (2) supervisor/coordinator, (3) event-driven reactions to CI/review, (4) dashboard for human oversight, (5) YAML config
- **Industry framing shifted to "orchestration is the competitive battleground"** (Deloitte 2026, Anthropic Trends Report's "orchestration era"); consolidation is happening at the model/platform layer, not (yet) among the standalone open-source orchestrators — no single one has emerged as outright winner
- The bottleneck has shifted from "can AI write code?" to "how do I run multiple agents in parallel without chaos?"

**Relevance**: This workspace already has the foundational primitives all orchestrators build on: worktree isolation scripts, draft-PR visibility, workforce protocol, multi-framework identity. The gaps relative to agent-orchestrator remain: (1) **no reaction system** — CI failures and review comments need manual agent re-engagement; (2) **no unified dashboard** of agent progress + CI + PRs; (3) **no automated task decomposition**; (4) **no inter-agent messaging** (Agent Teams, above, is the likeliest path to close this). A reaction system for CI/review events would deliver the most immediate value. See [#375](https://github.com/rolker/ros2_agent_workspace/issues/375).

---

## Operational-Assistant Behavior Under Live Time Pressure

**Added**: 2026-05-27 | **Sources**: [Google SRE incident response](https://sre.google/workbook/incident-response/), [incident.io AI SRE guide 2026](https://incident.io/blog/what-is-ai-sre-complete-guide-2026), [Sterile flight deck rule](https://en.wikipedia.org/wiki/Sterile_flight_deck_rule), [Sheridan & Verplank, levels of automation](https://www.hfes.org/Portals/0/Documents/Sheridan.pdf), [froggychips/sre-ai-copilot](https://github.com/froggychips/sre-ai-copilot), [Azure SRE Agent](https://azure.microsoft.com/en-us/products/sre-agent)

Key takeaways:
- **Mitigate before diagnose** (SRE): during an active incident, restore service first (rollback, drain, scale) and explicitly *defer* root-cause analysis until after stabilization. Investigation is time-boxed — don't deep-dive while the system (or, for us, scarce operating time) is impacted.
- **Sterile cockpit rule** (aviation, FAA 1981): during critical phases of flight, *only* activities essential to safe operation are permitted; all non-essential work is forbidden. Direct analog for a "live-ops" agent mode — no doc polish, refactors, or speculative deep dives while the boat is on the water.
- **Confidence-threshold escalation** (AI SRE 2026): agents auto-execute only *pre-approved runbooks for known failure classes*; below a configurable confidence threshold they **stop and hand off to a human with a prepared context summary**. Deterministic checks (a typed "FactStore") run *before* any LLM call, and conflicting facts cap confidence. A concrete anti-rabbit-hole mechanism.
- **Human-in-the-loop → human-on-the-loop as trust grows**: the 2026 industry default is approval-gated mitigation, relaxing to supervised autonomy as accuracy is validated — mirrors the workspace principle "tight by default, relaxable as confidence grows."
- **Adjustable / discretionary autonomy** (Sheridan & Verplank 1978, originally for *undersea teleoperators*): a selectable level-of-automation; operators rate systems more positively when they can change the autonomy level. Frames "deployment mode" as a discretionary autonomy shift, not a fixed behavior set.
- **Incident Commander + short runbooks** (ICS): one accountable human directs; runbooks stay short, version-controlled, alert-linked, and reviewed after each incident. The SRE lifecycle (Prepare → Detect → Respond → Recover → Learn) maps cleanly onto the deployment lifecycle ([0] start → [1] session / [2] recovery → [3] wrap-up / [4] next).

**Relevance**: Grounds the "deployment mode" design ([#495](https://github.com/rolker/ros2_agent_workspace/issues/495), [#477](https://github.com/rolker/ros2_agent_workspace/issues/477)) — an urgency-aware agent mode for live field deployments where on-water operating time is the scarce resource and agents have rabbit-holed during setup/troubleshooting. Sterile-cockpit + mitigate-before-diagnose + confidence-threshold-escalation supply a concrete "urgency contract" vocabulary; adjustable-autonomy and human-on-the-loop frame it as a *scoped, relaxable mode* — a behavioral sibling to field mode (ADR-0011), which already established the precedent of a mechanically-scoped exception to default agent rules.
