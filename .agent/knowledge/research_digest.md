# Research Digest: Workspace

<!-- Last updated: 2026-07-14 -->
<!-- If older than 30 days, consider running /research --refresh; entries older than 90 days should be flagged for review -->
<!-- 2026-07-14 full re-survey (6 parallel research agents): all 17 entries refreshed and re-stamped. Headline shifts this window: Claude 5 family launched (Fable 5 / Mythos 5, 2026-06-09; Sonnet 5 2026-06-30) — the model-lineup entry was renamed accordingly; SpaceX acquired Cursor ($60B); MCP 2026-07-28 release on track (stateless core; Roots/Sampling/Logging deprecated); GitHub's Azure migration missed its 50% target and Microsoft rented AWS capacity; Forgejo v16 ships 2026-07-15 with v15 as new LTS anchor; `just` 1.54/1.55 closed its incremental-build and parallelism gaps vs Make; Copilot code review now reads AGENTS.md. Workspace-wide note: any "Opus 4.8 is Anthropic's most capable model" framing elsewhere is stale — Fable 5 sits above it. -->

## Command Runner Alternatives to Make

**Added**: 2026-02-27 | **Updated**: 2026-07-14 | **Sources**: [just CHANGELOG](https://github.com/casey/just/blob/master/CHANGELOG.md), [just releases](https://github.com/casey/just/releases), [just-mcp](https://github.com/promptexecution/just-mcp), [Task releases](https://github.com/go-task/task/releases), [mise v2026.7.0](https://github.com/jdx/mise/releases/tag/v2026.7.0)

Key takeaways:
- `just` shipped **five releases in six weeks (1.51→1.56)** and closed its two most-cited gaps vs Make: **1.54 added cached recipes with file inputs/outputs + `--clean`** (Make's incremental-rebuild core) and **1.55 added `--jobs` parallelism** (Make's `-j`). Also: `[shell]` per-recipe shell, markdown justfiles, `[arg(...)]` validation attributes, `--fmt` stabilized; 1.56 (Jul 2026) is a large bugfix release
- The old "just is a command runner, not a build system" framing (e.g. LWN, Dec 2025) is now partially obsolete — it does incremental + parallel since 1.54/1.55
- **`just-mcp` status quo holds** (production-ready MCP server, stdio + Docker, agent-oriented) — no major new development Jun–Jul 2026
- `task` v3.52 (Jul 2026): `secret:` masking, `.gitignore`-aware fingerprinting, Azure DevOps remote Taskfiles; still more verbose than just
- `mise` v2026.7.0 deepened monorepo support (union config-roots, unified lockfiles); still bundles tool-version management that adds little for ROS 2 where `apt`/`rosdep` manage toolchains
- `colcon defaults.yaml` can absorb build flags currently hardcoded in scripts, regardless of which runner is chosen

**Relevance**: The workspace uses Make purely as a command runner, not for dependency-based builds. The case for `just` strengthened materially this window — cached recipes could replace the `.make/` stamp-file machinery natively, and just-mcp keeps the MCP-accessible-commands pathway open.

---

## AGENTS.md — Cross-Platform Agent Instruction Standard

**Added**: 2026-02-27 | **Updated**: 2026-07-14 | **Sources**: [agents.md spec](https://agents.md/), [Copilot code review reads AGENTS.md (2026-06-18)](https://github.blog/changelog/2026-06-18-copilot-code-review-agents-md-support-and-ui-improvements/), [AAIF adds 43 members](https://www.prnewswire.com/news-releases/agentic-ai-foundation-adds-43-new-members-as-enterprise-and-government-adoption-of-open-agent-standards-accelerates-302774361.html), [MCPA certification](https://aaif.io/blog/introducing-the-mcpa-the-first-official-certification-for-the-model-context-protocol/)

Key takeaways:
- **GitHub Copilot code review now reads root-level `AGENTS.md`** (2026-06-18) and applies its instructions when generating review feedback — the standard has extended from coding-agent guidance into the review surface
- Vendor-neutral spec stewarded by the Agentic AI Foundation (Linux Foundation); membership confirmed at **190 organizations** (the 2026-05-18 wave included the first significant government adopters: NSW Government, U.S. Army, PNNL, Sandia); no spec changes since
- **60,000+ open-source projects adopted** — still the cited figure, treat as an aging floor; measured 28.6% median runtime reduction, 16.6% token reduction
- Official supported-tool list: **Amp, Codex, Cursor, Devin, Factory, Gemini CLI, GitHub Copilot, Jules, VS Code**
- AAIF launched the **MCPA — first official MCP certification** (2026-07-10); flagship events dated: **AGNTCon + MCPCon Europe Sep 17–18 (Amsterdam), North America Oct 22–23 (San Jose)**
- Recommended size: ≤150 lines; closest `AGENTS.md` to the edited file wins; user prompts override all
- For multi-tool workspaces, the `@AGENTS.md` import pattern in framework-specific files (e.g., CLAUDE.md) is preferred over symlinking

**Relevance**: Already adopted in this workspace. AGENTS.md consolidates shared rules; framework-specific files contain only agent-specific behavioral rules. This eliminated the previous four-file duplication problem. Copilot code review reading AGENTS.md means our workspace rules now also steer the Copilot PR reviews this workspace already receives — worth verifying project repos' AGENTS.md coverage with that in mind.

---

## AI Agent Spec Writing Best Practices

**Added**: 2026-02-27 | **Updated**: 2026-07-14 | **Sources**: [Osmani, "How to Write a Good Spec for AI Agents" (O'Reilly, Jan 2026)](https://www.oreilly.com/radar/how-to-write-a-good-spec-for-ai-agents/), [Microsoft: Spec-Driven Development (2026-06-10)](https://developer.microsoft.com/blog/spec-driven-development-ai-native-engineering), [SDD 2026 guide (BCMS)](https://thebcms.com/blog/spec-driven-development), [SDD paper (arXiv:2602.00180)](https://arxiv.org/html/2602.00180v1)

Key takeaways:
- **Spec-Driven Development (SDD) is now the dominant codification**: Microsoft published a vendor-blog treatment (2026-06-10, spec as shared source of truth across the SDLC), every major tool has an SDD flavor (GitHub Spec Kit, AWS Kiro, Claude Code, Cursor, OpenSpec, BMAD, Tessl), **EARS** (Easy Approach to Requirements Syntax) has emerged as the near-universal acceptance-criteria format, and DeepLearning.AI runs a dedicated course
- Informed by GitHub's analysis of 2,500+ agent configuration files; "never commit secrets" confirmed as the single most common helpful constraint
- **Three-tier boundary system**: categorize rules as "Always" (autonomous), "Ask first" (human approval), or "Never" (hard stop) — more effective than flat rule lists
- **Self-audit instruction**, **SPEC.md as session anchor**, **conformance suites**, and the **two-phase task pattern** (draft spec in plan mode, then execute) all still hold
- **"Agent Experience (AX)" design** matured (OpenAPI schemas, `llms.txt`, explicit type defs for agent-consumed surfaces; first AX job postings); **per-role agent personas** with role-scoped AGENTS.md now also feed review (Copilot code review reads AGENTS.md)
- Line guidance: **150–200 lines, then split into nested/subdirectory files**

**Relevance**: The Always/Ask/Never taxonomy was adopted directly into AGENTS.md. The self-audit pattern maps to our post-task verification checklist. The per-role-persona idea maps to our specialist review sub-agents.

---

## Harness Engineering — Production-Scale Agent-First Development

**Added**: 2026-02-27 | **Updated**: 2026-07-14 | **Sources**: [Latent Space deep-dive interview w/ Lopopolo](https://www.latent.space/p/harness-eng), [OpenAI blog (Lopopolo, Feb 2026)](https://openai.com/index/harness-engineering/), [Fowler & Böckeler, "Harness engineering" (full article, Apr 2026)](https://martinfowler.com/articles/harness-engineering.html), [Agentic Harness Engineering (arXiv:2604.25850)](https://arxiv.org/abs/2604.25850)

Key takeaways:
- Team of 3–7 engineers built ~1M LOC product via Codex with **zero human-written code AND zero human code review** over 5 months; ~1,500 PRs. **Claim remains neither independently validated nor debunked** — OpenAI reiterated it Jun 2026; Fowler's counterpoint stands (the harness enforces how code is written, not that it does what users need)
- The **harness** (environment, constraints, feedback loops) matters more than the agent's capabilities; three components: **context engineering**, **architectural constraints**, **garbage collection** (background agents fighting entropy)
- AGENTS.md should be a ~150-line **map**, not an encyclopedia; **lint errors as agent teaching**; **"anything not in-context doesn't exist"**; **"corrections are cheap, waiting is expensive"**
- **"Optimism asymmetry" now has an empirical anchor**: the Agentic Harness Engineering paper (arXiv:2604.25850, Fudan/PKU) has an "evolve agent" auto-editing its own harness (Terminal-Bench 2 pass@1 69.7%→77.0% in 10 iterations, beating a human-designed harness) while quantifying **regression blindness** — fix-prediction precision 33.7% vs regression-prediction precision 11.8%
- Emerging follow-on framing: **"loop engineering"** (turn/goal/time-based loop taxonomies, deterministic stop conditions, token budgets) — practitioner-tier so far, no canonical source yet
- Fowler's Feb "first thoughts" memo was superseded by the full Apr 2 article (with Birgitta Böckeler) — cite the article, not the memo

**Relevance**: Validates several workspace patterns (worktree isolation, progressive disclosure, artifact-based communication). The map-vs-encyclopedia principle guides AGENTS.md/CLAUDE.md sizing. The optimism-asymmetry/regression-blindness finding reinforces our Quality Standard's insistence on tests + adversarial review rather than trusting agent self-assessment — especially for marine-safety code.

---

## Multi-Agent Engineering Patterns

**Added**: 2026-02-27 | **Updated**: 2026-07-14 | **Sources**: [GitHub blog, "Multi-agent workflows often fail" (Feb 2026)](https://github.blog/ai-and-ml/generative-ai/multi-agent-workflows-often-fail-heres-how-to-engineer-ones-that-dont/), [GitHub Copilot app (GA 2026-06-17)](https://github.blog/news-insights/product-news/github-copilot-app-the-agent-native-desktop-experience/), [Microsoft Learn: multi-agent patterns](https://learn.microsoft.com/en-us/agents/architecture/multi-agent-patterns), [awesome-harness-engineering](https://github.com/ai-boost/awesome-harness-engineering)

Key takeaways:
- Core thesis: most multi-agent failures come from **missing structure**, not model capability — treat agents as distributed system components
- **Typed schemas** for inter-agent data (Zod, JSON Schema) prevent field-name drift; **action schemas** (discriminated unions) prevent contradictory actions; **MCP as enforcement layer** validates tool I/O before execution (now institutionalized via AAIF's MCPA certification)
- **Distributed systems principles apply**: design for failure, validate at boundaries, constrain before scaling, log intermediate state, design for idempotency
- **GitHub productized the pattern**: the standalone **GitHub Copilot app** (GA 2026-06-17, Win/mac/Linux) runs each agent session in its own auto-managed **git worktree** (~10 concurrent), with Agent Merge, "Canvases" work surfaces, and sandboxes; GitHub Desktop 3.6 added worktrees too — isolation + control plane as the fix, now mainstream tooling
- The patterns remain bundled under the **"harness engineering"** umbrella — the two topics keep converging

**Relevance**: Not immediately actionable for single-agent workflows, but provides the engineering framework for when Agent Teams or multi-agent orchestration is adopted. The workspace's worktree isolation already maps well to distributed systems best practices.

---

## Claude Code Agent Teams

**Added**: 2026-02-27 | **Updated**: 2026-07-14 | **Sources**: [Claude Code Agent Teams docs](https://code.claude.com/docs/en/agent-teams), [Claude Code changelog](https://code.claude.com/docs/en/changelog), [subagents vs workflows vs teams (MindStudio)](https://www.mindstudio.ai/blog/claude-code-dynamic-workflows-vs-agent-teams-vs-sub-agents)

Key takeaways:
- **Still experimental, still gated** behind `CLAUDE_CODE_EXPERIMENTAL_AGENT_TEAMS=1` as of 2026-07-14 — no GA, no default-on flip; also unavailable on Claude Code on the web even with the flag
- **Setup flow simplified in v2.1.178 (2026-06-15): `TeamCreate`/`TeamDelete` removed** — every session now has one implicit team; teammates spawn directly via the Agent tool's `name` parameter, no setup step. New `Tool(param:value)` permission-rule syntax (e.g. `Agent(model:opus)`) landed alongside
- One session is team lead; teammates are full independent Claude Code instances, each with its own context window and tool access, that **talk peer-to-peer** via a mailbox and **claim work off a shared task list** — unlike subagents, which only report to their parent. Plan-approval, teammate hooks (`TeammateIdle`, `TaskCreated`, `TaskCompleted`), task dependencies with auto-unblock, and file-locking on claims all still as documented
- **Not merged with Dynamic Workflows** — they coexist as a three-tier choice: subagents (decomposable, coordination-free), Dynamic Workflows (deterministic scripted fan-out at scale), Agent Teams (~2–5 persistent named peers negotiating live)
- v2.1.198's background-by-default subagents (auto-commit/push/PR) absorb some of the use cases Teams targeted
- **Cost is linear in parallelism** — no separate agent pricing; N agents burn quota Nx

**Relevance**: The workspace's worktree infrastructure already provides the isolation Teams assume — each teammate works in its own worktree. The shared-task-list + peer-mailbox model (plus the new task-dependency + file-locking primitives) is directly relevant to the multi-host, multi-agent deployment lifecycle ([#495](https://github.com/rolker/ros2_agent_workspace/issues/495), [#470](https://github.com/rolker/ros2_agent_workspace/issues/470)): a deployment could run as a team (per-host loggers + a lead) rather than uncoordinated parallel sessions. Still experimental, so not yet a dependency to build on.

---

## Anthropic Model Lineup (Claude 5 / Opus 4.8), Fast Mode & Dynamic Workflows

**Added**: 2026-05-29 | **Updated**: 2026-07-14 | **Sources**: [Introducing Claude Fable 5 & Mythos 5](https://www.anthropic.com/news/claude-fable-5-mythos-5), [Introducing Claude Sonnet 5](https://www.anthropic.com/news/claude-sonnet-5), [Introducing Claude Opus 4.8](https://www.anthropic.com/news/claude-opus-4-8), [Claude Code: Workflows](https://code.claude.com/docs/en/workflows), [Claude Code changelog](https://code.claude.com/docs/en/changelog), [Dynamic Workflows (InfoQ)](https://www.infoq.com/news/2026/06/dynamic-workflows-claude-code/)

Key takeaways:
- **Claude 5 family launched 2026-06-09**: **Fable 5** (`claude-fable-5`, $10/$50 per MTok, 1M context, 128K output) is the first GA **Mythos-class** model — a tier *above* Opus; SOTA on nearly all tested benchmarks (~80.3% SWE-Bench Pro, ≈11 pts ahead of next best). **Mythos 5** = same model minus certain dual-use safeguards, gated to approved orgs (Project Glasswing). Fable 5 API differences: thinking always on (no `thinking` config), no raw CoT / no prefill / no sampling params, 30-day retention required (no ZDR), safety classifiers can return `stop_reason: "refusal"` with a **server-side fallback beta** that reruns declined requests on Opus 4.8. In Claude Code since v2.1.170.
- **Sonnet 5** (`claude-sonnet-5`, 2026-06-30): near-Opus agentic quality at $3/$15 (**intro $2/$10 through 2026-08-31**); SWE-Bench Pro 63.2% vs Opus 4.8's 69.2%; new tokenizer (~1.0–1.35× more tokens), adaptive thinking default, first Sonnet with `xhigh` effort; 1M/128K.
- **Lineup now**: Fable 5 ($10/$50) → Opus 4.8 ($5/$25, still the default recommended Opus and Fable's fallback target) → Sonnet 5 ($3/$15) → Haiku 4.5 ($1/$5). Opus 4.8 official numbers landed: SWE-bench Pro 69.2%, SWE-bench Verified 88.6% (now official, previously third-party).
- **Dynamic Workflows** still **research preview** (Enterprise/Team/Max): Claude-authored JS orchestration over background subagents; caps unchanged (1,000 agents/run, 16 concurrent). Trigger keyword renamed `workflow` → **`ultracode`** (v2.1.160 — it's the trigger keyword, *not* an effort level; API efforts are low→max), plus a workflow-size setting and OpenTelemetry run attributes (v2.1.202).
- **Fast mode** still research preview: Opus 4.8 fast at $10/$50 (~3× cheaper than the 4.7-era rate, up to 2.5× output speed); **Opus 4.7 fast mode deprecated** (hard-errors after ~2026-07-25); first-party API only.
- **Claude Code June–July**: v2.1.198 (Jul 1) made **subagents background-by-default with auto-commit/push/PR**, Claude in Chrome GA; stacked slash-skill invocations (2.1.199); org default models (2.1.196); `--safe-mode` + `/cd` (2.1.169).

**Relevance**: **Dynamic Workflows is the in-product orchestration primitive** the "Agent Orchestrators" entry below was tracking as a workspace gap ([#375](https://github.com/rolker/ros2_agent_workspace/issues/375)) — Claude-authored fan-out over background subagents composes naturally with our worktree isolation. The Claude 5 tiering matters for dispatch economics: Fable 5 for the hardest design/review passes, Opus 4.8 as the durable default, Sonnet 5 (intro-priced) for mechanical fan-out stages — relevant to `dispatch_subagent.sh --model` and container dispatch. Background-by-default subagents with auto-PR (v2.1.198) overlaps with what our worktree + draft-PR scripts hand-roll — watch for consolidation opportunities. **Caveat**: Dynamic Workflows and fast mode remain research preview — useful now, not yet stable enough to hard-wire into governed scripts.

---

## Model Context Protocol (MCP) — Industry Standard

**Added**: 2026-02-27 | **Updated**: 2026-07-14 | **Sources**: [MCP 2026-07-28 release candidate](https://blog.modelcontextprotocol.io/posts/2026-07-28-release-candidate/), [SDK betas for 2026-07-28](https://blog.modelcontextprotocol.io/posts/sdk-betas-2026-07-28/), [AAIF: MCP is growing up](https://aaif.io/blog/mcp-is-growing-up/), [Amazing ROS 2 MCP (Discourse)](https://discourse.openrobotics.org/t/amazing-ros-2-mcp-native-ros-2-mcp-server-for-ai-assisted-robotics-agents/55270)

Key takeaways:
- **2026-07-28 release is on track** (RC locked 2026-05-21, final text publishes in 2 weeks): **stateless protocol core** (no `initialize` handshake / session header — version+capabilities travel in `_meta` per request, plain load balancers work), **MCP Apps** (sandboxed-iframe server UIs routed through the same consent/audit path as tool calls), Tasks demoted to an extension, OAuth/OIDC-aligned auth, formal ≥12-month deprecation policy. **Roots, Sampling, and Logging are formally deprecated** in this revision; new MRTR (multi round-trip requests) for mid-call user interaction
- **Not a switch-off**: beta SDKs shipped 2026-06-29 (Python v2, TS v2, Go, C#); v1.x SDKs get security updates ≥6 months; no breaking change on July 28 for existing implementations
- Governance under the LF's **Agentic AI Foundation**; MCPA certification launched (2026-07-10); MCP Dev Summit NYC drew ~1,200 (2×), 110M monthly SDK downloads; enterprise theme: **"shadow MCP" found at 3–10× expected rates** → centralized gateway + registry is the consensus pattern
- **Registry**: ~9,650 servers in the official registry (May 2026); Glama indexes ~20,000
- **ROS 2 MCP servers keep expanding**: new native **"Amazing ROS 2 MCP"** (rclpy-direct, no rosbridge, Jun 2026), LCAS/ros2_mcp + wise-vision/ros2_mcp (introspection), lpigeon's server rebranded **robotmcp/ros-mcp-server** (ROS 1+2, Claude/GPT/Gemini); robotics MCP ecosystem now 50+ servers

**Relevance**: Strategic for this workspace. `just-mcp` could expose workspace commands to any agent. ROS 2 MCP servers could enable runtime system inspection during development — and the `/cmd_vel`-focused servers map directly onto our cmd_vel-only autonomy stack. MCP enforcement is the cross-agent equivalent of Claude Code hooks.

---

## ROS 2 Agent Frameworks

**Added**: 2026-02-27 | **Updated**: 2026-07-14 | **Sources**: [RAI (RobotecAI) releases](https://github.com/RobotecAI/rai/releases), [RHO paper arXiv:2606.16458](https://arxiv.org/abs/2606.16458), [ROSClaw arXiv:2603.26997](https://arxiv.org/abs/2603.26997), [Interop SIG: Open-RMF via MCP](https://discourse.openrobotics.org/t/interop-sig-02-july-2026-natural-language-control-of-open-rmf-fleets-via-the-model-context-protocol-mcp/55687), [Standard Motion Primitives proposal](https://discourse.openrobotics.org/t/a-standard-interface-from-llm-to-robot-actions/55894)

Key takeaways:
- **The established frameworks all went quiet** this window: RAI latest is still 2.12.0 (Apr 2026, langchain 1.x; 3 commits since late May), ROSA still v1.0.10 (Mar 2026), EmbodiedAgents 0.5.1 (Feb 2026), ROSClaw paper has no follow-up or public code
- **New paradigm entrant — RHO** ("Your Coding Agent is Secretly a Roboticist", arXiv:2606.16458, Jun 2026): trains coding agents to search interpretable neurosymbolic policy repositories from environment reward (no teleop demos) — 70% Robosuite SOTA, 2.5× prior on LIBERO-PRO; a *training-time* counterpoint to the previously all-runtime field
- **MCP is entering official ROS channels**: Interop SIG (2026-07-02) presented **Nayantra**, an MCP server exposing the Open-RMF REST API as LLM-callable tools (plain-English → multi-step RMF missions) — first prominent MCP-based LLM→ROS integration in an OSRF SIG
- **Standardization stirring**: a ROS Discourse proposal (2026-06-29) for **12 Standard Motion Primitives** as standard topics (any LLM → any robot), complementary to a URML intent-semantics effort — early talk, no code yet
- **Naming caution**: "rosclaw" is now three distinct things — the Kent State paper (no code), the dormant PlaiPin/rosclaw plugin repo, and the unrelated-but-active ros-claw/rosclaw ("AUTOSAR + Android for Embodied AI") — don't conflate
- No framework has become dominant — still fragmented, still mostly runtime-side (RHO excepted); MCP bridges runtime and development-time

**Relevance**: ROSA is the most immediately relevant for diagnosing integration issues by querying a running simulation from a coding session. ROSClaw's safety-envelope + audit-logging design is a convergent pattern with the ROS MCP servers and directly echoes our Quality Standard for the cmd_vel-only stack — worth watching even though it's runtime-side.

---

## Agentic Coding Market and Tool Landscape

**Added**: 2026-02-27 | **Updated**: 2026-07-14 | **Sources**: [Anthropic 2026 Agentic Coding Trends Report](https://resources.anthropic.com/2026-agentic-coding-trends-report), [CNBC: SpaceX acquires Cursor](https://www.cnbc.com/2026/06/16/spacex-spcx-cursor-acquisition-ipo.html), [OpenAI acquires Ona](https://openai.com/index/openai-to-acquire-ona/), [GitHub Agentic Workflows public preview](https://github.blog/changelog/2026-06-11-github-agentic-workflows-is-now-in-public-preview/), [Gartner enterprise AI coding agents](https://www.gartner.com/en/articles/enterprise-ai-coding-agent-market)

Key takeaways:
- **SpaceX acquired Cursor (Anysphere), announced 2026-06-16, $60B all-stock** — largest venture-backed-startup acquisition ever (post-SpaceX-IPO; Cursor at ~$4B annualized revenue; rationale: coding data for Grok training + xAI Colossus compute; close expected Q3 2026). Moves one of the top-3 tools into the SpaceX/xAI orbit
- **OpenAI acquired Ona (ex-Gitpod)** (~2026-06-12): run Codex agents persistently inside the customer's own cloud; Codex >5M weekly users (+400% YTD); OpenAI now at 7+ devtools acquisitions in 2026
- **Anthropic trajectory**: ~$47B annualized run rate reported 2026-05-29 (vs $9B end-2025); first quarterly operating profit projected Q2 2026; >1,000 customers spending $1M+/yr. (Claims that "Claude Code holds 54% share" or "Anthropic passed OpenAI in ARR" trace only to low-quality aggregators — not adopted here)
- **Market**: enterprise AI coding-agent market still ~$9.8–11B annualized (Apr 2026); Gartner Leaders = Anthropic, Cursor, GitHub, OpenAI; adoption-vs-production gap ~79% vs ~11% persists; **worktree isolation remains the industry-standard pattern**
- **GitHub Agentic Workflows moved to public preview 2026-06-11** (from technical preview) — still not GA; NL Markdown compiled to Actions YAML, read-only defaults, sandboxed containers behind an "Agent Workflow Firewall"; Copilot CLI has been GA since Apr 2026

**Relevance**: The convergence on worktree isolation validates our architecture. The quality-deficit numbers and adoption/production gap reinforce the importance of the harness (lints, CI, structural enforcement) over raw agent throughput. GitHub Agentic Workflows are still worth monitoring for issue triage and CI-failure-investigation automation.

---

## GitHub Outage Resilience and Offline Development

**Added**: 2026-03-04 | **Updated**: 2026-07-14 | **Sources**: [GitHub availability report, June 2026](https://github.blog/news-insights/company-news/github-availability-report-june-2026/), [The Register: outages persist as AI coding drives traffic (2026-06-12)](https://www.theregister.com/software/2026/06/12/github-outages-persist-as_ai_coding_drives_traffic_surge/5255125), [TechTimes: Microsoft taps AWS for GitHub capacity](https://www.techtimes.com/articles/318481/20260616/githubs-ai-agent-crisis-forces-microsoft-tap-aws-outages-break-enterprise-slas.htm), [githubstatus.com](https://www.githubstatus.com/)

Key takeaways:
- **June 2026 was the first measurable improvement**: 6 official incidents vs 9 in May and 10 in April; stated operating principle now "availability, then capacity, then features". But third-party status-page tallies counted ~25 incidents in the May 27–Jun 26 window (mostly narrow/short), and The Register put mid-June availability near 88% — the "many medium incidents" pattern continues even as major-incident count declines (Jul 1–14: Copilot budget-reset block ~13h, Pages delays, Actions 500s, runner-start delays)
- **Azure migration missed its 50%-by-July target**: monolith traffic peaked at 45%, Git services at 43%; ramp deliberately paused ~1 month after a May 21 stability incident, restarted with new safety gates; full completion now framed as **2027**
- **Microsoft contracted AWS capacity to keep GitHub running** (mid-June) — a competitor cloud as short-term buffer. Driver is AI-agent load: AI-agent PRs 4M/mo (Sep 2025) → 17M/mo (Mar 2026); Actions minutes 500M/wk (2023) → 2.1B/wk (early 2026); 275M commits/week
- Historical marker: the **Apr 23 Merge Queue bug** produced *incorrect commits* (merge groups >1 PR silently reverted prior merges) — GitHub-side automation can corrupt commit contents, so "core git ops unaffected" keeps that caveat
- **Multi-remote strategy** remains the most common resilience pattern (GitHub primary + Forgejo/GitLab/Gitea failover)

**Relevance**: This workspace depends on GitHub for issues, PRs, CI, and Copilot reviews. The sustained crisis strengthens the case for the offline-first resilience strategy already in progress ([#345](https://github.com/rolker/ros2_agent_workspace/issues/345)) — local coding continues during outages, but the issue-first workflow, `gh`-based worktree scripts, and PR review are blocked.

---

## git-bug — Distributed Offline-First Issue Tracker

**Added**: 2026-03-04 | **Updated**: 2026-07-14 | **Sources**: [git-bug releases](https://github.com/git-bug/git-bug/releases), [git-bug repo](https://github.com/git-bug/git-bug)

Key takeaways:
- Stores issues as git objects (under `refs/bugs`), not files — no working-tree clutter, full version history, distributed by default
- **Offline-first**: create, edit, comment without network; syncs via `git push/pull` to any remote
- **Bidirectional bridges**: GitHub, GitLab, Jira, Launchpad
- Multiple interfaces: CLI, TUI, web UI
- **v0.10.1 (May 2025) is still the latest release as of 2026-07-14** — 14 months without a release; repo alive but low-cadence (issue/PR activity through mid-2026, ~9.9k stars, no deprecation notice)
- **git-issue** (Spinellis) is a simpler file-based alternative

**Relevance**: Could serve as a local issue cache that stays in sync with GitHub when online and continues working offline — directly relevant given the worsening GitHub outage picture. The quiet release cadence is worth weighing before depending on it. The workspace already uses it (`sync_repos.py` includes git-bug; `gh_create_issue.sh` has a `GITBUG_CREATE=1` offline path).

---

## Local CI with Act (nektos/act)

**Added**: 2026-03-04 | **Updated**: 2026-07-14 | **Sources**: [nektos/act releases](https://github.com/nektos/act/releases), [act runners docs](https://nektosact.com/usage/runners.html), [Dagger vs GitHub Actions](https://computingforgeeks.com/dagger-vs-github-actions-comparison/)

Key takeaways:
- Runs GitHub Actions workflows locally using Docker; **works offline** with cached images; eliminates the commit-push-wait cycle
- Now **v0.2.89** (2026-06-01), actively maintained on a monthly cadence — recent releases are maintenance/dependency updates; last notable feature: scalar values + template expressions in matrix strategies (`${{ fromJSON(...) }}` dynamic matrices)
- Usability addition: the **GitHub Local Actions VS Code extension** (GUI front-end for act)
- Alternatives unchanged in substance: `gitlab-runner exec` still weak; **Dagger** remains complementary (pipelines-as-code that run identically on laptop and in CI), not a replacement
- Limitations: not 100% compatible with all Actions features (some marketplace actions need adaptation), requires Docker

**Relevance**: The workspace generates CI workflow templates (`.agent/templates/ci_workflow.yml`) for project repos. `act` could run these locally as a pre-push check or during outages (now more compelling given the sustained GitHub reliability crisis). The ROS 2 container images used in CI (`ros:jazzy-ros-core`) work with `act` since they're standard Docker images.

---

## ROS 2 Offline Development Patterns

**Added**: 2026-03-04 | **Updated**: 2026-07-14 | **Sources**: [ROS 2 releases](https://github.com/ros2/ros2/releases), [Jazzy patch 8 (2026-06-18)](https://discourse.openrobotics.org/t/new-packages-and-patch-release-for-jazzy-jalisco-2026-06-18/55545), [Lyrical Luth release notes](https://docs.ros.org/en/lyrical/Releases/Release-Lyrical-Luth.html), [colcon-core 0.21.0](https://pypi.org/pypi/colcon-core/json)

Key takeaways:
- **Jazzy remains actively maintained**: patch release 8 / sync 2026-06-18 brought 97 new packages + 608 updates; LTS support through May 2029 — **no migration urgency** for this Jazzy-based workspace
- **Lyrical Luth** (2026-05-22, LTS through May 2031) got its first patch 2026-06-23 (routine stabilization); highlights unchanged: Callback Group Events Executor (~10–15% less CPU), Rust generator default
- **colcon-core 0.21.0** (2026-06-01, first release since Oct 2025) is directly relevant to our `--symlink-install`-everywhere convention: adds a **fallback when setuptools is too recent for symlink install** (#735), plus non-package-name dependency support
- **Offline mechanics unchanged**: `rosdep` offline via `ROSDISTRO_INDEX_URL=file:///...`; local apt mirror or pre-installed Docker images; `colcon build`/`test` fully local; rosdep itself unchanged (0.26.0, Jun 2025)
- Online dependencies remain: initial `rosdep init/update`, `apt install` for new packages, `git clone` for source deps not yet in rosdep

**Relevance**: ROS 2 development is inherently local-friendly; `make build`/`make test` work offline once dependencies are installed. The gap remains in the governance layer (issues, PRs, reviews), not the build layer. Lyrical Luth is the distro to evaluate for any new LTS work, though this workspace currently targets Jazzy.

---

## Lightweight Self-Hosted Git Forges (Forgejo / Gitea)

**Added**: 2026-03-04 | **Updated**: 2026-07-14 | **Sources**: [Forgejo releases](https://forgejo.org/releases/), [Forgejo May 2026 report](https://forgejo.org/2026-05-monthly-report/), [Gitea releases](https://github.com/go-gitea/gitea/releases)

Key takeaways:
- Both are lightweight (~200MB RAM), single binary or Docker one-liner; offer repo hosting, built-in issue tracker, web UI, wiki, Actions-compatible CI runners; **GitLab CE is ~40× heavier**
- **Forgejo v16.0 ships 2026-07-15**: HTTP API for Actions artifacts + workflow/job logs, and **"Authorized Integrations"** (OIDC-style JWT federation from Forgejo Actions, GitHub Actions, GitLab CI/CD, AWS). **v11 LTS support ends 2026-07-16; v15 is now the LTS anchor (supported to 2027-07-15)** — quarterly major cadence holding
- **Gitea 1.27.0 released 2026-07-13**: workflow status badges, owner-level scoped workflows, job summaries, Jupyter rendering, reusable-workflow improvements, LFS/attachment security fixes. Caution flag: **1.26.3 (Jun 20) shipped a repo-page-breaking regression** — fixed next day in 1.26.4; skip 1.26.3
- **Forgejo Actions matured substantially** (v15): OIDC token support, `concurrency` blocks, matrix jobs, `runs-on` from earlier jobs, scheduled jobs — and v16's artifact/log APIs narrow the GH-compat gap further
- The two are **increasingly divergent, not "near-identical"**: version schemes fully decoupled (v16 vs v1.27), Forgejo's cadence + Actions feature lead mean feature sets no longer track 1:1; community governance reinforces the "safer long-term" point

**Relevance**: The workspace uses a GitLab instance on the robot network for repo sync. Forgejo/Gitea could replace it at a fraction of the resource cost while adding a built-in issue tracker and web UI. Forgejo Actions' OIDC/concurrency/matrix maturation makes a local CI mirror more viable. Combined with git-bug, this enables a fully local-first workflow where GitHub becomes a publication channel rather than a dependency. See [#345](https://github.com/rolker/ros2_agent_workspace/issues/345).

---

## Agent Orchestrators for Parallel Coding Agents

**Added**: 2026-03-11 | **Updated**: 2026-07-14 | **Sources**: [ComposioHQ/agent-orchestrator](https://github.com/ComposioHQ/agent-orchestrator), [Vibe Kanban shutdown](https://www.vibekanban.com/blog/shutdown), [Augment Code: open-source agent orchestrators](https://www.augmentcode.com/tools/open-source-agent-orchestrators), [Deloitte 2026: AI agent orchestration](https://www.deloitte.com/us/en/insights/industry/technology/technology-media-and-telecom-predictions/2026/ai-agent-orchestration.html)

Key takeaways:
- **Agent-orchestrator** (ComposioHQ, MIT) actively developed (nightly cadence, v0.10.2-nightly Jul 2026); repositioned as a "meta-harness agent IDE" — the orchestrator is itself an AI agent that decomposes the backlog, assigns tasks to coding agents (Claude Code, Codex, Cursor, Aider, Goose), and closes the loop on CI failures / review comments / merge conflicts
- **Vibe Kanban sunset is complete**: Bloop shutdown 2026-04-10, remote services removed, promised community handoff has not materialized (last release v0.1.44, 2026-04-24; issues unanswered) — treat as gone, not community-maintained
- **No open-source winner has emerged**; the stronger trend is **first-party absorption**: Anthropic's Dynamic Workflows, implicit Agent Teams, and background subagents with auto-commit/push/PR (Claude Code v2.1.198) are absorbing the table-stakes features third-party orchestrators filled
- Convergent patterns remain industry-standard: (1) git worktree isolation per agent, (2) supervisor/coordinator, (3) event-driven reactions to CI/review, (4) dashboard for human oversight, (5) YAML config
- The bottleneck remains "how do I run multiple agents in parallel without chaos?", but the answer is increasingly shipping inside the agent products themselves

**Relevance**: This workspace already has the foundational primitives all orchestrators build on: worktree isolation scripts, draft-PR visibility, workforce protocol, multi-framework identity. The gaps relative to agent-orchestrator remain: (1) **no reaction system** — CI failures and review comments need manual agent re-engagement; (2) **no unified dashboard** of agent progress + CI + PRs; (3) **no automated task decomposition**; (4) **no inter-agent messaging** (Agent Teams, above, is the likeliest path to close this). A reaction system for CI/review events would deliver the most immediate value. See [#375](https://github.com/rolker/ros2_agent_workspace/issues/375). **Update (2026-05-29)**: Claude Code's **Dynamic Workflows** (see the "Anthropic Model Lineup" entry above) now provides an in-product version of primitives (1)–(2) for the Claude Code runtime specifically — Claude-authored fan-out over isolated-context background subagents — narrowing the gap without a third-party orchestrator. **Update (2026-07-14)**: v2.1.198's background subagents with auto-commit/push/PR further narrow gap (1) for the Claude runtime.

---

## Operational-Assistant Behavior Under Live Time Pressure

**Added**: 2026-05-27 | **Updated**: 2026-07-14 | **Sources**: [Google SRE incident response](https://sre.google/workbook/incident-response/), [Azure SRE Agent GA](https://techcommunity.microsoft.com/blog/appsonazureblog/announcing-general-availability-for-the-azure-sre-agent/4500682), [Azure SRE Agent FAQ](https://learn.microsoft.com/en-us/azure/sre-agent/faq), [PagerDuty SRE Agent triage](https://www.pagerduty.com/blog/ai/new-enhancements-to-pagerdutys-sre-agent-triage-faster-without-waking-a-human/), [Sterile flight deck rule](https://en.wikipedia.org/wiki/Sterile_flight_deck_rule), [Sheridan & Verplank, levels of automation](https://www.hfes.org/Portals/0/Documents/Sheridan.pdf), [AIR: Agent Incident Response (arXiv:2602.11749)](https://arxiv.org/abs/2602.11749)

Key takeaways:
- **Mitigate before diagnose** (SRE): during an active incident, restore service first (rollback, drain, scale) and explicitly *defer* root-cause analysis until after stabilization. Investigation is time-boxed — don't deep-dive while the system (or, for us, scarce operating time) is impacted.
- **Sterile cockpit rule** (aviation, FAA 1981): during critical phases of flight, *only* activities essential to safe operation are permitted; all non-essential work is forbidden. Direct analog for a "live-ops" agent mode — no doc polish, refactors, or speculative deep dives while the boat is on the water.
- **Confidence-threshold escalation** (AI SRE 2026): agents auto-execute only *pre-approved runbooks for known failure classes*; below a configurable confidence threshold they **stop and hand off to a human with a prepared context summary**. Deterministic checks (a typed "FactStore") run *before* any LLM call, and conflicting facts cap confidence. A concrete anti-rabbit-hole mechanism.
- **Human-in-the-loop → human-on-the-loop as trust grows**: the 2026 industry default is approval-gated mitigation, relaxing to supervised autonomy as accuracy is validated — mirrors the workspace principle "tight by default, relaxable as confidence grows."
- **Adjustable / discretionary autonomy** (Sheridan & Verplank 1978, originally for *undersea teleoperators*): a selectable level-of-automation; operators rate systems more positively when they can change the autonomy level. Frames "deployment mode" as a discretionary autonomy shift, not a fixed behavior set.
- **Incident Commander + short runbooks** (ICS): one accountable human directs; runbooks stay short, version-controlled, alert-linked, and reviewed after each incident. The SRE lifecycle (Prepare → Detect → Respond → Recover → Learn) maps cleanly onto the deployment lifecycle ([0] start → [1] session / [2] recovery → [3] wrap-up / [4] next).
- **2026-07-14 update — the patterns are shipping as products**: Azure SRE Agent has been **GA since 2026-03-10** (1,300+ agents, 35k+ incidents mitigated at GA; June docs add an explicit **Reader-mode vs privileged-mode split** — a concrete adjustable-autonomy implementation — plus MCP-server custom tools); PagerDuty's SRE Agent (May 2026) markets *triage-before-waking-a-human* with persistent cross-incident memory (converging on the FactStore idea); new papers apply incident-response discipline *to* agents (AIR, arXiv:2602.11749) and simulate consequences of response strategies before acting (arXiv:2602.13156).

**Relevance**: Grounds the "deployment mode" design ([#495](https://github.com/rolker/ros2_agent_workspace/issues/495), [#477](https://github.com/rolker/ros2_agent_workspace/issues/477)) — an urgency-aware agent mode for live field deployments where on-water operating time is the scarce resource and agents have rabbit-holed during setup/troubleshooting. Sterile-cockpit + mitigate-before-diagnose + confidence-threshold-escalation supply a concrete "urgency contract" vocabulary; adjustable-autonomy and human-on-the-loop frame it as a *scoped, relaxable mode* — a behavioral sibling to field mode (ADR-0011), which already established the precedent of a mechanically-scoped exception to default agent rules.
