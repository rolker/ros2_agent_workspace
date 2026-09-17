# Research Digest: Workspace

<!-- Last updated: 2026-09-17 -->
<!-- If older than 30 days, consider running /research --refresh; entries older than 90 days should be flagged for review -->
<!-- 2026-09-11 full re-survey: all 17 prior entries re-checked and re-stamped, one new entry added (Scheduled & Background Maintenance Agents — the "janitor" pattern), prompted by issue #569. Headline shifts this window: **Claude Opus 5** shipped 2026-07-24 as Claude Code's default Opus (1M context, effort toggle) and **Fable 5.1 / Mythos 5.1** 2026-09-01 (~25-45% cheaper via cache-read pricing); **auto mode became the DEFAULT permission mode** for new Pro/Max/Team sessions on 2026-08-14 and **fork mode is on by default**; **cross-session messaging** landed (macOS/Linux); the **MCP 2026-07-28 spec shipped final** (stateless core); **AGENTS.md is now a Linux Foundation standard** read by 20+ tools across ~700k files; **SpaceX-Cursor closed 2026-08-14**; GitHub had **five August incidents including a 7h47m outage on Aug 17**; git-bug still has **no release since v0.10.1 (May 2025)** — now 16 months. Note for the janitor work: the garbage-collection pillar of harness engineering now has first-party product form (Claude Code Routines), and Anthropic's own advertised Routine examples include weekly documentation-drift detection. -->

## Command Runner Alternatives to Make

**Added**: 2026-02-27 | **Updated**: 2026-09-11 | **Sources**: [just CHANGELOG](https://github.com/casey/just/blob/master/CHANGELOG.md), [just releases](https://github.com/casey/just/releases), [just-mcp](https://github.com/promptexecution/just-mcp), [Task releases](https://github.com/go-task/task/releases), [LWN: Just, a command runner](https://lwn.net/Articles/1047715/)

Key takeaways:
- `just` shipped **1.57.0 (2026-07-19) and 1.58.0 (2026-08-03)**, sustaining the ~2-3 week cadence; the substantive gap-closing releases remain **1.54 (cached recipes with file inputs/outputs + `--clean`)** and **1.55 (`--jobs` parallelism)**, which together gave it Make's incremental-rebuild and parallel-execution core
- The old "just is a command runner, not a build system" framing is now obsolete — it does incremental + parallel since 1.54/1.55
- **`just-mcp` status quo holds** (production-ready MCP server, stdio + Docker, agent-oriented) — no major new development through Aug 2026
- `task` (go-task) and `mise` unchanged in substance this window; `mise` still bundles tool-version management that adds little for ROS 2 where `apt`/`rosdep` manage toolchains
- `colcon defaults.yaml` can absorb build flags currently hardcoded in scripts, regardless of which runner is chosen

**Relevance**: The workspace uses Make purely as a command runner, not for dependency-based builds. The case for `just` is unchanged from the last window and still materially better than when the workspace chose Make — cached recipes could replace the `.make/` stamp-file machinery natively, and just-mcp keeps the MCP-accessible-commands pathway open. No new argument to act on; revisit if the `.make/` stamp logic grows.

---

## AGENTS.md — Cross-Platform Agent Instruction Standard

**Added**: 2026-02-27 | **Updated**: 2026-09-11 | **Sources**: [agents.md spec](https://agents.md/), [Copilot code review reads AGENTS.md (2026-06-18)](https://github.blog/changelog/2026-06-18-copilot-code-review-agents-md-support-and-ui-improvements/), [AGENTS.md spec guide (Morph, 2026)](https://www.morphllm.com/agents-md-guide), [AGENTS.md vendor-support audit (2026-08-28)](https://blakecrosley.com/blog/agents-md-patterns), [AGENTS.md specification (ASDLC)](https://asdlc.io/practices/agents-md-spec/)

Key takeaways:
- **Now formally a Linux Foundation standard** under the Agentic AI Foundation — no single-vendor ownership. Reported reach has grown to **~700,000 files** and **20+ natively-supporting tools** (Codex, Cursor, Copilot, Gemini CLI, Aider, Windsurf, Zed, Factory, Jules and others); the widely-cited "60,000 repositories" figure is an aging floor, not current
- **GitHub Copilot code review reads root-level `AGENTS.md`** (since 2026-06-18) and applies it when generating review feedback — the standard now steers the review surface, not just the coding surface
- A **vendor-documentation audit dated 2026-08-28** re-verified which tools actually honor the file rather than merely claiming support — worth consulting before assuming a given agent reads it
- No breaking spec changes this window; recommended size guidance holds at **≤150 lines**, closest `AGENTS.md` to the edited file wins, user prompts override all
- AAIF flagship events: **AGNTCon + MCPCon Europe Sep 17-18 (Amsterdam)**, North America Oct 22-23 (San Jose)
- For multi-tool workspaces, the `@AGENTS.md` import pattern in framework-specific files (e.g. CLAUDE.md) is preferred over symlinking

**Relevance**: Already adopted here. Copilot code review reading AGENTS.md means our workspace rules steer the Copilot PR reviews we already receive, which raises the value of per-repo AGENTS.md coverage ([ADR-0017](../../docs/decisions/0017-extend-agents-md-to-project-repos.md), [#563](https://github.com/rolker/ros2_agent_workspace/issues/563)) — currently 10 of 45 project repos. The Linux Foundation stewardship removes the "single-vendor bet" risk that was a mild open question when the workspace adopted the file.

---

## AI Agent Spec Writing Best Practices

**Added**: 2026-02-27 | **Updated**: 2026-09-11 | **Sources**: [Osmani, "How to Write a Good Spec for AI Agents" (O'Reilly, Jan 2026)](https://www.oreilly.com/radar/how-to-write-a-good-spec-for-ai-agents/), [GitHub Spec Kit docs](https://github.github.com/spec-kit/), [Spec Kit review (2026)](https://vibecoding.app/blog/spec-kit-review), [SDD 2026 guide (BCMS)](https://thebcms.com/blog/spec-driven-development), [SDD paper (arXiv:2602.00180)](https://arxiv.org/html/2602.00180v1)

Key takeaways:
- **Spec-Driven Development remains the dominant codification** and has consolidated around GitHub's Spec Kit: **v1.0.1 (2026-08-21)**, a Python CLI at roughly **129,000 stars** with **30+ agent integrations** (Copilot, Claude Code, Cursor, Gemini CLI, Codex CLI, Goose, Windsurf and more) as of 2026-08-30
- **The evidence base has not caught up to the adoption.** Thoughtworks placed Spec Kit in the **Assess** ring (Radar vol. 34, Apr 2026) with an explicit caution that the workflows are elaborate and opinionated and that some tools generate spec files that are hard to review. Community claims of 60-80% fewer rework cycles are self-reported; there are still **no peer-reviewed defect-reduction or velocity metrics at scale**
- **EARS** (Easy Approach to Requirements Syntax) is the near-universal acceptance-criteria format
- **Three-tier boundary system** — "Always" (autonomous), "Ask first" (human approval), "Never" (hard stop) — still outperforms flat rule lists
- **Self-audit instruction**, **SPEC.md as session anchor**, **conformance suites**, and the **two-phase task pattern** (draft spec in plan mode, then execute) all still hold
- Line guidance: **150-200 lines, then split into nested/subdirectory files**

**Relevance**: The Always/Ask/Never taxonomy was adopted directly into AGENTS.md and the self-audit pattern maps to our post-task verification checklist. The Thoughtworks caution is the useful new input: our `plan-task` → `review-plan` → implement chain is a spec-driven workflow, and "spec files that are hard to review" is exactly the plan-drift failure mode we already see flagged by Copilot. Keep plans reviewable rather than exhaustive.

---

## Harness Engineering — Production-Scale Agent-First Development

**Added**: 2026-02-27 | **Updated**: 2026-09-11 | **Sources**: [OpenAI blog (Lopopolo, Feb 2026)](https://openai.com/index/harness-engineering/), [Fowler & Böckeler, "Harness engineering" (Apr 2026)](https://martinfowler.com/articles/harness-engineering.html), [Agentic Harness Engineering (arXiv:2604.25850)](https://arxiv.org/abs/2604.25850), [awesome-harness-engineering](https://github.com/ai-boost/awesome-harness-engineering), [Natural-Language Agent Harnesses (arXiv:2603.25723)](https://arxiv.org/pdf/2603.25723), [Harnessing Agent Skills (arXiv:2606.20631)](https://arxiv.org/pdf/2606.20631)

Key takeaways:
- Team of 3-7 engineers built ~1M LOC product via Codex with **zero human-written code AND zero human code review** over 5 months; ~1,500 PRs. **Claim remains neither independently validated nor debunked**; Fowler's counterpoint stands (the harness enforces how code is written, not that it does what users need)
- The **harness** (environment, constraints, feedback loops) matters more than the agent's capabilities; three components: **context engineering**, **architectural constraints**, **garbage collection** (background agents fighting entropy)
- **New this window — the six reusable primitives** every agent harness shares, added to the community reference in Aug 2026: **approval gates, structured error reporting, escalation paths, sandboxed workspaces, audit trails, verifiers**. This is a usable self-audit checklist: the workspace has all six in some form (permission modes / progress.md entries / checkpoints / worktrees + container dispatch / git history + dev logs / CI + `ci_local.sh`)
- AGENTS.md should be a ~150-line **map**, not an encyclopedia; **lint errors as agent teaching**; **"anything not in-context doesn't exist"**; **"corrections are cheap, waiting is expensive"**
- **Optimism asymmetry has an empirical anchor**: arXiv:2604.25850 (Fudan/PKU) has an "evolve agent" auto-editing its own harness (Terminal-Bench 2 pass@1 69.7%→77.0% in 10 iterations, beating a human-designed harness) while quantifying **regression blindness** — fix-prediction precision 33.7% vs regression-prediction precision 11.8%
- **Skills are getting an architectural treatment**: arXiv:2606.20631 proposes a reference architecture for skill-mediated LLM agents (progressive disclosure, skill selection, isolation) — the closest thing yet to a formal model of what our `.claude/skills/` collection does informally
- Framing has hardened into **three maturity phases: prompt engineering → context engineering → harness engineering**, with harness the main 2026 investment target

**Relevance**: Validates worktree isolation, progressive disclosure, and artifact-based communication. The six-primitive checklist is directly usable as an `audit-workspace` check. The regression-blindness finding reinforces our Quality Standard's insistence on tests plus adversarial review over agent self-assessment — especially for marine-safety code. **The garbage-collection pillar is the one we still do not have as a loop** — see the new "Scheduled & Background Maintenance Agents" entry below and [#569](https://github.com/rolker/ros2_agent_workspace/issues/569).

---

## Multi-Agent Engineering Patterns

**Added**: 2026-02-27 | **Updated**: 2026-09-11 | **Sources**: [GitHub blog, "Multi-agent workflows often fail" (Feb 2026)](https://github.blog/ai-and-ml/generative-ai/multi-agent-workflows-often-fail-heres-how-to-engineer-ones-that-dont/), [GitHub Copilot app (GA 2026-06-17)](https://github.blog/news-insights/product-news/github-copilot-app-the-agent-native-desktop-experience/), [Claude Code multi-agent orchestration patterns (2026)](https://thepromptshelf.dev/blog/claude-code-multi-agent-orchestration-patterns-2026/), [Agent teams vs sub-agents (MindStudio)](https://www.mindstudio.ai/blog/claude-code-agent-teams-vs-sub-agents)

Key takeaways:
- Core thesis unchanged: most multi-agent failures come from **missing structure**, not model capability — treat agents as distributed system components
- **Typed schemas** for inter-agent data prevent field-name drift; **action schemas** (discriminated unions) prevent contradictory actions; **MCP as enforcement layer** validates tool I/O before execution
- **Distributed systems principles apply**: design for failure, validate at boundaries, constrain before scaling, log intermediate state, design for idempotency
- **New practitioner consensus on mode selection**: match the orchestration mode to the task rather than defaulting to the heaviest. Sequential work, same-file edits, and step-dependent pipelines belong in **subagents**; only genuinely parallel, negotiation-heavy work justifies peer teams. The blunt version doing the rounds: native subagents cover ~80% of multi-agent needs
- **Cost control by configuration, not discipline**: commit per-role subagent configs to the repo pinning a model per role (review on a mid-tier model, lint on a small one) so nobody defaults every agent to the most expensive option
- **GitHub productized the isolation pattern**: the standalone Copilot app (GA 2026-06-17) runs each agent session in its own auto-managed **git worktree** (~10 concurrent), with Agent Merge and sandboxes; GitHub Desktop 3.6 added worktrees too

**Relevance**: The mode-selection guidance is a direct check on `run-issue`, which dispatches phases sequentially as fresh-context sub-agents — that is the recommended shape, not an under-use of teams. The committed-per-role-model idea maps onto `dispatch_subagent.sh --model` and our specialist review sub-agents, where model choice is currently ad hoc per invocation rather than pinned per role. Worth considering as a cheap, mechanical cost control.

---

## Claude Code Agent Teams

**Added**: 2026-02-27 | **Updated**: 2026-09-11 | **Sources**: [Claude Code Agent Teams docs](https://code.claude.com/docs/en/agent-teams), [Claude Code What's New](https://code.claude.com/docs/en/whats-new), [Agent teams vs sub-agents (MindStudio)](https://www.mindstudio.ai/blog/claude-code-agent-teams-vs-sub-agents)

Key takeaways:
- **Still experimental and env-gated** behind `CLAUDE_CODE_EXPERIMENTAL_AGENT_TEAMS=1` — the first-party weekly digests through **Week 34 (Aug 21, 2026)** announce no GA flip. Secondary blogs listing "Agent Teams" among August's shipped features are conflating the gated feature with the general multi-session work below; **do not treat Teams as GA**
- One session is team lead; teammates are full independent Claude Code instances with their own context windows that **talk peer-to-peer** via a mailbox and **claim work off a shared task list** — unlike subagents, which only report to their parent. Plan-approval, teammate hooks (`TeammateIdle`, `TaskCreated`, `TaskCompleted`), task dependencies with auto-unblock, and file-locking on claims all still as documented
- **What did ship generally is the cheaper adjacent capability**: **cross-session messaging** on macOS and Linux (Week 32, Aug 3-7) lets ordinary Claude Code sessions message each other, and typing **`@` mentions another session by name** (Week 33). This delivers the inter-agent-messaging gap-closer without the experimental flag
- **Fork mode is on by default** in interactive sessions since Week 33 (Aug 10-14) — Claude hands a side task to a subagent that inherits the full conversation
- Three-tier choice persists: subagents (decomposable, coordination-free), Dynamic Workflows (deterministic scripted fan-out at scale), Agent Teams (~2-5 persistent named peers negotiating live)
- **Cost is linear in parallelism** — no separate agent pricing; N agents burn quota N times over

**Relevance**: Our worktree infrastructure already provides the isolation Teams assume. The important correction this window is that **cross-session messaging is generally available while Teams is not** — the [WORKFORCE_PROTOCOL](../WORKFORCE_PROTOCOL.md) coordination gap (multiple terminals, no channel between them) can now be closed with a shipped feature instead of an experimental one. Teams remains too unstable to hard-wire into governed scripts.

---

## Anthropic Model Lineup (Fable 5.1 / Opus 5), Auto Mode & Dynamic Workflows

**Added**: 2026-05-29 | **Updated**: 2026-09-11 | **Sources**: [Introducing Claude Opus 5](https://www.anthropic.com/news/claude-opus-5), [Introducing Claude Fable 5.1 and Mythos 5.1](https://www.anthropic.com/claude-fable-and-mythos-5-1), [Introducing Claude Sonnet 5](https://www.anthropic.com/news/claude-sonnet-5), [Claude Code What's New](https://code.claude.com/docs/en/whats-new), [Models overview](https://platform.claude.com/docs/en/about-claude/models/overview), [Axios on Opus 5](https://www.axios.com/2026/07/24/anthropic-releases-new-model-opus-5)

Key takeaways:
- **Claude Opus 5 shipped 2026-07-24** and is **Claude Code's default Opus model**: 1M-token context, fast mode at $10/$50 per MTok, and a **low/medium/high effort toggle** that trades cost against capability. Positioned as the everyday model for developers, near Fable-class intelligence at roughly half the price. Default model on Claude Max, strongest model on Pro
- **Fable 5.1 and Mythos 5.1 shipped 2026-09-01**: same sticker price as Fable 5 but **~25% cheaper for typical workloads and up to ~45% for highly agentic work**, entirely through reduced cache-read pricing — which specifically favors long-running, high-cache-hit agent sessions like ours. Fable remains the recommendation for the most complex long-running autonomous tasks
- **Current lineup**: Fable 5.1 → Opus 5 → Sonnet 5 → Haiku 4.5. Any "Opus 4.8 is the current default" or "Fable 5 is the newest" framing elsewhere in the workspace is now stale
- **Auto mode became the DEFAULT permission mode** for new sessions on Pro, Max and Team plans starting **2026-08-14** (Week 32) — a classifier handles permission prompts, safe actions run uninterrupted, risky ones are blocked. It has been hardened progressively: blocks destructive git commands you didn't ask for (Week 25), blocks transcript tampering and asks before `rm -rf` on unresolved variables (Week 28), hard deny rules that override allow exceptions (Week 19)
- **Other shipped primitives worth knowing**: the **Monitor** tool streams background events into the conversation so an agent can tail logs and react live (Week 15); **`/loop`** self-paces when you omit an interval (Week 15); **`/autofix-pr`** turns on PR auto-fix from the terminal (Week 15); **`/code-review` runs as a background subagent** (Week 30); the **Claude Security plugin** runs a multi-agent vulnerability scan and turns chosen findings into patches (Week 30); **self-hosted environments** run Claude Code cloud sessions on infrastructure the organization operates (public beta, Team/Enterprise, Week 32)
- **Dynamic Workflows** still research preview: Claude-authored JS orchestration over background subagents, trigger keyword `ultracode`, caps of 1,000 agents per run and 16 concurrent

**Relevance**: Three things matter operationally here. First, **auto mode is now the default**, which changes the assumed baseline for every permission-allowlisting decision in `.claude/settings.json` — the allowlist work behind `dlog.sh` and `progress_append.sh` was solving prompt friction that auto mode now partly absorbs. Second, **Fable 5.1's cache-read pricing rewards exactly our usage shape** (long sessions, large stable context from AGENTS.md and knowledge files), which shifts the dispatch economics assumed by `dispatch_subagent.sh --model`. Third, **Monitor and `/loop`** are in-product primitives for the periodic-check pattern the janitor issue is about, though both are session-scoped rather than unattended.

---

## Scheduled & Background Maintenance Agents (the "janitor" pattern)

**Added**: 2026-09-11 | **Sources**: [Claude Code Routines launch (2026-04-14)](https://code.claude.com/docs/en/whats-new/2026-w16), [Routines tutorial: schedule, API, GitHub triggers (Builder.io)](https://www.builder.io/blog/claude-code-routines), [Anthropic adds routines for scheduled agent tasks (Tessl)](https://tessl.io/blog/anthropic-adds-routines-to-claude-code-for-scheduled-agent-tasks), [Background AI agents for KTLO automation (metacto)](https://www.metacto.com/blogs/background-agents-ktlo-autonomous), [Agentic maintenance (codemyspec)](https://codemyspec.com/blog/agentic-maintenance), [Continuous documentation as an agent-driven practice (AgentPatterns)](https://www.agentpatterns.ai/workflows/continuous-documentation/), [Mintlify: how to stop documentation drift](https://www.mintlify.com/library/how-to-stop-documentation-drift), [TEPA: revoking stale memories (arXiv:2608.07429)](https://arxiv.org/html/2608.07429v2)

Key takeaways:
- **The pattern now has first-party product form.** Claude Code **Routines** launched 2026-04-14: a saved prompt plus a repo set plus MCP connectors, run on Anthropic-managed cloud infrastructure, triggered by **a recurring schedule, a GitHub event (PR opened, release published), or an HTTP API call**. Research preview on paid plans, with daily run caps by plan (Pro 5, Max 15, Team/Enterprise 25) — the plan floor is reported inconsistently across secondary sources (some say Max and above), so verify against the current docs before designing around a tier. Created from the web UI, from `/schedule`, or from the Desktop app
- **Anthropic's own advertised Routine patterns are the janitor use case verbatim**: nightly backlog triage summarized to Slack, deploy verification that scans CI output and posts a report, and **weekly documentation-drift detection that flags outdated API docs**
- **The framing in the harness-engineering literature is "garbage collection scaled to generation throughput"**: as agents produce code faster, documentation drifts, naming diverges and dead code accumulates, so background agents scan for deviations and **open targeted, reviewable pull requests on a cadence** rather than editing silently. The PR is the human gate
- **Drift detection splits by trigger**: push-triggered detection catches user-facing changes as they ship, schedule-triggered detection sweeps for accumulated rot. Both converge on the same output — a reviewable diff, not a report nobody reads
- **Output discipline is the recurring failure mode**: tools that file per-finding tickets get muted. The pattern that survives is one rolling report or one PR per sweep
- **Agent memory research is converging on the same problem from the other side**: TEPA (arXiv:2608.07429, Aug 2026) formalizes *revoking* a memory when newer evidence supersedes it, moving superseded claims to an archive rather than letting stale evidence dominate retrieval. The general design principle now current: **persist each claim with versioned evidence, and flag the claim stale when its evidence changes**
- Hosted, first-party scheduling became the norm in 2026 — OpenAI scheduled tasks, xAI Grok Automations (2026-07-16), Claude Code Routines — but self-hosted scheduling (GitHub Actions cron, plain cron on a host) remains a fully valid path, and is the only one that reaches infrastructure the vendors cannot see

**Relevance**: This is the missing third pillar for this workspace, tracked as [#569](https://github.com/rolker/ros2_agent_workspace/issues/569) with [#235](https://github.com/rolker/ros2_agent_workspace/issues/235) and [#126](https://github.com/rolker/ros2_agent_workspace/issues/126) as the earlier framings. All four of our detectors already exist as skills — `audit-workspace`, `audit-project`, `issue-triage`, and the `/research` digest-staleness header — and all four are manual-trigger, which is why this digest sat 59 days past its own 30-day nag. Two design inputs land directly on the open decisions in #569. **On where it runs**: Routines are cloud-hosted and GitHub-repo-scoped, which fits the workspace repo but not the gitcloud-origin field repos or the `layers/` tree that only exists on a dev host — so a cron-dispatched container via `docker_run_agent.sh` remains the better fit for anything touching layers, with Routines plausible for a GitHub-only slice. **On output shape**: the literature independently confirms the operator preference already recorded in #569 — one rolling report or one reviewable PR per sweep, never per-finding issue spam. **On the evidence-versioning idea**: flagging a claim stale when its evidence changes is a sharper mechanism than date-based staleness, and maps onto the `.agents/README.md` verified-parameter tables, which go stale precisely when the source they cite changes.

---

## Model Context Protocol (MCP) — Industry Standard

**Added**: 2026-02-27 | **Updated**: 2026-09-11 | **Sources**: [The 2026-07-28 Specification (final)](https://blog.modelcontextprotocol.io/posts/2026-07-28/), [MCP roadmap](https://modelcontextprotocol.io/development/roadmap), [The Register: MCP breaks with its stateful past](https://www.theregister.com/devops/2026/07/23/model-context-protocol-prepares-to-break-with-its-stateful-past/5276722), [Google: scaling agent infrastructure with MCP stateless updates](https://developers.googleblog.com/scaling-ai-agent-infrastructure-with-the-mcp-stateless-updates/), [Amazing ROS 2 MCP (Discourse)](https://discourse.openrobotics.org/t/amazing-ros-2-mcp-native-ros-2-mcp-server-for-ai-assisted-robotics-agents/55270)

Key takeaways:
- **The 2026-07-28 specification published final, on schedule** — the RC tracked in the last refresh shipped intact: **stateless protocol core** (no `initialize` handshake or session header; version and capabilities travel per-request, so plain load balancers work without session affinity and failover between redundant backends is simpler — statelessness makes redundancy easy, it does not supply it), **Multi Round-Trip Requests** for mid-call user interaction, header-based routing, cacheable list results, authorization hardening, a formal extensions framework, and updated Tier 1 SDKs
- **Roots, Sampling and Logging are formally deprecated** in this revision; Tasks demoted to an extension; formal ≥12-month deprecation policy now in force
- **Not a switch-off**: v1.x SDKs get security updates for at least 6 months; existing implementations were not broken on July 28
- **Scale**: Tier 1 SDK downloads near half a billion per month, with TypeScript and Python each past 1 billion cumulative; public server count past 10,000; roughly 41% of surveyed software organizations report limited or broad production use
- Governance under the Linux Foundation's Agentic AI Foundation; enterprise theme unchanged — **"shadow MCP" discovered at 3-10× expected rates**, with centralized gateway plus registry the consensus response
- **ROS 2 MCP servers keep expanding**: native **"Amazing ROS 2 MCP"** (rclpy-direct, no rosbridge), LCAS/ros2_mcp and wise-vision/ros2_mcp for introspection, robotmcp/ros-mcp-server (ROS 1+2), plus an MCP server for analyzing ROS and ROS 2 bag files in natural language

**Relevance**: Strategic but still not adopted here. The bag-analysis MCP server is the most immediately interesting new item given how much of our field diagnosis starts from a ROS bag on the dev host. `just-mcp` could expose workspace commands to any agent. MCP enforcement remains the cross-agent equivalent of Claude Code hooks — relevant if we ever need the same guardrails under Copilot or Gemini CLI. Note the naming hazard for hydro audiences: "bag" here means ROS bag, not bathymetric BAG.

---

## ROS 2 Agent Frameworks

**Added**: 2026-02-27 | **Updated**: 2026-09-11 | **Sources**: [RAI (RobotecAI)](https://github.com/RobotecAI/rai), [RAI paper (arXiv:2505.07532)](https://arxiv.org/pdf/2505.07532), [ROSA (NASA JPL)](https://github.com/nasa-jpl/rosa), [ROSClaw (arXiv:2603.26997)](https://arxiv.org/html/2603.26997), [Semantic autonomy framework for VLM robots (arXiv:2605.02525)](https://arxiv.org/pdf/2605.02525)

Key takeaways:
- **The established frameworks remain quiet** — no new releases surfaced this window for RAI, ROSA or EmbodiedAgents; the field is still fragmented with no dominant framework
- **ROSClaw's contribution is the contract, not the code**: a model-agnostic executive layer over ROS 2 formalized as **affordance manifest, observation normalizer, action validator, audit logger**. Still no public implementation; its four model backends are early-2026 snapshots and therefore already dated
- **MCP is the live integration path**, not bespoke frameworks — see the MCP entry above
- **Naming caution persists**: "rosclaw" names three unrelated things (the Kent State paper with no code, a dormant plugin repo, and an unrelated active project) — don't conflate
- Runtime-side focus continues; the development-time gap that this workspace occupies is still largely unserved by these frameworks

**Relevance**: ROSA remains the most immediately useful for diagnosing integration issues by querying a running simulation from a coding session. ROSClaw's action-validator plus audit-logger pairing is a convergent pattern with our cmd_vel-only autonomy stack and the Collision Monitor reflex — the design echoes our Quality Standard even though the code does not exist. Nothing here is ready to adopt.

---

## Agentic Coding Market and Tool Landscape

**Added**: 2026-02-27 | **Updated**: 2026-09-11 | **Sources**: [SpaceX acquires Cursor (Developers Digest)](https://www.developersdigest.tech/blog/spacex-cursor-acquisition-developer-guide-2026), [SpaceX-Cursor deal analysis (Codex KB)](https://codex.danielvaughan.com/2026/06/05/spacex-cursor-acquisition-coding-agent-market-codex-cli-competitive-positioning/), [GitHub Agentic Workflows public preview](https://github.blog/changelog/2026-06-11-github-agentic-workflows-is-now-in-public-preview/), [AI coding assistant market share 2026](https://www.ideaplan.io/blog/ai-coding-assistant-market-share-2026), [Q3 2026 consolidation forecast](https://www.digitalapplied.com/blog/ai-coding-q3-2026-projection-tool-consolidation-forecast)

Key takeaways:
- **The SpaceX-Cursor acquisition closed 2026-08-14** (announced 2026-06-16, $60B all-stock) — the largest venture-backed-startup acquisition on record. Cursor passed $2B annualized revenue in March 2026, doubling from $1B in November 2025
- **The market has segmented rather than consolidated**: Cursor and Windsurf compete IDE-first; Claude Code and Codex CLI compete terminal-first. The **terminal-first segment is growing faster in percentage terms from a smaller base** — which is the segment this workspace lives in
- GitHub Copilot holds roughly **37% share with about 4.7 million paid subscribers**; Copilot CLI reached GA with autonomous coding; Codex CLI runs inside customer AWS billing via Bedrock (GA 2026-06-01)
- **GitHub Agentic Workflows remains public preview** (since 2026-06-11), not GA: natural-language Markdown compiled to Actions YAML, read-only defaults, sandboxed containers behind an "Agent Workflow Firewall"
- **Worktree isolation remains the industry-standard pattern** across every serious entrant
- Adoption-versus-production gap persists — broad trial, narrow production deployment

**Relevance**: The convergence on worktree isolation continues to validate our architecture. GitHub Agentic Workflows is the thing to watch for the janitor: natural-language workflows compiled to Actions YAML with sandboxing would be the GitHub-native way to run a scheduled sweep on the workspace repo, but it is still preview and would not reach the gitcloud field repos or the `layers/` tree.

---

## GitHub Outage Resilience and Offline Development

**Added**: 2026-03-04 | **Updated**: 2026-09-11 | **Sources**: [GitHub availability report, August 2026](https://github.blog/news-insights/company-news/github-availability-report-august-2026/), [The August 17 outage in detail](https://dev.to/jamilxt/github-is-down-the-august-17-2026-outage-in-detail-1e36), [GitHub reports five August incidents](https://blockchain.news/news/github-august-2026-availability-report), [githubstatus.com](https://www.githubstatus.com/)

Key takeaways:
- **August was worse than June's brief improvement**: five incidents of degraded performance, headlined by a **7 hour 47 minute outage on 2026-08-17** affecting API requests, webhooks, pull requests and Copilot. Peak web/API error rates hit **20%**, and raw/archive download failures hit **50%**. Root cause: traffic saturation at a single datacenter compounded by service-mesh scaling limits
- **Copilot Cloud Agent degraded separately on Aug 20** — task status updates delayed nearly 10 hours from database latency caused by a regional cloud outage
- **Aug 13**: Enterprise Cloud team synchronization degraded, with IdP group syncs delayed 3-13 hours (median 8)
- **The Azure migration continues but slowly** — three database primaries transitioned by late August; full completion still framed as 2027
- Repair items focus on capacity monitoring, retry policies that amplified impact, and core-service resiliency
- **Multi-remote strategy** remains the most common resilience pattern (GitHub primary with Forgejo/GitLab/Gitea failover)

**Relevance**: This workspace depends on GitHub for issues, PRs, CI and Copilot reviews, and the August numbers are a regression rather than a recovery. The gitcloud mirror and field-mode carve-out ([ADR-0011](../../docs/decisions/0011-field-mode-for-non-github-origins.md)) already cover the code path; the exposed surface is still governance — the issue-first workflow, `gh`-based worktree scripts, and PR review all stop. Note the interaction with [#612](https://github.com/rolker/ros2_agent_workspace/issues/612) and [#613](https://github.com/rolker/ros2_agent_workspace/issues/613): git-bug refs are not pushed to the field remote, so the offline issue cache that would cover a GitHub outage is itself not syncing. Also relevant to the janitor design — a scheduled sweep that depends on the GitHub API needs to degrade gracefully, not report a false green.

---

## git-bug — Distributed Offline-First Issue Tracker

**Added**: 2026-03-04 | **Updated**: 2026-09-11 | **Sources**: [git-bug releases](https://github.com/git-bug/git-bug/releases), [git-bug repo](https://github.com/git-bug/git-bug)

Key takeaways:
- Stores issues as git objects (under `refs/bugs`), not files — no working-tree clutter, full version history, distributed by default
- **Offline-first**: create, edit, comment without network; syncs via `git push/pull` to any remote
- **Bidirectional bridges**: GitHub, GitLab, Jira, Launchpad; interfaces are CLI, TUI and web
- **v0.10.1 (2025-05-19) is still the latest release as of 2026-09-11 — now 16 months without one.** The repo shows no deprecation notice, but the release cadence has effectively stopped
- **git-issue** (Spinellis) is a simpler file-based alternative

**Relevance**: The workspace already depends on this (`sync_repos.py` includes git-bug, `gh_create_issue.sh` has a `GITBUG_CREATE=1` offline path). Sixteen months without a release is a meaningful escalation of the maintenance risk flagged in the last two refreshes, and it lands on live problems: [#612](https://github.com/rolker/ros2_agent_workspace/issues/612) (push_remote.py does not push `refs/bugs`, so a synced field remote has current code and stale issues) and [#613](https://github.com/rolker/ros2_agent_workspace/issues/613) (import staleness is invisible — a deployment issue was unreadable on the boat with no warning). Both are our bugs rather than git-bug's, but the dependency is now on a project that has not shipped in over a year. Worth a deliberate decision rather than continued drift.

---

## Local CI with Act (nektos/act)

**Added**: 2026-03-04 | **Updated**: 2026-09-11 | **Sources**: [nektos/act releases](https://github.com/nektos/act/releases), [act runners docs](https://nektosact.com/usage/runners.html), [Local GitHub Actions with act, now with a visual interface](https://github.com/nektos/act/discussions/6091)

Key takeaways:
- Runs GitHub Actions workflows locally using Docker; **works offline** with cached images; eliminates the commit-push-wait cycle
- **v0.2.89 (2026-06-01) remains the latest** — the monthly cadence noted last window has not produced a release since, though the project is not dormant
- Usability addition: the **GitHub Local Actions VS Code extension** provides a GUI front end
- Alternatives unchanged: `gitlab-runner exec` still weak; **Dagger** remains complementary (pipelines-as-code that run identically on laptop and CI), not a replacement
- Limitations: not fully compatible with every Actions feature, requires Docker

**Relevance**: Largely settled for us. The workspace answered this question its own way with `ci_local.sh` and the git-note attestation scheme ([ADR-0018](../../docs/decisions/0018-local-first-ci-verification.md)), which does something `act` does not — it produces a verifiable merge-gate artifact rather than just running the workflow. `act` remains a fallback for mirroring hosted CI exactly, a role `--clean-room` already fills. No action.

---

## ROS 2 Offline Development Patterns

**Added**: 2026-03-04 | **Updated**: 2026-09-11 | **Sources**: [ROS 2 releases](https://github.com/ros2/ros2/releases), [Lyrical Luth release timeline](https://docs.ros.org/en/jazzy/Releases/lyrical/release-timeline.html), [ros2_control Jazzy release notes (Aug 2026)](https://control.ros.org/jazzy/doc/release_notes/release_notes.html), [colcon-core on PyPI](https://pypi.org/pypi/colcon-core/json)

Key takeaways:
- **Jazzy remains actively maintained** with LTS support through May 2029 — **no migration urgency** for this Jazzy-based workspace, and `ros2_control` published fresh Jazzy release notes in Aug 2026
- **Lyrical Luth** (2026-05-22, LTS through May 2031) received patch release 1 on 2026-06-23; highlights unchanged (Callback Group Events Executor, roughly 10-15% less CPU; Rust generator default)
- **colcon-core is now 0.21.1 (2026-08-05)**, a maintenance release over 0.21.0 — 0.21.0 is the one that matters for us, adding a **fallback when setuptools is too recent for symlink install**, directly relevant to our `--symlink-install`-everywhere convention
- **Offline mechanics unchanged**: `rosdep` offline via `ROSDISTRO_INDEX_URL=file:///...`; local apt mirror or pre-installed Docker images; `colcon build`/`test` fully local
- Online dependencies remain: initial `rosdep init/update`, `apt install` for new packages, `git clone` for source deps not yet in rosdep

**Relevance**: ROS 2 development is inherently local-friendly and `make build`/`make test` work offline once dependencies are installed. The gap remains in the governance layer, not the build layer. The colcon-core setuptools fallback is worth knowing about if a symlink-install failure ever surfaces on a freshly-rebuilt host. Lyrical Luth is the distro to evaluate for new LTS work, but nothing forces a move.

---

## Lightweight Self-Hosted Git Forges (Forgejo / Gitea)

**Added**: 2026-03-04 | **Updated**: 2026-09-11 | **Sources**: [Forgejo v16.0 release](https://forgejo.org/2026-07-release-v16-0/), [Forgejo releases](https://forgejo.org/releases/), [Gitea releases](https://github.com/go-gitea/gitea/releases), [Forgejo vs Gitea 2026](https://techfuelhq.com/homelab/forgejo-vs-gitea-2026/)

Key takeaways:
- Both are lightweight (~200MB RAM), single binary or Docker one-liner; repo hosting, built-in issue tracker, web UI, wiki, Actions-compatible CI runners. GitLab CE is roughly 8-40× heavier depending on the comparison
- **Forgejo v16.0 released 2026-07-16, with v16.0.3 current as of 2026-08-20.** Important caveat: **v16 is a non-LTS release supported only until 2026-10-29** — do not deploy it as a durable service. **v15 is the LTS anchor** (supported to 2027-07-15); v11 LTS ended 2026-07-16
- v16 adds an HTTP API for Actions artifacts and workflow/job logs, plus **"Authorized Integrations"** (OIDC-style JWT federation from Forgejo Actions, GitHub Actions, GitLab CI/CD, AWS)
- **Gitea is at 1.27.2 (2026-08-13)** on the 1.27.x line, with quarterly major releases
- **Forgejo Actions matured substantially** in v15 (OIDC tokens, `concurrency` blocks, matrix jobs, scheduled jobs), and v16's artifact/log APIs narrow the GitHub-compatibility gap further
- The two are **increasingly divergent**: version schemes fully decoupled (v16 vs 1.27), feature sets no longer track 1:1

**Relevance**: The workspace uses a GitLab instance on the robot network for repo sync (gitcloud). Forgejo or Gitea could replace it at a fraction of the resource cost while adding a built-in issue tracker and web UI. **The LTS detail is the actionable part**: any migration should target Forgejo v15, not the newer v16, which goes unsupported in October. Combined with git-bug — whose 16-month release gap is now a concern — a forge with a real built-in tracker looks more attractive than it did, since it would address the same offline-issue problem with a maintained dependency. See [#345](https://github.com/rolker/ros2_agent_workspace/issues/345).

---

## Agent Orchestrators for Parallel Coding Agents

**Added**: 2026-03-11 | **Updated**: 2026-09-11 | **Sources**: [ComposioHQ/agent-orchestrator](https://github.com/ComposioHQ/agent-orchestrator), [9 open-source agent orchestrators (Augment Code)](https://www.augmentcode.com/tools/open-source-agent-orchestrators), [awesome-agent-orchestrators](https://github.com/andyrewlee/awesome-agent-orchestrators), [From conductor to orchestrator (2026 guide)](https://htdocs.dev/posts/from-conductor-to-orchestrator-a-practical-guide-to-multi-agent-coding-in-2026/), [Agent orchestrator directory](https://yetanotherorchestrator.app/)

Key takeaways:
- **Agent-orchestrator** (ComposioHQ, MIT) remains the leading open-source option: manages fleets of coding agents in parallel, **each in its own git worktree**, positioned as a meta-harness that decomposes the backlog and closes the loop on CI failures and review comments
- **New entrants are small and early**: **Conductor** (Melty Labs, free Mac app, Claude Code-specific), **Emdash** (open source), **Baton** (free desktop app). A directory site and an awesome-list now exist, which is itself a signal that no winner has emerged
- **Vibe Kanban is gone** — shutdown completed 2026-04-10, no community handoff materialized
- **First-party absorption is still the stronger trend**: Dynamic Workflows, background subagents with auto-commit/push/PR, cross-session messaging, and the Copilot app's auto-managed worktrees are absorbing the table-stakes features third-party orchestrators filled
- **Git worktrees are universally the isolation mechanism** — every tool in the category converged on it independently

**Relevance**: The workspace already has the primitives all of these build on: worktree isolation scripts, draft-PR visibility, the workforce protocol, multi-framework identity. Gaps relative to agent-orchestrator have narrowed since the last refresh. **Gap (4), inter-agent messaging, is now closable with a shipped Claude Code feature** (cross-session messaging, Week 32) rather than an experimental one. **Gap (1), a reaction system for CI failures and review comments, remains the highest-value missing piece** ([#375](https://github.com/rolker/ros2_agent_workspace/issues/375)) — and note that it is the same shape as the janitor: something watching for a condition and re-engaging an agent without a human remembering to. Gap (2), a unified dashboard, is partly served by `make dashboard` and `claude agents`.

---

## Operational-Assistant Behavior Under Live Time Pressure

**Added**: 2026-05-27 | **Updated**: 2026-09-11 | **Sources**: [Google SRE incident response](https://sre.google/workbook/incident-response/), [Azure SRE Agent overview](https://learn.microsoft.com/en-us/azure/sre-agent/overview), [Azure SRE Agent incident response docs](https://github.com/MicrosoftDocs/azure-docs/blob/main/articles/sre-agent/incident-response.md), [PagerDuty Operations Cloud Spring 2026 release](https://www.pagerduty.com/newsroom/pagerduty-operations-cloud-spring-2026-release/), [PagerDuty + Azure AI SRE Agent integration](https://www.pagerduty.com/blog/ai/pagerduty-azure-ai-sre-agent/), [Sterile flight deck rule](https://en.wikipedia.org/wiki/Sterile_flight_deck_rule), [Sheridan & Verplank, levels of automation](https://www.hfes.org/Portals/0/Documents/Sheridan.pdf)

Key takeaways:
- **Mitigate before diagnose** (SRE): restore service first, explicitly defer root-cause analysis until after stabilization, time-box investigation
- **Sterile cockpit rule** (aviation, FAA 1981): during critical phases only activities essential to safe operation are permitted — the direct analog for live-ops agent mode
- **Confidence-threshold escalation**: agents auto-execute only pre-approved runbooks for known failure classes; below threshold they stop and hand off with a prepared context summary. Deterministic checks run before any model call, and conflicting facts cap confidence
- **Adjustable autonomy is now shipped product, not theory**: Azure SRE Agent (GA 2026-03-10) offers **Review mode** (an administrator approves write actions) versus **Autonomous mode** (applied without waiting), and Microsoft reports it has mitigated **over 35,000 production incidents** on its own infrastructure
- **The autonomy ratchet is visible in the roadmap**: PagerDuty announced early access to its SRE Agent as a **Fully Autonomous Responder in H2 2026**, and a multi-agent fabric where its agent interacts with AWS DevOps Agent and Azure AI SRE. The industry is moving human-in-the-loop → human-on-the-loop exactly as predicted
- **Cross-system handoff pattern worth noting**: the Azure agent opens GitHub issues pre-filled with root-cause detail and incident links, and keeps the incident updated with status and follow-ups — an incident that writes its own paper trail
- **Adjustable / discretionary autonomy** (Sheridan & Verplank 1978, originally for *undersea teleoperators*): operators rate systems more positively when they can change the autonomy level

**Relevance**: Grounds deployment mode ([ADR-0014](../../docs/decisions/0014-deployment-mode.md), [#495](https://github.com/rolker/ros2_agent_workspace/issues/495)), which is now implemented rather than proposed — the urgency contract, sterile-cockpit rule and mitigate-before-diagnose vocabulary all came from this line of work. The new input is the **Review/Autonomous mode split as a shipped, named pattern**: our deployment mode is a single mode with a fixed contract, where Azure's is an operator-selectable level. Worth considering when the recovery checklist ([#496](https://github.com/rolker/ros2_agent_workspace/issues/496)) is designed. The auto-paper-trail pattern is close to what `dlog.sh` does for deployment logs, and the industry ratchet toward full autonomy is a useful reminder that our approval gates are a deliberate choice, not a temporary limitation.

---

## Where ROS 2 and Open-Source Projects Keep Planning Documents

**Added**: 2026-09-17 | **Sources**: [REP-2004](https://github.com/ros-infrastructure/rep/blob/master/rep-2004.rst), [ROS 2 Developer Guide](https://github.com/ros2/ros2_documentation/blob/rolling/source/The-ROS-2-Project/Contributing/Developer-Guide.rst), [ROS 2 Roadmap source](https://github.com/ros2/ros2_documentation/blob/rolling/source/The-ROS2-Project/Roadmap.rst), [ros2/design repo](https://github.com/ros2/design) / [design.ros2.org](https://design.ros2.org), [ros-navigation/docs.nav2.org roadmaps.md](https://github.com/ros-navigation/docs.nav2.org/blob/rolling/docs/community/roadmaps.md), [GitHub community standards checklist](https://docs.github.com/en/communities/setting-up-your-project-for-healthy-contributions/about-community-profiles-for-public-repositories), [MADR](https://adr.github.io/madr/), [ros-maritime/community README](https://github.com/ros-maritime/community/blob/master/README.md)

Key takeaways:
- **No ROS 2 document prescribes where vision, roadmap, ADR, or status docs live.** REP-2004 is the only REP touching documentation at all, and it is scoped narrowly to package-level docs (feature/API docs, a "quality declaration") — it never mentions roadmap, vision, or ADRs
- The one exception with real, load-bearing guidance is **design docs**: the ROS 2 Developer Guide sends significant-change design docs to the separate `ros2/design` repo, published at design.ros2.org
- **Roadmap and Governance are published as documentation-website pages, not as planning files in a code repository**, even at the ros2 core level — their sources are `.rst` files tracked in the separate `ros2_documentation` repo, with the live feature list on a GitHub Project board
- The pattern repeats in the other large ROS 2 project whose roadmap source is linked above: Nav2 keeps its roadmap in the separate `docs.nav2.org` repo, not in the navigation2 code repo. The same survey observed MoveIt, ros2_control and Autoware publishing planning material on their documentation sites rather than as code-repo files; those observations are not linked here and should be re-verified before being relied on
- **General OSS diverges here**: GitHub's own community-standards checklist has no ROADMAP/VISION slot at all, yet Kubernetes sub-projects and CNCF's template still keep a lightweight in-repo `ROADMAP.md`/`GOVERNANCE.md` even when real planning happens elsewhere
- MADR's own documentation explicitly recommends `docs/decisions/` for ADRs — matching this workspace's existing convention, though no ROS 2 upstream project surveyed had a named ADR practice to compare against
- The one field/marine example found, `ros-maritime/community`, puts governance directly in `README.md` (no separate file); its README points to a GitHub Project board that tracks a subset of projects and to a public Google Doc for meeting agendas and minutes — no roadmap document as such was found

**Relevance**: Directly informs the design draft `docs/design/planning_document_vocabulary.md` and the roadmap template `.agent/templates/roadmap.md`, both landing via [PR #638](https://github.com/rolker/ros2_agent_workspace/pull/638) ([issue #628](https://github.com/rolker/ros2_agent_workspace/issues/628)): there is no upstream convention to defer to, so this workspace's choice to keep ADRs at `docs/decisions/` (matching MADR) while giving roadmap/vision their own lightweight in-repo treatment is a reasonable, evidence-grounded position rather than a deviation from a norm that doesn't exist.

---

## Roadmap Formats Compared with the Workspace Template

**Added**: 2026-09-17 | **Sources**: [Kubernetes SIG Release roadmap.md](https://github.com/kubernetes/sig-release/blob/master/roadmap.md), [Prometheus Roadmap](https://prometheus.io/docs/introduction/roadmap/), [containerd ROADMAP.md](https://github.com/containerd/containerd/blob/main/ROADMAP.md), [Argo CD docs/roadmap.md](https://github.com/argoproj/argo-cd/blob/master/docs/roadmap.md), [Rust Project Goals 2025H2](https://goals.rust-lang.org/2025h2/goals.html), [github/roadmap](https://github.com/github/roadmap) / [Projects board](https://github.com/orgs/github/projects/4247), [rolker/agent_workspace docs/ROADMAP.md](https://github.com/rolker/agent_workspace/blob/main/docs/ROADMAP.md)

Key takeaways:
- **Verdict: tweak, not switch.** No surveyed format is a superset of this workspace's roadmap template — every one is missing at least two of the ten experience-derived features (forcing function, health-doc pairing, wrap-up leftover capture, periodic pruning, mark-done-in-place with `#N` kept, mandatory tracker id, reasoned deferred items, "what's not on this roadmap," a stated maintenance cadence, and parent/child links)
- Features unique to this workspace's template — present in none of the surveyed formats listed above (all of which were fetched and checked): a **named forcing function** (deployment start / periodic sweep) that triggers *re-reading* the roadmap; **pairing with a companion health/drift document**; **wrap-up leftovers landing on the roadmap by rule**; and an explicit **"what's not on this roadmap"** section
- Worth adopting from the survey: an explicit **owner/contact per item** (Rust Project Goals' Contact + Task Owners, bot-pinged for updates); a **priority/flagship axis distinct from status** (Rust's Flagship vs. Other Goals split; GitHub roadmap's product-area grouping is a looser analog); a **"recently completed" section kept visible for one cycle rather than deleted immediately** (Kubernetes SIG Release's "Done Deliverables"; containerd relies on closed-issue history for the same effect); and a **"Last reviewed: <date>" stamp** — no surveyed format had one, but the gap is universal enough to be worth closing given the workspace already has a `date`-stamping convention
- The **containerd caution**: containerd's maintainers deliberately rejected a prose `ROADMAP.md` as their primary mechanism because "they quickly become out of date," preferring an issue-label query that can't drift from reality. This workspace's format shares the same risk (a prose status table can go stale); the template's forcing-function and pruning rules are the mitigation, and they are load-bearing rather than optional boilerplate — if that cadence lapses, the containerd critique applies directly
- Closest surveyed candidates (Rust Project Goals for ownership/cadence, `rolker/agent_workspace`'s own UX roadmap for form) still lack the forcing-function/health-doc coupling entirely, so no single format could replace this workspace's template without losing features

**Relevance**: Directly informs the roadmap template `.agent/templates/roadmap.md` landing via [PR #638](https://github.com/rolker/ros2_agent_workspace/pull/638) ([issue #628](https://github.com/rolker/ros2_agent_workspace/issues/628)): the recommendation was to keep the template's existing spine and graft in an owner/priority axis and a review-cadence stamp, not to adopt an external format wholesale.
