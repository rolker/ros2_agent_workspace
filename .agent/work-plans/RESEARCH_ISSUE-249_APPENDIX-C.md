# Appendix C: Spec-Driven Development and AI Agent Specification Practices

**Parent report**: [RESEARCH_ISSUE-249.md](RESEARCH_ISSUE-249.md)
**Date**: 2026-02-21
**Researcher**: Claude Code Agent (Claude Opus 4.6)

---

## C1. Overview: The Spec-Driven Development Movement

A convergence is happening across the AI-assisted development ecosystem: multiple
vendors and practitioners are independently arriving at the same conclusion — that
**structured specifications before code** produce better agent outcomes than
iterative prompting alone.

Key milestones:
- **Addy Osmani** (Google Chrome DevEx lead) published "How to Write a Good Spec
  for AI Agents" on [O'Reilly Radar](https://www.oreilly.com/radar/how-to-write-a-good-spec-for-ai-agents/)
  (Jan 2026), synthesizing practices from Claude Code, Copilot, and Gemini users
- **GitHub Spec Kit** (Sept 2025) — open-source toolkit for spec-driven development,
  agent-agnostic, 40k+ stars ([github/spec-kit](https://github.com/github/spec-kit))
- **Amazon Kiro** (July 2025, GA Dec 2025) — IDE with spec-driven development built
  into the editor, now Amazon's internal standard ([kiro.dev](https://kiro.dev/))
- **Anthropic** frames the same discipline as "context engineering" — the developer's
  role shifts from writing code to curating the context agents operate in

This appendix evaluates these developments against the workspace's existing workflow
and identifies specific gaps and opportunities.

---

## C2. Osmani's Five Principles for Agent Specifications

From the [O'Reilly Radar article](https://www.oreilly.com/radar/how-to-write-a-good-spec-for-ai-agents/)
and the [author's blog](https://addyosmani.com/blog/good-spec/):

### C2.1 Start high-level, let the agent elaborate

Write the *what* and *why* — goals, constraints, user needs. Let the agent draft
the *how*. Osmani calls this "waterfall in 15 minutes" — rapid structured planning
that prevents wasted cycles without becoming heavyweight process.

**Workspace relevance**: The `ISSUE_PLAN.md` template already separates "Problem
Analysis" from "Implementation Tasks." The gap is that the spec/problem phase is
informal — it lives in issue descriptions of varying quality rather than a
structured artifact. The plan template's "Problem Analysis" section could be
expanded or split into a formal spec step.

### C2.2 Structure like a PRD

Treat the spec as a professional Product Requirements Document: clear sections,
parseable by a "literal-minded" AI. Osmani recommends blending PRD (user-centric
context, the *why*) with SRS (technical specifics the agent needs).

**Workspace relevance**: The workspace's work plans are closer to implementation
plans than PRDs. They describe *approach* well but often skip *requirements* and
*acceptance criteria* — the agent decides what "done" looks like rather than the
human specifying it upfront.

### C2.3 Keep things modular

Break specs into manageable, focused sections. Don't put everything in one massive
prompt. Use `@imports` or file references for progressive disclosure.

**Workspace relevance**: Already practiced — CLAUDE.md uses `@` references, and
the `.agent/knowledge/` directory provides topic-specific context. The report's
§5c documents this pattern.

### C2.4 Build in guardrails — the three-tier boundary system

GitHub's analysis of 2,500+ agent instruction files found the most effective specs
use a **three-tier boundary system** rather than a flat rule list:

| Tier | Meaning | Examples in this workspace |
|------|---------|--------------------------|
| **Always do** | Agent proceeds without asking | Run tests before committing, use worktrees, include AI signature |
| **Ask first** | Requires human approval | Modifying CLAUDE.md, adding new dependencies, changing CI config |
| **Never do** | Hard stops, no exceptions | Commit to main, `git checkout` branches, skip pre-commit hooks, push to wrong branch |

**Workspace relevance**: CLAUDE.md currently uses a mix of imperative rules and
emphasized warnings (`IMPORTANT`, `NEVER`). The three-tier structure would make
the intent clearer for agents across frameworks. This is directly actionable for
the AGENTS.md restructuring recommended in the main report's Phase 4.

**Proposed restructuring sketch for AGENTS.md:**

```markdown
## Boundaries

### Always (proceed without asking)
- Use worktrees for all code changes (never work in the main tree)
- Include AI signature on all GitHub Issues/PRs/Comments
- Run pre-commit hooks on every commit
- Reference issue numbers in branches and PRs

### Ask First (get human approval)
- Adding or removing packages
- Changing build infrastructure (Makefile, CI workflows)
- Modifying agent instruction files (CLAUDE.md, AGENTS.md)
- Architectural decisions that would warrant an ADR

### Never
- Commit to main or default branches
- Use `git checkout` to switch branches
- Skip hooks with `--no-verify`
- Commit files containing secrets
- Push to branches not matching your session
```

### C2.5 Iterate — specs are living documents

Don't write and forget. Update specs as decisions are made. Version-control them.
Commit them to the repo so both humans and agents can track evolution.

**Workspace relevance**: Work plans are already git-tracked and update as work
progresses. The gap is that plans are often created *after* the approach is decided
rather than *before* as a negotiation artifact. Osmani's point is that the spec
should be the *first* commit, not the plan.

### C2.6 Conformance testing

Via Simon Willison: build conformance suites — language-independent tests (often
YAML-based) that any implementation must pass. The spec itself should be testable.
"Those who get the most out of coding agents tend to be those with strong testing
practices."

**Workspace relevance**: Directly reinforces the Promptfoo recommendation in the
main report's B5. The conformance suite concept extends beyond instruction file
testing to feature-level verification: if a spec defines acceptance criteria, those
criteria should be expressible as automated tests that the agent runs before
declaring the task complete.

### C2.7 Adjust detail to complexity

Don't under-spec hard problems (the agent flails) or over-spec trivial ones (the
agent wastes context). A six-area checklist for completeness: commands, testing,
project structure, code style, git workflow, boundaries.

**Workspace relevance**: The workspace doesn't currently distinguish task complexity
levels. All issues get the same work-plan treatment. A lightweight/standard/detailed
spec tier (matching the transparency levels proposed in §7h) would reduce ceremony
for simple tasks while ensuring complex ones get adequate specification.

---

## C3. GitHub Spec Kit — Architecture and Evaluation

### C3.1 What it is

[Spec Kit](https://github.com/github/spec-kit) is an MIT-licensed toolkit that
scaffolds a four-phase gated workflow. It drops templates and agent-specific
slash commands into a repository. It works with Claude Code, Copilot, Gemini CLI,
Cursor, Windsurf, Kilo Code, and 7+ other agents.

Status: experimental. Creator Den Delimarsky stressed it is "not a production
scenario." Despite this, 40k+ stars and active community adoption.

### C3.2 Four-phase gated workflow

Each phase produces a specific artifact. You do not advance until the current
phase is validated.

| Phase | Command | Artifact | Purpose |
|-------|---------|----------|---------|
| **Specify** | `/specify` | `spec.md` | Goals, constraints, user needs — no tech stack |
| **Plan** | `/plan` | `plan.md` | Architecture, stack, libraries, data flow |
| **Tasks** | `/tasks` | `tasks.md` | Small implementable units with dependency ordering |
| **Implement** | (agent works) | Code | Agent executes tasks, human verifies at checkpoints |

Additionally, `/analyze` cross-checks all documents against the constitution for
naming drift, conflicts, and inconsistencies.

### C3.3 Repository structure

```
.specify/
├── memory/
│   └── constitution.md          # Non-negotiable project principles
├── scripts/
│   ├── create-new-feature.sh
│   ├── update-agent-context.sh
│   └── check-prerequisites.sh
├── templates/
│   ├── spec-template.md
│   ├── plan-template.md
│   └── tasks-template.md
└── specs/
    └── 001-feature-name/
        ├── spec.md
        ├── plan.md
        ├── tasks.md
        └── contracts/

.claude/commands/                 # Agent-specific slash commands
├── specify.md
├── plan.md
├── tasks.md
└── implement.md
```

Notable design choices:
- **Per-feature directories** under `.specify/specs/` — each feature gets its own
  spec/plan/tasks lifecycle, numbered sequentially
- **Constitution** — a persistent document of project-level principles that the
  agent references at every phase. Analogous to CLAUDE.md but agent-agnostic
- **Contracts directory** — formalized API or interface contracts for the feature
- **Parallel task markers** — tasks tagged `[P]` can execute concurrently

### C3.4 Mapping to this workspace

| Spec Kit | This workspace | Assessment |
|----------|---------------|------------|
| Constitution | `CLAUDE.md` + proposed `AGENTS.md` | Equivalent — different name, same role |
| `/specify` → `spec.md` | Issue body + investigation | **Gap**: no formal spec artifact. Issue descriptions vary in quality; there's no template enforcing goals/constraints/acceptance-criteria before planning begins |
| `/plan` → `plan.md` | `.agent/work-plans/PLAN_ISSUE-N.md` | **Covered**: workspace has a mature plan workflow with templates, draft PRs, and update scripts |
| `/tasks` → `tasks.md` | Tasks section within plan | **Minor gap**: tasks are embedded in the plan rather than a separate trackable artifact. The plan template's "Implementation Tasks" section serves this role but lacks dependency ordering and parallel markers |
| `/analyze` validator | Nothing | **Gap**: no consistency checker across instruction files, plans, and code. The main report's B5 (instruction regression testing) partially addresses this for CLAUDE.md, but not for feature-level specs |
| Per-feature directories | Worktree per issue | Different mechanism, same isolation intent. Worktrees are stronger (full git isolation) but don't persist spec artifacts in a browsable directory |
| Contracts | Nothing formal | **Minor gap**: interface contracts between packages aren't formalized. For ROS 2, this would mean documenting expected topics/services/params as a contract |

### C3.5 Critical evaluation

**Strengths:**
- Agent-agnostic by design — templates work across 12+ tools
- The `/analyze` validator is a novel contribution not seen elsewhere
- Per-feature spec directories create a browsable history of design decisions
- The constitution concept cleanly separates "always true" principles from
  feature-specific requirements

**Weaknesses (from the [Scott Logic review](https://blog.scottlogic.com/2025/11/26/putting-spec-kit-through-its-paces-radical-idea-or-reinvented-waterfall.html)):**
- One specification step took **8 minutes** and produced a 444-line module contract
  that was 4x longer than the eventual implementation code
- Total markdown output across phases: ~1,600 lines of specs, plans, guides, and
  research docs — most described as "obvious and valueless transformations"
- The four-phase gate felt like "reinvented waterfall" — overhead that slowed
  iteration without proportional benefit for moderately complex tasks
- Verdict: "For now, the fastest path is still iterative prompting and review,
  not industrialised specification pipelines"

**Assessment for this workspace**: The full Spec Kit ceremony is overkill for most
workspace tasks (consistent with the report's "radical simplicity" theme from A4).
However, specific components are worth adopting:

1. **Constitution pattern** — already implemented as CLAUDE.md; formalizing a
   shared AGENTS.md (as recommended in §3 and §10) serves the same role
2. **Spec template for complex tasks** — add an optional "Requirements & Acceptance
   Criteria" section to the plan template for issues flagged as architecture-relevant
3. **`/analyze` concept** — a lightweight consistency checker (are referenced paths
   valid? do spec terms match code names?) would catch the drift the report
   identifies throughout §7

---

## C4. Amazon Kiro — Spec-Driven Development in an IDE

[Kiro](https://kiro.dev/) (Amazon, July 2025, GA Dec 2025) embeds spec-driven
development directly into the editor rather than relying on repo-level templates.
Pricing: free (50 interactions/month), $19/month (1,000), $39/month (3,000).

### C4.1 Relevant innovations

**Hooks** — user-defined prompts triggered by file changes. Unlike Claude Code's
lifecycle hooks (PreToolUse/PostToolUse), Kiro hooks are *file-change-triggered*:
"when any file in `src/models/` changes, run this validation prompt." This creates
a reactive enforcement layer that bridges the gap between pre-commit hooks (too
late) and real-time monitoring (too expensive).

**Bidirectional spec sync** — specs stay synchronized with the evolving codebase.
Developers can modify code and ask Kiro to update specs, or modify specs to refresh
tasks. This addresses the universal problem of specs drifting from implementation.

**CLI parity** — the Kiro agent works identically in the terminal and the IDE,
sharing steering files and MCP settings. This matters for CI integration and
headless agent workflows.

### C4.2 Relevance to this workspace

Kiro's file-change hooks validate the report's recommendation in §7f (defense in
depth) from a different angle. The workspace could implement a similar pattern
using Claude Code's PostToolUse hooks: after an Edit to files matching certain
paths (e.g., `package.xml`, `ARCHITECTURE.md`, `CLAUDE.md`), run a validation
check automatically.

Kiro's bidirectional spec sync is aspirational for the workspace — currently,
plans are manually updated. Automating "does the plan still match the code?" would
require the `/analyze`-style consistency checking identified as a gap in C3.4.

---

## C5. Three-Tier Boundary System — Detailed Analysis

The Osmani article cites [GitHub's analysis of 2,500+ agent instruction files](https://addyosmani.com/blog/good-spec/)
as the empirical basis for the three-tier boundary system. This section expands on
how to apply it to this workspace's instruction files.

### C5.1 Current state of CLAUDE.md boundaries

The workspace's CLAUDE.md uses several boundary patterns:

- **Bolded imperatives**: "**Never commit to `main`**"
- **Emphasized warnings**: "IMPORTANT:", "NEVER", "YOU MUST"
- **Implicit always-do**: sourcing `env.sh`, using worktrees, AI signatures
- **Implicit ask-first**: no explicit list; left to agent judgment

These are scattered throughout the document rather than collected in one scannable
section. An agent must read the entire file to build a mental model of what's
allowed, what needs approval, and what's forbidden.

### C5.2 Why three tiers outperform flat rules

From the GitHub analysis and supporting research (AgentIF, NeurIPS 2025 — cited
in main report B5):
- Agents follow ~150-200 instructions consistently; beyond that, compliance drops
- **Categorization aids prioritization** — agents treat "Never" rules as higher
  priority than "Always" rules, which is correct behavior
- **"Ask first" reduces both over-autonomy and under-autonomy** — without an
  explicit middle tier, agents either ask about everything (slow) or assume
  permission for everything (dangerous)
- The three tiers map naturally to enforcement mechanisms: "Always" → agent default
  behavior; "Ask first" → confirmation prompts or hooks; "Never" → hard blocks
  (hooks, CI, branch protection)

### C5.3 Mapping tiers to enforcement layers

Connecting the three-tier model to the enforcement hierarchy from §7g and §9:

| Tier | Instruction | Hook | CI | Container |
|------|-------------|------|----|-----------|
| **Always** | Listed in AGENTS.md | PostToolUse validates | CI checks for presence | N/A |
| **Ask first** | Listed in AGENTS.md | PreToolUse blocks + prompts user | PR review required | N/A |
| **Never** | Listed in AGENTS.md | PreToolUse hard-blocks | CI fails PR | Filesystem prevents |

This mapping shows that the three-tier system isn't just an instruction-level
concern — it's an architectural pattern that should be consistent across all
enforcement layers. A "Never" rule that's only in the instruction file but not
enforced in CI is a soft "Never" that will eventually be violated.

---

## C6. Recommendations

### For the main report's Phase 4 (Align Agent Instructions)

1. **Restructure AGENTS.md using three-tier boundaries** — consolidate the
   scattered rules in CLAUDE.md into Always/Ask-First/Never sections in the
   shared AGENTS.md. Keep CLAUDE.md for Claude-specific behavioral overrides only.
   This is the highest-value, lowest-effort change from this appendix.

2. **Add an optional "Requirements Spec" step for complex issues** — not a
   mandatory gate (avoid the Spec Kit waterfall problem), but a labeled template
   section that agents use when an issue is tagged `architecture-relevant` or
   similar. The spec captures goals, constraints, and acceptance criteria *before*
   the plan captures approach.

### For Appendix B (Technology Landscape)

3. **Add GitHub Spec Kit to B4** (Multi-Agent Coordination & Workspace
   Orchestration) with action: **Evaluate selectively**. The full four-phase
   ceremony is likely overkill, but the `/analyze` validator concept, constitution
   pattern, and per-feature spec directories are worth studying. The Scott Logic
   review's findings about overhead should temper enthusiasm.

4. **Add Amazon Kiro's Hooks to B2** (AI Agent Guardrails & Compliance
   Enforcement) as validation that file-change-triggered enforcement is becoming
   an industry pattern alongside Claude Code's lifecycle hooks.

### For the workspace's CLAUDE.md / AGENTS.md

5. **Adopt the conformance suite mindset** — when writing acceptance criteria in
   specs or plans, express them as verifiable assertions. This feeds naturally into
   the Promptfoo-based instruction regression testing recommended in B5, and into
   standard test suites for feature work.

### Not recommended

6. **Do not adopt Spec Kit wholesale.** The full four-phase gated workflow adds
   ceremony that conflicts with the workspace's "radical simplicity" principle
   (A4 theme 2). The workspace's existing workflow (issue → plan → draft PR →
   implement) is lighter and already covers ~70% of Spec Kit's value. Cherry-pick
   the useful concepts (constitution, `/analyze`, three-tier boundaries) rather
   than importing the framework.

---

## C7. Sources

- [How to Write a Good Spec for AI Agents — O'Reilly Radar (Addy Osmani)](https://www.oreilly.com/radar/how-to-write-a-good-spec-for-ai-agents/)
- [How to write a good spec for AI agents — addyosmani.com](https://addyosmani.com/blog/good-spec/)
- [How to write a good spec for AI agents — Substack](https://addyo.substack.com/p/how-to-write-a-good-spec-for-ai-agents)
- [GitHub Spec Kit — github/spec-kit](https://github.com/github/spec-kit)
- [Spec Kit — speckit.org](https://speckit.org/)
- [Diving Into SDD With GitHub Spec Kit — Microsoft Developer Blog](https://developer.microsoft.com/blog/spec-driven-development-spec-kit)
- [Putting Spec Kit Through Its Paces — Scott Logic](https://blog.scottlogic.com/2025/11/26/putting-spec-kit-through-its-paces-radical-idea-or-reinvented-waterfall.html)
- [GitHub Spec Kit Experiment: 'A Lot of Questions' — Visual Studio Magazine](https://visualstudiomagazine.com/articles/2025/09/16/github-spec-kit-experiment-a-lot-of-questions.aspx)
- [A Look at Spec Kit — Tessl](https://tessl.io/blog/a-look-at-spec-kit-githubs-spec-driven-software-development-toolkit/)
- [Kiro: Agentic AI development — kiro.dev](https://kiro.dev/)
- [Beyond Vibe Coding: Amazon Introduces Kiro — InfoQ](https://www.infoq.com/news/2025/08/aws-kiro-spec-driven-agent/)
- [Spec-Driven AI Coding With GitHub's Spec Kit — InfoWorld](https://www.infoworld.com/article/4062524/spec-driven-ai-coding-with-githubs-spec-kit.html)
- [GitHub Spec Kit: A Guide to Spec-Driven AI Development — IntuitionLabs](https://intuitionlabs.ai/articles/spec-driven-development-spec-kit)
- [Chapter 5: Spec-Driven Development with Claude Code — Agent Factory](https://agentfactory.panaversity.org/docs/General-Agents-Foundations/spec-driven-development)

---

**Authored-By**: `Claude Code Agent`
**Model**: `Claude Opus 4.6`
