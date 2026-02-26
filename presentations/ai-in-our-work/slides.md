---
marp: true
theme: ccom
paginate: true
header: ''
footer: 'CCOM/JHC — AI in Our Work'
---

<!-- _class: title -->
<!-- _paginate: false -->
<!-- _footer: '' -->

# AI in Our Work

## A Practical Look at AI Tools for Technical Teams

CCOM/JHC Working Group Discussion
March 2026

---

# What We'll Cover

1. **The AI tool spectrum** — from autocomplete to autonomous agents
2. **A real case study** — how we've been using an AI agent for ROS 2 development
3. **Where to start** — practical entry points for your own work
4. **What to watch out for** — honest limitations and risks
5. **Open discussion** — your questions, ideas, and concerns

<!-- This is a conversation, not a lecture. Jump in at any time. -->

---

<!-- _class: divider -->

# The Tool Spectrum

Not all AI tools are the same.

---

# From Autocomplete to Agents

| Level | What It Does | Examples |
|-------|-------------|----------|
| **Autocomplete** | Predicts the next few tokens as you type | GitHub Copilot inline suggestions |
| **Chat assistant** | Answers questions, generates snippets on request | ChatGPT, Claude.ai |
| **Inline copilot** | Understands your codebase context, suggests edits | Copilot Chat, Cursor |
| **Autonomous agent** | Plans and executes multi-step tasks independently | Claude Code, Copilot Workspace |

Most of us have used levels 1 and 2. The interesting space right now is levels 3 and 4.

---

# Key Insight

The tools differ in **autonomy** and **context**:

- **Autocomplete**: sees the current file
- **Chat**: sees what you paste in
- **Copilot**: sees your project
- **Agent**: sees your project, uses tools, runs commands, creates PRs

More context and more autonomy = more useful, but also more risk.

---

<!-- _class: divider -->

# Case Study

An AI agent doing real software engineering in our ROS 2 workspace.

---

# What We Set Up

An AI coding agent (Claude Code) working in a ROS 2 workspace with:

- **Guardrails**: cannot push to `main`, cannot checkout branches directly
- **Isolation**: every task runs in a separate git worktree
- **Issue-first policy**: no code without a ticket
- **PR review**: all changes go through pull requests, same as a human contributor
- **Identity tracking**: every commit and PR is signed by the agent

The infrastructure took deliberate effort to design.

---

# What the Agent Has Done

Real tasks completed through this workflow:

- Setting up workspace build infrastructure and CI
- Writing and updating documentation (verified against source code)
- Creating development tooling (worktree scripts, status reports)
- Refactoring configuration files
- Researching and analyzing technical specifications
- Responding to code review feedback and iterating

Each task: issue → branch → commits → PR → review → merge.

---

# What Worked Well

- **Boilerplate and scaffolding** — ROS 2 package setup, launch files, configs
- **Documentation** — generating docs from actual source code, not guesses
- **Refactoring** — renaming, restructuring, modernizing existing code
- **Explaining code** — "what does this node actually do?"
- **Iterating on review feedback** — surprisingly good at addressing PR comments

---

# What Didn't Work Well

- **Domain-specific knowledge** — AI has general knowledge but not depth in hydrographic survey, sonar processing, or specialized marine systems
- **Complex architecture decisions** — still needs heavy human guidance
- **Build system subtleties** — colcon/ament/CMake interactions trip it up
- **Non-determinism** — same prompt can produce different results each time
- **Overconfidence** — will confidently document a parameter that doesn't exist

---

<!-- _class: callout -->

## The Honest Summary

The overhead of setting up structure and guardrails is the real work.

Once that's in place, the AI coding part is the easy part — but **review is non-negotiable**. Treat AI output like you'd treat code from a new team member: verify everything.

---

<!-- _class: divider -->

# Where to Start

Practical entry points, ordered by effort.

---

# Low Friction — Start Here

Tasks you already know how to do, but find tedious:

- **Writing tests** for existing code — describe the function, get test cases
- **Documentation** — generate READMEs, docstrings, parameter descriptions from source
- **Boilerplate** — launch files, package.xml, CMakeLists.txt scaffolding
- **Code review assistance** — "explain this PR" or "what could go wrong here?"
- **Reformatting and cleanup** — style compliance, linting fixes

These are safe because **you can easily verify the output**.

---

# Medium Effort — Explore and Learn

Use AI to accelerate understanding:

- **"Explain this codebase to me"** — point it at an unfamiliar package
- **"What does this parameter do?"** — faster than tracing through source
- **API exploration** — "show me how to use the tf2 buffer in ROS 2"
- **Drafting implementations** — get a starting point, then refine
- **Converting between frameworks** — ROS 1 → ROS 2 migration patterns

Useful but requires more judgment to evaluate the output.

---

# Higher Investment — Agentic Workflows

Autonomous agents that plan and execute tasks:

- Requires upfront structure (branching policy, review process, guardrails)
- Best for **repetitive, well-defined tasks** across a codebase
- The payoff comes from volume — one task isn't worth the setup cost
- Think: "apply this pattern to 15 packages" or "update every launch file"

This is where our ROS 2 workspace experiment lives.

---

<!-- _class: divider -->

# What to Watch Out For

An honest assessment.

---

# Known Limitations

- **Confidently wrong** — AI doesn't signal uncertainty well. It will invent plausible-sounding parameter names, topic names, or API calls that don't exist.

- **Domain expertise gap** — general programming is strong; hydrographic processing, MBES data, survey standards — not so much.

- **The "vibe coding" trap** — accepting code you don't fully understand is technical debt with interest.

- **Security and IP** — understand what leaves your machine. Cloud-based tools send your code to external servers. Local models exist but are less capable.

---

# Practical Advice

- **Verify everything** — especially parameter names, topic names, and message types
- **Start with tasks you can judge** — don't ask AI to do something if you can't tell whether the result is correct
- **Keep the human in the loop** — code review isn't optional
- **Read the code** — if you can't explain what the AI wrote, rewrite it
- **Version control everything** — if the AI breaks something, you need to roll back

---

<!-- _class: divider -->

# Discussion

---

# Questions to Kick Things Off

- **What's a task you do regularly that feels like it should be automatable?**

- **What's your biggest concern about using AI in our work?**

- **Has anyone tried a particular AI tool? What was your experience?**

- **Where do you see the biggest opportunity for our group?**

---

<!-- _class: title -->
<!-- _paginate: false -->
<!-- _footer: '' -->

# Thank You

Questions, ideas, and concerns are welcome — now or anytime.

Slides and source available in the workspace repository.
