---
marp: true
theme: ccom
paginate: true
header: ''
footer: 'CCOM/JHC -- AI in Our Work'
---

<!-- _class: title -->
<!-- _paginate: false -->
<!-- _footer: '' -->

# AI in Our Work

## From Autocomplete to Autonomous Agents

CCOM/JHC AI Working Group Discussion
March 2026

---

# Before Agents

- About a year of **GitHub Copilot** in VS Code -- AI-enhanced autocomplete
- Free Gemini Pro subscription came with my phone -- tried **vibe coding** a web-based mission planner
- The agent had no idea what direction I wanted to take it
- Made poor architectural choices that were hard to improve
- Quick to start, but a dead end

---

<!-- _class: callout -->

## The Key Insight

> "You can't prompt-engineer your way out of vibe-coding AI slop.
> You need to craft a context that allows agents to meet your expectations."

Everything that follows is about *how* to craft that context.

---

# Building the Context

**January 9, 2026** -- created `ros_agent_workspace`

- Wanted to test agents on **real code**, not throwaway projects
- Started with Gemini CLI and Copilot CLI -- asked an agent how to support multiple frameworks, and it helped set up the workspace
- Also tried Google Antigravity during those early days

**First real test**: renaming project11 to unh_marine_autonomy, consolidating repos

- Tedious work, not hard thinking -- agents did it in minutes
- But the simulator didn't work afterward
- Led to building up tests: unit, then integration, then UI for agent-testable stack
- **Took longer than doing it manually** -- but got test coverage and documentation that wouldn't have existed otherwise

---

# Scaling Up

- **Claude Code** became framework of choice for its coding ability
- Goal from the start: **multiple agents working simultaneously**
- Asked an agent how to avoid conflicts -- learned about **git worktrees**
- Problems would get solved, then creep back -- **governance** emerged (ADRs, guiding principles)

**The Great Simplification**: agents generated so much structure (workflows, rules, skills) that the workspace became bloated -- had to delete **51 files** in one pruning session

Power tools produce sawdust too.

---

<!-- _class: callout -->

## Raising Agents = Parenting + Mentoring

**Toddler moments** -- doing something unexpected because you didn't say not to, but any reasonable adult would know better

*Had to programmatically block `git checkout` because agents kept bypassing worktrees*

**Intern moments** -- passable code with flaws that experience would have caught

*Security vulnerabilities (XSS, path traversal, injection) that needed multiple rounds of review*

You parent the toddler behavior with guardrails.
You mentor the intern behavior with code review.

---

# 2025: Writing Code by Hand

- ~**39,000** lines of code across project repos
- ~3,200 lines per month
- Just me, writing code

---

# 2026: Zero Lines of Code

- Lines of code written by me: **0**
- I have not written a single line of code this year

---

# 2026: What the Agents Wrote

- **~148,000 lines** in 11 weeks
- ~**4x** the annual output -- in under 3 months
- ~20 commits/month in 2025 --> ~354 commits/month in 2026 = **17x**
- **206** issues, **206** pull requests

<!-- The wood carver picked up power tools. -->

---

<!-- _class: callout -->

## The Woodworker Analogy

A skilled **wood carver** produces beautiful, intricate work by hand.

Gain access to **table saws, routers, scroll saws** -- and suddenly you can produce a lot more. But the products are not as intricate or elegant. They're *functional*.

It still takes someone who **understands wood** to get the most out of power tools. And there is a learning curve.

---

# What Agents Enabled

Things that **wouldn't have happened** without agent help:

- **ENC to Gazebo world builder** -- converts nautical charts to simulation worlds with terrain, piers, trees, waves. Too time-consuming to tackle alone. *(demo later)*
- **GeoZui4D revival** -- resurrecting a legacy geospatial tool to extract missing functionality
- **Test coverage and documentation** that always got skipped as a solo developer
- **Research and inspiration tracker** -- agents survey the rapidly evolving landscape so I can keep up

Infrastructure vs. project work: ~63% / 37% -- but the infrastructure *is* the multiplier.

---

# Agents as External Memory

The biggest workflow change: **I don't have to hold everything in my head**

- An idea strikes -- agent does a quick assessment -- opens a detailed issue
- That issue actually gets **reconsidered at triage** (not buried in a notebook)
- I can oversee **multiple threads** because the state is externalized in issues and PRs
- From **coder** to **manager**: I don't miss writing code -- it's the results that excite me, not the typing

The downside: GitHub-centered workflow adds latency. Evolving toward local-first with git-bug and local reviews.

---

# Beyond Code -- BizzyBoat

**This week**: pivoted from coding to setting up a new EchoBoat

- IzzyBoat's network had grown organically -- docs incomplete or outdated
- **Claude Code + tmux** used to:
  - Document IzzyBoat's current network state
  - Plan and implement BizzyBoat's network
  - Update firmware on both boats
  - Simplify VPN, test simultaneous operation and inter-robot communication

**~2 weeks of work done in 2 days**

Agents aren't just for writing code.

---

# What's Next

- **Containerized agents** -- letting them run more freely once I'm satisfied I can verify their output
- **Reducing permission prompts** -- the friction is real, but I'm not ready to let agents loose
- **Local-first workflow** -- git-bug for offline issues, better local reviews to cut GitHub latency
- **Research + inspiration tracker** -- the space is evolving so fast that keeping up is itself a task for agents

---

<!-- _class: callout -->

## One Last Thing

I sometimes struggle with communicating effectively.

Having an agent help me refine how I get my thoughts across has been genuinely useful -- not just for code, but for writing, planning, and yes, presentations.

I'll let you guess who helped me put this one together.

---

<!-- _class: title -->
<!-- _paginate: false -->
<!-- _footer: '' -->

# Discussion and Demos

Gazebo worlds generated from nautical charts

Live Claude Code *(if time permits)*

Your questions, ideas, and concerns
