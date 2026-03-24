---
marp: true
theme: ccom
paginate: true
header: ''
footer: 'CCOM/JHC -- Raising Agents'
---

<!-- _class: title -->
<!-- _paginate: false -->
<!-- _footer: '' -->

# Raising Agents

## From Autocomplete to Autonomous Agents

CCOM/JHC AI Working Group Discussion
March 2026

---

# Two Aha Moments

**Aha #1: AI Autocomplete** -- GitHub Copilot in VS Code for about a year

- A genuine productivity boost -- predicts what you're about to type
- Small step in retrospect, but it changed how coding *felt*

**Aha #2: Vibe Coding** -- free Gemini Pro subscription came with my phone

- Built a web-based mission planner through conversation alone
- Got to **90% effortlessly** -- a genuine wow moment
- But the last 10% was a wall: poor architectural choices, hard to improve
- The agent had no idea what direction I wanted to take it

Both moments made me think: *what if I go further?*

---

<!-- _class: callout -->

## The Key Insight

> "You can't prompt-engineer your way out of vibe-coding AI slop.
> You need to craft a context that allows agents to meet your expectations."

Everything that follows is about *how* to craft that context.

*(This is a genuine quote, not AI-generated!)*

---

# Building the Context

**January 9, 2026** -- created `ros2_agent_workspace` ([github.com/rolker/ros2_agent_workspace](https://github.com/rolker/ros2_agent_workspace))

- Wanted to test agents on **real code**, not throwaway projects
- Started with Gemini CLI, Copilot CLI, and Google Antigravity
- Asked an agent how to support multiple frameworks -- it helped set up the workspace

**First real test**: renaming project11 to unh_marine_autonomy, consolidating repos

- Tedious work, not hard thinking -- agents did it in minutes
- Simulator broke -- expected, it would have broken if I did it manually too
- But **I test by looking at the UI** and quickly know something's not right. Agents can't see the simulator, can't tell if things "look right"
- Real cost: **building tools that give agents the eyes I already have**
- Slowed this task down, but makes every future task faster

---

# How the Workspace Works

A **workspace repo** wraps multiple **project repos** with agent infrastructure:

- **Rules and guardrails** -- instruction files tell agents what to do and what not to do
- **Scripts** -- worktree management, status reports, issue creation, identity tracking
- **Skills** -- reusable prompts for common workflows (research, code review, testing)
- **Governance** -- ADRs and guiding principles that persist lessons learned

The project itself is layered: workspace contains **layers**, layers contain **repos**, repos contain **ROS 2 packages**. This makes things like worktree isolation harder than a single-repo project, but it reflects the real complexity of the system.

---

# Workspace Structure

```
ros2_agent_workspace/
├── AGENTS.md, CLAUDE.md          # Agent instruction files
├── Makefile                       # Build, test, lint, dashboard
├── .agent/
│   ├── scripts/                   # Worktree, identity, build, status
│   ├── knowledge/                 # ROS 2 patterns, review guides
│   ├── templates/                 # Issue, test, documentation templates
│   └── work-plans/                # Saved plans for active tasks
├── docs/decisions/                # Architecture Decision Records
├── configs/                       # Workspace manifest and layer config
└── layers/main/
    ├── underlay_ws/src/           # Dependencies (vrx, geographic_info, ...)
    ├── core_ws/src/               # Core repos (unh_marine_autonomy, ...)
    ├── simulation_ws/src/         # Simulation (unh_marine_simulation, ...)
    ├── sensors_ws/src/            # Sensors (cube_bathymetry, ...)
    ├── platforms_ws/src/          # Platforms (ben_description, ...)
    └── ui_ws/src/                 # UI (camp, rviz plugins, ...)
```

---

# Scaling Up

- A few weeks in, tried **Claude Code** -- better results, more comfortable running multiple agents in parallel
- Asked an agent how to avoid conflicts -- learned about **git worktrees** (isolated copies of the repo so agents don't step on each other's work on the main branch)
- Agents teach me too -- they know obscure command options I'd never look up

---

# The Great Simplification

**The problem**: each friction point got its own issue, each fix was isolated -- and fixes would break other fixes because nothing connected them

**The result**: a bloated, messy, disorganized workspace -- **51 files** of overlapping scripts, rules, and workflows

**The solution**:
- Agents scanned all scripts, extracted intents, consolidated into fewer scripts
- Established **ADRs and guiding principles** as the source of truth
- A **review process** that checks adherence prevents it from happening again

Power tools produce sawdust too.

---

<!-- _class: callout -->

## Raising Agents = Parenting + Mentoring

**Toddler moments** -- doing something unexpected because you didn't say not to, but any reasonable adult would know better

*Added a guardrail that blocks `git checkout` -- not to prevent it outright, but to nudge agents to re-read the rules and use a worktree instead. Google Antigravity didn't even try to branch -- it just edited files directly on main, then thought it was being clever when it hit the guardrail.*

**Intern moments** -- passable code with flaws that experience would have caught

*Hard-coding my machine's absolute path instead of using a relative one. Lacking the big-picture awareness that comes from years of working with the codebase.*

You parent the toddler behavior with guardrails.
You mentor the intern behavior with code review.

---

<!-- _class: divider -->

# So, Did It Work?

Measuring the impact.

---

# Measuring Productivity

In 2025, I wrote ~**39,000** lines of code.

---

# Measuring Productivity

In 2025, I wrote ~**39,000** lines of code.

In 2026, I wrote ~**0** lines of code.

---

# Measuring Productivity

In 2025, I wrote ~**39,000** lines of code.

In 2026, I wrote ~**0** lines of code.

In 2026, agents wrote ~**148,000** lines of code in 11 weeks.

- ~**4x** the annual output in under 3 months
- ~20 commits/month (2025) --> ~354 commits/month (2026) = **17x**
- **206** issues, **206** pull requests

---

<!-- _class: callout -->

## The Woodworker Analogy

A skilled **wood carver** produces beautiful, intricate work by hand.

Gain access to **table saws, routers, scroll saws** -- and suddenly you can produce a lot more. But the products are not as intricate or elegant. They're *functional*.

It still takes someone who **understands wood** to get the most out of power tools. And there is a learning curve.

---

# Growing Bandwidth

Early on: **one agent** working on the project, watching closely, maybe a second fixing a workspace bug

As the workspace improved, guardrails handled more of the supervision:

- One or two agents on **project work**
- One or two agents on **workspace improvements**
- Enough bandwidth left over for a **sidequest**

Things that **wouldn't have happened** without this bandwidth:

- **ENC to Gazebo world builder** -- converts nautical charts to simulation worlds with terrain, piers, trees, waves. Too time-consuming to tackle alone *(demo later)*
- **GeoZui4D revival** -- resurrecting a legacy geospatial tool to extract missing functionality
- **Test coverage and documentation** that always got skipped as a solo developer

---

# Agents as External Memory

The biggest workflow change: **I don't have to hold everything in my head**

- An idea strikes -- agent does a quick assessment -- opens a detailed issue
- That issue actually gets **reconsidered at triage** (not buried in a notebook)
- I can oversee **multiple threads** because the state is externalized in issues and PRs
- From **coder** to **manager**: I don't miss writing code -- it's the results that excite me, not the typing

The organization is still evolving -- dogfooding *(testing your own tools on yourself)* the brainstorm and research skills generated dozens of issues that needed their own triage. Now shifting toward roadmap documents that gather findings before opening targeted issues.

The workspace is a work in progress.

---

# Beyond Code -- BizzyBoat (The Problem)

**This week**: pivoted from coding to setting up a new EchoBoat

The network is not a simple wifi link:
- **Shore side**: operator laptop --> Teltonika router --> internet + wifi bridge
- **Boat side**: Teltonika router --> Starlink + cell modem backup + wifi bridge
- **Cloud**: VPN server for beyond-range connectivity
- **Five devices**, all configured via SSH command line

IzzyBoat's network had grown organically -- documentation was incomplete or outdated. Setting up BizzyBoat to match meant first figuring out what IzzyBoat actually looked like.

---

# Beyond Code -- BizzyBoat (The Solution)

**Claude Code + tmux** -- the agent surveyed all devices via CLI, parsed the output into organized reference docs

For setup, I did most steps manually in the web GUIs -- I was comfortable with that. The agent's role shifted to:
- **Reminding me** of values like IP addresses and config details
- **Checking my work** after each step via CLI commands
- **Helping me navigate** unfamiliar UIs more quickly
- **Documenting everything** as we went

**~2 weeks of work done in 2 days.** Agents aren't just for writing code.

Friction: every new agent session needed re-explanation of the rules. *(Live demo: let's open an issue about that...)*

---

# With That Extra Bandwidth...

A few more irons in the fire:

- **More autonomous agents** -- containerized, fewer permission prompts
- **Tighter loop** -- local-first workflow (git-bug, local reviews) to speed up review --> fix --> merge

---

# Keeping Up With a Fast-Moving Space

- **Research skill**: survey what's out there -- can it replace what we built, or can we borrow good ideas?
- **Inspiration tracker**: born from creating `agent_workspace` ([github.com/rolker/agent_workspace](https://github.com/rolker/agent_workspace)) -- a non-ROS derivative for simpler projects (including a video game for my daughter). Needed a way to track changes in repos we've borrowed from, complementing research which finds *new* sources

We improve by solving our own problems *and* keeping an eye on what others are doing.

---

<!-- _class: callout -->

## One Last Thing

I sometimes struggle with communicating effectively.

Having an agent help me refine how I get my thoughts across has been genuinely useful -- not just for code, but for writing, planning, and yes, presentations.

I'll let you guess who helped me put this one together.

<br><br>

*Authored-By: Claude Code Agent | Model: Claude Opus 4.6*

---

<!-- _class: title -->
<!-- _paginate: false -->
<!-- _footer: '' -->

# Discussion and Demos

Gazebo worlds generated from nautical charts

Live Claude Code *(if time permits)*

Your questions, ideas, and concerns
