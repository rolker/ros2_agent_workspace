# Guiding Principles for the ROS 2 Agent Workspace

**Status**: Draft
**Target Audience**: Agent Optimizer (Meta-Agent), Human Architects, Workspace Administrators

## 1. Executive Summary

This document establishes the "North Star" metrics and principles for this workspace. It serves as the primary instruction set for the **Agent Optimizer** skill. When auditing, refining, or synthesizing new capabilities, all decisions must align with these five pillars.

---

## 2. The Five Pillars

### I. High-Quality, Robust ROS 2 Artifacts
**"If it isn't verified, it doesn't exist."**

*   **The Standard**: All packages must pass `ament_lint`, `gtest` (if applicable), and build without warnings.
*   **Verification**: No code is "done" until verified by `./.agent/scripts/verify_change.sh` or `./.agent/scripts/test.sh`.
*   **Documentation**: Code must be self-documenting (standard ROS 2 Doxygen/Sphinx) *and* have clear high-level `README.md` files.
*   **Constraint**: Never lower logical quality to satisfy valid syntax. A compiling node that crashes is worse than a compile error.

### II. Continuous Self-Improvement
**"The workspace fixes itself."**

*   **Retrospectives**: Failures in CI or workflow must trigger a "Why?" analysis. If a rule was broken, the rule (or its enforcement) must be refined.
*   **Skill Evolution**: Instructions in `SKILL.md` are living documents. If a prompt consistently confuses agents, rewrite the prompt.
*   **Gap Filling**: If a user frequently asks for X, and X is hard, create a workflow for X.

### III. Multi-Agent Scalability
**"Many hands, one mind, zero collisions."**

*   **Identity**: Every commit and comment must be attributable to a specific agent identity (e.g., "Antigravity Agent").
*   **Isolation**: Work is done in `feature/` branches or isolated `worktrees`. The default branch (main/jazzy/rolling) is sacred and read-only.
*   **Handover**: The "Clean Campsite" rule. Agents must leave the workspace in a known, clean state (`git status` clean) when their session ends.
*   **Coordination**: Use Draft PRs and `work-plans/` to signal intent before writing code.

### IV. Radical Simplicity & Transparency
**"Keep it lean. Keep it clear."**

*   **The Lean Principle**: If a feature, script, or workflow isn't actively providing value, delete it. Minimize complexity, dependencies, and lines of code.
*   **Pragmatism (YAGNI)**: Do not over-engineer for hypothetical futures. Solve the immediate problem with the simplest robust solution.
*   **Visibility**: The user must be able to glance at the workspace (Git status, GitHub Issues) and know *exactly* what is happening.
*   **No Magic Boxes**: Scripts must be transparent. Prefer standard CLI tools and simple bash scripts over complex, opaque wrappers.
*   **Explainability**: If a human cannot understand the script in 30 seconds, it is too complex. Refactor or simplify.

### V. Industry Alignment
**"Don't reinvent the wheel; just make it roll smoother."**

*   **Standards**: Use standard ROS 2 tools (`colcon`, `colcon test`/`ctest`, `launch_testing`, `ros2 launch`) rather than custom wrappers that obscure the underlying mechanics.
*   **Conventions**: Follow REP-standards (e.g., REP-2004 for package layout).
*   **Ecosystem**: Embrace standard Linux/Git workflows. If a tool exists (e.g., `gh` CLI), use it instead of writing a custom API client.

---

## 3. Operational Directives for Agent Optimizer

When you are invoked to "Optimize the Agent":

1.  **Audit vs. Principle**:
    *   *Check*: "Is `package.xml` complete?" -> **Pillar I (Quality)**.
    *   *Check*: "Are agents stepping on each other?" -> **Pillar III (Multi-Agent)**.
    *   *Check*: "Is the documentation confusing?" -> **Pillar IV (Transparency)**.

2.  **Refactoring Logic**:
    *   If a script is "magical" and hidden, expose it or replace it with a standard tool (**Pillar V**).
    *   If an agent task fails often, create a `SKILL.md` with better prompts (**Pillar II**).

3.  **The "Bus Factor" Test**:
    *   If the AI disappears tomorrow, can the human user still build, run, and understand the code? If no, **Pillar IV** is violated. Fix it immediately.

4.  **Simplicity Check**:
    *   Before adding a new workflow or script, ask: "Can this be done with an existing tool?"
    *   If a file hasn't been touched in 6 months and isn't core infrastructure, propose deletion.

---

## 4. Implementation Metrics

*   **Build Success Rate**: % of `colcon build` runs that pass on the first try.
*   **Issue Cycle Time**: Time from "Taking this" to "Merged".
*   **Handover friction**: Number of times an agent has to clean up another agent's mess.
*   **Documentation Coverage**: % of public API methods with docstrings.
