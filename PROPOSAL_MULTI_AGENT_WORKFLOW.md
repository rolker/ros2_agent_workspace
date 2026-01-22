# Proposal: Multi-Agent Workflow System (RFC)

**Status**: Draft / Request for Comments
**Target**: Project11 / ROS 2 Agent Workspace

## 1. Executive Summary
We propose transforming the workspace into a "Multi-Agent Development Environment" that supports:
1.  **Parallel Work**: Multiple agents working on different tasks simultaneously.
2.  **Phased Execution**: Explicit Planning, Implementation, and Review phases.
3.  **Cross-Agent Collaboration**: Robust review loops between different AI models/frameworks.

## 2. Memory Architecture: "Source of Truth" vs. "Working Memory"

### The Problem
Agents are ephemeral. They forget context when a session ends. We need a persistent state that works across different environments (VS Code, CLI, GitHub Codespaces).

### The Solution
*   **Layer 1: The Hard Drive (GitHub Issues)**
    *   **Role**: The absolute Source of Truth.
    *   **Content**: High-level goals, requirements, decision records, and final status.
    *   **Access**: All agents (Copilot, Gemini, etc.) have API access to read/write Issues.
*   **Layer 2: The RAM (Local `task.md`)**
    *   **Role**: Immediate, low-latency working memory.
    *   **Content**: The granular checkbox list for the *current* session (e.g., "[ ] Fix typo in line 40").
    *   **Lifecycle**:
        1. Agent picks up Issue #123.
        2. Agent creates/updates `.worktrees/issue-123/task.md` based on Issue description.
        3. Agent checks off items as it executes tool calls.
        4. Before session end, Agent summarizes `task.md` progress into a GitHub Comment (saving state to Layer 1).
    *   **IDE Integration**: In VS Code/Antigravity, `task.md` is pinned open. In CLI, it's just a text file.

## 3. Parallelism: The "Isolation" Strategy

### The Requirement
"Having agents working in parallel is important." (User Feedback)

### The Challenge
ROS 2 workspaces use `colcon` to build layers. If Agent A and Agent B share the same `build/` directory, they will corrupt each other's compiles.

### The Proposed Solution: `git worktree` + Docker Sandboxing
> "Can docker containers be used to accomplish something similar? What about a sandboxing strategy?"

We can combine **Worktrees** (Source Isolation) with **Docker** (Runtime Isolation) for the ultimate "Antigravity Chamber".

1.  **Source Layer (Git)**:
    *   Agents create specific worktrees: `.worktrees/task-123`.
2.  **Runtime Layer (Docker)**:
    *   Instead of running `colcon build` on the host, the Agent spins up a **Disposable Container**.
    *   **Mounts**:
        *   Read-Only: `~/workspace/workspaces/core_ws` (The Underlay).
        *   Read-Write: `.worktrees/task-123` (The active task).
    *   **Execution**: The agent runs build/test commands *inside* this container.
3.  **Sandboxing Benefits**:
    *   **Safety**: An agent cannot accidentally `rm -rf /` the host.
    *   **Cleanliness**: No polling of the host's `.bashrc` or weird python path issues.
    *   **Consistency**: Every task starts from a known-good "Golden Image" of the OS.

## 4. Ecosystem: The Cross-Agent Review Loop

### The Requirement
"Have different agents/models/frameworks check each other's work."

### The Workflow
1.  **Submission**: Agent A (Implementer) pushes `feature/task-123` and opens a PR.
2.  **Trigger**:
    *   **Manual**: User asks Agent B (Reviewer) to "Check PR #123".
    *   **Automated**: GitHub Action triggers a "Review Agent" (via webhook or scheduled job).
3.  **Review Execution**:
    *   Agent B reads the *diff* (using `mcp_github` or `git diff main...feature/task-123`).
    *   Agent B validates against `CONTRIBUTING.md` and `AGENTS.md` rules.
4.  **Feedback Delivery**:
    *   Agent B uses `mcp_github` to post **Inline Comments** on the PR.
    *   *Why?* Because this is the universal "Review Interface" that users and other agents already understand.
5.  **The Response Loop**:
    *   Agent A wakes up, reads the PR comments.
    *   Agent A updates its local `task.md`:
        *   `[ ] Address comment on line 42 (Memory leak)`
    *   Agent A fixes code, commits, and resolves the conversation.

## 5. Cleanup & Simplification: Full Workspace Audit

> "Can we do the same for other files and directories in the workspace?"

We will expand the audit to the **Entire Workspace**, categorizing every file/folder into:
1.  **Core**: Essential for ROS 2 (e.g., `src/`, `package.xml`).
2.  **Agent Logic**: Essential for AI (e.g., `.agent/scripts/setup.sh`).
3.  **Legacy/Noise**: To be deleted or archived.

*Example Audit Plan:*
*   `configs/` -> Are all `.repos` files still used?
*   `ag_work/` (if exists) -> Temp files to clean?
*   `.agent/rules/` -> Are these rules actually followed?

## 7. Gaps & Technical Challenges (To Be Solved)

### 7.1. The Meta-Learning Loop (Retrospectives)
*   **Gap**: Agents fixing code is good, but agents fixing *rules* is better.
*   **Solution**: A **"Retrospective" Workflow** runs when a Task is marked `Done`.
    *   It analyzes PR comments: "Did we violate a rule?"
    *   It proposes updates to `CONTRIBUTING.md` or `.agent/rules/` to prevent recurrence.

### 7.2. Secret Management
*   **Gap**: Docker containers need git access.
*   **Solution**: Use `ssh-agent` forwarding or strict Volume Mounts for keys. *Never* write secrets to the container filesystem.

### 7.3. Resource Governance (The "OOM" Killer)
*   **Gap**: 3 concurrent `colcon build` jobs will freeze the machine.
*   **Solution**: A **"Resource Sentinel"** (simple lock file) ensures only 1 heavy build runs at a time, queueing others.

### 7.4. Human Debuggability
*   **Gap**: How to fix a broken agent container?
*   **Solution**: Standardize a `make attach-task-123` command to drop a human shell instantly into the agent's environment.

### 7.5. Dependency Drift
*   **Gap**: Agents `pip install` packages in Docker but forget `package.xml`.
*   **Solution**: A CI check (or Pre-Commit hook) that fails if imported modules are missing from `package.xml`.

## 8. Adaptive Quality Assurance Strategy

> "Let's keep it flexible... depending on the task, the development method will vary."

Instead of enforcing a rigid "One Size Fits All" TDD policy, we propose an **Adaptive Strategy**. The *Reviewer Agent* will determine the appropriate testing method based on the Task Type.

### 8.1. The Menu of Methodologies

#### Option A: Test-Driven Development (TDD)
*   **Best For**: Pure logic libraries, algorithms, utilities (e.g., `calc_distance()`, `path_smoother`).
*   **Workflow**: Red-Green-Refactor (Unit Tests).

#### Option B: Simulation-First Verification
*   **Best For**: Robotics behaviors, Navigation, Control loops.
*   **Workflow**:
    1.  Define the Scenario (e.g., "USV must drive through the gate").
    2.  Write the Code.
    3.  Verify success in Gazebo/Ignition.

#### Option C: Contract-Driven Development
*   **Best For**: Node integration, large system architecture.
*   **Workflow**:
    1.  Define the ROS 2 Interface (`.msg`, `.action`).
    2.  Implement the Node.
    3.  Verify purely that the node "Speaks the right language" (Topics/Types match).

### 8.2. Enforcement
The **Reviewer Agent** does not enforce "Unit Tests" blindly. Instead, it asks: **"Did you verify this work in a way that matches its complexity?"**
*   If you wrote a Math library, show me `gtest`.
*   If you wrote a Wall Follower, show me a Rosbag or Sim Screenshot.

## 9. Next Steps
1.  **Post as Issue**: We will post this entire text as a GitHub Issue on `ros2_agent_workspace`.
2.  **Community Review**: We (Agents & Humans) will discuss in the Issue comments.
3.  **Pilot**: Once the "Docker vs. Local" debate is settled in the Issue, we start Phase 2.
