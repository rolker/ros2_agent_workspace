# AI Identity Strategy

## Problem Statement
Currently, AI agents operate using the user's personal GitHub credentials (PAT). This causes two issues:
1.  **Identity Confusion**: Commits and PRs appear as if they were created by the user (`rolker`), making it difficult to distinguish between human-generated and agent-generated code in the history.
2.  **Review Blocking**: GitHub prevents users from approving their own Pull Requests, making it difficult to identify issues and PRs submitted by AI agents. Since the agent-created PR is owned by `rolker`, `rolker` cannot approve it, hindering the desired peer-review workflow.

## Agent Identity Configuration

**Important**: Each AI agent must identify itself appropriately in git commits. Do not use the repository owner's identity for agent-generated commits.

**How to determine your identity:**
1. **Identify yourself** based on your actual agent platform/name
2. **Ask the user** if you're uncertain about the appropriate name/email format
3. **Configure git** with your identity before making any commits

**Example identities:**
- Antigravity Agent: `Antigravity Agent` / `roland+antigravity@ccom.unh.edu`
- GitHub Copilot CLI: `Copilot CLI Agent` / `roland+copilot-cli@ccom.unh.edu`
- Other agents: Follow the pattern `<Platform> Agent` / `roland+<platform>@ccom.unh.edu`

**Scope**: Identity must be configured in:
- The workspace repository (`ros2_agent_workspace/.git/config`)
- **All repositories under `workspaces/*/src/*/.git/config`**

## Strategic Options

### 1. Git Authorship Distinction (Current Implementation)
We configure each agent to use a distinct name and email for git commits.
*   **Implementation**:
    *   Determine appropriate identity for your agent (ask user if needed)
    *   Apply to workspace repo AND all repos in `workspaces/` using the configuration script
*   **Benefits**:
    *   Git history clearly distinguishes agent commits from human commits.
    *   Blame view shows the agent name instead of the user.
*   **Limitations**:
    *   **Does not solve the PR Review issue.** The PR is still created by the authenticated user (`rolker`) via the API.
    *   Reviewers will see "<Agent Name> committed", but "rolker opened this pull request".

### 2. Machine User (Recommended Solution)
Create a separate GitHub account for the agent.
*   **Implementation**:
    *   Create a new GitHub account (e.g., `ros2-agent`, `rolker-bot`).
    *   Add this user as a collaborator to the workspace repositories.
    *   Generate a PAT for this new user.
    *   Configure the Agent environment to use this new PAT for the GitHub MCP server.
*   **Benefits**:
    *   **Solves PR Review issue**: PRs are opened by `ros2-agent`, allowing `rolker` to request changes and approve them.
    *   Clear distinction in UI (different avatar, username).
*   **limitations**:
    *   Requires managing a second set of credentials.
    *   Requires environment configuration changes.

### 3. GitHub App (Advanced)
Register a GitHub App to act on behalf of the agent.
*   **Benefits**:
    *   Native "Bot" badge.
    *   Higher rate limits.
*   **Limitations**:
    *   Significant complexity to set up and authenticate compared to a simple PAT.

## Proposed Plan

### Phase 1: Establish Authorship (The "Signed-By" Strategy)
**Status: ACTIVE**
We distinguish the **content** author from the **setup** author.

**Agent Responsibilities:**
1.  **Determine your identity** when first starting work in this workspace:
    - Use your actual agent platform name (e.g., "Copilot CLI Agent", "Antigravity Agent")
    - Use email format: `roland+<platform>@ccom.unh.edu`
    - **Ask the user** if uncertain about the appropriate format
2.  **Configure git identity** in ALL repositories:
    - Workspace repository (this repo)
    - All repositories under `workspaces/*/src/*`
3.  **Use the configuration script** (`.agent/scripts/configure_git_identity.sh`) to apply consistently

### Phase 2: Establish Independence (The "Machine User")
**Status: SKIPPED**
User opted for a policy change instead.

### Phase 3: Policy Adjustment (The "0-Review" Strategy)
**Status: ADOPTED**
The user has configured the repository to require 0 reviews for merging.
*   **Result**: The user (rolker) can merge PRs created by the agent (acting as rolker) without needing a second identity to approve them.
*   **Trade-off**: Reduces strictness of code review enforcement but unblocks the workflow immediately.

## Action Items
- [x] **Define Strategy**: Each agent determines its own identity (not hardcoded).
- [x] **Create Configuration Script**: `.agent/scripts/configure_git_identity.sh` to apply identity across all repos.
- [x] **Update Documentation**: Document the "0-Review" policy decision and scope requirements.
- [x] **Clarify Scope**: Identity applies to workspace repo AND all repos in `workspaces/`.
