# AI Identity Strategy

## Problem Statement
Currently, AI agents operate using the user's personal GitHub credentials (PAT). This causes two issues:
1.  **Identity Confusion**: Commits and PRs appear as if they were created by the user (`rolker`), making it difficult to distinguish between human-generated and agent-generated code in the history.
2.  **Review Blocking**: GitHub prevents users from approving their own Pull Requests. identifying issues and PRs submitted by AI agents. Since the agent-created PR is owned by `rolker`, `rolker` cannot approve it, hindering the desired peer-review workflow.

## Strategic Options

### 1. Git Authorship Distinction (Immediate Mitigation)
We can configure the agent to use a distinct name and email for git commits.
*   **Implementation**:
    *   Set `git config user.name "Antigravity Agent"`
    *   Set `git config user.email "roland+agent@ccom.unh.edu"` (using suffix or sub-addressing)
*   **Benefits**:
    *   Git history clearly distinguishes agent commits.
    *   Blame view shows "Antigravity Agent".
*   **Limitations**:
    *   **Does not solve the PR Review issue.** The PR is still created by the authenticated user (`rolker`) via the API.
    *   Reviewers will see "Antigravity Agent committed", but "rolker opened this pull request".

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
**Status: COMPLETED**
We have distinguished the **content** author from the **setup** author.
1.  Defined standard identity for the agent.
    *   Name: `Antigravity Agent`
    *   Email: `roland+antigravity@ccom.unh.edu`
2.  Updated the agent's workflow by configuring these credentials in the local git repo.
    *   *Executed via `git config` in the workspace.*

### Phase 2: Establish Independence (The "Machine User")
**Status: SKIPPED**
User opted for a policy change instead.

### Phase 3: Policy Adjustment (The "0-Review" Strategy)
**Status: ADOPTED**
The user has configured the repository to require 0 reviews for merging.
*   **Result**: The user (rolker) can merge PRs created by the agent (acting as rolker) without needing a second identity to approve them.
*   **Trade-off**: Reduces strictness of code review enforcement but unblocks the workflow immediately.

## Action Items
- [x] **Define Agent Identity**: Agree on the Name/Email to use.
- [x] **Configure Project**: Create a script or configuration to create a local `.git/config` override for the workspace.
- [x] **Update Documentation**: Document the "0-Review" policy decision.
