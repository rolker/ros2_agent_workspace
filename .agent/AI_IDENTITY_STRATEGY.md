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
3. **Choose the appropriate configuration method** (see below)
4. **Configure git** with your identity before making any commits

**Example identities:**
- Antigravity Agent: `Antigravity Agent` / `roland+antigravity@ccom.unh.edu`
- GitHub Copilot CLI: `Copilot CLI Agent` / `roland+copilot-cli@ccom.unh.edu`
- Gemini CLI: `Gemini CLI Agent` / `roland+gemini-cli@ccom.unh.edu`
- Other agents: Follow the pattern `<Platform> Agent` / `roland+<platform>@ccom.unh.edu`

### Configuration Methods: Ephemeral vs. Persistent

#### When to Use Ephemeral Identity (Host-Based Agents)

**Use for**: Copilot CLI, Gemini CLI, or any agent running directly on the host that shares the working copy with the human user.

**Method**: Source the environment variable script:
```bash
source .agent/scripts/set_git_identity_env.sh "Copilot CLI Agent" "roland+copilot-cli@ccom.unh.edu"
```

**Benefits**:
- ✅ Does NOT modify `.git/config` (user's identity remains intact)
- ✅ Identity applies only to current shell session
- ✅ User can commit as themselves after agent session ends
- ✅ Perfect for shared workspaces

**How it works**: Sets `GIT_AUTHOR_NAME`, `GIT_AUTHOR_EMAIL`, `GIT_COMMITTER_NAME`, `GIT_COMMITTER_EMAIL` environment variables which take precedence over `.git/config`.

#### When to Use Persistent Identity (Containerized Agents)

**Use for**: Antigravity, containerized agents, or dedicated agent-only checkouts.

**Method**: Run the configuration script:
```bash
./.agent/scripts/configure_git_identity.sh "Antigravity Agent" "roland+antigravity@ccom.unh.edu"
```

**Benefits**:
- ✅ Persists across all shell sessions
- ✅ Automatically configures all repositories in `workspaces/*/src/*`
- ✅ No need to re-run for each session

**Trade-offs**:
- ⚠️ Modifies `.git/config` persistently
- ⚠️ Not suitable for shared workspaces (disrupts user workflow)

### Decision Tree

```
Are you running in a container or isolated environment?
│
├─ YES → Use Persistent Identity
│         (./.agent/scripts/configure_git_identity.sh)
│
└─ NO → Do you share this working copy with a human user?
         │
         ├─ YES → Use Ephemeral Identity
         │         (source .agent/scripts/set_git_identity_env.sh)
         │
         └─ NO → Use Persistent Identity
                  (./.agent/scripts/configure_git_identity.sh)
```

**Scope**: Identity must apply to:
- The workspace repository (`ros2_agent_workspace`)
- **All repositories under `workspaces/*/src/*`** (Note: Ephemeral method applies to all git commands in the session)

## Strategic Options

### 1. Git Authorship Distinction (Current Implementation)
We configure each agent to use a distinct name and email for git commits.
*   **Implementation**:
    *   Determine appropriate identity for your agent (ask user if needed)
    *   Choose appropriate configuration method (ephemeral vs. persistent - see "Agent Identity Configuration" section above)
    *   **Ephemeral**: `source .agent/scripts/set_git_identity_env.sh` (for host-based agents in shared workspaces)
    *   **Persistent**: `./.agent/scripts/configure_git_identity.sh` (for containerized agents or dedicated checkouts)
*   **Benefits**:
    *   Git history clearly distinguishes agent commits from human commits.
    *   Blame view shows the agent name instead of the user.
    *   Ephemeral approach allows shared workspace usage without disrupting user workflow.
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

## GitHub Content Strategy (Mandatory)

To address the limitations of the "Git Authorship Distinction" strategy (specifically PR attribution), we implement a **Content Signature** protocol.

**Rule**: All AI agents must append a structured signature to the body of any GitHub Issue, Pull Request, or Comment they create via the API.

Details: [**`.agent/rules/common/ai-signature.md`**](rules/common/ai-signature.md)

## Proposed Plan

### Phase 1: Establish Authorship (The "Signed-By" Strategy)
**Status: ACTIVE**
We distinguish the **content** author from the **setup** author.

**Agent Responsibilities:**
1.  **Determine your identity** when first starting work in this workspace:
    - Use your actual agent platform name (e.g., "Copilot CLI Agent", "Antigravity Agent", "Gemini CLI Agent")
    - Use email format: `roland+<platform>@ccom.unh.edu`
    - **Ask the user** if uncertain about the appropriate format
2.  **Choose the appropriate configuration method**:
    - **Host-based agents (Copilot CLI, Gemini CLI)**: Use ephemeral identity (environment variables)
    - **Containerized agents (Antigravity)**: Use persistent identity (.git/config)
    - See "Configuration Methods: Ephemeral vs. Persistent" section above for details
3.  **Configure git identity** before making any commits:
    - **Ephemeral**: `source .agent/scripts/set_git_identity_env.sh "<Name>" "<Email>"`
    - **Persistent**: `./.agent/scripts/configure_git_identity.sh "<Name>" "<Email>"`

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
- [x] **Create Configuration Script**: `.agent/scripts/configure_git_identity.sh` to apply identity across all repos (persistent method).
- [x] **Create Ephemeral Script**: `.agent/scripts/set_git_identity_env.sh` for host-based agents in shared workspaces.
- [x] **Update Documentation**: Document both ephemeral and persistent approaches with decision tree.
- [x] **Clarify Scope**: Identity applies to workspace repo AND all repos in `workspaces/`.
- [x] **Enforce Content Signatures**: Agents must sign GitHub comments/PRs (see `rules/common/ai-signature.md`).
