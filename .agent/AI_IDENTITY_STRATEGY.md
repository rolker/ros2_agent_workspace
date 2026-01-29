# AI Identity Strategy

## Problem Statement
Currently, AI agents operate using the user's personal GitHub credentials (PAT). This causes two issues:
1.  **Identity Confusion**: Commits and PRs appear as if they were created by the user (`rolker`), making it difficult to distinguish between human-generated and agent-generated code in the history.
2.  **Review Blocking**: GitHub prevents users from approving their own Pull Requests, making it difficult to identify issues and PRs submitted by AI agents. Since the agent-created PR is owned by `rolker`, `rolker` cannot approve it, hindering the desired peer-review workflow.

## Agent Identity Configuration

**Important**: Each AI agent must identify itself appropriately in both git commits AND GitHub signatures (Issues/PRs/Comments).

### Full Identity Components

Each agent has a complete identity consisting of:
1. **Framework Name** - e.g., "Copilot CLI Agent", "Gemini CLI Agent"
2. **Email Address** - e.g., "roland+copilot-cli@ccom.unh.edu"
3. **Model Name** - e.g., "GPT-4o", "Gemini 2.0 Flash", "Claude 3.5 Sonnet"
4. **Framework ID** - e.g., "copilot", "gemini", "antigravity"

### Identity Source of Truth

Agent identity is determined from these sources (in order of preference):

1. **Environment variables** (most reliable) - `AGENT_NAME`, `AGENT_EMAIL`, `AGENT_MODEL`, `AGENT_FRAMEWORK`
   - Set by `set_git_identity_env.sh` when you configure your session
   - Always current for the active shell session
   - **Recommended method** for all identity needs

2. **Auto-detection** - Runtime detection via `.agent/scripts/detect_agent_identity.sh`
   - Automatically detects framework and model from environment
   - Can export variables or write to file

3. **`.agent/.identity` file** (if it exists) - Runtime-generated configuration file
   - Contains all identity components
   - **Note**: This is a git-ignored runtime file that may become stale if model/config changes during session
   - Template is at `.agent/.identity.template`

**How to determine your identity:**
1. **Use environment variables** (recommended) - Set by `set_git_identity_env.sh --detect`
2. **Read from** `.agent/.identity` file if it exists (check with `[ -f .agent/.identity ]`)
3. **Ask the user** if auto-detection fails
4. **Use fallback** values if necessary: "AI Agent" / "Unknown Model"
5. **Configure git** with your identity before making any commits

**Example identities:**
- Antigravity Agent: `Antigravity Agent` / `roland+antigravity@ccom.unh.edu` / `Gemini 2.5 Pro`
- GitHub Copilot CLI: `Copilot CLI Agent` / `roland+copilot-cli@ccom.unh.edu` / `GPT-4o`
- Gemini CLI: `Gemini CLI Agent` / `roland+gemini-cli@ccom.unh.edu` / `Gemini 2.0 Flash`
- Other agents: Follow the pattern `<Platform> Agent` / `roland+<platform>@ccom.unh.edu` / `<Model Name>`

### Configuration Methods: Ephemeral vs. Persistent

#### When to Use Ephemeral Identity (Host-Based Agents)

**Use for**: Copilot CLI, Gemini CLI, or any agent running directly on the host that shares the working copy with the human user.

**Method**: Source the environment variable script:
```bash
# Auto-detect (recommended)
source .agent/scripts/set_git_identity_env.sh --detect

# Or specify framework
source .agent/scripts/set_git_identity_env.sh --agent copilot

# Or manual (not recommended)
source .agent/scripts/set_git_identity_env.sh "Copilot CLI Agent" "roland+copilot-cli@ccom.unh.edu"
```

**Benefits**:
- ✅ Does NOT modify `.git/config` (user's identity remains intact)
- ✅ Identity applies only to current shell session
- ✅ User can commit as themselves after agent session ends
- ✅ Perfect for shared workspaces
- ✅ Exports `AGENT_MODEL` for use in GitHub signatures

**How it works**: Sets `GIT_AUTHOR_NAME`, `GIT_AUTHOR_EMAIL`, `GIT_COMMITTER_NAME`, `GIT_COMMITTER_EMAIL` environment variables which take precedence over `.git/config`. Also exports `AGENT_NAME`, `AGENT_EMAIL`, `AGENT_MODEL`, `AGENT_FRAMEWORK` for use in GitHub API signatures.

#### When to Use Persistent Identity (Containerized Agents)

**Use for**: Antigravity, containerized agents, or dedicated agent-only checkouts.

**Method**: Run the configuration script:
```bash
./.agent/scripts/configure_git_identity.sh "Antigravity Agent" "roland+antigravity@ccom.unh.edu"
```

**Benefits**:
- ✅ Persists across all shell sessions
- ✅ Automatically configures all repositories in `layers/*/src/*`
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
- **All repositories under `layers/*/src/*`** (Note: Ephemeral method applies to all git commands in the session)

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

The signature must include both:
1. **Agent Name** (`🤖 Authored-By`) - Who created the content
2. **Model Name** (`🧠 Model`) - Which AI model was used

### Model Identity Introspection

**Critical**: Agents must use their **actual runtime model name**, not copy example values.

**How to determine your model:**
1. **Use environment variables** (recommended) - Check `$AGENT_MODEL` set by `set_git_identity_env.sh`
2. **Read from `.agent/.identity` file** (if exists) - Check file exists first: `[ -f .agent/.identity ] && source .agent/.identity`
3. **Auto-detect** - Run `.agent/scripts/detect_agent_identity.sh --export`
4. **Fallback** - Use "Unknown Model" if detection fails

**Note on `.agent/.identity` file**: This is a runtime-generated file (git-ignored) that may become stale if the model changes during a session. Environment variables set by `set_git_identity_env.sh` are always current for the active shell and are the preferred method.

**DO NOT**:
- ❌ Copy example model names from documentation (e.g., "GPT-4o", "Gemini 2.0 Flash")
- ❌ Guess or assume your model name
- ❌ Hardcode model names based on examples

**Correct approach**:
```bash
# Use environment variables (preferred - always current)
echo "Model: $AGENT_MODEL"

# Or read from file if it exists (may be stale)
if [ -f .agent/.identity ]; then
    source .agent/.identity
    echo "Model: $AGENT_MODEL"
fi
```

Details: [**`.agent/rules/common/ai-signature.md`**](rules/common/ai-signature.md)

## Proposed Plan

### Phase 1: Establish Authorship (The "Signed-By" Strategy)
**Status: ACTIVE**
We distinguish the **content** author from the **setup** author.

**Agent Responsibilities:**
1.  **Determine your complete identity** when first starting work in this workspace:
    - **Framework name**: e.g., "Copilot CLI Agent", "Antigravity Agent", "Gemini CLI Agent"
    - **Email format**: `roland+<platform>@ccom.unh.edu`
    - **Model name**: Your actual runtime model (e.g., "GPT-4o", "Gemini 2.5 Pro")
    - **Auto-detect** using: `source .agent/scripts/set_git_identity_env.sh --detect`
    - **Or read from**: `.agent/.identity` file
    - **Ask the user** if auto-detection fails
2.  **Choose the appropriate configuration method**:
    - **Host-based agents (Copilot CLI, Gemini CLI)**: Use ephemeral identity (environment variables)
    - **Containerized agents (Antigravity)**: Use persistent identity (.git/config)
    - See "Configuration Methods: Ephemeral vs. Persistent" section above for details
3.  **Configure git identity** before making any commits:
    - **Ephemeral**: `source .agent/scripts/set_git_identity_env.sh --detect` (or `--agent <framework>`)
    - **Persistent**: `./.agent/scripts/configure_git_identity.sh "<Name>" "<Email>"`
4.  **Use correct model name in signatures**:
    - After configuration, `$AGENT_MODEL` environment variable will be set
    - Use this in all GitHub signatures (Issues/PRs/Comments)
    - Never copy example model names from documentation

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
- [x] **Clarify Scope**: Identity applies to workspace repo AND all repos in `layers/`.
- [x] **Enforce Content Signatures**: Agents must sign GitHub comments/PRs (see `rules/common/ai-signature.md`).
