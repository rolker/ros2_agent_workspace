# Agent Onboarding Checklist

Before starting ANY task in this workspace, complete these steps in order:

## 1. Read Critical Documentation (5 min)

These documents are **required reading** for every agent session:

- **[`AI_IDENTITY_STRATEGY.md`](AI_IDENTITY_STRATEGY.md)** ⚠️ **READ FIRST**
  - Explains how to identify yourself in git commits
  - Shows identity format: `<Platform> Agent` / `roland+<platform>@ccom.unh.edu`
  - Documents ephemeral vs. persistent identity configuration methods
  - Includes decision tree for choosing the right approach

- **[`rules/common/git-hygiene.md`](rules/common/git-hygiene.md)**
  - **Never commit to `main`** — always use feature branches
  - Branch naming: `feature/TASK-<ID>-<description>` or `fix/<description>`
  - Always stash/commit uncommitted changes before finishing

- **[`rules/common/ai-signature.md`](rules/common/ai-signature.md)**
  - **Signature Required**: Must append `**🤖 Authored-By**: `<Agent Name>`` to all GitHub Issues/PRs/Comments.

- **[`rules/common/github-cli-best-practices.md`](rules/common/github-cli-best-practices.md)**
  - **Protocol**: MUST use "File-First" approach (temp file + `--body-file`) for all multiline GitHub interactions (Issues/PRs).

- **[`AGENTS.md`](../AGENTS.md)**
  - Overview of agent responsibilities and constraints
  - Quick reference for what agents can/cannot do

## 2. Source the Environment (1 min)

Ensure your shell has the correct ROS 2 environment paths:

```bash
source .agent/scripts/env.sh
```

## 3. Configure Git Identity (1 min)

Before making ANY commits, configure your identity using the appropriate method:

### Host-Based Agents (Copilot CLI, Gemini CLI)

If you're running directly on the host and sharing the working copy with the user:

```bash
source .agent/scripts/set_git_identity_env.sh "Copilot CLI Agent" "roland+copilot-cli@ccom.unh.edu"
```

**Why this method?**
- ✅ Session-only (doesn't modify `.git/config`)
- ✅ User's identity remains intact for their own commits
- ✅ Perfect for shared workspaces

### Containerized Agents (Antigravity)

If you're running in a container or isolated environment:

```bash
./.agent/scripts/configure_git_identity.sh "Antigravity Agent" "roland+antigravity@ccom.unh.edu"
```

**Why this method?**
- ✅ Persists across sessions
- ✅ Configures workspace repo + all repos in `workspaces/*/src/`
- ✅ Isolated environment means no conflict with user

**Not sure which to use?** See the decision tree in [`AI_IDENTITY_STRATEGY.md`](AI_IDENTITY_STRATEGY.md).

## 4. Ticket the Task (2 min)

1.  **Check if a GitHub Issue exists** for your current objective.
2.  If not, **ask the user**: *"Should I open an issue to track this?"*
3.  Use the issue number in your branch name: `feature/ISSUE-<number>-<description>`.
4.  **Note**: When you submit the PR, you MUST include `Closes #<issue-number>` in the description.

## 5. Create a Feature Branch (30 sec)

```bash
git checkout -b feature/ISSUE-<number>-<description>
```

Example:
```bash
git checkout -b feature/ISSUE-45-standardize-issues
```

## 6. Discover Available Workflows (Optional, as needed)

When you need to perform a common task (build, test, status check, etc.):

1. **Check the workflow index**: [`AGENT_INDEX.md`](AGENT_INDEX.md)
2. **Or check the scripts reference**: [`scripts/README.md`](scripts/README.md)

This prevents "task creep" where you manually implement something that already has a workflow.

## 7. Review Other Key Resources

- **[`WORKFORCE_PROTOCOL.md`](WORKFORCE_PROTOCOL.md)** — Multi-agent coordination and GitHub Issues workflow
- **GitHub Issues** — Check open issues to see what's currently being worked on
- **[`README.md`](../README.md)** — Repository structure and usage
- **[`ROADMAP.md`](ROADMAP.md)** (Optional) — Legacy planning document; GitHub Issues are now the Source of Truth

## Quick Checklist for Your Session

Before starting work, verify:

- [ ] Read `AI_IDENTITY_STRATEGY.md`
- [ ] Read `rules/common/git-hygiene.md`
- [ ] Read `rules/common/ai-signature.md`
- [ ] Read `rules/common/github-cli-best-practices.md`
- [ ] Sourced environment: `source .agent/scripts/env.sh`
- [ ] Configured git identity (ephemeral or persistent, based on your environment)
- [ ] **Verified/Created GitHub Issue for task**
- [ ] Created feature branch with `feature/ISSUE-*` naming
- [ ] Checked `AGENT_INDEX.md` for existing workflows
- [ ] Reviewed GitHub Issues for ongoing tasks

## Why This Matters

Following this checklist prevents:
- ❌ Accidentally committing to main
- ❌ Using the wrong git identity (confusing commit history)
- ❌ Disrupting user workflow with persistent config in shared workspaces
- ❌ Asking the user questions already answered in docs
- ❌ Implementing workflows that already exist
- ❌ Duplicating work from other agents

---

**Need help?** Check this directory for additional guidance documents.
