# Agent Onboarding Checklist

Before starting ANY task in this workspace, complete these steps in order:

## 1. Read Critical Documentation (5 min)

These documents are **required reading** for every agent session:

- **[`.agent/AI_IDENTITY_STRATEGY.md`](.agent/AI_IDENTITY_STRATEGY.md)** ⚠️ **READ FIRST**
  - Explains how to identify yourself in git commits
  - Shows identity format: `<Platform> Agent` / `roland+<platform>@ccom.unh.edu`
  - Current agent: `Copilot CLI Agent` / `roland+copilot-cli@ccom.unh.edu`

- **[`.agent/rules/common/git-hygiene.md`](.agent/rules/common/git-hygiene.md)**
  - **Never commit to `main`** — always use feature branches
  - Branch naming: `feature/TASK-<ID>-<description>` or `fix/<description>`
  - Always stash/commit uncommitted changes before finishing

- **[`AGENTS.md`](AGENTS.md)**
  - Overview of agent responsibilities and constraints
  - Quick reference for what agents can/cannot do

## 2. Configure Git Identity (1 min)

Before making ANY commits, configure your identity:

```bash
./.agent/scripts/configure_git_identity.sh "Copilot CLI Agent" "roland+copilot-cli@ccom.unh.edu"
```

This configures git in:
- The workspace repository (this repo)
- All 19 repositories in `workspaces/*/src/`

## 3. Create a Feature Branch (30 sec)

```bash
git checkout -b feature/TASK-<ID>-<description>
```

Example:
```bash
git checkout -b feature/TASK-001-add-workflow-index
```

## 4. Discover Available Workflows (Optional, as needed)

When you need to perform a common task (build, test, status check, etc.):

1. **Check the workflow index**: [`.agent/AGENT_INDEX.md`](.agent/AGENT_INDEX.md)
2. **Or check the scripts reference**: [`.agent/scripts/README.md`](.agent/scripts/README.md)

This prevents "task creep" where you manually implement something that already has a workflow.

## 5. Review Other Key Resources

- **[`.agent/ROADMAP.md`](.agent/ROADMAP.md)** — What's currently being worked on
- **[`.agent/WORKFORCE_PROTOCOL.md`](.agent/WORKFORCE_PROTOCOL.md)** — Multi-agent coordination
- **[`README.md`](README.md)** — Repository structure and usage

## Quick Checklist for Your Session

Before starting work, verify:

- [ ] Read AI_IDENTITY_STRATEGY.md
- [ ] Read git-hygiene.md  
- [ ] Ran `configure_git_identity.sh`
- [ ] Created feature branch with `feature/TASK-*` naming
- [ ] Checked `.agent/AGENT_INDEX.md` for existing workflows
- [ ] Reviewed `.agent/ROADMAP.md` for ongoing tasks

## Why This Matters

Following this checklist prevents:
- ❌ Accidentally committing to main
- ❌ Using the wrong git identity (confusing commit history)
- ❌ Asking the user questions already answered in docs
- ❌ Implementing workflows that already exist
- ❌ Duplicating work from other agents

---

**Need help?** Check the [`.agent/`](.agent/) directory for additional guidance documents.
