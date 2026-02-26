# GitHub Copilot — Workspace Rules

Read and follow all rules in [`AGENTS.md`](../AGENTS.md) at the repository root.
That file contains the shared workspace rules for all AI agents.

## Environment Setup

```bash
source .agent/scripts/env.sh                    # ROS 2 + checkout guardrail
source .agent/scripts/set_git_identity_env.sh "Copilot CLI Agent" "roland+copilot-cli@ccom.unh.edu"
```

After sourcing, verify `$AGENT_MODEL` matches your actual model (from your system prompt).
If stale, update the default in `.agent/scripts/framework_config.sh` and commit the one-line fix.

## Copilot-Specific Notes

- **Native GitHub access**: Use `gh` CLI for fast PR/issue operations.
- **ROS 2**: Jazzy, Kilted, and Rolling (multi-distro support).
- **Build system**: colcon. **VCS tool**: vcstool. **Python**: 3.10+.


## References

- [`AGENTS.md`](../AGENTS.md) — Shared workspace rules (all agents)
- [`ARCHITECTURE.md`](../ARCHITECTURE.md) — System design and layering
- [`.agent/WORKTREE_GUIDE.md`](../.agent/WORKTREE_GUIDE.md) — Detailed worktree patterns
- [`.agent/AI_IDENTITY_STRATEGY.md`](../.agent/AI_IDENTITY_STRATEGY.md) — Multi-framework identity
- [`.agent/WORKFORCE_PROTOCOL.md`](../.agent/WORKFORCE_PROTOCOL.md) — Multi-agent coordination
- [`.agent/knowledge/`](../.agent/knowledge/) — ROS 2 development patterns and CLI best practices
- [`.agent/project_knowledge/`](../.agent/project_knowledge/) — Project-specific conventions (symlink, may not exist)
- [`.agent/templates/`](../.agent/templates/) — Issue and test templates
