# Workspace Knowledge

Workspace-level guidance for AI agents. All files in this directory are
version-controlled and apply regardless of which project repositories are
checked out in `layers/`.

## ROS 2 Development
- **[ROS 2 CLI Best Practices](ros2_cli_best_practices.md)**: Using the `ros2` CLI effectively as an automated agent.
- **[ROS 2 Development Patterns](ros2_development_patterns.md)**: Package structure and build patterns for the layered workspace.

## Agent Workflows
- **[Documentation Verification](documentation_verification.md)**: Mandatory verification workflow for writing accurate ROS 2 package documentation. Includes command cookbook and hallucination anti-patterns.

## Project-Specific Knowledge

Project-specific conventions and architecture docs are available via
`.agent/project_knowledge/` (a symlink to the manifest repo's `agent_context/`
directory, created by `setup.sh`). This symlink may not exist if the manifest
repo does not provide an `agent_context/` directory.

## Project-Level Agent Guides

Project repositories may contain a `.agents/README.md` at their root with repo-specific
guidance for agents. To create one, use the template at
[`../templates/project_agents_guide.md`](../templates/project_agents_guide.md).

---
*Note: This index is manually maintained. Update it when adding new knowledge files.*
