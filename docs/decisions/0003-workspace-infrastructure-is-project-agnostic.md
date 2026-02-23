# ADR-0003: Workspace Infrastructure Is Project-Agnostic

## Status

Accepted

## Context

This workspace was developed alongside a specific ROS 2 project (UNH Marine Autonomy /
Project 11). As the workspace infrastructure matured — worktree workflows, agent
instructions, build scripts, CI pipelines — it became clear that the tooling is generic
and could serve any ROS 2 project, not just the one it was built with.

The more important concern is project independence. If project repos start depending on
workspace conventions — expecting specific build wrappers, referencing workspace paths,
or assuming workspace-level agent infrastructure — they become harder to use outside this
workspace, harder for other contributors to approach, and harder to share. The workspace
should serve projects, not shape them.

Project-specific content also kept creeping into workspace-level files: the glossary
included UNH-specific terminology, instruction files referenced project-specific packages,
and knowledge documents assumed specific sensor configurations. This made the workspace
less reusable and blurred the boundary between infrastructure and project.

## Decision

The workspace repo contains only generic ROS 2 agent infrastructure. Project-specific
content belongs in project repos. Projects must never depend on the workspace — they
should always be usable, buildable, and testable standalone.

Separation mechanism:
- **Workspace repo** owns: infrastructure for building and working with projects *in the
  context of the workspace* (worktree workflows, agent instruction patterns, templates,
  scripts). This includes generic ROS 2 knowledge in `.agent/knowledge/`.
- **Project repos** own: everything needed to use the project standalone — architecture,
  ADRs, conventions, package-specific documentation. A project repo works without this
  workspace.
- **Manifest repo** provides project configuration: layer definitions, bootstrap config,
  and optionally project-level agent context. The manifest repo may be an independent
  repo or part of one of the project repos — this is the project owner's choice about
  whether to include workspace metadata in their repo.
- `.agent/project_knowledge/` is a gitignored symlink that aggregates project-level
  knowledge into the workspace. It complements `.agent/knowledge/` (which holds generic
  ROS 2 and non-project-specific knowledge) by making project-specific knowledge
  accessible in a parallel location without the workspace owning it.

Reusability test: someone should be able to fork this workspace for a different ROS 2
project by changing the bootstrap configuration and running setup, without needing
to remove project-specific content from the workspace itself. Conversely, a project
repo should be fully functional without this workspace.

## Consequences

**Positive:**
- The workspace is reusable across ROS 2 projects
- Clean separation of concerns — workspace improvements don't require project knowledge,
  and project changes don't require workspace changes
- The manifest repo pattern allows different projects to configure the same workspace
  differently
- The separation enables the workspace to offer templates and conventions that projects
  can adopt voluntarily — useful standalone, but also discoverable by the workspace when
  aggregated

**Negative:**
- Requires ongoing discipline to keep project-specific content out of workspace files
- The symlink-based knowledge aggregation (`.agent/project_knowledge/`) adds a layer of
  indirection that can be confusing
- Some convenience is lost — you can't just put everything in one place
