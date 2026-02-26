---
name: research
description: Survey external sources on a topic and maintain living research digests. Workspace digest is tracked; project digest lives in the manifest repo.
---

# Research

## Usage

```
/research <topic>                      # Add to workspace digest (default)
/research --scope project <topic>      # Add to project digest
/research --ingest <url>               # Extract takeaways from a URL
/research --refresh                    # Re-survey known topics, prune stale entries
```

## Overview

Maintain living research digests that capture external best practices, emerging
techniques, and relevant developments. This is **external research** — surveying
sources outside the workspace. For project introspection, use
`gather-project-knowledge`.

## Digests

Two digests, each git-tracked so they're shared across agents and sessions:

### Workspace digest

**Location**: `.agent/knowledge/research_digest.md`

Topics relevant to any project using this workspace:
- Agent workflow patterns and multi-framework coordination
- CI/CD for AI-assisted development
- Governance automation and enforcement patterns
- ROS 2 build system and tooling developments
- Worktree and git workflow patterns

### Project digest

**Location**: `.agents/workspace-context/research_digest.md` (in the manifest repo,
shared via the `project_knowledge` symlink)

Topics specific to the project domain:
- ROS 2 ecosystem developments (new packages, API changes)
- Domain-specific developments (marine autonomy, hydrography, sensors)
- Relevant conference papers or technical reports

## Digest Format

```markdown
# Research Digest: <scope>

<!-- Last updated: YYYY-MM-DD -->
<!-- If older than 30 days, consider running /research --refresh; entries older than 90 days should be flagged for review -->

## <Topic Title>

**Added**: YYYY-MM-DD | **Sources**: [link1](url), [link2](url)

Key takeaways:
- <concise finding>
- <concise finding>

**Relevance**: <why this matters to the workspace/project>

---

## <Next Topic>
...
```

## Workflow

### Adding research (`/research <topic>`)

1. Search the web for current information on the topic
2. Read and synthesize relevant sources
3. Check the existing digest for related entries — update rather than duplicate
4. Append or update the entry in the appropriate digest
5. Update the "Last updated" timestamp
6. Commit the change with a descriptive message

### Ingesting a URL (`/research --ingest <url>`)

1. Fetch and read the URL content
2. Extract key takeaways relevant to the workspace or project
3. Determine scope (workspace or project) from content — or ask if unclear
4. Append to the appropriate digest
5. Commit the change

### Refreshing (`/research --refresh`)

1. Read both digests
2. For each entry, check if it's still current:
   - Search for updates on the topic
   - Update findings if new information exists
   - Mark as stale or remove if no longer relevant
3. Update timestamps
4. Commit changes

## Guidelines

- **External sources only** — this skill surveys the web, papers, docs, and
  community resources. It does not scan the workspace codebase.
- **Concise entries** — each topic should be 5-15 lines. The digest is a
  reference, not a literature review.
- **Cite sources** — every entry needs at least one link.
- **Dedup** — check for existing entries on the same topic before adding.
  Update existing entries rather than creating duplicates.
- **Commit each update** — so the digest history is traceable.
- **Staleness** — entries older than 90 days without a refresh should be
  flagged for review. Fast-moving topics (ROS 2 releases) may need shorter
  windows.
