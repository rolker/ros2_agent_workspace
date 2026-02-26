---
name: brainstorm
description: Explore possibilities for a topic by combining research digests with project knowledge and governance context.
---

# Brainstorm

## Usage

```
/brainstorm <topic>
```

## Overview

Creative exploration tool. Reads research digests, project knowledge, and
governance docs to explore possibilities for a given topic. Output stays in
the conversation unless the user asks to save it.

**Lifecycle position**: Before `review-issue` — exploratory thinking before
an issue exists. Can also be used mid-task to explore alternatives.

## Steps

### 1. Load context

Read available knowledge sources:

- `.agent/knowledge/research_digest.md` — workspace research (if exists)
- `.agent/project_knowledge/research_digest.md` — project research (if exists)
- `.agent/project_knowledge/` — aggregated project knowledge (if exists)
- `docs/PRINCIPLES.md` — workspace principles (for feasibility checks)
- Project governance docs if the topic is project-specific

Check digest freshness — if older than 30 days, note it and suggest
running `/research --refresh` but don't block on it.

### 2. Explore the topic

For the given topic, consider:

- **What exists**: What does the workspace/project already have that relates?
- **What's possible**: What approaches could work? Draw on research digest
  entries for external patterns and best practices.
- **What's missing**: What gaps exist? What would need to be built or changed?
- **Trade-offs**: What are the pros and cons of different approaches?
- **Principle alignment**: Would any approach conflict with workspace or
  project principles?

### 3. Present findings

Structure the output as:

```markdown
## Brainstorm: <topic>

### Current State
<what exists today>

### Possibilities
1. **<approach>** — <description>
   - Pros: ...
   - Cons: ...
   - Research: <reference to digest entry if applicable>

2. **<approach>** — ...

### Gaps
- <what's missing that any approach would need>

### Principle Considerations
- <any principle alignment issues>

### Suggested Next Steps
- <concrete actions to explore further or begin implementation>
```

## Guidelines

- **Explore, don't decide** — present options with trade-offs. The user
  decides which direction to go.
- **Reference research** — when drawing on digest entries, cite them. This
  validates the digests' usefulness and helps the user trace your reasoning.
- **Stay grounded** — possibilities should be feasible within the workspace's
  architecture and constraints. Wild ideas are welcome but should be flagged
  as speculative.
- **Don't create issues** — brainstorming output may lead to issues, but
  the user should make that call.
- **Keep it conversational** — this is the most informal of the lifecycle
  skills. Be thorough but not rigid.
