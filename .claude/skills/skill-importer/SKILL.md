---
name: skill-importer
description: Evaluate and import external skills into the workspace. Checks for redundancy, format, safety, and source attribution before importing.
---

# Skill Importer

## Usage

```
/skill-importer <source-url-or-path>
```

## Overview

**Lifecycle position**: Utility — use when asked to "import a skill", "port a
skill", or "add the X skill from Y".

Evaluate external skills (e.g., from `anthropics/skills` or other repos) for
quality, redundancy, and safety before importing into the workspace. Keeps
the skill library lean by defaulting to rejection for borderline cases.

## Steps

### 1. Source analysis

Fetch and read the external skill:

1. **List files** in the remote skill directory.
2. **Read content** — the `SKILL.md` and any supporting files to understand
   the skill's logic and purpose.
3. **Record source attribution**:
   - Source repository URL
   - Commit SHA or version tag (if available)
   - Original author / license
   - Date of import evaluation

### 2. Quality evaluation

**Goal**: Keep the skills library **lean and effective**. Default to rejection.

Assess against these criteria:

| Criterion | Check |
|-----------|-------|
| **Redundancy** | Does any existing skill in `.claude/skills/` cover the same intent? If yes → **REJECT**. |
| **Utility** | Is this skill highly relevant to this ROS 2 workspace? Generic "coding tips" → REJECT. |
| **Safety** | Does it require dangerous permissions (arbitrary code execution, network access beyond `gh`)? |
| **Quality** | Is the SKILL.md well-structured with clear steps? Vague instructions → REJECT. |
| **Maintenance** | Does it depend on external scripts or services that may break? |

**Recommendation** (choose one):

- **REJECT** — Redundant, low quality, or not relevant. **Default for
  borderline cases.**
- **MERGE / UPDATE** — Overlaps with an existing skill but offers
  improvements. Recommend updating the existing skill instead.
- **MODIFY** — Provides unique value but needs adapting for this workspace.
- **ACCEPT** — High quality, unique, and immediately useful.

Present the recommendation with reasoning before proceeding. Stop here if
the recommendation is REJECT.

### 3. Adaptation planning

For ACCEPT or MODIFY recommendations, plan the adaptation:

- **Frontmatter**: Ensure `SKILL.md` has valid YAML frontmatter with `name`
  and `description` fields.
- **Paths**: Update to use `.claude/skills/` location. Use relative paths
  or reference workspace conventions.
- **Terminology**: Replace framework-specific language with generic terms
  where possible (skills are also used by Copilot and Gemini via adapter
  files).
- **References**: Point to existing workspace knowledge files and templates
  rather than duplicating content (e.g., reference
  `.agent/knowledge/documentation_verification.md` instead of embedding
  verification steps).
- **Dependencies**: Only reference scripts and files that exist in the
  current workspace. Do not assume external tooling.

Output a short adaptation plan listing files and their status
(Keep / Modify / Drop).

### 4. Import and verify

1. **Write files** to `.claude/skills/<skill-name>/`.
2. **Verify structure**:
   - `SKILL.md` exists with valid YAML frontmatter (`name`, `description`)
   - No references to non-existent files or scripts
   - No hardcoded paths outside the workspace
   - Source attribution is recorded in the SKILL.md (see step 1)
3. **Update skill index** — add the new skill to
   `.agent/knowledge/skill_workflows.md`.
4. **Update adapter skill lists** — add the skill name to the "Available
   workflow skills" line in `.github/copilot-instructions.md`,
   `.agent/instructions/gemini-cli.instructions.md`, and
   `.agent/AGENT_ONBOARDING.md`.
5. **Report**: Summarize what was imported, what was adapted, and the source
   attribution.

## Source Attribution

Every imported skill must include a source attribution section in its
SKILL.md or a comment in the frontmatter:

```markdown
## Source

- **Origin**: <repository URL or description>
- **Original commit**: `<sha>` (if applicable)
- **Import date**: <YYYY-MM-DD>
- **Adapted by**: <agent name>
```

This ensures provenance is traceable and licenses are respected.

## Guidelines

- **Default to rejection** — a lean skill library is better than a bloated one.
- **Check existing skills first** — read `.claude/skills/*/SKILL.md` to
  understand what already exists before evaluating.
- **Don't import scaffolding** — if a skill's value is just a template or
  boilerplate, it probably doesn't warrant a full skill entry.
- **Respect licenses** — check the source repository's license before
  importing. Note the license in the source attribution.
