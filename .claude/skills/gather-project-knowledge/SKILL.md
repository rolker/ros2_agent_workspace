---
name: gather-project-knowledge
description: Scan workspace repos and generate project knowledge summaries for `.agents/workspace-context/`. Requires a GitHub issue and layer worktree on the target repo.
---

# Gather Project Knowledge

## Overview

**Lifecycle position**: Utility — run after repo changes to refresh project
knowledge summaries. Not tied to the per-issue lifecycle.

This skill operates in two modes depending on the target repo:

- **Manifest repo mode**: Scans all workspace repos and generates full summaries
  (`workspace_overview.md`, `governance_summary.md`, `project_profiles/`).
  Detected when the repo contains `config/bootstrap.yaml`.
- **Non-manifest repo mode**: Generates a self-profile only, written to the
  repo's own `.agents/workspace-context/`.

Output is written to `.agents/workspace-context/` in the current repo.

**Prerequisites**: This skill requires a GitHub issue and a layer worktree on the
target repo. It does not create issues, worktrees, or PRs — it only generates and
commits content. Use the
[knowledge update issue template](../../../.agent/templates/knowledge_update_issue.md)
to create the issue, then:

```bash
.agent/scripts/worktree_create.sh --issue <N> --type layer --layer <layer> --packages <target-repo>
source .agent/scripts/worktree_enter.sh --issue <N>
cd <layer>_ws/src/<target-repo>
```

## Workflow

### 1. Run the discovery script

```bash
.agent/scripts/discover_governance.sh --json
```

This produces a JSON-lines inventory of all governance documents across the
workspace and project repos.

### 2. Scan project repos and build the package inventory

For every project repo under `layers/main/*/src/`:

- Glob for all `package.xml` files in the repo
- Extract the `<name>` element from each `package.xml` — this is the only
  authoritative source for package names (not directory names, not repo names)
- Read `<description>`, `<depend>`, `<build_depend>`, and `<exec_depend>`
  elements for dependencies
- Detect language: `CMakeLists.txt` → C++, `setup.py`/`setup.cfg` → Python,
  both → mixed
- Read `.agents/README.md` if it exists (curated agent guide)
- Read `.agents/workspace-context/` contents if they exist (existing knowledge provider)
- Note the layer each repo belongs to

After scanning, produce a **structured inventory table** in your context. This
table is the single source of truth for all subsequent profile and overview
writing. Do not paraphrase or summarize it — copy package names verbatim.

```markdown
| Repo | Layer | Packages (from `<name>` in `package.xml`) | Language | Key Dependencies |
|------|-------|-------------------------------------------|----------|-----------------|
| repo_name | layer_name | `pkg_a`, `pkg_b`, `pkg_c` | C++ | dep1, dep2 |
```

> **Why this matters**: The first run of this skill produced errors in all
> spot-checked profiles because directory names were used instead of `<name>`
> elements, and multi-hop summarization lost fidelity. See
> [#320](https://github.com/rolker/ros2_agent_workspace/issues/320) for details.

**Important constraints**:
- Never use directory names as package names — a directory may contain multiple
  packages, or the directory name may differ from the package name.
- Write profiles one at a time, referencing the inventory table directly.
  Do not delegate profile writing to a subagent that works from a natural
  language summary of the scan results.

### 3. Generate summaries

Write files to `.agents/workspace-context/` in the current repo. The set of
files depends on the mode.

#### Manifest repo mode

Detected when the repo contains `config/bootstrap.yaml`. Generates all three
summary types:

##### `workspace_overview.md`

High-level workspace inventory:

- Layer structure (which layers exist, what they contain)
- Package inventory table: repo, layer, packages, language, brief description
- Cross-repo relationships (shared dependencies, message types)
- Repos with governance docs vs repos without

##### `governance_summary.md`

Unified governance view organized by theme, not by repo:

- Workspace principles (from `docs/PRINCIPLES.md`) — short summary of each
- ADR index with one-line summaries and applicability
- Project-level principles (from any repo's `PRINCIPLES.md`) — note where
  they differ from or extend workspace principles
- Governance coverage: which repos have principles, ADRs, agent guides

##### `project_profiles/<repo>.md`

One file per project repo. For repos with an `.agents/README.md`, summarize it.
For repos without, generate a lightweight profile from the scan:

- Packages found (from `package.xml`)
- Language (C++ / Python / mixed — from `CMakeLists.txt` vs `setup.py`)
- Layer membership
- Key dependencies
- Whether governance docs exist
- Flag: "No `.agents/README.md` — consider creating one"

#### Non-manifest repo mode

For any project repo that is not the manifest repo. Generates a self-profile
only:

##### `.agents/workspace-context/<repo-name>.md`

A single profile of the current repo, using the same format as the manifest
repo's `project_profiles/<repo>.md` above. This allows the repo to provide
its own knowledge to the workspace without depending on a central scan.

### 4. Validate profiles against source

Before adding frontmatter or committing, validate every generated profile:

For each profile, glob for `package.xml` files in the corresponding repo and
extract `<name>` elements. Compare this list against the packages listed in the
profile. If there is any mismatch — missing packages, extra packages, or wrong
names — fix the profile before proceeding.

Quick validation approach:
```bash
# In the repo directory, list all actual package names:
find . -name package.xml -not -path '*/build/*' -not -path '*/install/*' -print0 \
  | xargs -0 sed -n 's/.*<name>\([^<]*\)<\/name>.*/\1/p'
```

Compare the output against what the profile claims. Every name must match
exactly. Do not proceed to step 5 until all profiles pass validation.

### 5. Add frontmatter to generated files

Every generated file should start with:

```markdown
<!-- Generated by gather-project-knowledge skill. Do not edit manually. -->
<!-- Regenerate by running this skill in a layer worktree for a knowledge-update issue. -->
<!-- Source: workspace at {workspace_repo_url} -->
<!-- Generated: {date} -->
```

### 6. Commit the changes

Stage all files in `.agents/workspace-context/` and commit with a message like:

```
Update project knowledge summaries

Scanned N repos across M layers. Found X packages.
Changes: [brief summary of what changed]
```

Report what was generated and what changed compared to the previous version.

## Output Structure

```
.agents/workspace-context/
├── workspace_overview.md
├── governance_summary.md
└── project_profiles/
    ├── unh_marine_autonomy.md
    ├── camp.md
    └── ...
```

## Guidelines

- **Package names come from `package.xml` only** — repo directory names,
  subdirectory names, and ROS package names are different things. The only
  authoritative source for a package name is the `<name>` element in
  `package.xml`. A repo named `s57_tools` may contain packages named
  `marine_charts`, `s57_layer`, etc. A directory named `vrx_urdf` may contain
  multiple packages with different names.
- **Verify against source** — every claim must come from actual files read during
  the scan. Do not guess package descriptions or dependencies.
- **Keep summaries concise** — the point is to save context window. A profile
  should be 20-50 lines, not a full reproduction of the source.
- **Flag gaps** — if a repo has no agent guide or no governance docs, note it.
  This helps prioritize documentation work.
- **Preserve existing content** — if `.agents/workspace-context/` already has
  hand-written files (not generated), do not overwrite them. Only update files
  with the generated-file frontmatter.
- **Respect other knowledge providers** — if a project repo has its own
  `.agents/workspace-context/`, read it as input but don't overwrite it.
  Summarize its content in the workspace-level view.
