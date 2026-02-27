# Knowledge Update Issue Template

Use this template to create issues for updating project knowledge summaries.
Create the issue on the repo where summaries should be written.

**Two modes**:
- **Manifest repo**: Full workspace scan — generates `workspace_overview.md`,
  `governance_summary.md`, and `project_profiles/` for all repos.
- **Non-manifest repo**: Self-profile only — generates a single profile for the
  current repo in its own `.agents/workspace-context/`.

## Issue Title

```
Update project knowledge summaries
```

## Issue Body

```markdown
## Summary

Regenerate project knowledge summaries in `.agents/workspace-context/`.

**Mode**: [manifest / non-manifest]
- **Manifest repo**: Scans all workspace repos and generates full summaries
  (`workspace_overview.md`, `governance_summary.md`, `project_profiles/`).
- **Non-manifest repo**: Generates a self-profile only for this repo.

## Steps

1. Create a layer worktree for this issue targeting this repo:
   ```bash
   .agent/scripts/worktree_create.sh --issue <N> --type layer --layer <layer> --packages <this-repo>
   source .agent/scripts/worktree_enter.sh --issue <N>
   cd <layer>_ws/src/<this-repo>
   ```

2. Run the `gather-project-knowledge` skill (Claude Code) or follow its
   instructions manually:
   - Run `.agent/scripts/discover_governance.sh --json` from the workspace root
   - Scan project repos for package metadata (all repos for manifest mode,
     current repo only for non-manifest mode)
   - Generate/update summaries in `.agents/workspace-context/`
   - Commit the changes

3. Push and open a PR for review.

## Context

- Last updated: [date or "never"]
- Trigger: [what prompted this update — new repo added, governance docs changed, periodic refresh]
- Mode: [manifest / non-manifest]
```
