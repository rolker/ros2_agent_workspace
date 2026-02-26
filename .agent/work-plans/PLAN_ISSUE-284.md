# Plan: Unify project agent directories under .agents/

**Issue**: #284
**Status**: Plan

## Problem Analysis

Project repos have two unrelated agent directory conventions:
- `.agents/README.md` — agent onboarding (referenced throughout AGENTS.md)
- `config/agent_context/` — workspace-facing knowledge (symlinked by setup.sh)

This is confusing. Unify under `.agents/` at project repo root.

## Changes

### 1. setup.sh — Update symlink target
Change from `config/agent_context/` to `.agents/workspace-context/`.
No backward compatibility — clean cutover.

### 2. project_agents_guide.md template — Expand .agents/ structure
Add `work-plans/` and `workspace-context/` to the template's layout,
with explanation of standalone vs workspace-integrated usage.

### 3. Documentation updates (5 files)
- `ARCHITECTURE.md` — 2 references to `agent_context/`
- `README.md` — 1 reference to `config/agent_context/`
- `.agent/knowledge/README.md` — 2 references to `agent_context/`
- `CLAUDE.md` — reference to `.agent/project_knowledge/` (path unchanged, but description update)
- `.agent/AGENT_ONBOARDING.md` — 1 reference

### 4. generate_knowledge.sh — references agent_context/
Being removed in #274. For now, update the comment to reference new path.
Or remove it in this PR since #274 hasn't started yet.

### Skip
- `RESEARCH_ISSUE-249.md` — historical research, not normative
- `AGENTS.md` — already references `.agents/README.md` correctly
- `.github/PULL_REQUEST_TEMPLATE.md` — already references `.agents/README.md`

## Principles Self-Check

| Principle | Status |
|---|---|
| A change includes its consequences | All 5+ doc files updated in same PR |
| Only what's needed | Clean cutover, no compatibility shim |
| Workspace vs. project separation | Convention defined in workspace, adopted by projects |

## Success Criteria

- `setup.sh` looks for `.agents/workspace-context/` (not `config/agent_context/`)
- All documentation references updated
- Template shows expanded `.agents/` structure
- No references to `agent_context/` remain in normative docs
