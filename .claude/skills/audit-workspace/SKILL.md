---
name: audit-workspace
description: Check the workspace against its own standards. Find rules without enforcement, drifted ADRs, stale docs, and missing consequences. Run periodically.
---

# Audit Workspace

## Usage

```
/audit-workspace
```

## Overview

Periodic governance health check — the "garbage collection" pattern. Verifies
that the workspace follows its own rules. Reports findings in the conversation.

**Not the same as `validate_workspace.py`** — that script checks structural
config (repos match `.repos` files, layers are set up correctly). This skill
checks governance: are rules enforced? Are docs current? Are ADRs still
accurate?

## Checklist

### 1. Principles enforcement

For each principle in `docs/PRINCIPLES.md`, check whether an enforcement
mechanism exists:

| Principle | Enforcement | Status |
|---|---|---|
| Human control and transparency | PR template consequence checklist | OK / Missing |
| Enforcement over documentation | Pre-commit hooks, CI checks | OK / Missing |
| ... | ... | ... |

Flag principles that exist only as documentation with no hook, CI check,
or guardrail.

### 2. ADR accuracy

For each ADR in `docs/decisions/`:

- Read the ADR's decision and consequences
- Verify the decision is still implemented as described
- Check that consequences listed have been addressed
- Flag any ADR whose status says "Accepted" but whose implementation has
  drifted

### 3. Script reference table

Compare the script reference table in `AGENTS.md` against actual scripts
in `.agent/scripts/`:

- Scripts listed in the table that don't exist → **stale reference**
- Scripts that exist but aren't in the table → **undocumented**
- Descriptions that don't match the script's actual behavior → **inaccurate**

### 4. Template validity

For each template in `.agent/templates/`:

- Is it referenced somewhere (AGENTS.md, knowledge docs, skills)?
- Does it reference files that exist?
- Is it consistent with current conventions (e.g., `.agents/` not
  `agent_context/`)?

### 5. Consequences map currency

Read the consequences map in `.agent/knowledge/principles_review_guide.md`:

- Do the "If you change..." items still exist at the listed paths?
- Are there new high-impact files not covered by the map?

### 6. Instruction file consistency

Check that framework adapter files are consistent with `AGENTS.md`:

- `.github/copilot-instructions.md`
- `.agent/instructions/gemini-cli.instructions.md`
- `CLAUDE.md`

Flag any rules in AGENTS.md that should be reflected in adapters but aren't.

### 7. Stale worktrees

```bash
.agent/scripts/worktree_list.sh
```

List any worktrees that appear abandoned (no recent commits, merged PRs).

## Report Format

```markdown
## Workspace Audit

**Date**: YYYY-MM-DD

### Summary

| Category | Findings |
|---|---|
| Principles enforcement | X of Y enforced |
| ADR accuracy | X of Y current |
| Script references | X stale, Y undocumented |
| Templates | X issues |
| Consequences map | X gaps |
| Instruction consistency | X issues |
| Stale worktrees | X found |

### Findings

#### <Category>

| Item | Status | Details |
|---|---|---|
| ... | OK / Issue | ... |

### Recommended Actions

- [ ] <specific action items>
```

## Guidelines

- **Report, don't fix** — this skill identifies issues. Fixing them should
  be separate issues with their own worktrees.
- **Be specific** — "AGENTS.md script table lists `generate_knowledge.sh`
  but it was removed in #274" is actionable. "Some references may be stale"
  is not.
- **Don't nitpick** — focus on things that would confuse agents or humans.
  Minor formatting inconsistencies aren't worth flagging.
- **Run periodically** — after a batch of PRs merge, or when starting a
  new work cycle. Not after every commit.
