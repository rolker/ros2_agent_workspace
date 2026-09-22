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

**Lifecycle position**: Utility/periodic — run periodically to check workspace
governance health. Not tied to the per-issue lifecycle.

Periodic governance health check — the "garbage collection" pattern. Verifies
that the workspace follows its own rules. Reports findings in the conversation.

**Not the same as `validate_workspace.py`** — that script checks structural
config (repos match `.repos` files, layers are set up correctly). This skill
checks governance: are rules enforced? Are docs current? Are ADRs still
accurate?

## Checklist

**Coverage is data, and every section reports it.** How deep this audit goes
has varied run to run — one run read all 10 principles and found four
enforcement gaps; the next spot-checked two and reported "no gaps found".
A consumer that diffs two such reports (`janitor-sweep`'s run-over-run diff,
[#651](https://github.com/rolker/ros2_agent_workspace/issues/651)) cannot
tell "fixed" from "not looked at" unless each section states what it
examined out of what exists. So each of the seven sections below ends with a
**coverage line**, and the report collects them in a `### Coverage` table
(§ Report Format) that a consumer reads as data:

- **Sections 1–2 may sample.** They are the only two whose input set is
  large enough for a full pass to be a real cost (10 principles, 19 ADRs at
  the time of writing — count what is there, never assume these numbers).
  A sampled section reports `<kind>: X of Y examined` **and names the X
  items** it examined — a bare fraction cannot be matched against a prior
  run's findings, so a nameless sample is not coverage data. `X == Y` is a
  full pass and needs no item list.
- **Sections 3–7 never sample.** Each iterates a small, fully enumerable
  set (`.agent/scripts/`, `.agent/templates/`, the consequences map's own
  item list, a fixed three-file adapter list, `worktree_list.sh`'s output),
  so covering all of it costs no more than sampling would. They report
  `<kind>: all N`, with N counted from the input — `all` with an N smaller
  than the directory holds is itself a finding, which is what makes an
  incomplete pass visible rather than assumed.
- **A section that could not run** (an input missing or unreadable) reports
  `<kind>: 0 of Y — SKIPPED(<reason>)`, never `OK`. The coverage line is
  what proves incompleteness; the narrative alone does not.

This makes coverage *reportable*, not self-verifying: nothing stops a run
from writing `principles: 10 of 10` without reading all ten. That is the
same trust boundary every judgement-pass skill in this workspace has; the
gain is that the claim is now explicit and diffable rather than implicit in
which findings happen to appear.

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

**Coverage line**: `principles: X of Y examined`, Y counted from
`docs/PRINCIPLES.md`; when X < Y, name the X principles examined.

### 2. ADR accuracy

For each ADR in `docs/decisions/`:

- Read the ADR's decision and consequences
- Verify the decision is still implemented as described
- Check that consequences listed have been addressed
- Flag any ADR whose status says "Accepted" but whose implementation has
  drifted

**Coverage line**: `ADRs: X of Y examined`, Y counted from
`docs/decisions/`; when X < Y, name the X ADRs examined (by number).

### 3. Script reference table

Compare the script reference table in `AGENTS.md` against actual scripts
in `.agent/scripts/`:

- Scripts listed in the table that don't exist → **stale reference**
- Scripts that exist but aren't in the table → **undocumented**
- Descriptions that don't match the script's actual behavior → **inaccurate**

**Coverage line**: `scripts: all N`, N counted from `.agent/scripts/`
(and the `AGENTS.md` table rows compared against it).

### 4. Template validity

For each template in `.agent/templates/`:

- Is it referenced somewhere (AGENTS.md, knowledge docs, skills)?
- Does it reference files that exist?
- Is it consistent with current conventions (e.g., `.agents/` not
  `agent_context/`)?

**Coverage line**: `templates: all N`, N counted from `.agent/templates/`.

### 5. Consequences map currency

Read the consequences map in `.agent/knowledge/principles_review_guide.md`:

- Do the "If you change..." items still exist at the listed paths?
- Are there new high-impact files not covered by the map?

**Coverage line**: `consequences-map items: all N`, N counted from the
map's own "If you change..." rows.

### 6. Instruction file consistency

Check that framework adapter files are consistent with `AGENTS.md`:

- `.github/copilot-instructions.md`
- `.agent/instructions/gemini-cli.instructions.md`
- `CLAUDE.md`

Flag any rules in AGENTS.md that should be reflected in adapters but aren't.

**Coverage line**: `adapters: all 3` (the three files above; a missing
adapter is a finding, and the line then reads `2 of 3 — <file> missing`).

### 7. Stale worktrees

```bash
.agent/scripts/worktree_list.sh
```

List any worktrees that appear abandoned (no recent commits, merged PRs).

**Coverage line**: `worktrees: all N`, N = the rows `worktree_list.sh`
printed.

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

### Coverage

| Section | Coverage | Items examined (when sampled) |
|---|---|---|
| 1. Principles enforcement | X of Y examined | <names, or "—" for a full pass> |
| 2. ADR accuracy | X of Y examined | <ADR numbers, or "—" for a full pass> |
| 3. Script references | all N | — |
| 4. Templates | all N | — |
| 5. Consequences map | all N | — |
| 6. Instruction consistency | all 3 | — |
| 7. Stale worktrees | all N | — |

<!-- One row per section, always all seven, in this order — a consumer keys
     on the section name. A section that could not run keeps its row:
     `0 of Y — SKIPPED(<reason>)`. The Summary table above says what was
     FOUND; this table says what was LOOKED AT. They are different claims and
     a reader needs both: "0 gaps" over "2 of 10 examined" is not a clean
     bill of health. -->

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
