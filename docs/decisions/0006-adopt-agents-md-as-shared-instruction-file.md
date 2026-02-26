# ADR-0006: Adopt AGENTS.md as shared instruction file

## Status

Proposed

## Context

The workspace maintains three near-identical instruction files for different AI agent
frameworks: `CLAUDE.md` (216 lines), `.github/copilot-instructions.md` (183 lines), and
`.agent/instructions/gemini-cli.instructions.md` (183 lines). These files share ~95% of
their content — git rules, worktree workflow, issue-first policy, build commands,
documentation accuracy rules, and more.

This duplication creates two problems:

1. **Maintenance burden**: Every rule change requires editing 3+ files. Contributors
   forget to update all copies, causing drift (e.g., the Copilot and Gemini files
   included project-specific layer names, violating ADR-0003).

2. **Contradiction risk**: When files drift, agents following different instruction files
   may behave inconsistently on the same rules.

Meanwhile, `AGENTS.md` has emerged as an industry-standard convention for providing
instructions to AI agents, supported by multiple frameworks and recognized by the
broader AI-assisted development community.

ADR-0005 established that framework-native mechanisms (including instruction files) are
"not load-bearing" — the backbone should be framework-agnostic. The shared rules in
our instruction files are inherently framework-agnostic and should be expressed once.

## Decision

Adopt a two-tier instruction file structure:

1. **`AGENTS.md`** (repo root): Contains all shared workspace rules that apply to every
   agent regardless of framework. Rules are organized using a three-tier boundary system
   (Always / Ask First / Never) for clarity.

2. **Framework adapters** (thin files, ~40-80 lines each): Each framework's instruction
   file becomes a compact adapter that references `AGENTS.md` and adds only
   framework-specific configuration (identity, env setup, framework-native features).

The adapters are:
- `CLAUDE.md` — imports `AGENTS.md` via `@AGENTS.md` syntax, adds Claude-specific
  setup and slash command notes
- `.github/copilot-instructions.md` — directs Copilot to read `AGENTS.md`, adds
  Copilot-specific notes
- `.agent/instructions/gemini-cli.instructions.md` — directs Gemini to read `AGENTS.md`,
  adds Gemini-specific notes

### Three-Tier Boundary System

Rules in `AGENTS.md` are categorized by autonomy level:

- **Always** (proceed autonomously): Standard workflow rules agents should follow
  without asking — worktree isolation, pre-commit hooks, AI signatures, etc.
- **Ask First** (get human approval): Actions with broad impact — modifying instruction
  files, changing CI configuration, adding/removing workspace layers.
- **Never** (hard stops): Actions that are blocked or prohibited — committing to main,
  using `git checkout`, skipping hooks, documenting from assumptions.

This system makes agent autonomy boundaries explicit rather than implicit, reducing
both over-cautious behavior and unauthorized actions.

## Consequences

### Positive

- **Single source of truth**: Shared rules live in one file. Changes propagate
  automatically to all frameworks.
- **ADR-0003 compliance**: The shared file uses generic architecture descriptions
  (no project-specific layer names), fixing the existing violation.
- **Reduced maintenance**: Adding a new rule means editing one file instead of three.
- **Clearer adapters**: Framework files become short and scannable — agents quickly
  find their framework-specific setup without wading through shared rules.
- **Industry alignment**: `AGENTS.md` is a recognized convention, making the workspace
  more accessible to new frameworks and contributors.

### Negative

- **Import mechanism varies**: Claude Code uses `@AGENTS.md` for auto-import; other
  frameworks require a prose instruction ("read and follow AGENTS.md"). This is
  inherent to the multi-framework landscape.
- **Two files to read**: Agents now read AGENTS.md + their adapter, instead of one
  self-contained file. The total content is smaller (no duplication), but the reading
  path has one extra hop.

### Neutral

- `.agent/AI_RULES.md` is converted to a redirect pointing to `AGENTS.md`. It can
  be removed entirely in a future cleanup cycle.

## References

- ADR-0003: Workspace infrastructure is project-agnostic
- ADR-0004: Enforcement hierarchy for agent compliance
- ADR-0005: Layered enforcement — framework-agnostic backbone
- [AGENTS.md convention](https://github.com/anthropics/agents-md)
