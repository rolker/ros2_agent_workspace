# Plan: Create shared principles review guide

**Issue**: #272
**Status**: Plan — awaiting review

## Problem Analysis

The workspace has 10 guiding principles and 6 ADRs but no systematic reference
for evaluating work against them. Lifecycle skills (triage-issue, review-pr,
plan-task, etc.) need a shared, framework-agnostic guide they can read to
produce consistent evaluations.

The triage comment on issue #272 flagged size risk: a per-principle prose
checklist could easily hit 300+ lines, defeating the "Only what's needed"
principle it's supposed to enforce.

## Proposed Approach

**Table-driven, not prose-heavy.** Each principle and ADR gets a concise row in
a lookup table rather than a full checklist section. Target: under 150 lines.

## File Structure

```
.agent/knowledge/principles_review_guide.md

  ## How to Use This Guide
  (3-4 lines: purpose, when to read it)

  ## Principle Quick Reference
  | Principle | Looks like | Watch for |
  (10 rows — one per principle)

  ## ADR Applicability
  | ADR | Triggered when | Key requirement |
  (6 rows — one per ADR)

  ## Consequences Map
  | If you change... | Also update... |
  (common change-consequence pairs)

  ## Governance Layering
  (~10 lines: workspace vs project scope, precedence rules)
```

## Implementation Tasks

- [ ] Write `principles_review_guide.md` using table format
- [ ] Update `.agent/knowledge/README.md` to index the new file
- [ ] Verify principle names match merged #271 text
- [ ] Self-review: does the guide pass its own principles check?

## Design Decisions

**Tables over prose**: Concise, scannable, context-efficient. Skills can
reference specific rows without loading paragraphs of explanation. More
detail can be added later if skills need it — start minimal.

**Single file**: All evaluation criteria in one place. No need to scatter
across multiple files when the content fits in ~120 lines.

**Framework-agnostic markdown**: Any agent framework can read this file.
No Claude-specific features in the content.

## Success Criteria

- File exists at `.agent/knowledge/principles_review_guide.md`
- Covers all 10 principles and all 6 ADRs
- Under 150 lines
- `.agent/knowledge/README.md` updated with index entry
- Principle names match current `docs/PRINCIPLES.md`
