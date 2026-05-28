---
issue: 499
---

# Issue #499 — Draft ADR-0014: deployment mode (behavioral autonomy mode + lifecycle tooling)

## Integrated Review
**Status**: complete
**When**: 2026-05-28 09:25 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #500 at `05a4a42`
**Sources**: 1 (Copilot R2 @ `05a4a42`) — no prior `progress.md`, no human reviews, no conversation comments
**Cross-source confirmations**: 0
**CI**: all-pass (Lint, Validate Documentation, commit identity)

### Findings
- [ ] (must-fix, Copilot R2) PR-body description still references marker-based activation (*"the activation marker + lifecycle skills may land before it's marked Accepted"* and *"activation via an operator-set marker (works dev + field)"*); contradicts the amended ADR (no marker, per-session via `/start-deployment`) — PR body
- [ ] (suggestion, Copilot R2) `Proposed (Draft)` status line is novel vocabulary; other Proposed ADRs (0005, 0008, 0009) and the template (`adr_template.md:67-69`) use plain `Proposed`. Drop `(Draft)`; trailing sentence already explains in-flight context — `docs/decisions/0014-deployment-mode.md:5`

### False positives
None.
