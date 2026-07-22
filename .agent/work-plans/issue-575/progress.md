---
issue: 575
---

# Issue #575 — ADR-0018: local-first CI verification (Phase 1 governance of #572) + AGENTS.md updates

## Integrated Review
**Status**: complete
**When**: 2026-07-22 13:55 -04:00
**By**: Claude Code Agent (Claude Fable 5)

**PR**: #576 at `e4235bb`
**Sources**: 2 (Copilot R1 @ `e4235bb`, CI rollup @ `e4235bb`)
**Cross-source confirmations**: 0
**CI**: all-pass

### Findings
- [ ] (trivial, Copilot) `ci_local.sh <repo>` placeholder ambiguous — the script takes a filesystem path; use `<project_repo_path>` per its usage header (Documentation Accuracy) — `AGENTS.md:404`

### False positives
- (Copilot) "Hosted Actions … remains" flagged as plural-subject agreement error — "Actions" is the GitHub Actions product name, which takes singular agreement (GitHub's own docs: "GitHub Actions is…"); optional rephrase to "Hosted CI" noted as a free clarity win.
