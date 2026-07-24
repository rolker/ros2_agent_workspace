---
issue: 586
---

# Issue #586 — docs: promote agent-memory process rules into AGENTS.md, WORKFORCE_PROTOCOL, and knowledge docs

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-24 15:42 -04:00
**By**: Claude Code Agent (Claude Fable 5)
**Verdict**: approved

**Branch**: feature/issue-586 at `70895a3`
**Mode**: pre-push
**Depth**: Standard (instruction files touched; 10 files +~300)
**Must-fix**: 0 | **Suggestions**: 2 (both fixed pre-push)
**Round**: 1 | **Ship**: recommended — two disjoint-lens adversarial passes; cross-pass-confirmed ADR-0018 wording collision and deployment-mode over-breadth both fixed; anchors/commands/links verified (incl. merge_pr.sh --issue behavior)

### Findings
- [x] (suggestion, cross-pass confirmed) 'never merge while pending' collided with ADR-0018 local-attestation merges — scoped per-repo-type — `AGENTS.md` Merging
- [x] (suggestion) field-host hard stop broad enough to forbid deployment-mode in-session mitigations — narrowed to host sysadmin — `AGENTS.md` Never
- [ ] (nit, user's call) timestamp/attribution rules now stated in AGENTS.md + deployment_mode.md + new knowledge docs — consistent today; consider one canonical home + cross-refs
