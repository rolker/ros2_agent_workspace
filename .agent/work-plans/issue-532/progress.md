---
issue: 532
---

# Issue #532 — container-auth token paths + `--check` preflight

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-20 21:29 +0000
**By**: Claude Code Agent (Claude Opus 4.8)
**Verdict**: changes-requested

**Branch**: feature/issue-532 at `2ac44d7`
**Mode**: pre-push
**Depth**: Deep (reason: auth/credential-token handling; AGENTS.md governance trigger)
**Must-fix**: 1 | **Suggestions**: 3

### Findings
- [ ] (must-fix) Preflight GH-token env check keys on `$GH_TOKEN`, but `docker_run_agent.sh` consumes `$AGENT_GH_TOKEN` (GH_TOKEN is only the container-facing export) — false negative on documented override, false positive on `GH_TOKEN`; check `$AGENT_GH_TOKEN` instead — `.agent/scripts/dispatch_subagent.sh:133`
- [ ] (suggestion) `entry_type_model` duplicates `skill_model` tiering with no shared source; test asserts parity for only 2 mappings — drift could pass CI — `.agent/scripts/dispatch_subagent.sh:86`
- [ ] (suggestion) `model_display` passthrough lets a pinned `--model <id>` embed a version in the handoff, contradicting the "don't invent a version" instruction — `.agent/scripts/dispatch_subagent.sh:98`
- [ ] (suggestion) `exit $?` after bare `preflight_check` is partially dead code under `set -e`; rewrite as explicit `if preflight_check` for clarity — `.agent/scripts/dispatch_subagent.sh:172`
