---
issue: 660
---

# Issue #660 — Port the external cross-model reviewer (Gemini via agy + Codex) from agent_workspace into review-code

## Issue Review
**Status**: complete
**When**: 2026-09-23 14:32 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)

**Issue**: #660
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Actions
- [ ] Pick a fresh ADR number for the parallel-sync-only decision (next free is 0020 as of this review) — `0015` is already taken here by `docs/decisions/0015-dispatch-handoff-context-contract.md` (unrelated topic); do not port the source's ADR file verbatim under its original number.
- [ ] Add `AGENTS.md` Script Reference rows for `cross_model_review.sh`, `_agy_review.sh`, `_cli_review.sh`, `_resolve_work_plans_dir.sh`, `_resolve_default_branch.sh`.
- [ ] Update `.agent/knowledge/inspiration_agent_workspace_digest.md`'s "Partially adopted" note (references #206/#212) once this lands.
- [ ] Treat `_resolve_work_plans_dir.sh` / `_resolve_default_branch.sh` as net-new shared helpers in this workspace (no existing equivalent — `check_branch_updates.sh` resolves the default branch inline today); scope the port to what the cross-model scripts need, and do not fold in a repo-wide refactor of existing inline resolvers into the new helpers.
- [ ] Re-verify the port table's script contents against `~/agent_workspace` `main` at plan/implementation time rather than this review's snapshot (`48b0d82`, "Merge pull request #341 from rolker/feature/issue-320") — the issue's own follow-up comment notes the source was still moving as of 2026-09-22.
- [ ] Decide design point 3 (new step 5g vs. replacing 5e's body) — recommend **5g** (new step): 5e's Copilot-specific machinery (skip-reasons, untrusted-PR safety gate, `--allow-all-tools` security note) doesn't generalize cleanly to a `--agents gemini,codex,copilot` call, and keeping them separate avoids destabilizing the working Copilot path.
