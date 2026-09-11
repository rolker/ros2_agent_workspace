---
issue: 569
---

# Issue #569 — Scheduled janitor for staleness/drift detectors

## Issue Review
**Status**: complete
**When**: 2026-09-11 12:40 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #569
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped (as narrowed by the 2026-09-11 scope-decision comment)

### Actions
- [ ] Settle in plan-task: `audit-project`'s repo-location step (`find layers/main/*/src/<repo-name>`) assumes a local layer checkout, which conflicts with the issue's "must not assume `layers/` exists; it clones what it needs" constraint. Decide whether `audit-project` itself gains a clone-fallback (benefits all callers) or the janitor shallow-clones into a scratch dir before invoking it. `issue-triage` needs no such fallback — it already enumerates repos from tracked `.repos` files via `list_overlay_repos.py`, not the checked-out tree.
- [ ] Include in the same PR: add the new skill to the cross-framework skill lists the Consequences Map requires — `.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md` — plus `.agent/knowledge/skill_workflows.md`, which also enumerates `audit-workspace`/`audit-project`/`issue-triage`/`research` and would go stale otherwise.
- [ ] Settle in plan-task: the durable-output design. The three existing periodic skills it chains (`audit-workspace`, `audit-project`, `issue-triage`) report to the conversation only — fine for manual trigger, since a human reads the same session. The janitor is explicitly meant to run unattended, so "one rolling GitHub issue, updated in place" needs to be the actual persisted record, not just a conversational summary that then gets posted; confirm the plan's write step treats the GitHub issue update as the durable output (mirroring `review-issue`'s canonical-local-write-then-best-effort-post pattern) rather than assuming GitHub is always reachable — the plan should say explicitly what happens when the API call to update the issue fails, per the false-green lesson from #609 the issue itself already cites for skipped/failed checks.
- [ ] Forward-looking, non-blocking: this slice explicitly defers the trigger mechanism, but note for that follow-up that updating the rolling issue is itself a GitHub *write*. If the eventual trigger is a container dispatch, ADR-0015/ADR-0019 already establish the pattern to reuse (host publishes; container produces + best-effort posts) — worth citing when that decision is made rather than re-deriving it.

## Notes (not blocking, recorded for the record)

**Scope / right repo**: Yes to both. The proposed skill (chain existing detectors, write one rolling report, no PRs opened) is workspace-generic tooling consistent with ADR-0003 and fits the existing `.claude/skills/{audit-workspace,audit-project,issue-triage,research}` pattern of "Utility/periodic, not tied to the per-issue lifecycle." The 2026-09-11 scope comment's split (sweep now, trigger later) is a good incremental slice — it removes the prior #564 sequencing dependency (correctly noted as lifted: the sweep now *produces* #564's enforcement inventory rather than depending on it) and keeps this PR reviewable on its own.

**Dependencies**: None blocking. All four detectors it chains already exist and are independently invocable. The research-digest half of the motivation is already satisfied — `.agent/knowledge/research_digest.md` was refreshed in PR #624 (merged), which added the "Scheduled & Background Maintenance Agents" entry that itself cites #569 and independently confirms two of the issue's design choices: single rolling report over per-finding issues, and (for the deferred trigger question) that Routines don't reach the `layers/` tree or gitcloud-origin field repos, so a cron+container path remains the fit for anything touching layers.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Report-only, no PRs opened by the sweep; findings surfaced for the operator to triage into issues manually — matches the operator's explicit no-issue-spam preference. |
| Enforcement over documentation | Watch | The mechanism stays manual-trigger this slice by design (trigger deferred); acceptable as an intentional, recorded sequencing choice, not a gap in this issue. |
| Capture decisions, not just implementations | OK | The scope split is recorded as an issue comment (2026-09-11); no ADR needed yet since no lasting architecture decision is made this slice (the trigger/architecture choice is explicitly deferred). |
| A change includes its consequences | Action needed | New-skill cross-adapter list updates (see Actions above) must land in the same PR — this is a standing Consequences Map item ("Workflow skill list (add/remove a skill)") this issue doesn't currently mention. |
| Only what's needed | OK | Chains existing detectors rather than rebuilding any of them; no new infra proposed beyond the report step. |
| Improve incrementally | OK | Explicitly sliced from the trigger/scheduling decision; single reviewable PR. |
| Test what breaks | Watch | The "state what was skipped/failed rather than report clean" constraint (carried over from #609) needs an actual degraded-path test (GitHub API unreachable, a repo not onboarded, `layers/` absent), not just the happy path. |
| Workspace vs. project separation | OK | Generic skill; no project-specific content baked in. Repo rotation should stay data-driven (reads onboarded-repo list), not hardcoded. |
| Workspace improvements cascade to projects | OK | Applies uniformly across onboarded repos via the existing `audit-project` rotation concept. |
| Primary framework first, portability where free | OK | Authored as a Claude Code skill, consistent with the other three detectors it chains. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0003 (project-agnostic workspace) | Yes | Satisfied — sweep and report logic are generic; per-repo rotation is config/data, not hardcoded project content. |
| ADR-0013 (progress.md entry vocabulary) | Watch, not blocking | This skill is Utility/periodic like its three siblings, none of which write `progress.md` — consistent, not a violation. But flag explicitly in the plan that the *rolling GitHub issue* is the durable record for this one (unlike its siblings, meant to run unattended), so a failed post isn't silently indistinguishable from "nothing to report." |
| ADR-0015 / ADR-0019 (dispatch handoff contract / container model) | Not triggered this slice | The issue explicitly defers the trigger mechanism (cloud Routine vs. cron container vs. anacron), which is where these ADRs would apply (GitHub write auth to update the rolling issue, container-vs-in-process rationale). Recorded as a forward pointer only. |

### Consequences

- Skill list additions owed in the same PR: `.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md`, `.agent/knowledge/skill_workflows.md` (all four currently enumerate `audit-workspace`/`audit-project`/`issue-triage`/`research`; confirmed via grep).
- `audit-project`'s local-checkout assumption (`find layers/main/*/src/<repo-name>`) is a real integration gap against the issue's own "clones what it needs" constraint — needs a design decision in plan-task, not just wiring the skill call.

### Recommendations

- Reuse `review-issue`'s two-step output pattern (canonical local artifact first, best-effort GitHub post second) for the rolling-issue update, so a GitHub outage degrades to "report written, not yet posted" rather than silently reporting nothing.
- When the trigger-mechanism follow-up issue is filed, point it at ADR-0015/ADR-0019 up front rather than re-deriving the container-vs-write-auth reasoning from scratch.

## Plan Authored
**Status**: complete
**When**: 2026-09-11 12:46 -04:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-569/plan.md` at `8afd8b5`
**Branch**: feature/issue-569 at `8afd8b5`
**Phases**: single

Four items the `## Issue Review` entry left for this phase are decided in the plan
as **[D1]**–**[D4]**: (D1) the layer-checkout assumption is fixed at its source — a new
`.agent/scripts/resolve_repo_checkout.sh` (layer checkout, else shallow clone into
`.agent/scratchpad/`) that `audit-project` step 1 calls, with layer-dependent checks
reporting SKIPPED in clone mode, rather than a janitor-local workaround; (D2) the four
skill-list sites land in the same PR; (D3) durable output is a local canonical write
first, then the rolling issue (body updated in place + sweep posted as a comment), with
a named POST FAILED path and a next-run backlog post so an unattended failed publish
cannot vanish, plus a per-check OK/FINDINGS/SKIPPED/FAILED contract so a report can only
read clean when all four checks completed; (D4) the deferred trigger decision is pointed
at ADR-0015/ADR-0019 in the skill's deferred-trigger section.

### Open questions
- [ ] The PR edits four instruction files (`AGENTS.md` + three adapters) — Ask First; confirm blanket approval or review at PR time.
- [ ] Should the consequences-map row on durable-findings skills gain a clause for non-issue-scoped skills? Out of this PR unless wanted.
- [ ] May the sweep create the rolling issue on first run (title `Janitor sweep report (rolling)`, no label), or will the operator open it by hand?
