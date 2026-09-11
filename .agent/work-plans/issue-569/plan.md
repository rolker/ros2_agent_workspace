# Plan: Scheduled janitor for staleness/drift detectors — slice one, the sweep

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/569

## Context

Four staleness/drift detectors exist and are all manual-trigger: `audit-workspace`,
`audit-project`, `issue-triage`, and the research-digest freshness nag in
`.agent/knowledge/research_digest.md`. Nothing chains them and nothing persists their
findings — each reports only into the conversation that ran it.

Per the 2026-09-11 scope-decision comment on the issue, this slice delivers **the sweep
only**. The trigger (Routine / anacron / Actions cron) is explicitly deferred, and the
former #564 sequencing gate is lifted — the first sweep *produces* #564's enforcement
inventory. The sweep opens no PRs and files no per-finding issues: it maintains **one
rolling report issue**. It must not assume `layers/` exists.

Four items were left for this plan by the `## Issue Review` entry; each is decided below
and marked **[D1]**–**[D4]**.

## Approach

1. **[D1] Fix the repo-location gap at its source, not in the janitor.** `audit-project`
   step 1 resolves a repo with `find layers/main/*/src/<repo>` — a hard dependency on a
   local layer checkout. Add `.agent/scripts/resolve_repo_checkout.sh <repo-name>`, which
   prints `<path>\t<layer|clone>` and exits non-zero with a reason on stderr when it
   cannot produce a checkout. Resolution order: (a) an existing
   `layers/main/*/src/<repo>` checkout; (b) otherwise a shallow clone
   (`--depth 1 --filter=blob:none`) of the URL from `list_overlay_repos.py` into
   `.agent/scratchpad/janitor-repos/<repo>` (gitignored), refreshed if already present.
   Rewrite `audit-project` step 1 to call it and to state that in `clone` mode the
   layer-dependent checks (step 5 `colcon test`, step 7 "correct layer") report
   **SKIPPED (no layer checkout)**, never OK. Rationale: the janitor is not the only
   caller that will run outside a full layer tree, and a janitor-local workaround would
   leave the defect in place for every other caller.
2. **Write the janitor skill** at `.claude/skills/janitor-sweep/SKILL.md` (Utility/periodic,
   matching its three siblings). Usage: `/janitor-sweep [--repos <a,b,c>] [--dry-run]`.
   Per-check contract: every check ends as **OK / FINDINGS / SKIPPED(reason) /
   FAILED(reason)**; a check that could not run is never rendered as a pass.
3. **Rotation, stateless by construction.** Candidate set = overlay repos from
   `list_overlay_repos.py` filtered to (i) `github.com` origins — a non-GitHub origin
   (gitcloud) is listed as **excluded: not reachable from a generic runner**, not silently
   dropped — and (ii) repos onboarded far enough to audit, probed remotely with
   `gh api repos/<owner>/<repo>/contents/AGENTS.md`. Sort the survivors by name, chunk by
   3, and pick chunk `ISO-week mod chunk-count`. Deterministic, needs no persisted cursor,
   and cycles all repos in `ceil(N/3)` weeks. `--repos` overrides for a hand-run. The
   report always lists the full candidate set with each repo's in/out status and reason.
4. **Chain the four checks**: `audit-workspace` (full), `audit-project` on the rotation
   chunk, `issue-triage --stale-days 90`, and a digest-freshness check that reads the
   `<!-- Last updated: YYYY-MM-DD -->` header of `.agent/knowledge/research_digest.md`
   against the 30/90-day thresholds the file itself declares.
5. **[D3] Durable output: local canonical write, then the rolling issue, with an explicit
   failure path.** Mirror `review-issue`'s pattern. (a) Always write the full report to
   `.agent/scratchpad/janitor/<YYYY-MM-DD>-sweep.md` first — this write does not depend on
   network or auth. (b) Locate the rolling issue by **exact** title
   `Janitor sweep report (rolling)` on `rolker/ros2_agent_workspace`
   (`gh issue list --state open --search '... in:title'`, exact-match filtered in `jq`);
   zero matches → create it once; **two or more matches → FAILED(ambiguous rolling issue),
   never guess**. (c) Update the issue *body* to the current snapshot and post the sweep as
   a *comment*, so "updated in place" and an audit trail both hold. (d) If (b) or (c)
   fails, the run's headline is `POST FAILED — report written to <path>, not published`,
   the local file is left in place, and **the next run detects any unpublished report in
   that directory and posts it before its own** — an unattended failed post must not
   vanish just because no human read the session. No findings are ever converted into
   individual issues; the operator triages from the report.
6. **Report shape.** The report leads with a coverage header — `Checks: X of 4 completed,
   Y skipped, Z failed` — followed by the per-check status table, then findings grouped by
   check, then `Repos not audited this run (reason)`. The words "clean" / "no findings"
   may only appear when all four checks completed. This is the #609 false-green rule
   applied to a report.
7. **[D2] Land the cross-framework list updates in the same PR** (Consequences Map,
   "Workflow skill list (add/remove a skill)"): `.github/copilot-instructions.md`,
   `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md` (all
   three carry the same `Available workflow skills:` enumeration), plus the Utility-skills
   table in `.agent/knowledge/skill_workflows.md`, plus the `AGENTS.md` script-reference
   row for the new script.
8. **[D4] Forward-looking note, non-blocking.** In the skill's "Deferred: trigger" section,
   record that updating the rolling issue is a GitHub **write**, so whichever trigger is
   chosen must satisfy [ADR-0015](../../../docs/decisions/0015-dispatch-handoff-context-contract.md)
   (container produces, host publishes — a dispatched container has no GitHub write auth)
   and [ADR-0019](../../../docs/decisions/0019-what-contains-a-dispatched-agent.md)
   (what containment does and does not buy). The later decision cites these rather than
   re-deriving them. No ADR is written in this slice — no lasting architecture decision is
   made here.
9. **Test** `resolve_repo_checkout.sh` in `.agent/scripts/tests/test_resolve_repo_checkout.sh`
   (registered in `run_script_tests.sh`): prefers an existing layer checkout; clones when
   `layers/` is absent; **exits non-zero with a reason when the clone fails** (the
   false-green path); rejects a repo absent from every manifest.

## Files to Change

| File | Change |
|------|--------|
| `.claude/skills/janitor-sweep/SKILL.md` | New — the sweep procedure, per-check status contract, report format, rolling-issue write path, deferred-trigger note |
| `.agent/scripts/resolve_repo_checkout.sh` | New — layer-checkout-or-shallow-clone resolver; prints `path\tmode`; fails loud |
| `.agent/scripts/tests/test_resolve_repo_checkout.sh` | New — four cases above |
| `.agent/scripts/tests/run_script_tests.sh` | Register the new test |
| `.claude/skills/audit-project/SKILL.md` | Step 1 uses the resolver; steps 5 and 7 report SKIPPED in `clone` mode |
| `AGENTS.md` | Script-reference row for `resolve_repo_checkout.sh` (**instruction file — Ask First**) |
| `.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md` | Add `janitor-sweep` to the skill enumeration (**instruction files — Ask First**) |
| `.agent/knowledge/skill_workflows.md` | Add `janitor-sweep` to the Utility-skills table |
| `.gitignore` | Ignore `.agent/scratchpad/` already covers the report + clone cache — verify, no change expected |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Report-only. No PRs, no per-finding issues, exactly one rolling issue; the operator decides what becomes work. |
| Enforcement over documentation | The sweep is still hand-run this slice — a recorded sequencing choice, not a gap. The one mechanically enforceable piece (repo resolution) gets a script and a test. |
| A change includes its consequences | Step 7 lands all four skill-list sites plus the script table in this PR. |
| Test what breaks | The resolver is tested, including its failure path. The report's degraded behaviour is procedure, not code — stated as explicit report rows rather than claimed as tested. |
| Only what's needed | Chains existing detectors; adds one 40-line script. No scheduler, no new infra. |
| Workspace vs. project separation | Rotation is derived from `.repos` manifests and a remote AGENTS.md probe — no repo names hardcoded (ADR-0003). |
| Improve incrementally | Sweep now, trigger later, as the operator scoped it. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0003 (project-agnostic workspace) | Yes | Rotation and checks are data-driven; nothing project-specific is baked into the skill. |
| ADR-0013 (progress.md vocabulary) | Considered, not triggered | `progress.md` is per-issue-keyed; the janitor is not issue-scoped. Its durable record is the rolling issue, consistent with its three periodic siblings, none of which write `progress.md`. Stated explicitly in the skill so a failed post is distinguishable from "nothing to report". |
| ADR-0015 / ADR-0019 (dispatch handoff / containment) | Not this slice | Cited as the forward pointer for the deferred trigger decision (step 8). |
| ADR-0017 (AGENTS.md in project repos) | Indirectly | The remote AGENTS.md probe reuses ADR-0017's marker as the onboarding signal; `audit-project`'s currency check is unchanged. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Workflow skill list (add a skill) | Three adapter files + `skill_workflows.md` | Yes — step 7 |
| A script in `.agent/scripts/` | `AGENTS.md` script table | Yes — step 7 |
| A framework skill | That framework's adapter file | Yes — step 7 |
| Add a skill producing durable findings | Consequences-map row says "persist a typed `progress.md` entry" | No — argued inapplicable (ADR table above); the map row arguably wants a clarifying clause for non-issue-scoped skills. Open Question 2. |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): `.claude/skills/audit-project/SKILL.md` (step 1
  becomes inaccurate the moment the resolver lands); `AGENTS.md` script table;
  `.agent/knowledge/skill_workflows.md`; the three framework adapter skill lists.
- **Agent-instruction candidates** (proposals only): a one-line clarification to the
  `.agent/knowledge/principles_review_guide.md` consequences-map row about durable-findings
  skills that are not issue-scoped (Open Question 2). Not applied in this PR.

## Open Questions

- [ ] This PR edits four instruction files (`AGENTS.md` and the three adapters), which is
  **Ask First** under AGENTS.md § Boundaries. The edits are additive list/table rows required
  by the Consequences Map — confirm that blanket approval covers them, or review them
  individually at PR time.
- [ ] Should the consequences-map row on durable-findings skills gain a clause for
  non-issue-scoped skills, or is the existing reading (siblings don't write `progress.md`)
  sufficient? Deliberately out of this PR unless the operator wants it in.
- [ ] The rolling issue does not exist yet. Confirm the sweep may create it on first run
  (title `Janitor sweep report (rolling)`, no label — none of the existing labels fit and
  `gh_create_issue.sh` validates against `.agent/github_metadata.json`), or whether the
  operator prefers to open it by hand first.

## Estimated Scope

Single PR.
