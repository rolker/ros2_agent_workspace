# Plan: Sweep split by scope + commit-via-PR publish + run-over-run diff + finding tiers

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/635

## Context

`janitor-sweep` (`.claude/skills/janitor-sweep/SKILL.md`) runs four checks
(workspace governance, project governance, issue staleness, research-digest
freshness) and writes one local report to `.agent/scratchpad/janitor/`. It is
report-only today: no PRs, no commits, nothing published.

The issue's own Scope text reads as if the commit-and-PR publish path applies
to both the workspace and per-project checks. It does not: the operator's
2026-09-18 comment on this issue narrows it — **publish-by-commit ships for
the workspace check only in this slice; project-repo checks stay
report-only** until the health-rollup shape (per-repo PR / one document per
project / health-follows-roadmap) is decided. That decision is explicitly
**not made** — this plan does not make it either.

The design draft (`docs/design/planning_document_vocabulary.md`) fixes the
mechanism this plan builds: `docs/health.md`, committed via PR under a
dedicated identity on a `skill/janitor-*` branch, replaced each run, human
review before merge (§ Publishing what the sweep finds). The redaction gate
(#626) it names is merged (PR #646) — the publish path may go live. The bot
identity (`Janitor Sweep Agent`) and the weekly trigger are #636, out of
scope here; hand-run testing in this PR uses the implementing agent's normal
identity.

The "Provisional decisions with no review scheduled" row
(draft § "The design-mode half") reads `## Decisions made this deployment`
sections that `/wrap-up-deployment` is supposed to write (#642, not shipped).
No such section exists anywhere in the repo today, so the row's data source
is currently always empty — the parser must treat that as a normal empty
result, never `FAILED`.

## Approach

1. **Split the report by scope, not by check number.** Today's four checks
   map cleanly onto two scopes already:
   - **Workspace scope**: check 1 (`audit-workspace`) + check 4
     (research-digest freshness) — both grade the workspace repo itself.
   - **Project scope**: check 2 (`audit-project`, per repo in the rotation)
     + check 3 (`issue-triage`, which already scans only overlay/project
     repos, never the workspace repo).
   Rewrite `SKILL.md` step 4 (report format) so the report has two top-level
   sections, `## Workspace` and `## Projects`, each carrying its own checks,
   findings, and status line — instead of one flat `### Check Status` table
   for all four. Keep the per-check status contract (OK/FINDINGS/SKIPPED/
   FAILED) unchanged; only the grouping changes.

2. **Add finding tiers, applied across both scopes.** Every finding gets
   classified into exactly one of five tiers, in this severity order:
   work that can be lost → unowned safety bugs → rules that have bitten with
   no enforcement → contradictions in the record → drift. Add a tier-mapping
   table to `SKILL.md` (§ "Run the four checks" or a new subsection) that
   assigns each existing finding *kind* to a tier, conservatively, so the
   mapping is mechanical rather than judgment-per-finding:
   - work-that-can-be-lost: uncommitted/unpushed work flagged by
     `audit-workspace`/`audit-project`, stale worktrees.
   - unowned-safety-bugs: `audit-project`'s Quality Standard gaps that name a
     safety-relevant failure mode (error handling, silent failure, missing
     validation) with no tracking issue.
   - rules-bitten-no-enforcement: a documented rule with no CI/hook backing
     it (ADR-0005's own test) — `audit-workspace`'s enforcement-gap section.
   - contradictions-in-the-record: `audit-workspace`/`audit-project` findings
     that two tracked documents disagree, and the new provisional-decisions
     row (below) once #642 exists.
   - drift (catch-all/default): everything else — stale docs, digest
     freshness, stale issues. Any finding not matched by a more specific rule
     above defaults here, so the mapping never fails closed.
   Render findings grouped by tier within each scope section, replacing the
   current per-check `#### <check name>` findings list.

3. **Run-over-run diff, every tier of every scope.** For the workspace
   scope, the previous run's state is the last committed `docs/health.md`
   (read via `git show HEAD:docs/health.md` before writing the new one).
   The janitor-sweep skill worktree doesn't exist yet at this point in the
   run — it's created later, in the publish step below — so this read
   actually runs from the main checkout; the same command is restated once
   the worktree exists, reading the same committed answer either way. Diff
   findings by a stable key (tier + check + one-line description) and
   render three subsections under each tier: `New`, `Resolved`,
   `Unchanged`. For the project scope (still report-only, no committed
   history), do a best-effort diff against the most recent prior local
   report for the same repo under `.agent/scratchpad/janitor/` when one
   exists, and say plainly when it does not (no prior report / prior report
   for a different chunk) rather than rendering an empty diff as "no
   changes." Every tier in both scopes' report sections carries this
   diff-state — the workspace scope renders it as `New`/`Resolved`/
   `Unchanged` subsections per tier (one committed history to diff
   against), the project scope renders the same three states as an inline
   `[New/Resolved/Unchanged]` tag per finding (many repos, independent
   best-effort local histories — per-tier subsections wouldn't stay
   legible). The one exception is the Projects "provisional decisions"
   sub-list, which isn't diffed because its own row text already states
   whether a decision has a review scheduled.

4. **Publish-by-commit — workspace scope only.** After the workspace-scope
   checks and tiering/diff are done:
   - Use the skill-worktree convention (`worktree_create.sh --skill
     janitor-sweep --type workspace`, branch `skill/janitor-sweep-<ts>`) —
     requires step 6 below.
   - **Redact before writing**: pass the rendered workspace section through
     `redact_text` (from `.agent/scripts/redact.sh`, with
     `REDACT_PATH_PREFIXES` set for the workspace root and `$HOME`,
     already wired up earlier in the run) before it touches disk. This is a
     mandatory code-level gate, not an authoring reminder — `docs/health.md`
     is committed to a public repo, the findings text is synthesized from
     `audit-workspace`/`audit-project` output that was never itself routed
     through `redact.sh`, and this skill is meant to run unattended once
     #636 wires the trigger.
   - Write the rendered (redacted) workspace section to `docs/health.md` at
     the repo root (the path the design draft's expected-location table
     fixes).
   - Commit with per-invocation `-c user.name=/-c user.email=` identity
     (AGENTS.md § Agent Commit Identity); for this PR's hand-run testing,
     the implementing agent's own identity, not `Janitor Sweep Agent`
     (that's #636's job per the host notes).
   - Push and open a **non-draft** PR (Copilot code review does not review
     draft PRs — open it non-draft so the review fires), title e.g. `Janitor sweep:
     workspace health <date>`, replacing any existing open PR from a prior
     `skill/janitor-*` branch rather than stacking a new one per run — **the
     new PR must be pushed and opened first; only then is the old PR
     commented-on, closed, and its branch deleted** (never the reverse: a
     failed new-push/PR-create must leave the previously-published PR
     intact, per Local Review round 1's must-fix).
   - **Never commit `docs/health.md` into a project repo in this slice** —
     project-scope findings stay in the local
     `.agent/scratchpad/janitor/<ts>-sweep.md` report exactly as today.
   - A human still merges (AGENTS.md § Merging, "green CI is not review");
     `SKILL.md` states this explicitly, matching the design draft.

5. **Provisional decisions row.** Add a tolerant scan, run once per project
   repo in check 2's rotation (project scope, report-only — not part of the
   workspace docs/health.md, since the workspace repo has no dev logs):
   search the repo's `<log_dir>/**/*.md` (from `.agents/deployment.yaml`'s
   `log_dir:`, when that config file exists; skip the repo silently,
   counted as "no deployment config" not a failure, when it doesn't) for a
   `## Decisions made this deployment` heading. No heading anywhere in any
   scanned repo → the row renders "no provisional decisions found this run"
   (empty, `OK`), never `FAILED`. Where a heading exists, parse entries
   loosely (decision text, date made, where recorded, review owed) and flag
   an entry as unscheduled when its "review owed" text names no date or
   forcing-function phrase, aged from its date. Feed matches into the
   contradictions-in-the-record tier (they signal work that outpaced its own
   review, closest existing category) as a distinct labeled sub-list, so a
   later #642-driven refinement can retarget the tier without re-deriving
   the scan.

6. **Allowlist `janitor-sweep` for skill worktrees.** Add `"janitor-sweep"`
   to `ALLOWED_SKILLS` in `.agent/scripts/worktree_create.sh` (currently
   `("research" "inspiration-tracker")`, line 387). No other change needed —
   the skill-worktree machinery (branch naming, synthetic ID) is already
   generic over the skill name.

7. **Update the Consequences Map exception clause.** In
   `.agent/knowledge/principles_review_guide.md`'s Consequences Map row for
   "Add a workflow skill that produces durable findings," the sentence "Only
   `janitor-sweep` has a durable output today (a local report file under
   `.agent/scratchpad/janitor/`)" goes stale once part of its output is a
   committed `docs/health.md` PR. Rewrite to state the split: the workspace
   portion durably publishes as a committed, PR-reviewed `docs/health.md`;
   the project portion remains a local report file under
   `.agent/scratchpad/janitor/`, pending the rollup-shape decision.

## Files to Change

| File | Change |
|------|--------|
| `.claude/skills/janitor-sweep/SKILL.md` | Restructure report into workspace/project scope sections; add finding-tier classification and rendering; add run-over-run diff (committed-history-based for workspace, best-effort local-report-based for projects); add publish-by-commit step (worktree, commit, push, PR) gated to workspace scope only; add provisional-decisions tolerant scan for project scope; update "Deferred: publishing and the trigger" section to record that publishing is no longer fully deferred (workspace half is live; project half stays deferred) |
| `.agent/scripts/worktree_create.sh` | Add `"janitor-sweep"` to `ALLOWED_SKILLS` (line 387) |
| `.agent/knowledge/principles_review_guide.md` | Rewrite the janitor-sweep durable-output sentence in the Consequences Map to state the workspace/project split |
| `AGENTS.md` | Add `janitor-sweep` to the Skill Worktree Exception's "Allowed skills" line, alongside `worktree_create.sh`'s allowlist (must-fix from Plan Review — was missing from this table even though the plan's own Consequences row already committed to it) |

### Implementation notes (added during implementation, not re-planned)

Folded in per the Plan Review's must-fix and two suggestions
(`.agent/work-plans/issue-635/progress.md` § Plan Review):

- **`AGENTS.md` above** — added to this table, and edited alongside
  `worktree_create.sh` in the same commit.
- **`.agent/knowledge/skill_workflows.md`** was also updated (not originally
  in the Files to Change table): its utility-skills table described
  `janitor-sweep` as "report-only, publishes nothing", which goes stale the
  moment workspace scope commits `docs/health.md`. Caught by the grep for
  other stale references the implementation prompt asked for.
- **PR-replacement mechanism (step 4 of Approach, suggestion)** — implemented
  concretely in `SKILL.md` § Publish the workspace scope, sub-step 7f (after
  the round-1 Local Review fix reordered publish so the new PR opens first
  in 7e and the old one is replaced second in 7f, never the reverse):
  `gh pr list --state open --json number,headRefName` filtered by the
  `skill/janitor-sweep-` prefix (excluding this run's own new branch), then
  `gh pr comment` (naming the new PR's real URL, captured from `gh pr
  create`'s own output) + `gh pr close` on each match, then `git push origin
  --delete` the old branch (failure recorded, not silently swallowed).
- **First-run diff behavior (step 3 of Approach, suggestion)** — stated
  explicitly in `SKILL.md` § Run-over-run diff: a failed or empty `git show
  HEAD:docs/health.md` (no prior commit, or the file absent on the base
  branch) renders every workspace-scope finding under `New` with an explicit
  "first committed run — no prior health document" note, never treated as an
  error.
- No deviations from the Approach section's numbered steps — all seven
  landed as scoped. `docs/health.md` itself was not created in this PR, per
  the implementation instructions; the sweep creates it on its first real
  run.

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Documentation Accuracy (AGENTS.md) | `docs/health.md` content is generated from check output, not hand-typed; timestamps in the report use `date`, per existing `SKILL.md` step 4 discipline, unchanged here |
| Quality Standard | The safety-tier ("unowned safety bugs") is deliberately not dismissed as a nit — it is its own tier, ranked second only to lost work |
| ADR-0005 (layered enforcement) | The new tiers are report content, not new enforcement; "rules that have bitten with no enforcement" tier explicitly surfaces ADR-0005's own test without adding a new gate |
| AGENTS.md § Merging ("green CI is not review") | Stated explicitly in the publish step; the PR is opened non-draft specifically so Copilot review fires, and a human still merges |
| #609 false-green rule | Preserved unchanged — this plan does not touch the four-state check contract, only how findings are grouped and where the workspace-scope result is written |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0013 (`progress.md` entry-type vocabulary) | No — `janitor-sweep`'s exception in the Consequences Map already covers it (updated in step 7, not overridden) |
| ADR-0015 (dispatch/handoff — container produces, host publishes) | No new dispatch path introduced; publish happens from the worktree run by the agent that has GitHub write auth, consistent with the ADR |
| ADR-0018 (local-first CI verification) | Not triggered — `docs/health.md` lands in the workspace repo, which is exempt from the local-CI merge path (hosted checks stay required) |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `janitor-sweep`'s durable-output shape | Consequences Map exception clause (`principles_review_guide.md`) | Yes — step 7 |
| `worktree_create.sh`'s `ALLOWED_SKILLS` | `AGENTS.md` § Skill Worktree Exception's "Allowed skills" line | Yes — follow-up commit alongside step 6 |
| A roadmap or health document (per Consequences Map row already present) | `docs/design/planning_document_vocabulary.md` kinds table if the kind or location changed | No — this plan uses the path the table already fixes (`docs/health.md`); no update needed |
| The provisional-decisions row's data source | Re-check once #642 ships and defines the actual `## Decisions made this deployment` shape | No — explicitly deferred; flagged as an Open Question below |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): `.agent/knowledge/principles_review_guide.md`'s Consequences Map exception sentence (step 7); `AGENTS.md`'s Skill Worktree Exception "Allowed skills" line, since it will be factually wrong once `worktree_create.sh` adds `janitor-sweep`.
- **Agent-instruction candidates**: None — this is groundwork already scoped and named by the design draft; no new pattern surfaced that isn't already captured there.

## Open Questions

- The provisional-decisions row's parser is built against the *proposed* shape (decision, date, where recorded, review owed) since #642 hasn't shipped and no real instance exists to verify against. It will need a follow-up once #642 lands and produces real `## Decisions made this deployment` sections — noted inline in `SKILL.md` as a known-provisional parser, not held open here.
- Whether an aged, still-unscheduled provisional decision should ever escalate beyond "listed" is an explicit open question in the design draft itself ("What happens when the provisional queue outruns the review") — this plan does not decide it; the row just lists and ages.
- The health-rollup shape for project repos (per-repo PR / one doc per project / health-follows-roadmap) is the operator's open decision, not this plan's to make — project scope stays report-only pending it.

## Estimated Scope

Single PR. The bulk of the work is a rewrite of one skill file
(`.claude/skills/janitor-sweep/SKILL.md`); the other two files change by one
line and one sentence respectively. No package code, no tests to run beyond
a hand-invoked sweep exercising both the workspace-commit path and the
project-report-only path.
