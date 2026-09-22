# Workspace roadmap

What this workspace is aiming for, and what is deferred. Scope is the workspace
itself — agent instructions, worktree and build tooling, skills, governance. The
autonomy software it hosts has its own roadmaps in its own repos. Durable
direction lives here; specific bounded work lives in issues, referenced by
number.

**Parent roadmap**: none — this is a root.<br>
**Roadmaps beneath this one**: none in this repo.<br>
**Last reviewed**: 2026-09-17 — stamped when the forcing function last fired.

Project roots carry their own. The framework repo (`unh_marine_autonomy`) is the
first one expected to, with the BizzyBoat roadmap
([`unh_echoboats_project11`](https://github.com/rolker/unh_echoboats_project11/blob/main/docs/roadmap.md)
— still at its pre-existing `docs/roadmap.md` path until the rename to a root
`ROADMAP.md` lands) beneath it.

**Health document**: [`docs/health.md`](docs/health.md), first committed
2026-09-21 by the `janitor-sweep` skill, which replaces it wholesale on every
`--publish` run — the sweep is local-first as of
[#652](https://github.com/rolker/ros2_agent_workspace/issues/652), so an
ordinary run leaves this file alone and writes its report to the host that
ran it (see *How this roadmap stays useful*). That path is the one
the [planning-document vocabulary draft](docs/design/planning_document_vocabulary.md)
fixes for it — a workspace choice, with no external convention either way, and
deliberately under `docs/` rather than beside this file at the root.

*Created 2026-09-17 under
[#628](https://github.com/rolker/ros2_agent_workspace/issues/628), absorbing the
direction asked for in
[#249](https://github.com/rolker/ros2_agent_workspace/issues/249). #249 stays
open as its own Phase-2 tracker.*

## End goal

**Projects get better faster because agents work on them.** The workspace exists
to make AI agents effective contributors to ROS 2 projects — robust,
well-documented, well-tested packages for real autonomous marine systems — and
the measure of improvement is whether the *projects* improve, not whether the
workspace gets more sophisticated. The infrastructure stays project-agnostic by
design ([ADR-0003](docs/decisions/0003-workspace-infrastructure-is-project-agnostic.md)),
so it is useful beyond its origin. See [`README.md` § Vision](README.md#vision).

## Guiding properties

Items are prioritised by how much they improve these four, which are #249's, kept
verbatim in substance:

1. **Simple to use** — a small number of clear entry points, not dozens of
   scripts to remember.
2. **Self-documenting** — the workspace explains itself to a new agent or human
   without a tour.
3. **Durable** — decisions are captured where they cannot be silently reverted.
4. **Sustainable** — improvement is a continuing practice, not periodic rework.

## Active threads

*No `Owner` or `Priority` column: every item here has the same owner (Roland,
with agent help), and the flagship thread this cycle is the roadmap + health
loop — which is why it is first.*

### The roadmap + health loop (this document's own reason to exist)

| Item | Issue | Status | Notes |
|---|---|---|---|
| Planning-document vocabulary + two-root rule (draft + this roadmap) | #628 | in progress | The draft in [`docs/design/`](docs/design/planning_document_vocabulary.md); this is its first roadmap instance |
| Conventional-path discovery in `janitor-sweep` / `audit-project` | [#634](https://github.com/rolker/ros2_agent_workspace/issues/634) | planned | Probes exactly the draft's expected-location table; graceful absence is the headline requirement |
| Sweep split by scope + commit-via-PR publish + run-over-run diff + finding tiers | [#635](https://github.com/rolker/ros2_agent_workspace/issues/635) | done | Merged 2026-09-21 as PR #647; first live run PR #648. Committed `docs/health.md` is now `--publish`-only (#652) |
| The trigger — weekly cloud Routine under `Janitor Sweep Agent` | [#636](https://github.com/rolker/ros2_agent_workspace/issues/636) | deferred | Backburnered 2026-09-21 (operator direction, recorded in [#652](https://github.com/rolker/ros2_agent_workspace/issues/652)) in favour of a local health report. Parked, not cancelled: `--publish` (#652) keeps the commit-and-PR path this issue will use once unparked. Still owes the run cap and confirmation that the GitHub connection reaches every overlay repo with write access. #569 stays open until it lands |
| Local-first sweep: `--publish` opt-in, local diff source, optional per-project health report | [#652](https://github.com/rolker/ros2_agent_workspace/issues/652) | in progress | Every run writes a local report; committing `docs/health.md` is opt-in. Health follows the roadmap: a project root with a `ROADMAP.md` gets a second local report |
| What-next port: health-report refresh trigger + per-area ranking for parallel agents | [#653](https://github.com/rolker/ros2_agent_workspace/issues/653) | planned | Design draft first, per the issue's own framing — it is a port of an existing idea, not a new mechanism |
| `unh_marine_autonomy`: `VISION.md` folded into README § Vision, root `ROADMAP.md`, kind markers | [unh_marine_autonomy#394](https://github.com/rolker/unh_marine_autonomy/issues/394) | planned | Filed in that repo, not here |
| `unh_echoboats_project11`: roadmap parent line, move to root `ROADMAP.md`, health placement | [unh_echoboats_project11#492](https://github.com/rolker/unh_echoboats_project11/issues/492) | planned | Filed in that repo, not here; the move is a rename, so history is preserved |
| Promote the vocabulary draft to an ADR | [#637](https://github.com/rolker/ros2_agent_workspace/issues/637) | planned | Gated on the reader, the sweep, and a second ROS 2 project exercising the rule |
| Scheduled staleness/drift sweep (the parent thread) | [#569](https://github.com/rolker/ros2_agent_workspace/issues/569) | in progress | Hand-run once; stays open until the trigger lands |

### Honest reporting from the tooling

The workspace grades other people's repos; its own scripts have to be trusted
first. This thread is what stops a green result from meaning nothing.

| Item | Issue | Status | Notes |
|---|---|---|---|
| Sweep redaction: host paths and credentials reaching diagnostic output | [#626](https://github.com/rolker/ros2_agent_workspace/issues/626) | planned | **Gates the commit-publish path** — a health document is a public artifact |
| `issue-triage` silently truncates repos with >100 open issues | [#627](https://github.com/rolker/ros2_agent_workspace/issues/627) | planned | A detector that under-reports without saying so |
| `merge_pr.sh` reports "CI checks failed" for a repo with no CI | [#610](https://github.com/rolker/ros2_agent_workspace/issues/610) | planned | Merge-gate honesty |

### Simplification and documentation consolidation (#249 Phase 2)

| Item | Issue | Status | Notes |
|---|---|---|---|
| Simplify scripts and Makefile interface | [#263](https://github.com/rolker/ros2_agent_workspace/issues/263) | planned | Directly serves guiding property 1 |
| Consolidate and reduce documentation overlap | [#264](https://github.com/rolker/ros2_agent_workspace/issues/264) | planned | Serves property 2 |
| CI enforcement checks for workspace rules | [#265](https://github.com/rolker/ros2_agent_workspace/issues/265) | planned | Serves property 3 — a rule not in CI is a suggestion (ADR-0005) |
| Evaluate MCP servers for ROS 2 workflows | [#266](https://github.com/rolker/ros2_agent_workspace/issues/266) | planned | Spun off from #249 |

### Governance record upkeep

| Item | Issue | Status | Notes |
|---|---|---|---|
| Backfill the ADR Applicability table in `.agent/knowledge/principles_review_guide.md` | — | planned | The table runs 0001–0010 then 0013: **0011, 0012 and 0014–0019 have no row at all** (verified 2026-09-17). Exactly the quiet drift the roadmap + health loop exists to surface |
| A `Provisional` ADR status for decisions made in deployment mode and owed a design-mode review | [#620](https://github.com/rolker/ros2_agent_workspace/issues/620) | planned | Specified in the [vocabulary draft](docs/design/planning_document_vocabulary.md#decisions-and-drafts-across-the-deployment-boundary) on 2026-09-18; lands in ADR-0001 and the ADR template when the draft is promoted (#637). Wrap-up decision list filed as [#642](https://github.com/rolker/ros2_agent_workspace/issues/642); the review pass is filed once #635's health row exists |
| Resolve the three ADRs still `Proposed` while cited as binding | — | planned | 0005 (layered enforcement), 0008 (ROS 2 conventions), 0009 (Python packaging), verified 2026-09-17 |

## Recently completed

Kept for one cycle — one sweep — then pruned out. Empty is a normal state.

| Item | Issue | Completed |
|---|---|---|
| Staleness detectors chained into one local sweep report | [#569](https://github.com/rolker/ros2_agent_workspace/issues/569) | [PR#625](https://github.com/rolker/ros2_agent_workspace/pull/625) merged 2026-09-14. #569 itself stays open for the trigger |
| False-green sweep across `sync` / `pull` / `validate` | [#609](https://github.com/rolker/ros2_agent_workspace/issues/609) | Closed 2026-08-24 |
| Audit coverage as data: a `Resolved` finding must mean the run looked | [#651](https://github.com/rolker/ros2_agent_workspace/issues/651) | [PR#655](https://github.com/rolker/ros2_agent_workspace/pull/655) merged 2026-09-22. `audit-workspace` / `audit-project` now report a `### Coverage` table the sweep's diff gates on; an unexamined section carries its findings forward as `Not re-examined` rather than resolving them |

## Deferred

| Item | Issue | Deferred because |
|---|---|---|
| "The README contains everything, concisely" | [#249](https://github.com/rolker/ros2_agent_workspace/issues/249) | Under consideration, never decided. Carried forward explicitly rather than closed with #249's other items |
| A central architecture document consulted at issue, plan and implementation time | [#249](https://github.com/rolker/ros2_agent_workspace/issues/249) | **Largely already answered** by [`ARCHITECTURE.md`](ARCHITECTURE.md). What is still owed is the *consult-and-update cadence*, which the Consequences Map and the roadmap + health loop are meant to provide — recorded here rather than dropped |
| Widening the Consequences Map to every `docs/` document kind | — | It has no row for a prose page under `docs/` that is neither a principle nor an ADR. The row this workspace needed (roadmap / health) landed under #628; the general case is not urgent |

**A decision to protect, not revisit**: per-framework instruction duplication
(`CLAUDE.md`, `.github/copilot-instructions.md`,
`.agent/instructions/gemini-cli.instructions.md`) is **deliberate** — frameworks
may prefer different layouts (#249). It is listed here because it is the kind of
thing a consolidation pass reverts by accident while tidying.

## What's not on this roadmap

- Specific bug fixes and features — those are issues.
- The autonomy software itself — that lives in the project repos' own roadmaps.
- Anything merged more than a cycle ago. *Recently completed* holds the last
  cycle's worth and is pruned; beyond that, history is git's job.

## How this roadmap stays useful

- **Read it at the periodic sweep** — the workspace's forcing function, the
  counterpart to what deployments do for a platform repo. **Stated plainly: that
  cadence is not running yet.** The sweep exists and has been hand-run once
  ([#569](https://github.com/rolker/ros2_agent_workspace/issues/569)), but
  nothing schedules it until the trigger sub-issue of #628 lands. Until then the
  forcing function is the hand-run sweep and plan review; after it, the schedule.
- **Read it together with the health document** once one exists. The roadmap says
  where we want to go; health says what will stop us. Work appearing in both goes
  first.
- **Leftovers land here** — things that came up, are not bounded enough for an
  issue, and are not being done next.
- **Prune periodically.** A deferred item nobody has pulled toward in months is
  dropped, not deferred. Edit it out.
- **Mark items done in place**, keeping the `#<N>`, so the loop shows in git
  history.

---

Instantiated from [`.agent/templates/roadmap.md`](.agent/templates/roadmap.md).
Kind and structure: [`docs/design/planning_document_vocabulary.md`](docs/design/planning_document_vocabulary.md).
