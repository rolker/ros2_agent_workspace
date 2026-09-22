---
issue: 654
---

# Issue #654 — Local rosdep keys: workspace-owned ROSDEP_SOURCE_PATH aggregating per-repo rosdep.yaml, upstream PRs owed at merge

## Issue Review
**Status**: complete
**When**: 2026-09-22 10:32 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)

**Issue**: #654
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Actions
- [ ] Add the new `.agent/scripts/rosdep_local_sources.sh` to the AGENTS.md Script Reference table (consequences-map row "A script in `.agent/scripts/`" — table entry not mentioned in issue scope).
- [ ] Clarify how the agent-container-image path (scope item 3) interacts with the existing bake-time `stage_rosdep_manifests.sh` / `docker_run_agent.sh --build` mechanism (#520) — does the new local `rosdep.yaml` set need its own build-context staging step, or is `ROSDEP_SOURCE_PATH` resolved only at container runtime against a mounted workspace?
- [ ] Note the implied `Makefile` touch-point for wiring the new staleness/upstream-PR-missing check into the `validate` target's recipe (consequences-map row "A script in `.agent/scripts/` → ... Makefile if it has a target" — not explicit in scope item 4).

### Findings (not blocking)

**Scope Assessment**
- Well-scoped, but large: 5 scope items spanning declaration format, an aggregation script, three separate CI integration points (`ci_local.sh` inner script, agent container image, hosted-workflow doc), an enforcement check, and a knowledge note. The pieces are genuinely interdependent (the enforcement check needs the aggregation mechanism; the CI paths need the same source list) so a single PR is defensible, but it's worth watching for scope creep during implementation — items 3's three integration points could be split into a fast-follow if the PR grows unwieldy.
- Right repo: yes. The aggregation script glob-scans `layers/main/*/src/*/rosdep.yaml` generically — no project-specific coupling — consistent with ADR-0003 (workspace infra is project-agnostic). The per-repo `rosdep.yaml` declaration itself lives in the *consuming* project repo (unh_marine_autonomy), correctly left out of this issue's scope.
- Dependencies: soft dependency on rolker/unh_marine_autonomy#397 (the consumer that needs `python3-pystac`/`snakemake`), but the test plan uses synthetic fixture repos, so this issue is independently testable and not blocked on #397 landing first. Parallel-tracked with rolker/agent_workspace#310 (writing up the general non-ROS-package dependency policy) — correctly deferred there per the issue's own text ("tried here before it is written down elsewhere").
- Verified against current code: `bootstrap.sh` and `ci_local.sh`'s inner script today only `rosdep init`/`rosdep update` against the system default `/etc/ros/rosdep/sources.list.d/20-default.list` — no existing `ROSDEP_SOURCE_PATH` override exists anywhere in `.agent/scripts/` or the `Makefile`, confirming this is genuinely new plumbing at the three touch points the issue names, not a duplicate of existing logic.

**Principle Alignment**

| Principle | Status | Notes |
|---|---|---|
| Enforcement over documentation | OK | Item 4 (make-validate staleness/missing-PR check) pairs enforcement with item 5's knowledge note, rather than documenting the policy alone. |
| Capture decisions, not just implementations | OK | ADR deliberately deferred until the mechanism survives a second instance — matches the workspace's recent ADR-fatigue lesson (no premature ADRs while a design is still evolving); the decision itself is recorded in the issue text and will land in the knowledge note. |
| Only what's needed | OK | Explicitly scoped to the Ubuntu-shipped-but-unkeyed case; the harder "Ubuntu doesn't ship it at all" case is named and deferred rather than solved speculatively. |
| A change includes its consequences | Action needed | See Actions above (AGENTS.md script table, Makefile touch-point). |
| Improve incrementally | Watch | Large but cohesive; see Scope Assessment. |
| Workspace vs. project separation | OK | Generic aggregation mechanism; no BizzyBoat/unh_marine_autonomy specifics baked into workspace scripts. |
| Workspace improvements cascade to projects | OK | Any project repo can opt in by adding its own `rosdep.yaml`; item 3's "documented one-step equivalent" for hosted CI keeps project repos independently buildable per ADR-0003. |
| Test what breaks | OK | Test plan targets real failure modes: idempotent regeneration, resolve success/failure gated on the env being sourced, in-container install, and staleness detection with a fixture that resolves upstream. |

**ADR Applicability**

| ADR | Triggered | Notes |
|---|---|---|
| 0003 — Project-agnostic workspace | Yes | Satisfied — see Scope Assessment. |
| 0004/0005 — Enforcement hierarchy / layered enforcement | Yes | The new check runs via `make validate`, which today is *not* invoked by hosted CI (`.github/workflows/validate.yml` runs `make lint` and `make test-scripts`, not `make validate`) — this mirrors `validate_workspace.py`'s existing local-only role, so it's consistent with current practice, not a new gap introduced by this issue. Worth keeping in mind during plan-task if the staleness check is meant to be load-bearing rather than advisory. |
| 0009 — Python package management | Yes (Tier 1) | `python3-pystac`/`snakemake` are Ubuntu 24.04 apt packages needed at runtime by a ROS-adjacent tool, matching Tier 1 ("ROS 2 dependency... available via apt/rosdep"). The issue's own "open case" callout (non-Ubuntu-shipped libraries → venv, out of scope) correctly avoids conflicting with ADR-0009's Tier 2/3, which govern the workspace `.venv` only. Note ADR-0009's own status is still "Proposed," not "Accepted" — not a blocker for this issue, just a standing gap unrelated to #654. |
| 0018 — Local-first CI verification | Yes | Touches `ci_local.sh`'s inner script (rosdep source install before `rosdep install`). Should verify during implementation that the new source-list step doesn't interact badly with the existing `upstream-repo:`/`rosdep-skip-keys:` attestation note lines for repos using `upstream.repos` — not called out in the issue, likely orthogonal, but worth a look. |

**Consequences**
- AGENTS.md Script Reference table needs a new row for `.agent/scripts/rosdep_local_sources.sh` (not currently in scope).
- Makefile needs a wiring point for the new validate-time check (implied by item 4, not explicit).
- `.agent/knowledge/dependency_policy.md` is a net-new knowledge doc — no existing doc to reconcile against.

**Recommendations**
- Add the AGENTS.md script-table entry and the Makefile wiring explicitly to the plan-task scope (see Actions).
- During plan-task, confirm the agent-container-image bake path (item 3) either reuses `stage_rosdep_manifests.sh`'s existing manifest-staging pattern or explains why runtime-only `ROSDEP_SOURCE_PATH` resolution is sufficient for the container case.
