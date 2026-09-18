# Plan: #481 sub (G): delete push_gateway + obsolete scratchpad request dirs

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/493

## Context

The gateway pattern was the original mechanism for letting sandboxed containers
signal the host to do git push + gh operations on their behalf (via signal JSON
files in `.agent/scratchpad/`). Issue #481/#490 replaced this with the
host-orchestrator model: the host (`dispatch_subagent.sh` / `/run-issue`) drives
all git push and `gh` operations inline using its own credentials, reading
`progress.md` as the intent record. The gateway scripts and scratchpad dirs are
therefore dead code with no callers in the live workflow.

Scope decision (confirmed by user): **full sweep** — delete all three scripts,
remove all live references (docker_run_agent.sh, README.md, Makefile, generated
skill), and remove the now-empty `.agent/` read-write mount rationale from the
devcontainer README. Historical work-plan records (`.agent/work-plans/`,
`.agent/work-artifacts/`) are left untouched.

**Prerequisite**: #492 (host orchestrator) was merged in PR #525. This task
(#493) is now unblocked.

## Approach

1. **Delete the three gateway scripts** — `push_gateway.sh`, `push_request.sh`,
   `issue_request.sh` from `.agent/scripts/`. They are orphaned by the
   host-orchestrator model and have no remaining callers.

2. **Gut the post-exit gateway block in `docker_run_agent.sh`** — remove the
   interactive push/issue-request post-exit code (lines ~499–534) and the
   scratchpad dir pre-create for push-requests (line ~301). Update the file
   header comment (line 6) to drop the "check for pending push requests" clause.
   The `DISPATCH_MODE` early-exit block (lines ~492–497) and its comment are
   also stale now that the gateway is gone — simplify to just surfacing the exit
   code. Retain all other logic (build, mounts, container launch, auth, etc.).

3. **Remove the Makefile `push-gateway` target** — delete the `push-gateway`
   target body (lines 304–305), its help text line (line 90), and its entry in
   the `.PHONY` list (line 46). Then run `make generate-skills` to have the
   generate script delete the stale `.claude/skills/make_push-gateway/` skill
   directory.

4. **Rewrite the devcontainer README gateway sections** — replace the gateway-
   centric content in `.devcontainer/agent/README.md` with prose describing the
   host-orchestrator model:
   - Opening tagline (line 5): update "pushes and PR creation happen from the
     host via the push gateway" → "pushes and PR creation happen from the host
     orchestrator using its own credentials, with `progress.md` as the intent
     record."
   - Quick Start block: remove the `push_request.sh` step 3 item and the
     "Launcher detects pending push request" step 4 item; replace with a note
     that the host orchestrator (dispatch_subagent.sh / /run-issue) reads
     `progress.md` after the container exits and performs git push + gh ops.
   - Security Model section: update "All pushes and PR creation happen on the
     host via the push gateway" → describe the host-orchestrator model; remove
     the ".agent/scratchpad/ is read-write for push request signal files" clause
     (scratchpad remains rw for temp files, just not for signal files).
   - Delete the entire `## Push Gateway Workflow` section (lines ~181–201),
     including the Manual push gateway subsection.
   - Final note in Read-Only GitHub Access section (line 252): update "Those
     actions go through the push gateway on the host" → "Those actions are
     performed by the host orchestrator using its own credentials."

5. **Note on AGENTS.md** — The Script Reference table in `AGENTS.md` does NOT
   currently list `push_gateway.sh`, `push_request.sh`, or `issue_request.sh`
   (confirmed by grep). No row removal is needed. `AGENTS.md` is an "Ask First"
   instruction file per workspace rules; the host already approved the full sweep,
   so this is called out explicitly: no AGENTS.md edit is required for this PR.

6. **Verify no live references remain** — after the above changes, run:
   ```bash
   grep -rn "push.gateway\|push_gateway\|push-gateway\|push_request\|issue_request" \
       . --include="*.sh" --include="*.md" --include="Makefile" \
       | grep -v ".agent/work-plans/" | grep -v ".agent/work-artifacts/" \
       | grep -v ".git/"
   ```
   Expect zero hits (in live code).

7. **Verify the generated skill is gone** — after `make generate-skills`:
   ```bash
   ls .claude/skills/ | grep make_push
   ```
   Expect no output.

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/push_gateway.sh` | **Delete** (host-side gateway processor) |
| `.agent/scripts/push_request.sh` | **Delete** (container-side signal writer) |
| `.agent/scripts/issue_request.sh` | **Delete** (container-side issue signal writer) |
| `.agent/scripts/docker_run_agent.sh` | Remove post-exit gateway block, scratchpad mkdir, stale header comment, gateway fallback message |
| `Makefile` | Remove `push-gateway` from `.PHONY`, help text, and target body; run `make generate-skills` |
| `.claude/skills/make_push-gateway/SKILL.md` | **Delete** (generated by `make generate-skills` — regeneration removes it after the Makefile target is gone) |
| `.devcontainer/agent/README.md` | Rewrite gateway-centric sections to describe host-orchestrator model; delete `## Push Gateway Workflow` section |

**Not touched**: `AGENTS.md` (gateway scripts were never in the Script Reference table), `.agent/work-plans/`, `.agent/work-artifacts/`, historical `PLAN_ISSUE-*` files.

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Completeness | Full sweep: all three scripts + all live references removed in one PR; no partial cleanup |
| Documentation accuracy | README rewrite must accurately describe the host-orchestrator model (verified against `dispatch_subagent.sh` and `docker_run_agent.sh` post-cleanup) |
| Atomic commits | One commit per logical unit: (a) delete scripts, (b) docker_run_agent.sh cleanup, (c) Makefile + generate-skills, (d) README rewrite |
| AGENTS.md "Ask First" | Editing AGENTS.md is "Ask First"; this PR does NOT need to — gateway rows were never in the table |
| Workspace cleanliness | Scratchpad `push-requests/` and `issue-requests/` dirs are gitignored transient dirs (never tracked), so no `git rm` needed — the `mkdir -p` in docker_run_agent.sh is removed; existing dirs on disk will be naturally ignored |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0006 (AGENTS.md as shared instruction file) | No | AGENTS.md not modified; gateway scripts were not in its Script Reference table |
| ADR-0004 (enforcement hierarchy) | No | This is a deletion of orphaned scripts, not a new enforcement mechanism |
| ADR-0013 (progress.md vocabulary) | Yes — indirectly | `progress.md` is now the sole intent record for container→host communication; the plan aligns with that ADR's intent |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Delete `push_gateway.sh` | Makefile `push-gateway` target, devcontainer README, `docker_run_agent.sh` gateway invocation, generated skill | Yes — all covered in steps 2–5 |
| Delete `push_request.sh` | `docker_run_agent.sh` scratchpad mkdir (removed in step 2); README Quick Start text (step 4) | Yes |
| Delete `issue_request.sh` | `docker_run_agent.sh` issue-dir scan (removed in step 2); README (step 4) | Yes |
| Remove Makefile `push-gateway` | Run `make generate-skills` to drop `.claude/skills/make_push-gateway/` | Yes (step 3) |
| `.agent/scratchpad/push-requests/` and `issue-requests/` | Dirs are gitignored, not tracked — no `git rm` needed; `mkdir -p` in docker_run_agent.sh removed | Yes (step 2) |

## Open Questions

- [ ] No open questions — user confirmed full-sweep scope; plan is review-plan-ready.

## Estimated Scope

Single PR. All changes are in the workspace repo. Four atomic commits:
1. Delete scripts (`push_gateway.sh`, `push_request.sh`, `issue_request.sh`)
2. Clean up `docker_run_agent.sh`
3. Remove Makefile `push-gateway` target + run `make generate-skills`
4. Rewrite `.devcontainer/agent/README.md`

## Redone fresh on main (2026-09-18)

The June branch for this issue fell 755 commits behind `main` and conflicted
in `docker_run_agent.sh` and `.devcontainer/agent/README.md` — both files were
reworked in August for the anonymous-volume ownership fix
([#604](https://github.com/rolker/ros2_agent_workspace/issues/604)). Rather
than rebase the stale branch through those conflicts, the deletions in this
plan were redone from scratch against current `main`, using the June attempt's
plan and progress notes above as the reference for scope and rationale.
