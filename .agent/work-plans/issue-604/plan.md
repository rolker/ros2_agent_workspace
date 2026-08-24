# Plan: docker_run_agent.sh worktree build/install/log anonymous volumes come up root-owned

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/604

## Context

#602 (merged in #603) added a mount-side shield (`docker_run_agent.sh`
section 4b) for the *dispatched worktree's* own `*_ws/{build,install,log}`
anonymous volumes, mirroring section 4's existing shield for
`layers/main/*_ws`. It missed the ownership half: `agent-entrypoint.sh`'s
chown loop (lines 29-41) only walks `$WORKSPACE_ROOT/layers/main/*_ws` — it
has no equivalent loop for `$WORKTREE_ROOT/*_ws`, even though `WORKTREE_ROOT`
is already forwarded into the container (`docker_run_agent.sh:526`,
`-e "WORKTREE_ROOT=$WORKTREE_PATH"`). Docker initializes an anonymous
volume from the image at the mount path, not from anything host-side, so
without a matching entrypoint chown the volume comes up `root:root` and the
dropped-privilege `ros` user can't write to it — a dispatched agent can't
build in its own worktree.

`review-issue`'s completed pass (`.agent/work-plans/issue-604/progress.md`,
`## Issue Review`) verified the root cause and fix against current source
and flagged three actions this plan must cover: (1) the entrypoint chown
loop itself, (2) regression coverage for the mount↔chown pairing — preferably
the low-cost `--print-mounts`-driven invariant check in this same PR, and
(3) a scope caveat noting `test_layer_sourcing.sh` Check 4 covers neither
worktree-scoped nor container-side ownership.

## Approach

1. **Add the second entrypoint chown loop** in
   `.devcontainer/agent/agent-entrypoint.sh`, immediately after the existing
   `layers/main/*_ws` loop (step 1, lines 29-41). Guard on
   `[ -n "${WORKTREE_ROOT:-}" ] && [ -d "$WORKTREE_ROOT" ]` (the var is only
   set by `docker_run_agent.sh`, never by a bare `docker run`), then iterate
   `"$WORKTREE_ROOT"/*_ws` with the same `[ -d ] && [ ! -L ]` guard
   `docker_run_agent.sh` section 4b uses, so a layer worktree's symlinked
   `*_ws` siblings (pointing back into `layers/main`, already chowned by the
   first loop) aren't double-chowned or reached-through. Reuse the identical
   chown/mkdir body (`chown -R` on an existing dir, else `mkdir -p` + `chown`)
   so both loops stay visibly the same mechanism.
2. **Preserve rationale in the new loop's comment**, adapting section 4b's
   "mkdir-as-invoking-user precaution" language to the entrypoint's
   chown-not-mkdir mechanism, and folding in the issue's "reasoning error
   worth recording" point (an anonymous volume is initialized from the
   image, not the bind-mounted host dir — so host-side `mkdir` alone can
   never fix ownership; only the entrypoint `chown` does) so a future editor
   of section 4b doesn't repeat the mistake.
3. **Add regression coverage as a `--print-mounts`-driven invariant check.**
   `--print-mounts` already dumps every anonymous-volume mount
   `docker_run_agent.sh` adds, and it's Docker-free (no container run
   needed) — `test_docker_run_mount_args.sh` already exercises it with a
   fabricated tree via `DRA_ROOT_DIR_OVERRIDE`. Add a new test,
   `.agent/scripts/tests/test_entrypoint_chown_coverage.sh`, that:
   - Reuses (or factors out into a small shared fixture helper) the same
     fabricated layer-worktree tree as `test_docker_run_mount_args.sh`
     (a real `*_ws` under both `layers/main` and the worktree, plus a
     symlinked sibling).
   - Runs `docker_run_agent.sh --print-mounts` for that worktree and
     collects every mounted `*_ws/{build,install,log}` path.
   - For each mounted path, asserts it falls under one of the two prefixes
     `agent-entrypoint.sh` actually chowns: statically greps
     `agent-entrypoint.sh` for the two loop headers (`"$WORKSPACE_ROOT"/layers/main/*_ws`
     and `"$WORKTREE_ROOT"/*_ws`) and fails loud if either pattern is absent
     — this is what would have caught #604: section 4b existing with no
     matching entrypoint loop.
   - This is the "every anonymous volume the launcher adds must have a
     corresponding entrypoint chown" invariant from the issue, stated as an
     executable check rather than only in prose.
   - A true container-side smoke test (start the sandbox, write into
     `build/` as the dropped `ros` user) would be the stronger check but
     needs a live Docker dispatch harness; per the issue's own
     recommendation and the review's Action 2, that's left as a documented
     follow-up rather than blocking this PR (noted in Open Questions below
     in case the user wants it folded in now instead).
4. **Add the scope caveat** to `test_layer_sourcing.sh`'s Check 4 header
   comment (lines 18-25) and its `AGENTS.md` script-table entry (line 582),
   noting explicitly that Check 4 only stats `layers/main/*_ws` on the
   **host** and does not cover worktree-scoped `*_ws` dirs or **container**-side
   (anonymous-volume) ownership — the gap #604 exposed. This keeps the next
   reader from assuming Check 4 is exhaustive (review's Consequences note:
   the `AGENTS.md` table already slightly overstates Check 4's coverage).
5. **Run the fix path once locally** (mkdir a scratch `*_ws/build` etc. and
   confirm the new loop's logic — chown vs. mkdir+chown branches — by
   reading, since a full container dispatch isn't available in this
   worktree) and run `make test-scripts` to confirm the new test and the
   existing `test_docker_run_mount_args.sh` both pass.

## Files to Change

| File | Change |
|------|--------|
| `.devcontainer/agent/agent-entrypoint.sh` | Add second ownership-fix loop for `$WORKTREE_ROOT/*_ws/{build,install,log}`, mirroring the `layers/main` loop and the `[ -d ] && [ ! -L ]` guard from `docker_run_agent.sh` section 4b; comment carries the "anonymous volume inits from image, not host mkdir" rationale. |
| `.agent/scripts/tests/test_entrypoint_chown_coverage.sh` (new) | `--print-mounts`-driven static invariant check: every worktree/layer `*_ws` mount path the launcher adds has a matching entrypoint chown-loop prefix. |
| `.agent/scripts/test_layer_sourcing.sh` | Extend Check 4's header comment (lines 18-25) with a one-line scope caveat: host-side, `layers/main`-only, not container-side. |
| `AGENTS.md` | Extend the `test_layer_sourcing.sh` script-table row (line 582) with the same scope caveat so the table isn't overstated. |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Test what breaks | This fix ships with the `--print-mounts`-driven invariant test (step 3), so the mount↔chown pairing is enforced going forward rather than left as prose — directly answers the review's "Action needed" flag. |
| Enforcement over documentation | The issue's proposed invariant ("every anonymous volume added must have a matching entrypoint chown") is implemented as a script check, not just a comment — moves it from Watch to enforced. |
| A change includes its consequences | Includes the `test_layer_sourcing.sh` / `AGENTS.md` caveat so documentation doesn't imply broader coverage than Check 4 actually has. |
| Improve incrementally | Second entrypoint loop mirrors the first exactly; new test mirrors `test_docker_run_mount_args.sh`'s existing fixture pattern rather than inventing a new one. No scope creep into the separate "should a dispatched agent see host-built layer installs" question (explicitly out of scope per the issue). |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0002 — Worktree isolation | Yes | New entrypoint loop reuses the `[ -d ] && [ ! -L ]` guard so a layer worktree's symlinked `*_ws` siblings are not double-chowned or reached-through into `layers/main`. |
| 0016 — Runtime vs. baked layer chaining | Watch | Not directly changed, but its regression guard (`test_layer_sourcing.sh`) gets the scope caveat so its coverage claim stays accurate after this fix lands. |
| 0004/0005 — Enforcement hierarchy / layered enforcement | Yes | The new `--print-mounts`-driven test adds a script-layer enforcement point for the mount↔chown invariant, addressing the review's flagged gap rather than deferring it. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `agent-entrypoint.sh` ownership step | Comment/rationale text so future edits to section 4b don't repeat the #604 mistake | Yes (step 2) |
| Mount↔chown invariant | Executable regression test | Yes (step 3) |
| `test_layer_sourcing.sh` Check 4 scope claim | `AGENTS.md` script-table row describing it | Yes (step 4) |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): `AGENTS.md`'s `test_layer_sourcing.sh`
  script-table row (line 582) currently reads as if Check 4 covers
  "mountpoint ownership" generally; it must be narrowed to host-side,
  `layers/main`-only, per step 4 above.
- **Agent-instruction candidates** (proposals only): none beyond what's
  already captured in the code comments this plan adds — the "anonymous
  volume inits from the image, not the host dir" Docker mechanic is subtle
  enough that a short note in `.agent/knowledge/` (e.g. a Docker/devcontainer
  gotchas doc, if one exists) could be worth proposing separately, but that's
  an operator decision outside this PR's scope.

## Open Questions

- Should the container-side smoke test (start the real sandbox, write into
  `build/` as the dropped `ros` user) be added in *this* PR instead of
  deferred, given this is the second time this exact area has shipped
  without regression coverage (#602 → #604)? Plan defaults to deferring it
  as a documented follow-up per the issue's own recommendation, but flagging
  for explicit sign-off since "the permanent solve is five minutes away"
  language in `AGENTS.md`'s Quality Standard cuts the other way if a live
  dispatch harness turns out to be readily available.

## Estimated Scope

Single PR — one entrypoint script change, one new regression test, two doc
touch-ups (`test_layer_sourcing.sh` comment + `AGENTS.md` row). No package
interfaces, params, topics, or ROS build changes.
