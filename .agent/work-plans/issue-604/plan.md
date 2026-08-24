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

1. **Add the second entrypoint chown loop.** *(Implemented with a deviation:
   both loops moved out of `agent-entrypoint.sh` into a new
   `.devcontainer/agent/fix-volume-ownership.sh` that the entrypoint calls.
   Reason: the container-side smoke test the user opted into below has to
   exercise the ownership step alone — running the full entrypoint would drag
   in the ROS sourcing and rosdep pass. The loops are otherwise exactly as
   planned.)* Originally: in
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
   - For each mounted path, asserts it is reached by the ownership loops. The
     *as-built* check compares the launcher's mounted set against the set the
     real loops in `fix-volume-ownership.sh` reach (run over the same fixture
     with `chown` stubbed), **superseding this plan's original grep-for-loop-
     headers design** — a reformatted loop would have failed that grep for no
     real reason, while set comparison still fails on a genuinely uncovered
     mount. This is what would have caught #604: section 4b existing with no
     matching ownership loop.
   - This is the "every anonymous volume the launcher adds must have a
     corresponding entrypoint chown" invariant from the issue, stated as an
     executable check rather than only in prose.
   - **A container-side test is included in this PR** (user decision at
     the plan checkpoint; the review had verified that the deferral premise —
     "no live dispatch harness available" — was false: docker works here and
     the agent image is built). As built it is end-to-end, not a smoke test of
     the ownership script alone: it runs the *working tree's*
     `agent-entrypoint.sh` as the image ENTRYPOINT over real anonymous volumes
     and probes both mountpoints as the user the entrypoint hands off to. That
     scope change came from the pre-push review, which mutation-proved the
     earlier direct-invocation version could not detect #604 — the defect was
     in the entrypoint's *argument list*, which a direct call bypasses. It
     skips cleanly with no Docker, no image, or a non-local daemon, keeping
     `run_script_tests.sh` hermetic in CI.
   - Both layers are mutation-checked. Deleting loop 2 of
     `fix-volume-ownership.sh` fails both layers; deleting the
     `"${WORKTREE_ROOT:-}"` argument at the entrypoint call site — #604 itself
     — fails the container layer (and, by construction, cannot be seen by the
     static one).
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
| `.devcontainer/agent/fix-volume-ownership.sh` (new) | Both ownership loops, extracted from the entrypoint so the container smoke test can run them alone. Header records the load-bearing mechanic: an anonymous volume initializes from the image, not the host dir, so only this chown makes it writable. |
| `.devcontainer/agent/Dockerfile` | `COPY` the new script; add `STARTUP_SCRIPTS_SHA` build arg + `org.ros2-agent.startup-scripts-sha` label. |
| `.agent/scripts/docker_run_agent.sh` | Correct section 4b's rationale (the host-side mkdir is *not* what makes section 4 work — the entrypoint chown is), and add the launch-time stale-image warning. |
| `Makefile` | `agent-build` delegates to `docker_run_agent.sh --build-only`. As first written it ran its own `docker build` with an inline copy of the digest formula over a different directory; the review showed that stamps a digest the launcher can never match from a worktree build (a permanent false "stale"). One builder, one formula, one directory. |
| `.devcontainer/agent/README.md` | Document that startup-script changes need a rebuild, and the warning that detects it. |
| `.devcontainer/agent/agent-entrypoint.sh` | Call the extracted script; original plan: add second ownership-fix loop for `$WORKTREE_ROOT/*_ws/{build,install,log}`, mirroring the `layers/main` loop and the `[ -d ] && [ ! -L ]` guard from `docker_run_agent.sh` section 4b; comment carries the "anonymous volume inits from image, not host mkdir" rationale. |
| `.agent/scripts/tests/test_entrypoint_chown_coverage.sh` (new) | Two layers: a `--print-mounts`-driven static invariant check (mounted set ⊆ chowned set, plus bad-root and workspace-worktree cases) and a container end-to-end run of the real entrypoint. |
| `.agent/scripts/tests/test_agent_image_build_paths.sh` (new) | Guards the single build path the staleness marker depends on: `make agent-build` delegates to `--build-only`, the digest has one implementation, the Dockerfile ARG/LABEL name matches what the launcher reads back. |
| `docs/decisions/0016-runtime-vs-baked-layer-chaining.md` | References cross-reference addendum (ADR-0012) scoping the Check-4 coverage claim in its Consequences — the same overstatement narrowed in the script header and the `AGENTS.md` row. |
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

- ~~Should the container-side smoke test be added in *this* PR instead of
  deferred?~~ **Resolved at the plan checkpoint: include it in this PR.** The
  deferral premise was false — `review-plan` verified docker works on this host
  and `ros2-agent-workspace-agent:latest` is already built. Implemented in
  `test_entrypoint_chown_coverage.sh` layer B, skipping cleanly without Docker.

- ~~**New, found during implementation:** the launcher only builds when the image
  is *missing*, so this fix would not have reached anyone with an existing
  image — the container would keep running the baked (broken) entrypoint. Added
  the `STARTUP_SCRIPTS_SHA` label + launch-time staleness warning to close that,
  since a fix that silently doesn't apply is the same failure class the issue is
  about. Flagged for sign-off as scope added beyond the reviewed plan.~~
  **Resolved at the review checkpoint: the guard stays in this PR**, with its
  three reviewed defects repaired here rather than split out — the absent-label
  case now warns (an unmarked image is precisely the pre-#604 population), the
  two build paths were collapsed into one so the digest cannot disagree with
  itself, and the digest helper can no longer abort the launcher it is only
  supposed to warn from.

## Estimated Scope

Single PR — the entrypoint ownership fix (extracted to its own script), one new
two-layer regression test, the stale-image guard, and three doc touch-ups
(`test_layer_sourcing.sh` comment, `AGENTS.md` row, devcontainer README). No
package interfaces, params, topics, or ROS build changes.
