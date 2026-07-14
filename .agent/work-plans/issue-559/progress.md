---
issue: 559
---

# Issue #559 — Fix O(N²) layer sourcing and inverted overlay precedence in setup.bash

## Issue Review
**Status**: complete
**When**: 2026-07-14 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #559
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Summary

The issue identifies two correctness bugs and one performance problem in
`.agent/scripts/setup.bash` and `.agent/scripts/build.sh`, with benchmarks,
root-cause analysis, and a concrete five-step fix. The root causes are verified
by reading the actual scripts:

- **`setup.bash:86`**: sources `install/setup.bash` (baked chained version)
  instead of `install/local_setup.bash`, causing O(N²) Python invocations (one
  full chain expansion per layer). Confirmed in code.
- **`build.sh:78`**: `source "$SCRIPT_DIR/setup.bash" > /dev/null` before
  building any layer loads the full ROS 2 prefix path, so when colcon builds
  `underlay_ws` first, every other layer is already present as a parent — baked
  into `underlay_ws/install/setup.sh` permanently. Confirmed in code.
- **Inverted precedence**: result of sourcing in ascending order with chained
  setup; the fast alternative produces correct canonical order.

### Scope Assessment

**Well-scoped?** Yes — change is bounded to `.agent/scripts/` (setup.bash,
build.sh, worktree_create.sh templates) plus a one-time clean rebuild. Single PR.
**Right repo?** Yes — workspace infrastructure, not project-repo content.
**Dependencies?** None identified. A one-time clean rebuild is a manual step (not
a script dependency), documented in the issue.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Issue documents root cause, benchmarks, and expected outcomes clearly |
| Enforcement over documentation | OK | Fix is behavioral, not documentation-only |
| Capture decisions, not just implementations | Watch | Switching from `install/setup.bash` to `install/local_setup.bash` (runtime vs baked chaining) is a design choice worth an ADR; it isn't currently documented in `docs/decisions/` |
| A change includes its consequences | Watch | Issue mentions sweeping `.agent/` for other `install/setup.bash` references and updating generated worktree scripts; existing worktrees age out. Implementer should track the sweep explicitly — grep result should appear in the PR. `WORKTREE_GUIDE.md` may need a note about the generated-script change |
| Only what's needed | OK | Solves concrete, measured pain (4.5s → 0.5s per agent command); no speculative scope |
| Improve incrementally | OK | Five-step plan is bounded; step 3 (worktree generators) is a natural scope inclusion to prevent regressions on new worktrees |
| Test what breaks | Watch | Verification section lists manual timing/diff steps, not automated tests. The timing regression would be easy to catch with a timed `source` in CI; the precedence correctness could be checked with a script. Consider adding at least one automated check |
| Workspace vs. project separation | OK | All changes in workspace infra scripts |
| Workspace improvements cascade to projects | OK | Generated worktree scripts get the fix; existing worktrees noted as aging out naturally |
| Primary framework first, portability where free | OK | No framework coupling in these shell scripts |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| 0001 — Adopt ADRs | Yes | The switch from baked-chain (`setup.bash`) to runtime-chain (`local_setup.bash`) sourcing is a design decision that future agents need to understand — should be recorded |
| 0002 — Worktree isolation | OK | Standard; use worktree for implementation |
| 0007 — Retain Make with Dependency Tracking | Watch | `build.sh` change should not break Make's stamp-file dependency semantics; verify `make build` still works correctly end-to-end |
| 0013 — progress.md entry-type vocabulary | OK | This entry follows the Issue Review type per ADR-0013 |

### Consequences

Per the consequences map, changing `.agent/scripts/` (setup.bash, build.sh,
worktree_create.sh) requires:
- **AGENTS.md script reference table** — descriptions reference script purpose
  only, not implementation; minor note may still be warranted if the semantics
  change visibly (e.g., "sources `local_setup.bash` per layer").
- **`.agent/WORKTREE_GUIDE.md`** — documents generated worktree scripts; if the
  generated preambles change, the guide should reflect it.
- **Sweep result** — grep for direct `install/setup.bash` sourcing across
  `.agent/` (container/dispatch flows, other hooks) should be documented in the
  PR; this is listed in the issue but not tracked as an explicit checklist item.

### Actions
- [ ] Record the runtime-vs-baked chaining design decision in an ADR (ADR-0001 trigger — strongly recommended before or alongside implementation to prevent accidental reversion by a future agent).
- [ ] Sweep `.agent/` for all direct `install/setup.bash` sourcing; document findings in PR description.
- [ ] Update `.agent/WORKTREE_GUIDE.md` if generated worktree script preambles change.
- [ ] Consider adding an automated timing/correctness check (e.g., timed `source` assertion in CI or a Makefile validate target) to prevent O(N²) regression.
