---
issue: 522
---

# Issue #522 — Rosdep bake follow-up (commit make agent-build staging, skip ignored packages, resilient bake)

## Implementation
**Status**: complete
**When**: 2026-06-14 13:05 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

### Changes
- **Fix A** `Makefile`: `agent-build` stages via `stage_rosdep_manifests.sh` (trap-cleaned) before `docker build` — the staging step lost in #520 (edited in main tree by mistake, never committed).
- **Fix B** `stage_rosdep_manifests.sh`: skip packages under `COLCON_IGNORE`/`AMENT_IGNORE`/`CATKIN_IGNORE`. Verified: 108→107 staged, `image_warper` (ROS1, COLCON_IGNORE'd, bad `project11` key) excluded.
- **Fix C** `Dockerfile`: all-at-once bake with dynamic `--skip-keys` (compute unresolvable external keys via `rosdep check`, skip only those) — keeps cross-layer `--ignore-src` correct while making one bad key non-fatal. Chosen over literal per-layer, which breaks cross-layer local deps (ui_ws→sensors_ws).
- `README.md`: documented ignore-skip + skip-keys resilience.

### Verification
- `bash -n` clean (helper); Makefile recipe parses (`make -n agent-build`).
- Staging skips image_warper (107 staged, 0 image_warper).
- **Confirmed**: real `make agent-build` (exit 0) staged 107 (skipped image_warper), computed `--skip-keys libqt5opengl5-dev`, and **installed real deps** (ros-jazzy-grid-map-rviz-plugin, zenoh, rqt-*, …) — vs. the #520 build which baked zero. Staging dir trap-cleaned.

### Related
- Completes #520. rqt_operator_tools#62 fixes the one real bad key (libqt5opengl5-dev→libqt5-opengl-dev); skip-keys makes the bake resilient regardless.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-15 03:24 +00:00
**By**: Claude Code Agent (Claude Opus 4.8)
**Verdict**: approved

**Branch**: feature/issue-522 at `01b24a4`
**Mode**: pre-push
**Depth**: Standard (reason: Makefile override-trigger + 137/14 lines, 6 files; no Deep trigger)
**Must-fix**: 0 | **Suggestions**: 4

### Findings
- [x] (suggestion) Makefile trap armed after staging — partial `.rosdep-manifests/` leak window if staging fails; arm before, for parity with docker_run_agent.sh — `Makefile:277-279`. FIXED: trap now armed before the stage call.
- [x] (suggestion) `tr '\n' ' '` leaves trailing space → one inert empty `--skip-keys` entry; trim it — `.devcontainer/agent/Dockerfile:103`. FIXED: appended `sed 's/  *$//'`.
- [ ] (suggestion, declined) sed only catches "Cannot locate rosdep definition"; other unresolvable forms fall through to launch-time (graceful via `|| echo WARNING`) — `.devcontainer/agent/Dockerfile:101-102`. LEFT AS-IS: standard unresolvable-key form; other forms degrade gracefully to launch-time install; broadening risks over-matching.
- [x] (suggestion) `$(CURDIR)` absolute staging path vs relative build-context/trap; works via make cwd, pass explicit STAGE_DIR for consistency — `Makefile:278`. FIXED: Makefile passes an explicit `STAGE_DIR` to the helper and trap.

Static analysis clean (shellcheck, bash -n, make -n). Core ignore-walk + dynamic skip-keys verified correct by two disjoint-lens adversarial passes (one read rosdep source) and a recorded real `make agent-build`. No must-fix.

### Findings resolution
**By**: Claude Code Agent (Claude Opus 4.8 (1M context)) — 2026-06-14. Sandboxed (container) review = approved, 0 must-fix. 3 of 4 suggestions applied; 1 left graceful-by-design. Trim is cosmetic (the empty `--skip-keys` was inert), so no rebuild needed — the prior real `make agent-build` already validated the bake mechanism.
