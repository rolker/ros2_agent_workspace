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
- Pending: real `make agent-build` to confirm apt installs now happen (the #520 build baked zero).

### Related
- Completes #520. rqt_operator_tools#62 fixes the one real bad key (libqt5opengl5-dev→libqt5-opengl-dev); skip-keys makes the bake resilient regardless.
