---
issue: 520
---

# Issue #520 — Bake layer rosdep dependencies into the agent Docker image at build time

## Issue Review
**Status**: complete
**When**: 2026-06-14 10:20 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Issue**: #520
**Comment**: https://github.com/rolker/ros2_agent_workspace/issues/520#issuecomment-4701987031
**Scope verdict**: well-scoped

### Actions
- [ ] Update `.devcontainer/agent/README.md` to document dep-baking and rebuild triggers (consequences).
- [ ] Revisit the `docker_run_agent.sh` row in AGENTS.md Script Reference if `--build` semantics change.
- [ ] Give the manifest staging dir a deterministic name, gitignore it, and regenerate fresh each `--build` (no stale bake).
- [ ] Add a Dockerfile comment explaining the manifests-only idiom (source is mounted, not baked).
- [ ] State the ADR-0003 nuance in the PR: image is a per-workspace dep snapshot, not project-agnostic at runtime.
- [ ] Manually verify both paths: clean rebuild → all layers skip; added dep → only delta installs.
- [ ] Confirm the entrypoint loop still falls through to launch-time install for a layer added after the last image build.

## Plan Authored
**Status**: complete
**When**: 2026-06-14 10:32 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-520/plan.md` at `1b4d985`
**PR**: https://github.com/rolker/ros2_agent_workspace/pull/521 (`[PLAN]` prefix)
**Phases**: single

### Open questions
- [x] rosdep cache ownership → **add a root-side `rosdep update`** before the bake install, keep the user-side update (resolved with user 2026-06-14).
- [x] Build-failure posture → **best-effort + log** (`|| true`), matches the #489 entrypoint (resolved with user 2026-06-14).

## Plan Review
**Status**: complete
**When**: 2026-06-14 11:16 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context)) (in-context — author self-review)

**Plan**: `.agent/work-plans/issue-520/plan.md` at `815fd42`
**PR**: https://github.com/rolker/ros2_agent_workspace/pull/521
**Verdict**: changes-requested

### Findings
- [ ] (must-fix) Manifest-gather glob `layers/main/*_ws/src/*/package.xml` is non-recursive — it catches only 21 of 108 actual `package.xml` files; multi-package repos (marine_control, unh_marine_autonomy, udp_bridge, vrx, etc.) keep their manifests in subdirs, so most layer deps would NOT bake and the launch-time apt storm would persist for them, defeating the goal. Gather recursively (`find <src> -name package.xml`) preserving structure. — `plan.md:28`
- [ ] (suggestion) "102 package.xml files" is inaccurate (shallow glob=21, recursive=108); correct the count and tie it to the recursive gather. — `plan.md:20`
- [ ] (suggestion) Bake step should `apt-get update` before `rosdep install` — the dev-tools blocks end with `rm -rf /var/lib/apt/lists/*` (Dockerfile L33/41/49), so the apt index is empty at the bake point; otherwise installs fail. Make explicit alongside the root-side `rosdep update`. — `plan.md:39`
- [ ] (suggestion) Note that `rosdep install --from-paths` over the staging dir requires each `package.xml` in its own directory (rosdep reads package.xml per-dir); the structure-preserving copy already does this but worth stating as the test invariant. — `plan.md:31`
