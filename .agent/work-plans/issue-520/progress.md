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
- [x] (must-fix) Manifest-gather glob `layers/main/*_ws/src/*/package.xml` is non-recursive — it catches only 21 of 108 actual `package.xml` files; multi-package repos (marine_control, unh_marine_autonomy, udp_bridge, vrx, etc.) keep their manifests in subdirs, so most layer deps would NOT bake and the launch-time apt storm would persist for them, defeating the goal. Gather recursively (`find <src> -name package.xml`) preserving structure. — `plan.md:28` → **FIXED**: Approach step 1 now specifies a recursive `find`, per-dir staging, and the 21-vs-108 rationale (verified live: shallow=21, recursive=108).
- [x] (suggestion) "102 package.xml files" is inaccurate (shallow glob=21, recursive=108); correct the count and tie it to the recursive gather. — `plan.md:20` → **FIXED**: Context now reads 108 (recursive).
- [x] (suggestion) Bake step should `apt-get update` before `rosdep install` — the dev-tools blocks end with `rm -rf /var/lib/apt/lists/*` (Dockerfile L33/41/49), so the apt index is empty at the bake point; otherwise installs fail. Make explicit alongside the root-side `rosdep update`. — `plan.md:39` → **FIXED**: Approach step 4 now opens with `apt-get update` and closes with `rm -rf /var/lib/apt/lists/*`.
- [x] (suggestion) Note that `rosdep install --from-paths` over the staging dir requires each `package.xml` in its own directory (rosdep reads package.xml per-dir); the structure-preserving copy already does this but worth stating as the test invariant. — `plan.md:31` → **FIXED**: per-dir invariant stated in Approach step 1 and the Test-what-breaks row.

### Resolution
**By**: Claude Code Agent (Claude Opus 4.8 (1M context)) — 2026-06-14
All four findings addressed in plan revision; plan is now implementation-ready.

## Implementation
**Status**: complete
**When**: 2026-06-14 11:40 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-520/plan.md` at `0365c1b` (implemented as written; no material drift)

### Changes
- `.agent/scripts/docker_run_agent.sh` — recursive manifest-staging block in the `--build` path; gathers every layer `package.xml` (`find`, per-dir, structure preserved) into `.devcontainer/agent/.rosdep-manifests/`, regenerated fresh each build.
- `.devcontainer/agent/Dockerfile` — root-side bake before `USER $USERNAME`: `COPY .rosdep-manifests/` → `apt-get update` → `rosdep update` → `rosdep install --from-paths /tmp/rosdep-manifests` → cleanup. Best-effort + log on each step. Kept the user-side `rosdep update`.
- `.devcontainer/agent/.dockerignore` — new; constrains build context (excludes README.md / cruft).
- `.gitignore` — ignore `.devcontainer/agent/.rosdep-manifests/`.
- `.devcontainer/agent/README.md` — new "Dependency Baking" section + rosdep-failures troubleshooting update.
- `AGENTS.md` — `docker_run_agent.sh` Script Reference row notes `--build` stages manifests.

### Verification
- `bash -n docker_run_agent.sh` passes.
- Staging loop run standalone against the live workspace: **108/108** manifests staged, structure preserved, one `package.xml` per dir (rosdep `--from-paths` invariant holds) — confirms the review-plan must-fix (recursive gather) is satisfied; a shallow glob would have staged 21.
- `.rosdep-manifests/` confirmed gitignored; no staging artifact in the worktree.
- Not yet exercised: a full `docker build` (slow; needs Docker + network). Recommend the host run `make agent-build` once to confirm the bake resolves and a subsequent launch reports "dependencies satisfied — skipping".

### Next
- [ ] `/review-code` (pre-push) before pushing.
- [ ] Host smoke test: `make agent-build` then a dispatch launch, confirm the launch-time rosdep check skips.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-14 12:12 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))
**Verdict**: approved

**Branch**: feature/issue-520 at `7c0a47b`
**Mode**: pre-push
**Depth**: Standard (reason: ~105 LoC across 6 infra files; touches AGENTS.md governance file; plan exists)
**Must-fix**: 0 | **Suggestions**: 2 (1 fixed, 1 declined)

### Findings
- [x] (suggestion) Host-side staging dir `.devcontainer/agent/.rosdep-manifests/` left in working tree (and on build failure) — `docker_run_agent.sh`. FIXED: scoped EXIT trap + post-build `rm`; verified cleaned on success and failure paths.
- [ ] (suggestion, declined) Hard-fail the build on `apt-get update` failure instead of best-effort — `Dockerfile:91`. DECLINED: contradicts the user's settled "best-effort + log" decision (2026-06-14); a transient mirror blip degrades to launch-time install rather than blocking every rebuild.

### Specialists
- Static analysis: shellcheck passed (pre-commit hook at commit time).
- Claude Adversarial Lens A (logic): no correctness bugs; goal achieved (recursive `--from-paths`, best-effort RUN exits 0, empty-COPY safe).
- Claude Adversarial Lens B (systemic/safety): isolation sound (only text manifests enter context; bake root is build-time only), caching sound (content-hash COPY), two suggestions above.
- Copilot Adversarial: off (default; not opted in).
