# Plan: gather-project-knowledge: fix data fidelity errors in profile generation

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/320

## Context

The first run of `gather-project-knowledge` ([unh_marine_autonomy#96](https://github.com/rolker/unh_marine_autonomy/pull/96))
produced profiles with factual errors in all 3 of 3 spot-checked repos.
Root causes: multi-hop data loss between scan agents → synthesis → profile-writing
subagent, directory names confused with package names, and hallucination under
context pressure. The fix targets the skill definition in
`.claude/skills/gather-project-knowledge/SKILL.md`.

The review comment on the issue recommends considering a mechanical validation
approach (enforcement over documentation) rather than relying solely on agent
discipline.

## Approach

1. **Add structured intermediate format requirement** — After step 2 (scanning),
   require the agent to produce a machine-readable inventory: a markdown table
   mapping each repo to its packages (from `<name>` elements in `package.xml`),
   language, and layer. This table becomes the single source of truth for all
   subsequent profile writing. The key instruction: package names must come from
   `<name>` elements in `package.xml`, never from directory names.

2. **Add validation step before commit** — Add a new step between generation (3)
   and frontmatter (4): "For each profile, glob for `package.xml` files in the
   repo and compare `<name>` elements against the profile's package list. Fix
   any mismatches before proceeding." This is the enforcement-aligned fix.

3. **Reduce handoff hops** — Add an explicit instruction that profile writing
   must reference the structured inventory table directly, not a natural language
   summary. Prohibit delegating profile writing to a subagent that doesn't have
   access to the raw scan data.

4. **Clarify package vs. directory distinction** — Add a guideline explicitly
   warning that repo directory names, subdirectory names, and ROS package names
   are different things. The only authoritative source is `<name>` in `package.xml`.

## Files to Change

| File | Change |
|------|--------|
| `.claude/skills/gather-project-knowledge/SKILL.md` | Revise steps 2-3, add validation step, add guidelines |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Enforcement over documentation | The validation step (glob + compare) provides a mechanical check within the skill workflow. A standalone helper script could further enforce this, but the review comment suggests this as optional — the in-skill validation is the minimum viable enforcement. |
| A change includes its consequences | The bad data in [unh_marine_autonomy#96](https://github.com/rolker/unh_marine_autonomy/pull/96) needs correction. This plan fixes the skill only; the data fix should be a follow-up re-run of the improved skill (separate issue/PR on the project repo). Note this in the PR description. |
| Only what's needed | Three targeted changes to one file. No new scripts, no new abstractions. |
| Capture decisions, not just implementations | The root cause analysis from the issue body should be preserved as a comment block in SKILL.md so future maintainers understand why these constraints exist. |
| Workspace vs. project separation | The skill definition is workspace infra. The output goes to project repos. This fix stays in the workspace repo. Correct. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0003 — Project-agnostic workspace | Yes | Skill remains generic — validation logic uses `package.xml` which is a standard ROS 2 artifact, not project-specific. |
| 0004 — Enforcement hierarchy | Watch | The validation step is instruction-level enforcement (agent reads and follows SKILL.md). A helper script would add mechanical enforcement. Recommend noting in the PR that a helper script is a future option if instruction-level proves insufficient. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `.claude/skills/gather-project-knowledge/SKILL.md` | Framework adapter file; regenerate skills if needed | No — description change is minor; check if `make generate-skills` updates anything |
| Bad data in unh_marine_autonomy#96 | Re-run skill to regenerate correct profiles | No — follow-up issue on project repo |

## Open Questions

- Should the data fix (re-running the improved skill on `unh_marine_autonomy`) be
  scoped into this issue, or tracked as a separate follow-up issue?
- Should we add a helper script (e.g., `validate_project_knowledge.sh`) for mechanical
  enforcement, or is the in-skill validation step sufficient for now?

## Estimated Scope

Single PR — one file changed (SKILL.md), ~30-40 lines added/modified.
