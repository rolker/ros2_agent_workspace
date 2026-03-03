# ADR-0008: Follow ROS 2 official conventions by default

## Status

Proposed

## Context

AI agents working in this workspace repeatedly introduce code that diverges from
established ROS 2 community conventions — non-standard launch file naming, missing
or mismatched license headers, incorrect `package.xml` dependency tags. These are
not controversial decisions; they are well-documented ROS 2 norms that agents should
follow without per-item reminders.

Individual issues tracked specific violations ([#327] launch naming, [#131]
licensing), but the root cause is the same: no umbrella decision states "follow
ROS 2 conventions by default," and the knowledge file that agents consult
(`.agent/knowledge/ros2_development_patterns.md`) was ambiguous on key points.

Documenting each convention as its own ADR would create an ever-growing list of
"do the obvious thing" records. A single umbrella decision is more maintainable
and keeps ADRs focused on genuinely non-obvious choices.

[#327]: https://github.com/rolker/ros2_agent_workspace/issues/327
[#131]: https://github.com/rolker/ros2_agent_workspace/issues/131

## Decision

**Follow ROS 2 official conventions unless a project-specific ADR says otherwise.**

1. **ROS 2 community conventions are the default.** Naming, packaging, licensing,
   message design, launch file structure, and other practices documented in the
   [ROS 2 documentation](https://docs.ros.org/en/rolling/) and relevant REPs are
   presumed correct for this workspace.

2. **Conventions target Rolling.** When the workspace targets a different distro
   (e.g., Jazzy), exceptions are allowed where a newer Rolling convention is not
   yet feasible. Distro-specific differences should be noted in the knowledge file.
   See [#261] for multi-distro support plans.

3. **The workspace summary lives in
   `.agent/knowledge/ros2_development_patterns.md`.** This file captures the
   conventions most relevant to agent work in a concise, actionable format. It is
   a living document — update it when new conventions need emphasis or when agents
   are observed making recurring mistakes.

4. **Deliberate deviations require their own ADR.** If a project or the workspace
   intentionally breaks from a ROS 2 convention, the rationale must be recorded in
   an ADR (workspace-level or project-level). Without an ADR, the convention stands.

5. **When in doubt, match existing code.** If a project repo already follows a
   consistent pattern, match it rather than introducing a mix. If the existing
   pattern conflicts with a ROS 2 convention, file an issue to discuss alignment
   rather than silently "fixing" it.

6. **This ADR is the "instructions" layer.** Per ADR-0004, instructions alone are
   insufficient for rules that matter. Enforcement via pre-commit hooks and CI
   ([#265], [#327]) is expected as separate work following the enforcement
   hierarchy.

[#261]: https://github.com/rolker/ros2_agent_workspace/issues/261
[#265]: https://github.com/rolker/ros2_agent_workspace/issues/265

### Key references

| Source | Covers |
|--------|--------|
| [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Developer-Guide.html) | Package structure, code style, documentation |
| [REP-144](https://ros.org/reps/rep-0144.html) | Package naming |
| [REP-140](https://reps.openrobotics.org/rep-0140/) | Package manifest (`package.xml`) format |
| [REP-2004](https://reps.openrobotics.org/rep-2004/) | Package quality categories |
| [Launch system tutorial](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-system.html) | Launch file naming |
| [Migration guide](https://docs.ros.org/en/jazzy/How-To-Guides/Migrating-from-ROS1/Migrating-Packages.html) | Licensing guidance |

## Consequences

**Positive:**
- Agents have a clear, low-overhead rule: follow ROS 2 conventions, check the
  knowledge file, and match existing project patterns
- New conventions can be reinforced by updating one knowledge file instead of
  creating a new ADR each time
- Deliberate deviations are explicitly recorded, preventing accidental reversion

**Negative:**
- The knowledge file becomes load-bearing — if it is wrong or incomplete, agents
  will follow bad guidance (mitigated: reviewed via normal PR process; periodic
  health sweep [#235] checks for drift)
- "Follow the official docs" is broad; agents may still miss conventions not
  called out in the knowledge file (mitigated: update the file when a recurring
  mistake is observed)
- Conventions may shift between ROS 2 distros; the knowledge file must be
  reviewed when the workspace targets a new distro

[#235]: https://github.com/rolker/ros2_agent_workspace/issues/235
