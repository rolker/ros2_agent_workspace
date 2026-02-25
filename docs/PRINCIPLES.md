# Guiding Principles

These principles inform every change to the workspace — not just the current effort.
They serve as review criteria for PRs and design decisions.

## 1. Workspace vs. project separation

Workspace infrastructure is generic ROS 2 tooling; project-specific content belongs in
project repos. The workspace should be useful to any ROS 2 project, not coupled to a
single one.

## 2. Primary framework first, portability where free

Use the full capabilities of the active framework (currently Claude Code). Where rules
are naturally framework-agnostic — build commands, coding conventions, architecture —
express them portably. Don't hobble the primary tool for the sake of frameworks that
aren't in use.

## 3. Capture decisions, not just implementations

Use ADRs or equivalent so future agents and humans know *why*, not just *what*. Design
decisions buried in issue comments or commit history get accidentally reverted.

## 4. Enforcement over documentation

Rules that matter must be enforced mechanically — CI, hooks, branch protection — not
just written down. Documentation tells you what to do; enforcement ensures it happens.

## 5. Radical simplicity

Resist adding process, files, or tools unless the pain they solve is concrete and
demonstrated. The right amount of complexity is the minimum needed for the current
situation.

## 6. Test what breaks

Test logic and interfaces, not framework glue. Prioritize tests that catch
regressions which matter — sensor failures, timing issues, degraded conditions.
Don't chase coverage targets; focus on the failure modes that are hard to
find in the field.

## 7. Improve incrementally

Deliver small, reviewed changes rather than large rework efforts. Each change
should leave the workspace better than it found it without requiring a
comprehensive redesign.

## 8. Workspace improvements cascade to projects

When we invest in improving workflow quality — PR templates, CI patterns, hooks,
documentation standards — the benefit shouldn't stop at the workspace repo boundary.
Design improvements to be portable across repos from the start; the workspace serves
as the reference implementation, project repos adopt what fits.
