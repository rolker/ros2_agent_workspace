# PR Template — Reference for Project Repos

Copy this to `.github/PULL_REQUEST_TEMPLATE.md` in a project repo and adapt
the architecture impact checklist to reflect that project's concerns.

The common sections (Summary, Related issue, What changed, Testing) should
stay consistent across repos. The architecture impact checklist is where
project-specific items go.

## Adapting the architecture checklist

Replace the example items below with concerns relevant to the project. Good
checklist items are ones where forgetting to update something causes real
problems downstream — broken interfaces, silent behavior changes, stale docs.

Examples by project type:
- **ROS 2 multi-package repo**: message/service definitions, node interfaces
  (topics, parameters, services), inter-package dependencies, launch structure
- **Single-package repo**: public API changes, configuration format, dependencies
- **Library repo**: API surface, ABI compatibility, versioning

Keep it short (5-8 items max). If the checklist is too long, people ignore it.

---

## Template starts below

```markdown
## Summary

<!-- Brief description of what changed and why. -->

## Related issue

<!-- Link to the issue this PR addresses. -->
Closes #

## What changed

<!-- Bullet list of the key changes. -->

-

## Testing

<!-- How was this tested? colcon test, manual verification, etc. -->

-

## Architecture impact

<!-- Replace these example items with project-specific concerns. -->

- [ ] No architecture impact (routine change within existing patterns)
- [ ] Architecture-relevant change (check all that apply below):
  - [ ] Changes message, service, or action definitions
  - [ ] Changes node interfaces (topics, parameters, services)
  - [ ] Changes inter-package dependencies
  - [ ] Changes launch file structure or default configuration

### If architecture-relevant:
- [ ] README or package documentation updated
- [ ] ADR created (if a new architectural decision)
```
