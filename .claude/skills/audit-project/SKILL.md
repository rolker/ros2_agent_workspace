---
name: audit-project
description: Check a project repo against workspace and project-level conventions. Reports governance coverage, documentation gaps, and test status.
---

# Audit Project

## Usage

```
/audit-project [<repo-name>]
```

If no repo name is given, audit the project repo in the current directory
(when working in a layer worktree).

## Overview

**Lifecycle position**: Utility/periodic — run before or after repo work to
check project-level governance. Not tied to the per-issue lifecycle.

Check a project repo against workspace standards and its own governance docs.
Reports what's present, what's missing, and what may have drifted. Useful
for onboarding to a repo, identifying documentation gaps, or verifying
governance adoption.

**Not the same as `audit-workspace`** — that checks workspace-level governance.
This checks a single project repo.

## Steps

### 1. Identify the repo

If a repo name is given, find it under `layers/main/*/src/<repo-name>`.
If not, use the current directory. Verify it's a valid project repo
(has at least one `package.xml`).

```bash
# Find repo location
find layers/main/*/src/<repo-name> -maxdepth 0 -type d 2>/dev/null
```

### 2. Check governance coverage

Using the governance template (`.agent/templates/project_governance.md`)
as reference, check what exists:

| Item | Status | Path |
|---|---|---|
| `.agents/README.md` | Present / Missing | ... |
| `PRINCIPLES.md` | Present / Missing | ... |
| `ARCHITECTURE.md` | Present / Missing | ... |
| `docs/decisions/` | Present / Missing (N ADRs) | ... |
| `.agents/workspace-context/` | Present / Missing | ... |

This is a **coverage report**, not a mandate — not every repo needs full
governance. But missing items should be noted.

### 3. Check agent guide quality

If `.agents/README.md` exists, check it against the template
(`.agent/templates/project_agents_guide.md`):

- Does it have the expected sections? (Package inventory, layout,
  architecture, build & test, pitfalls)
- Are empty sections present? (Should be removed per template instructions)
- Do listed file paths actually exist in the repo?
- Does the package inventory match actual `package.xml` files?

### 4. Check package metadata

For each `package.xml` in the repo:

- Is the description filled in (not empty or placeholder)?
- Are dependencies listed?
- Does it have a license?
- Is the maintainer field populated?

### 5. Check test status

For each package:

- Do test files exist? (`test/`, `tests/`, `*_test.py`, `*_test.cpp`)
- If available, run or report last known test results:

```bash
# env.sh must be sourced in the same shell — agents run each command in a fresh subprocess
source .agent/scripts/env.sh && cd layers/main/<layer>_ws && colcon test --packages-select <package> && colcon test-result --verbose
```

Report test existence and pass/fail, not test quality.

### 6. Check documentation

- Does a top-level `README.md` exist?
- Do packages have individual READMEs?
- Are launch files documented?
- Are custom message/service/action files documented?

### 7. Cross-reference with workspace

- Is this repo listed in a `.repos` config file?
- Is it in the expected layer?
- Does the workspace's `.agent/project_knowledge/` symlink (pointing to
  `.agents/workspace-context/`) include content from this repo?

## Report Format

```markdown
## Project Audit: <repo-name>

**Location**: `layers/main/<layer>_ws/src/<repo-name>`
**Packages**: N packages (list)

### Governance Coverage

| Item | Status |
|---|---|
| `.agents/README.md` | Present / Missing |
| `PRINCIPLES.md` | Present / Missing |
| ... | ... |

### Agent Guide

<findings if .agents/README.md exists, or "No agent guide — consider
creating one with the project_agents_guide.md template">

### Package Metadata

| Package | Description | License | Maintainer | Tests |
|---|---|---|---|---|
| `pkg_name` | OK / Missing | OK / Missing | OK / Missing | Exist / Missing |

### Documentation

| Item | Status |
|---|---|
| Top-level README | Present / Missing |
| ... | ... |

### Workspace Integration

| Check | Status |
|---|---|
| Listed in .repos | Yes / No |
| Correct layer | Yes / No |
| ... | ... |

### Recommended Actions

- [ ] <specific action items>
```

## Guidelines

- **Report, don't fix** — identify gaps, don't fill them. Fixes should be
  separate issues.
- **Coverage, not quality** — check what exists, not whether it's good.
  "README exists" is objective; "README is well-written" is subjective.
- **Flag adoption level** — reference the governance template's adoption
  levels (minimal/standard/full) and note where this repo falls.
- **Don't run tests by default** — only run tests if the user asks. Just
  check whether test files exist.
