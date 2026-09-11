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
(when working in a layer worktree). If one is given, the repo is located with
`.agent/scripts/resolve_repo_checkout.sh` — a layer checkout when one exists,
otherwise a shallow clone — so the audit also runs from a worktree, a fresh
clone, or a container, none of which have `layers/`.

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

If no repo name is given, use the current directory (a layer worktree) and
treat the mode as `layer`. Verify it's a valid project repo (has at least one
`package.xml`).

If a repo name is given, resolve it with `resolve_repo_checkout.sh`, which
**does not assume `layers/` exists** — it prefers an existing layer checkout
and otherwise shallow-clones the URL the workspace manifests declare. `layers/`
is gitignored and absent in every worktree, fresh clone and container, so
audits run from those places would otherwise find nothing.

```bash
# Prints "<path><TAB><layer|clone>"; every failure exits non-zero with a reason
if ! resolved=$(.agent/scripts/resolve_repo_checkout.sh <repo-name>); then
    # exit 3 = no repo manifest configured at all (run `make setup-all`)
    # exit 4 = repo not listed in any manifest that was read
    # exit 5 = clone/refresh failed    exit 6 = manifest unreadable
    echo "FAILED: could not resolve <repo-name> (see stderr)"
    exit 1
fi
REPO_PATH=${resolved%%$'\t'*}
REPO_MODE=${resolved##*$'\t'}
```

Record `REPO_MODE` — the audit's report header names it, and in `clone` mode
the two genuinely layer-dependent checks (the optional `colcon test` run in
step 5, and step 7's "correct layer") must report **SKIPPED (no layer
checkout)**, never OK. A check that could not run is never rendered as a pass.
Everything else — governance coverage, the agent guide, package metadata,
test-file existence, documentation — reads the working tree and is unaffected
by the mode.

### 2. Check governance coverage

Using the governance template (`.agent/templates/project_governance.md`)
as reference, check what exists:

| Item | Status | Path |
|---|---|---|
| `AGENTS.md` (root) | Present / Missing / Stale | ... |
| `.agents/README.md` | Present / Missing | ... |
| `PRINCIPLES.md` | Present / Missing | ... |
| `ARCHITECTURE.md` | Present / Missing | ... |
| `docs/decisions/` | Present / Missing (N ADRs) | ... |
| `.agents/workspace-context/` | Present / Missing | ... |

For the root `AGENTS.md`, also run the **currency check**: the file must
contain the template's stable `## Quality Standard` heading (the marker
`.agent/templates/project_agents_md.md` instructs repos to keep verbatim):

```bash
AGENTS_FILE="$REPO_PATH/AGENTS.md"
if [ ! -f "$AGENTS_FILE" ]; then
    echo "MISSING: no root AGENTS.md"
elif ! grep -q '^## Quality Standard' "$AGENTS_FILE"; then
    echo "STALE: Quality Standard section missing"
fi
```

Report `Stale` when the file exists but the marker is absent — it predates
the template or lost its standalone review context (ADR-0017).

This is a **coverage report**, not a mandate — not every repo needs full
governance. But missing items should be noted, and a missing root
`AGENTS.md` means Copilot code review runs uninstructed on that repo.

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

Test-file existence is the default check and works in either mode — it reads
the working tree. Per the Guidelines below, tests are **not** run unless the
user asks.

Only when the user asks, and only in `layer` mode, run them — `colcon` needs a
built layer workspace, which a clone is not:

```bash
# layer mode only; in clone mode report "SKIPPED (no layer checkout)"
# setup.bash must be sourced in the same shell — agents run each command in a fresh subprocess
source .agent/scripts/setup.bash && cd layers/main/<layer>_ws && colcon test --packages-select <package> && colcon test-result --verbose
```

Report test existence and pass/fail, not test quality.

### 6. Check documentation

- Does a top-level `README.md` exist?
- Do packages have individual READMEs?
- Are launch files documented?
- Are custom message/service/action files documented?

### 7. Cross-reference with workspace

- Is this repo listed in a `.repos` config file?
- Is it in the expected layer? (`layer` mode only — in `clone` mode there is no
  layer checkout to compare against, so report **SKIPPED (no layer
  checkout)**.)
- Does the workspace's `.agent/project_knowledge/` symlink (pointing to
  `.agents/workspace-context/`) include content from this repo?

## Report Format

```markdown
## Project Audit: <repo-name>

**Location**: `<resolved path>` (`layer` — `layers/main/<layer>_ws/src/<repo-name>`,
or `clone` — `.agent/scratchpad/janitor-repos/<repo-name>`)
**Checkout mode**: layer / clone
**Packages**: N packages (list)

### Governance Coverage

| Item | Status |
|---|---|
| `AGENTS.md` (root) | Present / Missing / Stale |
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
| Correct layer | Yes / No / SKIPPED (no layer checkout) |
| ... | ... |

### Recommended Actions

- [ ] <specific action items>
```

When the root `AGENTS.md` is Missing or Stale, list it as a distinct
recommended action (fix = instantiate/refresh from
`.agent/templates/project_agents_md.md`, e.g. via `onboard-project`) —
don't fold it into a generic "improve governance" item.

## Guidelines

- **Report, don't fix** — identify gaps, don't fill them. Fixes should be
  separate issues.
- **Coverage, not quality** — check what exists, not whether it's good.
  "README exists" is objective; "README is well-written" is subjective.
- **Flag adoption level** — reference the governance template's adoption
  levels (minimal/standard/full) and note where this repo falls.
- **Don't run tests by default** — only run tests if the user asks. Just
  check whether test files exist.
