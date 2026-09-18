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

Both branches below must leave `REPO_PATH` and `REPO_MODE` set — every later
step reads them, and an unset `REPO_PATH` turns step 2's currency check into a
read of `/AGENTS.md`.

The scripts are addressed through the **main workspace root**, not a relative
path: `.agent/scripts/` does not exist beside you in a layer worktree (you are
inside the project repo there), and `configs/manifest` exists only in the main
checkout.

```bash
# The workspace root is the nearest ancestor that HOLDS the workspace
# (`.agent/scripts/` + `configs/`) — not whatever repo you happen to be
# standing in. `workspace_root.sh` has the last word: it validates
# $WORKSPACE_ROOT when set, and hops a worktree to the MAIN checkout, where
# `layers/` and the `configs/manifest` symlink actually live. The walk is
# inline because a script cannot be called from a directory not yet found —
# and it STARTS at $WORKSPACE_ROOT when that is set, which is what makes "set
# WORKSPACE_ROOT" a real remedy: the variable is read only *inside* the
# script, so a walk that always began at $(pwd) could never reach a copy of
# the script to read it, and the remedy printed below would be inert on
# exactly the path that prints it. `[ -f ]` + `bash` rather than `[ -x ]`:
# exec bits are lost on a noexec mount, an unpacked archive or a CIFS share,
# and a missing +x is not a reason to walk past the workspace. $WORKSPACE_ROOT
# is normalised to an ABSOLUTE path before the walk: `dirname .` is `.`, so a
# relative value would make the loop below spin forever. A value that cannot
# be entered at all falls back to $(pwd) so the walk still terminates — the
# failure is then reported below, naming the $WORKSPACE_ROOT that was set.
d=$(cd "${WORKSPACE_ROOT:-$(pwd)}" 2>/dev/null && pwd) || d=$(pwd); ROOT=""
while [ "$d" != "/" ]; do
    if [ -f "$d/.agent/scripts/workspace_root.sh" ]; then
        ROOT=$(bash "$d/.agent/scripts/workspace_root.sh") || ROOT=""
        break
    fi
    d=$(dirname "$d")
done
```

An empty `$ROOT` means no workspace was found above you (a project repo cloned
somewhere else entirely). The **named-repo** branch below cannot run without
one — report `FAILED: no workspace root above <where the walk started> — run
the audit from the workspace, or set WORKSPACE_ROOT to one` and stop. The **current-directory** branch
still works: it reads the tree in front of it, and the two layer-dependent
checks report SKIPPED exactly as they do in `clone` mode.

This replaces an earlier `git --git-common-dir` derivation that answered with
*whatever repo you were standing in*: in a layer worktree that is the project
repo's own root, where `.agent/scripts/` and `configs/` do not exist — so
`resolve_repo_checkout.sh` was unfindable and the `case` below misread a
genuine layer checkout as mode `clone`. Asking the operator to remember to pass
the root is not a fix; finding it is.

**No repo name given** — audit the current directory. Verify it is a project
repo (at least one `package.xml`) before auditing it, and **observe** the mode
rather than asserting it: the current directory is only a `layer` checkout if
it actually sits under a layer tree. A project repo cloned somewhere else — or
this skill re-entered on a resolver clone — is mode `clone`, and step 8's
"correct layer" check must report SKIPPED there rather than answering from an
assumption it never tested.

```bash
REPO_PATH=$(pwd)
if ! find "$REPO_PATH" -maxdepth 2 -name package.xml -print -quit | grep -q .; then
    echo "FAILED: $REPO_PATH has no package.xml — not a project repo"
    exit 1
fi
case "$REPO_PATH" in
    "$ROOT"/layers/*/src/*) REPO_MODE=layer ;;
    *)                      REPO_MODE=clone ;;
esac
```

**A repo name given** — resolve it with `resolve_repo_checkout.sh`, which
**does not assume `layers/` exists**: it prefers an existing layer checkout and
otherwise shallow-clones the URL the workspace manifests declare, at the
version they pin. `layers/` is gitignored and absent in every worktree, fresh
clone and container, so audits run from those places would otherwise find
nothing.

```bash
# Prints "<path><TAB><layer|clone>"; every failure exits non-zero with a reason
if [ -z "$ROOT" ]; then
    echo "FAILED: no workspace root above ${WORKSPACE_ROOT:-$(pwd)} — run from the workspace, or set WORKSPACE_ROOT to one"
    exit 1
fi
if ! resolved=$("$ROOT/.agent/scripts/resolve_repo_checkout.sh" <repo-name>); then
    # exit 2 = usage (including a repo name that is not a single path segment)
    # exit 3 = no repo manifest configured at all (run `make setup-all`)
    # exit 4 = repo not listed in any manifest that was read
    # exit 5 = no checkout produced: a clone/refresh failed, the temp dir,
    #          cache directory or per-repo lock could not be set up, or a
    #          helper it needs locally (redact.sh, manifest_fallback.sh) is
    #          missing, will not load, or answered unrecognisably
    # exit 6 = manifest unreadable, or the entry is malformed (no url:, a url
    #          in no recognised form, or an unsafe version:)
    # exit 7 = declared in two manifests with a conflicting url or version:
    #          pin — two different trees, so do not guess
    echo "FAILED: could not resolve <repo-name> (see stderr)"
    exit 1
fi
REPO_PATH=${resolved%%$'\t'*}
REPO_MODE=${resolved##*$'\t'}
```

Each of those exits is a distinct FAILED reason to report — none of them is
"repo not found", and none is a reason to continue with an unset `REPO_PATH`.

Record `REPO_MODE` — the audit's report header names it, and it tells the
reader what the audited tree *is*. A `clone` was chosen by the manifests: their
url, their pinned `version:`. A `layer` checkout is the operator's working tree
taken **as is** — the resolver does not compare its origin or its checked-out
ref against the manifests (a healthy workspace differs legitimately in url form
and sits on feature branches, so the comparison would manufacture findings), so
a `layer` finding is a finding about the code on this host, which may be ahead
of, behind, or on a different branch from what the manifests pin. Name the mode
in the report header for exactly that reason.

In `clone` mode
the two genuinely layer-dependent checks (the optional `colcon test` run in
step 5, and step 8's "correct layer") must report **SKIPPED (no layer
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
# Addressed through $ROOT for the same reason step 1 is: a relative
# .agent/scripts/ or layers/ does not exist beside you in a layer worktree.
# Step 1's $ROOT is the real workspace root (workspace_root.sh), including
# from inside a layer worktree, so these paths resolve there too — but check
# it is non-empty first: no workspace was found means no layer workspace to
# build in, which is SKIPPED, not a failure to paper over.
# setup.bash must be sourced in the same shell — agents run each command in a fresh subprocess
source "$ROOT/.agent/scripts/setup.bash" && cd "$ROOT/layers/main/<layer>_ws" \
    && colcon test --packages-select <package> && colcon test-result --verbose
```

Report test existence and pass/fail, not test quality.

### 6. Check documentation

- Does a top-level `README.md` exist?
- Do packages have individual READMEs?
- Are launch files documented?
- Are custom message/service/action files documented?

### 7. Check planning documents

Probe the four planning-document kinds named in the expected-location table
in [`docs/design/planning_document_vocabulary.md`](../../../docs/design/planning_document_vocabulary.md)
(vision, roadmap, decision, health) with `.agent/scripts/planning_doc_probe.sh`,
addressed through `$ROOT` for the same reason step 1 is — `.agent/scripts/`
does not exist beside a project repo checked out under a layer worktree. As
in step 5, check `$ROOT` is non-empty first: no workspace root above you
means the probe script itself is unreachable — that is `SKIPPED`, not a
failure to paper over. Also check the script actually exists **and is
executable** at that path before invoking it — a layer worktree whose main
checkout predates this script landing on `main` would otherwise hit a raw
"No such file" error, and a present-but-non-executable script (e.g. an
`x`-bit lost in a checkout/transfer) would otherwise hit a raw "Permission
denied" instead of the intended `SKIPPED`.

```bash
if [ -z "$ROOT" ]; then
    echo "SKIPPED (no workspace root — planning_doc_probe.sh unreachable)"
elif [ ! -x "$ROOT/.agent/scripts/planning_doc_probe.sh" ]; then
    echo "SKIPPED (planning_doc_probe.sh not found or not executable under $ROOT/.agent/scripts/)"
else
    "$ROOT/.agent/scripts/planning_doc_probe.sh" "$REPO_PATH"
fi
```

This prints four TSV lines (`<kind>\t<present|absent>\t<relative-path>`),
which render as the **Planning Documents** report section below. **This is
descriptive only — absence of any kind is never a finding and never drives a
Recommended Actions entry.** The design draft is explicit that the
expected-location table is a published expectation, not a requirement: a
repo that keeps a planning document somewhere else, or doesn't keep one at
all, is not in violation. Report `Present` / `Not found` — deliberately not
"Missing", the word the Governance Coverage table above uses for items that
do drive a recommendation. A `SKIPPED` probe (either guard above) renders as
a single note in the Planning Documents section instead of the four-row
table — there is no per-kind data to show, and four blank "Not found" rows
would misreport a probe that never ran as one that ran and found nothing.

### 8. Cross-reference with workspace

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

### Planning Documents

| Kind | Status | Location |
|---|---|---|
| vision | Present / Not found | `README.md` (Present) / — (Not found) |
| roadmap | Present / Not found | `ROADMAP.md` (Present) / — (Not found) |
| decision | Present / Not found | `docs/decisions` (Present) / — (Not found) |
| health | Present / Not found | `docs/health.md` (Present) / — (Not found) |

<!-- Location matches what planning_doc_probe.sh actually emits: the kind's
     path on Present, and an EMPTY field on Not found — never the expected
     path repeated as if it were found. Render the empty TSV field as "—",
     not as the expected path. -->

<!-- Descriptive only — a repo publishing a planning document somewhere else,
     or not at all, is not in violation. This section never contributes a
     Recommended Actions entry (docs/design/planning_document_vocabulary.md). -->

<!-- If step 7 reported SKIPPED, render that single note here instead of the
     table above — there is no per-kind data from a probe that did not run. -->

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
