---
name: issue-triage
description: Scan overlay repositories for GitHub issues, categorize them, flag stale items, and cross-reference with workspace tracking.
---

# Issue Triage

## Usage

```
/issue-triage [--repo <repo-name>] [--stale-days <N>]
```

Without `--repo`, scans all overlay repositories. Default stale threshold is
90 days.

## Overview

**Lifecycle position**: Utility/periodic — run to get a cross-repo view of
open issues, identify stale items, and ensure nothing is falling through the
cracks.

Scans overlay repositories for open GitHub issues using `gh` CLI, categorizes
them by type and priority, flags stale issues, and cross-references with
workspace-level tracking.

## Steps

### 1. Enumerate repositories

Enumerate against the **main workspace root**, never the current directory:
`configs/` lives only in the main checkout, so a worktree-relative run
enumerates **zero** repos on a host that has dozens — and would then trip the
empty-list guard below as a false FAILED with the wrong remedy. This skill is
routinely run from a worktree (directly, and as one of `janitor-sweep`'s four
checks), so this is the normal case, not an edge one.

```bash
# The workspace root is the nearest ancestor that HOLDS the workspace
# (`.agent/scripts/` + `configs/`) — not whatever repo you happen to be
# standing in. `workspace_root.sh` has the last word: it validates
# $WORKSPACE_ROOT when set, and hops a worktree to the MAIN checkout, where
# `layers/` and the `configs/manifest` symlink actually live. The walk is
# inline because a script cannot be called from a directory not yet found.
d=$(pwd); ROOT=""
while [ "$d" != "/" ]; do
    if [ -x "$d/.agent/scripts/workspace_root.sh" ]; then
        ROOT=$("$d/.agent/scripts/workspace_root.sh") || ROOT=""
        break
    fi
    d=$(dirname "$d")
done
if [ -z "$ROOT" ]; then
    echo "FAILED(workspace root: none above $(pwd) — run from the workspace, or set WORKSPACE_ROOT)"
    exit 1
fi
```

(This resolves the workspace from a *layer* worktree too, where the earlier
`git --git-common-dir` form answered with the project repo's own root — which
has neither the manifests nor the scripts this skill goes on to call.)

Enumerate through the fallback, **never** with a bare
`list_overlay_repos.py "$ROOT/..."` call of its own. Where `$ROOT/configs/manifest`
is absent — a fresh clone or a container, where `layers/` does not exist and
`configs/manifest` is a symlink into it — the manifests have to be cloned
first, and an enumeration that runs before that prints `[]` at **exit 0**,
which the empty-list guard below then maps to the one remedy this skill's own
rules forbid. So there is exactly one enumeration site, and it is on the
fallback's success path:

```bash
source "$ROOT/.agent/scripts/manifest_fallback.sh"
if extra=$(manifest_config_dir "$ROOT"); then
    python3 "$ROOT/.agent/scripts/list_overlay_repos.py" ${extra:+--config-dir "$extra"}
else
    case $? in
        3) echo "FAILED: no repo manifest configured — run 'make setup-all'" ;;
        5) echo "FAILED: manifest repo unreachable — the reason is on stderr" ;;
        6) echo "FAILED: manifest refresh — the reason is on stderr" ;;
        *) echo "FAILED: manifest fallback exited unexpectedly" ;;
    esac
fi
```

Every failure arm is terminal — none falls through to an enumeration, which
would report zero repos as a clean triage. rc 6 (a cached manifest clone that
could not be refreshed) is terminal too, and for the opposite reason: the
cached manifest is readable, so the triage would run to completion over a repo
list this host could not verify and report it as a finished scan.

This outputs a JSON list of `{name, url, version, source_file}` for all overlay repos.
Parse the `owner/repo` from each URL.

If `--repo` was specified, filter to just that repository.

**An empty list is a failure, not an empty answer.** With the root anchored and
the manifest fallback attempted, an empty list means nothing is configured at
all — and `list_overlay_repos.py` prints `[]` at **exit 0** in that state. A
triage that scanned zero repos reports no stale issues, which reads identically
to "there are none" (#609). So:

- Empty list → stop and report
  **`FAILED: no repo manifest configured — run 'make setup-all'`**. Do not
  produce a triage report.
- Non-zero exit from the script → **`FAILED: manifest unreadable`** with its
  stderr. Also not an empty list.
- `manifest_config_dir` exit **5** — the bootstrap pointer names a manifest
  repo this host could not clone (unreachable remote, no credentials, a
  pointer and a manifest that disagree) → **`FAILED: manifest repo
  unreachable`** with its stderr reason. Never `make setup-all`: that
  command's own first step is this same clone, so it cannot fix it. Exit
  **3** — no `configs/manifest` *and* no usable bootstrap pointer — is the
  one state whose remedy really is `make setup-all`.
- `--repo <name>` that matches nothing in a manifest that *was* read → report
  that the repo is not listed, rather than triaging an empty set.

Every report this skill produces states how many repos were scanned, so a
partial scan is never indistinguishable from a clean one. A repo whose
`gh issue list` errors (auth, rate limit, network) is named and makes the run
**FAILED** — never an all-clear over the repos that happened to answer.

### 2. Fetch open issues per repo

For each repository:

```bash
gh issue list --repo <owner/repo> --state open --json number,title,labels,createdAt,updatedAt,url,assignees --limit 100
```

Collect all results into a unified list with the repo name attached.

### 3. Categorize issues

Classify each issue by type based on labels and title keywords:

| Category | Indicators |
|----------|------------|
| Bug | `bug` label, "fix", "crash", "error" in title |
| Enhancement | `enhancement` label, "add", "improve", "support" in title |
| Documentation | `documentation` label, "doc", "readme" in title |
| Test | `test` label, "test", "coverage" in title |
| Infrastructure | `ci`, `build`, `infra` labels |
| Uncategorized | No matching indicators |

### 4. Flag stale issues

An issue is stale if:
- `updatedAt` is more than `<stale-days>` days ago (default: 90)
- It has no assignee

The `updatedAt` field already reflects all activity (comments, label changes,
assignments), so no separate comment check is needed.

### 5. Cross-reference with workspace

Check whether issues are being tracked in the workspace:

```bash
# Check for existing worktrees or branches referencing the issue
git branch --list "feature/issue-<N>" "feature/ISSUE-<N>-*" 2>/dev/null
```

Also check if the issue number appears in any open PRs:

```bash
ISSUE_NUM=<N>
gh pr list --repo <workspace-repo> --state open --json title,url --jq ".[] | select(.title | test(\"\\b${ISSUE_NUM}\\b\"))"
```

### 6. Generate report

```markdown
## Issue Triage Report

**Scanned**: <N> repositories
**Total open issues**: <N>
**Stale issues**: <N> (> <stale-days> days without update)

### By Repository

#### <repo-name>

| # | Title | Category | Age | Stale | Tracked |
|---|-------|----------|-----|-------|---------|
| <N> | <title> | Bug/Enhancement/... | <days> days | Yes/No | Yes/No |

### Summary by Category

| Category | Count |
|----------|-------|
| Bug | <N> |
| Enhancement | <N> |
| ... | ... |

### Stale Issues (Action Needed)

| Repo | # | Title | Last Updated | Suggestion |
|------|---|-------|--------------|------------|
| <repo> | <N> | <title> | <date> | Close / Assign / Update |

### Untracked Issues

Issues not referenced in any workspace branch or PR:

| Repo | # | Title | Category |
|------|---|-------|----------|
| <repo> | <N> | <title> | <category> |
```

## Guidelines

- **Read-only** — this skill reports, it does not create issues, close them,
  or modify labels.
- **Use `gh` CLI** — all GitHub queries go through `gh`, not the API directly.
- **Respect rate limits** — for workspaces with many repos, consider scanning
  in batches. The `--repo` flag helps focus on one repo at a time.
- **Cross-repo awareness** — issues may reference other repos. Note
  cross-references but don't follow them recursively.
- **Stale ≠ invalid** — stale issues may still be relevant. Flag them for
  human review, don't recommend closing without context.
