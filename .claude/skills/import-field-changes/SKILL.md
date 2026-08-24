---
name: import-field-changes
description: Batch-import field changes from a secondary remote (e.g., gitcloud) back to GitHub for review. For each repo with remote-ahead commits, creates an issue, opens a draft PR, and pre-reviews the diff against the Quality Standard.
---

# Import Field Changes

## Usage

```
/import-field-changes
```

## Overview

Batch-import field changes from a secondary remote (e.g., gitcloud) back to
GitHub for review. For each repo with remote-ahead commits: creates an issue,
opens a draft PR, and pre-reviews the diff against the Quality Standard.

**Lifecycle position**: field deployment → push to gitcloud → **import-field-changes** → triage-reviews → merge

## Steps

### 1. Read project config

Read `.agent/project_config.yaml` for the field remote name:

```yaml
# .agent/project_config.yaml (gitignored)
field_remote: gitcloud
```

If the file doesn't exist or `field_remote` is missing, stop with:

> `.agent/project_config.yaml` not found or missing `field_remote` key.
> Create it with: `echo "field_remote: gitcloud" > .agent/project_config.yaml`

### 2. Fetch and detect changes

Run from the workspace root:

```bash
python3 .agent/scripts/pull_remote.py --remote <field_remote> --json \
    >/tmp/field_report.json 2>/tmp/field_report.err
echo "exit: $?"
```

This fetches all repos and writes a JSON array to **stdout**. Every entry
carries an explicit `state`:

| `state` | Meaning | What this skill does |
|---------|---------|----------------------|
| `ahead` | The field remote carries commits this checkout does not. Entry also has repo name, path, default branch, ahead/behind counts, `diverged`, `commits`, and `commits_truncated` (the commit list caps at 50). | Import it — step 3. |
| `no-local-branch` | The default branch exists only as a remote-tracking ref — what `vcs import` leaves for a SHA- or tag-pinned manifest entry — so **no comparison was possible**. A normal, supported workspace state, not a failure. | Do **not** claim it is clean. Report it in the summary and tell the operator to check the branch out if that repo needs reconciling. |

**An empty array does not mean "nothing to import" on its own.** Check all
three signals before concluding anything:

1. **Exit status.** `0` means every repo was actually compared. **Non-zero
   means at least one repo was never compared** — a failed fetch, a git probe
   that failed, a repo configured with no checkout in a required layer, a
   `.repos` file that would not parse, or an enumeration that produced no repos
   at all (running from a workspace *worktree* does this: there is no
   `configs/manifest`, so the report is `[]` and every repo is invisible).
2. **stderr.** Each such repo is printed as its own `ERROR: <repo>: <reason>`
   line. The JSON on stdout is then a **partial** report, not a complete one.
3. **The `state` of each entry**, per the table above.

So:

- **Exit 0 and an empty array** — and only this — is "No field changes to
  import". Report it and stop.
- **Non-zero exit**: **stop and surface the `ERROR:` lines to the operator by
  name.** Never report "no field changes to import" over a report that is
  missing repos — a field hotfix that never reconciles to GitHub, reported
  green, is the failure this whole path exists to prevent (#609). Import any
  `ahead` entries the partial report *did* contain if the operator says to
  proceed, but say plainly which repos went unchecked and why.
- **Exit 0 with only `no-local-branch` entries** is not "nothing to import"
  either — it is "nothing was compared in those repos". Report them.

### 3. For each repo with changes

Process the `state: ahead` entries sequentially. `no-local-branch` entries carry
no commit list and are reported in step 4, not imported.

**First, check for an in-flight deployment (bundling path).** Before the
default import-issue/PR flow below, check whether *this repo* already has a
deployment in flight:

- an **open issue with the `deployment` label** in the repo, AND
- a **local worktree on that issue's `feature/issue-<N>` branch** (the
  deployment branch `/start-deployment` created).

```bash
gh issue list -R <owner/repo> --label deployment --state open --json number,title
.agent/scripts/worktree_list.sh        # or: git -C <path> worktree list
```

If both hold, the field commits in this repo are typically just the per-host
logs (gabby/salmon/mercat) plus a small field-verified config tweak, and the
established preference is to **bundle them into the deployment PR** (one PR
`Closes #<deployment>`) rather than open a second import issue/PR. Surface the
option to the operator:

> "`<repo>` has an in-flight deployment (#N, worktree present). Merge
> `<field_remote>/<default_branch>` into `feature/issue-N` to bundle into the
> deployment PR (preserves field SHAs), instead of a separate import PR?"

On confirmation, **merge — never cherry-pick**:

```bash
git -C <worktree_path> merge --no-ff <field_remote>/<default_branch>
git -C <worktree_path> push
```

Then **skip 3c–3d for this repo** — the deployment PR picks up the merge.
Record it in the summary as bundled into #<deployment>.

**Why merge, not cherry-pick:** cherry-pick creates new SHAs for the same
content, so `origin/<branch>` diverges from `<field_remote>/<branch>` and the
next `push_remote.py` reconcile needs a force-push (the re-divergence pain in
[#495](https://github.com/rolker/ros2_agent_workspace/issues/495) gap 7).
Merging `<field_remote>/<default_branch>` keeps the original field commits
reachable in history, so origin and the field remote stay reconcilable without a
force-push.

**Scope:** only the deployment's *own* repo (logs + small field config) bundles
this way. Substantive field **code** in *other* repos still gets its own import
issue + PR via the steps below. If there is no in-flight deployment for the
repo, proceed with the default flow.

#### 3a. Summarize the diff

Read the diff between local default branch and remote:

```bash
git -C <path> diff <default_branch>..<remote>/<default_branch> --stat
git -C <path> log --oneline <default_branch>..<remote>/<default_branch>
```

#### 3b. Pre-review against Quality Standard

Examine the diff for Quality Standard concerns:
- Are there tests for new functionality?
- Do topic names match actual published topics (check for remap mismatches)?
- Is error handling present for failure modes?
- Are scripts idempotent?
- Any hardcoded paths or credentials?

Note findings for the issue body.

#### 3c. Create issue in the project repo

Title: `Field import: <repo_name> (<YYYY-MM-DD>)`

Body should include:
- List of commits being imported
- Pre-review findings (if any)
- Whether the repo is diverged (merge needed)

Use `.agent/scripts/gh_create_issue.sh` from the project repo directory.

#### 3d. Create branch and PR

Create the branch without checking it out (avoids changing main tree HEAD):

**Non-diverged case** (remote ahead, local not ahead):
```bash
git -C <path> branch feature/issue-<N> <remote>/<default_branch>
git -C <path> push -u origin feature/issue-<N>
```

**Diverged case** (both sides have commits):
```bash
# Branch from remote HEAD — merge will happen in worktree
git -C <path> branch feature/issue-<N> <remote>/<default_branch>
git -C <path> push -u origin feature/issue-<N>
```

Create draft PR (run from within the project repo directory):
```bash
cd <path>
gh pr create --draft --title "Field import: <repo_name> (<date>)" \
  --body-file <body> --base <default_branch>
```

Note: for diverged repos, note in the PR body that a merge with the
default branch is needed before this can be merged.

#### 3e. Handle pre-review findings

If pre-review found issues that should be fixed:
1. Create a worktree for the issue
2. Apply fixes (add tests, fix topic names, etc.)
3. Commit and push to the PR branch
4. Note fixes in the PR description

If no issues found, the PR is ready for Copilot review as-is.

### 4. Report summary

Output a table:

```markdown
## Field Import Summary

| Repo | Commits | Diverged | Issue | PR | Pre-review |
|------|---------|----------|-------|-----|------------|
| <name> | <N> | Yes/No | #<N> | #<N> | Clean / <N> findings |
```

Then, and **never silently omitted**, the repos that were *not* compared —
every `ERROR:` line from stderr and every `no-local-branch` entry:

```markdown
### Not checked (no conclusion drawn)

| Repo | Why | Remedy |
|------|-----|--------|
| <name> | probe failed: <reason from stderr> | <what the reason says to do> |
| <name> | no local `<branch>` — pinned manifest entry | `git -C <path> checkout <branch>` if this repo needs reconciling |
```

If this section is empty, say so explicitly ("all configured repos were
compared") rather than leaving its absence to be read as coverage.

## Guidelines

- **Never edit in the main tree** — all fixes go through worktrees
- **One issue per repo** — even if the repo has multiple unrelated field commits
  (exception: the bundling path reuses the deployment issue for the deployment's
  own repo instead of opening an import issue)
- **Pre-review is advisory** — findings go in the issue, not auto-fixed unless
  the fix is trivial and unambiguous (e.g., missing shebang)
- **Diverged repos need human judgment** — report them prominently, don't
  auto-merge
- **Never cherry-pick field commits** — always branch *from* the remote ref
  (default flow) or **merge** it (bundling path). Cherry-pick rewrites SHAs and
  diverges origin from the field remote, forcing a force-push at the next
  reconcile (#495 gap 7)
- **An absence is never evidence** — a repo missing from the report was not
  found clean, it was not checked. Read the exit status and the `ERROR:`
  lines before drawing any conclusion from an empty result (#609)
- **This skill does not resync gitcloud** — after PRs merge, use
  `push_remote.py` manually to update gitcloud
