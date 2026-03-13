---
name: onboard-project
description: Interactive audit and onboarding for project repos. Checks for CI, pre-commit, agent guide, GitHub settings, and labels. Offers to fix gaps or open issues.
---

# Onboard Project

## Usage

```
/onboard-project <repo-name>
```

Where `<repo-name>` is a directory under `layers/main/<layer>_ws/src/`.

## Overview

**Lifecycle position**: Utility — run to bring a project repo up to workspace
standards. Not tied to the per-issue lifecycle. Can be re-run on partially
onboarded repos (idempotent).

Interactive audit that checks a project repo against onboarding standards,
presents each gap, and lets the user choose how to handle it. All "fix now"
items are batched into a single PR on the project repo.

**Complements `audit-project`**: that skill reports gaps; this skill fixes them.

## Steps

### 1. Locate the repo

Find the repo under `layers/main/*_ws/src/<repo-name>`. Verify it exists and
has at least one `package.xml`.

```bash
REPO_DIR=$(find layers/main/*_ws/src/<repo-name> -maxdepth 0 -type d 2>/dev/null | head -1)
```

Determine the layer name from the path (e.g., `core_ws` → `core`).

Determine the default branch:
```bash
cd "$REPO_DIR" && \
  (git symbolic-ref refs/remotes/origin/HEAD 2>/dev/null | sed 's|refs/remotes/origin/||' || \
   gh repo view --json defaultBranchRef --jq '.defaultBranchRef.name' 2>/dev/null)
```

Discover packages:
```bash
find "$REPO_DIR" -name package.xml -not -path '*/build/*' -not -path '*/install/*' -print0 \
  | xargs -0 sed -n 's/.*<name>\([^<]*\)<\/name>.*/\1/p'
```

Get the GitHub repo slug:
```bash
cd "$REPO_DIR" && gh repo view --json nameWithOwner --jq '.nameWithOwner'
```

### 2. Run the audit

Check each onboarding item and record its status. For each item, determine
whether it's already done, missing, or partially done.

#### Checklist

| Item | Check | Fix approach |
|------|-------|-------------|
| **Pre-commit config** | `.pre-commit-config.yaml` exists at repo root | Copy from `.agent/templates/pre-commit-config.yaml`, adjust protected branches to match repo's default branch |
| **CI workflow** | `.github/workflows/*.yml` exists with colcon build/test | Copy from `.agent/templates/ci_workflow.yml`, fill in default branch and package list |
| **Agent guide** | `.agents/README.md` exists at repo root | Generate from `.agent/templates/project_agents_guide.md` by reading repo code (packages, launch files, nodes, topics) |
| **Branch protection** | GitHub branch ruleset requires PRs on default branch | Configure via `gh api` — requires admin access |
| **Copilot auto-review** | Copilot is configured to review PRs | Check via `gh api` — requires admin access |
| **Package labels** | `pkg:` labels exist (multi-package repos only) | Create via `gh label create` for each package |
| **git-bug bridge** | `git bug bridge list` in repo shows GitHub bridge | Configure bridge with `git bug bridge configure` + initial pull |
| **License headers** | Source files have copyright/license headers matching `package.xml` license | Report per ADR-0008; too invasive for batch fix — always open issue |

### 3. Present findings interactively

For each gap found, present the finding and ask the user to choose:

- **Fix now** — add to the batch of fixes for this run
- **Open issue** — create an issue on the project repo describing the gap
- **Skip** — move on, don't fix or track

Present items in this order (quick wins first):
1. Pre-commit config
2. CI workflow
3. Package labels (multi-package repos)
4. git-bug bridge (if git-bug is installed)
5. Branch protection / Copilot auto-review
6. Agent guide (`.agents/README.md`)
7. License headers (always suggest "open issue" — too invasive for batch fix)

Skip items that are already in place. If everything is already done, report
that and exit.

### 4. Handle GitHub API items (branch protection, Copilot, labels)

These require specific permissions. Before attempting:

```bash
# Check if we can write repo settings
gh api repos/<owner>/<repo> --jq '.permissions.admin' 2>/dev/null
```

If admin access is not available, report clearly:
> "Branch protection and Copilot auto-review require admin access to the
> repo. Current credentials don't have admin permissions. Skipping these
> items — configure them manually in GitHub repo settings."

For labels (requires write but not admin):
```bash
gh label create "pkg:<package_name>" --description "<package description>" \
  --color "5319E7" -R <owner>/<repo>
```

For git-bug bridge (if git-bug is installed):
```bash
if command -v git-bug &>/dev/null; then
    cd "$REPO_DIR" && git bug bridge list 2>/dev/null | grep -q "github" || {
        GH_TOKEN=$(gh auth token 2>/dev/null)
        OWNER=$(cd "$REPO_DIR" && gh repo view --json owner --jq '.owner.login' 2>/dev/null)
        REPO=$(cd "$REPO_DIR" && gh repo view --json name --jq '.name' 2>/dev/null)
        git bug bridge new --name github --target github \
            --owner "$OWNER" --project "$REPO" --token "$GH_TOKEN" --non-interactive
        git bug pull
    }
fi
```

### 5. Implement "fix now" items

If the user selected any items to fix now:

**Create an issue on the project repo** to track the onboarding work:
```bash
BODY_FILE=$(mktemp /tmp/gh_body.XXXXXX.md)
cat << EOF > "$BODY_FILE"
## Onboarding improvements

Adding workspace-standard infrastructure to this repo:

- [ ] item 1
- [ ] item 2
...

Generated by the \`onboard-project\` skill.

---
**Authored-By**: \`$AGENT_NAME\`
**Model**: \`$AGENT_MODEL\`
EOF
ISSUE_URL=$(gh issue create -R <owner>/<repo> --title "Onboard repo with workspace standards" \
  --body-file "$BODY_FILE")
rm "$BODY_FILE"
ISSUE_NUM=$(echo "$ISSUE_URL" | grep -o '[0-9]*$')
```

**Create a layer worktree** on the project repo using the new issue number:
```bash
.agent/scripts/worktree_create.sh --issue "$ISSUE_NUM" --type layer --layer <layer> --packages <repo-name>
source .agent/scripts/worktree_enter.sh --issue "$ISSUE_NUM"
cd <layer>_ws/src/<repo-name>
```

**Apply fixes** (in the worktree, on the project repo):

- **Pre-commit config**: Copy template, replace protected branch placeholders
  with the repo's actual default branch. If the repo has Python code (check
  for `setup.py` or `*.py` files in package dirs), uncomment the Python
  linting hooks (black, flake8).

- **CI workflow**: Copy template, replace:
  - `PLACEHOLDER_DEFAULT_BRANCH` → repo's default branch
  - `PLACEHOLDER_PACKAGE_LIST` → discovered package names (space-separated)
  - `repo` in the symlink step → the repo directory name
  - Add any known extra dependency cloning steps if discoverable from
    `package.xml` dependencies that aren't in rosdep

- **Agent guide** (`.agents/README.md`): Use the template from
  `.agent/templates/project_agents_guide.md`. Read the repo's actual code to
  fill in sections: package inventory (from `package.xml`), layout (from
  directory structure), key files, dependencies. Follow the documentation
  verification workflow — every claim must be verified against source.

- **Package labels**: Create via `gh label create` (done in step 4, no
  file changes needed).

- **Branch protection / Copilot**: Configure via `gh api` (done in step 4,
  no file changes needed).

**Commit and open PR**:
```bash
git add -A
git commit -m "Add workspace-standard infrastructure

Onboarding: [list items added]

Closes <owner>/<repo>#<N>"

git push -u origin HEAD

PR_BODY_FILE=$(mktemp /tmp/gh_body.XXXXXX.md)
cat << EOF > "$PR_BODY_FILE"
## Summary

Add workspace-standard infrastructure to this repo:

- [list items added]

Closes #<N>

---
**Authored-By**: \`$AGENT_NAME\`
**Model**: \`$AGENT_MODEL\`
EOF
gh pr create --title "Onboard repo with workspace standards" \
  --body-file "$PR_BODY_FILE" -R <owner>/<repo>
rm "$PR_BODY_FILE"
```

### 6. Handle "open issue" items

For each item the user chose to defer:

```bash
BODY_FILE=$(mktemp /tmp/gh_body.XXXXXX.md)
cat << EOF > "$BODY_FILE"
## <item description>

<specific details about what's missing and how to fix it>

Identified by the \`onboard-project\` skill.

---
**Authored-By**: \`$AGENT_NAME\`
**Model**: \`$AGENT_MODEL\`
EOF
gh issue create -R <owner>/<repo> --title "<item title>" \
  --body-file "$BODY_FILE"
rm "$BODY_FILE"
```

### 7. Summary

Report to the user:

```markdown
## Onboarding Summary: <repo-name>

### Fixed (PR)
- [x] <item> — [PR #N](url)

### Deferred (issues opened)
- [ ] <item> — [Issue #N](url)

### Skipped
- <item>

### Already in place
- <item>
```

Include clickable GitHub links for every PR and issue (use `gh` to look
them up — never guess URLs).

## Guidelines

- **Interactive, not autonomous** — always ask the user before making changes.
  The skill presents options; the user decides.
- **One repo at a time** — don't batch multiple repos. Run the skill again
  for each repo.
- **Idempotent** — safe to re-run. Items already in place are reported as
  such and skipped.
- **Single PR** — all "fix now" items go into one PR on the project repo.
  This keeps the onboarding change reviewable as a unit.
- **Self-contained artifacts** — generated CI workflows and pre-commit
  configs must work standalone in the project repo. No dependency on
  workspace scripts or paths (ADR-0003).
- **Graceful permission handling** — GitHub API operations (branch
  protection, Copilot, labels) require specific permissions. Check before
  attempting; report clearly if unavailable. Never fail the entire run
  because of a permission issue.
- **License headers are always deferred** — adding headers to every source
  file is too invasive for a batch fix. Always suggest "open issue" for this
  item.
- **Verify documentation against source** — when generating `.agents/README.md`,
  every claim about packages, topics, parameters, and dependencies must be
  verified by reading the actual source code. Use the documentation
  verification workflow from `.agent/knowledge/documentation_verification.md`.
