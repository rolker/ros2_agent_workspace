---
name: inspiration-tracker
description: Track external projects for portable enhancements and interesting patterns. Supports fork-style file diffs and inspiration-style surveys with per-project digests.
---

# Inspiration Tracker

## Usage

```
/inspiration-tracker                  # List tracked projects, pick one to check
/inspiration-tracker <name>           # Check a specific tracked project
/inspiration-tracker add              # Add a new project interactively
/inspiration-tracker add <repo-url>   # Add a specific repo to the registry
```

## Overview

**Lifecycle position**: Utility — run periodically to discover portable
enhancements and interesting patterns from tracked external projects.

Maintains a registry of external projects (`.agent/knowledge/inspiration_registry.yml`)
and per-project digests that track what's been reviewed, ported, skipped, or deferred.
Supports two project types with different comparison strategies:

- **fork**: File-level diff for repos with shared structure (e.g., a repo
  this workspace was forked from or vice versa)
- **inspiration**: Structured survey + changelog for unrelated projects with
  interesting patterns

## Registry

The project registry lives at `.agent/knowledge/inspiration_registry.yml` (git-tracked).

```yaml
projects:
  agent_workspace:
    type: fork
    repo: rolker/agent_workspace
    description: Fork of this workspace with independent evolution
    domain_patterns: [ros2, colcon, layer, overlay, gazebo, launch]
    categories: [scripts, skills, adrs, templates, knowledge, makefile, config]

  superpowers:
    type: inspiration
    repo: obra/superpowers
    description: Composable skills framework for Claude Code
    interest_areas:
      - skills architecture
      - TDD methodology
      - subagent patterns
```

### Registry fields

**Common fields (both types)**:
- `type`: `fork` or `inspiration`
- `repo`: GitHub `owner/repo` slug
- `description`: One-line description of the project

**Fork-only fields**:
- `domain_patterns`: list of strings — filenames matching these are auto-tagged
  as "domain-specific — probably skip"
- `categories`: which directories to compare (see fork comparison table below)

**Inspiration-only fields**:
- `interest_areas`: list of topics/patterns to focus on during surveys and
  changelog reviews

## Steps

### 1. Entry point

**No arguments**:
- Read the registry. If empty, offer to seed it:
  - Add `agent_workspace` (fork type)
  - Search the web for interesting projects
  - Enter a repo URL manually
- If registry has entries, list them and ask which to check.

**`<name>`**: Look up in registry and proceed to step 2.

**`add`**: Jump to the interactive add flow (step 11).

**`add <repo-url>`**: Pre-fill the repo URL and jump to the add flow (step 11).

### 2. Ensure local copy

All clones live in `.agent/scratchpad/inspiration/<name>/` (gitignored,
ephemeral — recreated on any machine where this workspace is checked out).

```bash
CLONE_DIR=".agent/scratchpad/inspiration/<name>"
if [ -d "$CLONE_DIR/.git" ]; then
    cd "$CLONE_DIR" && git fetch origin
else
    git clone --depth=1 "https://github.com/<repo>.git" "$CLONE_DIR"
fi
UPSTREAM_SHA=$(cd "$CLONE_DIR" && git rev-parse HEAD)
```

### 3. Create skill worktree

Create and enter a skill worktree for the digest commit. This gives the
skill a branch to commit to throughout the run.

```bash
.agent/scripts/worktree_create.sh --skill inspiration-tracker --type workspace
source .agent/scripts/worktree_enter.sh --skill inspiration-tracker
```

If a skill worktree already exists (from a previous incomplete run), enter it.

### 4. Load digest state

Read `.agent/knowledge/inspiration_<name>_digest.md` (if it exists) to check
which items have already been reviewed and what decisions were made (ported,
skipped, deferred). The digest also records the last-checked commit SHA,
needed for changelog mode in step 6.

### 5. Gather GitHub context

For both project types, query the upstream repo for activity:

```bash
# Open issues (most recent 20)
gh issue list -R <repo> --limit 20 --json number,title,labels,updatedAt

# Recently closed issues (last 30 days)
gh issue list -R <repo> --state closed --json number,title,labels,updatedAt \
  | jq '[.[] | select(.updatedAt > "YYYY-MM-DDT00:00:00Z")]'

# Open PRs
gh pr list -R <repo> --json number,title,labels,updatedAt,headRefName

# Recently merged PRs (last 30 days)
gh pr list -R <repo> --state merged --json number,title,labels,updatedAt \
  | jq '[.[] | select(.updatedAt > "YYYY-MM-DDT00:00:00Z")]'
```

Present a summary: "What's happening in `<name>`" — grouped by theme
(infrastructure, skills, docs, etc.).

### 6. Run type-specific comparison

#### Fork type — file-level diff

Compare these categories between the local clone and this workspace:

| Category | Path pattern | Method |
|----------|-------------|--------|
| Scripts | `.agent/scripts/` | File listing diff + content diff for shared files |
| Skills | `.claude/skills/` | Directory listing diff |
| ADRs | `docs/decisions/` | File listing diff + title comparison |
| Templates | `.agent/templates/` | File listing diff |
| Knowledge | `.agent/knowledge/` | File listing diff |
| Makefile | `Makefile` | `.PHONY` target listing diff |
| Config | `AGENTS.md`, `ARCHITECTURE.md` | Size/hash as change indicator |

For each category, identify:
- **Upstream-only files**: candidates to port
- **Shared files with differences**: potential enhancements (show brief diff summary)
- **Local-only files**: informational, no action needed

Auto-classify items matching `domain_patterns` as "domain-specific — probably skip".

#### Inspiration type — survey or changelog

**First run** (no digest exists yet): Perform an initial survey.
- Read the repo's README, directory structure, and key config files
- Produce a structured summary mapped to workspace categories:
  governance model, skills/commands, isolation strategy, identity management,
  testing approach, CI/CD patterns, documentation patterns
- Focus on `interest_areas` from the registry entry
- Record the summary in the digest

**Subsequent runs** (digest exists): Changelog mode.
- Use the GitHub compare API to see what changed since the last-checked SHA:
  ```bash
  gh api repos/<owner>/<repo>/compare/<last-sha>...<current-sha> \
    --jq '{commits: [.commits[].commit.message], files: [.files[].filename]}'
  ```
- Cross-reference with GitHub activity from step 5
- Highlight changes relevant to `interest_areas`
- Read changed files in the local clone for detailed understanding
- Summarize what's new since last check

### 7. Present findings and commit digest checkpoint

**Section 1: Upstream Activity**
- Summary of open/recent issues and PRs from step 5

**Section 2: Findings**
- Fork type: new/changed files with domain-specific auto-tags
- Inspiration type: survey results or changelog highlights

Present all findings to the user, then **commit the digest checkpoint**
before asking for decisions. This preserves the research even if the
conversation ends before the user triages all items.

Write/update `.agent/knowledge/inspiration_<name>_digest.md` with:

```markdown
# Inspiration Digest: <name>

Type: fork | inspiration
Last checked: YYYY-MM-DD
Repo: <owner>/<repo> @ <commit-sha>

## Survey Summary (inspiration type only)

<structured summary from initial survey>

## Activity Snapshot

- N open issues, M open PRs
- Notable: <brief summary of relevant items>

## Pending Review

- `item` — <brief description> (YYYY-MM-DD)

## Roadmapped

- `feature` — added to ROADMAP.md (YYYY-MM-DD)

## Skipped

- `feature` — reason (YYYY-MM-DD)

## Deferred

- `feature` — revisit later (YYYY-MM-DD)
```

Commit the digest in the skill worktree:

```bash
git add .agent/knowledge/inspiration_<name>_digest.md
git commit -m "docs: update inspiration digest for <name>

Research checkpoint — findings gathered, decisions pending."
```

Then, for each **new or changed** item not already decided in the digest,
ask the user to choose:

- **Add to roadmap** — append to the "To Consider" section of `docs/ROADMAP.md`
- **Skip** (with reason) — record in digest, won't be re-prompted
- **Defer** — record in digest, will be re-prompted on next run

Items with existing decisions are shown as a summary at the end.

### 8. Act on decisions

**Add to roadmap**: Append items to the "To Consider" section of `docs/ROADMAP.md`,
grouped under a heading for this project and date. Do NOT create GitHub issues —
issues are created later when work is ready to begin, typically during a
`/brainstorm` session that reviews the roadmap.

```markdown
### From <name> (YYYY-MM-DD)

- **<title>** — <brief description>. Source: <repo> — <file or pattern>
```

**Skip/Defer**: Record in digest only.

### 9. Update digest with decisions

Update the digest to move items from "Pending Review" to their final
sections (Roadmapped, Skipped, or Deferred). Commit the update:

```bash
git add .agent/knowledge/inspiration_<name>_digest.md
git commit -m "docs: record inspiration-tracker decisions for <name>"
```

### 10. Push and create PR

Push the skill branch and create a PR for the digest update.

```bash
git push -u origin HEAD

BODY_FILE=$(mktemp /tmp/gh_body.XXXXXX.md)
cat << 'EOF' > "$BODY_FILE"
## Inspiration Tracker: <name>

<Brief summary of findings and decisions>

---
**Authored-By**: `$AGENT_NAME`
**Model**: `$AGENT_MODEL`
EOF
gh pr create --title "Update inspiration digest: <name>" --body-file "$BODY_FILE"
rm -f "$BODY_FILE"
```

### 11. Interactive add flow

When invoked with `add` or `add <url>`:

1. **Get the repo**: If URL provided, extract `owner/repo`. Otherwise, ask:
   - Enter a GitHub repo URL or `owner/repo` slug
   - Search the web for projects (optionally with user-supplied search terms)
   - Browse search results and select

2. **Validate**: Check the repo exists via `gh repo view -R <repo>`.

3. **Choose type**: Ask fork or inspiration.

4. **Configure type-specific fields**:
   - Fork: ask which categories to compare, any domain patterns to filter
   - Inspiration: ask which interest areas to focus on

5. **Create/enter skill worktree**: The registry is git-tracked, so edits
   must go through a PR. Create (or enter) the skill worktree before writing:
   ```bash
   .agent/scripts/worktree_create.sh --skill inspiration-tracker --type workspace
   source .agent/scripts/worktree_enter.sh --skill inspiration-tracker
   ```
   If a skill worktree already exists (from a previous run), just enter it.

6. **Add to registry**: Append the new entry to `inspiration_registry.yml`
   and commit in the skill worktree.

7. **Optionally run first check**: Offer to immediately check the newly added
   project (reuses the skill worktree created in step 5). If the user declines,
   push the branch and create a PR for the registry commit (per main workflow
   step 10).

## Source

- **Origin**: https://github.com/rolker/agent_workspace
- **Original commit**: `7de77085ec4dff6b2f1d301759fdd6ba35bd4471`
- **Import date**: 2026-03-23
- **Adapted by**: Claude Code Agent

## Guidelines

- **Interactive, not autonomous** — always present findings and let the user
  decide. Never add to roadmap without confirmation.
- **Discovery, not implementation** — this skill identifies and triages
  enhancements. Findings go to the roadmap's "To Consider" section. GitHub
  issues are created later (during `/brainstorm`) when work is ready to begin.
- **One project per run** — check one project at a time for focused review.
- **Single PR per run** — each run produces at most one PR (the digest
  update in the skill worktree). Implementation work gets its own issues
  and PRs.
- **Idempotent** — safe to re-run. Items with decisions aren't re-prompted
  (except deferred items, which resurface).
- **Commit early, update later** — the digest is committed after research
  (step 7) and updated after decisions (step 9). This preserves research
  even if the conversation is interrupted.
- **Scratchpad clones are ephemeral** — all clones in
  `.agent/scratchpad/inspiration/<name>/` (gitignored). Registry and digests
  are git-tracked and portable across machines.
- **Shallow clones for reading, GitHub API for history** — local clones use
  `--depth=1` for speed; changelog tracking uses the GitHub compare API
  instead of local git history. This keeps clones lightweight.
- **GitHub API rate limits** — the skill uses ~6-7 API calls per project per
  run (issues, PRs, compare). Authenticated GitHub API allows 5000 calls/hour,
  so periodic manual use is well within limits.
- **Domain pattern filtering** — fork type only. Reduces noise for repos that
  share structure but have domain-specific content.
- **Skill worktree created at start** — the worktree is created in step 3
  and used for all digest commits throughout the run.
