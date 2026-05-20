---
name: review-code
description: Lead reviewer that orchestrates specialist sub-reviews (static analysis, governance, plan drift, adversarial) to evaluate a PR or pre-push diff. Scales review depth to change risk. Produces a unified structured report and persists findings to progress.md.
---

# Review Code

## Usage

```
/review-code [--base <branch>] [--skip-static] [--no-progress] [--issue <N>] [--no-copilot] [light|standard|deep]
                                            # pre-push: diff vs default branch
/review-code <pr-number-or-url> [--skip-static] [--no-copilot] [--allow-untrusted-copilot] [light|standard|deep]
                                            # post-PR: diff vs PR base
```

Flags:
- **depth keyword** (`light` / `standard` / `deep`, positional) overrides
  automatic classification.
- **`--base <branch>`** (pre-push only) overrides the default base branch.
- **`--skip-static`** (both modes) suppresses the Static Analysis
  Specialist. Useful when pre-commit was already clean or linters were
  run separately; the report header notes the skip.
- **`--no-progress`** (pre-push only) opts out of `progress.md`
  persistence. Use for skill worktrees and one-off branches that don't
  have an associated issue. Does not apply to post-PR mode — a PR with
  no closing references is already handled by step 8's no-issue
  fallthrough.
- **`--issue <N>`** (pre-push only) explicit issue number override.
  Use when the branch name doesn't match `feature/issue-<N>` /
  `feature/ISSUE-<N>-…` (e.g., skill worktrees, manually named
  branches). When passed, the branch-name extraction at step 1 is
  skipped. Mutually compatible with `--no-progress` — `--no-progress`
  wins (no persistence regardless of issue number).
- **`--no-copilot`** (both modes) suppresses the Copilot Adversarial
  Specialist (5e) for this invocation. Useful when the Copilot CLI is
  installed but you want to avoid the Premium-request cost on a
  specific run.
- **`--allow-untrusted-copilot`** (post-PR only) overrides the
  external-PR safety gate that suppresses Copilot Adversarial when the
  PR head is from a fork or a non-collaborator author. Without this
  flag, the specialist routes to the skipped-with-notice path on such
  PRs because `--allow-all-tools` grants Copilot file/shell access to
  the local worktree; running it against an untrusted contributor's
  diff exposes that capability to attacker-controlled prompt content.
  Pass only when you have read the diff and accept the risk.

## Overview

**Lifecycle position**: review-issue → plan-task → review-plan →
implement → **review-code** → push / open PR → triage-reviews

Multi-specialist code review system. A lead reviewer gathers context,
classifies review depth based on change risk, dispatches specialist
sub-reviews in parallel, collects findings, deduplicates, applies a
silence filter, produces a unified report, and appends a step entry to
the issue's `progress.md` so findings persist across sessions. Does not
post comments or modify the PR unless the user asks.

**Two modes**:
- **Pre-push** (default, no arg) — diffs against the current repo's
  default branch. Run before `git push` / opening a PR to catch findings
  while still locally fixable. No PR-side context (comments, PR body)
  available.
- **Post-PR** (`<N>` or URL) — diffs against the PR base branch via
  `gh pr view`. Includes PR-side context: existing comments, linked
  issue, plan file referenced in PR body.

**Depth tiers** (see `.agent/knowledge/review_depth_classification.md`):
- **Light** — Static Analysis + Copilot Adversarial (small, low-risk
  changes)
- **Standard** — Static Analysis + Governance + Plan Drift + Claude
  Adversarial + Copilot Adversarial (medium changes or
  governance-touching files)
- **Deep** — Same as Standard with both Adversarial specialists primed
  for security / concurrency / lifecycle (large changes, security, or
  cross-layer)

**Specialists**:
- **Static Analysis** — runs linters with ament-aligned configs on
  changed files
- **Governance** — evaluates against principles, ADRs, and the
  consequences map
- **Plan Drift** — compares implementation against the work plan (if one
  exists)
- **Claude Adversarial** — fresh-context Claude subagent that re-reads
  the diff cold (Standard + Deep)
- **Copilot Adversarial** — synchronous Copilot CLI dispatch that re-reads
  the diff cold (Light + Standard + Deep). Skipped with a one-line
  notice when `copilot` is unavailable; suppress entirely with
  `--no-copilot`.

## Steps

### 1. Detect mode and gather diff context

Parse the arguments in this order: extract `--skip-static` (sets
`SKIP_STATIC=true`), `--no-progress` (sets `NO_PROGRESS=true`, pre-push
only — emit an error if passed in post-PR mode), `--issue <N>` (sets
`USER_ISSUE=<N>`, pre-push only — emit an error if passed in post-PR
mode, where `closingIssuesReferences` is authoritative), `--no-copilot`
(sets `NO_COPILOT=1`, suppresses the Copilot Adversarial Specialist in
step 5e), `--allow-untrusted-copilot` (sets `ALLOW_UNTRUSTED_COPILOT=1`,
post-PR only — emit an error if passed in pre-push mode, where the
gate doesn't apply), `--base <branch>` (sets `USER_BASE`), then the
optional depth keyword (`light` / `standard` / `deep` — positional).
Classify what remains:

- Empty → **pre-push mode**
- A number or `https://github.com/.../pull/<N>` → **post-PR mode**

The depth keyword, `--skip-static`, and `--no-copilot` may appear in
any order around the PR number / URL or `--base <branch>`. The same
syntax applies in both modes. Examples:

```
/review-code                                # pre-push, auto-classify
/review-code deep                           # pre-push, force Deep
/review-code --base develop                 # pre-push, override base
/review-code --base develop deep            # pre-push with override + force Deep
/review-code --skip-static                  # pre-push, skip static analysis
/review-code --skip-static light            # pre-push, light + skip static
/review-code --no-progress                  # pre-push, don't write progress.md
/review-code --issue 460                    # pre-push, override branch-name issue extraction
/review-code --no-copilot                   # pre-push, skip Copilot Adversarial
/review-code 42                             # post-PR, auto-classify
/review-code 42 standard                    # post-PR, force Standard
/review-code 42 --skip-static               # post-PR, skip static analysis
/review-code 42 --no-copilot                # post-PR, skip Copilot Adversarial
```

#### Pre-push mode

```bash
# Repo root and base branch — start from the default, override if
# the invocation included `--base <branch>`.
REPO_ROOT=$(git rev-parse --show-toplevel)
# Default branch — try local git first (works for any remote type,
# including non-GitHub field-mode origins and non-`main` defaults like
# `jazzy` or `master`); fall back to gh on GitHub-origin repos that
# haven't had `git remote set-head` run; final fallback is `main`.
# Capture symbolic-ref's exit status BEFORE applying any transformation,
# because `git symbolic-ref ... | sed ...` exits with sed's status —
# sed reads an empty pipe successfully when symbolic-ref fails, which
# would silently leave DEFAULT_BRANCH empty and skip the fallback chain.
if REMOTE_HEAD=$(git symbolic-ref refs/remotes/origin/HEAD 2>/dev/null); then
    DEFAULT_BRANCH="${REMOTE_HEAD#refs/remotes/origin/}"
else
    DEFAULT_BRANCH=$(gh repo view --json defaultBranchRef --jq '.defaultBranchRef.name' 2>/dev/null || echo "main")
fi
# USER_BASE is set by the caller when `--base <branch>` is parsed off
# the argument list; otherwise it's empty and the default applies.
BASE="${USER_BASE:-$DEFAULT_BRANCH}"

# Try to refresh the base from the remote. Tolerate offline / no-remote
# cases — fetch failure isn't fatal; we'll use whatever local
# `origin/$BASE` ref exists.
if ! git fetch origin "$BASE" --quiet 2>/dev/null; then
    if git rev-parse --verify "origin/$BASE" &>/dev/null; then
        echo "⚠️  Could not fetch origin/$BASE; reviewing against the local copy (may be stale)." >&2
    else
        echo "Error: no local origin/$BASE ref and fetch failed. Pass --base <branch> or run online." >&2
        exit 1
    fi
fi

# Diff and stats
git diff "origin/$BASE...HEAD" --stat
git diff "origin/$BASE...HEAD"
git diff "origin/$BASE...HEAD" --numstat   # per-file +/- counts

# Linked issue: explicit --issue <N> wins; otherwise extract from branch
# name (feature/issue-<N> or feature/ISSUE-<N>-...).
BRANCH=$(git branch --show-current)
if [[ -n "$USER_ISSUE" ]]; then
    ISSUE_NUM="$USER_ISSUE"
else
    ISSUE_NUM=$(echo "$BRANCH" | grep -oE 'issue-[0-9]+|ISSUE-[0-9]+' | grep -oE '[0-9]+' | head -1)
fi
```

`--base <branch>` is parsed from the argument list before the snippet
runs and exposed as `USER_BASE`. Default-branch resolution prefers
`git symbolic-ref refs/remotes/origin/HEAD` (the locally cached default
that `git clone` sets up) so project repos with non-`main` defaults
(`jazzy`, `master`) and non-GitHub field-mode origins resolve
correctly without an internet round-trip. If the local symbolic ref is
missing (older clone, or `git remote set-head` was never run), the
snippet tries `gh repo view`, then falls back to `main` as a last
resort. If `git fetch` then also fails and no local `origin/$BASE`
ref exists, the snippet stops with an error rather than silently
diffing against nothing.

If no issue number can be extracted from the branch name, note "no
linked issue" in the report header — the review still runs but
plan-drift and progress.md persistence are skipped.

#### Post-PR mode

```bash
# PR metadata (includes the parsed list of closing-issue references)
gh pr view <N> --json title,body,baseRefName,headRefName,headRefOid,files,additions,deletions,url,comments,reviews,closingIssuesReferences

# Full diff
gh pr diff <N>

# Primary closing-linked issue. Use `closingIssuesReferences` (parsed
# by GitHub from `Closes #N` / `Fixes #N` / `Resolves #N` keywords) —
# do NOT just grep `#[0-9]+` from the body. PRs often mention many
# issue numbers (cross-references, context); only closing-references
# represent the issue this PR is actually completing. If a PR has
# multiple closing references, use the first; if none, treat as no
# linked issue (review still runs, but plan-drift and progress.md
# persistence are skipped).
ISSUE_NUM=$(gh pr view <N> --json closingIssuesReferences --jq '.closingIssuesReferences[0].number // empty')
```

In both modes, identify:
- What repo the diff lives in (workspace or project repo?)
- What files changed and in which directories
- The linked issue and its requirements (when resolvable)
- Whether a work plan exists at `.agent/work-plans/issue-<N>/plan.md` in
  the repo that owns the issue

Read the **full content** of each changed file (not just the diff hunks)
to understand surrounding context.

### 2. Classify review depth

Load `.agent/knowledge/review_depth_classification.md` and apply the risk
signals from step 1:

1. Count total lines changed (additions + deletions).
2. Count files changed.
3. Check file paths against the workspace-repo and project-repo override
   trigger lists.
4. Check Deep promotion triggers (security-relevant, cross-layer, ADR add
   or substantive ADR rewrite).
5. Apply tier promotion logic — highest tier wins.

**User override**: If the `/review-code` invocation includes a depth
keyword (`light`, `standard`, or `deep`), use that tier instead of the
automatic classification.

Record the tier and the primary signal that determined it for the report
header.

### 3. Load project context

For project repo PRs:
- Read `.agents/README.md` for architecture overview, key files,
  cross-layer dependencies, and pitfalls.
- Check for `.agents/review-context.yaml` — if present, use it for the
  compact relevance map (packages, topics, dependencies).
- **Staleness check**: If `review-context.yaml` exists, compare its
  `context_generated_from_sha` field against the current HEAD of the
  project repo. If they differ, include a warning in the report header:

  > ⚠ Review context is stale (generated from `<sha>`; repo HEAD is `<sha>`).
  > Consider running `/gather-project-knowledge` to refresh.

  If `review-context.yaml` does not exist, note this in the report
  header:

  > ℹ No review-context.yaml found. Review proceeds with .agents/README.md only.

- Read project `PRINCIPLES.md` if it exists.
- Check `.agent/project_knowledge/` symlink for workspace-level project
  summaries.

### 4. Classify changed files for static analysis

Determine the linter profile for each changed file:

| File location | Language detection | Linter config profile |
|---|---|---|
| `layers/*/src/**/*.py` | Python | ament (max-line-length=99, ament ignores) |
| `layers/*/src/**/*.cpp`, `*.hpp` | C++ | ament (cpplint, cppcheck) |
| `.agent/scripts/*.py` | Python | workspace (max-line-length=100, Black compat) |
| `.agent/scripts/*.sh` | Shell | workspace (shellcheck --severity=warning) |
| `*.yaml`, `*.yml` | YAML | yamllint (max-line-length=120) |
| `*.xml`, `*.launch.xml` | XML | xmllint |
| `*.md` | Markdown | (no linter — content review only) |

See `.agent/knowledge/review_static_analysis.md` for full tool configs.

### 5. Dispatch specialists

Dispatch specialists based on the depth tier from step 2. Run independent
specialists in parallel — use the `Agent` tool with subagents when
available so each runs in its own context window; otherwise evaluate
sequentially.

#### Light tier

Run:
- **5a. Static Analysis Specialist**
- **5e. Copilot Adversarial Specialist** (Standard prompt; skipped with
  notice if `copilot` unavailable; suppressed entirely by `--no-copilot`)

#### Standard tier

Run all of:
- **5a. Static Analysis Specialist**
- **5b. Governance Specialist**
- **5c. Plan Drift Specialist** (if a plan exists)
- **5d. Claude Adversarial Specialist** (Standard prompt)
- **5e. Copilot Adversarial Specialist** (Standard prompt; skipped with
  notice if `copilot` unavailable; suppressed entirely by `--no-copilot`)

#### Deep tier

Same as Standard, but both adversarial specialists (**5d. Claude
Adversarial** and **5e. Copilot Adversarial**) run with the Deep prompt
(broader file horizon plus an explicit security / concurrency /
lifecycle checklist).

---

#### 5a. Static Analysis Specialist

**Skip if `SKIP_STATIC=true`** (`--skip-static` flag passed in step 1):
emit no findings for this specialist and note "Static analysis skipped
(--skip-static)" so the report header can surface it. At Light tier
with `--skip-static` **and** Copilot Adversarial also suppressed
(`--no-copilot` or the CLI unavailable), zero specialists run and the
silence filter produces the "No findings" output — that's the
documented behavior, not a bug. If only `--skip-static` is set on
Light, the Copilot Adversarial Specialist (5e) still runs and may
produce findings.

Otherwise, run linters on **changed files only**, using the config
profile from step 4. See `.agent/knowledge/review_static_analysis.md`
for exact commands and flags.

If **no linter profile matches any changed file**, report this
explicitly: "No static analysis profile configured for these file types
(`.ext1`, `.ext2`)." Don't silently produce an empty findings section —
the reviewer and user need to know that absence of findings means "not
checked", not "code is clean."

Report each finding as:
- File, line number, tool name, message
- Skip findings on lines not touched by this PR (context-only lines)

#### 5b. Governance Specialist

Load governance context:
- `.agent/knowledge/principles_review_guide.md` — evaluation criteria
- `docs/PRINCIPLES.md` — workspace principles
- `docs/decisions/*.md` — ADRs (scan titles, read those triggered by this
  change)
- Project-level governance (if applicable)

**Principle evaluation**: For each relevant principle, assess the PR:

| Verdict | Meaning |
|---|---|
| **Pass** | PR clearly adheres |
| **Watch** | Not a violation, but worth noting |
| **Concern** | Potential violation that should be addressed |
| **N/A** | Principle doesn't apply |

Skip principles that clearly don't apply.

**ADR compliance**: Using the ADR applicability table, identify triggered
ADRs. For each: does the PR comply with the key requirement?

**Consequence check**: Using the consequences map, check if this PR
changes something in the "If you change..." column. Are the corresponding
"Also update..." items addressed? Mark each as Done or Missing.

**Existing review comments** (post-PR mode only): Check for unresolved
human and bot comments:

```bash
.agent/scripts/fetch_pr_reviews.sh --pr <N>
```

Note unresolved human comments (high priority), valid bot findings, and
false positives.

#### 5c. Plan Drift Specialist

If a work plan exists at `.agent/work-plans/issue-<N>/plan.md`:
- Read the plan's "Approach" and "Files to Change" sections.
- Compare against the actual diff:
  - Files listed in plan but not changed? (incomplete)
  - Files changed but not in plan? (scope creep or oversight)
  - Approach deviations? (different from what was planned)
- Report deviations as suggestions (not must-fix — plans are guides, not
  contracts).

If no work plan exists, skip this specialist.

#### 5d. Claude Adversarial Specialist

**Activates at**: Standard, Deep.

Launch as a **fresh Claude subagent** via the `Agent` tool with no
context from the other specialists. The adversarial reviewer reads the
diff and full changed files independently — that fresh-context dispatch
is the whole point. An independent reviewer that agrees with the
governance specialist is a stronger signal than one told what to look
for.

**Cross-repo limitation**: the Adversarial Specialist only sees the diff
and the files it explicitly opens. In a layered workspace, cross-repo
consequences (a workspace ADR change that affects project repos, a shared
message-package edit that breaks downstream nodes) won't surface from
fresh-context reading alone. The Governance Specialist carries that load
via the consequences map; Adversarial is not a substitute for it.

Standard-tier prompt focus areas:
- Missed edge cases and boundary conditions
- Assumption violations (what does the code assume that might not hold?)
- Subtle bugs (off-by-one, race conditions, resource leaks)
- Logic errors (does the code actually do what the PR claims?)

Deep-tier prompt adds:
- Security implications (injection, auth bypass, data exposure)
- Concurrency / lifecycle (lock ordering, init/destroy ordering, signal
  handling, shutdown paths)
- Cross-cutting effects (does this change interact with caching,
  retries, error propagation, or other system-wide behavior?)

Report findings in the same format as other specialists (file, line,
severity, description). The silence filter (step 6) deduplicates overlap
with other specialists' findings.

> **Cross-model adversarial** is wired in as the Copilot Adversarial
> Specialist (step 5e below). The tmux-orchestrated Gemini/Codex
> dispatch from upstream `cross_model_review.sh` remains unadopted —
> see `inspiration_agent_workspace_digest.md` "Partially adopted".

#### 5e. Copilot Adversarial Specialist

**Activates at**: Light, Standard, Deep.

A second adversarial pass that uses the GitHub Copilot CLI
(`@github/copilot` ≥ v1.0.48) as a synchronous, cross-model reader of
the same diff. Same fresh-context principle as 5d — Copilot sees only
the diff prompt, no other specialists' findings. The cross-model
signal value (a second vendor flagging an issue independently is a
stronger signal than the same model agreeing with itself) is the
reason this runs at every tier, not just Deep.

**Availability probe**. Skip with a one-line notice (not a failure)
when the CLI is missing, so field-mode hosts (gabby, salmon — no
GitHub credentials) don't block the rest of the review:

```bash
# Resolve copilot to an absolute path so invocation survives subshells
# that didn't load the user's shell init. nvm-managed copilot binaries
# (~/.nvm/versions/node/*/bin/copilot) are PATH-injected by ~/.bashrc,
# but ~/.bashrc early-returns for non-interactive shells — so Agent-tool
# sub-shells (and non-interactive `bash -l`) never load nvm even though
# the binary is installed. Probe in order: current PATH, then explicit
# nvm sourcing, then a glob for nvm-managed installs.
COPILOT_BIN=""
if [[ "$NO_COPILOT" == "1" ]]; then
    # User opted out for this invocation; no "skipped" notice needed.
    SKIP_COPILOT=1
elif COPILOT_BIN=$(command -v copilot 2>/dev/null) && [ -x "$COPILOT_BIN" ]; then
    : # Found on current PATH. `[ -x ]` filters out alias/function
      # returns from `command -v` (e.g., "alias copilot='...'" or a
      # shell function name) — only an actual executable path passes.
elif [ -s "$HOME/.nvm/nvm.sh" ] && \
     COPILOT_BIN=$(bash -c '. "$HOME/.nvm/nvm.sh" >/dev/null 2>&1; command -v copilot' 2>/dev/null) && \
     [ -x "$COPILOT_BIN" ]; then
    : # Found via nvm — use the absolute path so subsequent invocations don't need nvm.
else
    # Glob fallback for nvm installs where nvm.sh sourcing didn't help
    # (e.g., multiple node versions, default-alias misconfigured). Use
    # `sort -V` to pick the newest node version — `v18.12.1` sorts
    # before `v24.12.0` lexicographically (1<2), but a Copilot version
    # installed under newer node is what the user almost certainly
    # wants. All same-platform copilot shims symlink to the same
    # npm-loader.js so the choice is mostly theoretical; it matters
    # only when older nvm-managed node versions have different copilot
    # versions installed under them.
    # printf is used instead of ls so the no-match case is bash's
    # literal unmatched-glob string (caught below by `[ -x ]` failing)
    # rather than ls's stderr.
    candidate=$(printf '%s\n' "$HOME"/.nvm/versions/node/*/bin/copilot | sort -V | tail -1)
    if [ -n "$candidate" ] && [ -x "$candidate" ]; then
        COPILOT_BIN="$candidate"
    else
        COPILOT_SKIP_REASON="copilot CLI not installed (probed PATH, nvm.sh, and ~/.nvm glob)"
        SKIP_COPILOT=1
    fi
fi
if [[ "$SKIP_COPILOT" != "1" ]] && ! "$COPILOT_BIN" --version >/dev/null 2>&1; then
    # `copilot --version` is a presence check, not an auth check —
    # `--version` typically prints without contacting the API. We keep
    # it as a "binary at least runs" sanity guard. Real auth failures
    # are detected after the invocation below.
    COPILOT_SKIP_REASON="copilot --version failed (binary broken or missing dependencies)"
    SKIP_COPILOT=1
fi
```

**Untrusted-PR safety gate** (post-PR mode only). Because
`--allow-all-tools` grants Copilot file/shell access to the local
worktree, do not invoke it against attacker-controlled prompt content.
Before dispatch, check whether the PR head is from a fork or a
non-collaborator author and gate accordingly:

```bash
if [[ "$MODE" == "post-PR" && "$SKIP_COPILOT" != "1" ]]; then
    PR_AUTHOR_ASSOC=$(gh pr view "$PR" --json authorAssociation --jq '.authorAssociation')
    PR_HEAD_REPO=$(gh pr view "$PR" --json headRepository,baseRepository \
        --jq 'if .headRepository.nameWithOwner == .baseRepository.nameWithOwner then "owner" else "fork" end')
    # Trusted: PR head is the base repo AND author is OWNER/MEMBER/COLLABORATOR.
    # Anything else (fork, contributor, first-time contributor, none) is gated.
    if [[ "$PR_HEAD_REPO" == "fork" ]] || \
       [[ "$PR_AUTHOR_ASSOC" != "OWNER" && "$PR_AUTHOR_ASSOC" != "MEMBER" && "$PR_AUTHOR_ASSOC" != "COLLABORATOR" ]]; then
        if [[ "$ALLOW_UNTRUSTED_COPILOT" == "1" ]]; then
            : # User explicitly bypassed the gate. Proceed.
        else
            COPILOT_SKIP_REASON="external PR (head=$PR_HEAD_REPO, author=$PR_AUTHOR_ASSOC); pass --allow-untrusted-copilot after reviewing the diff to bypass"
            SKIP_COPILOT=1
        fi
    fi
fi
```

When `SKIP_COPILOT=1` and `NO_COPILOT` is unset, the report includes:
`Copilot Adversarial skipped: <COPILOT_SKIP_REASON>`. When
`NO_COPILOT=1`, the specialist is omitted from the report entirely.
Post-call empty findings or auth-error output also route to the
skipped-notice path (see the post-invocation guard below) so
unauthenticated hosts don't surface as silent zero-finding reviews.

**Prompt body**. Light + Standard reuse the same prompt as the Standard
Claude Adversarial prompt (the four focus areas under 5d: missed edge
cases, assumption violations, subtle bugs, logic errors). Deep adds
the same three Deep-tier focus areas as 5d (security, concurrency /
lifecycle, cross-cutting effects). Reusing one prompt body per tier
pair keeps the two adversarial specialists comparable — the cross-model
signal is "two vendors reading the same brief", not "two prompts on
the same diff".

**Invocation** (only when `SKIP_COPILOT` is unset after both probes
above).

```bash
PROMPT_FILE=$(mktemp /tmp/copilot_adv_prompt.XXXXXX)
FINDINGS_FILE=$(mktemp /tmp/copilot_adv_findings.XXXXXX)
trap 'rm -f "$PROMPT_FILE" "$FINDINGS_FILE"' EXIT  # tempfile cleanup
# Build $PROMPT_FILE from the diff + the tier-appropriate prompt body above.

# Bound the call so a hung Copilot CLI (network drop, model overload,
# stuck stdin negotiation) doesn't block the entire review. 300 s is
# generous for a single-diff prompt; tune if Deep-tier prompts on
# large diffs need more.
timeout 300 "$COPILOT_BIN" -p "" --allow-all-tools < "$PROMPT_FILE" > "$FINDINGS_FILE" 2>&1
COPILOT_EXIT=$?

# Strip the trailing `Changes / Requests / Tokens` metadata block. The
# block starts with a "Changes" line and runs to EOF; cut from there.
# If a future Copilot CLI version changes the footer format, this is
# a no-op (no `^Changes$` line to match) and the metadata will leak
# into the report — visible enough to prompt a regex update.
sed -i '/^Changes$/,$d' "$FINDINGS_FILE"

# Post-invocation guard: route timeout / non-zero exit / empty output /
# auth-error text to the skipped-notice path rather than surfacing the
# stderr (captured via 2>&1) verbatim as findings.
if [[ "$COPILOT_EXIT" == "124" ]]; then
    SKIP_COPILOT=1
    COPILOT_SKIP_REASON="copilot CLI timed out after 300s"
elif [[ "$COPILOT_EXIT" != "0" ]]; then
    SKIP_COPILOT=1
    # Capture findings-file contents in the reason itself — the EXIT
    # trap will delete the tempfile before the user can inspect it.
    COPILOT_SKIP_REASON="copilot CLI exited $COPILOT_EXIT: $(head -c 200 "$FINDINGS_FILE")"
elif [[ ! -s "$FINDINGS_FILE" ]]; then
    SKIP_COPILOT=1
    COPILOT_SKIP_REASON="copilot produced no output (likely not authenticated)"
elif grep -qiE 'please run .copilot. to authenticate|not authenticated|sign in to GitHub' "$FINDINGS_FILE"; then
    SKIP_COPILOT=1
    COPILOT_SKIP_REASON="copilot CLI not authenticated"
fi
# Read $FINDINGS_FILE before the EXIT trap fires.
```

The empty-value form `-p ""` plus stdin is what activates Copilot's
headless mode; `--allow-all-tools` is required so the CLI doesn't
prompt for permission (which would hang on stdin). Smoke-tested
locally on `@github/copilot` v1.0.48.

**Security note on `--allow-all-tools`**. The flag grants Copilot
permission to execute any tool the CLI exposes (file reads, shell
commands, etc.) on this host. The two main use cases have different
threat models:

- **Pre-push mode** — the diff is the user's own authored work in
  their own worktree. The prompt is constructed locally, the worktree
  is already under the user's control. Accepted threat model.
- **Post-PR mode on owner / collaborator PRs** — same threat model as
  pre-push: the diff was authored by someone whose code we already
  treat as trusted.
- **Post-PR mode on external contributor PRs** — diff content is
  attacker-controlled. `--allow-all-tools` exposes file/shell access
  to that content's prompt-injection surface. The "untrusted-PR
  safety gate" above auto-suppresses 5e in this case; the
  `--allow-untrusted-copilot` flag is the explicit bypass after the
  reviewer has read the diff and accepted the risk. Don't add the flag
  to a CI config or a wrapper script that auto-invokes review-code on
  contributor PRs — that defeats the gate.

Reuse of this invocation pattern outside `review-code` should retain
the same trusted-input precondition or supply its own gate.

**Context cost**. Copilot autoloads workspace context on launch — a
trivial prompt floors at ~25.5k tokens and consumes one Premium
request. v1 accepts that cost as the tradeoff for default-on
all-tier activation. If the cost proves unacceptable in practice,
tighten via `--add-dir <worktree>` (scope context to the worktree) or
isolated `-C /tmp/scratch` invocation (diff + governance docs only).
See the cost-evaluation follow-up issue
([#467](https://github.com/rolker/ros2_agent_workspace/issues/467))
for the data-collection plan.

Report findings in the same format as other specialists. The silence
filter (step 6) deduplicates overlap with 5d and with the other
specialists.

### 6. Apply silence filter

Collect all findings from all dispatched specialists and filter:

1. **Deduplicate** — if multiple specialists flag the same issue (common
   between adversarial and governance), keep the more specific one.
2. **Drop linter-enforced nits** — if pre-commit or CI already catches
   it, don't report it again (the author will see it on commit/push).
3. **Merge related findings** — group findings about the same logical
   issue.
4. **Classify severity**:
   - **Must-fix** — bugs, security issues, principle violations, missing
     consequences
   - **Suggestion** — improvements worth the author's time
   - Drop anything below suggestion threshold.
5. **Silence check** — if no findings survive the filter, report "No
   issues found." Don't invent feedback to fill the report. Target: ≥85%
   of reported findings should be actionable.

### 7. Produce the report

```markdown
## Code Review: <#N or branch> — <title>

**PR**: <url>          (post-PR mode)
**Branch**: `<branch>` (pre-push mode)
**Issue**: #<issue> — <issue-title>
**Repo**: workspace | <project-repo>
**Files changed**: <count> (+<additions> -<deletions>)
**Review depth**: <Light|Standard|Deep> (reason: <primary signal>)
**Static analysis**: <run | skipped (--skip-static)>
**Copilot Adversarial**: <run | skipped (<reason>) | suppressed (--no-copilot)>
**Context**: <status of review-context.yaml — fresh / stale / not found / N/A>

### Must-Fix

| # | Source | File | Line | Finding |
|---|--------|------|------|---------|
| 1 | <specialist> | `path` | 42 | Description |

### Suggestions

| # | Source | File | Line | Finding |
|---|--------|------|------|---------|
| 1 | <specialist> | `path` | 10 | Description |

### Governance

| Principle | Verdict | Notes |
|---|---|---|
| ... | ... | ... |

| ADR | Triggered | Compliant | Notes |
|---|---|---|---|
| ... | ... | ... | ... |

| Changed | Required update | Status |
|---|---|---|
| ... | ... | Done / Missing |

### Plan Adherence

<comparison summary, or "No work plan found">

### Existing Review Comments

<post-PR mode only — summary of unresolved comments, if any>

### Summary

<1-3 sentence overall assessment>

### Recommended Actions

- [ ] <specific action items, if any>
```

**Light tier condensed format** — skip Governance, Plan Adherence, and
Existing Review Comments sections. Use:

```markdown
## Code Review: <#N or branch> — <title>

**PR / Branch**: ...
**Review depth**: Light (reason: <primary signal>)
**Static analysis**: skipped (--skip-static)         <!-- include only when SKIP_STATIC=true -->
**Copilot Adversarial**: <run | skipped (<reason>) | suppressed (--no-copilot)>

### Static Analysis

| # | File | Line | Finding |
|---|------|------|---------|
| 1 | `path` | 42 | Description |

### Copilot Adversarial

| # | File | Line | Finding |
|---|------|------|---------|
| 1 | `path` | 17 | Description |

<!-- Or, if the specialist was skipped or suppressed: -->
<!-- Copilot Adversarial skipped: <reason>     (when copilot unavailable) -->
<!-- (Section omitted entirely when invoked with --no-copilot) -->

No governance concerns for a change of this scope.
```

When `SKIP_STATIC=true` and **both** `SKIP_COPILOT=1` (from `--no-copilot` or the availability/gate probes in 5e) — i.e., Light + `--skip-static` with Copilot also suppressed = zero specialists — drop the `### Static Analysis` and `### Copilot Adversarial` tables entirely and use the No-findings format below. The two `<header>: skipped` lines carry the only signals that need to surface.

**No findings format** — if no findings exist across all sections:

```markdown
## Code Review: <#N or branch> — <title>

**PR / Branch**: ...
**Review depth**: <tier> (reason: <signal>)
**Static analysis**: skipped (--skip-static)         <!-- include only when SKIP_STATIC=true -->
**Copilot Adversarial**: <skipped (<reason>) | suppressed (--no-copilot)>  <!-- include only when SKIP_COPILOT=1 -->
No issues found. LGTM.
```

### 8. Persist review summary to progress.md

After outputting the report to the conversation, append a step entry to
`progress.md` so findings survive across sessions.

**Skip this entire step** when `NO_PROGRESS=true` (`--no-progress` flag,
pre-push only). Add a one-line note to the report Summary: "Progress
persistence skipped (--no-progress)." Use this when the review is on a
skill worktree or one-off branch that doesn't have an associated issue
and shouldn't accumulate a timeline. Step 8 also no-ops naturally when
no issue number can be resolved (rare field-mode case) — that's
distinct from the explicit opt-out and gets a different Summary
message: "Progress persistence skipped (no linked issue)."

**Locate or create progress.md**: Use the issue number resolved in step 1.
Determine which repo owns the linked issue (workspace repo for workspace
issues, project repo for project issues). Check
`.agent/work-plans/issue-<N>/progress.md` in the owning repo's worktree
first. If it doesn't exist there, check the current worktree. If neither
exists, create it in the owning repo's worktree (or current worktree if
no owning worktree exists). Fetch the issue title via:

```bash
gh issue view <N> --repo <owner/repo> --json title --jq '.title'
```

Frontmatter for new files:

```yaml
---
issue: <N>
---

# Issue #<N> — <issue title>
```

Append this step entry. The snippet below shows the post-PR header; in
pre-push mode change just the header to `## Local Review (Pre-Push)` so
the same issue can carry both a pre-push and a post-PR entry on its
timeline without one overwriting the other. Append only one header line
— never both.

```markdown

## Local Review
**Status**: complete
**When**: <YYYY-MM-DD HH:MM ±HH:MM>
**By**: <agent name> (<model>)
**Verdict**: <approved|changes-requested>

**PR**: #<pr-number> at `<short-sha>`       <!-- post-PR mode; omit in pre-push -->
**Branch**: <branch-name> at `<short-sha>`  <!-- pre-push mode; omit in post-PR -->
**Mode**: <pre-push | post-PR>
**Depth**: <tier> (reason: <signal>)
**Must-fix**: <count> | **Suggestions**: <count>

### Findings
- [ ] (must-fix) <one-line summary> — `file:line`
- [ ] (suggestion) <one-line summary> — `file:line`
```

If no findings survived the silence filter, set `**Verdict**: approved`,
`**Must-fix**: 0 | **Suggestions**: 0`, and write a single checkbox
item under `### Findings` so the section stays uniformly parseable
per ADR-0013's checkbox-list schema:
`- [ ] No issues found. LGTM.`

Key points:
- Use `- [ ]` checkboxes so findings can be checked off as addressed.
- Include only the one-line summary and location, not the full
  description.
- Commit progress.md after appending. Run `git add` and `git commit` in
  the worktree where progress.md was found or created (which may differ
  from the current working directory):
  ```bash
  git -C <worktree-path> add .agent/work-plans/issue-<N>/progress.md
  git -C <worktree-path> \
      -c user.name="$AGENT_NAME" \
      -c user.email="$AGENT_EMAIL" \
      commit -m "progress: local review for #<N>"
  ```
  The per-invocation `-c` overrides are required by
  [AGENTS.md § Agent Commit Identity](../../../AGENTS.md#agent-commit-identity)
  on agent-convention branches.
- If no issue number was resolved in step 1, skip persistence and note
  this in the report Summary ("Progress persistence skipped (no linked
  issue)") — the same canonical wording used in the step-8 intro above.

## Guidelines

- **Report first, then persist** — output the review in the conversation,
  append a step to `progress.md`, and commit it (step 8). The user
  decides whether to post it as a PR comment, request changes, or act on
  findings.
- **Be specific** — "Must-fix: null check missing before `result.data`
  access at line 42" is useful. "Watch: could add more error handling"
  is not.
- **Read the code** — don't just check file names. Read full files and
  the diff to evaluate correctness and principle adherence.
- **Silence is a feature** — saying nothing when there's nothing to say
  is better than generating low-value comments. If the code is fine, say
  so briefly.
- **Project governance** — for project repo PRs, apply both workspace and
  project governance. Note conflicts between them if any.
- **Severity matters** — every finding must be classified as must-fix or
  suggestion. Unclassified findings are noise.
- **Context-aware linting** — use ament configs for ROS package code,
  pre-commit configs for workspace infrastructure code. Never mix them.
- **Depth is transparent** — always show the tier and reason in the
  report header. If the user disagrees with the classification, they can
  re-run with an explicit depth keyword.
- **Pre-push mode caveats** — no PR comments, no PR body to extract a
  plan reference from, no `Closes #` link unless it's already in the
  branch name. Plan-drift still works (plan is a file in the repo);
  Existing-Review-Comments doesn't.
