---
name: review-code
description: Lead reviewer that orchestrates specialist sub-reviews (static analysis, governance, plan drift, adversarial, cross-model, local cross-model) to evaluate a PR or pre-push diff. Scales review depth to change risk. Produces a unified structured report and persists findings to progress.md.
---

# Review Code

## Usage

```
/review-code [--base <branch>] [--skip-static] [--no-progress] [--issue <N>] [--no-cross-model] [--local] [light|standard|deep]
                                            # pre-push: diff vs default branch
/review-code <pr-number-or-url> [--skip-static] [--no-cross-model] [--allow-untrusted-cross-model] [--local] [light|standard|deep]
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
- **`--no-cross-model`** (both modes) **opts out** of the Cross-Model
  Adversarial Specialist (5e) at Standard/Deep, where it is
  **default on** ([#660](https://github.com/rolker/ros2_agent_workspace/issues/660)).
  Pass it to skip the Gemini/Codex dispatch for this invocation — e.g. a
  field-mode host with neither CLI installed, or a quick re-review where
  the cross-model read isn't needed again. Light never dispatches 5e
  regardless of this flag.
- **`--local`** (both modes) opts in to the Local Model Adversarial
  Specialist (5f). Local review is **off by default**: it costs no API
  quota (local Ollama inference), but on current hardware it is by far
  the long-pole specialist (~7 min per run, and a ~500-line diff can
  exceed 10 min on 8GB-VRAM-class hardware — see
  [#570](https://github.com/rolker/ros2_agent_workspace/issues/570) /
  [#590](https://github.com/rolker/ros2_agent_workspace/issues/590)),
  so the wall-clock cost is paid only when chosen. Pass `--local` when
  a quota-free cross-model read is worth the wait — e.g. a high-stakes
  diff, or to keep the offline field-mode reviewer exercised. When
  opted in but the Ollama server or the model is unavailable, the
  specialist skips itself with a one-line notice. (`--no-local` is
  accepted as a deprecated no-op, since local review is already off by
  default.)
- **`--allow-untrusted-cross-model`** (post-PR only) overrides the
  external-PR safety gate that suppresses Cross-Model Adversarial when
  the PR head is from a fork or a non-collaborator author. Without this
  flag, 5e routes to the skipped-with-notice path on such PRs because
  the codex arm runs with local filesystem read access; running it
  against an untrusted contributor's diff exposes that capability to
  attacker-controlled prompt content. Pass only when you have read the
  diff and accept the risk. Meaningless (and a no-op) alongside
  `--no-cross-model`.

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
- **Light** — Static Analysis + one Claude Adversarial pass (small,
  low-risk changes)
- **Standard** — Static Analysis + Governance + Plan Drift + two
  disjoint-lens Claude Adversarial passes + Cross-Model Adversarial
  (Gemini + Codex, default on) (medium changes or governance-touching
  files)
- **Deep** — Same as Standard with the two Claude Adversarial passes
  primed for security / concurrency / lifecycle (large changes,
  security, or cross-layer)

Cross-Model Adversarial is **default on at Standard + Deep**, opt-out
via `--no-cross-model`
([#660](https://github.com/rolker/ros2_agent_workspace/issues/660)) — it
runs as an additional cross-model read (Gemini + Codex, in parallel) on
top of the in-house Claude passes. Local Model Adversarial is
**opt-in at every tier** via `--local` — quota-free local inference,
but the wall-clock long pole on current hardware
([#590](https://github.com/rolker/ros2_agent_workspace/issues/590)).

**Specialists**:
- **Static Analysis** — runs linters with ament-aligned configs on
  changed files
- **Governance** — evaluates against principles, ADRs, and the
  consequences map
- **Plan Drift** — compares implementation against the work plan (if one
  exists)
- **Claude Adversarial** — fresh-context Claude subagent(s) that re-read
  the diff cold. One pass at Light; **two passes with disjoint lenses**
  at Standard + Deep (Lens A: logic / edge cases / assumptions; Lens B:
  security / concurrency / lifecycle / cross-cutting). Two independent
  in-house reads are the default adversarial signal.
- **Cross-Model Adversarial** — **default on at Standard + Deep**
  (`--no-cross-model` to opt out) synchronous Gemini + Codex dispatch
  via `.agent/scripts/cross_model_review.sh` that re-reads the diff
  cold, adding a true second/third-vendor read. A per-agent failure
  (CLI unavailable, timeout, auth error) is noted in the report without
  failing the review.
- **Local Model Adversarial** — **opt-in** (`--local`) cross-model
  read by a locally served Ollama model (default `qwen3.5:35b`) via
  `.agent/scripts/local_review.sh`. Quota-free but low-trust:
  validated at ~50% precision, so its findings need corroboration or
  spot-checking before they reach must-fix (see 5f). Off by default —
  the run is the review's wall-clock long pole on current hardware
  ([#590](https://github.com/rolker/ros2_agent_workspace/issues/590)).
  Skips itself with a one-line notice when opted in but the server or
  model is unavailable.

## Steps

### 1. Detect mode and gather diff context

Parse the arguments in this order: extract `--skip-static` (sets
`SKIP_STATIC=true`), `--no-progress` (sets `NO_PROGRESS=true`, pre-push
only — emit an error if passed in post-PR mode), `--issue <N>` (sets
`USER_ISSUE=<N>`, pre-push only — emit an error if passed in post-PR
mode, where `closingIssuesReferences` is authoritative),
`--no-cross-model` (sets `NO_CROSS_MODEL=1`, opts out of the Cross-Model
Adversarial Specialist in step 5e — default on at Standard/Deep),
`--allow-untrusted-cross-model` (sets `ALLOW_UNTRUSTED_CROSS_MODEL=1`,
post-PR only — emit an error if passed in pre-push mode, where the gate
doesn't apply; with `NO_CROSS_MODEL=1` it has no effect, so emit a
one-line note "`--allow-untrusted-cross-model` ignored: no effect with
`--no-cross-model`" rather than silently dropping it), `--local` (sets
`LOCAL=1`, opts in to the Local Model Adversarial Specialist in
step 5f — off by default), `--no-local` (**recognized and
discarded**: a deprecated no-op since local review is already off by
default — consume the token here so it never falls through to the
classification step below as a stray argument), `--base <branch>`
(sets `USER_BASE`), then the optional depth keyword (`light` /
`standard` / `deep` — positional). Classify what remains:

- Empty → **pre-push mode**
- A number or `https://github.com/.../pull/<N>` → **post-PR mode**

The depth keyword, `--skip-static`, and `--no-cross-model` may appear
in any order around the PR number / URL or `--base <branch>`. The same
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
/review-code --no-cross-model               # pre-push, opt out of Cross-Model Adversarial
/review-code --local                        # pre-push, opt in to Local Model Adversarial
/review-code 42                             # post-PR, auto-classify
/review-code 42 standard                    # post-PR, force Standard
/review-code 42 --skip-static               # post-PR, skip static analysis
/review-code 42 --allow-untrusted-cross-model  # post-PR, bypass the fork/non-collaborator gate
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
- **5d. Claude Adversarial Specialist** — **one pass** (Lens A only)
- **5f. Local Model Adversarial Specialist** — only if `LOCAL=1`
  (`--local`); skipped with notice if Ollama/model unavailable

5e (Cross-Model Adversarial) never runs at Light, regardless of
`--no-cross-model` — it is a Standard/Deep specialist.

#### Standard tier

Run all of:
- **5a. Static Analysis Specialist**
- **5b. Governance Specialist**
- **5c. Plan Drift Specialist** (if a plan exists)
- **5d. Claude Adversarial Specialist** — **two passes** with disjoint
  lenses (Lens A + Lens B; Standard prompt)
- **5e. Cross-Model Adversarial Specialist** — **default on**, unless
  `NO_CROSS_MODEL=1` (`--no-cross-model`); a per-agent failure (CLI
  unavailable, timeout, gated on an untrusted PR) is noted with a
  one-line notice per agent, not a whole-specialist skip
- **5f. Local Model Adversarial Specialist** — only if `LOCAL=1`
  (`--local`); skipped with notice if Ollama/model unavailable

#### Deep tier

Same as Standard, but the two **5d. Claude Adversarial** passes run with
the Deep prompt (broader file horizon plus an explicit security /
concurrency / lifecycle checklist). **5e. Cross-Model Adversarial** runs
as at Standard (its prompt is not tier-differentiated the way 5d's is —
see 5e's "Prompt body" note). If opted in with `--local`, **5f. Local
Model Adversarial** runs as at other tiers (its prompt is not
tier-differentiated — the diff is the whole horizon a local model can
reliably handle).

---

#### 5a. Static Analysis Specialist

**Skip if `SKIP_STATIC=true`** (`--skip-static` flag passed in step 1):
emit no findings for this specialist and note "Static analysis skipped
(--skip-static)" so the report header can surface it. Light tier always
runs the single Claude Adversarial pass (5d), so `--skip-static` on
Light still produces an adversarial read — there is no longer a
zero-specialist path at Light. (The only way to reach zero specialists
is the unsupported combination of skipping static analysis on a tier
where every other specialist is also inapplicable; the silence filter
produces the "No findings" output if that ever occurs.)

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

**Doc-impact check**: Beyond the map, verify the change carries its
documentation consequences and check the plan's `## Documentation &
Instruction Impact` section against the diff. (Tier note: 5b runs at
Standard and Deep only — a Light-tier diff gets no reviewer-side doc-impact
check and relies on the plan section plus the review-plan step-4 dimension;
that is an accepted gap, sized to Light's <50-line no-trigger diffs.)

- If the PR changes package **parameters, topics, or services**, confirm
  the package README / API docs — and `.agents/review-context.yaml` if it
  maps them — were updated in the same PR. Stale docs are **Missing**.
- If implementation **surfaced a reusable pattern or pitfall**, check
  whether the plan flagged it as an instruction-update candidate. If a
  pattern clearly worth capturing was missed, raise it as a **candidate**
  (a proposal for the operator — never an auto-applied edit to
  `.agent/knowledge/`, `.agents/README.md`, or an instruction file).

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

**Activates at**: Light (one pass), Standard, Deep (two passes each).

Launch as **fresh Claude subagent(s)** via the `Agent` tool with no
context from the other specialists. The adversarial reviewer reads the
diff and full changed files independently — that fresh-context dispatch
is the whole point. An independent reviewer that agrees with the
governance specialist is a stronger signal than one told what to look
for.

**Two disjoint-lens passes at Standard + Deep.** Independent of 5e's
Gemini/Codex cross-model read, the in-house cross-read signal — two
*independent* readers, not one model agreeing with itself — comes from
running **two separate fresh subagents with non-overlapping focus
areas**. Each pass is its own `Agent` dispatch; they do not share
context with each other or with the other specialists. Dispatch them in
parallel.

- **Lens A — logic & correctness**:
  - Missed edge cases and boundary conditions
  - Assumption violations (what does the code assume that might not hold?)
  - Subtle bugs (off-by-one, race conditions, resource leaks)
  - Logic errors (does the code actually do what the PR claims?)
- **Lens B — systemic & safety**:
  - Security implications (injection, auth bypass, data exposure)
  - Concurrency / lifecycle (lock ordering, init/destroy ordering, signal
    handling, shutdown paths)
  - Cross-cutting effects (does this change interact with caching,
    retries, error propagation, or other system-wide behavior?)

**Per-tier dispatch**:
- **Light** — a **single** pass using **Lens A** only (small, low-risk
  changes rarely need the systemic lens; keeps Light fast).
- **Standard** — **both** lenses, each at the standard file horizon
  (diff + directly-touched files).
- **Deep** — **both** lenses at a **broader file horizon** (callers,
  callees, and cross-module consumers of changed symbols) with
  heightened scrutiny. The Standard→Deep difference is horizon and
  rigor, not which lenses run.

Giving each pass a single lens keeps the two reads genuinely
independent rather than one prompt restating another; the silence
filter (step 6) folds any overlap between Lens A and Lens B into a
single finding and flags it as cross-pass confirmed.

**Cross-repo limitation**: each Adversarial pass only sees the diff
and the files it explicitly opens. In a layered workspace, cross-repo
consequences (a workspace ADR change that affects project repos, a shared
message-package edit that breaks downstream nodes) won't surface from
fresh-context reading alone. The Governance Specialist carries that load
via the consequences map; Adversarial is not a substitute for it.

Report findings in the same format as other specialists (file, line,
severity, description). Label each finding's source with its lens
(`Claude Adversarial / Lens A`, `Claude Adversarial / Lens B`) so the
report shows which read caught it.

**Cross-model adversarial** coverage comes from step 5e below — an
independent second-vendor read (Gemini + Codex) on top of the two
in-house Claude lenses, dispatched via `cross_model_review.sh`
(ported from `rolker/agent_workspace`, ADR-0015).

#### 5e. Cross-Model Adversarial Specialist (Gemini + Codex)

**Activates at**: Standard + Deep, **default on**. `--no-cross-model`
opts out for this invocation (both pre-push and post-PR modes). Light
never dispatches it (matches 5b/5c's Light exclusion).

An independent cross-model pass dispatched by
`.agent/scripts/cross_model_review.sh` (ported from
`rolker/agent_workspace`'s ADR-0015 parallel-sync dispatch design,
issue #660) — Gemini via the `agy` CLI and Codex via `codex exec`, both
run in parallel, blocking until the slower finishes. Same fresh-context
principle as 5d — each CLI sees only the diff prompt, no other
specialists' findings. The value this buys over the two in-house Claude
lenses is a genuine second (and third) *vendor* independently reading
the same diff.

**Agent selection**: always `gemini,codex` — there is no Copilot arm in
this specialist (the Copilot CLI runs Claude or GPT models under the
hood, so it added no new vendor next to Codex, and cost Premium quota;
removed entirely — see this issue's plan). If the calling agent *is*
Gemini CLI or Codex CLI, drop that one agent from the `--agents` list
rather than having it review itself: determine the caller's framework
from `$AGENT_FRAMEWORK` if set, else `source
.agent/scripts/detect_cli_env.sh || true` (it has no Codex CLI
detection, so a Codex caller is recognised only through
`$AGENT_FRAMEWORK`); normalize (lowercase,
`gemini-cli`→`gemini`, `codex-cli`→`codex`) and remove a matching entry.

**Untrusted-PR safety gate** (post-PR mode only). Even though neither
`_agy_review.sh` nor claude/gemini's arms of `_cli_review.sh` grant tool
access, the codex arm runs with `-s read-only -a never` pinned but still
has local filesystem *read* access — it could read and quote local
files (env, other repos, credentials) into review text under a
prompt-injected diff. Before dispatch, check whether the PR head is
from a fork or a non-collaborator author:

```bash
if [[ "$MODE" == "post-PR" ]]; then
    # The repo passed as --repo below, else the current checkout's.
    REPO_SLUG="${REPO_SLUG:-$(gh repo view --json nameWithOwner --jq .nameWithOwner)}"
    # `gh pr view --json` exposes neither the author's association nor the
    # base repo, so both come from the REST pull object. A failed lookup
    # (gh prints the error body on stdout, so test its status) skips the
    # specialist with a reason: it fails closed, never reads as trusted.
    if PR_TRUST=$(gh api "repos/${REPO_SLUG}/pulls/${PR}" \
            --jq '[.author_association, (if .head.repo.full_name == .base.repo.full_name then "owner" else "fork" end)] | @tsv' 2>/dev/null); then
        IFS=$'\t' read -r PR_AUTHOR_ASSOC PR_HEAD_REPO <<< "$PR_TRUST"
    else
        PR_AUTHOR_ASSOC="" PR_HEAD_REPO=""
    fi
    # Trusted: PR head is the base repo AND author is OWNER/MEMBER/COLLABORATOR.
    if [[ -z "$PR_HEAD_REPO" ]]; then
        SKIP_CROSS_MODEL=1
        CROSS_MODEL_SKIP_REASON="could not look up PR #${PR}'s author and head repo in ${REPO_SLUG}; pass --allow-untrusted-cross-model after reviewing the diff to bypass"
        [[ "$ALLOW_UNTRUSTED_CROSS_MODEL" == "1" ]] && SKIP_CROSS_MODEL=0
    elif [[ "$PR_HEAD_REPO" != "owner" ]] || \
       [[ "$PR_AUTHOR_ASSOC" != "OWNER" && "$PR_AUTHOR_ASSOC" != "MEMBER" && "$PR_AUTHOR_ASSOC" != "COLLABORATOR" ]]; then
        if [[ "$ALLOW_UNTRUSTED_CROSS_MODEL" == "1" ]]; then
            : # User explicitly bypassed the gate. Proceed.
        else
            SKIP_CROSS_MODEL=1
            CROSS_MODEL_SKIP_REASON="external PR (head=$PR_HEAD_REPO, author=$PR_AUTHOR_ASSOC); pass --allow-untrusted-cross-model after reviewing the diff to bypass"
        fi
    fi
fi
```

When skipped by this gate, the report includes:
`Cross-Model Adversarial skipped: <CROSS_MODEL_SKIP_REASON>`. Pre-push
mode never gates — the diff is the user's own authored work in their
own worktree, same threat model as 5d.

**Invocation**. `cross_model_review.sh` resolves each agent's binary
itself and marks an unavailable one as a per-agent failure without
aborting the run, so no separate availability-probe block is needed the
way the deleted Copilot specialist needed one.

```bash
PROMPT_PLAN_CONTEXT=""  # cross_model_review.sh reads the plan itself if one exists

# Bash-tool-safe bounds: the Bash tool's hard cap is 600s. Keeping every
# agent's OUTER bound below it means the script's own timeout fires and
# emits its EXIT= marker BEFORE the harness kills the whole invocation on
# a wedged CLI — a hard kill prints no triplet for ANY agent, since the
# triplets follow the last one. Outer bounds: codex = AGENT_TIMEOUT +
# AGENT_KILL_AFTER (10s default) = 490s; gemini = AGY_PRINT_TIMEOUT +
# GEMINI_BACKSTOP_MARGIN + AGENT_KILL_AFTER = 490s. The margin must be set
# too: its 300s default alone would put gemini at 730s. AGY_PRINT_TIMEOUT
# is a Go duration and NEEDS its unit — a bare `420` exits 2 before any
# agent runs. Observed on this workspace: codex ~1 min, gemini ~5 min on
# a ~350 KB Deep prompt. Where the runtime can background a command (no
# cap), leave all four unset and use the script's defaults instead.
export AGENT_TIMEOUT=480
export AGY_PRINT_TIMEOUT=420s
export GEMINI_BACKSTOP_MARGIN=60

# Post-PR mode
.agent/scripts/cross_model_review.sh --pr <N> --agents gemini,codex --repo owner/repo

# Pre-push mode — pass the origin/$BASE ref step 1 just fetched. Without
# a base the script diffs against the LOCAL default branch when one
# exists, and in this workspace the main tree's local branch moves only
# on `make sync`, so a stale one puts already-merged commits in the prompt.
.agent/scripts/cross_model_review.sh --branch "origin/$BASE" --agents gemini,codex [--no-progress]
```

Pass `--repo <owner/repo>` (post-PR mode) when the PR lives in a
different repo than the current working directory. Pass `--no-progress`
in pre-push mode for skill worktrees / one-off branches (mirrors the
top-level `--no-progress` flag's own reason for existing).

**Reading results**: stdout carries `MODE=parallel-sync` and then one
`AGENT=`/`FINDINGS_FILE=`/`EXIT=` triplet per agent. Key on each agent's
`EXIT=` line, not the script's overall exit status — the script exits 3
whenever *any* agent failed, but a failed agent (CLI not installed,
timeout, non-zero exit, empty response, or a structured error) is noted
in the report while the other's findings are used as normal; one
agent's failure never blocks the other and never fails the review. Exit
3 with **no** `AGENT=` triplets means the shared prompt could not be
built (diff fetch failed or empty) — nothing ran. Exit 1 means no listed
agent had a usable CLI, or (post-PR only) `gh` itself is missing.
Either way, report the specialist as unavailable rather than silently
omitting it. A findings file never holds a half-review — each agent's
helper (`_agy_review.sh` for gemini, `_cli_review.sh` for codex)
truncates it first and writes either the review text or the failure
reason, so a completed-but-empty or auth-error response is distinguished
from a genuine zero-finding review.

**Trust weighting**: full weight — same standing 5d carries in the
silence filter (step 6), not 5f's low-trust discount. A finding
corroborated across gemini/codex/Claude is flagged as cross-model
confirmed.

**Prompt body and `## Plan Context`**. Each agent receives the diff plus
the combined Lens A + Lens B focus areas (edge cases, assumptions,
subtle bugs, logic errors, plus security/concurrency/lifecycle/
cross-cutting) — there is no per-lens split for the cross-model read,
matching how the deleted Copilot specialist ran. When a work plan exists
at `.agent/work-plans/issue-<N>/plan.md`, `cross_model_review.sh`
appends a `## Plan Context` section (the plan's `## Approach`, capped at
200 lines) via `_plan_approach.py`'s CommonMark parser — framed as
context ("flag divergences, do not review the plan"), never included
under `--no-progress`. If `markdown-it-py` isn't installed (`make lint`
hasn't populated `.venv` yet), the extractor degrades to one `WARNING:`
line and omits the section rather than failing the run.

Report findings in the same format as other specialists. The silence
filter (step 6) deduplicates overlap with 5d and with the other
specialists.

#### 5f. Local Model Adversarial Specialist

**Opt-in at every tier** via `--local` (`LOCAL=1`); off by default
([#590](https://github.com/rolker/ros2_agent_workspace/issues/590)).
When `LOCAL` is unset, skip the entire specialist and omit it from the
report — absence is the default state.

A quota-free cross-model pass served by a **local Ollama model**
(default `qwen3.5:35b`), dispatched through
`.agent/scripts/local_review.sh`. Same fresh-context principle as
5d/5e — the model sees only the diff and a one-line task context, no
other specialists' findings. Unlike 5e's codex arm, the local model gets
**no tool access at all** (prompt-only HTTP call), so there is no
untrusted-PR gate: adversarial diff content can at worst skew its
findings, which the low-trust weighting below already discounts.

**Invocation** (run in parallel with the other specialists — it is
typically the wall-clock long pole, ~7 min on the reference
workstation):

```bash
CTX_FILE=$(mktemp /tmp/local_review_ctx.XXXXXX)
LOCAL_FINDINGS=$(mktemp /tmp/local_review_out.XXXXXX)
LOCAL_ERR=$(mktemp /tmp/local_review_err.XXXXXX)
trap 'rm -f "$CTX_FILE" "$LOCAL_FINDINGS" "$LOCAL_ERR"' EXIT
echo "<one-line summary: issue #N — what the change does>" > "$CTX_FILE"

# Same diff the other specialists reviewed (step 1): origin/$BASE in
# pre-push mode, the PR diff in post-PR mode — never the local base
# branch, which may be stale in a worktree.
LOCAL_EXIT=0
if [[ "$MODE" == "post-PR" ]]; then
    gh pr diff "$PR" \
        | "$REPO_ROOT/.agent/scripts/local_review.sh" --context "$CTX_FILE" \
        > "$LOCAL_FINDINGS" 2>"$LOCAL_ERR" || LOCAL_EXIT=$?
else
    git diff "origin/$BASE...HEAD" \
        | "$REPO_ROOT/.agent/scripts/local_review.sh" --context "$CTX_FILE" \
        > "$LOCAL_FINDINGS" 2>"$LOCAL_ERR" || LOCAL_EXIT=$?
fi
# The `|| LOCAL_EXIT=$?` form keeps the assignment reachable under an
# errexit shell — a bare `LOCAL_EXIT=$?` on the next line would never
# run when the pipeline fails and `set -e` is active.
# Exit 2 = unavailable (no server / model not pulled / no jq): skip
#          with the one-line reason from $LOCAL_ERR — not a failure.
# Exit 1 (or anything else non-zero) = invocation error: skip with
#          notice; never let a local-model hiccup block the review.
# Exit 0 = findings in $LOCAL_FINDINGS.
```

The helper handles the availability probes, the bounded timeout
(`LOCAL_REVIEW_TIMEOUT`, default 900 s — raise it for large diffs;
reasoning time grows with diff size), and reasoning-model plumbing
(HTTP API with `think: true` — the `ollama run` CLI path can swallow
the entire answer after a full reasoning pass; see the script header).
Model and endpoint are overridable via `LOCAL_REVIEW_MODEL` /
`LOCAL_REVIEW_URL` for hosts with different local models.

When skipped (either exit code), the report includes one line:
`Local Adversarial skipped: <stderr reason>`.

**Low-trust weighting.** Calibration on a known-ground-truth diff
([#570](https://github.com/rolker/ros2_agent_workspace/issues/570)):
qwen3.5:35b caught 2 of 4 real findings a full review round had
caught (including one of Copilot's), but ~half its findings were
speculative. Treat this source accordingly in step 6:

- A local finding **corroborated** by any other specialist is a strong
  cross-model confirmation — flag it as such (same as a 5e
  cross-confirmation).
- An **uncorroborated** local finding enters the report at
  **Suggestion severity at most**, and only after the lead reviewer
  spot-checks it against the diff (its confident tone is not
  evidence). Discard findings that reference unchanged context lines
  or environments the code visibly does not target — those are its
  documented failure modes.

The specialist is opt-in
([#590](https://github.com/rolker/ros2_agent_workspace/issues/590)):
the real catches come at zero quota cost, but on current hardware the
run is the review's wall-clock long pole, so that cost is paid only
when a reviewer chooses it. The same helper remains the offline
field-mode reviewer — opt in on a dev review periodically to keep it
exercised and trustworthy in the field.

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
   - **Guidance-doc calibration (#537).** When the changed file is a
     *guidance document* — prose an agent reads and *adapts* (a
     `.claude/skills/**/SKILL.md`, a `.agent/knowledge/` doc, an ADR) rather
     than code it *executes verbatim* — apply a lighter bar: a finding an agent
     following the procedure would obviously catch or adapt on its own is a
     **Suggestion, not a Must-fix**. Reserve Must-fix for guidance that would
     actively mislead — a command that fails silently, a wrong path/flag, an
     internal contradiction, a missing consequence. A SKILL.md is applied with
     judgment, not executed literally; holding prose to executable-grade
     precision is what makes a review loop fail to converge. (Genuinely
     executable snippets *inside* a guidance doc — a copy-paste command block —
     keep the code bar.)
5. **Silence check** — if no findings survive the filter, report "No
   issues found." Don't invent feedback to fill the report. Target: ≥85%
   of reported findings should be actionable.

**Convergence assessment (pre-push, #537).** Before writing the report, in
pre-push mode, assess whether the review loop is converging — so the host
(`/run-issue`) gets a ship-vs-continue signal instead of looping a guidance-doc
review indefinitely:

- **Round** = the count of prior `## Local Review (Pre-Push)` entries for this
  branch in `progress.md`, plus 1 (this review). Round 1 has no prior entry.
- **Ship verdict**:
  - **recommended** when there are **no Must-fix** findings (only Suggestions,
    or nothing) — the diff is shippable; remaining Suggestions can be applied or
    tracked.
  - **recommended** at **round ≥ 2** when the Must-fix count is **low and not
    rising** versus the previous round and the remaining must-fixes are
    mechanical/clear (low ≈ **≤ 2** must-fixes, each a precise file:line fix with
    an obvious correction — not a design question) — recommend addressing them
    and shipping rather than another full round (each round costs a full
    dispatch + re-review cycle for diminishing return, especially on guidance
    docs after the calibration above).
  - **continue** when Must-fix is **rising, high, or includes a genuine
    design / correctness concern** that warrants another independent read.
- Surface the round, verdict, and a one-line reason in the report header
  (`Round`) and Summary, and in the `progress.md` entry (step 8), so the
  orchestrator can route on it. The verdict is **advisory** — the operator/host
  decides; this skill never blocks a ship.

### 7. Produce the report

```markdown
## Code Review: <#N or branch> — <title>

**PR**: <url>          (post-PR mode)
**Branch**: `<branch>` (pre-push mode)
**Issue**: #<issue> — <issue-title>
**Repo**: workspace | <project-repo>
**Files changed**: <count> (+<additions> -<deletions>)
**Review depth**: <Light|Standard|Deep> (reason: <primary signal>)
**Round**: <N> (pre-push) — **Ship: <recommended | continue>** (see Convergence)
**Static analysis**: <run | skipped (--skip-static)>
**Claude Adversarial**: <1 pass (Lens A) | 2 passes (Lens A + Lens B)>
**Cross-Model Adversarial**: <run (gemini,codex) | run (gemini) — codex unavailable | off (--no-cross-model) | skipped (<reason>, --allow-untrusted-cross-model to bypass)>
**Local Adversarial**: <off (default) | run (<model>, --local) | skipped (<reason>, --local)>
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
**Round**: <N> (pre-push) — **Ship: <recommended | continue>**   <!-- pre-push only; see Convergence assessment -->
**Static analysis**: skipped (--skip-static)         <!-- include only when SKIP_STATIC=true -->
**Claude Adversarial**: 1 pass (Lens A)
**Local Adversarial**: <off (default) | run (<model>, --local) | skipped (<reason>, --local)>

### Static Analysis

| # | File | Line | Finding |
|---|------|------|---------|
| 1 | `path` | 42 | Description |

### Claude Adversarial

| # | File | Line | Finding |
|---|------|------|---------|
| 1 | `path` | 17 | Description |

<!-- No Cross-Model Adversarial line/section at Light — 5e never dispatches at this tier. -->

### Local Adversarial (<model>)

| # | File | Line | Finding |
|---|------|------|---------|
| 1 | `path` | 23 | Description (cross-model confirmed by <specialist> / uncorroborated — spot-checked) |

<!-- Local Adversarial skipped: <reason>   (when opted in but Ollama/model unavailable or errored) -->
<!-- Omitted entirely by default, when --local was not passed. -->

No governance concerns for a change of this scope.
```

Light always runs at least Static Analysis + the single Claude
Adversarial pass, so the report always has an adversarial section. (The
former Light + `--skip-static` "zero specialists" case no longer exists,
since Claude Adversarial is now unconditional at Light.)

**No findings format** — if no findings exist across all sections:

```markdown
## Code Review: <#N or branch> — <title>

**PR / Branch**: ...
**Review depth**: <tier> (reason: <signal>)
**Static analysis**: skipped (--skip-static)         <!-- include only when SKIP_STATIC=true -->
**Cross-Model Adversarial**: <run (gemini,codex) | off (--no-cross-model) | skipped (<reason>)>  <!-- include only at Standard/Deep; shows the default-on pass ran clean vs. was opted out / skipped -->
**Local Adversarial**: <run (<model>, --local) | skipped (<reason>, --local)>  <!-- include only when LOCAL=1; shows that an opted-in local pass ran-clean vs. was skipped -->
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

File creation (parent dir + frontmatter `issue: <N>` + `# Issue #<N> — <issue
title>` heading) is handled by `progress_append.sh` below — pass the fetched
title via `--title`.

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
**Round**: <N> | **Ship**: <recommended | continue> — <one-line reason>  <!-- pre-push only; from the Convergence assessment (#537) -->

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
- Append **and** commit in one prompt-free step via
  [`progress_append.sh`](../../../.agent/scripts/progress_append.sh)
  ([#594](https://github.com/rolker/ros2_agent_workspace/issues/594)) — never
  inline `cat >>` + `git commit` (both prompt). `-C` targets the worktree
  where progress.md was found or created (which may differ from the current
  working directory); the script creates the file with frontmatter if absent,
  commits only that file, and forms the `progress: <entry type> for #<N>`
  message from the entry heading:
  ```bash
  .agent/scripts/progress_append.sh -C <worktree-path> <N> --title "<issue title>" <<'ENTRY'
  ## Local Review (Pre-Push)
  ...the entry as specified above...
  ENTRY
  ```
  Identity comes from `$AGENT_NAME`/`$AGENT_EMAIL` (or `--name`/`--email`);
  the script fails loud when unset, satisfying
  [AGENTS.md § Agent Commit Identity](../../../AGENTS.md#agent-commit-identity)
  on agent-convention branches.
- If no issue number was resolved in step 1, skip persistence and note
  this in the report Summary ("Progress persistence skipped (no linked
  issue)") — the same canonical wording used in the step-8 intro above.

### Next step

Lifecycle: **Local Review** → push / open PR → **triage-reviews**

That path is for an **approved** pre-push review. If the verdict is
**changes-requested**, the host (`/run-issue`) instead dispatches
**`address-findings`** to work the open findings from this
`## Local Review (Pre-Push)` entry, then re-dispatches `review-code` — the diff
is not pushed until a pre-push review comes back **approved**.

Once findings are addressed and the branch is pushed (or a PR opened), hand off
to the next phase in a **fresh-context sub-agent** — independence between
lifecycle steps is what makes the timeline trustworthy. Use the dispatcher:

    .agent/scripts/dispatch_subagent.sh --mode in-process --issue <N> --skill triage-reviews

The sub-agent reads the last `## Local Review` entry in
`.agent/work-plans/issue-<N>/progress.md` for your output (plus live PR review
comments), and writes its own `## Integrated Review` entry when done.

**No auto-chaining (Scope E):** this skill never dispatches the next phase itself —
the host orchestrator (`/run-issue`,
[#492](https://github.com/rolker/ros2_agent_workspace/issues/492)) drives,
pausing at user checkpoints. This step only emits this prompt and its
`progress.md` entry.

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
