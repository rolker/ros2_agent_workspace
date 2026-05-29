#!/bin/bash
# .agent/scripts/merge_pr.sh
# Merge a PR, remove its worktree, delete the local+remote branch, and `make sync`.
#
# Keyed on the WORKTREE / ISSUE, not the global PR number — PR numbers are
# per-repo, so a bare PR number is ambiguous across the workspace repo and the
# many project repos under layers/main/*_ws/src/*. The PR number is *derived*
# from (repo, branch). See issue #488 / its work plan.
#
# Usage (resolution, most-natural first):
#   merge_pr.sh                       # cwd is inside a worktree → merge that branch's PR
#   merge_pr.sh --issue <N> [--repo-slug <slug>]
#   merge_pr.sh --pr <N> --repo-slug <slug>   # escape hatch: headless / no worktree
# Options:
#   --no-wait    skip the pre-merge CI wait (use when CI is known green)
#
# Field-mode repos (non-GitHub origin) have no GitHub PR; this script refuses
# them before any `gh` call (ADR-0011) — they use the field workflow.
#
# Steps: resolve → field-mode guard → wait for CI → merge (--merge) →
#        remove worktree → delete branches → make sync.

set -eo pipefail

if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
    echo "Error: execute this script, don't source it." >&2
    return 1 2>/dev/null || exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=.agent/scripts/_worktree_helpers.sh
source "$SCRIPT_DIR/_worktree_helpers.sh"   # extract_gh_slug
# shellcheck source=.agent/scripts/field_mode.sh
source "$SCRIPT_DIR/field_mode.sh"          # is_field_mode

# Workspace root = the main worktree (always printed first by `git worktree list`,
# in absolute form, regardless of invocation cwd). Robust when invoked from
# inside a worktree (cf. agent_workspace #146).
ROOT_DIR=$({ git -C "$SCRIPT_DIR" worktree list --porcelain 2>/dev/null \
    | head -n1 | sed 's/^worktree //'; } || true)
if [[ -z "$ROOT_DIR" ]]; then
    echo "ERROR: merge_pr.sh must run from within a git repository." >&2
    exit 1
fi

# ---- args -------------------------------------------------------------------
ARG_ISSUE=""; ARG_REPO_SLUG=""; ARG_PR=""; NO_WAIT=false
while [[ $# -gt 0 ]]; do
    case "$1" in
        --issue)
            if [[ $# -lt 2 ]]; then echo "ERROR: missing value for --issue" >&2; exit 2; fi
            ARG_ISSUE="$2"; shift 2 ;;
        --repo-slug)
            if [[ $# -lt 2 ]]; then echo "ERROR: missing value for --repo-slug" >&2; exit 2; fi
            ARG_REPO_SLUG="$2"; shift 2 ;;
        --pr)
            if [[ $# -lt 2 ]]; then echo "ERROR: missing value for --pr" >&2; exit 2; fi
            ARG_PR="$2"; shift 2 ;;
        --no-wait)   NO_WAIT=true; shift ;;
        *) echo "ERROR: unknown argument: $1" >&2
           echo "Usage: $0 [--issue <N> | --pr <N>] [--repo-slug <slug>] [--no-wait]" >&2
           exit 2 ;;
    esac
done

# Resolution outputs:
REPO_PATH=""     # local git working tree (for field-mode check + branch delete)
GH_REPO=""       # owner/repo for `gh -R`
BRANCH=""        # feature/issue-<N>
ISSUE_NUM=""
REPO_SLUG=""     # "" for the workspace repo; project-repo dir name for a layer repo
PR_NUM="$ARG_PR"
HAVE_WORKTREE=false  # true when resolution found a real worktree (cwd/--issue);
                     # false for the headless --pr escape hatch. Gates removal
                     # without reconstructing the (sanitized) worktree dir name.

issue_from_branch() {  # echo the issue number from a feature/issue-<N> branch
    echo "$1" | sed -nE 's#^feature/[iI][sS][sS][uU][eE]-([0-9]+).*#\1#p'
}

# Given a worktree dir, echo the project-repo git toplevel inside it. Workspace
# worktrees ARE git repos. A layer worktree dir contains one or MORE per-package
# repos under <layer>_ws/src/<repo>. Picking the *first* .git would be
# non-deterministic for a multi-repo layer worktree (`--packages a,b` spanning
# different repos) and could merge/delete a branch in the WRONG repo — so resolve
# only when there's exactly ONE inner repo; on more than one, error and point at
# the `--pr <N> --repo-slug <repo-dir>` escape hatch, which targets a repo by its
# dir name in the main tree (its slug there is the actual src/<dir> name, not the
# worktree-naming slug --issue/--repo-slug keys on). `|| true` keeps find's
# non-zero (unreadable subdir) from aborting under `set -eo pipefail`; the explicit
# count is the deterministic gate.
repo_path_in_worktree() {
    local wt="$1"
    if [ -e "$wt/.git" ]; then echo "$wt"; return 0; fi
    local gits=()
    while IFS= read -r g; do gits+=("$g"); done \
        < <(find "$wt" -maxdepth 4 -name .git 2>/dev/null || true)
    if [ "${#gits[@]}" -eq 0 ]; then
        echo "ERROR: no git repo found inside worktree $wt." >&2
        return 1
    fi
    if [ "${#gits[@]}" -gt 1 ]; then
        echo "ERROR: worktree $wt holds multiple package repos; merge-pr can't pick one safely." >&2
        echo "  Use the escape hatch: merge_pr.sh --pr <N> --repo-slug <repo-dir> (repos found:)" >&2
        for g in "${gits[@]}"; do echo "    $(basename "$(dirname "$g")")" >&2; done
        return 1
    fi
    dirname "${gits[0]}"
}

# slug for worktree_remove: "" if under .workspace-worktrees, else the repo dir name.
slug_for_repo_path() {
    case "$1" in
        "$ROOT_DIR"/.workspace-worktrees/*) echo "" ;;
        *) basename "$1" ;;
    esac
}

if [[ -n "$ARG_PR" ]]; then
    # --- escape hatch: explicit PR (+ repo-slug to locate the local repo) ---
    # No silent fallback to the workspace on an unknown slug — a typo must NOT
    # quietly target the workspace repo for a merge/branch-delete. Use the
    # literal slug "workspace" for the workspace repo.
    if [[ -z "$ARG_REPO_SLUG" || "$ARG_REPO_SLUG" == "workspace" ]]; then
        REPO_PATH="$ROOT_DIR"
        REPO_SLUG=""
    else
        REPO_PATH=$(find "$ROOT_DIR/layers/main" -maxdepth 3 -type d -name "$ARG_REPO_SLUG" -print -quit 2>/dev/null || true)
        if [[ -z "$REPO_PATH" ]]; then
            echo "ERROR: --repo-slug '$ARG_REPO_SLUG' not found under layers/main." >&2
            echo "  Check the spelling, or pass --repo-slug workspace for the workspace repo." >&2
            exit 1
        fi
        REPO_SLUG="$ARG_REPO_SLUG"
    fi
elif [[ -n "$ARG_ISSUE" ]]; then
    # --- by issue number: find the worktree, derive the repo ---
    ISSUE_NUM="$ARG_ISSUE"
    local_wt=""
    if [[ -n "$ARG_REPO_SLUG" ]]; then
        for cand in "$ROOT_DIR/.workspace-worktrees/issue-${ARG_REPO_SLUG}-${ISSUE_NUM}" \
                    "$ROOT_DIR/layers/worktrees/issue-${ARG_REPO_SLUG}-${ISSUE_NUM}"; do
            [ -d "$cand" ] && { local_wt="$cand"; break; }
        done
    else
        # Scan BOTH the workspace and layer worktree dirs; resolve when there's
        # exactly one match, error on ambiguity (mirrors worktree_remove). So
        # `--issue <N>` works for a layer issue too when it's unambiguous.
        matches=()
        [ -d "$ROOT_DIR/.workspace-worktrees/issue-workspace-${ISSUE_NUM}" ] && \
            matches+=("$ROOT_DIR/.workspace-worktrees/issue-workspace-${ISSUE_NUM}")
        for d in "$ROOT_DIR/layers/worktrees/issue-"*"-${ISSUE_NUM}"; do
            [ -d "$d" ] && matches+=("$d")
        done
        if [[ "${#matches[@]}" -gt 1 ]]; then
            echo "ERROR: issue #${ISSUE_NUM} matches multiple worktrees — pass --repo-slug:" >&2
            for m in "${matches[@]}"; do echo "    $(basename "$m" | sed -E 's/^issue-(.+)-[0-9]+$/\1/')" >&2; done
            exit 1
        fi
        [[ "${#matches[@]}" -eq 1 ]] && local_wt="${matches[0]}"
    fi
    if [[ -z "$local_wt" ]]; then
        echo "ERROR: no worktree found for issue #${ISSUE_NUM}${ARG_REPO_SLUG:+ (repo-slug ${ARG_REPO_SLUG})}." >&2
        echo "  Run from inside the worktree, or pass --repo-slug, or use --pr <N> --repo-slug <slug>." >&2
        exit 1
    fi
    REPO_PATH=$(repo_path_in_worktree "$local_wt")
    REPO_SLUG=$(slug_for_repo_path "$REPO_PATH")
    HAVE_WORKTREE=true
else
    # --- cwd mode (dominant): the worktree you're standing in is the answer ---
    REPO_PATH=$(git -C "$PWD" rev-parse --show-toplevel 2>/dev/null || true)
    if [[ -z "$REPO_PATH" || "$REPO_PATH" == "$ROOT_DIR" ]]; then
        echo "ERROR: not inside a feature worktree (cwd resolves to the main tree or no repo)." >&2
        echo "  For a layer worktree, cd into the package dir (<layer>_ws/src/<repo>) — the" >&2
        echo "  worktree root itself resolves to the main tree. Or pass --issue <N> /" >&2
        echo "  --pr <N> --repo-slug <slug>." >&2
        exit 1
    fi
    REPO_SLUG=$(slug_for_repo_path "$REPO_PATH")
    HAVE_WORKTREE=true
fi

# BRANCH_REPO = the owning repo's MAIN checkout (NOT the worktree, which gets
# removed before branch deletion). Workspace → ROOT_DIR; layer → the project
# repo under layers/main. Used for field-mode, origin lookup, and branch delete.
if [[ -z "$REPO_SLUG" ]]; then
    BRANCH_REPO="$ROOT_DIR"
else
    BRANCH_REPO=$(find "$ROOT_DIR/layers/main" -maxdepth 3 -type d -name "$REPO_SLUG" -print -quit 2>/dev/null || true)
    [[ -z "$BRANCH_REPO" ]] && BRANCH_REPO="$REPO_PATH"
fi

# Branch + issue (skip if PR mode will resolve them via gh).
if [[ -z "$PR_NUM" ]]; then
    BRANCH=$(git -C "$REPO_PATH" branch --show-current 2>/dev/null || true)
    [[ -z "$ISSUE_NUM" ]] && ISSUE_NUM=$(issue_from_branch "$BRANCH")
    if [[ -z "$ISSUE_NUM" ]]; then
        echo "ERROR: branch '${BRANCH:-<none>}' is not 'feature/issue-<N>'; can't derive the issue." >&2
        echo "  Skill worktrees and non-issue branches aren't supported — use --pr <N> --repo-slug <slug>." >&2
        exit 1
    fi
fi

# ---- field-mode guard (BEFORE any gh call) ----------------------------------
if is_field_mode "$BRANCH_REPO"; then
    echo "ERROR: $BRANCH_REPO is a field-mode repo (non-GitHub origin) — it has no GitHub PR." >&2
    echo "  Field repos push without PRs; use the field workflow, not merge-pr." >&2
    exit 1
fi

GH_REPO=$(extract_gh_slug "$(git -C "$BRANCH_REPO" remote get-url origin 2>/dev/null || echo "")")
if [[ -z "$GH_REPO" ]]; then
    echo "ERROR: could not resolve a GitHub owner/repo for $BRANCH_REPO." >&2
    exit 1
fi

# ---- resolve the PR (derive number/branch when keyed by worktree) -----------
if [[ -z "$PR_NUM" ]]; then
    PR_JSON=$(gh pr view "$BRANCH" -R "$GH_REPO" --json number,state,headRefName 2>/dev/null || true)
    [[ -z "$PR_JSON" ]] && { echo "ERROR: no PR found for branch '$BRANCH' in $GH_REPO." >&2; exit 1; }
    PR_NUM=$(echo "$PR_JSON" | jq -r '.number')
    PR_STATE=$(echo "$PR_JSON" | jq -r '.state')
else
    PR_JSON=$(gh pr view "$PR_NUM" -R "$GH_REPO" --json state,headRefName 2>/dev/null || true)
    [[ -z "$PR_JSON" ]] && { echo "ERROR: PR #$PR_NUM not found in $GH_REPO." >&2; exit 1; }
    PR_STATE=$(echo "$PR_JSON" | jq -r '.state')
    BRANCH=$(echo "$PR_JSON" | jq -r '.headRefName')
    [[ -z "$ISSUE_NUM" ]] && ISSUE_NUM=$(issue_from_branch "$BRANCH")
fi
if [[ "$PR_STATE" != "OPEN" ]]; then
    echo "ERROR: PR #$PR_NUM in $GH_REPO is $PR_STATE, not OPEN." >&2; exit 1
fi

echo "========================================"
echo "Merging PR #${PR_NUM}  ($GH_REPO, branch $BRANCH, issue #${ISSUE_NUM:-?})"
echo "========================================"

# ---- wait for CI ------------------------------------------------------------
if [[ "$NO_WAIT" == false ]]; then
    echo "  Waiting for CI..."
    if ! gh pr checks "$PR_NUM" -R "$GH_REPO" --watch --fail-fast; then
        url=$(gh pr view "$PR_NUM" -R "$GH_REPO" --json url --jq '.url' 2>/dev/null || echo "")
        echo "ERROR: CI checks failed${url:+ ($url)}. Fix and re-run, or pass --no-wait." >&2
        exit 1
    fi
fi

# ---- merge ------------------------------------------------------------------
echo "  Merging (--merge)..."
# No --yes: `gh pr merge` is already non-interactive once a merge-method flag
# (--merge) is given — it only prompts when the method is unspecified. (`gh pr
# merge` has no --yes flag anyway; passing it errors as an unknown flag.) Safe
# for the headless --pr escape hatch; the banner above is the human gate.
GIT_EDITOR=true gh pr merge "$PR_NUM" -R "$GH_REPO" --merge || {
    echo "ERROR: merge failed for PR #$PR_NUM ($GH_REPO)." >&2; exit 1; }
echo "  ✅ merged"

# ---- remove worktree (from ROOT, never from inside the worktree) ------------
# NO --force: worktree_remove must keep its uncommitted-changes guard, so we
# don't destroy unpushed work. It's non-interactive (refuses-on-dirty). The PR
# is already merged at this point, so if removal fails we ABORT before
# branch-deletion/sync rather than leave a forced, partial-cleanup state — and
# print the exact commands to finish cleanup once the worktree is resolved.
cd "$ROOT_DIR"
# Gate on whether resolution actually found a worktree (HAVE_WORKTREE) rather
# than reconstructing the worktree dir name — worktree_create/worktree_remove
# sanitize the slug (e.g. `my-pkg` → `issue-my_pkg-N`), so a reconstructed path
# could miss a real worktree and skip removal while branches still get deleted.
# worktree_remove does its own (sanitizing) resolution from --issue/--repo-slug.
if [[ "$HAVE_WORKTREE" == true && -n "$ISSUE_NUM" ]]; then
    # Always pass --repo-slug. Empty REPO_SLUG means the workspace repo, so use
    # the literal "workspace" slug — without it, worktree_remove searches
    # layers/worktrees FIRST and could remove a colliding LAYER worktree at the
    # same issue number instead of the workspace one (Copilot R4 on PR #494).
    remove_slug="${REPO_SLUG:-workspace}"
    echo "  Removing worktree..."
    if ! "$SCRIPT_DIR/worktree_remove.sh" --issue "$ISSUE_NUM" --repo-slug "$remove_slug"; then
        {
            echo "ERROR: worktree removal failed (uncommitted changes in the worktree?)."
            echo "  PR #$PR_NUM is already MERGED. Resolve the worktree, then finish cleanup:"
            echo "    $SCRIPT_DIR/worktree_remove.sh --issue $ISSUE_NUM --repo-slug $remove_slug"
            echo "    git -C $BRANCH_REPO branch -D $BRANCH && git -C $BRANCH_REPO push origin --delete $BRANCH"
            echo "    make -C $ROOT_DIR sync"
        } >&2
        exit 1
    fi
    echo "  ✅ worktree removed"
else
    echo "  (no worktree resolved for issue #${ISSUE_NUM:-?} — skipping removal;"
    echo "   if one exists, remove it separately with worktree_remove.sh --issue $ISSUE_NUM)"
fi

# ---- delete local + remote branch on the owning repo ------------------------
echo "  Deleting branches..."
git -C "$BRANCH_REPO" branch -D "$BRANCH" 2>/dev/null && echo "  ✅ local branch deleted" || true
git -C "$BRANCH_REPO" push origin --delete "$BRANCH" 2>/dev/null && echo "  ✅ remote branch deleted" || true

# ---- sync -------------------------------------------------------------------
echo "  Syncing all repos..."
make -C "$ROOT_DIR" sync

echo ""
echo "========================================"
echo "✅ Done: PR #${PR_NUM} merged, cleaned up, and synced."
echo "========================================"
