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

issue_from_branch() {  # echo the issue number from a feature/issue-<N> branch
    echo "$1" | sed -nE 's#^feature/[iI][sS][sS][uU][eE]-([0-9]+).*#\1#p'
}

# Given a worktree dir, echo the project-repo git toplevel inside it (workspace
# worktrees ARE git repos; layer worktree dirs contain per-package worktrees).
repo_path_in_worktree() {
    local wt="$1"
    if [ -e "$wt/.git" ]; then echo "$wt"; return 0; fi
    local inner
    # `-print -quit` returns the first match and exits 0 (no SIGPIPE from a
    # closed `head` pipe); `|| true` swallows find's non-zero on an unreadable
    # subdir so `set -eo pipefail` can't abort here.
    inner=$(find "$wt" -maxdepth 4 -name .git -print -quit 2>/dev/null || true)
    [ -n "$inner" ] && dirname "$inner"
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
if [[ -n "$REPO_SLUG" ]]; then
    WT_DIR="$ROOT_DIR/layers/worktrees/issue-${REPO_SLUG}-${ISSUE_NUM}"
else
    WT_DIR="$ROOT_DIR/.workspace-worktrees/issue-workspace-${ISSUE_NUM}"
fi
if [[ -n "$ISSUE_NUM" && -d "$WT_DIR" ]]; then
    echo "  Removing worktree..."
    if ! "$SCRIPT_DIR/worktree_remove.sh" --issue "$ISSUE_NUM" ${REPO_SLUG:+--repo-slug "$REPO_SLUG"}; then
        {
            echo "ERROR: worktree removal failed (uncommitted changes in the worktree?)."
            echo "  PR #$PR_NUM is already MERGED. Resolve the worktree, then finish cleanup:"
            echo "    $SCRIPT_DIR/worktree_remove.sh --issue $ISSUE_NUM ${REPO_SLUG:+--repo-slug $REPO_SLUG}"
            echo "    git -C $BRANCH_REPO branch -D $BRANCH && git -C $BRANCH_REPO push origin --delete $BRANCH"
            echo "    make -C $ROOT_DIR sync"
        } >&2
        exit 1
    fi
    echo "  ✅ worktree removed"
else
    echo "  (no worktree for issue #${ISSUE_NUM:-?} — skipping removal)"
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
