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
#   merge_pr.sh --pr <N> --repo-slug <slug>   # escape hatch: headless / no worktree.
#                                             # --repo-slug REQUIRED here (PR #s are
#                                             # per-repo); use "workspace" for the
#                                             # workspace repo.
# Options:
#   --no-wait    skip the pre-merge verification entirely (use when CI is known green)
#
# Env (verification tuning; see the "verify before merging" block):
#   MERGE_PR_SETTLE_ATTEMPTS  re-polls when a repo has workflows but no checks
#                             have registered yet (default 3)
#   MERGE_PR_SETTLE_SECONDS   delay between those polls (default 10)
#
# Exit codes:
#   0   merged and verified (hosted checks green, or an ADR-0018 full-scope
#       ci-local attestation on the head), cleaned up, synced
#   1   refused, or a later step failed (nothing merged unless stated)
#   2   usage error
#   42  MERGED WITHOUT VERIFICATION: the repo has no CI at the head and no
#       ci-local attestation. The merge and cleanup DID happen; the distinct
#       code exists so this outcome is greppable rather than one stderr line
#       among many (issue #610). `make merge-pr` surfaces it as "Error 42".
#
# Field-mode repos (non-GitHub origin) have no GitHub PR; this script refuses
# them before any `gh` call (ADR-0011) — they use the field workflow.
#
# Steps: resolve → field-mode guard → verify (hosted checks, or an ADR-0018
#        full-scope ci-local attestation on the exact head — which satisfies the
#        gate for a PROJECT repo whether or not the repo has workflows, but
#        never over a RED hosted signal; issue #610) →
#        merge (--merge) → remove worktree → delete branches → make sync.

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
# shellcheck source=.agent/scripts/_ci_verification_helpers.sh
source "$SCRIPT_DIR/_ci_verification_helpers.sh"   # ci_local_attestation_status

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
           echo "Usage: $0 [--issue <N> [--repo-slug <slug>] | --pr <N> --repo-slug <slug>] [--no-wait]" >&2
           exit 2 ;;
    esac
done

# --issue and --pr are alternative resolution modes; the dispatch below prefers
# --pr if both are set, so accepting both would silently act on the --pr target
# (a typo or a stale `make merge-pr ISSUE=… PR=…` carryover could merge/clean up
# the WRONG PR). For a destructive tool, reject the conflict outright.
if [[ -n "$ARG_ISSUE" && -n "$ARG_PR" ]]; then
    echo "ERROR: --issue and --pr are mutually exclusive (pick one resolution mode)." >&2
    echo "  Usage: $0 [--issue <N> [--repo-slug <slug>] | --pr <N> --repo-slug <slug>] [--no-wait]" >&2
    exit 2
fi

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
RESOLVED_WT=""       # the resolved worktree directory (cwd/--issue modes), used
                     # only by the legacy-naming guard below.

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
    # --- escape hatch: explicit PR (+ REQUIRED repo-slug to locate the repo) ---
    # PR numbers are per-repo, so a bare --pr <N> is ambiguous across the workspace
    # repo and the many project repos. Require --repo-slug explicitly rather than
    # silently assuming the workspace — a forgotten slug must NOT merge/delete the
    # WRONG repo's PR #N. Use the literal "workspace" for the workspace repo; a
    # project repo's slug is its src/<dir> name under layers/main. (An unknown slug
    # is a hard error below — never a silent workspace fallback.)
    if [[ -z "$ARG_REPO_SLUG" ]]; then
        echo "ERROR: --pr mode requires --repo-slug (PR numbers are per-repo)." >&2
        echo "  Use --repo-slug workspace for the workspace repo, or" >&2
        echo "  --repo-slug <repo-dir> for a project repo under layers/main." >&2
        exit 2
    elif [[ "$ARG_REPO_SLUG" == "workspace" ]]; then
        REPO_PATH="$ROOT_DIR"
        REPO_SLUG=""
    else
        # Anchor on '*_ws/src/*' so a `colcon build` artifact at
        # <layer>_ws/install/<pkg> or <layer>_ws/build/<pkg> (same depth+name as
        # the source) can't be picked before the real source repo (issue #514).
        # Anchored on the layer-workspace structure (not a bare '*/src/*'): a
        # workspace checked out under a path containing a /src/ segment would
        # still match install/build dirs with the bare form.
        REPO_PATH=$(find "$ROOT_DIR/layers/main" -maxdepth 3 -type d -path '*_ws/src/*' -name "$ARG_REPO_SLUG" -print -quit 2>/dev/null || true)
        if [[ -z "$REPO_PATH" ]]; then
            echo "ERROR: --repo-slug '$ARG_REPO_SLUG' not found under layers/main." >&2
            echo "  Check the spelling, or pass --repo-slug workspace for the workspace repo." >&2
            exit 1
        fi
        # Belt-and-suspenders: the candidate must be a git repo root, not a stray
        # dir that slipped past the path filter. Only error on found-but-not-a-
        # git-root — an empty find already exited above.
        if [[ "$(git -C "$REPO_PATH" rev-parse --show-toplevel 2>/dev/null)" != "$REPO_PATH" ]]; then
            echo "ERROR: --repo-slug '$ARG_REPO_SLUG' resolved to '$REPO_PATH', which is not a git repo root." >&2
            echo "  Expected a project repo under layers/main/<layer>_ws/src/." >&2
            exit 1
        fi
        REPO_SLUG="$ARG_REPO_SLUG"
    fi
elif [[ -n "$ARG_ISSUE" ]]; then
    # --- by issue number: find the worktree, derive the repo ---
    ISSUE_NUM="$ARG_ISSUE"
    local_wt=""
    if [[ -n "$ARG_REPO_SLUG" ]]; then
        # worktree_create/worktree_remove name the dir with the *sanitized* slug
        # (non-[A-Za-z0-9_] → _), so sanitize the same way before lookup — else a
        # hyphenated --repo-slug (my-pkg, dir issue-my_pkg-N) is never found. Only
        # this --issue worktree-dir lookup needs it; the --pr branch above matches
        # an actual src/<dir> name under layers/main and must stay raw.
        wt_slug=$(echo "$ARG_REPO_SLUG" | sed 's/[^A-Za-z0-9_]/_/g')
        for cand in "$ROOT_DIR/.workspace-worktrees/issue-${wt_slug}-${ISSUE_NUM}" \
                    "$ROOT_DIR/layers/worktrees/issue-${wt_slug}-${ISSUE_NUM}"; do
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
    RESOLVED_WT="$local_wt"
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
    RESOLVED_WT="$REPO_PATH"   # workspace worktree → REPO_PATH is the worktree dir
fi

# BRANCH_REPO = the owning repo's MAIN checkout (NOT the worktree, which gets
# removed before branch deletion). Workspace → ROOT_DIR; layer → the project
# repo under layers/main. Used for field-mode, origin lookup, and branch delete.
if [[ -z "$REPO_SLUG" ]]; then
    BRANCH_REPO="$ROOT_DIR"
else
    # Anchor on '*_ws/src/*' for the same reason as the --pr derivation above:
    # exclude colcon install/build artifacts that share the source's depth+name
    # (issue #514).
    BRANCH_REPO=$(find "$ROOT_DIR/layers/main" -maxdepth 3 -type d -path '*_ws/src/*' -name "$REPO_SLUG" -print -quit 2>/dev/null || true)
    # An EMPTY result is a legitimate signal — fall through to the REPO_PATH
    # fallback below. Only error when a non-empty candidate isn't a git repo root.
    if [[ -n "$BRANCH_REPO" && "$(git -C "$BRANCH_REPO" rev-parse --show-toplevel 2>/dev/null)" != "$BRANCH_REPO" ]]; then
        echo "ERROR: repo-slug '$REPO_SLUG' resolved to '$BRANCH_REPO', which is not a git repo root." >&2
        echo "  Expected a project repo under layers/main/<layer>_ws/src/." >&2
        exit 1
    fi
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

# ---- legacy-worktree guard (BEFORE the irreversible merge) ------------------
# Bare `.workspace-worktrees/issue-<N>` dirs predate the `issue-workspace-<N>`
# convention. worktree_remove can't target them when given `--repo-slug workspace`
# (exact-path match only), and dropping the slug would risk removing a colliding
# LAYER worktree at the same issue number (the R4 bug we fixed). merge-pr supports
# only the current naming — fail here, BEFORE merging, so we never leave a
# merged-but-uncleaned worktree. (Only cwd-mode can reach a legacy dir; --issue
# resolution looks up `issue-workspace-<N>` and never finds the bare form.)
if [[ "$HAVE_WORKTREE" == true && -z "$REPO_SLUG" && -n "$RESOLVED_WT" \
      && "$(basename "$RESOLVED_WT")" == "issue-${ISSUE_NUM}" ]]; then
    echo "ERROR: legacy worktree dir 'issue-${ISSUE_NUM}' (no repo slug) isn't supported by merge-pr cleanup." >&2
    echo "  Merge the PR, then remove the worktree manually:" >&2
    echo "    $SCRIPT_DIR/worktree_remove.sh --issue ${ISSUE_NUM}" >&2
    exit 1
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

# ---- verify before merging --------------------------------------------------
# ADR-0018 decision 1 in full (#610): a project-repo PR whose head carries a
# full-scope `refs/notes/ci-local` attestation satisfies the merge gate WITHOUT
# waiting for hosted Actions — whether or not the repo also has workflows.
#
# PRECEDENCE (the attestation never overrides a red hosted signal; AGENTS.md
# § Merging: "never merge past a red signal from whichever verification
# applies"):
#
#   hosted checks FAILING              → REFUSE, attestation or not
#   hosted checks PENDING + attestation → merge on the attestation, no wait
#   hosted checks PENDING, no attestation → wait on them (unchanged)
#   hosted checks PASSING              → merge (unchanged)
#   hosted checks in an UNRECOGNIZED state → wait on them (fail closed)
#   no checks registered, workflows exist at the head → settle, re-poll, REFUSE
#   no checks, no workflows, attestation → merge on the attestation
#   no checks, no workflows, no attestation → merge with a loud warning and
#                                             exit $MERGE_UNVERIFIED_EXIT
#
# The workspace repo never takes any attestation path: its hosted checks are
# required (ADR-0018 decision 4).
ATTESTED=false                  # set when a note authorized the merge; gates the
                                # refs/notes/ci-local push below (decision 5)
MERGED_UNVERIFIED=false         # set in the no-CI/no-attestation state
# Distinct exit status for "merged, but nothing verified this commit". The merge
# and all cleanup still happen; the code makes that outcome greppable in a
# transcript instead of one stderr line among many. Documented in the usage
# header above, AGENTS.md § Merge verification, and `make help`.
MERGE_UNVERIFIED_EXIT=42

# Settle window for the workflows-but-no-checks state, overridable so tests
# don't sleep. Both reach arithmetic contexts, so validate them as integers —
# an unvalidated value in `$(( ))`/`[[ -gt ]]` is a command-execution surface.
SETTLE_ATTEMPTS="${MERGE_PR_SETTLE_ATTEMPTS:-3}"
SETTLE_SECONDS="${MERGE_PR_SETTLE_SECONDS:-10}"
for v in SETTLE_ATTEMPTS SETTLE_SECONDS; do
    if [[ ! "${!v}" =~ ^[0-9]+$ ]]; then
        echo "ERROR: MERGE_PR_${v#SETTLE_} must be a non-negative integer (got '${!v}')." >&2
        exit 2
    fi
done

# Echo "<count> <state>" for the PR's status checks, or exit 1 on failure.
#   state ∈ failing | pending | passing | unknown   (count 0 → "0 passing")
#
# A FAILED `gh` CALL IS NOT AN EMPTY ARRAY: auth expiry, network loss, and rate
# limiting must never be classified as "this repo has no CI" — that would merge
# on a warning when we simply could not see the checks.
check_rollup_state() {
    local json rc out
    json=$(gh pr view "$PR_NUM" -R "$GH_REPO" --json statusCheckRollup 2>/dev/null); rc=$?
    if [[ $rc -ne 0 || -z "$json" ]]; then
        echo "ERROR: could not read check status for PR #$PR_NUM ($GH_REPO) — gh failed (auth/network/rate limit?). Not merging." >&2
        exit 1
    fi
    # Classify each entry, then aggregate. A CheckRun carries status/conclusion;
    # a StatusContext carries state. Anything we do not recognize is "unknown"
    # and is waited on, never shortcut past. The `type != "array"` guard fails
    # closed on a gh schema change: `length` alone yields 0 for a MISSING key
    # exactly as for an empty array, which would look like "no CI".
    out=$(jq -e -r '
        def verdict:
          (.status? // null) as $st
          | (.conclusion? // null) as $cc
          | (.state? // null) as $s
          | if $st != null and ($st | ascii_upcase) != "COMPLETED" then "pending"
            elif $cc != null then
              (if (["SUCCESS","NEUTRAL","SKIPPED"] | index($cc | ascii_upcase)) then "pass" else "fail" end)
            elif $s != null then
              (($s | ascii_upcase) as $u
               | if $u == "SUCCESS" or $u == "EXPECTED" then "pass"
                 elif $u == "PENDING" then "pending"
                 else "fail" end)
            else "unknown" end;
        if (.statusCheckRollup | type) != "array" then error("not an array") else
          (.statusCheckRollup | map(verdict)) as $v
          | "\($v | length) " +
            (if   ($v | map(select(. == "fail"))    | length) > 0 then "failing"
             elif ($v | map(select(. == "unknown")) | length) > 0 then "unknown"
             elif ($v | map(select(. == "pending")) | length) > 0 then "pending"
             else "passing" end)
        end' <<<"$json" 2>/dev/null) || {
        echo "ERROR: unexpected gh output while reading check status for PR #$PR_NUM ($GH_REPO). Not merging." >&2
        exit 1
    }
    echo "$out"
}

pr_url() { gh pr view "$PR_NUM" -R "$GH_REPO" --json url --jq '.url' 2>/dev/null || echo ""; }

# The checkout that is actually sitting on the PR head — what ci_local.sh must
# be pointed at. BRANCH_REPO is the MAIN checkout (on the default branch), so
# naming it in the recourse would attest the wrong commit.
attest_hint() {
    local candidate="$REPO_PATH" at=""
    at=$(git -C "$candidate" rev-parse HEAD 2>/dev/null || echo "")
    if [[ "$at" == "$HEAD_SHA" ]]; then
        echo "$SCRIPT_DIR/ci_local.sh $candidate"
    else
        echo "$SCRIPT_DIR/ci_local.sh <a checkout of $BRANCH_REPO sitting on $HEAD_SHA>"
    fi
}

if [[ "$NO_WAIT" == false ]]; then
    echo "  Checking CI status..."
    # `$( )` runs in a SUBSHELL, so check_rollup_state's `exit 1` on a gh
    # failure would only kill that subshell — the status must be propagated
    # explicitly here, or an unreadable check status would fall through as an
    # empty rollup (the exact fail-open this issue exists to remove).
    ROLLUP_LINE=$(check_rollup_state) || exit 1
    read -r ROLLUP_COUNT ROLLUP_STATE <<<"$ROLLUP_LINE"
    if [[ ! "$ROLLUP_COUNT" =~ ^[0-9]+$ || -z "$ROLLUP_STATE" ]]; then
        echo "ERROR: could not classify the check status for PR #$PR_NUM ($GH_REPO). Not merging." >&2
        exit 1
    fi

    # The head commit: the attestation is keyed on it, and the workflows probe
    # asks about THIS commit rather than the default branch. Validate it as a
    # full sha before it is used as both a URL ref and a git revision.
    HEAD_SHA=$(gh pr view "$PR_NUM" -R "$GH_REPO" --json headRefOid --jq '.headRefOid' 2>/dev/null || echo "")
    if [[ ! "$HEAD_SHA" =~ ^[0-9a-f]{40}$ ]]; then
        echo "ERROR: could not resolve a valid head commit for PR #$PR_NUM ($GH_REPO) — got '${HEAD_SHA:-<empty>}'. Not merging." >&2
        exit 1
    fi

    # ADR-0018 decision 4: the workspace repo's hosted checks are required and
    # no attestation substitutes. Identify it BOTH by path and by GitHub slug —
    # a workspace worktree created outside `.workspace-worktrees/` would defeat
    # a path compare alone, and the exemption must not be losable that way.
    IS_WORKSPACE_REPO=false
    if [[ "$BRANCH_REPO" == "$ROOT_DIR" ]]; then
        IS_WORKSPACE_REPO=true
    else
        WORKSPACE_SLUG=$(extract_gh_slug "$(git -C "$ROOT_DIR" remote get-url origin 2>/dev/null || echo "")")
        [[ -n "$WORKSPACE_SLUG" && "$GH_REPO" == "$WORKSPACE_SLUG" ]] && IS_WORKSPACE_REPO=true
    fi

    # Is a full-scope ci-local attestation on THIS head available as evidence?
    # Consulted for project repos in every state that reaches it (ADR-0018
    # decision 1) — never for the workspace repo, and never to override a red
    # hosted signal (see gate_on_hosted_checks).
    attestation_available() {
        [[ "$IS_WORKSPACE_REPO" == true ]] && return 1
        ATTEST_VERDICT=$(ci_local_attestation_status "$BRANCH_REPO" "$HEAD_SHA") && return 0
        return 1
    }

    # Apply the precedence above to a non-empty rollup. Returns normally when
    # the gate is satisfied; exits 1 when it is not.
    gate_on_hosted_checks() {
        local state="$1" url
        if [[ "$state" == "failing" ]]; then
            url=$(pr_url)
            {
                echo "ERROR: CI checks failed${url:+ ($url)}."
                echo "       A ci-local attestation does NOT override a red hosted signal (ADR-0018:"
                echo "       never merge past a red signal from whichever verification applies)."
                echo "       Fix the failures and re-run; --no-wait skips verification entirely and is not the answer."
            } >&2
            exit 1
        fi
        if [[ "$state" == "pending" ]] && attestation_available; then
            echo "  ✅ $ATTEST_VERDICT (ADR-0018 decision 1) — hosted checks are still pending;"
            echo "     merging on the attestation without waiting for them."
            ATTESTED=true
            return 0
        fi
        # passing → returns at once; pending/unknown without an attestation →
        # the wait we have always done.
        echo "  Waiting for CI..."
        if ! gh pr checks "$PR_NUM" -R "$GH_REPO" --watch --fail-fast; then
            url=$(pr_url)
            echo "ERROR: CI checks failed${url:+ ($url)}. Fix and re-run, or pass --no-wait." >&2
            exit 1
        fi
    }

    if [[ "$ROLLUP_COUNT" -gt 0 ]]; then
        gate_on_hosted_checks "$ROLLUP_STATE"
    else
        # An empty rollup is AMBIGUOUS. A repo that does have CI presents `[]`
        # when the head was just pushed and Actions has not registered runs
        # yet, when every workflow is paths-filtered and none matched, or when
        # a suite is queued with no runs. Reading `[]` as "no CI configured"
        # would turn today's fail-CLOSED misreport into a fail-OPEN merge.
        # Resolve it with a positive probe instead of an inference.
        #
        # Does .github/workflows exist AT THIS COMMIT? Do NOT use
        # `gh api repos/<repo>/actions/workflows`: it reports total_count: 2
        # for rolker/mru_transform, a repo with no workflow files at all,
        # because dynamically-registered Copilot reviewer workflows appear
        # there. The ?ref= form answers the right question — whether THIS
        # commit carries CI, not the default branch.
        # `VAR=$(cmd)` under `set -e` would abort the script on the 404 that is
        # this probe's most informative answer — capture the status explicitly.
        probe_rc=0
        WORKFLOW_PROBE=$(gh api "repos/$GH_REPO/contents/.github/workflows?ref=$HEAD_SHA" 2>&1) || probe_rc=$?
        if [[ $probe_rc -eq 0 ]]; then
            HAS_WORKFLOWS=true
        elif grep -qE '(HTTP 404|Not Found)' <<<"$WORKFLOW_PROBE"; then
            # A 404 alone is NOT proof that the path is absent. GitHub returns a
            # byte-identical 404 for a repo/path the token cannot read
            # (Contents scope missing) and for a ref it cannot resolve
            # ("No commit found for the ref" is also a 404 body). A token with
            # PR read but no Contents read would classify EVERY project repo as
            # no-CI and merge it on a warning. So confirm positively: list the
            # repository root at the same ref. If that succeeds, Contents read
            # works and the ref resolves, so the workflows 404 really does mean
            # "this commit has no .github/workflows".
            control_rc=0
            CONTROL_PROBE=$(gh api "repos/$GH_REPO/contents?ref=$HEAD_SHA" 2>&1) || control_rc=$?
            if [[ $control_rc -eq 0 ]]; then
                HAS_WORKFLOWS=false
            else
                {
                    echo "ERROR: cannot tell whether $GH_REPO has CI at $HEAD_SHA."
                    echo "       .github/workflows returned 404, but so did the repository root at the"
                    echo "       same ref — so the 404 may mean 'no permission to read contents' or"
                    echo "       'unknown ref', not 'no workflows'. Not merging on an unreadable probe."
                    echo "       Check the token's Contents:read scope and that $HEAD_SHA is pushed."
                    echo "       gh said: $(head -1 <<<"$CONTROL_PROBE")"
                } >&2
                exit 1
            fi
        else
            # A probe we could not run is not a probe that said "no CI".
            echo "ERROR: could not probe .github/workflows for $GH_REPO at $HEAD_SHA (auth/network/rate limit?). Not merging." >&2
            echo "       gh said: $(head -1 <<<"$WORKFLOW_PROBE")" >&2
            exit 1
        fi

        if [[ "$HAS_WORKFLOWS" == true || "$IS_WORKSPACE_REPO" == true ]]; then
            # ---- CI exists, checks not registered (yet) ----------------------
            # Short settle/re-poll: a registration race resolves in seconds.
            polls=1                       # the read above was already a poll
            for ((i = 0; i < SETTLE_ATTEMPTS; i++)); do
                [[ "$SETTLE_SECONDS" -gt 0 ]] && sleep "$SETTLE_SECONDS"
                ROLLUP_LINE=$(check_rollup_state) || exit 1
                read -r ROLLUP_COUNT ROLLUP_STATE <<<"$ROLLUP_LINE"
                [[ "$ROLLUP_COUNT" =~ ^[0-9]+$ ]] || { echo "ERROR: could not classify the check status for PR #$PR_NUM ($GH_REPO). Not merging." >&2; exit 1; }
                polls=$((polls + 1))
                [[ "$ROLLUP_COUNT" -gt 0 ]] && break
            done
            if [[ "$ROLLUP_COUNT" -gt 0 ]]; then
                # Checks appeared during the settle window: gate on them exactly
                # as if they had been there from the start.
                gate_on_hosted_checks "$ROLLUP_STATE"
            else
                url=$(pr_url)
                if [[ "$IS_WORKSPACE_REPO" == true ]]; then
                    {
                        echo "ERROR: no checks have registered yet for PR #$PR_NUM ($GH_REPO) after $polls poll(s)."
                        echo "       The workspace repo's hosted checks are required (ADR-0018 decision 4) —"
                        echo "       a ci-local attestation does not substitute here. Wait for Actions to"
                        echo "       start, then re-run merge-pr.${url:+ ($url)}"
                    } >&2
                else
                    {
                        echo "ERROR: $GH_REPO has .github/workflows at $HEAD_SHA but no checks have"
                        echo "       registered for PR #$PR_NUM after $polls poll(s). Either Actions has not"
                        echo "       started them yet, or every workflow is filtered out for this PR's"
                        echo "       paths/branches, or the directory holds no runnable workflow."
                        echo "       Not merging: a repo with CI must be verified by it."
                        echo "       Check the PR's Checks tab${url:+ ($url)}, then re-run merge-pr."
                    } >&2
                fi
                exit 1
            fi
        else
            # ---- project repo, genuinely no CI at this head ------------------
            if attestation_available; then
                echo "  ✅ $ATTEST_VERDICT (ADR-0018) — treating as merge verification."
                ATTESTED=true
            else
                {
                    echo "⚠️  WARNING: $GH_REPO has no CI workflows at $HEAD_SHA and ${ATTEST_VERDICT:-no attestation was found}."
                    echo "    Merging with NO automated verification."
                    echo "    To merge on evidence instead, run it against a checkout of THIS head:"
                    echo "      $(attest_hint)"
                    echo "    then re-run merge-pr. An attestation for an earlier commit does not"
                    echo "    count — re-run it after every push."
                } >&2
                MERGED_UNVERIFIED=true
            fi
        fi
    fi
fi

# ---- push the attestation that authorized this merge ------------------------
# ADR-0018 decision 5, delegated to ci_local_push_attestation() (which handles
# the two states a bare `git push origin refs/notes/ci-local` fails in — see its
# comment). Pushed BEFORE the merge so a failure is visible while the merge is
# still in the operator's hands; a failure warns rather than aborts, because the
# verification itself already happened.
if [[ "$ATTESTED" == true ]]; then
    echo "  Pushing refs/notes/ci-local (ADR-0018 decision 5)..."
    if push_msg=$(ci_local_push_attestation "$BRANCH_REPO"); then
        echo "  ✅ $push_msg"
    else
        {
            echo "WARNING: $push_msg"
            echo "         The merge proceeds — the verification already happened — but the"
            echo "         attestation may not be published for anyone else. Retry with:"
            echo "           git -C $BRANCH_REPO fetch origin '+refs/notes/ci-local:refs/notes/origin-ci-local' &&"
            echo "           git -C $BRANCH_REPO -c core.notesRef=refs/notes/ci-local notes merge -s cat_sort_uniq refs/notes/origin-ci-local &&"
            echo "           git -C $BRANCH_REPO push origin refs/notes/ci-local"
        } >&2
    fi
fi

# ---- merge ------------------------------------------------------------------
echo "  Merging (--merge)..."
# No --yes: `gh pr merge` is already non-interactive once a merge-method flag
# (--merge) is given — it only prompts when the method is unspecified. (`gh pr
# merge` has no --yes flag anyway; passing it errors as an unknown flag.) Safe
# for the headless --pr escape hatch; the banner above is the human gate.
# --match-head-commit pins the merge to the commit that was actually verified:
# without it a push landing between the check above and this call would merge a
# commit nothing attested (gh then refuses with a head-mismatch error).
GIT_EDITOR=true gh pr merge "$PR_NUM" -R "$GH_REPO" --merge \
        ${HEAD_SHA:+--match-head-commit "$HEAD_SHA"} || {
    echo "ERROR: merge failed for PR #$PR_NUM ($GH_REPO)." >&2
    echo "       (If gh reports a head mismatch, the PR was pushed to after verification —" >&2
    echo "        re-run merge-pr so the new head is verified too.)" >&2
    exit 1; }
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

# ---- unverified-merge signal ------------------------------------------------
# Everything above succeeded, but nothing verified the merged commit. Say so
# once more where it cannot be lost in the scroll, and exit with a DISTINCT
# status so the outcome is greppable in a transcript / detectable by a caller.
if [[ "$MERGED_UNVERIFIED" == true ]]; then
    {
        echo ""
        echo "########################################################################"
        echo "## MERGED WITHOUT VERIFICATION — PR #${PR_NUM} ($GH_REPO)"
        echo "##   head $HEAD_SHA has no hosted CI and no ci-local attestation."
        echo "##   Nothing checked that this commit builds or that its tests pass."
        echo "##   exit status $MERGE_UNVERIFIED_EXIT (merge-pr's unverified-merge code)"
        echo "########################################################################"
    } >&2
    exit "$MERGE_UNVERIFIED_EXIT"
fi
