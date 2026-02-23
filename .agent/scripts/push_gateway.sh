#!/bin/bash
# .agent/scripts/push_gateway.sh
# Host-side interactive script to process outbox items from sandboxed containers.
#
# Processes two types of requests:
#   1. Push requests  — git push + PR creation
#   2. Issue requests — GitHub issue creation
#
# Usage:
#   push_gateway.sh                       # Process all pending requests
#   push_gateway.sh --worktree-id <id>    # Process specific worktree only
#   push_gateway.sh --issue <N>           # Process worktrees matching issue N

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"
PUSH_DIR="$ROOT_DIR/.agent/scratchpad/push-requests"
ISSUE_DIR="$ROOT_DIR/.agent/scratchpad/issue-requests"

# ---------- Argument parsing ----------

FILTER_WORKTREE_ID=""
FILTER_ISSUE=""

show_usage() {
    cat <<'EOF'
Usage: push_gateway.sh [OPTIONS]

Process pending outbox items from sandboxed agent containers.
Handles push requests (git push + PR) and issue requests (gh issue create).

Options:
  --worktree-id <id>   Process only requests from this worktree
  --issue <N>          Process worktrees matching issue number N
  -h, --help           Show this help

Scans:
  .agent/scratchpad/push-requests/    — pending push + PR requests
  .agent/scratchpad/issue-requests/   — pending issue creation requests
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --worktree-id|--issue)
            if [[ $# -lt 2 ]]; then
                echo "ERROR: $1 requires a value." >&2
                show_usage >&2
                exit 1
            fi
            ;;&
        --worktree-id)
            FILTER_WORKTREE_ID="$2"; shift 2 ;;
        --issue)
            FILTER_ISSUE="$2"; shift 2 ;;
        -h|--help)
            show_usage; exit 0 ;;
        *)
            echo "Unknown option: $1" >&2
            show_usage >&2
            exit 1 ;;
    esac
done

# ---------- Helper: confine file path to scratchpad ----------
# Prevents path traversal attacks where a container-written JSON points
# body_file at an arbitrary host file (e.g., ~/.ssh/id_rsa).
SCRATCHPAD_DIR="$ROOT_DIR/.agent/scratchpad"

validate_body_file() {
    local file_path="$1"
    [ -n "$file_path" ] || return 1
    [ -f "$file_path" ] || return 1
    local real_path
    real_path="$(realpath "$file_path" 2>/dev/null)" || return 1
    case "$real_path" in
        "$SCRATCHPAD_DIR"/*) return 0 ;;
        *)
            echo "  WARNING: body_file '$file_path' is outside scratchpad — ignoring." >&2
            return 1
            ;;
    esac
}

# ---------- Helper: validate repo_path is under workspace root ----------
validate_repo_path() {
    local repo_path="$1"
    [ -d "$repo_path" ] || return 1
    local real_path
    real_path="$(realpath "$repo_path" 2>/dev/null)" || return 1
    case "$real_path" in
        "$ROOT_DIR"/*) ;;
        *)
            echo "  ERROR: repo_path '$repo_path' is outside workspace root — skipping." >&2
            return 1
            ;;
    esac
    if ! git -C "$real_path" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
        echo "  ERROR: repo_path '$repo_path' is not a git repository — skipping." >&2
        return 1
    fi
    return 0
}

# ---------- Helper: validate branch name ----------
# Prevents option injection and dangerous refspecs in git push/gh pr create.
# Container-written branch names are untrusted input and must be validated
# before use in git commands. Shell metacharacter safety in printed fallback
# commands is handled separately via printf %q at the call sites.
validate_branch_name() {
    local branch="$1"

    # Reject empty, missing, or jq-null branch names
    if [ -z "$branch" ] || [ "$branch" = "null" ]; then
        echo "  ERROR: Branch name is empty or missing." >&2
        return 1
    fi

    # Reject dangerous git options and refspecs (checked before generic ^[-+] so
    # specific options like --mirror get a more informative error message)
    case "$branch" in
        --all|--mirror|--tags|--force|--delete|--prune|HEAD|@)
            echo "  ERROR: Branch name '$branch' is a dangerous refspec or git option." >&2
            return 1
            ;;
        refs/*)
            # Full refs are not allowed — gh pr create --head expects short names
            echo "  ERROR: Full refs are not allowed; use the short branch name (e.g., 'feature/issue-123' instead of 'refs/heads/feature/issue-123')." >&2
            return 1
            ;;
    esac

    # Reject branch names starting with '-' (option injection) or '+' (force-push refspec)
    if [[ "$branch" =~ ^[-+] ]]; then
        echo "  ERROR: Branch name '$branch' starts with '-' or '+' (potential option/refspec injection)." >&2
        return 1
    fi

    # Validate with git's own ref format checker
    if ! git check-ref-format --branch "$branch" >/dev/null 2>&1; then
        echo "  ERROR: Branch name '$branch' is not a valid git branch name." >&2
        return 1
    fi

    return 0
}

# ---------- Helper: check if a worktree ID matches the filter ----------

matches_filter() {
    local wt_id="$1"
    if [ -n "$FILTER_WORKTREE_ID" ]; then
        [ "$wt_id" = "$FILTER_WORKTREE_ID" ]
    elif [ -n "$FILTER_ISSUE" ]; then
        # Worktree IDs end with -<issue_number>
        [[ "$wt_id" =~ -${FILTER_ISSUE}$ ]]
    else
        return 0  # no filter — match all
    fi
}

# ========== PUSH REQUESTS ==========

process_push_requests() {
    [ -d "$PUSH_DIR" ] || return 0

    local pending_files=()

    for signal_file in "$PUSH_DIR"/*.json; do
        [ -f "$signal_file" ] || continue
        local status
        status=$(jq -r '.status // "pending"' "$signal_file" 2>/dev/null || echo "unknown")
        [ "$status" = "pending" ] || continue

        # Check worktree ID filter
        local wt_id
        wt_id=$(jq -r '.worktree_id // ""' "$signal_file" 2>/dev/null || echo "")
        if [ -z "$wt_id" ]; then
            # Legacy: derive from filename
            wt_id="$(basename "$signal_file" .json)"
        fi
        matches_filter "$wt_id" || continue

        pending_files+=("$signal_file")
    done

    [ ${#pending_files[@]} -gt 0 ] || return 0

    echo "========================================="
    echo "  Push Requests (${#pending_files[@]} pending)"
    echo "========================================="
    echo ""

    for signal_file in "${pending_files[@]}"; do
        local issue branch repo_slug repo_path commit_count pr_title body_file requested_at
        issue=$(jq -r '.issue' "$signal_file")
        branch=$(jq -r '.branch' "$signal_file")
        repo_slug=$(jq -r '.repo_slug' "$signal_file")
        repo_path=$(jq -r '.repo_path' "$signal_file")
        commit_count=$(jq -r '.commit_count' "$signal_file")
        pr_title=$(jq -r '.pr_title' "$signal_file")
        body_file=$(jq -r '.pr_body_file // ""' "$signal_file")
        requested_at=$(jq -r '.requested_at' "$signal_file")

        # Validate repo_path is under workspace root and is a git repo
        if ! validate_repo_path "$repo_path"; then
            echo "  Skipping push request from $signal_file"
            echo ""
            continue
        fi

        # Validate branch name (prevent option injection and dangerous refspecs)
        if ! validate_branch_name "$branch"; then
            echo "  Skipping push request from $signal_file due to invalid branch name."
            echo ""
            continue
        fi

        # Re-derive repo_slug from the actual git remote (don't trust container JSON)
        local origin_url
        origin_url="$(git -C "$repo_path" remote get-url origin 2>/dev/null || echo "")"
        if [ -n "$origin_url" ]; then
            repo_slug="$(printf '%s' "$origin_url" | sed -E 's|.*github\.com[:/]||;s|\.git$||')"
        fi

        echo "-----------------------------------------"
        echo "  Issue:      #$issue"
        echo "  Branch:     $branch"
        echo "  Repo:       $repo_slug"
        echo "  Commits:    $commit_count"
        echo "  PR Title:   $pr_title"
        echo "  Requested:  $requested_at"
        echo "-----------------------------------------"
        echo ""

        while true; do
            read -rp "Action: [y]es push / [d]iff / [s]kip / [c]ancel all? " choice
            case "$choice" in
                y|Y|yes)
                    echo ""
                    echo "Pushing $branch to origin..."

                    if ! git -C "$repo_path" push -u origin "$branch"; then
                        echo "ERROR: git push failed." >&2
                        echo "You may need to push manually from: $repo_path" >&2
                        break
                    fi

                    echo "Push successful."
                    echo ""

                    echo "Creating pull request..."
                    local pr_args=(
                        pr create
                        --repo "$repo_slug"
                        --head "$branch"
                        --title "$pr_title"
                    )

                    # Confine body_file to scratchpad (prevents path traversal)
                    if validate_body_file "$body_file"; then
                        pr_args+=(--body-file "$body_file")
                    else
                        pr_args+=(--body "Closes #$issue")
                    fi

                    local pr_url
                    if pr_url=$(gh "${pr_args[@]}" 2>&1); then
                        echo "PR created: $pr_url"
                        local tmp
                        tmp=$(mktemp)
                        jq '.status = "pushed"' "$signal_file" > "$tmp" && mv "$tmp" "$signal_file"
                    else
                        echo "WARNING: PR creation failed: $pr_url" >&2
                        echo "You can create it manually:" >&2
                        printf '  gh pr create --repo %s --head %s --title %s\n' "$(printf %q "$repo_slug")" "$(printf %q "$branch")" "$(printf %q "$pr_title")" >&2
                        # Mark as pushed (git push succeeded) but note PR failure
                        local tmp
                        tmp=$(mktemp)
                        jq '.status = "pushed_no_pr"' "$signal_file" > "$tmp" && mv "$tmp" "$signal_file"
                    fi

                    echo ""
                    break
                    ;;

                d|D|diff)
                    echo ""
                    echo "Showing commits on $branch:"
                    echo ""
                    local default_branch
                    default_branch="$(git -C "$repo_path" symbolic-ref refs/remotes/origin/HEAD 2>/dev/null | sed 's|refs/remotes/origin/||' || echo 'main')"
                    git -C "$repo_path" log --oneline "origin/$default_branch..HEAD" 2>/dev/null || \
                        git -C "$repo_path" log --oneline -"$commit_count"
                    echo ""
                    echo "Diff:"
                    git -C "$repo_path" diff "origin/$default_branch..HEAD" 2>/dev/null || \
                        git -C "$repo_path" diff HEAD~"$commit_count"..HEAD
                    echo ""
                    ;;

                s|S|skip)
                    echo "Skipped."
                    local tmp
                    tmp=$(mktemp)
                    jq '.status = "skipped"' "$signal_file" > "$tmp" && mv "$tmp" "$signal_file"
                    echo ""
                    break
                    ;;

                c|C|cancel)
                    echo "Cancelled — remaining requests left as pending."
                    exit 0
                    ;;

                *)
                    echo "Invalid choice. Use y/d/s/c."
                    ;;
            esac
        done
    done
}

# ========== ISSUE REQUESTS ==========

process_issue_requests() {
    [ -d "$ISSUE_DIR" ] || return 0

    local pending_files=()

    # Scan worktree subdirectories
    for wt_dir in "$ISSUE_DIR"/*/; do
        [ -d "$wt_dir" ] || continue
        local wt_id
        wt_id="$(basename "$wt_dir")"
        matches_filter "$wt_id" || continue

        for signal_file in "$wt_dir"*.json; do
            [ -f "$signal_file" ] || continue
            local status
            status=$(jq -r '.status // "pending"' "$signal_file" 2>/dev/null || echo "unknown")
            [ "$status" = "pending" ] || continue
            pending_files+=("$signal_file")
        done
    done

    [ ${#pending_files[@]} -gt 0 ] || return 0

    echo "========================================="
    echo "  Issue Requests (${#pending_files[@]} pending)"
    echo "========================================="
    echo ""

    for signal_file in "${pending_files[@]}"; do
        local repo_slug title body_file labels_raw requested_at source_issue
        repo_slug=$(jq -r '.repo_slug' "$signal_file")
        title=$(jq -r '.title' "$signal_file")
        body_file=$(jq -r '.body_file // ""' "$signal_file")
        labels_raw=$(jq -r '.labels | if length > 0 then join(", ") else "none" end' "$signal_file" 2>/dev/null || echo "none")
        requested_at=$(jq -r '.requested_at' "$signal_file")
        source_issue=$(jq -r '.source_issue // "unknown"' "$signal_file")

        echo "-----------------------------------------"
        echo "  Repo:       $repo_slug"
        echo "  Title:      $title"
        echo "  Labels:     $labels_raw"
        echo "  Source:     worktree issue #$source_issue"
        echo "  Requested:  $requested_at"
        echo "-----------------------------------------"
        echo ""

        while true; do
            read -rp "Action: [y]es create / [v]iew body / [s]kip / [c]ancel all? " choice
            case "$choice" in
                y|Y|yes)
                    echo ""
                    echo "Creating issue in $repo_slug..."

                    local gh_args=(
                        issue create
                        --repo "$repo_slug"
                        --title "$title"
                    )

                    # Confine body_file to scratchpad (prevents path traversal)
                    if validate_body_file "$body_file"; then
                        gh_args+=(--body-file "$body_file")
                    fi

                    # Add labels individually
                    local labels_arr
                    labels_arr=$(jq -r '.labels[]' "$signal_file" 2>/dev/null || true)
                    while IFS= read -r label; do
                        [ -n "$label" ] || continue
                        gh_args+=(--label "$label")
                    done <<< "$labels_arr"

                    local issue_url
                    if issue_url=$(gh "${gh_args[@]}" 2>&1); then
                        echo "Issue created: $issue_url"

                        local tmp
                        tmp=$(mktemp)
                        jq --arg url "$issue_url" '.status = "created" | .issue_url = $url' "$signal_file" > "$tmp" && mv "$tmp" "$signal_file"
                    else
                        echo "WARNING: Issue creation failed: $issue_url" >&2
                        echo "You can create it manually:" >&2
                        printf '  gh issue create --repo %s --title %s\n' "$(printf %q "$repo_slug")" "$(printf %q "$title")" >&2
                    fi
                    echo ""
                    break
                    ;;

                v|V|view)
                    echo ""
                    if validate_body_file "$body_file"; then
                        echo "--- Issue Body ---"
                        cat "$body_file"
                        echo ""
                        echo "--- End ---"
                    else
                        echo "(no body provided or file outside scratchpad)"
                    fi
                    echo ""
                    ;;

                s|S|skip)
                    echo "Skipped."
                    local tmp
                    tmp=$(mktemp)
                    jq '.status = "skipped"' "$signal_file" > "$tmp" && mv "$tmp" "$signal_file"
                    echo ""
                    break
                    ;;

                c|C|cancel)
                    echo "Cancelled — remaining requests left as pending."
                    exit 0
                    ;;

                *)
                    echo "Invalid choice. Use y/v/s/c."
                    ;;
            esac
        done
    done
}

# ========== Main ==========

FOUND_PUSH=false
FOUND_ISSUE=false

# Process push requests first (higher priority)
if [ -d "$PUSH_DIR" ]; then
    FOUND_PUSH=true
fi
if [ -d "$ISSUE_DIR" ]; then
    FOUND_ISSUE=true
fi

if [ "$FOUND_PUSH" = false ] && [ "$FOUND_ISSUE" = false ]; then
    echo "No outbox directories found."
    exit 0
fi

process_push_requests
process_issue_requests

echo "========================================="
echo "  Gateway complete."
echo "========================================="
