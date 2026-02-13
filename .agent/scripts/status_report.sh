#!/bin/bash

# ROS2 Agent Workspace Status Report
# Comprehensive workspace status: repository sync, git status, GitHub PRs/issues, tests.
#
# Usage:
#   status_report.sh [OPTIONS]
#
# OPTIONS:
#   --quick          Quick local-only status (alias for --skip-sync --skip-github)
#   --skip-sync      Skip repository fetch step (faster, may show stale data)
#   --skip-github    Skip GitHub PR/issue queries (offline mode)
#   --pr-triage      Classify PR review comments as critical/minor (extra API calls)
#   --help           Show this help message
#
# EXAMPLES:
#   status_report.sh                    # Full status with sync and GitHub
#   status_report.sh --quick            # Fast local-only check
#   status_report.sh --skip-sync        # Skip fetch, keep GitHub queries
#   status_report.sh --pr-triage        # Full status + PR comment classification
#
# DEPENDENCIES:
#   Required: vcs, git, python3, jq
#   Optional: gh (GitHub CLI) - for PR/issue tracking and --pr-triage

# Note: do not use 'set -e' here; many status checks are best-effort and may fail.

# Suppress Python deprecation warnings from vcstool
export PYTHONWARNINGS="ignore::DeprecationWarning"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

# Parse command-line arguments
SKIP_SYNC=false
SKIP_GITHUB=false
PR_TRIAGE=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --quick)
            SKIP_SYNC=true
            SKIP_GITHUB=true
            shift
            ;;
        --skip-sync)
            SKIP_SYNC=true
            shift
            ;;
        --skip-github)
            SKIP_GITHUB=true
            shift
            ;;
        --pr-triage)
            PR_TRIAGE=true
            shift
            ;;
        --help)
            awk 'NR>1 && /^# Note: do not use/ {exit} NR>1 {sub(/^# ?/, ""); print}' "$0"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Worktree detection - adjust paths based on context
WORKTREE_INFO=""
if [[ "$ROOT_DIR" == *"/layers/worktrees/"* ]]; then
    WORKTREE_INFO="layer worktree"
    LAYERS_DIR="$ROOT_DIR"  # In layer worktree, ROOT_DIR contains *_ws dirs
elif [[ "$ROOT_DIR" == *"/.workspace-worktrees/"* ]]; then
    WORKTREE_INFO="workspace worktree"
    LAYERS_DIR="$ROOT_DIR/layers/main"
else
    LAYERS_DIR="$ROOT_DIR/layers/main"
fi

echo "# Workspace Status Report"
echo "**Date**: $(date)"
if [ -n "$WORKTREE_INFO" ]; then
    echo "**Context**: Running in $WORKTREE_INFO"
fi
echo ""

#######################################
# STEP 1: SYNC REPOSITORIES
#######################################

if [ "$SKIP_SYNC" = false ]; then
    echo "## Step 1: Syncing Repositories"
    echo ""

    # Sync root repository (use subshell to avoid changing working directory)
    (
        cd "$ROOT_DIR" || exit 0
        echo -n "Syncing root repository... "
        if git fetch --quiet 2>/dev/null; then
            echo "✅"
        else
            echo "⚠️ (fetch failed or skipped)"
        fi
    )

    # Sync layer repositories
    if command -v vcs &> /dev/null; then
        for ws_dir in "$LAYERS_DIR"/*; do
            if [ -d "$ws_dir/src" ]; then
                ws_name=$(basename "$ws_dir" | sed 's/_ws//')
                echo -n "Syncing $ws_name workspace... "

                # Use subshell to avoid changing working directory
                (
                    cd "$ws_dir/src" || exit 0
                    if vcs custom --git --args fetch --quiet >/dev/null 2>&1; then
                        echo "✅"
                    else
                        echo "⚠️ (some repos may have failed)"
                    fi
                )
            fi
        done
    else
        echo "⚠️ vcs command not found - skipping layer sync"
    fi

    echo ""
    echo "---"
    echo ""
fi

#######################################
# STEP 2: REPOSITORY STATUS
#######################################

STEP_NUM=1
if [ "$SKIP_SYNC" = false ]; then
    STEP_NUM=2
fi

echo "## Step $STEP_NUM: Repository Status"
echo ""

# Root Repository Status
echo "### Root Repository"
cd "$ROOT_DIR" || exit
if command -v git &> /dev/null; then
    if [ -n "$(git status --porcelain)" ]; then
        echo "- **Status**: ⚠️ Modified"
        echo "- **Branch**: $(git branch --show-current)"
        echo ""
        echo "**Modified Files:**"
        echo '```'
        git status --short
        echo '```'
    else
        echo "- **Status**: ✅ Clean"
        echo "- **Branch**: $(git branch --show-current)"
    fi
else
    echo "- **Status**: ❓ Git not found"
fi
echo ""

# Layer Repositories (VCS)
if ! command -v vcs &> /dev/null; then
    echo "### Layers"
    echo "**Error**: \`vcs\` command not found. Please install \`python3-vcstool\`."
else
    # Fetch expected repositories (including underlay) for tracking checks
    EXPECTED_REPOS=$(python3 "$SCRIPT_DIR/list_overlay_repos.py" --include-underlay --format names 2>/dev/null)
    if [ $? -ne 0 ] || [ -z "$EXPECTED_REPOS" ]; then
        EXPECTED_REPOS=""
    fi

    for ws_dir in "$LAYERS_DIR"/*; do
        if [ -d "$ws_dir/src" ]; then
            ws_name=$(basename "$ws_dir" | sed 's/_ws//')

            if ! cd "$ws_dir/src"; then
                echo "Warning: could not enter workspace directory '$ws_dir/src'; skipping."
                continue
            fi

            # Get status with branch info
            raw_output=$(vcs custom --git --args status --porcelain -b 2>/dev/null || echo "")

            clean_count=0
            modified_count=0
            modified_repos=()

            # Process the output
            current_repo=""
            is_dirty=false
            sync_status=""
            branch=""

            process_repo() {
                if [ "$current_repo" != "" ]; then
                    local status_str=""
                    if [ "$is_dirty" = true ]; then
                        status_str="⚠️ Modified"
                    fi

                    if [ "$sync_status" != "" ]; then
                        status_str="${status_str:+$status_str, }$sync_status"
                    fi

                    # Fetch expected branch from config
                    if [ -f "$SCRIPT_DIR/get_repo_info.py" ]; then
                        expected_branch=$(python3 "$SCRIPT_DIR/get_repo_info.py" "$current_repo" 2>/dev/null)
                    else
                        expected_branch="unknown"
                    fi

                    if [ "$expected_branch" != "unknown" ] && [ -n "$expected_branch" ]; then
                         if [ "$branch" != "$expected_branch" ]; then
                            warning="🔀 $branch (Want: $expected_branch)"
                            status_str="${status_str:+$status_str, }$warning"
                         fi
                    elif [ "$branch" != "jazzy" ] && [ "$current_repo" != "ros2_agent_workspace" ]; then
                         status_str="${status_str:+$status_str, }🔀 Non-Jazzy?"
                    fi

                    if [ -n "$EXPECTED_REPOS" ]; then
                        if ! echo "$EXPECTED_REPOS" | grep -qx "$current_repo"; then
                             status_str="${status_str:+$status_str, }❓ Untracked"
                        fi
                    fi

                    if [ "$status_str" != "" ]; then
                        modified_repos+=("$current_repo|$status_str|$branch")
                        modified_count=$((modified_count + 1))
                    else
                        clean_count=$((clean_count + 1))
                    fi
                fi
            }

            while IFS= read -r line; do
                if [[ "$line" == "="* ]]; then
                    # Process previous repo
                    process_repo

                    # Start new repo
                    current_repo=$(echo "$line" | sed 's/^=== \(.*\) (git) ===$/\1/')
                    is_dirty=false
                    sync_status=""
                    branch=""
                elif [[ "$line" =~ ^##[[:space:]](.*)$ ]]; then
                    # Extract branch and sync status
                    branch_line="${BASH_REMATCH[1]}"
                    if [[ "$branch_line" =~ \.\.\..*\[ahead[[:space:]]([0-9]+)\] ]]; then
                        sync_status="⬆️ Ahead ${BASH_REMATCH[1]}"
                    elif [[ "$branch_line" =~ \.\.\..*\[behind[[:space:]]([0-9]+)\] ]]; then
                        sync_status="⬇️ Behind ${BASH_REMATCH[1]}"
                    elif [[ "$branch_line" =~ \.\.\..*\[ahead[[:space:]]([0-9]+),[[:space:]]behind[[:space:]]([0-9]+)\] ]]; then
                        sync_status="↕️ Ahead ${BASH_REMATCH[1]}, Behind ${BASH_REMATCH[2]}"
                    fi
                    # Extract branch name (first word)
                    branch=$(echo "$branch_line" | awk '{print $1}')
                elif [[ "$line" =~ ^[[:space:]]?[MADRCU?!] ]]; then
                    is_dirty=true
                fi
            done <<< "$raw_output"

            # Handle last repo
            process_repo

            total=$((clean_count + modified_count))

            echo "### Workspace: $ws_name (Total: $total, Clean: $clean_count, Attention: $modified_count)"

            if [ $modified_count -gt 0 ]; then
                echo ""
                echo "| Repository | Status | Branch |"
                echo "|---|---|---|"
                for repo_entry in "${modified_repos[@]}"; do
                    IFS='|' read -r repo_name status_info branch_info <<< "$repo_entry"
                    printf "| %-30s | %-40s | %-20s |\n" "$repo_name" "$status_info" "$branch_info"
                done
            fi
            echo ""
        fi
    done
fi

echo "---"
echo ""

#######################################
# STEP 3: GITHUB PULL REQUESTS
#######################################

if [ "$SKIP_GITHUB" = false ] && command -v gh &> /dev/null; then
    # Ensure jq is available for GitHub-related JSON parsing
    if ! command -v jq &> /dev/null; then
        echo "Error: 'jq' is required for GitHub status checks but is not installed."
        echo "Install 'jq' (https://stedolan.github.io/jq/) or re-run this script with --skip-github to skip GitHub checks."
        SKIP_GITHUB=true
    fi
fi

if [ "$SKIP_GITHUB" = false ] && command -v gh &> /dev/null && command -v jq &> /dev/null; then
    # Generate repository list once for both PR and Issues sections.
    # The sed filters are applied sequentially to normalize URLs to "owner/repo":
    # - For overlay repos we expect HTTPS URLs; we strip the https:// prefix and trailing ".git".
    REPOS=$(python3 "$SCRIPT_DIR/list_overlay_repos.py" --include-underlay 2>/dev/null | jq -r '.[].url' 2>/dev/null | sed 's|https://github.com/||' | sed 's|.git$||' || true)

    # Add root repository, normalizing both SSH and HTTPS origin URLs to "owner/repo":
    # - first sed removes SSH prefix "git@github.com:" if present
    # - second sed removes HTTPS prefix "https://github.com/" if present
    #   (for any given URL, only one of these prefixes will match)
    # - third sed removes the trailing ".git"
    ROOT_REPO=$(cd "$ROOT_DIR" && git remote get-url origin 2>/dev/null | sed 's|git@github.com:||' | sed 's|https://github.com/||' | sed 's|.git$||' || true)

    if [ -n "$ROOT_REPO" ]; then
        REPOS=$(echo -e "$ROOT_REPO\n$REPOS")
    fi

    # Remove duplicates and sort
    REPOS=$(echo "$REPOS" | sort -u | grep -v '^$')
fi

if [ "$SKIP_GITHUB" = false ]; then
    STEP_NUM=$((STEP_NUM + 1))
    echo "## Step $STEP_NUM: GitHub Pull Requests"
    echo ""

    if ! command -v gh &> /dev/null; then
        echo "⚠️ **GitHub CLI (\`gh\`) not found**"
        echo ""
        echo "Install with: \`sudo apt install gh\` or \`brew install gh\`"
        echo "Then authenticate: \`gh auth login\`"
        echo ""
    elif ! gh auth status &> /dev/null; then
        echo "⚠️ **GitHub CLI not authenticated**"
        echo ""
        echo "Please authenticate with: \`gh auth login\`"
        echo "This is required to fetch PR and issue information."
        echo ""
    else
        # Repository list already generated above

        # Query PRs for each repository
        PR_COUNT=0
        PR_OUTPUT=""

        for repo in $REPOS; do
            if [ -z "$repo" ]; then
                continue
            fi

            # Query GitHub API for open PRs
            prs=$(gh pr list --repo "$repo" --json number,title,url --limit 100 2>/dev/null || echo "[]")

            if [ -n "$prs" ] && [ "$prs" != "[]" ]; then
                # Process each PR
                while read -r pr_line; do
                    if [ -z "$pr_line" ]; then
                        continue
                    fi

                    number=$(echo "$pr_line" | jq -r '.number')
                    title=$(echo "$pr_line" | jq -r '.title')
                    url=$(echo "$pr_line" | jq -r '.url')

                    # Truncate title if too long
                    if [ ${#title} -gt 60 ]; then
                        title="${title:0:57}..."
                    fi

                    # Format as table row
                    repo_name=$(basename "$repo")
                    PR_OUTPUT+="| $repo_name | [#$number]($url) | $title |"$'\n'
                    PR_COUNT=$((PR_COUNT + 1))
                done < <(echo "$prs" | jq -c '.[]' 2>/dev/null)
            fi
        done

        if [ $PR_COUNT -gt 0 ]; then
            echo "| Repository | PR | Title |"
            echo "|------------|-----|-------|"
            printf '%s\n' "$PR_OUTPUT"
            echo "**Total Open PRs**: $PR_COUNT"
        else
            echo "✅ No open pull requests"
        fi
        echo ""
    fi

    echo "---"
    echo ""
fi

#######################################
# STEP 3b: PR TRIAGE (optional)
#######################################

if [ "$PR_TRIAGE" = true ] && [ "$SKIP_GITHUB" = false ] && command -v gh &> /dev/null && command -v jq &> /dev/null && gh auth status &> /dev/null; then
    STEP_NUM=$((STEP_NUM + 1))
    echo "## Step $STEP_NUM: PR Comment Triage"
    echo ""

    # classify_comment: keyword-based critical/minor classification
    # Ported from pr_status.sh for behavioral parity
    classify_comment() {
        local comment_body=$1
        if echo "$comment_body" | grep -iE "(security|vulnerability|bug|error|critical|dangerous|unsafe|leak)" > /dev/null; then
            echo "critical"
            return
        fi
        if echo "$comment_body" | grep -iE "(style|formatting|typo|naming|convention|prefer|consider|suggestion)" > /dev/null; then
            echo "minor"
            return
        fi
        echo "minor"
    }

    TRIAGE_OUTPUT=""
    TRIAGE_TOTAL=0
    TRIAGE_NEED_REVIEW=0
    TRIAGE_CRITICAL=0
    TRIAGE_MINOR=0
    TRIAGE_READY=0

    for repo in $REPOS; do
        if [ -z "$repo" ]; then
            continue
        fi

        # Get open PRs for this repo
        prs=$(gh pr list --repo "$repo" --json number,title,reviewDecision --limit 100 2>/dev/null || echo "[]")
        if [ "$prs" = "[]" ] || [ -z "$prs" ]; then
            continue
        fi

        while read -r pr_line; do
            if [ -z "$pr_line" ]; then
                continue
            fi

            pr_number=$(echo "$pr_line" | jq -r '.number')
            pr_title=$(echo "$pr_line" | jq -r '.title')
            review_decision=$(echo "$pr_line" | jq -r '.reviewDecision // "PENDING"')

            # Truncate title
            if [ ${#pr_title} -gt 50 ]; then
                pr_title="${pr_title:0:47}..."
            fi

            # Fetch review comments for this PR (2 API calls: reviews + comments)
            reviews_count=$(gh api "repos/$repo/pulls/$pr_number/reviews" --jq 'length' 2>/dev/null || echo "0")
            comments=$(gh api "repos/$repo/pulls/$pr_number/comments" 2>/dev/null | jq -c '.[]' 2>/dev/null || true)

            critical_count=0
            minor_count=0

            if [ -n "$comments" ]; then
                while IFS= read -r comment; do
                    if [ -z "$comment" ]; then
                        continue
                    fi
                    body=$(echo "$comment" | jq -r '.body // ""')
                    severity=$(classify_comment "$body")
                    if [ "$severity" = "critical" ]; then
                        critical_count=$((critical_count + 1))
                    else
                        minor_count=$((minor_count + 1))
                    fi
                done <<< "$comments"
            fi

            # Determine category
            category=""
            if [ "$reviews_count" -eq 0 ] 2>/dev/null; then
                category="needs_review"
                TRIAGE_NEED_REVIEW=$((TRIAGE_NEED_REVIEW + 1))
            elif [ "$critical_count" -gt 0 ]; then
                category="critical"
                TRIAGE_CRITICAL=$((TRIAGE_CRITICAL + 1))
            elif [ "$minor_count" -gt 0 ]; then
                category="minor"
                TRIAGE_MINOR=$((TRIAGE_MINOR + 1))
            elif [ "$review_decision" = "APPROVED" ]; then
                category="ready"
                TRIAGE_READY=$((TRIAGE_READY + 1))
            else
                category="needs_review"
                TRIAGE_NEED_REVIEW=$((TRIAGE_NEED_REVIEW + 1))
            fi

            repo_name=$(basename "$repo")
            TRIAGE_OUTPUT+="| $repo_name | #$pr_number | $pr_title | $category | $critical_count | $minor_count |"$'\n'
            TRIAGE_TOTAL=$((TRIAGE_TOTAL + 1))
        done < <(echo "$prs" | jq -c '.[]' 2>/dev/null)
    done

    if [ "$TRIAGE_TOTAL" -gt 0 ]; then
        echo "| Repository | PR | Title | Category | Critical | Minor |"
        echo "|---|---|---|---|---|---|"
        printf '%s' "$TRIAGE_OUTPUT"
        echo ""
        echo "**Summary**: $TRIAGE_TOTAL PRs — $TRIAGE_NEED_REVIEW need review, $TRIAGE_CRITICAL critical, $TRIAGE_MINOR minor, $TRIAGE_READY ready"
    else
        echo "✅ No open PRs to triage"
    fi
    echo ""
    echo "---"
    echo ""
fi

#######################################
# STEP 4: GITHUB ISSUES
#######################################

if [ "$SKIP_GITHUB" = false ]; then
    STEP_NUM=$((STEP_NUM + 1))
    echo "## Step $STEP_NUM: GitHub Issues"
    echo ""

    if ! command -v gh &> /dev/null; then
        echo "⚠️ **GitHub CLI (\`gh\`) not found** (same as above)"
        echo ""
    elif ! gh auth status &> /dev/null; then
        echo "⚠️ **GitHub CLI (\`gh\`) is not authenticated**"
        echo "   Run \`gh auth login\` to configure GitHub authentication."
        echo ""
    else
        # Check if REPOS variable is set (should be from PR section above)
        if [ -z "${REPOS:-}" ]; then
            echo "ℹ️  No repositories available for GitHub issue query."
            echo ""
        else
            # Query issues for each repository
            ISSUE_COUNT=0
            ISSUE_OUTPUT=""

            for repo in $REPOS; do
                if [ -z "$repo" ]; then
                    continue
                fi

                # Query GitHub API for open issue count using search API (accurate total_count)
                count=$(gh api -X GET search/issues -f q="repo:$repo is:issue is:open" --jq '.total_count' 2>/dev/null || echo "0")

                # Ensure count is a valid integer
                if ! [[ "$count" =~ ^[0-9]+$ ]]; then
                    count=0
                fi

                if [ "$count" -gt 0 ]; then
                    repo_name=$(basename "$repo")
                    issue_url="https://github.com/$repo/issues"
                    ISSUE_OUTPUT+="| [$repo_name]($issue_url) | $count |"$'\n'
                    ISSUE_COUNT=$((ISSUE_COUNT + count))
                fi
            done

            if [ ${#ISSUE_OUTPUT} -gt 0 ]; then
                echo "| Repository | Open Issues |"
                echo "|------------|-------------|"
                printf '%s\n' "$ISSUE_OUTPUT"
                echo "**Total Open Issues**: $ISSUE_COUNT"
            else
                echo "✅ No open issues"
            fi
        fi
        echo ""
    fi

    echo "---"
    echo ""
fi

#######################################
# STEP 5: LATEST TEST STATUS
#######################################

SUMMARY_JSON="$ROOT_DIR/.agent/scratchpad/test_summary.json"
if [ -f "$SUMMARY_JSON" ]; then
    STEP_NUM=$((STEP_NUM + 1))
    echo "## Step $STEP_NUM: Latest Test Status"
    python3 -c "
import json, sys
try:
    with open('$SUMMARY_JSON') as f:
        data = json.load(f)
    ts = data.get('timestamp', 'Unknown')
    overall = '✅ Passed' if data.get('overall_success') else '❌ Failed'
    print(f'**Date**: {ts}')
    print(f'**Overall**: {overall}')
    print('')
    print('| Layer | Result | Summary |')
    print('|---|---|---|')
    for layer in data.get('layers', []):
        r = layer.get('result')
        res = '✅ Passed' if r == 'passed' else ('⏭️ Skipped' if r == 'skipped' else '❌ Failed')
        summ = layer.get('summary', '')
        name = layer.get('name')
        print(f'| {name} | {res} | {summ} |')
except Exception as e:
    print(f'Error parsing test summary: {e}')
"
    echo ""
    echo "---"
    echo ""
fi

#######################################
# COMPLETION
#######################################

echo "✅ **Status report complete**"
