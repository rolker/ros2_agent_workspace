#!/bin/bash
# PR Status Dashboard
# Shows PR pipeline status and provides interactive management
#
# NOTE: Basic PR comment triage (critical/minor classification) is now
# available in status_report.sh --pr-triage, which operates across all
# workspace repos. This script is still the right tool for interactive mode,
# JSON output, and --next-critical/--next-minor agent queries.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export REPO_ROOT
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
ORANGE='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Emojis
EMOJI_REVIEW="🟡"
EMOJI_CRITICAL="🔴"
EMOJI_MINOR="🟠"
EMOJI_READY="🟢"
EMOJI_SEARCH="🔍"
EMOJI_CHART="📊"

# Discover all workspace repos (root + overlay + underlay)
# Returns one owner/repo slug per line, deduplicated and sorted
discover_repos() {
    local repos=()

    # Root workspace repo
    # NOTE: gh CLI only supports GitHub — skip non-GitHub remote URLs
    local root_url
    root_url=$(git -C "$REPO_ROOT" remote get-url origin 2>/dev/null || echo "")
    if [ -n "$root_url" ] && [[ "$root_url" == *"github.com"* ]]; then
        local root_slug
        root_slug=$(echo "$root_url" | sed -E 's#.*github\.com[:/]##' | sed 's/\.git$//')
        if [ -n "$root_slug" ]; then
            repos+=("$root_slug")
        fi
    fi

    # Overlay + underlay repos via list_overlay_repos.py
    local repo_json
    repo_json=$(python3 "$SCRIPT_DIR/list_overlay_repos.py" --include-underlay 2>/dev/null || echo "[]")
    while IFS= read -r url; do
        # Skip non-GitHub URLs — gh CLI is GitHub-only
        if [ -n "$url" ] && [ "$url" != "null" ] && [[ "$url" == *"github.com"* ]]; then
            local slug
            slug=$(echo "$url" | sed -E 's#.*github\.com[:/]##' | sed 's/\.git$//')
            if [ -n "$slug" ]; then
                repos+=("$slug")
            fi
        fi
    done < <(echo "$repo_json" | jq -r '.[].url // empty')

    # Deduplicate and sort
    if [ ${#repos[@]} -gt 0 ]; then
        printf '%s\n' "${repos[@]}" | sort -u
    fi
}

# Function to fetch all open PRs with detailed info
# Optional arg: repo slug (owner/repo) for cross-repo queries
fetch_prs() {
    local args=(--state open --json "number,title,updatedAt,reviewDecision" --limit 100)
    if [ -n "${1:-}" ]; then
        args+=(--repo "$1")
    fi
    gh pr list "${args[@]}"
}

# Function to get review comments for a PR
# Args: pr_number [repo_slug]
get_review_comments() {
    local pr_number=$1
    local repo=${2:-"{owner}/{repo}"}
    gh api --paginate "/repos/${repo}/pulls/${pr_number}/comments" 2>/dev/null | jq -c '.[]'
}

# Function to classify comment severity
# Returns: critical, minor, or empty
classify_comment() {
    local comment_body=$1

    # Critical keywords
    if echo "$comment_body" | grep -iE "(security|vulnerability|bug|error|critical|dangerous|unsafe|leak)" > /dev/null; then
        echo "critical"
        return
    fi

    # Style/minor keywords
    if echo "$comment_body" | grep -iE "(style|formatting|typo|naming|convention|prefer|consider|suggestion)" > /dev/null; then
        echo "minor"
        return
    fi

    # Default to minor
    echo "minor"
}

# Function to analyze PR status
# Args: pr_json [repo_slug]
analyze_pr() {
    local pr_json=$1
    local repo=${2:-"{owner}/{repo}"}
    local repo_name=""
    if [ "$repo" != "{owner}/{repo}" ]; then
        repo_name="$repo"
    fi
    local number
    number=$(echo "$pr_json" | jq -r '.number')
    local title
    title=$(echo "$pr_json" | jq -r '.title')
    local updated
    updated=$(echo "$pr_json" | jq -r '.updatedAt')
    local review_decision
    review_decision=$(echo "$pr_json" | jq -r '.reviewDecision // "PENDING"')

    # Get review info separately
    local reviews_count
    reviews_count=$(gh api --paginate "/repos/${repo}/pulls/${number}/reviews" \
        --jq 'length' 2>/dev/null | awk '{s+=$1} END {print (s=="") ? 0 : s}' || echo "0")

    # Get review comments
    local comments
    comments=$(get_review_comments "$number" "$repo")
    local comment_count=0

    # Classify comments
    local critical_count=0
    local minor_count=0

    if [ -n "$comments" ]; then
        while IFS= read -r comment; do
            if [ -z "$comment" ]; then
                continue
            fi
            comment_count=$((comment_count + 1))
            local body
            body=$(echo "$comment" | jq -r '.body // ""')
            local severity
            severity=$(classify_comment "$body")

            if [ "$severity" = "critical" ]; then
                critical_count=$((critical_count + 1))
            else
                minor_count=$((minor_count + 1))
            fi
        done <<< "$comments"
    fi

    # Determine category
    local category=""
    if [ "$reviews_count" -eq 0 ]; then
        category="needs_review"
    elif [ "$critical_count" -gt 0 ]; then
        category="critical"
    elif [ "$minor_count" -gt 0 ]; then
        category="minor"
    elif [ "$review_decision" = "APPROVED" ]; then
        category="ready"
    else
        category="needs_review"
    fi

    # Calculate time since update
    local time_ago
    if command -v gdate >/dev/null 2>&1; then
        # Prefer GNU date if available as gdate (common on macOS via coreutils)
        time_ago=$(gdate -d "$updated" +%s 2>/dev/null || echo "0")
    elif date -d "$updated" +%s >/dev/null 2>&1; then
        # GNU date available as date (Linux)
        time_ago=$(date -d "$updated" +%s 2>/dev/null || echo "0")
    else
        # Fallback for BSD/macOS date; assumes ISO-8601 like 2024-02-03T12:34:56Z
        time_ago=$(date -j -u -f "%Y-%m-%dT%H:%M:%SZ" "$updated" "+%s" 2>/dev/null || echo "0")
    fi
    local now
    now=$(date +%s)
    local diff=$((now - time_ago))
    local hours=$((diff / 3600))
    local days=$((diff / 86400))

    local time_str=""
    if [ "$days" -gt 0 ]; then
        time_str="${days}d ago"
    elif [ "$hours" -gt 0 ]; then
        time_str="${hours}h ago"
    else
        time_str="recently"
    fi

    # Output JSON for aggregation
    # Only include repo field when non-empty (i.e., in --all-repos mode)
    local base_json
    base_json=$(jq -n \
        --arg category "$category" \
        --arg number "$number" \
        --arg title "$title" \
        --arg time "$time_str" \
        --arg critical "$critical_count" \
        --arg minor "$minor_count" \
        '{category: $category, number: $number, title: $title, time: $time, critical: ($critical | tonumber), minor: ($minor | tonumber)}')
    if [ -n "$repo_name" ]; then
        echo "$base_json" | jq --arg repo "$repo_name" '. + {repo: $repo}'
    else
        echo "$base_json"
    fi
}

# Pre-analyze all PRs in a raw JSON array, returning a JSON array of analyzed objects
# Args: prs_json [repo_slug]
_pre_analyze_prs() {
    local prs_json=$1
    local repo=${2:-""}
    local all_analyzed="[]"

    while IFS= read -r pr; do
        local analyzed
        if [ -n "$repo" ]; then
            analyzed=$(analyze_pr "$pr" "$repo")
        else
            analyzed=$(analyze_pr "$pr")
        fi
        all_analyzed=$(echo "$all_analyzed" | jq --argjson item "$analyzed" '. + [$item]')
    done < <(echo "$prs_json" | jq -c '.[]')

    echo "$all_analyzed"
}

# Function to display dashboard (single-repo mode)
# Delegates to display_analyzed_dashboard after pre-analyzing
# Args: prs_json [interactive] [repo_slug]
display_dashboard() {
    local prs_json=$1
    local repo=${3:-""}
    local analyzed
    analyzed=$(_pre_analyze_prs "$prs_json" "$repo")
    display_analyzed_dashboard "$analyzed"
}

# Function to display a category
display_category() {
    local label=$1
    local emoji=$2
    local color=$3
    shift 3
    local items=("$@")

    if [ ${#items[@]} -eq 0 ]; then
        return
    fi

    echo -e "${color}${emoji} ${label} (${#items[@]})${NC}"
    for item in "${items[@]}"; do
        local number
        number=$(echo "$item" | jq -r '.number')
        local title
        title=$(echo "$item" | jq -r '.title' | cut -c1-50)
        local time
        time=$(echo "$item" | jq -r '.time')
        local critical_count
        critical_count=$(echo "$item" | jq -r '.critical')
        local minor_count
        minor_count=$(echo "$item" | jq -r '.minor')
        local item_repo
        item_repo=$(echo "$item" | jq -r '.repo // ""')

        if [ -n "$item_repo" ]; then
            printf "  [%s] #%-4s %-40s (last: %s)\n" "$item_repo" "$number" "$title" "$time"
        else
            printf "  #%-4s %-50s (last: %s)\n" "$number" "$title" "$time"
        fi

        if [ "$critical_count" -gt 0 ]; then
            echo "        → $critical_count critical comment(s)"
        fi
        if [ "$minor_count" -gt 0 ]; then
            echo "        → $minor_count minor comment(s)"
        fi
    done
    echo ""
}

# Wrap a pre-analyzed JSON array with summary counts
# Args: analyzed_json_array (stdin or $1)
_wrap_json_summary() {
    local analyzed_json=${1:-$(cat)}
    echo "$analyzed_json" | jq '{
        summary: {
            total: (. | length),
            needs_review: ([.[] | select(.category == "needs_review")] | length),
            critical: ([.[] | select(.category == "critical")] | length),
            minor: ([.[] | select(.category == "minor")] | length),
            ready: ([.[] | select(.category == "ready")] | length)
        },
        prs: .
    }'
}

# Function to output in JSON format (for agents)
# Delegates to _pre_analyze_prs + _wrap_json_summary
# Args: prs_json [repo_slug]
output_json() {
    local prs_json=$1
    local repo=${2:-""}
    local analyzed
    analyzed=$(_pre_analyze_prs "$prs_json" "$repo")
    _wrap_json_summary "$analyzed"
}

# Function to get next actionable PR (single-repo mode)
# Delegates to _pre_analyze_prs + get_next_analyzed_pr
# Args: prs_json [category] [repo_slug]
get_next_pr() {
    local prs_json=$1
    local category=${2:-"critical"}
    local repo=${3:-""}
    local analyzed
    analyzed=$(_pre_analyze_prs "$prs_json" "$repo")
    get_next_analyzed_pr "$analyzed" "$category"
}

# Function to output simple text summary (single-repo mode)
# Delegates to _pre_analyze_prs + output_analyzed_simple
# Args: prs_json [repo_slug]
output_simple() {
    local prs_json=$1
    local repo=${2:-""}
    local analyzed
    analyzed=$(_pre_analyze_prs "$prs_json" "$repo")
    output_analyzed_simple "$analyzed"
}

# Function to display from pre-analyzed JSON array
# Used by --all-repos mode to avoid re-analyzing PRs
# Args: analyzed_json_array [title_suffix]
display_analyzed_dashboard() {
    local analyzed_json=$1
    local title_suffix=${2:-""}

    echo -e "${BLUE}${EMOJI_SEARCH} PR Status Dashboard${title_suffix}${NC}"
    echo "===================="
    echo ""

    local needs_review=()
    local critical=()
    local minor=()
    local ready=()
    while IFS= read -r item; do
        local cat
        cat=$(echo "$item" | jq -r '.category')
        case "$cat" in
            needs_review) needs_review+=("$item") ;;
            critical) critical+=("$item") ;;
            minor) minor+=("$item") ;;
            ready) ready+=("$item") ;;
        esac
    done < <(echo "$analyzed_json" | jq -c '.[]')

    display_category "NEEDS REVIEW" "$EMOJI_REVIEW" "$YELLOW" "${needs_review[@]}"
    display_category "CRITICAL ISSUES" "$EMOJI_CRITICAL" "$RED" "${critical[@]}"
    display_category "MINOR ISSUES" "$EMOJI_MINOR" "$ORANGE" "${minor[@]}"
    display_category "READY TO MERGE" "$EMOJI_READY" "$GREEN" "${ready[@]}"

    local total=$((${#needs_review[@]} + ${#critical[@]} + ${#minor[@]} + ${#ready[@]}))
    echo ""
    echo -e "${BLUE}${EMOJI_CHART} Summary:${NC} $total open PRs | ${#needs_review[@]} need review | $((${#critical[@]} + ${#minor[@]})) need fixes | ${#ready[@]} ready"
    echo ""
}

# Function to output simple text from pre-analyzed JSON array
# Args: analyzed_json_array
output_analyzed_simple() {
    local analyzed_json=$1

    local needs_review=()
    local critical=()
    local minor=()
    local ready=()
    while IFS= read -r item; do
        local cat
        cat=$(echo "$item" | jq -r '.category')
        case "$cat" in
            needs_review) needs_review+=("$item") ;;
            critical) critical+=("$item") ;;
            minor) minor+=("$item") ;;
            ready) ready+=("$item") ;;
        esac
    done < <(echo "$analyzed_json" | jq -c '.[]')

    echo "SUMMARY: ${#needs_review[@]} need review, ${#critical[@]} critical, ${#minor[@]} minor, ${#ready[@]} ready"

    if [ ${#critical[@]} -gt 0 ]; then
        echo ""
        echo "CRITICAL ISSUES:"
        for item in "${critical[@]}"; do
            local number title crit min item_repo
            number=$(echo "$item" | jq -r '.number')
            title=$(echo "$item" | jq -r '.title')
            crit=$(echo "$item" | jq -r '.critical')
            min=$(echo "$item" | jq -r '.minor')
            item_repo=$(echo "$item" | jq -r '.repo // ""')
            if [ -n "$item_repo" ]; then
                echo "  [$item_repo] #$number: $title ($crit critical, $min minor)"
            else
                echo "  #$number: $title ($crit critical, $min minor)"
            fi
        done
    fi

    if [ ${#needs_review[@]} -gt 0 ]; then
        echo ""
        echo "NEEDS REVIEW:"
        for item in "${needs_review[@]}"; do
            local number title item_repo
            number=$(echo "$item" | jq -r '.number')
            title=$(echo "$item" | jq -r '.title')
            item_repo=$(echo "$item" | jq -r '.repo // ""')
            if [ -n "$item_repo" ]; then
                echo "  [$item_repo] #$number: $title"
            else
                echo "  #$number: $title"
            fi
        done
    fi

    if [ ${#ready[@]} -gt 0 ]; then
        echo ""
        echo "READY TO MERGE:"
        for item in "${ready[@]}"; do
            local number title item_repo
            number=$(echo "$item" | jq -r '.number')
            title=$(echo "$item" | jq -r '.title')
            item_repo=$(echo "$item" | jq -r '.repo // ""')
            if [ -n "$item_repo" ]; then
                echo "  [$item_repo] #$number: $title"
            else
                echo "  #$number: $title"
            fi
        done
    fi
}

# Function to get next actionable PR from pre-analyzed JSON array
# Args: analyzed_json_array category
get_next_analyzed_pr() {
    local analyzed_json=$1
    local category=${2:-"critical"}

    while IFS= read -r item; do
        local pr_category
        pr_category=$(echo "$item" | jq -r '.category')
        if [ "$pr_category" = "$category" ]; then
            echo "$item"
            return 0
        fi
    done < <(echo "$analyzed_json" | jq -c '.[]')

    return 1
}

# Function to run interactive mode
run_interactive() {
    while true; do
        clear
        local prs
        prs=$(fetch_prs)
        display_dashboard "$prs" true

        echo "What would you like to do?"
        echo "1) Review a PR (launch Copilot review)"
        echo "2) Fix issues on a PR (launch agent)"
        echo "3) Merge a PR"
        echo "4) Refresh"
        echo "q) Quit"
        echo ""
        read -p "> " choice

        case "$choice" in
            1)
                read -p "Enter PR number: " pr_num
                echo "Launching Copilot review for PR #$pr_num..."
                gh pr review "$pr_num" --copilot || true
                read -p "Press enter to continue..."
                ;;
            2)
                read -p "Enter PR number: " pr_num
                echo "To launch agent for PR #$pr_num, use:"
                echo "  gh copilot 'Check comments on PR $pr_num and fix the issues'"
                read -p "Press enter to continue..."
                ;;
            3)
                read -p "Enter PR number: " pr_num
                echo "Merging PR #$pr_num..."
                gh pr merge "$pr_num" --squash || true
                read -p "Press enter to continue..."
                ;;
            4)
                # Refresh (loop continues)
                ;;
            q|Q)
                echo "Goodbye!"
                exit 0
                ;;
            *)
                echo "Invalid choice"
                read -p "Press enter to continue..."
                ;;
        esac
    done
}

# Main
main() {
    local interactive=false
    local all_repos=false
    local mode="dashboard"  # dashboard, json, simple, next-critical, next-minor

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            -i|--interactive)
                interactive=true
                shift
                ;;
            --all-repos)
                all_repos=true
                shift
                ;;
            --json)
                mode="json"
                shift
                ;;
            --simple)
                mode="simple"
                shift
                ;;
            --next-critical)
                mode="next-critical"
                shift
                ;;
            --next-minor)
                mode="next-minor"
                shift
                ;;
            --next-review)
                mode="next-review"
                shift
                ;;
            -h|--help)
                echo "Usage: $0 [OPTIONS]"
                echo ""
                echo "Options:"
                echo "  -i, --interactive    Run in interactive mode (for human use)"
                echo "  --all-repos         Query all workspace repos (cross-repo triage)"
                echo "  --json              Output full status as JSON (for agents)"
                echo "  --simple            Output simple text summary (for agents)"
                echo "  --next-critical     Output next PR with critical issues (for agents)"
                echo "  --next-minor        Output next PR with minor issues (for agents)"
                echo "  --next-review       Output next PR needing review (for agents)"
                echo "  -h, --help          Show this help message"
                echo ""
                echo "Examples:"
                echo "  $0                       # Show dashboard (human-friendly)"
                echo "  $0 --interactive         # Interactive menu (human-friendly)"
                echo "  $0 --all-repos           # Dashboard across all workspace repos"
                echo "  $0 --all-repos --simple  # Simple text triage across all repos"
                echo "  $0 --all-repos --json    # JSON output with repo field"
                echo "  $0 --simple              # Simple text output (agent-friendly)"
                echo "  $0 --json                # JSON output (agent-friendly)"
                echo "  $0 --next-critical       # Get next critical PR (agent-friendly)"
                exit 0
                ;;
            *)
                echo "Unknown option: $1"
                exit 1
                ;;
        esac
    done

    # Reject unsupported flag combinations
    if [ "$all_repos" = true ] && [ "$interactive" = true ]; then
        echo "Error: --interactive is not supported with --all-repos"
        exit 1
    fi

    if [ "$all_repos" = true ]; then
        # Multi-repo mode: iterate all workspace repos, analyze once, reuse output functions
        local repos
        repos=$(discover_repos)
        local merged_prs="[]"

        while IFS= read -r repo; do
            if [ -z "$repo" ]; then
                continue
            fi
            if [ "$mode" = "dashboard" ]; then
                echo -e "${BLUE}Fetching PRs from ${repo}...${NC}" >&2
            fi
            local repo_prs
            repo_prs=$(fetch_prs "$repo" 2>/dev/null || echo "[]")

            # Tag each PR with repo slug for downstream functions
            repo_prs=$(echo "$repo_prs" | jq --arg repo "$repo" '[.[] | . + {_repo: $repo}]')
            merged_prs=$(echo "$merged_prs $repo_prs" | jq -s '.[0] + .[1]')
        done <<< "$repos"

        # Pre-analyze all PRs with their repo context
        local all_analyzed="[]"
        while IFS= read -r pr; do
            local pr_repo
            pr_repo=$(echo "$pr" | jq -r '._repo // ""')
            local analyzed
            analyzed=$(analyze_pr "$pr" "$pr_repo")
            all_analyzed=$(echo "$all_analyzed" | jq --argjson item "$analyzed" '. + [$item]')
        done < <(echo "$merged_prs" | jq -c '.[]')

        # Dispatch to output mode using shared helpers
        case "$mode" in
            json)
                _wrap_json_summary "$all_analyzed"
                ;;
            simple)
                output_analyzed_simple "$all_analyzed"
                ;;
            next-critical)
                get_next_analyzed_pr "$all_analyzed" "critical" || echo "No PRs with critical issues found"
                ;;
            next-minor)
                get_next_analyzed_pr "$all_analyzed" "minor" || echo "No PRs with minor issues found"
                ;;
            next-review)
                get_next_analyzed_pr "$all_analyzed" "needs_review" || echo "No PRs needing review found"
                ;;
            dashboard)
                display_analyzed_dashboard "$all_analyzed" " (All Repos)"
                ;;
        esac
    else
        # Single-repo mode (original behavior)
        local prs
        prs=$(fetch_prs)

        if [ "$interactive" = true ]; then
            run_interactive
        else
            case "$mode" in
                json)
                    output_json "$prs"
                    ;;
                simple)
                    output_simple "$prs"
                    ;;
                next-critical)
                    get_next_pr "$prs" "critical" || echo "No PRs with critical issues found"
                    ;;
                next-minor)
                    get_next_pr "$prs" "minor" || echo "No PRs with minor issues found"
                    ;;
                next-review)
                    get_next_pr "$prs" "needs_review" || echo "No PRs needing review found"
                    ;;
                dashboard)
                    display_dashboard "$prs"
                    ;;
            esac
        fi
    fi
}

main "$@"
