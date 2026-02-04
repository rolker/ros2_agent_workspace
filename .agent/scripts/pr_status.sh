#!/bin/bash
# PR Status Dashboard
# Shows PR pipeline status and provides interactive management

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
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

# Function to fetch all open PRs with detailed info
fetch_prs() {
    gh pr list \
        --state open \
        --json number,title,updatedAt,reviewDecision \
        --limit 100
}

# Function to get review comments for a PR
get_review_comments() {
    local pr_number=$1
    gh api "/repos/{owner}/{repo}/pulls/${pr_number}/comments" 2>/dev/null | jq -c '.[]'
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
analyze_pr() {
    local pr_json=$1
    local number=$(echo "$pr_json" | jq -r '.number')
    local title=$(echo "$pr_json" | jq -r '.title')
    local updated=$(echo "$pr_json" | jq -r '.updatedAt')
    local review_decision=$(echo "$pr_json" | jq -r '.reviewDecision // "PENDING"')
    
    # Get review info separately
    local reviews_count=$(gh api "/repos/{owner}/{repo}/pulls/${number}/reviews" --jq 'length' 2>/dev/null || echo "0")
    
    # Get review comments
    local comments=$(get_review_comments "$number")
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
            local body=$(echo "$comment" | jq -r '.body // ""')
            local severity=$(classify_comment "$body")
            
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
    local time_ago=$(date -d "$updated" +%s 2>/dev/null || echo "0")
    local now=$(date +%s)
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
    jq -n \
        --arg category "$category" \
        --arg number "$number" \
        --arg title "$title" \
        --arg time "$time_str" \
        --arg critical "$critical_count" \
        --arg minor "$minor_count" \
        '{category: $category, number: $number, title: $title, time: $time, critical: ($critical | tonumber), minor: ($minor | tonumber)}'
}

# Function to display dashboard
display_dashboard() {
    local prs_json=$1
    local interactive=${2:-false}
    
    echo -e "${BLUE}${EMOJI_SEARCH} PR Status Dashboard${NC}"
    echo "===================="
    echo ""
    
    # Categorize PRs
    local needs_review=()
    local critical=()
    local minor=()
    local ready=()
    
    while IFS= read -r pr; do
        local analyzed=$(analyze_pr "$pr")
        local category=$(echo "$analyzed" | jq -r '.category')
        
        case "$category" in
            needs_review)
                needs_review+=("$analyzed")
                ;;
            critical)
                critical+=("$analyzed")
                ;;
            minor)
                minor+=("$analyzed")
                ;;
            ready)
                ready+=("$analyzed")
                ;;
        esac
    done < <(echo "$prs_json" | jq -c '.[]')
    
    # Display categories
    display_category "NEEDS REVIEW" "$EMOJI_REVIEW" "$YELLOW" "${needs_review[@]}"
    display_category "CRITICAL ISSUES" "$EMOJI_CRITICAL" "$RED" "${critical[@]}"
    display_category "MINOR ISSUES" "$EMOJI_MINOR" "$ORANGE" "${minor[@]}"
    display_category "READY TO MERGE" "$EMOJI_READY" "$GREEN" "${ready[@]}"
    
    # Summary
    local total=$((${#needs_review[@]} + ${#critical[@]} + ${#minor[@]} + ${#ready[@]}))
    echo ""
    echo -e "${BLUE}${EMOJI_CHART} Summary:${NC} $total open PRs | ${#needs_review[@]} need review | $((${#critical[@]} + ${#minor[@]})) need fixes | ${#ready[@]} ready"
    echo ""
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
        local number=$(echo "$item" | jq -r '.number')
        local title=$(echo "$item" | jq -r '.title' | cut -c1-50)
        local time=$(echo "$item" | jq -r '.time')
        local critical=$(echo "$item" | jq -r '.critical')
        local minor=$(echo "$item" | jq -r '.minor')
        
        printf "  #%-4s %-50s (last: %s)\n" "$number" "$title" "$time"
        
        if [ "$critical" -gt 0 ]; then
            echo "        → $critical critical comment(s)"
        fi
        if [ "$minor" -gt 0 ]; then
            echo "        → $minor minor comment(s)"
        fi
    done
    echo ""
}

# Function to output in JSON format (for agents)
output_json() {
    local prs_json=$1
    local all_prs=[]
    
    while IFS= read -r pr; do
        local analyzed=$(analyze_pr "$pr")
        all_prs=$(echo "$all_prs" | jq --argjson item "$analyzed" '. + [$item]')
    done < <(echo "$prs_json" | jq -c '.[]')
    
    echo "$all_prs" | jq '{
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

# Function to get next actionable PR
get_next_pr() {
    local prs_json=$1
    local category=${2:-"critical"}
    
    while IFS= read -r pr; do
        local analyzed=$(analyze_pr "$pr")
        local pr_category=$(echo "$analyzed" | jq -r '.category')
        
        if [ "$pr_category" = "$category" ]; then
            echo "$analyzed"
            return 0
        fi
    done < <(echo "$prs_json" | jq -c '.[]')
    
    # No PR found in requested category
    return 1
}

# Function to output simple text summary (for agents)
output_simple() {
    local prs_json=$1
    
    local needs_review=()
    local critical=()
    local minor=()
    local ready=()
    
    while IFS= read -r pr; do
        local analyzed=$(analyze_pr "$pr")
        local category=$(echo "$analyzed" | jq -r '.category')
        
        case "$category" in
            needs_review) needs_review+=("$analyzed") ;;
            critical) critical+=("$analyzed") ;;
            minor) minor+=("$analyzed") ;;
            ready) ready+=("$analyzed") ;;
        esac
    done < <(echo "$prs_json" | jq -c '.[]')
    
    echo "SUMMARY: ${#needs_review[@]} need review, ${#critical[@]} critical, ${#minor[@]} minor, ${#ready[@]} ready"
    
    if [ ${#critical[@]} -gt 0 ]; then
        echo ""
        echo "CRITICAL ISSUES:"
        for item in "${critical[@]}"; do
            local number=$(echo "$item" | jq -r '.number')
            local title=$(echo "$item" | jq -r '.title')
            local crit=$(echo "$item" | jq -r '.critical')
            local min=$(echo "$item" | jq -r '.minor')
            echo "  #$number: $title ($crit critical, $min minor)"
        done
    fi
    
    if [ ${#needs_review[@]} -gt 0 ]; then
        echo ""
        echo "NEEDS REVIEW:"
        for item in "${needs_review[@]}"; do
            local number=$(echo "$item" | jq -r '.number')
            local title=$(echo "$item" | jq -r '.title')
            echo "  #$number: $title"
        done
    fi
    
    if [ ${#ready[@]} -gt 0 ]; then
        echo ""
        echo "READY TO MERGE:"
        for item in "${ready[@]}"; do
            local number=$(echo "$item" | jq -r '.number')
            local title=$(echo "$item" | jq -r '.title')
            echo "  #$number: $title"
        done
    fi
}

# Function to run interactive mode
run_interactive() {
    while true; do
        clear
        local prs=$(fetch_prs)
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
    local mode="dashboard"  # dashboard, json, simple, next-critical, next-minor
    
    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            -i|--interactive)
                interactive=true
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
    
    # Fetch PRs
    local prs=$(fetch_prs)
    
    # Execute based on mode
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
}

main "$@"
