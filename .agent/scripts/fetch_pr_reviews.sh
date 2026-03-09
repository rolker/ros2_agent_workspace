#!/bin/bash
# Fetch PR Reviews and CI Status
# Retrieves all reviews and CI check status for a PR.
# Outputs structured JSON to stdout.
#
# Usage: fetch_pr_reviews.sh --pr <number>
#
# Designed to consolidate multiple gh api calls into a single script invocation,
# reducing permission prompts when called from agent skills.

if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
    echo "Error: This script should be executed, not sourced." >&2
    echo "  Run: ${BASH_SOURCE[0]} $*" >&2
    return 1
fi

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export REPO_ROOT
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# --- Argument parsing ---

PR_NUMBER=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --pr)
            if [[ -z "${2:-}" ]]; then
                echo "Error: --pr requires a PR number" >&2
                echo "Usage: $0 --pr <number>" >&2
                exit 1
            fi
            if ! [[ "$2" =~ ^[0-9]+$ ]]; then
                echo "Error: --pr value must be a positive integer, got: '$2'" >&2
                exit 1
            fi
            PR_NUMBER="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 --pr <number>"
            echo ""
            echo "Fetch all PR reviews and CI check status."
            echo "Outputs structured JSON to stdout."
            echo ""
            echo "Options:"
            echo "  --pr <number>   PR number (required)"
            echo "  -h, --help      Show this help message"
            exit 0
            ;;
        *)
            echo "Error: Unknown option: $1" >&2
            echo "Usage: $0 --pr <number>" >&2
            exit 1
            ;;
    esac
done

if [ -z "$PR_NUMBER" ]; then
    echo "Error: --pr <number> is required" >&2
    exit 1
fi

# --- Get repo slug ---

REPO_SLUG="$(gh repo view --json nameWithOwner --jq '.nameWithOwner')"

# --- Get PR head SHA ---

HEAD_SHA="$(gh pr view "$PR_NUMBER" --json headRefOid --jq '.headRefOid')"

if [ -z "$HEAD_SHA" ]; then
    echo "Error: Could not determine head SHA for PR #${PR_NUMBER}" >&2
    exit 1
fi

# Log to stderr so it doesn't pollute JSON output
echo "PR: #${PR_NUMBER} in ${REPO_SLUG}" >&2
echo "HEAD SHA: ${HEAD_SHA}" >&2

# --- Fetch all reviews on the PR ---

if ! ALL_REVIEWS="$(gh api --paginate "repos/${REPO_SLUG}/pulls/${PR_NUMBER}/reviews")"; then
    echo "Error: Failed to fetch reviews for PR #${PR_NUMBER}" >&2
    exit 1
fi

# Extract all reviews with commit_id for timeline reasoning
# --paginate returns concatenated JSON arrays; merge them before processing
REVIEWS="$(echo "$ALL_REVIEWS" | jq -s 'add' | jq -c '
    [.[] | {
        review_id: .id,
        submitted_at: .submitted_at,
        state: .state,
        body: .body,
        commit_id: .commit_id,
        user_login: .user.login,
        user_type: .user.type
    }]
')"

REVIEW_COUNT="$(echo "$REVIEWS" | jq 'length')"
echo "Found ${REVIEW_COUNT} review(s)" >&2

# --- Fetch all review comments ---

if ! ALL_COMMENTS="$(gh api --paginate "repos/${REPO_SLUG}/pulls/${PR_NUMBER}/comments")"; then
    echo "Error: Failed to fetch review comments for PR #${PR_NUMBER}" >&2
    exit 1
fi

# --- Fetch conversation comments (issue-level comments on the PR) ---

if ! ALL_CONVERSATION="$(gh api --paginate "repos/${REPO_SLUG}/issues/${PR_NUMBER}/comments")"; then
    echo "Warning: Failed to fetch conversation comments for PR #${PR_NUMBER}" >&2
    ALL_CONVERSATION="[]"
fi

CONVERSATION_COMMENTS="$(echo "$ALL_CONVERSATION" | jq -s 'add' | jq -c '
    [.[] | {
        comment_id: .id,
        created_at: .created_at,
        body: .body,
        user_login: .user.login,
        user_type: .user.type,
        html_url: .html_url
    }]
')"

echo "Found $(echo "$CONVERSATION_COMMENTS" | jq 'length') conversation comment(s)" >&2

# --- Fetch CI check-runs ---

if ! CI_CHECKS_RAW="$(gh api --paginate "repos/${REPO_SLUG}/commits/${HEAD_SHA}/check-runs")"; then
    echo "Warning: Failed to fetch CI check-runs for PR #${PR_NUMBER}" >&2
    CI_CHECKS="[]"
else
    CI_CHECKS="$(echo "$CI_CHECKS_RAW" | jq -s '[.[].check_runs[] | {name, conclusion, html_url}]')"
fi

echo "Found $(echo "$CI_CHECKS" | jq 'length') CI check(s)" >&2

# --- Assemble JSON output ---

# Use --slurpfile instead of --argjson to avoid "Argument list too long" on large responses
COMMENTS_TMPFILE="$(mktemp /tmp/pr_review_comments.XXXXXX.json)"
# --paginate returns concatenated JSON arrays; merge them before writing
echo "$ALL_COMMENTS" | jq -s 'add' > "$COMMENTS_TMPFILE"

OUTPUT="$(echo "$REVIEWS" | jq -c --slurpfile comments "$COMMENTS_TMPFILE" \
    --arg pr "$PR_NUMBER" \
    --arg repo "$REPO_SLUG" \
    --arg head_sha "$HEAD_SHA" \
    --argjson ci_checks "$CI_CHECKS" \
    --argjson conversation_comments "$CONVERSATION_COMMENTS" '
    {
        pr: ($pr | tonumber),
        repo: $repo,
        head_sha: $head_sha,
        reviews: [
            .[] | . as $review | {
                review_id: .review_id,
                submitted_at: .submitted_at,
                state: .state,
                body: .body,
                commit_id: .commit_id,
                user_login: .user_login,
                user_type: .user_type,
                comments: [
                    $comments[0][] | select(.pull_request_review_id == $review.review_id) | {
                        path: .path,
                        line: (.line // .original_line // null),
                        side: (.side // null),
                        body: .body,
                        diff_hunk: .diff_hunk,
                        user_login: .user.login,
                        user_type: .user.type
                    }
                ]
            }
        ],
        ci_checks: $ci_checks,
        conversation_comments: $conversation_comments
    }
')"

rm -f "$COMMENTS_TMPFILE"

TOTAL_COMMENTS="$(echo "$OUTPUT" | jq '[.reviews[].comments | length] | add // 0')"
echo "Total comments across reviews: ${TOTAL_COMMENTS}" >&2

echo "$OUTPUT"
