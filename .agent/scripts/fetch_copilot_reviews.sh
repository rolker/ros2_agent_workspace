#!/bin/bash
# Fetch Copilot Review Comments
# Retrieves GitHub Copilot review comments on a PR that were submitted after
# the HEAD commit's timestamp. Outputs structured JSON to stdout.
#
# Usage: fetch_copilot_reviews.sh --pr <number>
#
# Designed to consolidate multiple gh api calls into a single script invocation,
# reducing permission prompts when called from agent skills.
#
# Limitations:
# - Uses committer date (not author date) as the cutoff. Rebases can make these
#   diverge; committer date reflects the most recent action on the branch.
# - If the PR was force-pushed after Copilot reviewed, HEAD may not match what
#   Copilot reviewed. Comments may reference stale line numbers.

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
            echo "Fetch Copilot review comments submitted after the HEAD commit."
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

# --- Get HEAD commit timestamp ---

# Use committer date (ISO 8601) — reflects most recent action on the branch.
HEAD_TIMESTAMP="$(git log -1 --format='%cI' 2>/dev/null || true)"

# Normalize to UTC so comparisons work regardless of local timezone
if [ -n "$HEAD_TIMESTAMP" ]; then
    HEAD_TIMESTAMP="$(date -u -d "$HEAD_TIMESTAMP" '+%Y-%m-%dT%H:%M:%SZ')"
fi

if [ -z "$HEAD_TIMESTAMP" ]; then
    echo "Error: No commits found on current branch" >&2
    exit 1
fi

# Log to stderr so it doesn't pollute JSON output
echo "ℹ️  PR: #${PR_NUMBER} in ${REPO_SLUG}" >&2
echo "ℹ️  HEAD timestamp (committer date): ${HEAD_TIMESTAMP}" >&2

# --- Fetch all reviews on the PR ---

if ! ALL_REVIEWS="$(gh api --paginate "repos/${REPO_SLUG}/pulls/${PR_NUMBER}/reviews")"; then
    echo "Error: Failed to fetch reviews for PR #${PR_NUMBER}" >&2
    exit 1
fi

# --- Filter to Copilot reviews submitted after HEAD ---

# Copilot's login is "github-actions[bot]" or "copilot" depending on integration.
# We filter by user.login containing "copilot" (case-insensitive) or the app slug.
# Also include "github-actions[bot]" reviews that have COMMENTED state (common for Copilot).
#
# Compare timestamps by converting to epoch seconds for reliable cross-timezone comparison.

FILTERED_REVIEWS="$(echo "$ALL_REVIEWS" | jq -c --arg cutoff "$HEAD_TIMESTAMP" '
    def to_epoch: sub("\\.[0-9]+"; "") | strptime("%Y-%m-%dT%H:%M:%S%Z") | mktime;
    ($cutoff | to_epoch) as $cutoff_epoch |
    [.[] | select(
        ((.submitted_at | to_epoch) > $cutoff_epoch) and
        (
            (.user.login | test("copilot"; "i")) or
            (.user.login == "github-actions[bot]" and .state == "COMMENTED")
        )
    ) | {review_id: .id, submitted_at: .submitted_at, state: .state, body: .body}]
')"

REVIEW_COUNT="$(echo "$FILTERED_REVIEWS" | jq 'length')"
echo "ℹ️  Found ${REVIEW_COUNT} Copilot review(s) after HEAD" >&2

if [ "$REVIEW_COUNT" -eq 0 ]; then
    # Output empty result
    jq -n \
        --arg pr "$PR_NUMBER" \
        --arg repo "$REPO_SLUG" \
        --arg head_timestamp "$HEAD_TIMESTAMP" \
        '{
            pr: ($pr | tonumber),
            repo: $repo,
            head_timestamp: $head_timestamp,
            reviews_after_head: []
        }'
    exit 0
fi

# --- Fetch comments for each matching review ---

# Fetch all review comments on the PR (paginated)
if ! ALL_COMMENTS="$(gh api --paginate "repos/${REPO_SLUG}/pulls/${PR_NUMBER}/comments")"; then
    echo "Error: Failed to fetch review comments for PR #${PR_NUMBER}" >&2
    exit 1
fi

# Build the final output by matching comments to their review IDs
OUTPUT="$(echo "$FILTERED_REVIEWS" | jq -c --argjson comments "$ALL_COMMENTS" --arg pr "$PR_NUMBER" --arg repo "$REPO_SLUG" --arg head_timestamp "$HEAD_TIMESTAMP" '
    {
        pr: ($pr | tonumber),
        repo: $repo,
        head_timestamp: $head_timestamp,
        reviews_after_head: [
            .[] | . as $review | {
                review_id: .review_id,
                submitted_at: .submitted_at,
                state: .state,
                body: .body,
                comments: [
                    $comments[] | select(.pull_request_review_id == $review.review_id) | {
                        path: .path,
                        line: (.line // .original_line // null),
                        side: (.side // null),
                        body: .body,
                        diff_hunk: .diff_hunk
                    }
                ]
            }
        ]
    }
')"

TOTAL_COMMENTS="$(echo "$OUTPUT" | jq '[.reviews_after_head[].comments | length] | add // 0')"
echo "ℹ️  Total comments across reviews: ${TOTAL_COMMENTS}" >&2

echo "$OUTPUT"
