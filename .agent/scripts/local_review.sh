#!/bin/bash
# Local Model Adversarial Review
# Sends a diff to a locally served Ollama model and prints its review
# findings to stdout. Used by the review-code skill (specialist 5f) and
# usable standalone — including offline / field mode, where the
# GitHub- and Claude-based review pipeline is unavailable.
#
# Usage:
#   git diff main...HEAD | local_review.sh            # diff on stdin
#   local_review.sh --base main                        # diff computed here
#   local_review.sh --base main --context ctx.txt      # extra task context
#
# Environment:
#   LOCAL_REVIEW_MODEL    model tag (default: qwen3.5:35b)
#   LOCAL_REVIEW_URL      Ollama base URL (default: http://localhost:11434)
#   LOCAL_REVIEW_TIMEOUT  request timeout seconds (default: 600)
#   LOCAL_REVIEW_NUM_CTX  context window tokens (default: 16384)
#
# Exit codes:
#   0  review produced (findings on stdout)
#   1  invocation error (timeout, HTTP error, empty answer)
#   2  unavailable (server down or model not pulled) — callers should
#      treat this as skip-with-notice, not failure
#
# Uses the HTTP API rather than `ollama run`: with reasoning models the
# CLI's thinking handling can swallow the entire answer (observed with
# qwen3.5:35b + --hidethinking on ollama 0.32.0), while /api/chat
# returns thinking and content as separate fields.

if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
    echo "Error: This script should be executed, not sourced." >&2
    echo "  Run: ${BASH_SOURCE[0]} $*" >&2
    return 1
fi

set -euo pipefail

MODEL="${LOCAL_REVIEW_MODEL:-qwen3.5:35b}"
BASE_URL="${LOCAL_REVIEW_URL:-http://localhost:11434}"
TIMEOUT="${LOCAL_REVIEW_TIMEOUT:-600}"
NUM_CTX="${LOCAL_REVIEW_NUM_CTX:-16384}"

BASE_BRANCH=""
CONTEXT_FILE=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --base)
            if [[ -z "${2:-}" ]]; then
                echo "Error: --base requires a branch name" >&2
                exit 1
            fi
            BASE_BRANCH="$2"
            shift 2
            ;;
        --context)
            if [[ -z "${2:-}" || ! -r "${2:-}" ]]; then
                echo "Error: --context requires a readable file" >&2
                exit 1
            fi
            CONTEXT_FILE="$2"
            shift 2
            ;;
        --model)
            if [[ -z "${2:-}" ]]; then
                echo "Error: --model requires a model tag" >&2
                exit 1
            fi
            MODEL="$2"
            shift 2
            ;;
        -h|--help)
            sed -n '2,28p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        *)
            echo "Error: unknown argument: $1" >&2
            exit 1
            ;;
    esac
done

# --- Gather the diff ---

if [[ -n "$BASE_BRANCH" ]]; then
    DIFF=$(git diff --merge-base "$BASE_BRANCH" HEAD)
elif [[ ! -t 0 ]]; then
    DIFF=$(cat)
else
    echo "Error: pipe a diff on stdin or pass --base <branch>" >&2
    exit 1
fi

if [[ -z "$DIFF" ]]; then
    echo "Error: diff is empty — nothing to review" >&2
    exit 1
fi

# --- Availability probes (exit 2 = skip-with-notice) ---

if ! curl -sf --max-time 5 "$BASE_URL/api/version" >/dev/null 2>&1; then
    echo "local review unavailable: no Ollama server at $BASE_URL" >&2
    exit 2
fi

if ! curl -sf --max-time 10 "$BASE_URL/api/tags" 2>/dev/null \
        | jq -e --arg m "$MODEL" '.models[] | select(.name == $m)' >/dev/null; then
    echo "local review unavailable: model '$MODEL' not pulled (ollama pull $MODEL)" >&2
    exit 2
fi

# --- Build the prompt ---

CONTEXT=""
if [[ -n "$CONTEXT_FILE" ]]; then
    CONTEXT=$(cat "$CONTEXT_FILE")
fi

PROMPT=$(cat <<EOF
You are a rigorous code reviewer for a ROS 2 robotics workspace
(autonomous survey boats — robustness is mandatory, silent failures are
unacceptable). Review the following diff.

${CONTEXT:+Task context: $CONTEXT

}Focus areas: missed edge cases and boundary conditions; assumption
violations; subtle bugs (off-by-one, race conditions, resource leaks);
logic errors (does the code do what it claims?); security implications;
concurrency and lifecycle ordering; cross-cutting effects; portability
(GNU vs BSD tools); silent-failure paths; misleading comments.

Rules: review only what the diff changes (lines starting with + or -);
do not flag unchanged context lines. Only report defects you can trace
to specific changed lines — no speculation about environments or inputs
the code visibly does not target. Number the findings. For each give:
file, line(s), why it is wrong, and a concrete failure scenario. Do not
pad with style nits. If you find nothing, say "No findings."

DIFF:

$DIFF
EOF
)

# --- Invoke ---

RESPONSE_FILE=$(mktemp /tmp/local_review.XXXXXX.json)
trap 'rm -f "$RESPONSE_FILE"' EXIT

HTTP_CODE=$(jq -n \
        --arg model "$MODEL" \
        --arg prompt "$PROMPT" \
        --argjson num_ctx "$NUM_CTX" \
        '{model: $model,
          messages: [{role: "user", content: $prompt}],
          stream: false, think: true,
          options: {num_ctx: $num_ctx}}' \
    | curl -s --max-time "$TIMEOUT" \
        -o "$RESPONSE_FILE" -w '%{http_code}' \
        -H 'Content-Type: application/json' \
        -d @- "$BASE_URL/api/chat") || {
    echo "local review failed: request timed out or connection lost (${TIMEOUT}s limit)" >&2
    exit 1
}

if [[ "$HTTP_CODE" != "200" ]]; then
    echo "local review failed: HTTP $HTTP_CODE: $(head -c 200 "$RESPONSE_FILE")" >&2
    exit 1
fi

CONTENT=$(jq -r '.message.content // empty' "$RESPONSE_FILE")

if [[ -z "$CONTENT" ]]; then
    echo "local review failed: model returned an empty answer (done_reason: $(jq -r '.done_reason // "unknown"' "$RESPONSE_FILE"))" >&2
    exit 1
fi

echo "## Local Adversarial ($MODEL)"
echo
echo "$CONTENT"
