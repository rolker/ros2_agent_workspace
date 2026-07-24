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
#   LOCAL_REVIEW_TIMEOUT  request timeout seconds (default: 900; scale
#                         up for large diffs — reasoning time grows
#                         with diff size, and a ~500-line diff can
#                         exceed 600s on 8GB-VRAM-class hardware)
#   LOCAL_REVIEW_NUM_CTX  context window tokens (default: 32768; the
#                         script fails loud, with the size it needed,
#                         when the prompt would overflow this — Ollama
#                         would otherwise truncate silently. A
#                         post-response check on the server's measured
#                         prompt_eval_count backstops the byte-based
#                         estimate: a run whose ingested tokens reach
#                         the window ceiling fails loud too. The
#                         window must also fit the model's reasoning:
#                         qwen3.5:35b thinks for ~7k tokens on even a
#                         100-line diff, hence the large default and
#                         the 12k-token answer/reasoning headroom)
#
# Exit codes:
#   0  review produced (findings on stdout)
#   1  invocation error (timeout, HTTP error, empty answer, oversized
#      prompt, bad configuration)
#   2  unavailable (server down, model not pulled, or jq missing) —
#      callers should treat this as skip-with-notice, not failure
#   Callers must treat any other exit status as 1 (defensive: an
#   unhandled tool failure under `set -e` can surface its own code).
#
# Reasoning models: requests are sent with think:true so the model can
# reason before answering; if the server rejects that for a
# non-reasoning LOCAL_REVIEW_MODEL, the request is retried once with
# think:false, so non-reasoning models work without configuration.
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
TIMEOUT="${LOCAL_REVIEW_TIMEOUT:-900}"
NUM_CTX="${LOCAL_REVIEW_NUM_CTX:-32768}"

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
            # Print the header comment block (everything from line 2 to
            # the first non-comment line) so the help text cannot drift
            # from the documentation above as lines are added/removed.
            awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "${BASH_SOURCE[0]}"
            exit 0
            ;;
        *)
            echo "Error: unknown argument: $1" >&2
            exit 1
            ;;
    esac
done

if ! [[ "$TIMEOUT" =~ ^[0-9]+$ ]] || ! [[ "$NUM_CTX" =~ ^[0-9]+$ ]]; then
    echo "Error: LOCAL_REVIEW_TIMEOUT and LOCAL_REVIEW_NUM_CTX must be plain integers (got '$TIMEOUT' / '$NUM_CTX')" >&2
    exit 1
fi

# --- Gather the diff ---

if [[ -n "$BASE_BRANCH" ]]; then
    DIFF=$(git diff --merge-base "$BASE_BRANCH" HEAD) || {
        echo "Error: git diff failed for base '$BASE_BRANCH' (not a ref?)" >&2
        exit 1
    }
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

if ! command -v jq >/dev/null 2>&1; then
    echo "local review unavailable: jq not installed" >&2
    exit 2
fi

if ! curl -sf --max-time 5 "$BASE_URL/api/version" >/dev/null 2>&1; then
    echo "local review unavailable: no Ollama server at $BASE_URL" >&2
    exit 2
fi

# Accept both fully-tagged names and the bare form users pass to
# `ollama run` (Ollama registers untagged pulls as "<name>:latest").
if ! curl -sf --max-time 10 "$BASE_URL/api/tags" 2>/dev/null \
        | jq -e --arg m "$MODEL" \
            '.models[] | select(.name == $m or .name == ($m + ":latest"))' >/dev/null; then
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

# Fail loud on context overflow — Ollama silently truncates oversized
# prompts (dropping the instructions and the head of the diff), which
# would produce a confident review of a tail fragment. ~3 bytes/token
# is a typical ratio for diff text, not a guaranteed bound — dense
# tokenization (byte-level BPE fallback on unusual bytes) can beat it,
# so a post-response check on the server's measured prompt_eval_count
# backstops this estimate. The headroom must cover reasoning +
# answer: a thinking model that runs out of window mid-reasoning
# returns an empty answer with done_reason=length (observed at 2048
# headroom on a ~500-line diff).
EST_TOKENS=$(( ${#PROMPT} / 3 + 12288 ))
if (( EST_TOKENS > NUM_CTX )); then
    echo "Error: prompt (~$EST_TOKENS tokens incl. answer headroom) exceeds num_ctx=$NUM_CTX;" >&2
    echo "  re-run with LOCAL_REVIEW_NUM_CTX=$EST_TOKENS (needs RAM) or review a smaller diff" >&2
    exit 1
fi

# --- Invoke ---

PROMPT_FILE=$(mktemp /tmp/local_review_prompt.XXXXXX)
BODY_FILE=$(mktemp /tmp/local_review_body.XXXXXX)
RESPONSE_FILE=$(mktemp /tmp/local_review_resp.XXXXXX)
trap 'rm -f "$PROMPT_FILE" "$BODY_FILE" "$RESPONSE_FILE"' EXIT

# Prompt travels via file + --rawfile: as an --arg it would be a single
# execve argument, capped at ~128 KB on Linux (MAX_ARG_STRLEN) — large
# diffs would fail before ever reaching the model.
printf '%s' "$PROMPT" > "$PROMPT_FILE"

build_body() {  # $1 = "true" | "false" (think flag)
    jq -n \
        --arg model "$MODEL" \
        --rawfile prompt "$PROMPT_FILE" \
        --argjson num_ctx "$NUM_CTX" \
        --argjson think "$1" \
        '{model: $model,
          messages: [{role: "user", content: $prompt}],
          stream: false, think: $think,
          options: {num_ctx: $num_ctx}}' > "$BODY_FILE"
}

send_request() {
    # curl runs alone here (jq already wrote $BODY_FILE) so a failure
    # is attributable: exit 28 = timeout, anything else = transport.
    CURL_EXIT=0
    HTTP_CODE=$(curl -s --max-time "$TIMEOUT" \
        -o "$RESPONSE_FILE" -w '%{http_code}' \
        -H 'Content-Type: application/json' \
        -d @"$BODY_FILE" "$BASE_URL/api/chat") || CURL_EXIT=$?
    if [[ "$CURL_EXIT" == "28" ]]; then
        echo "local review failed: request timed out (${TIMEOUT}s limit; LOCAL_REVIEW_TIMEOUT to raise)" >&2
        exit 1
    elif [[ "$CURL_EXIT" != "0" ]]; then
        echo "local review failed: connection to $BASE_URL failed (curl exit $CURL_EXIT)" >&2
        exit 1
    fi
}

build_body true
send_request

# Non-reasoning models reject think:true with a 400; retry once without.
if [[ "$HTTP_CODE" == "400" ]] \
        && grep -qi "does not support thinking" "$RESPONSE_FILE"; then
    build_body false
    send_request
fi

if [[ "$HTTP_CODE" != "200" ]]; then
    echo "local review failed: HTTP $HTTP_CODE: $(head -c 200 "$RESPONSE_FILE")" >&2
    exit 1
fi

CONTENT=$(jq -r '.message.content // empty' "$RESPONSE_FILE" 2>/dev/null) || {
    echo "local review failed: non-JSON response from $BASE_URL (proxy in the way?)" >&2
    exit 1
}

# Silent-truncation guard: the pre-flight size check estimates tokens
# from bytes, and dense tokenization can beat the estimate — the server
# then silently drops the head of the prompt (the instructions) and the
# model confidently reviews a tail fragment. Truncation is detectable
# post-hoc from the tokens the server actually ingested: the pre-check
# guarantees an untruncated prompt leaves >= 12288 tokens of window
# free, so an ingested count at the ceiling can only mean the prompt
# was cut down to fit (or the estimate undershot so badly the answer
# is untrustworthy anyway). Caveat: server-side prompt caching may
# report fewer ingested tokens than the full prompt, so this check can
# miss — it is a backstop for the pre-check, not a replacement.
PROMPT_EVAL=$(jq -r '.prompt_eval_count // 0' "$RESPONSE_FILE" 2>/dev/null || echo 0)
if (( PROMPT_EVAL >= NUM_CTX - 1024 )); then
    echo "local review failed: prompt filled the context window (server ingested $PROMPT_EVAL of num_ctx=$NUM_CTX tokens) — input was likely silently truncated;" >&2
    echo "  raise LOCAL_REVIEW_NUM_CTX (needs RAM) or review a smaller diff" >&2
    exit 1
fi

if [[ -z "$CONTENT" ]]; then
    DONE_REASON=$(jq -r '.done_reason // "unknown"' "$RESPONSE_FILE" 2>/dev/null || echo "unparseable")
    if [[ "$DONE_REASON" == "length" ]]; then
        echo "local review failed: reasoning consumed the context window before an answer was produced (done_reason: length); raise LOCAL_REVIEW_NUM_CTX (current: $NUM_CTX)" >&2
    else
        echo "local review failed: model returned an empty answer (done_reason: $DONE_REASON)" >&2
    fi
    exit 1
fi

echo "## Local Adversarial ($MODEL)"
echo
echo "$CONTENT"
