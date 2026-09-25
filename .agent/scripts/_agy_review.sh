#!/usr/bin/env bash
# Run one headless agy (Gemini CLI successor) review turn and validate it.
#
# Usage:
#   _agy_review.sh <agy-bin> <prompt-file> <findings-file> [<print-timeout>]
#
# Called by cross_model_review.sh for the "gemini" agent (in its own
# background job alongside the other agents, ADR-0015), so there is one
# invocation and one success test.
#
# Why this exists (issues #274, #288):
#   * agy print mode takes the prompt as the -p argument value; large PR
#     prompts exceed the kernel's per-argument limit (~128 KiB) and fail
#     with "Argument list too long". Its stream-json input mode reads the
#     prompt from stdin instead, one NDJSON message per line.
#   * When the model calls a tool that headless mode cannot approve (any
#     shell command), agy auto-denies it, prints a notice on stderr, and
#     exits 0 with an EMPTY response. The exit code is therefore not a
#     usable failure signal; the terminal `result` event is.
#     When that happens this helper resumes the same conversation ONCE
#     (--conversation <id>) with a "that was denied, answer in text only"
#     message, inside the same print-timeout budget (#660).
#   * When --print-timeout expires mid-turn agy also exits 0 with
#     status SUCCESS and a partial (possibly empty) response; the only
#     signal is a stderr line containing "print timeout after".
#
# Contract:
#   * This script OWNS the findings file: it is truncated first thing, and
#     receives either the review text or a failure reason. The caller must
#     not redirect stdout onto it. The caller appends the
#     "--- Review complete ---" / "--- Review failed ---" marker.
#   * Exit 0 only when agy exited 0, emitted a result event with
#     status SUCCESS and a non-empty response, and did not time out.
#     Exit 1 otherwise (findings file holds the reason). Exit 2 on usage
#     errors (also recorded in the findings file when it is writable).
#   * The caller wraps this helper in an outer `timeout` backstop set
#     ABOVE <print-timeout> (cross_model_review.sh: GEMINI_BACKSTOP). The
#     print-timeout handling here stays the primary path — the backstop
#     only fires if this helper never returns at all, so it cannot race
#     the timeout-then-partial-response contract above.
#   * No temp files survive any exit path this script can observe: the
#     EXIT trap covers normal exits and the signal traps turn a kill into
#     an exit so it still fires — after waiting for agy to die (with a
#     bounded escalation to SIGKILL), so the temp dir is never removed
#     under a running agy. SIGKILL is the exception — no trap runs,
#     so the `agy-review.XXXXXX` dir would be left behind. The only sender
#     is `timeout -k` on the caller's backstop (a wedged helper), and
#     cross_model_review.sh closes that gap by pointing TMPDIR at a
#     scratch root it owns and removes itself.
#   * All diagnostics go to stderr; stdout is unused.
#
# Verified against agy 1.2.8 (2026-09-22): see the plan for issue #288.

set -uo pipefail

usage() {
    echo "Usage: $0 <agy-bin> <prompt-file> <findings-file> [<print-timeout>]" >&2
}

if [[ $# -lt 3 || $# -gt 4 ]]; then
    usage
    exit 2
fi

AGY_BIN="$1"
PROMPT_FILE="$2"
FINDINGS_FILE="$3"
PRINT_TIMEOUT="${4:-30m}"

# Truncate the findings file before anything else can fail. Without this a
# guard failure would leave the previous run's review in place under a
# fresh "--- Review failed ---" marker, and review-code would read stale
# findings as current (the #288 failure class in a different coat).
if ! : > "$FINDINGS_FILE"; then
    echo "ERROR: cannot write findings file: ${FINDINGS_FILE}" >&2
    exit 2
fi

# Write a failure reason into the findings file and exit 1.
fail() {
    local reason="$1"
    {
        echo "agy review did not produce a usable result."
        echo ""
        echo "Reason: ${reason}"
    } > "$FINDINGS_FILE"
    echo "ERROR: agy review failed: ${reason}" >&2
    exit 1
}

# Configuration error: recorded like a failure, but exit 2 so it reads as
# "this run was never startable" rather than "agy misbehaved".
config_fail() {
    local reason="$1"
    {
        echo "agy review could not be started."
        echo ""
        echo "Reason: ${reason}"
    } > "$FINDINGS_FILE"
    echo "ERROR: ${reason}" >&2
    exit 2
}

if ! command -v jq >/dev/null 2>&1; then
    fail "jq is required to build the stream-json prompt and parse agy's result event (see bootstrap.sh)"
fi
if [[ ! -r "$PROMPT_FILE" ]]; then
    fail "prompt file not readable: ${PROMPT_FILE}"
fi
# Accept a bare name (resolved on PATH) or a path; `test -x` alone only
# looks at the current directory for a bare name.
AGY_BIN_RESOLVED=$(command -v "$AGY_BIN" 2>/dev/null || true)
if [[ -z "$AGY_BIN_RESOLVED" || ! -x "$AGY_BIN_RESOLVED" ]]; then
    fail "agy binary not found or not executable: ${AGY_BIN}"
fi

# This helper's SIGKILL escalation must fit inside the caller's
# `timeout -k` grace, or the caller kills this helper first and an agy
# that ignored SIGTERM is orphaned (#313 round 2). The caller passes its
# AGENT_KILL_AFTER in the environment; absent it, there is no outer
# grace to fit inside and nothing to check.
REVIEW_KILL_ESCALATION="${REVIEW_KILL_ESCALATION:-5}"
to_seconds() {
    [[ "$1" =~ ^([0-9]+(\.[0-9]+)?)([smhd]?)$ ]] || return 1
    local number="${BASH_REMATCH[1]}" unit="${BASH_REMATCH[3]}" mult=1
    case "$unit" in m) mult=60 ;; h) mult=3600 ;; d) mult=86400 ;; *) mult=1 ;; esac
    awk -v n="$number" -v m="$mult" 'BEGIN { printf "%.0f", n * m }'
}
if ! ESCALATION_SECONDS=$(to_seconds "$REVIEW_KILL_ESCALATION"); then
    config_fail "REVIEW_KILL_ESCALATION value '${REVIEW_KILL_ESCALATION}' is not a duration (a number of seconds, optionally with an s/m/h suffix)"
fi
if [[ -n "${AGENT_KILL_AFTER:-}" ]]; then
    if ! KILL_AFTER_SECONDS=$(to_seconds "$AGENT_KILL_AFTER"); then
        config_fail "AGENT_KILL_AFTER value '${AGENT_KILL_AFTER}' is not a duration"
    fi
    # Strictly greater, which also rules out 0: `timeout -k 0` disables
    # the caller's SIGKILL instead of sending it at once.
    if [[ "$KILL_AFTER_SECONDS" -le "$ESCALATION_SECONDS" ]]; then
        config_fail "AGENT_KILL_AFTER (${AGENT_KILL_AFTER}) must be greater than REVIEW_KILL_ESCALATION (${REVIEW_KILL_ESCALATION}): the caller's SIGKILL would land on this helper before it could SIGKILL an agy that ignored SIGTERM, orphaning it. Raise AGENT_KILL_AFTER or lower REVIEW_KILL_ESCALATION."
    fi
fi

# Temp files: the NDJSON input line, agy's stdout (event stream), and agy's
# stderr. Removed on every exit path — the EXIT trap covers normal exits
# and the signal traps turn a kill into an exit so it still fires; on
# failure the useful parts are copied into the findings file first.
TMP_DIR=$(mktemp -d -t agy-review.XXXXXX) || fail "mktemp failed"
trap 'rm -rf "$TMP_DIR"' EXIT
# Armed before agy is launched (AGY_PID empty until then) so a signal in
# the launch window cannot leave agy running behind an exited helper.
# TERM is the live path under cross_model_review.sh: this helper runs as a
# background child of a non-interactive shell, where bash makes SIGINT
# ignored (and an ignored signal cannot be trapped). The INT trap is for a
# direct interactive invocation of this script, where Ctrl-C does arrive.
#
# The handler waits for agy to actually die, escalating to SIGKILL after
# REVIEW_KILL_ESCALATION seconds (#313): exiting straight after the
# `kill` would let the EXIT trap remove TMP_DIR under an agy still
# writing into it, and would defeat the caller's `timeout -k` backstop —
# that SIGKILL is aimed at this helper, so once we are gone an agy that
# ignored SIGTERM keeps running. The escalation is validated above to fit
# inside the caller's grace (AGENT_KILL_AFTER).
# The agy runs as the leader of its own process group (setsid), and every
# signal below goes to that whole group. Signalling only its PID missed a
# child it had started that ignored SIGTERM: the agy died, this helper
# exited, the caller then saw a finished job and never reached its
# kill_tree, and the child ran on (#660). In a non-interactive shell a
# background job is not a group leader, so setsid execs in place and the
# PID from `$!` is the group id. Where setsid is missing (macOS) the
# signals fall back to the PID alone.
if command -v setsid >/dev/null 2>&1; then
    AGY_SETSID=(setsid)
else
    AGY_SETSID=()
fi
signal_agy() {
    if [[ ${#AGY_SETSID[@]} -gt 0 ]]; then
        kill -"$1" -- -"$2" 2>/dev/null
    else
        kill -"$1" "$2" 2>/dev/null
    fi
}
AGY_PID=""
AGY_LAUNCHING=false
AGY_LAUNCH_PREV=""
terminate_child() {
    local code="$1" watchdog i
    # Re-entrancy: a second signal would otherwise start a second
    # watchdog and clobber $watchdog, leaking the first one.
    trap '' INT TERM HUP
    # A signal can land between the `&` that starts the agy and the
    # `AGY_PID=$!` that records it: the agy then exists but is unrecorded,
    # and would outlive this helper. `$!` names it iff it moved since the
    # launch began (#660).
    if [[ -z "$AGY_PID" && "$AGY_LAUNCHING" == true && "${!:-}" != "$AGY_LAUNCH_PREV" ]]; then
        AGY_PID=$!
    fi
    if [[ -n "$AGY_PID" ]]; then
        signal_agy TERM "$AGY_PID"
        # `wait` returns the moment the agy dies, so a clean shutdown
        # costs milliseconds, not the escalation window. The watchdog
        # only matters for a group member that ignores SIGTERM. It is NOT
        # waited on: a subshell sleeping in `sleep` defers the TERM we
        # send it until that sleep ends, so waiting would reintroduce the
        # full window on every clean exit.
        # The watchdog is cancelled with SIGKILL, never TERM: it is forked
        # while this handler has INT/TERM/HUP ignored, an ignored
        # disposition is inherited (by its `sleep` too), and a TERM that
        # lands before any reset inside it is lost. A lost cancel let it
        # outlive every clean shutdown by the whole escalation window and
        # then `kill -9` whatever process had been given the dead CLI's
        # PID (`kill -0` checks a PID, not identity). Its `sleep` is
        # killed first, while still findable under it, so no orphan
        # `sleep` is left either. rolker/ros2_agent_workspace #660.
        ( sleep "$REVIEW_KILL_ESCALATION"
          signal_agy 0 "$AGY_PID" && signal_agy KILL "$AGY_PID" ) &
        watchdog=$!
        wait "$AGY_PID" 2>/dev/null
        # The agy is gone, but a child of it that ignored the TERM may
        # not be: give the group the rest of the window, then SIGKILL it.
        # Group mode only — without setsid this would signal a bare PID
        # that was just reaped and may already be reused.
        if [[ ${#AGY_SETSID[@]} -gt 0 ]]; then
            for ((i = 0; i < ESCALATION_SECONDS * 10; i++)); do
                signal_agy 0 "$AGY_PID" || break
                sleep 0.1
            done
            signal_agy KILL "$AGY_PID"
        fi
        pkill -KILL -P "$watchdog" 2>/dev/null
        kill -KILL "$watchdog" 2>/dev/null
        AGY_PID=""
    fi
    exit "$code"
}
trap 'terminate_child 130' INT
trap 'terminate_child 143' TERM HUP
INPUT_FILE="${TMP_DIR}/input.ndjson"
STREAM_FILE="${TMP_DIR}/stream.ndjson"
STDERR_FILE="${TMP_DIR}/stderr.txt"

# Exactly one NDJSON line: jq -Rs slurps the whole prompt as a single
# string and escapes it (size bounded only by memory); -c keeps the
# message on one line, which is the stream-json contract.
if ! jq -c -Rs '{event: "user", message: {role: "user", content: .}}' "$PROMPT_FILE" > "$INPUT_FILE"; then
    fail "could not encode the prompt as a stream-json message"
fi

# One agy turn. Args: <ndjson-input> <stream-out> <stderr-out>
# <print-timeout> [extra agy args, e.g. --conversation <id>]. Sets AGY_EXIT.
#
# No pipeline here: the exit status is agy's own, not jq's. `-p=` is the
# verified spelling for "print mode, prompt comes from stdin" on agy 1.2.8:
# a bare -p swallows the next flag as its prompt and `-p ""` is rejected
# as an empty prompt.
# agy runs as a background child and is waited on, so a TERM/INT sent
# to this helper (cross_model_review.sh's cleanup on interrupt) reaches
# agy at once instead of being deferred until the turn ends on its own.
agy_turn() {
    local input="$1" out="$2" err="$3" timeout="$4"
    shift 4
    AGY_LAUNCH_PREV="${!:-}"
    AGY_LAUNCHING=true
    "${AGY_SETSID[@]}" "$AGY_BIN_RESOLVED" "$@" \
        --input-format=stream-json \
        --output-format=stream-json \
        --print-timeout "$timeout" \
        --disable-slash-commands \
        -p= < "$input" > "$out" 2> "$err" &
    AGY_PID=$!
    AGY_LAUNCHING=false
    AGY_EXIT=0
    wait "$AGY_PID" || AGY_EXIT=$?
    # The turn is over: nothing agy started may outlive it. Group mode
    # only: a bare PID was just reaped and may already be reused.
    [[ ${#AGY_SETSID[@]} -gt 0 ]] && signal_agy KILL "$AGY_PID"
    AGY_PID=""
}

# The print timeout is this helper's TOTAL budget, retry included, so the
# caller's backstop (set above it) still never races a live turn.
if ! PRINT_TIMEOUT_SECONDS=$(to_seconds "$PRINT_TIMEOUT"); then
    config_fail "print timeout '${PRINT_TIMEOUT}' is not a duration"
fi
TURN_STARTED=$(date +%s)
agy_turn "$INPUT_FILE" "$STREAM_FILE" "$STDERR_FILE" "$PRINT_TIMEOUT"

# Last 20 lines of stderr, for failure reports: a fatal error lands at
# the end, after any startup chatter.
stderr_excerpt() {
    if [[ -s "$STDERR_FILE" ]]; then
        printf '\nagy stderr (last 20 lines):\n'
        tail -n 20 "$STDERR_FILE"
    fi
}

# The last result event wins (stream-json emits exactly one per turn).
# Read line-wise with fromjson? so a stray non-JSON stdout line (update
# banner, notice) is skipped instead of aborting the whole parse.
last_result() { jq -R -c 'fromjson? | select(.event == "result") | .result' "$1" 2>/dev/null | tail -n 1 || true; }
RESULT_JSON=$(last_result "$STREAM_FILE")

# One retry for the denied-tool case. The "no tools" footer is only a
# request: agy has no switch that removes its built-in tools, so the model
# still sometimes calls one (RunCommand), headless mode denies it, and
# the turn ends with an EMPTY response — three live reviews in a row
# (#660). Resuming the SAME conversation (--conversation <id>, verified
# on agy 1.2.11) with a plain "that was denied, answer in text" costs one
# short turn, not a second full review. Only when the first turn is
# otherwise clean, names its conversation, and there is at least
# RETRY_MIN_SECONDS of the budget left.
RETRY_MIN_SECONDS=30
RETRY_NOTE=""
if [[ "$AGY_EXIT" -eq 0 && -n "$RESULT_JSON" ]] \
    && ! grep -q 'print timeout after' "$STDERR_FILE" 2>/dev/null \
    && jq -e '.status == "SUCCESS"
              and ((.response // "") | test("^[[:space:]]*$"))
              and ((.denied_actions // []) | length > 0)' <<< "$RESULT_JSON" >/dev/null 2>&1; then
    CONVERSATION_ID=$(jq -r '.conversation_id // empty' <<< "$RESULT_JSON")
    FIRST_DENIED=$(jq -r '(.denied_actions // []) | map(.display_name // .action) | join(", ")' <<< "$RESULT_JSON")
    REMAINING=$(( PRINT_TIMEOUT_SECONDS - ($(date +%s) - TURN_STARTED) ))
    if [[ -z "$CONVERSATION_ID" ]]; then
        RETRY_NOTE=" No retry: the result named no conversation to resume."
    elif [[ "$REMAINING" -lt "$RETRY_MIN_SECONDS" ]]; then
        RETRY_NOTE=" No retry: only ${REMAINING}s of the ${PRINT_TIMEOUT} budget was left."
    else
        RETRY_INPUT="${TMP_DIR}/retry.ndjson"
        RETRY_TEXT="Your tool call (${FIRST_DENIED}) was denied: this is a headless session that cannot run or approve ANY tool. Do not call any tools. Write the complete review now, as text only, from the diff and context already in this conversation, in the output format the first message asked for."
        if ! jq -c -n --arg c "$RETRY_TEXT" '{event: "user", message: {role: "user", content: $c}}' > "$RETRY_INPUT"; then
            fail "could not encode the retry message as a stream-json message"
        fi
        STREAM_FILE="${TMP_DIR}/stream-retry.ndjson"
        STDERR_FILE="${TMP_DIR}/stderr-retry.txt"
        agy_turn "$RETRY_INPUT" "$STREAM_FILE" "$STDERR_FILE" "${REMAINING}s" --conversation "$CONVERSATION_ID"
        RESULT_JSON=$(last_result "$STREAM_FILE")
        RETRY_NOTE=" This was after one retry in the same conversation (the first turn's ${FIRST_DENIED} call was denied)."
        RETRIED=true
    fi
fi

if [[ "$AGY_EXIT" -ne 0 ]]; then
    fail "agy exited ${AGY_EXIT}$(stderr_excerpt)"
fi
# Timeout before the result-event check: an expiry can truncate the
# stream, and "print timeout" is the right reason then, not "no result".
if grep -q 'print timeout after' "$STDERR_FILE" 2>/dev/null; then
    fail "print timeout (${PRINT_TIMEOUT}) expired with the turn in progress; partial output discarded$(stderr_excerpt)"
fi
if [[ -z "$RESULT_JSON" ]]; then
    fail "agy emitted no result event$(stderr_excerpt)"
fi

# One jq call per field. (A single @tsv pass was tried and rejected: tab
# is IFS whitespace, so `read` collapses an empty field and shifts the
# error message into the wrong variable.) `.error` may be a string or an
# object, hence tostring.
STATUS=$(jq -r '.status // "MISSING"' <<< "$RESULT_JSON")
RESPONSE=$(jq -r '.response // ""' <<< "$RESULT_JSON")
ERROR_MSG=$(jq -r '(.error // "") | tostring' <<< "$RESULT_JSON")
DENIED=$(jq -r '(.denied_actions // []) | map(.display_name // .action) | join(", ")' <<< "$RESULT_JSON")
DENIED_COUNT=$(jq -r '(.denied_actions // []) | length' <<< "$RESULT_JSON")

if [[ "$STATUS" != "SUCCESS" ]]; then
    # Output-token cutoff (#336): name it precisely. Matched against agy's
    # own error text and stderr only, never RESPONSE — the response is the
    # model's text and can legitimately quote the phrase (a review of a
    # diff that mentions it), which must not relabel an unrelated error.
    # Captured live (#336 round 2): status non-SUCCESS with `.error` a
    # plain string, verbatim:
    #   Your previous response was cut off because it exceeded the output token limit
    #   Please continue from where you left off, keeping your response shorter
    #   Retries remaining: 3
    # So `.error` is the observed channel; the stderr tail stays checked
    # as a cheap second channel in case a CLI version moves it. Both
    # halves must sit on the SAME line: across the whole blob, an unrelated
    # "connection was cut off" plus a separate banner naming the token
    # limit would mislabel an ordinary failure as a cutoff.
    cutoff_source="${ERROR_MSG}"$'\n'"$(tail -n 20 "$STDERR_FILE" 2>/dev/null || true)"
    if grep -qiE 'cut off.*output token limit|output token limit.*cut off' <<< "$cutoff_source"; then
        fail "response was cut off because it exceeded the output token limit (status ${STATUS}); the prompt or response was too large for this turn${ERROR_MSG:+: ${ERROR_MSG}}$(stderr_excerpt)"
    fi
    fail "result status ${STATUS}${ERROR_MSG:+: ${ERROR_MSG}}$(stderr_excerpt)"
fi
if [[ -z "${RESPONSE//[[:space:]]/}" ]]; then
    if [[ "$DENIED_COUNT" -gt 0 ]]; then
        fail "empty response; ${DENIED_COUNT} tool action(s) were auto-denied in headless mode (${DENIED}). The reviewer works from the embedded diff only; it must not call any tools (file reads and shell commands are both denied headlessly).${RETRY_NOTE}$(stderr_excerpt)"
    fi
    fail "empty response${ERROR_MSG:+: ${ERROR_MSG}}$(stderr_excerpt)"
fi

# Success. Plain > is safe: the file was truncated above and readers key
# off the caller's completion marker, appended only after we exit.
if ! printf '%s\n' "$RESPONSE" > "$FINDINGS_FILE"; then
    fail "could not write the findings file: ${FINDINGS_FILE}"
fi
if [[ "$DENIED_COUNT" -gt 0 ]]; then
    if ! printf '\n> Note: %s tool action(s) were denied in headless mode (%s); the review ran with less context than the model asked for.\n' \
        "$DENIED_COUNT" "$DENIED" >> "$FINDINGS_FILE"; then
        fail "could not write the findings file: ${FINDINGS_FILE}"
    fi
fi
if [[ "${RETRIED:-false}" == true ]]; then
    if ! printf '\n> Note: the first turn ended empty after a denied %s call; this review is the answer to one follow-up turn in the same conversation.\n' \
        "$FIRST_DENIED" >> "$FINDINGS_FILE"; then
        fail "could not write the findings file: ${FINDINGS_FILE}"
    fi
fi
exit 0
