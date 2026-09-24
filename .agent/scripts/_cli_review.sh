#!/usr/bin/env bash
# Run one headless codex / claude / copilot review turn and validate it.
#
# Usage:
#   _cli_review.sh <agent> <bin> <prompt-file> <findings-file> [<timeout-label>]
#
# <agent> is codex, claude or copilot. Gemini has its own helper
# (_agy_review.sh): its stream-json prompt encoding and result-event
# parsing share no shape with these three, so folding it in here would
# add a fourth case with nothing in common but the trap block.
#
# <timeout-label> is informational only — the bound itself is the caller's
# outer `timeout -k` (cross_model_review.sh: AGENT_TIMEOUT). It is quoted
# in failure reasons so a reader knows which bound the run was under.
#
# Why this exists (issue #313, folding in #212):
#   Before this helper the codex/claude/copilot arms of
#   cross_model_review.sh were bare `timeout ... "$bin" -p < prompt >
#   findings 2>&1` calls, gated on the CLI's exit code alone. Every
#   failure mode that does not move the exit code therefore landed in the
#   findings file as if it were a review:
#     * an empty response (the #288 class: a headless permission denial or
#       an aborted turn that still exits 0);
#     * a quota / rate-limit / auth error printed as ordinary output;
#     * codex's stdout transcript (banner, echoed prompt, tool chatter)
#       stored as the review body because `2>&1` merged everything.
#   Each arm now gets the same forced gate gemini has had since #288.
#
# What may FAIL a run (the rule, #313 round 2):
#   Only a signal the CLI itself reports as machine state may fail a
#   review: a non-zero exit status, a missing or empty result, or a
#   structured error field (claude's `.is_error` / `.error`). Error TEXT
#   never fails a run — it only explains a failure one of those signals
#   has already established, by choosing the reason line.
#
#   This is not caution, it is the only thing that works. Two live runs
#   of this very branch were failed by text scanning: codex echoes the
#   whole prompt — diff included — into BOTH its stdout transcript and
#   its stderr, along with every tool call it makes and that tool's
#   output, so a review of a diff mentioning a rate limit reads exactly
#   like a rate-limited CLI. No channel codex writes is clean, and no
#   heading/length/opener heuristic separates "# Error Handling in
#   auth.py" from "API Error: Quota exceeded" (both were tried and both
#   failed; see the plan's Implementation Notes). A transient
#   "[WARN] overloaded, retrying" on stderr is not a failed review
#   either. The cost of the rule is that a CLI which exits 0 with a
#   polite quota message as its whole answer is reported as a review;
#   the cost of the alternative is discarding real reviews, which is
#   worse and was observed twice.
#
# Contract (identical to _agy_review.sh, deliberately):
#   * This script OWNS the findings file: it is truncated as the very
#     first statement, before any guard can fail, and afterwards holds
#     either the review text or a failure reason — never the previous
#     run's review under a fresh failure marker. The caller must not
#     redirect stdout onto it, and appends the "--- Review complete ---" /
#     "--- Review failed ---" marker itself.
#   * Exit 0 only when the CLI exited 0 and produced a non-empty result
#     (and, for claude, a structurally successful one). Exit 1 otherwise
#     (findings file holds the reason). Exit 2 on usage or configuration
#     errors (also recorded in the findings file when it is writable).
#   * The CLI runs as a background child that is `wait`ed on, with
#     INT/TERM/HUP traps armed BEFORE the spawn, so a TERM from the
#     caller's `timeout -k` or its cleanup reaches the CLI itself instead
#     of being deferred until the turn ends on its own. Leaving a CLI
#     running would burn quota on an abandoned review. The handler then
#     WAITS for the child, escalating to SIGKILL after
#     REVIEW_KILL_ESCALATION seconds (default 5), so this helper never
#     exits out from under a live CLI.
#   * No temp files survive any exit path this script can observe (EXIT
#     trap plus signal traps that exit). SIGKILL is the exception; the
#     caller closes that gap by handing us a TMPDIR it owns and sweeps
#     (cross_model_review.sh: AGENT_TMP_ROOT).
#   * All diagnostics go to stderr; stdout is unused.
#
# Verified against codex-cli 0.155.1, claude 2.x and copilot 1.0.61 help
# output on this host (2026-09-22): `codex exec [PROMPT]` reads stdin when
# no prompt argument is given and `-o/--output-last-message <FILE>` writes
# only the final message; `claude -p --output-format json` emits a single
# result object and `--permission-prompts none` auto-denies anything that
# would prompt; `copilot -p <text> -s` prints only the agent response,
# with `--available-tools`, `--disable-builtin-mcps` and `--no-ask-user`
# available to strip a headless run down to no tools at all, and the
# `-p ""` + stdin form is the one verified in #212.

set -uo pipefail

usage() {
    echo "Usage: $0 <agent: codex|claude|copilot> <bin> <prompt-file> <findings-file> [<timeout-label>]" >&2
}

if [[ $# -lt 4 || $# -gt 5 ]]; then
    usage
    exit 2
fi

AGENT="$1"
CLI_BIN="$2"
PROMPT_FILE="$3"
FINDINGS_FILE="$4"
TIMEOUT_LABEL="${5:-}"

# Truncate the findings file before anything else can fail. Without this a
# guard failure would leave the previous run's review in place under a
# fresh "--- Review failed ---" marker, and review-code would read stale
# findings as current (_agy_review.sh:69-72 does the same, for the same
# reason).
if ! : > "$FINDINGS_FILE"; then
    echo "ERROR: cannot write findings file: ${FINDINGS_FILE}" >&2
    exit 2
fi

# Write a failure reason into the findings file and exit 1.
fail() {
    local reason="$1"
    {
        echo "${AGENT} review did not produce a usable result."
        echo ""
        echo "Reason: ${reason}"
    } > "$FINDINGS_FILE"
    echo "ERROR: ${AGENT} review failed: ${reason}" >&2
    exit 1
}

# Usage error, recorded in the findings file so the caller's failure
# marker has a reason above it rather than an empty file.
usage_fail() {
    local reason="$1"
    {
        echo "${AGENT} review could not be started."
        echo ""
        echo "Reason: ${reason}"
    } > "$FINDINGS_FILE"
    echo "ERROR: ${reason}" >&2
    usage
    exit 2
}

case "$AGENT" in
    codex|claude|copilot) ;;
    *) usage_fail "unsupported agent '${AGENT}' (codex, claude and copilot run here; gemini runs through _agy_review.sh)" ;;
esac

if [[ ! -r "$PROMPT_FILE" ]]; then
    fail "prompt file not readable: ${PROMPT_FILE}"
fi
# Accept a bare name (resolved on PATH) or a path; `test -x` alone only
# looks at the current directory for a bare name.
CLI_BIN_RESOLVED=$(command -v "$CLI_BIN" 2>/dev/null || true)
if [[ -z "$CLI_BIN_RESOLVED" || ! -x "$CLI_BIN_RESOLVED" ]]; then
    fail "${AGENT} binary not found or not executable: ${CLI_BIN}"
fi
if [[ "$AGENT" == "claude" ]] && ! command -v jq >/dev/null 2>&1; then
    fail "jq is required to parse claude's --output-format json result (see bootstrap.sh)"
fi

# The helper's own escalation window must fit inside the caller's
# `timeout -k` grace, or the caller SIGKILLs this helper before it can
# SIGKILL a CLI that ignored SIGTERM — and that CLI is then orphaned,
# burning quota with nothing left to stop it. The caller passes its
# AGENT_KILL_AFTER in the environment; when it is absent (direct
# invocation) there is no outer grace to fit inside and nothing to check.
REVIEW_KILL_ESCALATION="${REVIEW_KILL_ESCALATION:-5}"
to_seconds() {
    [[ "$1" =~ ^([0-9]+(\.[0-9]+)?)([smhd]?)$ ]] || return 1
    local number="${BASH_REMATCH[1]}" unit="${BASH_REMATCH[3]}" mult=1
    case "$unit" in m) mult=60 ;; h) mult=3600 ;; d) mult=86400 ;; *) mult=1 ;; esac
    awk -v n="$number" -v m="$mult" 'BEGIN { printf "%.0f", n * m }'
}
if ! ESCALATION_SECONDS=$(to_seconds "$REVIEW_KILL_ESCALATION"); then
    usage_fail "REVIEW_KILL_ESCALATION value '${REVIEW_KILL_ESCALATION}' is not a duration (a number of seconds, optionally with an s/m/h suffix)"
fi
if [[ -n "${AGENT_KILL_AFTER:-}" ]]; then
    if ! KILL_AFTER_SECONDS=$(to_seconds "$AGENT_KILL_AFTER"); then
        usage_fail "AGENT_KILL_AFTER value '${AGENT_KILL_AFTER}' is not a duration"
    fi
    # Both zero is the one legal equal case: the caller wants no grace at
    # all, and this helper escalates immediately to match.
    if [[ "$KILL_AFTER_SECONDS" -le "$ESCALATION_SECONDS" ]] \
        && ! [[ "$KILL_AFTER_SECONDS" -eq 0 && "$ESCALATION_SECONDS" -eq 0 ]]; then
        usage_fail "AGENT_KILL_AFTER (${AGENT_KILL_AFTER}) must be greater than REVIEW_KILL_ESCALATION (${REVIEW_KILL_ESCALATION}): the caller's SIGKILL would land on this helper before it could SIGKILL a CLI that ignored SIGTERM, orphaning that CLI. Raise AGENT_KILL_AFTER or lower REVIEW_KILL_ESCALATION (set both to 0 for no grace at all)."
    fi
fi

# Known error markers. These NEVER fail a run (see the header rule) —
# they only pick a more useful reason line for a failure that the exit
# status, an empty result or a structured error field has already
# established.
ERROR_MARKER_RE='quota|rate limit|usage limit|overloaded|not logged in|unauthorized|authentication (failed|error|required)|please (log ?in|sign in)'

# If the already-failing run's diagnostics name a known condition, say so
# in the reason. Returns the note (possibly empty) on stdout.
marker_note() {
    local file="$1" hit
    [[ -s "$file" ]] || return 0
    hit=$(grep -m1 -iE "$ERROR_MARKER_RE" "$file" 2>/dev/null || true)
    [[ -n "$hit" ]] && printf ' This looks like a quota / rate-limit / authentication problem: %s' "$hit"
    return 0
}

# Temp files (codex's final-message file, each CLI's stdout/stderr logs)
# live under the TMPDIR the caller hands us, and are removed on every exit
# path this script can trap.
TMP_DIR=$(mktemp -d -t cli-review.XXXXXX) || fail "mktemp failed"
trap 'rm -rf "$TMP_DIR"' EXIT
# Armed before the CLI is launched (CLI_PID empty until then) so a signal
# in the launch window cannot leave the CLI running behind an exited
# helper. TERM is the live path under cross_model_review.sh: this helper
# runs as a background child of a non-interactive shell, where bash makes
# SIGINT ignored (and an ignored signal cannot be trapped). The INT trap
# is for a direct interactive invocation, where Ctrl-C does arrive.
#
# The handler does not just signal and leave: it waits for the CLI to
# actually die, escalating to SIGKILL after REVIEW_KILL_ESCALATION
# seconds. Exiting straight after the `kill` would (a) let the EXIT trap
# remove TMP_DIR out from under a CLI still writing into it, and (b)
# defeat the caller's `timeout -k` backstop, whose SIGKILL is aimed at
# this helper — once we are gone it has nothing left to kill and a CLI
# that ignored SIGTERM would keep running, burning quota on an abandoned
# review. The escalation is validated above to fit inside the caller's
# grace (AGENT_KILL_AFTER).
CLI_PID=""
terminate_child() {
    local code="$1" watchdog
    # Re-entrancy: a second signal (repeated Ctrl-C, TERM then HUP) would
    # otherwise start a second watchdog and clobber $watchdog, leaking
    # the first one.
    trap '' INT TERM HUP
    if [[ -n "$CLI_PID" ]]; then
        kill "$CLI_PID" 2>/dev/null
        # `wait` returns the moment the CLI dies, so a clean shutdown
        # costs milliseconds, not the escalation window. The watchdog
        # only matters for a CLI that ignores SIGTERM. It is NOT waited
        # on: a subshell sleeping in `sleep` defers the TERM we send it
        # until that sleep ends, so waiting would reintroduce the full
        # window on every clean exit.
        # The watchdog resets INT/TERM/HUP first: it is forked while this
        # handler has them ignored, an ignored disposition is inherited,
        # and without the reset the `kill "$watchdog"` below is a no-op —
        # the watchdog would outlive every clean shutdown by the whole
        # escalation window and then `kill -9` whatever process had been
        # given the dead CLI's PID (`kill -0` checks a PID, not identity).
        # rolker/ros2_agent_workspace #660.
        ( trap - INT TERM HUP
          sleep "$REVIEW_KILL_ESCALATION"
          kill -0 "$CLI_PID" 2>/dev/null && kill -9 "$CLI_PID" 2>/dev/null ) &
        watchdog=$!
        wait "$CLI_PID" 2>/dev/null
        kill "$watchdog" 2>/dev/null
        CLI_PID=""
    fi
    exit "$code"
}
trap 'terminate_child 130' INT
trap 'terminate_child 143' TERM HUP

STDOUT_FILE="${TMP_DIR}/stdout.txt"
STDERR_FILE="${TMP_DIR}/stderr.txt"
CODEX_OUT_FILE="${TMP_DIR}/final-message.txt"

# Last 20 lines of a log, for failure reports: a fatal error lands at the
# end, after any startup chatter.
log_excerpt() {
    local label="$1" file="$2"
    if [[ -s "$file" ]]; then
        printf '\n%s (last 20 lines):\n' "$label"
        tail -n 20 "$file"
    fi
}

bound_note() {
    [[ -n "$TIMEOUT_LABEL" ]] && printf ' (run under the caller bound %s)' "$TIMEOUT_LABEL"
}

# Run the CLI as a waited-on background child, so a TERM/INT trapped here
# reaches it at once instead of being deferred until the turn ends on its
# own. Args: <stdout-file> <stderr-file, or "-" to merge into stdout>
# followed by the command.
#
# The prompt redirect goes on the backgrounded command itself, never on
# the call to this function: bash points an asynchronous command's stdin
# at /dev/null unless the command carries its own redirect, so a redirect
# one level up would hand the CLI an empty prompt.
run_cli() {
    local out="$1" err="$2"
    shift 2
    if [[ "$err" == "-" ]]; then
        "$@" < "$PROMPT_FILE" > "$out" 2>&1 &
    else
        "$@" < "$PROMPT_FILE" > "$out" 2> "$err" &
    fi
    CLI_PID=$!
    CLI_EXIT=0
    wait "$CLI_PID" || CLI_EXIT=$?
    CLI_PID=""
}

case "$AGENT" in
    codex)
        # No [PROMPT] argument, so codex reads the prompt from stdin
        # (passing one would make stdin an appended <stdin> block
        # instead). `-o` writes ONLY the final assistant message; the
        # stdout transcript — banner, echoed prompt, tool chatter — goes
        # to a log that is discarded on success and excerpted into the
        # reason on failure, never into the findings file.
        #
        # NOTHING codex prints can fail this run. It replays the prompt,
        # every tool call and every tool's output on BOTH stdout and
        # stderr, so neither channel is a diagnostics channel: a review
        # of a diff that mentions a rate limit, or a `jq: error` line
        # from a command codex itself ran, reads exactly like a failing
        # CLI. Two live runs of this branch were failed that way (#313
        # rounds 1 and 2). Only the exit status and an empty/missing `-o`
        # file fail codex; the text is kept for the reason line.
        DIAG_LABEL='codex transcript'
        DIAG_FILE="$STDOUT_FILE"
        # -s read-only -a never (rolker/ros2_agent_workspace #660): pinned
        # explicitly rather than trusting codex-cli's current defaults, so
        # a future CLI release or a host ~/.codex/config.toml override
        # cannot silently grant write/full-access execution to a headless
        # review turn reading an untrusted diff. Both flags are GLOBAL
        # `codex` options, not `exec`-subcommand options (confirmed via
        # `codex exec --help` vs `codex --help` on codex-cli 0.156.1) —
        # they must precede `exec`, not follow it alongside `-o`.
        run_cli "$STDOUT_FILE" "$STDERR_FILE" "$CLI_BIN_RESOLVED" -s read-only -a never exec -o "$CODEX_OUT_FILE"
        if [[ "$CLI_EXIT" -ne 0 ]]; then
            fail "codex exited ${CLI_EXIT}$(bound_note)$(marker_note "$STDERR_FILE")$(log_excerpt 'codex transcript' "$STDOUT_FILE")$(log_excerpt 'codex stderr' "$STDERR_FILE")"
        fi
        if [[ ! -s "$CODEX_OUT_FILE" ]]; then
            fail "empty response: codex wrote no final message (its --output-last-message file is missing or empty). In headless mode this is what an aborted turn looks like — the exit code stays 0.$(marker_note "$STDERR_FILE")$(log_excerpt 'codex transcript' "$STDOUT_FILE")$(log_excerpt 'codex stderr' "$STDERR_FILE")"
        fi
        RESULT=$(cat "$CODEX_OUT_FILE")
        ;;
    claude)
        # --output-format json gives exactly one result object, so a
        # truncated or empty turn is detectable; --permission-prompts none
        # auto-denies anything that would prompt instead of hanging (the
        # headless behaviour _agy_review.sh relies on). No
        # --permission-mode: plan mode's terminal move is presenting a
        # plan for approval, which would pass every gate below while
        # putting a plan, not a review, in the findings file.
        DIAG_LABEL='claude stderr'
        DIAG_FILE="$STDERR_FILE"
        run_cli "$STDOUT_FILE" "$STDERR_FILE" \
            "$CLI_BIN_RESOLVED" -p --output-format json --permission-prompts none
        # JSON before exit status (#336): claude 2.1.281 exits 1 for every
        # captured failure — unknown --model (404), --max-turns and
        # --max-budget-usd hit — and puts the reason only in its JSON
        # result. Failing on the exit code first discarded that reason.
        # The exit code still fails the run in every case: it is the
        # reason when there is no JSON, it is appended to a JSON-reported
        # failure, and a success-looking JSON with a non-zero exit fails
        # on the exit code at the end of this arm.
        # `jq -e .` alone accepts ANY truthy JSON value — a bare string,
        # a number, an array — and the field reads below then abort jq
        # with "Cannot index string with string". This helper runs
        # `set -uo pipefail` without `-e`, so an unguarded jq failure
        # would not stop it: it would carry on with an empty value and
        # misreport the result instead of recording why (#313 round 2,
        # codex must-fix 1). Require an object before indexing it, and
        # keep every extraction inside a guarded block so a jq failure
        # still lands in fail().
        if ! jq -e 'type == "object"' "$STDOUT_FILE" >/dev/null 2>&1; then
            # No JSON to read: a non-zero exit (crash, SIGKILL → 137) is
            # then the only reason there is, so keep it and its notes.
            if [[ "$CLI_EXIT" -ne 0 ]]; then
                fail "claude exited ${CLI_EXIT}$(bound_note)$(marker_note "$STDERR_FILE")$(log_excerpt 'claude stderr' "$STDERR_FILE")"
            fi
            fail "claude did not emit a JSON result object$(log_excerpt 'claude stdout' "$STDOUT_FILE")$(log_excerpt 'claude stderr' "$STDERR_FILE")"
        fi
        # `.error` may be a string or an object; take .message when it is
        # an object. Without this, an is_error / bad-subtype failure whose
        # `.result` is empty reports no cause at all.
        # One jq call per field (no @tsv: `.result` is multi-line review
        # text, which @tsv would escape into a single line), each one
        # guarded so a jq failure lands in fail() instead of leaving the
        # field empty (there is no `set -e` here to stop the helper).
        # `printf -v` rather than a command substitution: fail() exits,
        # and an exit inside $( ) would only end the subshell, leaving
        # the helper running with an empty value and no reason recorded.
        read_claude_field() {
            local name="$1" expr="$2" value
            value=$(jq -r "$expr" "$STDOUT_FILE" 2>>"${TMP_DIR}/jq-error.txt") || return 1
            printf -v "$name" '%s' "$value"
        }
        IS_ERROR="false"; SUBTYPE="missing"; RESULT=""; ERROR_MSG=""
        ERRORS_ARR=""; API_ERROR_STATUS=""; TERMINAL_REASON=""
        CLAUDE_READ_FAILED=""
        read_claude_field IS_ERROR '(.is_error // false) | tostring' || CLAUDE_READ_FAILED=".is_error"
        read_claude_field SUBTYPE '.subtype // "missing"' || CLAUDE_READ_FAILED=".subtype"
        read_claude_field RESULT '.result // "" | tostring' || CLAUDE_READ_FAILED=".result"
        read_claude_field ERROR_MSG '(.error // "") | if type == "object" then (.message // tostring) else tostring end' || CLAUDE_READ_FAILED=".error"
        # `.errors` is where claude 2.1.281 puts the max-turns and
        # max-budget reasons (with `.result` null). Type-safe: string
        # elements join as-is, any other element is tojson'd (jq's join
        # rejects non-strings), and a non-array value is tostring'd.
        read_claude_field ERRORS_ARR '(.errors // []) | if type == "array" then (map(if type == "string" then . else tojson end) | join("; ")) else tostring end' || CLAUDE_READ_FAILED=".errors"
        read_claude_field API_ERROR_STATUS '(.api_error_status // empty) | tostring' || CLAUDE_READ_FAILED=".api_error_status"
        read_claude_field TERMINAL_REASON '(.terminal_reason // empty) | tostring' || CLAUDE_READ_FAILED=".terminal_reason"
        if [[ -n "$CLAUDE_READ_FAILED" ]]; then
            fail "claude's JSON result could not be read (${CLAUDE_READ_FAILED})$(log_excerpt 'jq error' "${TMP_DIR}/jq-error.txt")$(log_excerpt 'claude stdout' "$STDOUT_FILE")"
        fi
        # Preference: `.errors`, then `.error`, then `.result` (an unknown
        # --model has no `.errors`; its reason is in `.result`).
        CLAUDE_DETAIL="${RESULT:-}"
        [[ -n "$ERROR_MSG" ]] && CLAUDE_DETAIL="${ERROR_MSG}${RESULT:+ | result: ${RESULT}}"
        [[ -n "$ERRORS_ARR" ]] && CLAUDE_DETAIL="${ERRORS_ARR}${RESULT:+ | result: ${RESULT}}"
        [[ -n "$API_ERROR_STATUS" ]] && CLAUDE_DETAIL="${CLAUDE_DETAIL} (api_error_status: ${API_ERROR_STATUS})"
        [[ -n "$TERMINAL_REASON" ]] && CLAUDE_DETAIL="${CLAUDE_DETAIL} (terminal_reason: ${TERMINAL_REASON})"
        CLAUDE_EXIT_NOTE=""
        [[ "$CLI_EXIT" -ne 0 ]] && CLAUDE_EXIT_NOTE=" (claude also exited ${CLI_EXIT})"
        # Structured fields and the exit status are the ONLY things that
        # fail claude here (the header rule): `.is_error`, a non-success
        # `.subtype`, a non-zero exit, or an empty `.result` — never the
        # text of a successful result.
        if [[ "$IS_ERROR" == "true" ]]; then
            fail "claude returned is_error=true (subtype ${SUBTYPE})${CLAUDE_DETAIL:+: ${CLAUDE_DETAIL}}${CLAUDE_EXIT_NOTE}$(log_excerpt 'claude stderr' "$STDERR_FILE")"
        fi
        if [[ "$SUBTYPE" != "success" ]]; then
            fail "claude result subtype is '${SUBTYPE}', not 'success'${CLAUDE_DETAIL:+: ${CLAUDE_DETAIL}}${CLAUDE_EXIT_NOTE}$(log_excerpt 'claude stderr' "$STDERR_FILE")"
        fi
        # A clean-looking result from a CLI that still exited non-zero is
        # not trustworthy: fail on the exit code.
        if [[ "$CLI_EXIT" -ne 0 ]]; then
            fail "claude exited ${CLI_EXIT} despite a successful-looking JSON result$(bound_note)$(marker_note "$STDERR_FILE")$(log_excerpt 'claude stderr' "$STDERR_FILE")"
        fi
        ;;
    copilot)
        # #212/#274: the prompt goes over STDIN, never argv — a single
        # argv string is capped at MAX_ARG_STRLEN (128 KiB on Linux)
        # regardless of ARG_MAX, and Deep-tier prompts pass that. There is
        # deliberately NO prompt-size guard here: stdin has no such limit,
        # so a bound could only reject large reviews that would otherwise
        # work, re-imposing the ceiling the stdin path removed. The
        # channel is enforced by the stdin-contract test in
        # test_cross_model_review.sh, which asserts the prompt is absent
        # from argv and present on stdin.
        #
        # Least privilege: a reviewer needs no tools at all — the diff is
        # in the prompt — and the diff is untrusted input, so
        # --allow-all-tools would be a privilege escalation driven by
        # whatever the PR contains. --available-tools='' removes the tool
        # set, --disable-builtin-mcps removes the built-in MCP servers,
        # and --no-ask-user stops the agent blocking on a question no one
        # can answer headlessly. -p "" keeps print mode without putting
        # the prompt on argv; -s prints only the agent response (no stats
        # footer), so the output is used as-is — no second strip, which
        # could truncate a review body containing a footer-looking line.
        #
        # PENDING ONE LIVE CONFIRMATION when Copilot quota returns
        # (#313). Two parts of this invocation rest on `copilot --help`
        # for 1.0.61 plus #212's run on 1.0.48, not on a live 1.0.61 run:
        #   * `--available-tools=''` — if the argument validator rejects
        #     an empty list, drop it and deny the dangerous tools instead
        #     (`--deny-tool='shell' --deny-tool='write'`), keeping
        #     --disable-builtin-mcps and --no-ask-user;
        #   * `-p ""` — the empty print-mode prompt that keeps the real
        #     prompt on stdin. #212 verified this form on 1.0.48; if a
        #     later version rejects an empty prompt string, the fix is a
        #     placeholder like `-p "Review the input on stdin."`, NEVER
        #     the prompt itself on argv (#274: MAX_ARG_STRLEN).
        DIAG_LABEL='copilot stderr'
        DIAG_FILE="$STDERR_FILE"
        run_cli "$STDOUT_FILE" "$STDERR_FILE" \
            "$CLI_BIN_RESOLVED" -p "" -s --available-tools='' --disable-builtin-mcps --no-ask-user
        # Exit status and an empty `-s` output are the only failures
        # (the header rule); a transient "[WARN] overloaded, retrying" on
        # stderr is not one, and the text of a real review never is.
        if [[ "$CLI_EXIT" -ne 0 ]]; then
            fail "copilot exited ${CLI_EXIT}$(bound_note)$(marker_note "$STDERR_FILE")$(log_excerpt 'copilot stderr' "$STDERR_FILE")"
        fi
        RESULT=$(cat "$STDOUT_FILE")
        ;;
esac

if [[ -z "${RESULT//[[:space:]]/}" ]]; then
    # DIAG_FILE is the channel that actually carries this CLI's
    # diagnostics: codex merges nothing into stderr, so for it the
    # transcript is the only place a reason can be found.
    fail "empty response$(bound_note). In headless mode this is what a permission denial or an aborted turn looks like — the exit code stays 0.$(marker_note "$STDERR_FILE")$(log_excerpt "$DIAG_LABEL" "$DIAG_FILE")"
fi
# No text scan on the result. A CLI that exits 0 with a polite quota
# message as its whole answer is passed through as a review — see the
# header rule for why that is the lesser evil: every attempt to catch it
# by text also discarded real reviews (#313 rounds 1 and 2).

# Success. Plain > is safe: the file was truncated above and readers key
# off the caller's completion marker, appended only after we exit.
if ! printf '%s\n' "$RESULT" > "$FINDINGS_FILE"; then
    fail "could not write the findings file: ${FINDINGS_FILE}"
fi
exit 0
