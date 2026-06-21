#!/bin/bash
# Regression tests for dispatch_subagent.sh — the no-docker surface
# (arg validation, skill->entry-type mapping, handoff-prompt content,
# in-process emission). Container mode requires docker + a real worktree
# and is covered by the manual layer-worktree verification in #490.
#
# Run: .agent/scripts/test_dispatch_subagent.sh

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DISPATCH="$SCRIPT_DIR/dispatch_subagent.sh"
ROOT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"
if [[ "$ROOT_DIR" == *"/.workspace-worktrees/"* ]]; then
    ROOT_DIR="${ROOT_DIR%%/.workspace-worktrees/*}"
elif [[ "$ROOT_DIR" == *"/layers/worktrees/"* ]]; then
    ROOT_DIR="${ROOT_DIR%%/layers/worktrees/*}"
fi

# A throwaway worktree so the script's worktree-resolution succeeds.
TEST_ISSUE=999999
FAKE_WT="$ROOT_DIR/.workspace-worktrees/issue-workspace-$TEST_ISSUE"
FAKE_WT2="$ROOT_DIR/layers/worktrees/issue-test-$TEST_ISSUE"
mkdir -p "$FAKE_WT/.agent/work-plans/issue-$TEST_ISSUE"
trap 'rm -rf "$FAKE_WT" "$FAKE_WT2"' EXIT

PASS=0; FAIL=0
ok()   { PASS=$((PASS+1)); printf '  ok   - %s\n' "$1"; }
bad()  { FAIL=$((FAIL+1)); printf '  FAIL - %s\n' "$1"; [ -n "${2:-}" ] && printf '         %s\n' "$2"; }

# assert the command exits non-zero AND stderr contains a substring
assert_fails() {
    local desc="$1" needle="$2"; shift 2
    local out rc
    out="$("$DISPATCH" "$@" 2>&1)"; rc=$?
    if [ "$rc" -eq 0 ]; then bad "$desc" "expected non-zero exit"; return; fi
    case "$out" in *"$needle"*) ok "$desc" ;; *) bad "$desc" "stderr missing: $needle";; esac
}

# assert the command succeeds AND stdout contains a substring
assert_emits() {
    local desc="$1" needle="$2"; shift 2
    local out rc
    out="$("$DISPATCH" "$@" 2>/dev/null)"; rc=$?
    if [ "$rc" -ne 0 ]; then bad "$desc" "expected zero exit, got $rc"; return; fi
    case "$out" in *"$needle"*) ok "$desc" ;; *) bad "$desc" "stdout missing: $needle";; esac
}

echo "dispatch_subagent.sh tests"

# --- validation ---
assert_fails "rejects bad --mode" "must be 'in-process' or 'container'" \
    --mode bogus --issue "$TEST_ISSUE" --skill review-code
assert_fails "requires --issue" "--issue <N> is required" \
    --mode in-process --skill review-code
assert_fails "rejects skill+prompt-file together" "exactly one of --skill or --prompt-file" \
    --mode in-process --issue "$TEST_ISSUE" --skill review-code --prompt-file /tmp/nope
assert_fails "requires a task (skill or prompt-file)" "one of --skill" \
    --mode in-process --issue "$TEST_ISSUE"
assert_fails "rejects missing prompt-file" "--prompt-file not found" \
    --mode container --issue "$TEST_ISSUE" --prompt-file /nonexistent/path.md
assert_fails "rejects unknown worktree" "no worktree found" \
    --mode in-process --issue 888888 --skill review-code
assert_fails "rejects bad --output-format" "must be one of stream-json|json|text" \
    --mode container --issue "$TEST_ISSUE" --skill review-code --output-format yaml
assert_emits "accepts a valid --output-format (in-process)" "BEGIN HANDOFF" \
    --mode in-process --issue "$TEST_ISSUE" --skill review-code --output-format json

# --- in-process handoff content ---
assert_emits "embeds git -c identity literals" 'git -c user.name="Claude Code Agent" -c user.email="roland+claude-code@ccom.unh.edu"' \
    --mode in-process --issue "$TEST_ISSUE" --skill review-code
assert_emits "embeds the no-push / no--no-verify rule" 'Never use `--no-verify`' \
    --mode in-process --issue "$TEST_ISSUE" --skill review-code
assert_emits "embeds the progress.md exit contract" ".agent/work-plans/issue-$TEST_ISSUE/progress.md" \
    --mode in-process --issue "$TEST_ISSUE" --skill review-code
assert_emits "embeds the no-credentials-in-output rule" "Never write credentials/tokens" \
    --mode in-process --issue "$TEST_ISSUE" --skill review-code
assert_emits "has copy markers" "BEGIN HANDOFF" \
    --mode in-process --issue "$TEST_ISSUE" --skill review-code

# --- skill -> entry-type mapping ---
assert_emits "review-code -> Local Review" 'final `## Local Review` entry' \
    --mode in-process --issue "$TEST_ISSUE" --skill review-code
assert_emits "review-plan -> Plan Review" 'final `## Plan Review` entry' \
    --mode in-process --issue "$TEST_ISSUE" --skill review-plan
assert_emits "plan-task -> Plan Authored" 'final `## Plan Authored` entry' \
    --mode in-process --issue "$TEST_ISSUE" --skill plan-task
assert_emits "triage-reviews -> Integrated Review" 'final `## Integrated Review` entry' \
    --mode in-process --issue "$TEST_ISSUE" --skill triage-reviews
assert_emits "unknown skill -> generic typed-entry clause" "append your final typed entry" \
    --mode in-process --issue "$TEST_ISSUE" --skill frobnicate

# --- explicit --entry-type overrides the skill default ---
assert_emits "--entry-type overrides skill mapping" 'final `## Custom Type` entry' \
    --mode in-process --issue "$TEST_ISSUE" --skill review-code --entry-type "Custom Type"

# --- prompt-file mode uses the file as the task body ---
PF="$(mktemp /tmp/dispatch_test_prompt.XXXXXX.md)"
echo "UNIQUE_TASK_MARKER_42" > "$PF"
assert_emits "prompt-file content becomes the task body" "UNIQUE_TASK_MARKER_42" \
    --mode in-process --issue "$TEST_ISSUE" --prompt-file "$PF"
rm -f "$PF"

# --- per-phase model mapping (in-process reports the recommended model) ---
assert_emits "review-code -> opus model" "Recommended model: opus" \
    --mode in-process --issue "$TEST_ISSUE" --skill review-code
assert_emits "triage-reviews -> opus model" "Recommended model: opus" \
    --mode in-process --issue "$TEST_ISSUE" --skill triage-reviews
assert_emits "review-issue -> sonnet model" "Recommended model: sonnet" \
    --mode in-process --issue "$TEST_ISSUE" --skill review-issue
assert_emits "plan-task -> sonnet model" "Recommended model: sonnet" \
    --mode in-process --issue "$TEST_ISSUE" --skill plan-task
assert_emits "unknown skill -> sonnet default" "Recommended model: sonnet" \
    --mode in-process --issue "$TEST_ISSUE" --skill frobnicate
assert_emits "raw prompt -> sonnet default" "Recommended model: sonnet" \
    --mode in-process --issue "$TEST_ISSUE" --prompt-file /etc/hostname
assert_emits "--model overrides skill mapping" "Recommended model: haiku" \
    --mode in-process --issue "$TEST_ISSUE" --skill review-code --model haiku

# --- identity override via env ---
out="$(AGENT_NAME="Tester Bot" AGENT_EMAIL="roland+tester@ccom.unh.edu" \
    "$DISPATCH" --mode in-process --issue "$TEST_ISSUE" --skill review-code 2>/dev/null)"
case "$out" in
    *'git -c user.name="Tester Bot" -c user.email="roland+tester@ccom.unh.edu"'*) ok "honors AGENT_NAME/AGENT_EMAIL override" ;;
    *) bad "honors AGENT_NAME/AGENT_EMAIL override" "override not reflected in handoff" ;;
esac

# --- ambiguous worktree match fails loud (#526) ---
# Two worktrees (workspace FAKE_WT + layer FAKE_WT2) match the same bare issue
# number. The dispatcher must refuse to guess and exit non-zero, telling the
# operator to disambiguate with --repo-slug. (Was warn-and-proceed before #526.)
mkdir -p "$FAKE_WT2/.agent/work-plans/issue-$TEST_ISSUE"
out="$("$DISPATCH" --mode in-process --issue "$TEST_ISSUE" --skill review-code 2>&1)"; rc=$?
if [ "$rc" -eq 0 ]; then
    bad "fails loud on ambiguous worktree match" "expected non-zero exit, got 0"
else
    case "$out" in
        *"worktrees match issue #$TEST_ISSUE"*"refusing to guess"*) ok "fails loud on ambiguous worktree match" ;;
        *) bad "fails loud on ambiguous worktree match" "stderr missing the refuse-to-guess message" ;;
    esac
fi
rm -rf "$FAKE_WT2"

# --- --context-file injects host-fetched read context into the handoff (#552) ---
# Composable with --skill: the body lands AND the skill keeps its entry type.
CTXF="$(mktemp /tmp/dispatch_test_ctx.XXXXXX.md)"
echo "INJECTED_CONTEXT_MARKER_77" > "$CTXF"
assert_emits "--context-file content lands in the handoff" "INJECTED_CONTEXT_MARKER_77" \
    --mode in-process --issue "$TEST_ISSUE" --skill review-issue --context-file "$CTXF"
assert_emits "--context-file adds the injected-context heading" "Injected GitHub context" \
    --mode in-process --issue "$TEST_ISSUE" --skill review-issue --context-file "$CTXF"
assert_emits "--context-file is composable with --skill (entry-type kept)" 'final `## Issue Review` entry' \
    --mode in-process --issue "$TEST_ISSUE" --skill review-issue --context-file "$CTXF"
assert_fails "rejects missing --context-file" "--context-file not found" \
    --mode in-process --issue "$TEST_ISSUE" --skill review-issue --context-file /nonexistent/ctx.md
rm -f "$CTXF"

# --- sourced unit tests for the Gap-2 freshness gate (#552) ---
# Source the script (the source-guard returns before any dispatch runs), then
# exercise is_fresh_entry / last_entry_signature directly — the gate lives in the
# container path, unreachable by execution without docker. Run in a subshell
# because sourcing enables `set -euo pipefail`; re-disable -e inside so the
# intentional non-zero returns (the "not fresh" cases) don't abort the subshell.
SOURCED_RESULT="$(
    # shellcheck disable=SC1090  # $DISPATCH is resolved at runtime; the guard returns
    source "$DISPATCH" >/dev/null 2>&1
    set +e
    p=0; f=0
    chk() { if [ "$2" = "$3" ]; then p=$((p+1)); else f=$((f+1)); echo "MISMATCH: $1 (want $3 got $2)"; fi; }

    # is_fresh_entry branches
    is_fresh_entry 0 1 "" "T1|complete";            chk "append 0->1 fresh"            "$?" 0
    is_fresh_entry 1 1 "T0|failed" "T1|complete";   chk "replace 1->1 diff-sig fresh"  "$?" 0
    is_fresh_entry 1 1 "T0|failed" "T0|failed";     chk "no-write identical not-fresh" "$?" 1
    is_fresh_entry 1 1 "T0|failed" "";              chk "empty post-sig not-fresh"     "$?" 1

    # last_entry_signature against a REPLACE fixture (count stays 1->1, the #466 case)
    TMP="$(mktemp -d)"
    cat > "$TMP/progress.md" <<PEOF
# Issue $TEST_ISSUE

## Issue Review
**Status**: failed
**When**: 2026-06-21 10:00 +00:00
**By**: X (Y)
PEOF
    pre="$(last_entry_signature "$TMP/progress.md" "Issue Review")"
    cat > "$TMP/progress.md" <<PEOF
# Issue $TEST_ISSUE

## Issue Review
**Status**: complete
**When**: 2026-06-21 10:01 +00:00
**By**: X (Y)
PEOF
    post="$(last_entry_signature "$TMP/progress.md" "Issue Review")"
    rm -rf "$TMP"
    [ -n "$pre" ] && [ -n "$post" ] && [ "$pre" != "$post" ]; chk "last_entry_signature replace differs" "$?" 0
    is_fresh_entry 1 1 "$pre" "$post";              chk "fixture replace 1->1 is fresh" "$?" 0

    echo "SUBPASS=$p SUBFAIL=$f"
)"
echo "$SOURCED_RESULT" | grep '^MISMATCH:' || true
sub_p="$(printf '%s\n' "$SOURCED_RESULT" | sed -n 's/.*SUBPASS=\([0-9]*\).*/\1/p')"
sub_f="$(printf '%s\n' "$SOURCED_RESULT" | sed -n 's/.*SUBFAIL=\([0-9]*\).*/\1/p')"
if [ -n "$sub_p" ] && [ "${sub_f:-1}" = "0" ]; then
    ok "sourced freshness-gate units ($sub_p checks)"
else
    bad "sourced freshness-gate units" "subpass=${sub_p:-?} subfail=${sub_f:-?}"
fi

echo ""
echo "Passed: $PASS  Failed: $FAIL"
[ "$FAIL" -eq 0 ]
