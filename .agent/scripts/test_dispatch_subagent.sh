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
mkdir -p "$FAKE_WT/.agent/work-plans/issue-$TEST_ISSUE"
trap 'rm -rf "$FAKE_WT"' EXIT

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

# --- in-process handoff content ---
assert_emits "embeds git -c identity literals" 'git -c user.name="Claude Code Agent" -c user.email="roland+claude-code@ccom.unh.edu"' \
    --mode in-process --issue "$TEST_ISSUE" --skill review-code
assert_emits "embeds the no-push / no--no-verify rule" 'Never use `--no-verify`' \
    --mode in-process --issue "$TEST_ISSUE" --skill review-code
assert_emits "embeds the progress.md exit contract" ".agent/work-plans/issue-$TEST_ISSUE/progress.md" \
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

echo ""
echo "Passed: $PASS  Failed: $FAIL"
[ "$FAIL" -eq 0 ]
