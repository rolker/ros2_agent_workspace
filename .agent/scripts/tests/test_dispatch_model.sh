#!/bin/bash
# .agent/scripts/tests/test_dispatch_model.sh
# Unit-tests the pure model-mapping helpers in dispatch_subagent.sh:
#   - entry_type_model (#539): a raw --prompt-file dispatch with no --skill must
#     pick Opus for reasoning-heavy entry types, not silently default to Sonnet.
#   - model_display (#540): alias -> version-less display name the dispatcher
#     hands the sub-agent to stamp (so it never invents "Opus 4.6").
# Extracts just those functions (no worktree / no dispatch) so it runs in CI.
#
# Run: bash .agent/scripts/tests/test_dispatch_model.sh

set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DISPATCH="$SCRIPT_DIR/../dispatch_subagent.sh"
TEST_PASS=0
TEST_FAIL=0

# Pull the self-contained mapping functions out and define them here — each runs
# from its `name() {` line to the next column-0 `}` (the case's inner braces are
# indented, so `^}` only matches the function close).
eval "$(sed -n '/^entry_type_model() {/,/^}/p; /^model_display() {/,/^}/p; /^skill_model() {/,/^}/p' "$DISPATCH")"

check() { # check <desc> <expected> <actual>
    if [ "$2" = "$3" ]; then echo "PASS: $1"; TEST_PASS=$((TEST_PASS + 1))
    else echo "FAIL: $1 (expected '$2', got '$3')"; TEST_FAIL=$((TEST_FAIL + 1)); fi
}

# #539 — entry-type -> model
check "Implementation -> opus"            opus   "$(entry_type_model 'Implementation')"
check "Local Review (Pre-Push) -> opus"   opus   "$(entry_type_model 'Local Review (Pre-Push)')"
check "Integrated Review -> opus"         opus   "$(entry_type_model 'Integrated Review')"
check "Plan Review -> opus"               opus   "$(entry_type_model 'Plan Review')"
check "Issue Review -> sonnet"            sonnet "$(entry_type_model 'Issue Review')"
check "Plan Authored -> sonnet"           sonnet "$(entry_type_model 'Plan Authored')"
check "unknown entry-type -> sonnet"      sonnet "$(entry_type_model 'Whatever')"

# Drift guard: each entry type's model MUST equal the model of the skill that
# writes it (the two case statements must not diverge — see #539 / the
# keep-in-sync comment on entry_type_model).
check "parity Local Review (Pre-Push) == review-code"  "$(skill_model review-code)"     "$(entry_type_model 'Local Review (Pre-Push)')"
check "parity Integrated Review == triage-reviews"     "$(skill_model triage-reviews)"  "$(entry_type_model 'Integrated Review')"
check "parity Implementation == address-findings"      "$(skill_model address-findings)" "$(entry_type_model 'Implementation')"
check "parity Plan Review == review-plan"              "$(skill_model review-plan)"     "$(entry_type_model 'Plan Review')"
check "parity Issue Review == review-issue"            "$(skill_model review-issue)"    "$(entry_type_model 'Issue Review')"
check "parity Plan Authored == plan-task"             "$(skill_model plan-task)"       "$(entry_type_model 'Plan Authored')"

# #540 — version-less display names
check "display opus"                      "Claude Opus"   "$(model_display opus)"
check "display sonnet"                    "Claude Sonnet" "$(model_display sonnet)"
check "display passthrough"               "custom-x"      "$(model_display 'custom-x')"

echo
echo "dispatch model-mapping tests: $TEST_PASS passed, $TEST_FAIL failed"
[ "$TEST_FAIL" -eq 0 ]
