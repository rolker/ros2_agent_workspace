#!/bin/bash
# .agent/scripts/tests/test_dlog.sh
# Tests the prompt-free deployment log appender (.agent/scripts/dlog.sh).
#
# The whole point of dlog.sh is to be BOTH prompt-free (one allowlist entry)
# AND accurately stamped (a measured `date`, not a typed time) — resolving
# #515 (timestamp accuracy) + #516 (per-entry prompts). These cases pin that
# it appends a well-formed, timestamped entry and fails loudly on bad usage.
#
# Run: bash .agent/scripts/tests/test_dlog.sh

set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DLOG="$SCRIPT_DIR/../dlog.sh"
TEST_PASS=0
TEST_FAIL=0

pass() { echo "PASS: $1"; TEST_PASS=$((TEST_PASS + 1)); }
fail() { echo "FAIL: $1"; TEST_FAIL=$((TEST_FAIL + 1)); }

TMPD="$(mktemp -d)"
trap 'rm -rf "$TMPD"' EXIT
LOG="$TMPD/2026-01-01_dev_logs.md"
: > "$LOG"

# 1. appends a well-formed entry stamped with a real `date` value
out=$("$DLOG" "$LOG" "boat in water" 2>&1); rc=$?
if [ "$rc" -eq 0 ] \
    && grep -qE '^\*\*[0-9]{4}-[0-9]{2}-[0-9]{2} [0-9]{2}:[0-9]{2} [+-][0-9]{2}:[0-9]{2}\*\* ' "$LOG" \
    && grep -q 'boat in water' "$LOG"; then
    pass "appends a timestamped, well-formed entry"
else
    fail "appends a timestamped, well-formed entry (rc=$rc, out=$out)"
fi

# 2. multi-arg message is joined, not dropped
"$DLOG" "$LOG" "in water" "(operator-reported ~12:35)" > /dev/null 2>&1
if grep -qF 'in water (operator-reported ~12:35)' "$LOG"; then
    pass "joins a multi-arg message"
else
    fail "joins a multi-arg message"
fi

# 3. appends rather than truncates (both entries present)
if [ "$(grep -cE '^\*\*[0-9]' "$LOG")" -eq 2 ]; then
    pass "appends rather than truncates"
else
    fail "appends rather than truncates"
fi

# 4. logfile but no message -> usage error, exit 2
"$DLOG" "$LOG" > /dev/null 2>&1
[ "$?" -eq 2 ] && pass "missing message exits 2" || fail "missing message exits 2"

# 5. no args -> usage error, exit 2
"$DLOG" > /dev/null 2>&1
[ "$?" -eq 2 ] && pass "no args exits 2" || fail "no args exits 2"

# 6. nonexistent log directory -> error, exit 1 (fail loud, don't create stray dirs)
"$DLOG" "$TMPD/nope/x.md" "msg" > /dev/null 2>&1
[ "$?" -eq 1 ] && pass "nonexistent log dir exits 1" || fail "nonexistent log dir exits 1"

echo
echo "dlog.sh tests: $TEST_PASS passed, $TEST_FAIL failed"
[ "$TEST_FAIL" -eq 0 ]
