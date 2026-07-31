#!/bin/bash
# .agent/scripts/tests/test_progress_append.sh
# Tests the prompt-free progress.md appender (.agent/scripts/progress_append.sh, #594).
#
# The script's whole contract is scope discipline: derived target path, fixed
# commit message, only-that-file commits, fail-loud identity. These cases pin
# each of those properties plus the failure modes.
#
# Run: bash .agent/scripts/tests/test_progress_append.sh

set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PA="$SCRIPT_DIR/../progress_append.sh"
TEST_PASS=0
TEST_FAIL=0

pass() { echo "PASS: $1"; TEST_PASS=$((TEST_PASS + 1)); }
fail() { echo "FAIL: $1"; TEST_FAIL=$((TEST_FAIL + 1)); }

TMPD="$(mktemp -d)"
trap 'rm -rf "$TMPD"' EXIT
REPO="$TMPD/repo"
mkdir -p "$REPO"
git -C "$REPO" init -q -b main
export AGENT_NAME="Test Agent"
export AGENT_EMAIL="test+agent@example.com"
PROG=".agent/work-plans/issue-7/progress.md"

# 1. creates file with frontmatter + title heading, appends entry, commits with
#    the fixed message and the agent identity
out=$(printf '## Local Review (Pre-Push)\n**Status**: complete\n' \
      | "$PA" -C "$REPO" 7 --title "Fix the thing" 2>&1); rc=$?
if [ "$rc" -eq 0 ] \
    && grep -q '^issue: 7$' "$REPO/$PROG" \
    && grep -q '^# Issue #7 — Fix the thing$' "$REPO/$PROG" \
    && grep -q '^## Local Review (Pre-Push)$' "$REPO/$PROG" \
    && [ "$(git -C "$REPO" log -1 --format=%s)" = "progress: local review (pre-push) for #7" ] \
    && [ "$(git -C "$REPO" log -1 --format='%an <%ae>')" = "Test Agent <test+agent@example.com>" ]; then
    pass "creates file (frontmatter+title), appends, commits with fixed message + agent identity"
else
    fail "creates file (frontmatter+title), appends, commits (rc=$rc, out=$out)"
fi

# 2. second append: both entries present, file not truncated, new commit
printf '## Implementation\n**Status**: complete\n' | "$PA" -C "$REPO" 7 > /dev/null 2>&1
if grep -q '^## Local Review (Pre-Push)$' "$REPO/$PROG" \
    && grep -q '^## Implementation$' "$REPO/$PROG" \
    && [ "$(git -C "$REPO" log -1 --format=%s)" = "progress: implementation for #7" ]; then
    pass "appends rather than truncates; per-entry commit"
else
    fail "appends rather than truncates; per-entry commit"
fi

# 3. scope: a dirty index is untouched — unrelated staged file stays staged
#    and out of the progress commit
echo "unrelated" > "$REPO/unrelated.txt"
git -C "$REPO" add unrelated.txt
printf '## Issue Review\nbody\n' | "$PA" -C "$REPO" 7 > /dev/null 2>&1
committed=$(git -C "$REPO" show --name-only --format= HEAD)
if [ "$committed" = "$PROG" ] \
    && git -C "$REPO" diff --cached --name-only | grep -q '^unrelated.txt$'; then
    pass "commits only the progress file; unrelated staged file untouched"
else
    fail "commits only the progress file (committed: $committed)"
fi
git -C "$REPO" reset -q unrelated.txt && rm -f "$REPO/unrelated.txt"

# 4. non-numeric issue number -> exit 2, no commit
head_before=$(git -C "$REPO" rev-parse HEAD)
printf '## X\n' | "$PA" -C "$REPO" seven > /dev/null 2>&1
rc=$?
[ "$rc" -eq 2 ] && [ "$(git -C "$REPO" rev-parse HEAD)" = "$head_before" ] \
    && pass "non-numeric issue exits 2" || fail "non-numeric issue exits 2 (rc=$rc)"

# 5. empty stdin -> exit 2, file untouched
"$PA" -C "$REPO" 7 < /dev/null > /dev/null 2>&1
rc=$?
[ "$rc" -eq 2 ] && pass "empty stdin exits 2" || fail "empty stdin exits 2 (rc=$rc)"

# 6. entry without '## ' heading -> exit 2
printf 'not a heading\n' | "$PA" -C "$REPO" 7 > /dev/null 2>&1
rc=$?
[ "$rc" -eq 2 ] && pass "missing '##' heading exits 2" || fail "missing '##' heading exits 2 (rc=$rc)"

# 7. identity unset (env cleared, no args) -> exit 2, fail loud, no fallback commit
out=$(printf '## X Review\n' | env -u AGENT_NAME -u AGENT_EMAIL "$PA" -C "$REPO" 7 2>&1)
rc=$?
if [ "$rc" -eq 2 ] && printf '%s' "$out" | grep -qi 'identity'; then
    pass "unset identity fails loud (exit 2)"
else
    fail "unset identity fails loud (rc=$rc, out=$out)"
fi

# 8. --name/--email args substitute for env identity
out=$(printf '## Plan Review\nx\n' \
      | env -u AGENT_NAME -u AGENT_EMAIL "$PA" -C "$REPO" 7 --name "Arg Agent" --email "arg@example.com" 2>&1)
rc=$?
if [ "$rc" -eq 0 ] && [ "$(git -C "$REPO" log -1 --format=%an)" = "Arg Agent" ]; then
    pass "--name/--email args substitute for env identity"
else
    fail "--name/--email args substitute for env identity (rc=$rc, out=$out)"
fi

# 9. new file without --title gets the bare heading
printf '## Issue Review\nx\n' | "$PA" -C "$REPO" 8 > /dev/null 2>&1
if grep -q '^# Issue #8$' "$REPO/.agent/work-plans/issue-8/progress.md"; then
    pass "no --title -> bare '# Issue #N' heading"
else
    fail "no --title -> bare '# Issue #N' heading"
fi

# 10. outside a git repo -> exit 2
NOREPO="$TMPD/norepo"
mkdir -p "$NOREPO"
printf '## X Review\n' | "$PA" -C "$NOREPO" 7 > /dev/null 2>&1
rc=$?
[ "$rc" -eq 2 ] && pass "non-repo dir exits 2" || fail "non-repo dir exits 2 (rc=$rc)"

# 11. CRLF / padded heading: trailing \r and spaces must not leak into the
#     commit subject (entry type is trimmed).
printf '## Plan Authored  \r\nbody\n' | "$PA" -C "$REPO" 9 > /dev/null 2>&1
subj=$(git -C "$REPO" log -1 --format=%s)
if [ "$subj" = "progress: plan authored for #9" ]; then
    pass "trailing CR/space in heading trimmed from commit subject"
else
    fail "trailing CR/space in heading trimmed (subj='$subj')"
fi

# 12. idempotency: if the entry is already the file tail, a re-run must not
#     double-append AND must exit 0 — the already-committed success path is not
#     a git failure (exit 3). The rc assertion pins that exit semantics; without
#     it the entry-count check alone passes even on the spurious exit-3.
PROG9="$REPO/.agent/work-plans/issue-9/progress.md"
before=$(grep -c '^## Implementation$' "$PROG9")
printf '## Implementation\nsame body\n' | "$PA" -C "$REPO" 9 > /dev/null 2>&1   # first append+commit
printf '## Implementation\nsame body\n' | "$PA" -C "$REPO" 9 > /dev/null 2>&1   # identical re-run
rc=$?
after=$(grep -c '^## Implementation$' "$PROG9")
if [ "$before" -eq 0 ] && [ "$after" -eq 1 ] && [ "$rc" -eq 0 ]; then
    pass "identical entry as file tail is not re-appended, re-run exits 0 (idempotent replay)"
else
    fail "idempotent replay (before=$before after=$after rc=$rc)"
fi

echo ""
echo "test_progress_append: $TEST_PASS passed, $TEST_FAIL failed"
[ "$TEST_FAIL" -eq 0 ]
