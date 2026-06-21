#!/bin/bash
# .agent/scripts/tests/test_dispatch_worktree_resolution.sh
# Regression test for #526: dispatch_subagent.sh worktree resolution must FAIL
# LOUD when >1 worktree matches a bare issue number (cross-repo collision),
# and --repo-slug must disambiguate. Uses --mode in-process so resolution runs
# but no container is launched. Creates two fake layer worktrees under the
# resolved workspace root, then cleans them up.
#
# Run: bash .agent/scripts/tests/test_dispatch_worktree_resolution.sh

set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DISPATCH="$SCRIPT_DIR/../dispatch_subagent.sh"

# Resolve ROOT_DIR exactly as dispatch_subagent.sh does (strip worktree prefix).
# This test lives one dir deeper than dispatch_subagent.sh (tests/), so it walks
# up THREE levels (tests -> scripts -> .agent -> root), not two. Getting this
# wrong points the fake worktrees at .agent/layers/... while the script looks
# under <root>/layers/... — a mismatch the worktree-prefix strip masks locally
# (when run from inside a worktree) but CI exposes (#548).
ROOT_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
case "$ROOT_DIR" in
    *"/.workspace-worktrees/"*) ROOT_DIR="${ROOT_DIR%%/.workspace-worktrees/*}" ;;
    *"/layers/worktrees/"*)     ROOT_DIR="${ROOT_DIR%%/layers/worktrees/*}" ;;
esac

N=999990
WTBASE="$ROOT_DIR/layers/worktrees"
A="$WTBASE/issue-zzztesta-$N"
B="$WTBASE/issue-zzztestb-$N"
# Underscore dir name, as worktree_create.sh would produce from a hyphenated slug.
C="$WTBASE/issue-zzz_test_c-$N"
mkdir -p "$A/.agent/work-plans/issue-$N" "$B/.agent/work-plans/issue-$N" \
         "$C/.agent/work-plans/issue-$N"
cleanup() { rm -rf "$A" "$B" "$C"; }
trap cleanup EXIT

TEST_PASS=0
TEST_FAIL=0
pass() { echo "PASS: $1"; TEST_PASS=$((TEST_PASS + 1)); }
fail() { echo "FAIL: $1"; TEST_FAIL=$((TEST_FAIL + 1)); }

# 1. Bare --issue with two matches -> fail loud (non-zero, names both candidates).
out=$("$DISPATCH" --mode in-process --issue "$N" --skill review-code 2>&1); rc=$?
if [ "$rc" -ne 0 ] \
    && printf '%s' "$out" | grep -q "refusing to guess" \
    && printf '%s' "$out" | grep -q "zzztesta" \
    && printf '%s' "$out" | grep -q "zzztestb"; then
    pass "ambiguous bare --issue fails loud, listing both candidates"
else
    fail "ambiguous bare --issue fails loud (rc=$rc; out=$out)"
fi

# 2. --repo-slug resolves the exact match (no multi-match error; reaches handoff).
out=$("$DISPATCH" --mode in-process --issue "$N" --repo-slug zzztesta --skill review-code 2>&1); rc=$?
if [ "$rc" -eq 0 ] && ! printf '%s' "$out" | grep -q "refusing to guess"; then
    pass "--repo-slug disambiguates (resolves, no multi-match error)"
else
    fail "--repo-slug disambiguates (rc=$rc; out=$out)"
fi

# 3. --repo-slug with no matching worktree -> clear no-worktree error (not a guess).
out=$("$DISPATCH" --mode in-process --issue "$N" --repo-slug nonexistent --skill review-code 2>&1); rc=$?
if [ "$rc" -ne 0 ] && printf '%s' "$out" | grep -q "no worktree found"; then
    pass "--repo-slug with no match errors cleanly"
else
    fail "--repo-slug with no match errors cleanly (rc=$rc; out=$out)"
fi

# 4. Hyphenated --repo-slug is sanitized (- -> _) to match the dir name.
out=$("$DISPATCH" --mode in-process --issue "$N" --repo-slug zzz-test-c --skill review-code 2>&1); rc=$?
if [ "$rc" -eq 0 ] && ! printf '%s' "$out" | grep -qE "refusing to guess|no worktree found"; then
    pass "hyphenated --repo-slug is sanitized and resolves"
else
    fail "hyphenated --repo-slug is sanitized and resolves (rc=$rc; out=$out)"
fi

# 5. Non-numeric --issue is rejected.
out=$("$DISPATCH" --mode in-process --issue "../etc" --skill review-code 2>&1); rc=$?
if [ "$rc" -ne 0 ] && printf '%s' "$out" | grep -q "must be a number"; then
    pass "non-numeric --issue rejected"
else
    fail "non-numeric --issue rejected (rc=$rc; out=$out)"
fi

echo
echo "dispatch resolution tests: $TEST_PASS passed, $TEST_FAIL failed"
[ "$TEST_FAIL" -eq 0 ]
