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

# resolve_progress_file fixtures (#550):
#   LAYER  — depth-7 nested project repo; resolver must find the NESTED file.
#   WS     — workspace layout (file at worktree root); resolver returns ROOT.
#   ABSENT — repo dir exists but no progress.md; resolver returns ROOT fallback.
LAYER="$WTBASE/issue-zzzlayer-$N"
WS="$WTBASE/issue-zzzws-$N"
ABSENT="$WTBASE/issue-zzzabsent-$N"
mkdir -p "$LAYER/ui_ws/src/fake_repo/.agent/work-plans/issue-$N" \
         "$WS/.agent/work-plans/issue-$N" \
         "$ABSENT/ui_ws/src/fake_repo"
: > "$LAYER/ui_ws/src/fake_repo/.agent/work-plans/issue-$N/progress.md"
: > "$WS/.agent/work-plans/issue-$N/progress.md"

cleanup() { rm -rf "$A" "$B" "$C" "$LAYER" "$WS" "$ABSENT"; }
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

# 6-8. resolve_progress_file directly (#550). Source the script (the source-guard
# loads only the function defs, no dispatch) in a subshell so the script's
# `set -euo pipefail` does not leak into this test shell, then call the resolver.

# 6. Layer-nested fixture at depth 7 -> resolver returns the NESTED path
#    (guards the maxdepth-7 off-by-one; -maxdepth 6 would miss it).
# shellcheck disable=SC1090  # $DISPATCH is resolved at runtime; sourced for its funcs only
got=$(source "$DISPATCH"; resolve_progress_file "$LAYER" "$N")
want="$LAYER/ui_ws/src/fake_repo/.agent/work-plans/issue-$N/progress.md"
if [ "$got" = "$want" ]; then
    pass "resolve_progress_file finds depth-7 layer-nested progress.md"
else
    fail "resolve_progress_file depth-7 layer (got=$got; want=$want)"
fi

# 7. Workspace fixture (file at worktree root) -> resolver returns the ROOT path.
# shellcheck disable=SC1090
got=$(source "$DISPATCH"; resolve_progress_file "$WS" "$N")
want="$WS/.agent/work-plans/issue-$N/progress.md"
if [ "$got" = "$want" ]; then
    pass "resolve_progress_file returns root path for workspace layout"
else
    fail "resolve_progress_file workspace root (got=$got; want=$want)"
fi

# 8. Absent file (repo dir exists, no progress.md) -> resolver returns ROOT fallback.
# shellcheck disable=SC1090
got=$(source "$DISPATCH"; resolve_progress_file "$ABSENT" "$N")
want="$ABSENT/.agent/work-plans/issue-$N/progress.md"
if [ "$got" = "$want" ]; then
    pass "resolve_progress_file returns root fallback when file absent"
else
    fail "resolve_progress_file absent fallback (got=$got; want=$want)"
fi

echo
echo "dispatch resolution tests: $TEST_PASS passed, $TEST_FAIL failed"
[ "$TEST_FAIL" -eq 0 ]
