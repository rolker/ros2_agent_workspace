#!/bin/bash
# .agent/scripts/tests/test_resolve_repo_checkout.sh
# Tests for .agent/scripts/resolve_repo_checkout.sh
#
# Hermetic, per this suite's contract (temp sandboxes, no network). Each case
# builds a throwaway workspace root: a temp dir holding `.agent/scripts` as a
# symlink to the real one (bash's `cd` is logical, and Python's `abspath` does
# not resolve symlinks, so both the script's and list_overlay_repos.py's root
# walks land on the fake root) plus a `configs/*.repos` manifest. Only
# `.agent/scripts` is linked, never the whole `.agent/` — the resolver's clone
# cache lives at `<root>/.agent/scratchpad/`, which must stay inside the temp
# tree so the cases cannot see each other's clones or touch the real
# workspace. The clone cases point the manifest at a bare git repo created in
# the same temp tree and served over `file://` — no remote is ever contacted.

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REAL_SCRIPTS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"   # .../.agent/scripts
TEST_PASS=0
TEST_FAIL=0

TMPDIR_ROOT=$(mktemp -d /tmp/test_resolve_repo_checkout.XXXXXX)
cleanup() { rm -rf "$TMPDIR_ROOT"; }
trap cleanup EXIT

# A fake workspace root whose .agent/scripts is the real one. $1 = case name.
make_root() {
    local root="$TMPDIR_ROOT/$1"
    mkdir -p "$root/configs" "$root/.agent"
    ln -s "$REAL_SCRIPTS_DIR" "$root/.agent/scripts"
    echo "$root"
}

# A bare repo with one commit, usable as a file:// origin. $1 = name.
make_origin() {
    local name="$1"
    local work="$TMPDIR_ROOT/origins/$name.work"
    local bare="$TMPDIR_ROOT/origins/$name.git"
    mkdir -p "$work"
    git -C "$work" init -q -b main
    echo "# $name" > "$work/README.md"
    git -C "$work" add README.md
    git -C "$work" -c user.name=Test -c user.email=test@example.invalid commit -qm "init"
    git clone -q --bare "$work" "$bare"
    echo "$bare"
}

write_manifest() {
    local root="$1" name="$2" url="$3"
    cat > "$root/configs/test.repos" <<EOF
repositories:
  $name:
    type: git
    url: $url
    version: main
EOF
}

run_resolver() {
    local root="$1" repo="$2"
    "$root/.agent/scripts/resolve_repo_checkout.sh" "$repo" 2>"$TMPDIR_ROOT/stderr"
}

pass() { echo "✅ PASS: $1"; TEST_PASS=$((TEST_PASS + 1)); }
fail() { echo "❌ FAIL: $1"; TEST_FAIL=$((TEST_FAIL + 1)); }

# --- 1. an existing layer checkout wins --------------------------------------
root=$(make_root layer_wins)
origin=$(make_origin layer_wins)
write_manifest "$root" "demo_repo" "file://$origin"
mkdir -p "$root/layers/main/demo_ws/src/demo_repo"
out=$(run_resolver "$root" demo_repo); rc=$?
expected="$root/layers/main/demo_ws/src/demo_repo	layer"
if [ "$rc" -eq 0 ] && [ "$out" = "$expected" ]; then
    pass "existing layer checkout resolves as mode 'layer'"
else
    fail "layer checkout: rc=$rc out='$out' (expected 0 / '$expected')"
fi

# --- 2. clones when layers/ is absent ----------------------------------------
root=$(make_root clones)
origin=$(make_origin clones)
write_manifest "$root" "demo_repo" "file://$origin"
out=$(run_resolver "$root" demo_repo); rc=$?
path=${out%%$'\t'*}
mode=${out##*$'\t'}
if [ "$rc" -eq 0 ] && [ "$mode" = "clone" ] && [ -f "$path/README.md" ]; then
    pass "no layer checkout → shallow clone, mode 'clone'"
else
    fail "clone: rc=$rc out='$out' ($(cat "$TMPDIR_ROOT/stderr"))"
fi
CLONE_ROOT="$root"

# --- 3. refreshes an existing clone ------------------------------------------
out2=$(run_resolver "$CLONE_ROOT" demo_repo); rc=$?
if [ "$rc" -eq 0 ] && [ "$out2" = "$out" ]; then
    pass "second run refreshes the existing clone in place"
else
    fail "refresh: rc=$rc out='$out2' (expected 0 / '$out')"
fi

# --- 4. clone failure is exit 5, with a reason -------------------------------
root=$(make_root clone_fails)
write_manifest "$root" "demo_repo" "file://$TMPDIR_ROOT/origins/does_not_exist.git"
out=$(run_resolver "$root" demo_repo); rc=$?
if [ "$rc" -eq 5 ] && grep -q "clone of" "$TMPDIR_ROOT/stderr"; then
    pass "failed clone → exit 5 with a reason on stderr"
else
    fail "clone failure: rc=$rc (expected 5), stderr='$(cat "$TMPDIR_ROOT/stderr")'"
fi

# --- 5. repo absent from a manifest that WAS read is exit 4 ------------------
root=$(make_root not_listed)
origin=$(make_origin not_listed)
write_manifest "$root" "other_repo" "file://$origin"
out=$(run_resolver "$root" demo_repo); rc=$?
if [ "$rc" -eq 4 ] && grep -q "not listed in any of the" "$TMPDIR_ROOT/stderr"; then
    pass "repo absent from a populated manifest → exit 4"
else
    fail "not listed: rc=$rc (expected 4), stderr='$(cat "$TMPDIR_ROOT/stderr")'"
fi

# --- 6. no manifest configured at all is exit 3, NOT exit 4 ------------------
# The false-green path: list_overlay_repos.py prints [] at exit 0 here, which
# a caller would otherwise render as "nothing to audit".
root=$(make_root no_manifest)
out=$(run_resolver "$root" demo_repo); rc=$?
if [ "$rc" -eq 3 ] && grep -q "no repo manifest configured" "$TMPDIR_ROOT/stderr"; then
    pass "no manifest configured → exit 3, distinct from 'repo not listed'"
else
    fail "no manifest: rc=$rc (expected 3), stderr='$(cat "$TMPDIR_ROOT/stderr")'"
fi

# --- 7. usage error ----------------------------------------------------------
root=$(make_root usage)
"$root/.agent/scripts/resolve_repo_checkout.sh" >/dev/null 2>&1; rc=$?
if [ "$rc" -eq 2 ]; then
    pass "missing argument → exit 2"
else
    fail "usage: rc=$rc (expected 2)"
fi

echo ""
echo "Passed: $TEST_PASS  Failed: $TEST_FAIL"
[ "$TEST_FAIL" -eq 0 ]
