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
#
# Every failure case also asserts that **stdout is empty**: the whole point of
# the script's exit-code vocabulary is that a caller never receives an empty
# string as if it were a path (#609).

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REAL_SCRIPTS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"   # .../.agent/scripts
TEST_PASS=0
TEST_FAIL=0

TMPDIR_ROOT=$(mktemp -d /tmp/test_resolve_repo_checkout.XXXXXX)
cleanup() { chmod -R u+rwX "$TMPDIR_ROOT" 2>/dev/null; rm -rf "$TMPDIR_ROOT"; }
trap cleanup EXIT

GIT_ID=(-c user.name=Test -c user.email=test@example.invalid)

# A fake workspace root whose .agent/scripts is the real one. $1 = case name.
make_root() {
    local root="$TMPDIR_ROOT/$1"
    mkdir -p "$root/configs" "$root/.agent"
    ln -s "$REAL_SCRIPTS_DIR" "$root/.agent/scripts"
    echo "$root"
}

# A bare repo with one commit on `main`, usable as a file:// origin, plus an
# optional second branch carrying a distinguishing file. $1 = name,
# $2 = extra branch (optional).
make_origin() {
    local name="$1" branch="${2:-}"
    local work="$TMPDIR_ROOT/origins/$name.work"
    local bare="$TMPDIR_ROOT/origins/$name.git"
    mkdir -p "$work"
    git -C "$work" init -q -b main
    echo "# $name" > "$work/README.md"
    git -C "$work" add README.md
    git -C "$work" "${GIT_ID[@]}" commit -qm "init"
    if [ -n "$branch" ]; then
        git -C "$work" checkout -q -b "$branch"
        echo "$branch" > "$work/BRANCH"
        git -C "$work" add BRANCH
        git -C "$work" "${GIT_ID[@]}" commit -qm "$branch"
        git -C "$work" checkout -q main
    fi
    git clone -q --bare "$work" "$bare"
    echo "$bare"
}

# Add a commit to an origin's `main` so a refresh has something to pick up.
advance_origin() {
    local name="$1"
    local work="$TMPDIR_ROOT/origins/$name.work"
    local bare="$TMPDIR_ROOT/origins/$name.git"
    echo "second" > "$work/SECOND"
    git -C "$work" add SECOND
    git -C "$work" "${GIT_ID[@]}" commit -qm "second"
    git -C "$work" push -q "$bare" main
}

# $1 = root, $2 = repo name, $3 = url, $4 = version (default main)
write_manifest() {
    local root="$1" name="$2" url="$3" version="${4:-main}"
    cat > "$root/configs/test.repos" <<EOF
repositories:
  $name:
    type: git
    url: $url
    version: $version
EOF
}

run_resolver() {
    local root="$1" repo="$2"
    "$root/.agent/scripts/resolve_repo_checkout.sh" "$repo" 2>"$TMPDIR_ROOT/stderr"
}

stderr_text() { cat "$TMPDIR_ROOT/stderr"; }

pass() { echo "✅ PASS: $1"; TEST_PASS=$((TEST_PASS + 1)); }
fail() { echo "❌ FAIL: $1"; TEST_FAIL=$((TEST_FAIL + 1)); }

# --- 1. an existing layer checkout wins --------------------------------------
root=$(make_root layer_wins)
origin=$(make_origin layer_wins)
write_manifest "$root" "demo_repo" "file://$origin"
mkdir -p "$root/layers/main/demo_ws/src/demo_repo"
touch "$root/layers/main/demo_ws/src/demo_repo/package.xml"
out=$(run_resolver "$root" demo_repo); rc=$?
expected="$root/layers/main/demo_ws/src/demo_repo	layer"
if [ "$rc" -eq 0 ] && [ "$out" = "$expected" ]; then
    pass "existing layer checkout resolves as mode 'layer'"
else
    fail "layer checkout: rc=$rc out='$out' (expected 0 / '$expected')"
fi

# --- 1b. an EMPTY layer dir is not a checkout --------------------------------
# `vcs import` leaves an empty src/<repo> behind when it fails partway. Taking
# it as mode 'layer' hands the caller a valid path with nothing in it — an
# empty success. It must fall through to the clone path instead.
root=$(make_root empty_layer_dir)
origin=$(make_origin empty_layer_dir)
write_manifest "$root" "demo_repo" "file://$origin"
mkdir -p "$root/layers/main/demo_ws/src/demo_repo"
out=$(run_resolver "$root" demo_repo); rc=$?
mode=${out##*$'\t'}
if [ "$rc" -eq 0 ] && [ "$mode" = "clone" ] && stderr_text | grep -q "is empty"; then
    pass "empty layers/.../src/<repo> is not accepted as a layer checkout"
else
    fail "empty layer dir: rc=$rc out='$out' stderr='$(stderr_text)'"
fi

# --- 1c. an unreadable layer dir fails loudly, with empty stdout -------------
if [ "$(id -u)" -eq 0 ]; then
    echo "⏭️  SKIP: unreadable-layer-dir case (running as root; chmod cannot deny)"
else
    root=$(make_root unreadable_layer)
    origin=$(make_origin unreadable_layer)
    write_manifest "$root" "demo_repo" "file://$origin"
    mkdir -p "$root/layers/main/demo_ws/src/demo_repo"
    touch "$root/layers/main/demo_ws/src/demo_repo/package.xml"
    chmod 000 "$root/layers/main/demo_ws/src/demo_repo"
    out=$(run_resolver "$root" demo_repo); rc=$?
    chmod 755 "$root/layers/main/demo_ws/src/demo_repo"
    if [ "$rc" -eq 5 ] && [ -z "$out" ] && stderr_text | grep -q "not readable"; then
        pass "unreadable layer checkout → exit 5 with empty stdout, never an empty path at rc 0"
    else
        fail "unreadable layer dir: rc=$rc out='$out' stderr='$(stderr_text)'"
    fi
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
    fail "clone: rc=$rc out='$out' ($(stderr_text))"
fi
CLONE_ROOT="$root"
CLONE_PATH="$path"
CLONE_OUT="$out"

# --- 2b. the clone honours the manifest's version pin ------------------------
# The manifests pin an explicit `version:` for every repo, and it is not always
# the remote's default branch — measured on this host's manifest (35 overlay
# repos, 2026-09-11), `rqt_marine_radar` pins `jazzy` while the remote still
# defaults to `noetic`. A clone that takes the default branch would grade
# different code than a layer checkout of the same repo.
root=$(make_root version_pin)
origin=$(make_origin version_pin jazzy)
write_manifest "$root" "demo_repo" "file://$origin" "jazzy"
out=$(run_resolver "$root" demo_repo); rc=$?
path=${out%%$'\t'*}
if [ "$rc" -eq 0 ] && [ -f "$path/BRANCH" ] && [ "$(cat "$path/BRANCH" 2>/dev/null)" = "jazzy" ]; then
    pass "clone checks out the manifest's pinned version, not the remote default"
else
    fail "version pin: rc=$rc out='$out' BRANCH='$(cat "$path/BRANCH" 2>/dev/null)' ($(stderr_text))"
fi

# --- 3. refreshes an existing clone in place (not a re-clone) ----------------
# A sentinel untracked file survives `fetch` + `reset --hard` but not a
# `rm -rf` + re-clone, so it distinguishes the two paths; the new upstream
# commit proves the tree was actually advanced rather than left stale.
touch "$CLONE_PATH/.sentinel"
advance_origin clones
out2=$(run_resolver "$CLONE_ROOT" demo_repo); rc=$?
if [ "$rc" -eq 0 ] && [ "$out2" = "$CLONE_OUT" ] && [ -f "$CLONE_PATH/.sentinel" ] && [ -f "$CLONE_PATH/SECOND" ]; then
    pass "second run refreshes the existing clone in place and picks up new upstream commits"
else
    fail "refresh: rc=$rc out='$out2' sentinel=$([ -f "$CLONE_PATH/.sentinel" ] && echo yes || echo no) advanced=$([ -f "$CLONE_PATH/SECOND" ] && echo yes || echo no) ($(stderr_text))"
fi

# --- 3b. a cached clone whose origin no longer matches the manifest ----------
# The cache is keyed on repo name alone. Reusing a tree whose origin has since
# changed audits the old remote's code under the new remote's name.
origin2=$(make_origin clones_moved)
write_manifest "$CLONE_ROOT" "demo_repo" "file://$origin2"
out3=$(run_resolver "$CLONE_ROOT" demo_repo); rc=$?
new_url=$(git -C "$CLONE_PATH" remote get-url origin 2>/dev/null)
if [ "$rc" -eq 0 ] && [ "$out3" = "$CLONE_OUT" ] && [ "$new_url" = "file://$origin2" ] \
   && stderr_text | grep -q "re-cloning"; then
    pass "cached clone whose origin no longer matches the manifest url is re-cloned"
else
    fail "url mismatch: rc=$rc origin='$new_url' (expected 'file://$origin2') ($(stderr_text))"
fi

# --- 4. clone failure is exit 5, with a reason and empty stdout --------------
root=$(make_root clone_fails)
write_manifest "$root" "demo_repo" "file://$TMPDIR_ROOT/origins/does_not_exist.git"
out=$(run_resolver "$root" demo_repo); rc=$?
if [ "$rc" -eq 5 ] && [ -z "$out" ] && stderr_text | grep -q "clone of"; then
    pass "failed clone → exit 5 with a reason on stderr and empty stdout"
else
    fail "clone failure: rc=$rc out='$out' (expected 5 / empty), stderr='$(stderr_text)'"
fi

# --- 5. repo absent from a manifest that WAS read is exit 4 ------------------
root=$(make_root not_listed)
origin=$(make_origin not_listed)
write_manifest "$root" "other_repo" "file://$origin"
out=$(run_resolver "$root" demo_repo); rc=$?
if [ "$rc" -eq 4 ] && [ -z "$out" ] && stderr_text | grep -q "not listed in any of the"; then
    pass "repo absent from a populated manifest → exit 4"
else
    fail "not listed: rc=$rc out='$out' (expected 4 / empty), stderr='$(stderr_text)'"
fi

# --- 6. no manifest configured at all is exit 3, NOT exit 4 ------------------
# The false-green path: list_overlay_repos.py prints [] at exit 0 here, which
# a caller would otherwise render as "nothing to audit".
root=$(make_root no_manifest)
out=$(run_resolver "$root" demo_repo); rc=$?
if [ "$rc" -eq 3 ] && [ -z "$out" ] && stderr_text | grep -q "no repo manifest configured"; then
    pass "no manifest configured → exit 3, distinct from 'repo not listed'"
else
    fail "no manifest: rc=$rc out='$out' (expected 3 / empty), stderr='$(stderr_text)'"
fi

# --- 6b. an unparseable manifest is exit 6, NOT exit 3 or 4 -----------------
root=$(make_root bad_manifest)
printf 'repositories:\n  demo_repo:\n   - this is not a mapping\n     : neither is this\n' \
    > "$root/configs/test.repos"
out=$(run_resolver "$root" demo_repo); rc=$?
if [ "$rc" -eq 6 ] && [ -z "$out" ]; then
    pass "unparseable manifest → exit 6, never 'repo not found'"
else
    fail "bad manifest: rc=$rc out='$out' (expected 6 / empty), stderr='$(stderr_text)'"
fi

# --- 6c. a listed repo with no url: is a malformed entry (6), not absent (4) -
root=$(make_root no_url_key)
cat > "$root/configs/test.repos" <<'EOF'
repositories:
  demo_repo:
    type: git
    version: main
EOF
out=$(run_resolver "$root" demo_repo); rc=$?
if [ "$rc" -eq 6 ] && [ -z "$out" ] && stderr_text | grep -q "no 'url:' key"; then
    pass "listed repo with no url: → exit 6 (malformed entry), not exit 4"
else
    fail "no url key: rc=$rc out='$out' (expected 6 / empty), stderr='$(stderr_text)'"
fi

# --- 6d. a url in no recognised form is a malformed entry -------------------
root=$(make_root bad_url)
write_manifest "$root" "demo_repo" "--upload-pack=evil"
out=$(run_resolver "$root" demo_repo); rc=$?
if [ "$rc" -eq 6 ] && [ -z "$out" ] && stderr_text | grep -q "unrecognised url"; then
    pass "url in no recognised form → exit 6, never handed to git clone"
else
    fail "bad url: rc=$rc out='$out' (expected 6 / empty), stderr='$(stderr_text)'"
fi

# --- 7. the same name in two manifests with different urls is ambiguous -----
root=$(make_root ambiguous)
origin=$(make_origin ambiguous)
origin_b=$(make_origin ambiguous_b)
write_manifest "$root" "demo_repo" "file://$origin"
cat > "$root/configs/other.repos" <<EOF
repositories:
  demo_repo:
    type: git
    url: file://$origin_b
    version: main
EOF
out=$(run_resolver "$root" demo_repo); rc=$?
if [ "$rc" -eq 7 ] && [ -z "$out" ] && stderr_text | grep -q "conflicting urls"; then
    pass "same repo name with conflicting urls → exit 7, never a silent first-match"
else
    fail "ambiguous: rc=$rc out='$out' (expected 7 / empty), stderr='$(stderr_text)'"
fi

# --- 8. usage errors ---------------------------------------------------------
root=$(make_root usage)
"$root/.agent/scripts/resolve_repo_checkout.sh" >/dev/null 2>&1; rc=$?
if [ "$rc" -eq 2 ]; then
    pass "missing argument → exit 2"
else
    fail "usage (no arg): rc=$rc (expected 2)"
fi

"$root/.agent/scripts/resolve_repo_checkout.sh" a b >/dev/null 2>&1; rc=$?
if [ "$rc" -eq 2 ]; then
    pass "extra arguments → exit 2"
else
    fail "usage (two args): rc=$rc (expected 2)"
fi

# A repo name reaches a cache path that is handed to `rm -rf`, and vcstool
# manifest keys are paths. Anything but a single path segment is refused.
for bad in "../../etc" "foo/bar" "-rf" "." ".." "" "a b"; do
    out=$("$root/.agent/scripts/resolve_repo_checkout.sh" "$bad" 2>/dev/null); rc=$?
    if [ "$rc" -ne 2 ] || [ -n "$out" ]; then
        fail "repo name '$bad' should be a usage error: rc=$rc out='$out'"
        bad_name_ok=0
    fi
done
if [ "${bad_name_ok:-1}" -eq 1 ]; then
    pass "a repo name that is not a single path segment → exit 2 (never reaches rm -rf)"
fi

# --- 9. run from a git worktree, resolve against the MAIN root ---------------
# The primary environment: a fully set-up host, the sweep invoked from a
# worktree. `layers/` and `configs/manifest` live only in the main checkout, so
# a worktree-relative resolution would find nothing and report the workspace
# unconfigured.
root=$(make_root worktree)
origin=$(make_origin worktree)
write_manifest "$root" "demo_repo" "file://$origin"
mkdir -p "$root/layers/main/demo_ws/src/demo_repo"
touch "$root/layers/main/demo_ws/src/demo_repo/package.xml"
git -C "$root" init -q -b main
git -C "$root" add -A >/dev/null 2>&1
git -C "$root" "${GIT_ID[@]}" commit -qm "workspace" >/dev/null 2>&1
wt="$TMPDIR_ROOT/worktree_wt"
if git -C "$root" "${GIT_ID[@]}" worktree add -q -b wt "$wt" >/dev/null 2>&1; then
    out=$("$wt/.agent/scripts/resolve_repo_checkout.sh" demo_repo 2>"$TMPDIR_ROOT/stderr"); rc=$?
    expected="$root/layers/main/demo_ws/src/demo_repo	layer"
    if [ "$rc" -eq 0 ] && [ "$out" = "$expected" ]; then
        pass "run from a worktree, resolves against the MAIN workspace root"
    else
        fail "worktree: rc=$rc out='$out' (expected 0 / '$expected'), stderr='$(stderr_text)'"
    fi
else
    fail "worktree: could not create the test worktree"
fi

echo ""
echo "Passed: $TEST_PASS  Failed: $TEST_FAIL"
[ "$TEST_FAIL" -eq 0 ]
