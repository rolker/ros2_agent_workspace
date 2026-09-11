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
# The manifest-fallback cases do the same one level up: the tracked bootstrap
# pointer is a plain string, and `WORKSPACE_MANIFEST_GIT_BASE` points the
# derived clone url at a local fixture, so cloning the MANIFEST repo is
# exercised without the network either.
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

# --- 6e. a version: in no ref-safe form is a malformed entry -----------------
# `version:` reaches `git clone --branch` and `git fetch <refspec>`. A value
# starting with `-` is read as an OPTION — `--upload-pack=<script>` executes
# it — and because a fetch that consumed its argument as a flag still
# succeeds, the run used to exit 0 reporting mode `clone` with the pin
# silently unhonoured. Every one of these must be refused before git sees it.
sentinel="$TMPDIR_ROOT/upload_pack_ran"
rm -f "$sentinel"
cat > "$TMPDIR_ROOT/evil.sh" <<EOF
#!/bin/bash
touch "$sentinel"
exit 1
EOF
chmod +x "$TMPDIR_ROOT/evil.sh"
bad_version_ok=1
bad_version_n=0
for badver in "--upload-pack=$TMPDIR_ROOT/evil.sh" "-x" "foo..bar" "has space" "main;rm -rf /" 'v$(touch /tmp/nope)' "refs/heads/@{-1}"; do
    bad_version_n=$((bad_version_n + 1))
    root=$(make_root "bad_version_$bad_version_n")
    origin=$(make_origin "bad_version_$bad_version_n")
    write_manifest "$root" "demo_repo" "file://$origin" "$badver"
    out=$(run_resolver "$root" demo_repo); rc=$?
    if [ "$rc" -ne 6 ] || [ -n "$out" ]; then
        fail "version '$badver' should be a malformed entry: rc=$rc out='$out' stderr='$(stderr_text)'"
        bad_version_ok=0
    fi
done
if [ -e "$sentinel" ]; then
    fail "version '--upload-pack=...' reached git: the helper script was EXECUTED"
    bad_version_ok=0
fi
if [ "$bad_version_ok" -eq 1 ]; then
    pass "a version: that is not a SHA or a ref-safe name → exit 6, never handed to git"
fi

# --- 6f. a full-SHA pin is honoured, and a SHA that is not there fails loud --
# The SHA path is the one `clone --branch` cannot serve, so it goes through
# clone + fetch + detach — the path where an unhonoured pin used to exit 0.
root=$(make_root sha_pin)
origin=$(make_origin sha_pin)
sha=$(git -C "$TMPDIR_ROOT/origins/sha_pin.work" rev-parse HEAD)
write_manifest "$root" "demo_repo" "file://$origin" "$sha"
out=$(run_resolver "$root" demo_repo); rc=$?
path=${out%%$'\t'*}
if [ "$rc" -eq 0 ] && [ "$(git -C "$path" rev-parse HEAD 2>/dev/null)" = "$sha" ]; then
    pass "a full-SHA version: pin is fetched and checked out at that commit"
else
    fail "sha pin: rc=$rc out='$out' head='$(git -C "$path" rev-parse HEAD 2>/dev/null)' expected '$sha' ($(stderr_text))"
fi

root=$(make_root sha_pin_absent)
origin=$(make_origin sha_pin_absent)
write_manifest "$root" "demo_repo" "file://$origin" "0123456789abcdef0123456789abcdef01234567"
out=$(run_resolver "$root" demo_repo); rc=$?
if [ "$rc" -eq 5 ] && [ -z "$out" ] && stderr_text | grep -q "failed"; then
    pass "a SHA pin the remote does not have → exit 5, never rc 0 with the pin unhonoured"
else
    fail "absent sha pin: rc=$rc out='$out' (expected 5 / empty), stderr='$(stderr_text)'"
fi

# --- 6g. no version: at all refreshes against the remote's HEAD --------------
# FETCH_REF falls back to HEAD when the manifest pins nothing; that branch is
# only reached on the second (refresh) run.
root=$(make_root no_version)
origin=$(make_origin no_version)
cat > "$root/configs/test.repos" <<EOF
repositories:
  demo_repo:
    type: git
    url: file://$origin
EOF
out=$(run_resolver "$root" demo_repo); rc=$?
path=${out%%$'\t'*}
advance_origin no_version
out2=$(run_resolver "$root" demo_repo); rc2=$?
if [ "$rc" -eq 0 ] && [ "$rc2" -eq 0 ] && [ -f "$path/SECOND" ]; then
    pass "a manifest with no version: clones, then refreshes against the remote's HEAD"
else
    fail "no version: rc=$rc rc2=$rc2 advanced=$([ -f "$path/SECOND" ] && echo yes || echo no) ($(stderr_text))"
fi

# --- 6h. no configs/manifest at all: clone the MANIFEST repo, then the repo ---
# `configs/manifest` is a symlink into the layer tree, so a host without
# `layers/` has no manifests either — "it clones what it needs" has to cover
# the manifest first. The bootstrap pointer is a tracked file, and
# WORKSPACE_MANIFEST_GIT_BASE points the derived clone url at a local fixture,
# so this stays hermetic: no network, ever.
make_manifest_origin() {
    # $1 = owner, $2 = repo, $3 = demo repo url to declare
    local owner="$1" name="$2" demo_url="$3"
    local work="$TMPDIR_ROOT/manifest_work/$owner/$name"
    local bare="$TMPDIR_ROOT/manifest_origins/$owner/$name.git"
    mkdir -p "$work/config/repos" "$(dirname "$bare")"
    cat > "$work/config/repos/core.repos" <<EOF
repositories:
  demo_repo:
    type: git
    url: $demo_url
    version: main
EOF
    git -C "$work" init -q -b main
    git -C "$work" add -A
    git -C "$work" "${GIT_ID[@]}" commit -qm "manifest"
    git clone -q --bare "$work" "$bare"
}

root=$(make_root manifest_fallback)
origin=$(make_origin manifest_fallback)
make_manifest_origin testowner testmanifest "file://$origin"
echo "https://raw.githubusercontent.com/testowner/testmanifest/main/config/bootstrap.yaml" \
    > "$root/configs/project_bootstrap.url"
out=$(WORKSPACE_MANIFEST_GIT_BASE="file://$TMPDIR_ROOT/manifest_origins" \
      "$root/.agent/scripts/resolve_repo_checkout.sh" demo_repo 2>"$TMPDIR_ROOT/stderr"); rc=$?
path=${out%%$'\t'*}
mode=${out##*$'\t'}
if [ "$rc" -eq 0 ] && [ "$mode" = "clone" ] && [ -f "$path/README.md" ]; then
    pass "no configs/manifest: the manifest repo is cloned from the bootstrap pointer, then the repo"
else
    fail "manifest fallback: rc=$rc out='$out' ($(stderr_text))"
fi

# A second run reuses the cached manifest clone rather than failing on it.
out2=$(WORKSPACE_MANIFEST_GIT_BASE="file://$TMPDIR_ROOT/manifest_origins" \
       "$root/.agent/scripts/resolve_repo_checkout.sh" demo_repo 2>"$TMPDIR_ROOT/stderr"); rc=$?
if [ "$rc" -eq 0 ] && [ "$out2" = "$out" ]; then
    pass "a cached manifest clone is refreshed and reused on the next run"
else
    fail "manifest fallback (cached): rc=$rc out='$out2' ($(stderr_text))"
fi

# --- 6h2. repointing the bootstrap pointer wins over the cached manifest -----
# The manifest cache is keyed on the repo NAME, so a pointer moved to a
# different owner's manifest of the same name lands on the existing clone.
# Reusing it would enumerate the whole rotation from a manifest the workspace
# no longer points at, at exit 0 — the resolver already refuses the same thing
# one level down for project repos.
origin2=$(make_origin manifest_fallback_repointed)
make_manifest_origin otherowner testmanifest "file://$origin2"
echo "https://raw.githubusercontent.com/otherowner/testmanifest/main/config/bootstrap.yaml" \
    > "$root/configs/project_bootstrap.url"
out3=$(WORKSPACE_MANIFEST_GIT_BASE="file://$TMPDIR_ROOT/manifest_origins" \
       "$root/.agent/scripts/resolve_repo_checkout.sh" demo_repo 2>"$TMPDIR_ROOT/stderr"); rc=$?
path3=${out3%%$'\t'*}
cached_manifest_url=$(git -C "$root/.agent/scratchpad/manifest-repo/testmanifest" \
                      remote get-url origin 2>/dev/null)
if [ "$rc" -eq 0 ] \
   && [ "$cached_manifest_url" = "file://$TMPDIR_ROOT/manifest_origins/otherowner/testmanifest.git" ] \
   && grep -q "manifest_fallback_repointed" "$path3/README.md"; then
    pass "a cached manifest clone whose origin no longer matches the bootstrap pointer is re-cloned"
else
    fail "manifest repoint: rc=$rc out='$out3' origin='$cached_manifest_url' ($(stderr_text))"
fi

# --- 6h3. a WORKSPACE_MANIFEST_GIT_BASE in no url form is refused ------------
# The base is the root of the trust chain and comes from the environment; an
# unvalidated one starting with `-` is read by git as an OPTION.
out=$(WORKSPACE_MANIFEST_GIT_BASE="--upload-pack=evil" \
      "$root/.agent/scripts/resolve_repo_checkout.sh" demo_repo 2>"$TMPDIR_ROOT/stderr"); rc=$?
if [ "$rc" -eq 5 ] && [ -z "$out" ] \
   && stderr_text | grep -q "WORKSPACE_MANIFEST_GIT_BASE"; then
    pass "a WORKSPACE_MANIFEST_GIT_BASE in no recognised url form is refused, never handed to git"
else
    fail "git base validation: rc=$rc out='$out' (expected 5 / empty), stderr='$(stderr_text)'"
fi

# --- 6i. a manifest clone that FAILED is exit 5, never "no manifest" (3) -----
# The pointer named a manifest repo and we could not get it. Reporting that as
# "no repo manifest configured — run make setup-all" would send the operator
# to a command that cannot fix a broken remote.
root=$(make_root manifest_fallback_fails)
echo "https://raw.githubusercontent.com/testowner/absentmanifest/main/config/bootstrap.yaml" \
    > "$root/configs/project_bootstrap.url"
out=$(WORKSPACE_MANIFEST_GIT_BASE="file://$TMPDIR_ROOT/manifest_origins" \
      "$root/.agent/scripts/resolve_repo_checkout.sh" demo_repo 2>"$TMPDIR_ROOT/stderr"); rc=$?
if [ "$rc" -eq 5 ] && [ -z "$out" ] && stderr_text | grep -q "could not clone the manifest repo"; then
    pass "an unreachable manifest repo → exit 5, distinct from 'no manifest configured'"
else
    fail "manifest fallback failure: rc=$rc out='$out' (expected 5 / empty), stderr='$(stderr_text)'"
fi

# --- 6j. a bootstrap pointer in an underivable form is exit 3, and says so ---
root=$(make_root manifest_fallback_unsupported)
echo "https://example.internal/some/other/place/bootstrap.yaml" > "$root/configs/project_bootstrap.url"
out=$(run_resolver "$root" demo_repo); rc=$?
if [ "$rc" -eq 3 ] && [ -z "$out" ] && stderr_text | grep -q "cannot derive a git url"; then
    pass "a bootstrap pointer this fallback cannot derive a git url from → exit 3, with the reason"
else
    fail "unsupported pointer: rc=$rc out='$out' (expected 3 / empty), stderr='$(stderr_text)'"
fi

# --- 6k. the per-repo lock actually serialises, and its wait is bounded ------
# The cache is shared by every worktree on a host, and the refresh path runs
# `rm -rf` over it. An untimed wait would let one wedged run block the host
# forever with nothing said; a run that ignored the lock would race.
if command -v flock >/dev/null 2>&1; then
    root=$(make_root flock_waits)
    origin=$(make_origin flock_waits)
    write_manifest "$root" "demo_repo" "file://$origin"
    lockfile="$root/.agent/scratchpad/janitor-repos/.demo_repo.lock"
    mkdir -p "$(dirname "$lockfile")"

    # Hold the lock briefly, then release: the resolver must WAIT, not fail.
    ( flock 9; sleep 2 ) 9>"$lockfile" &
    holder=$!
    sleep 0.3
    out=$(run_resolver "$root" demo_repo); rc=$?
    wait "$holder" 2>/dev/null
    if [ "$rc" -eq 0 ] && [ "${out##*$'\t'}" = "clone" ]; then
        pass "a held clone-cache lock is waited for, not raced past"
    else
        fail "flock wait: rc=$rc out='$out' ($(stderr_text))"
    fi

    # Hold it past the (overridden) wait: a named failure with empty stdout,
    # never an indefinite hang and never a resolve over a racing tree.
    ( flock 9; sleep 5 ) 9>"$lockfile" &
    holder=$!
    sleep 0.3
    out=$(RESOLVE_LOCK_TIMEOUT=1 "$root/.agent/scripts/resolve_repo_checkout.sh" demo_repo \
          2>"$TMPDIR_ROOT/stderr"); rc=$?
    kill "$holder" 2>/dev/null
    wait "$holder" 2>/dev/null
    if [ "$rc" -eq 5 ] && [ -z "$out" ] && stderr_text | grep -q "could not lock the clone cache"; then
        pass "a lock held past the wait → exit 5 with a reason, never an indefinite hang"
    else
        fail "flock timeout: rc=$rc out='$out' (expected 5 / empty), stderr='$(stderr_text)'"
    fi
else
    echo "⏭️  SKIP: flock cases (flock not available on this host)"
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

    # ...and the clone branch from the same worktree: with no layer checkout,
    # the clone must still land in the MAIN root's cache, not the worktree's.
    # A worktree has no `layers/` of its own, so this is the arrangement every
    # real sweep run hits.
    rm -rf "$root/layers"
    out=$("$wt/.agent/scripts/resolve_repo_checkout.sh" demo_repo 2>"$TMPDIR_ROOT/stderr"); rc=$?
    path=${out%%$'\t'*}
    mode=${out##*$'\t'}
    if [ "$rc" -eq 0 ] && [ "$mode" = "clone" ] \
       && [ "$path" = "$root/.agent/scratchpad/janitor-repos/demo_repo" ] \
       && [ -f "$path/README.md" ]; then
        pass "run from a worktree with no layer checkout, clones into the MAIN root's cache"
    else
        fail "worktree clone: rc=$rc out='$out' (expected mode clone under $root), stderr='$(stderr_text)'"
    fi
else
    fail "worktree: could not create the test worktree"
fi

echo ""
echo "Passed: $TEST_PASS  Failed: $TEST_FAIL"
[ "$TEST_FAIL" -eq 0 ]
