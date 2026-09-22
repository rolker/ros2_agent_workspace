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
#
# Hermetic includes the AMBIENT ENVIRONMENT, not just the filesystem. Every
# variable the scripts under test read is unset once here, and the cases that
# exercise one set it per invocation: an inherited `$BOOTSTRAP_URL` overrides
# the tracked pointer the manifest-fallback cases build their fixture around
# (11 of these cases fail with one exported), `$WORKSPACE_ROOT` and
# `$WORKSPACE_MANIFEST_GIT_BASE` redirect the root and the derived clone url
# out of the temp tree, and `$REDACT_PATH_PREFIXES` rewrites the diagnostics
# the failure cases match on. None of it is the operator's fault when it
# happens — the suite must not read the environment it was launched from.

set -uo pipefail

unset WORKSPACE_ROOT BOOTSTRAP_URL WORKSPACE_MANIFEST_GIT_BASE \
      RESOLVE_LOCK_TIMEOUT REDACT_PATH_PREFIXES

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REAL_SCRIPTS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"   # .../.agent/scripts
TEST_PASS=0
TEST_FAIL=0

TMPDIR_ROOT=$(mktemp -d /tmp/test_resolve_repo_checkout.XXXXXX)
cleanup() { chmod -R u+rwX "$TMPDIR_ROOT" 2>/dev/null; rm -rf "$TMPDIR_ROOT"; }
trap cleanup EXIT

GIT_ID=(-c user.name=Test -c user.email=test@example.invalid)
TAB=$'\t'

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
    # $1 = owner, $2 = repo, $3 = demo repo url to declare,
    # $4 = config_path to DECLARE in bootstrap.yaml (default: config — where
    #      the pointer says bootstrap.yaml itself sits), $5 = git_url to
    #      declare (default: this fixture's own url), $6 = branch to declare
    #      (default: main). A real manifest repo carries the bootstrap.yaml the
    #      pointer names, and the helper reads it as authoritative after the
    #      clone, exactly as setup_layers.sh does — so the fixture has one.
    local owner="$1" name="$2" demo_url="$3"
    local declared_config_path="${4:-config}"
    local work="$TMPDIR_ROOT/manifest_work/$owner/$name"
    local bare="$TMPDIR_ROOT/manifest_origins/$owner/$name.git"
    local declared_url="${5:-file://$TMPDIR_ROOT/manifest_origins/$owner/$name.git}"
    local declared_branch="${6:-main}"
    mkdir -p "$work/config" "$work/$declared_config_path/repos" "$(dirname "$bare")"
    cat > "$work/config/bootstrap.yaml" <<EOF
git_url: $declared_url
branch: $declared_branch
layer: core
config_path: $declared_config_path
EOF
    cat > "$work/$declared_config_path/repos/core.repos" <<EOF
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

# --- 6h4. manifest_fallback.sh refuses to be EXECUTED ------------------------
# Executed, it would define its functions and exit 0 having done nothing — a
# silent success from the one script whose job is to refuse those.
out=$(bash "$REAL_SCRIPTS_DIR/manifest_fallback.sh" 2>"$TMPDIR_ROOT/stderr"); rc=$?
if [ "$rc" -eq 2 ] && [ -z "$out" ] && stderr_text | grep -q "must be sourced"; then
    pass "manifest_fallback.sh executed → exit 2 with a reason, never a silent 0"
else
    fail "source guard: rc=$rc out='$out' (expected 2 / empty), stderr='$(stderr_text)'"
fi

# --- 6h4a. ...and without its redact.sh it returns 5, never 2 ---------------
# 2 already means "you executed this file instead of sourcing it". Reusing it
# for "the redact.sh I route diagnostics through is missing" made the two
# indistinguishable to a caller, and neither of the two callers checked the
# `source` status at all: the condition surfaced as `manifest_config_dir:
# command not found` and was then reported as exit 3, "no repo manifest
# configured — run make setup-all". 5 is what resolve_repo_checkout.sh already
# answers for the identical condition.
mkdir -p "$TMPDIR_ROOT/no_redact"
cp "$REAL_SCRIPTS_DIR/manifest_fallback.sh" "$TMPDIR_ROOT/no_redact/"
out=$(bash -c "source '$TMPDIR_ROOT/no_redact/manifest_fallback.sh'" 2>"$TMPDIR_ROOT/stderr"); rc=$?
if [ "$rc" -eq 5 ] && [ -z "$out" ] && stderr_text | grep -q "cannot load redact.sh"; then
    pass "manifest_fallback.sh sourced without redact.sh → 5 (the resolver's code), not 2"
else
    fail "redact guard: rc=$rc out='$out' (expected 5 / empty), stderr='$(stderr_text)'"
fi

# A redact.sh that EXISTS but will not load is the same answer: the rc of the
# `source` is checked, not inferred from the file being present.
mkdir -p "$TMPDIR_ROOT/bad_redact"
cp "$REAL_SCRIPTS_DIR/manifest_fallback.sh" "$TMPDIR_ROOT/bad_redact/"
printf 'redact_url() {\n' > "$TMPDIR_ROOT/bad_redact/redact.sh"
out=$(bash -c "source '$TMPDIR_ROOT/bad_redact/manifest_fallback.sh'" 2>"$TMPDIR_ROOT/stderr"); rc=$?
if [ "$rc" -eq 5 ] && [ -z "$out" ] && stderr_text | grep -q "cannot load redact.sh"; then
    pass "a redact.sh that will not load → 5 as well, never a half-defined shell"
else
    fail "redact load failure: rc=$rc out='$out' (expected 5 / empty), stderr='$(stderr_text)'"
fi

# --- 6h4c. resolve_repo_checkout.sh's OWN pre-redact.sh refusals -------------
# The no_redact/bad_redact fixtures just above copy-and-SOURCE
# manifest_fallback.sh, which only exercises THAT script's own guard
# (manifest_fallback.sh:107-108). resolve_repo_checkout.sh is EXECUTED, not
# sourced, and has two bare-echo refusals of its own (missing redact.sh;
# a redact.sh present but that fails to load) that neither this fixture nor
# any other in this file reaches — copy-and-execute a standalone copy of the
# resolver itself instead.
mkdir -p "$TMPDIR_ROOT/resolver_no_redact"
cp "$REAL_SCRIPTS_DIR/resolve_repo_checkout.sh" "$TMPDIR_ROOT/resolver_no_redact/"
out=$(bash "$TMPDIR_ROOT/resolver_no_redact/resolve_repo_checkout.sh" demo_repo 2>"$TMPDIR_ROOT/stderr"); rc=$?
if [ "$rc" -eq 5 ] && [ -z "$out" ] \
   && stderr_text | grep -q "cannot load redact.sh beside it" \
   && ! stderr_text | grep -qF "$TMPDIR_ROOT"; then
    pass "resolve_repo_checkout.sh without redact.sh beside it → 5, no absolute path on stderr"
else
    fail "resolver missing-redact: rc=$rc out='$out' (expected 5 / empty), stderr='$(stderr_text)'"
fi

# A redact.sh that fails to load must NOT itself leak the path via bash's own
# parse-time error (a syntactically broken file like an unterminated function
# would print "<absolute path>: line N: syntax error..." straight to stderr
# before this script's own guard ever runs) — so this fixture is syntactically
# valid bash that simply returns non-zero and defines nothing, the same
# "half-defined shell" failure mode the guard exists to catch.
mkdir -p "$TMPDIR_ROOT/resolver_bad_redact"
cp "$REAL_SCRIPTS_DIR/resolve_repo_checkout.sh" "$TMPDIR_ROOT/resolver_bad_redact/"
printf '#!/bin/bash\n# deliberately broken: fails to load, defines nothing\nreturn 1\n' \
    > "$TMPDIR_ROOT/resolver_bad_redact/redact.sh"
out=$(bash "$TMPDIR_ROOT/resolver_bad_redact/resolve_repo_checkout.sh" demo_repo 2>"$TMPDIR_ROOT/stderr"); rc=$?
if [ "$rc" -eq 5 ] && [ -z "$out" ] \
   && stderr_text | grep -q "cannot load redact.sh beside it" \
   && ! stderr_text | grep -qF "$TMPDIR_ROOT"; then
    pass "resolve_repo_checkout.sh with a redact.sh that will not load → 5, no absolute path on stderr"
else
    fail "resolver bad-redact: rc=$rc out='$out' (expected 5 / empty), stderr='$(stderr_text)'"
fi

# --- 6h4b. a cached manifest that cannot be REFRESHED is a failure ----------
# The cached clone is still readable, which is exactly why a refresh failure
# needs its own answer: returning 0 with a stderr warning let the caller
# enumerate from a manifest of unknown age and still report a completed run —
# the report-level false green. rc 6 out of the helper, exit 5 out of the
# resolver ("a clone or refresh failed"), with the reason on stderr.
root=$(make_root manifest_refresh_fails)
origin=$(make_origin manifest_refresh_fails)
make_manifest_origin refreshowner testmanifest "file://$origin"
echo "https://raw.githubusercontent.com/refreshowner/testmanifest/main/config/bootstrap.yaml" \
    > "$root/configs/project_bootstrap.url"
out=$(WORKSPACE_MANIFEST_GIT_BASE="file://$TMPDIR_ROOT/manifest_origins" \
      "$root/.agent/scripts/resolve_repo_checkout.sh" demo_repo 2>"$TMPDIR_ROOT/stderr"); rc=$?
if [ "$rc" -ne 0 ]; then
    fail "manifest refresh setup: first resolve should succeed, rc=$rc ($(stderr_text))"
else
    # The manifest clone is now cached. Take its origin away: the next run must
    # refuse the cached copy rather than quietly enumerate from it.
    rm -rf "$TMPDIR_ROOT/manifest_origins/refreshowner/testmanifest.git"
    out=$(WORKSPACE_MANIFEST_GIT_BASE="file://$TMPDIR_ROOT/manifest_origins" \
          "$root/.agent/scripts/resolve_repo_checkout.sh" demo_repo 2>"$TMPDIR_ROOT/stderr"); rc=$?
    if [ "$rc" -eq 5 ] && [ -z "$out" ] \
       && stderr_text | grep -q "could not refresh the cached manifest repo"; then
        pass "a cached manifest that could not be refreshed → failure, never a silent stale read"
    else
        fail "manifest refresh failure: rc=$rc out='$out' (expected 5 / empty), stderr='$(stderr_text)'"
    fi
fi

# --- 6h4c. the cloned bootstrap.yaml is authoritative about config_path ------
# The pointer path can only say WHERE bootstrap.yaml sits. setup_layers.sh
# reads git_url/branch/config_path out of the file itself (config_path
# defaulting to `config`), so a manifest repo that keeps its .repos somewhere
# other than beside its bootstrap.yaml used to be reported as "the bootstrap
# pointer and the manifest repo disagree" — loud, but blaming a disagreement
# that was really this derivation's limit.
root=$(make_root bootstrap_config_path)
origin=$(make_origin bootstrap_config_path)
make_manifest_origin cfgowner testmanifest "file://$origin" "manifests"
echo "https://raw.githubusercontent.com/cfgowner/testmanifest/main/config/bootstrap.yaml" \
    > "$root/configs/project_bootstrap.url"
out=$(WORKSPACE_MANIFEST_GIT_BASE="file://$TMPDIR_ROOT/manifest_origins" \
      "$root/.agent/scripts/resolve_repo_checkout.sh" demo_repo 2>"$TMPDIR_ROOT/stderr"); rc=$?
path=${out%%$'\t'*}
if [ "$rc" -eq 0 ] && [ -f "$path/README.md" ]; then
    pass "bootstrap.yaml's own config_path is honoured, not the pointer's path"
else
    fail "bootstrap config_path: rc=$rc out='$out' ($(stderr_text))"
fi

# --- 6h4d. ...and a git_url it does not agree with is named accurately -------
# A fork whose bootstrap names a different repo would otherwise enumerate from
# a manifest this workspace never uses, or be reported as a missing directory.
root=$(make_root bootstrap_url_disagrees)
origin=$(make_origin bootstrap_url_disagrees)
make_manifest_origin urlowner testmanifest "file://$origin" "config" \
    "https://github.com/someoneelse/othermanifest.git"
echo "https://raw.githubusercontent.com/urlowner/testmanifest/main/config/bootstrap.yaml" \
    > "$root/configs/project_bootstrap.url"
out=$(WORKSPACE_MANIFEST_GIT_BASE="file://$TMPDIR_ROOT/manifest_origins" \
      "$root/.agent/scripts/resolve_repo_checkout.sh" demo_repo 2>"$TMPDIR_ROOT/stderr"); rc=$?
if [ "$rc" -eq 5 ] && [ -z "$out" ] \
   && stderr_text | grep -q "is not the one its own bootstrap names"; then
    pass "a bootstrap.yaml naming a different manifest repo → exit 5, with the real reason"
else
    fail "bootstrap url disagreement: rc=$rc out='$out' (expected 5 / empty), stderr='$(stderr_text)'"
fi

# --- 6h4e. ...and so is a branch it does not agree with ----------------------
root=$(make_root bootstrap_branch_disagrees)
origin=$(make_origin bootstrap_branch_disagrees)
make_manifest_origin branchowner testmanifest "file://$origin" "config" "" "jazzy"
echo "https://raw.githubusercontent.com/branchowner/testmanifest/main/config/bootstrap.yaml" \
    > "$root/configs/project_bootstrap.url"
out=$(WORKSPACE_MANIFEST_GIT_BASE="file://$TMPDIR_ROOT/manifest_origins" \
      "$root/.agent/scripts/resolve_repo_checkout.sh" demo_repo 2>"$TMPDIR_ROOT/stderr"); rc=$?
if [ "$rc" -eq 5 ] && [ -z "$out" ] \
   && stderr_text | grep -q "the pointer is stale"; then
    pass "a bootstrap.yaml declaring a different branch → exit 5, naming both branches"
else
    fail "bootstrap branch disagreement: rc=$rc out='$out' (expected 5 / empty), stderr='$(stderr_text)'"
fi

# --- 6h4f. url FORM never decides the git_url cross-check --------------------
# The declared url and the derived one must be compared as REPO IDENTITIES,
# not as strings: `setup_layers.sh` accepts an scp-form url, a url with no
# `.git`, and a trailing `/`, and this derivation emits none of those — so a
# raw string compare hard-failed at exit 5 on a bootstrap that agrees
# perfectly, reporting it as "not the one its own bootstrap names".
root=$(make_root bootstrap_url_forms)
origin=$(make_origin bootstrap_url_forms)
make_manifest_origin formowner testmanifest "file://$origin" "config" \
    "git@github.com:formowner/testmanifest.git"
echo "https://raw.githubusercontent.com/formowner/testmanifest/main/config/bootstrap.yaml" \
    > "$root/configs/project_bootstrap.url"
out=$(WORKSPACE_MANIFEST_GIT_BASE="file://$TMPDIR_ROOT/manifest_origins" \
      "$root/.agent/scripts/resolve_repo_checkout.sh" demo_repo 2>"$TMPDIR_ROOT/stderr"); rc=$?
if [ "$rc" -eq 0 ] && [ -n "$out" ]; then
    pass "an scp-form git_url in bootstrap.yaml agrees with the derived url, not exit 5"
else
    fail "scp-form git_url: rc=$rc out='$out' (expected 0), stderr='$(stderr_text)'"
fi

# --- 6h4g. ...on BOTH branches, including the default (no GIT_BASE) ----------
# With WORKSPACE_MANIFEST_GIT_BASE unset — the branch every real host takes,
# and the one no end-to-end case can reach without the network, since the
# derived url is then github.com — the comparison key is exercised directly.
# The HOST is part of the key there (it is not redirected), and dropped when
# the variable IS set, which is that variable's whole purpose.
url_key_case() {
    # $1 = WORKSPACE_MANIFEST_GIT_BASE value ("" = unset), $2, $3 = the urls,
    # $4 = "agree"|"differ", $5 = description
    local got
    got=$(
        if [ -n "$1" ]; then export WORKSPACE_MANIFEST_GIT_BASE="$1"; else unset WORKSPACE_MANIFEST_GIT_BASE; fi
        # shellcheck source=/dev/null
        source "$REAL_SCRIPTS_DIR/manifest_fallback.sh"
        a=$(_manifest_fallback_url_key "$2")
        b=$(_manifest_fallback_url_key "$3")
        [ "$a" = "$b" ] && echo agree || echo "differ ($a vs $b)"
    )
    case "$got" in
        "$4"*) pass "url key: $5" ;;
        *)     fail "url key: $5 — expected $4, got '$got'" ;;
    esac
}

url_key_case "" "https://github.com/o/r.git" "git@github.com:o/r.git" agree \
    "scp-form and https name the same repo (GIT_BASE unset)"
url_key_case "" "https://github.com/o/r.git" "https://github.com/o/r" agree \
    "a missing .git does not change the identity (GIT_BASE unset)"
url_key_case "" "https://github.com/o/r.git" "https://github.com/o/r/" agree \
    "a trailing slash does not change the identity (GIT_BASE unset)"
url_key_case "" "https://github.com/o/r.git" "https://GitHub.com/o/r.git" agree \
    "the host is compared case-insensitively (GIT_BASE unset)"
url_key_case "" "https://github.com/o/r.git" "https://gitlab.com/o/r.git" differ \
    "a DIFFERENT host still disagrees when the base was not redirected"
url_key_case "" "https://github.com/o/r.git" "https://github.com/other/r.git" differ \
    "a different owner still disagrees (GIT_BASE unset)"
url_key_case "file:///tmp/mirror" "https://github.com/o/r.git" "file:///tmp/mirror/o/r.git" agree \
    "with GIT_BASE set the host is exempt — only <owner>/<repo> must agree"
url_key_case "file:///tmp/mirror" "https://github.com/o/r.git" "file:///tmp/mirror/o/other.git" differ \
    "with GIT_BASE set a different repo still disagrees"

# --- 6h4h. manifest_bootstrap_identity: the pointer is parsed in ONE place ---
# The identity derivation (which owner/repo/branch/config_path the tracked
# `configs/project_bootstrap.url` names) was extracted out of
# manifest_config_dir so a caller can ask WHICH project a checkout is
# configured for without paying for — or being failed by — a manifest clone it
# does not need (#652). Extracted code with no direct test is how a
# "behaviour-preserving" refactor quietly stops being one: the
# manifest_config_dir cases above cover the happy path through the clone, and
# these cover the parse itself, including every refusal that must NOT print a
# half-parsed line to stdout.
IDENTITY_CASE_N=0
bootstrap_identity_case() {
    # $1 = pointer file contents ("<none>" writes no pointer file at all),
    # $2 = BOOTSTRAP_URL value ("" = unset), $3 = expected rc,
    # $4 = expected stdout (empty on every refusal), $5 = description
    local pointer="$1" bootstrap_env="$2" want_rc="$3" want_out="$4" desc="$5"
    local root out rc
    IDENTITY_CASE_N=$((IDENTITY_CASE_N + 1))
    root=$(make_root "bootstrap_identity_$IDENTITY_CASE_N")
    if [ "$pointer" != "<none>" ]; then
        printf '%s\n' "$pointer" > "$root/configs/project_bootstrap.url"
    fi
    # The redirection lives INSIDE the substitution: on `out=$(...) 2>file`
    # bash expands the substitution before applying the assignment's own
    # redirection, so the reason would reach the terminal and the file would
    # be left empty — and this helper's "a refusal states a reason" assertion
    # would then fail on every correctly-behaving refusal.
    out=$(
        {
            if [ -n "$bootstrap_env" ]; then export BOOTSTRAP_URL="$bootstrap_env"; else unset BOOTSTRAP_URL; fi
            # shellcheck source=/dev/null
            source "$REAL_SCRIPTS_DIR/manifest_fallback.sh"
            manifest_bootstrap_identity "$root"
        } 2>"$TMPDIR_ROOT/stderr"
    )
    rc=$?
    if [ "$rc" -ne "$want_rc" ] || [ "$out" != "$want_out" ]; then
        fail "bootstrap identity: $desc — rc=$rc (want $want_rc), out='$out' (want '$want_out'), stderr='$(stderr_text)'"
    elif [ "$want_rc" -ne 0 ] && [ ! -s "$TMPDIR_ROOT/stderr" ]; then
        # A refusal with no reason on stderr is the same dead end as an empty
        # success: the caller is told nothing it can act on.
        fail "bootstrap identity: $desc — refused at rc $rc with no reason on stderr"
    else
        pass "bootstrap identity: $desc"
    fi
}

bootstrap_identity_case \
    "https://raw.githubusercontent.com/someowner/somerepo/main/config/bootstrap.yaml" "" \
    0 "someowner${TAB}somerepo${TAB}main${TAB}config" \
    "a valid pointer prints the four-field TSV"
bootstrap_identity_case \
    "https://raw.githubusercontent.com/someowner/somerepo/jazzy/some/deep/config/bootstrap.yaml" "" \
    0 "someowner${TAB}somerepo${TAB}jazzy${TAB}some/deep/config" \
    "a multi-segment config path stays whole, and the branch is the segment before it"
bootstrap_identity_case \
    "https://raw.githubusercontent.com/someowner/somerepo.git/main/config/bootstrap.yaml" "" \
    0 "someowner${TAB}somerepo${TAB}main${TAB}config" \
    "a trailing .git on the repo segment is stripped"
bootstrap_identity_case \
    "https://raw.githubusercontent.com/someowner/somerepo/main/config/" "" \
    3 "" \
    "a pointer with a trailing / and no bootstrap.yaml tail is refused, stdout empty"
bootstrap_identity_case \
    "git@github.com:someowner/somerepo.git" "" \
    3 "" \
    "an ssh scp-form git url is refused, not guessed at — stdout empty"
bootstrap_identity_case \
    "https://github.com/someowner/somerepo" "" \
    3 "" \
    "a repo web url is refused as well — only the raw-content form is derivable"
bootstrap_identity_case \
    "<none>" "" \
    3 "" \
    "no pointer file at all → 3 with an empty stdout, never a blank identity"
bootstrap_identity_case \
    "" "" \
    3 "" \
    "an empty (whitespace-only) pointer file → 3, not a parse of the empty string"
bootstrap_identity_case \
    "https://raw.githubusercontent.com/someowner/somerepo/main/../../etc/bootstrap.yaml" "" \
    3 "" \
    "a config path containing .. is refused rather than interpolated into a path"
bootstrap_identity_case \
    "https://raw.githubusercontent.com/pointerowner/pointerrepo/main/config/bootstrap.yaml" \
    "https://raw.githubusercontent.com/envowner/envrepo/rolling/cfg/bootstrap.yaml" \
    0 "envowner${TAB}envrepo${TAB}rolling${TAB}cfg" \
    "\$BOOTSTRAP_URL overrides the tracked pointer, as in setup_layers.sh"
bootstrap_identity_case \
    "https://raw.githubusercontent.com/pointerowner/pointerrepo/main/config/bootstrap.yaml" \
    "not-a-url" \
    3 "" \
    "an unparseable \$BOOTSTRAP_URL is refused even when the pointer file is valid — the override is not silently ignored"

# --- 6h4i. manifest_config_dir's exit 3 still names BOTH absent things ---
# The extraction moved the pointer diagnostic into manifest_bootstrap_identity,
# which can only speak about the pointer. manifest_config_dir reaches it only
# after finding no configs/manifest either, and the operator it sends to
# `make setup-all` needs both halves: a message about the pointer alone reads
# as a pointer problem, for which `make setup-all` looks like the wrong remedy.
root=$(make_root config_dir_exit3_names_both)
rc=0
out=$(
    {
        unset BOOTSTRAP_URL
        # shellcheck source=/dev/null
        source "$REAL_SCRIPTS_DIR/manifest_fallback.sh"
        manifest_config_dir "$root"
    } 2>"$TMPDIR_ROOT/stderr"
) || rc=$?
err=$(stderr_text)
if [ "$rc" -ne 3 ] || [ -n "$out" ]; then
    fail "manifest_config_dir with no manifest and no pointer → rc=$rc (want 3), out='$out' (want empty)"
elif [[ "$err" != *"configs/project_bootstrap.url"* ]] || [[ "$err" != *"configs/manifest"* ]]; then
    fail "manifest_config_dir exit 3 names only one of the two absent things: '$err'"
else
    pass "manifest_config_dir exit 3 names both the missing manifest and the missing pointer"
fi

# A pointer that is PRESENT but unusable takes the same exit 3, and the added
# half has to read correctly there too: "no configs/manifest ... either" claims
# a second absence when the first thing was a malformed presence. Only the
# missing-pointer branch was covered before, so the wording could regress
# unseen on the branch operators actually hit after editing the pointer.
root=$(make_root config_dir_exit3_malformed_pointer)
echo "https://github.com/someowner/somerepo" > "$root/configs/project_bootstrap.url"
rc=0
out=$(
    {
        unset BOOTSTRAP_URL
        # shellcheck source=/dev/null
        source "$REAL_SCRIPTS_DIR/manifest_fallback.sh"
        manifest_config_dir "$root"
    } 2>"$TMPDIR_ROOT/stderr"
) || rc=$?
err=$(stderr_text)
if [ "$rc" -ne 3 ] || [ -n "$out" ]; then
    fail "manifest_config_dir with a malformed pointer → rc=$rc (want 3), out='$out' (want empty)"
elif [[ "$err" != *"raw.githubusercontent.com"* ]] || [[ "$err" != *"configs/manifest"* ]]; then
    fail "malformed-pointer exit 3 names only one of the two halves: '$err'"
elif [[ "$err" == *"either"* ]]; then
    fail "malformed-pointer exit 3 says 'either', which claims the pointer was absent: '$err'"
else
    pass "manifest_config_dir exit 3 reads correctly for a PRESENT but unusable pointer"
fi

# --- 6h5. a configs/*.repos manifest on disk is never bypassed for a clone ---
# The early return has to recognise EVERY layout get_overlay_repos reads
# (configs/manifest/repos AND configs/*.repos). Recognising only the first made
# the helper attempt a network clone on a workspace whose manifest was right
# there, and the resolver turned that clone's failure into exit 5 before the
# on-disk manifest was ever opened — a false RED over a workable state. The
# pointer here derives an absent manifest repo on purpose: if the early return
# regresses, the clone fails and this case goes to 5.
root=$(make_root manifest_on_disk_wins)
origin=$(make_origin manifest_on_disk_wins)
write_manifest "$root" "demo_repo" "file://$origin"
echo "https://raw.githubusercontent.com/testowner/absentmanifest/main/config/bootstrap.yaml" \
    > "$root/configs/project_bootstrap.url"
out=$(WORKSPACE_MANIFEST_GIT_BASE="file://$TMPDIR_ROOT/manifest_origins" \
      "$root/.agent/scripts/resolve_repo_checkout.sh" demo_repo 2>"$TMPDIR_ROOT/stderr"); rc=$?
if [ "$rc" -eq 0 ] && [ -n "$out" ] && ! stderr_text | grep -q "manifest_fallback"; then
    pass "a configs/*.repos manifest on disk short-circuits the fallback, never a clone"
else
    fail "on-disk configs/*.repos: rc=$rc out='$out' ($(stderr_text))"
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

# --- 6k2. the lock file itself cannot be OPENED (e.g. an unwritable cache
#          dir) — must be reported through say(), never as bash's own raw
#          "Permission denied" naming the absolute lock path -----------------
if command -v flock >/dev/null 2>&1 && [ "$(id -u)" -ne 0 ]; then
    root=$(make_root lock_open_fails)
    origin=$(make_origin lock_open_fails)
    write_manifest "$root" "demo_repo" "file://$origin"
    cache_dir="$root/.agent/scratchpad/janitor-repos"
    mkdir -p "$cache_dir"
    chmod 555 "$cache_dir"
    out=$(run_resolver "$root" demo_repo); rc=$?
    chmod 755 "$cache_dir"
    if [ "$rc" -eq 5 ] && [ -z "$out" ] \
       && stderr_text | grep -q "could not open the lock file" \
       && ! stderr_text | grep -qi "permission denied"; then
        pass "an unopenable lock file → exit 5 via say(), never bash's own raw permission-denied line"
    else
        fail "lock open failure: rc=$rc out='$out' (expected 5 / empty), stderr='$(stderr_text)'"
    fi
elif [ "$(id -u)" -eq 0 ]; then
    echo "⏭️  SKIP: lock-open-failure case (running as root; chmod cannot deny)"
else
    echo "⏭️  SKIP: lock-open-failure case (flock not available on this host)"
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

# --- 7b. ...and the same url with conflicting VERSION pins is just as ambiguous
# The url is not the identity of a checkout. Two manifests naming this repo at
# one url and two different `version:` pins describe two different trees, and
# keying the ambiguity check on the url alone took matches[0] in sorted-manifest
# order — cloning whichever sorted first and auditing the wrong branch, silently,
# where the conflicting-url case is loud. The message must name both pins and
# the files they came from, so the operator can resolve it without hunting.
root=$(make_root ambiguous_version)
origin=$(make_origin ambiguous_version release)
write_manifest "$root" "demo_repo" "file://$origin" "main"
cat > "$root/configs/other.repos" <<EOF
repositories:
  demo_repo:
    type: git
    url: file://$origin
    version: release
EOF
out=$(run_resolver "$root" demo_repo); rc=$?
if [ "$rc" -eq 7 ] && [ -z "$out" ] \
   && stderr_text | grep -q "conflicting urls or version pins" \
   && stderr_text | grep -q "main" && stderr_text | grep -q "release" \
   && stderr_text | grep -q "test.repos" && stderr_text | grep -q "other.repos"; then
    pass "same url with conflicting version pins → exit 7, naming both pins and their manifests"
else
    fail "ambiguous version: rc=$rc out='$out' (expected 7 / empty), stderr='$(stderr_text)'"
fi

# ...and the same pin declared twice is NOT ambiguous: agreeing manifests are
# not a conflict, and turning them into one would fail every workspace that
# declares a repo in two layers at the same version.
root=$(make_root agreeing_manifests)
origin=$(make_origin agreeing_manifests)
write_manifest "$root" "demo_repo" "file://$origin" "main"
cat > "$root/configs/other.repos" <<EOF
repositories:
  demo_repo:
    type: git
    url: file://$origin
    version: main
EOF
out=$(run_resolver "$root" demo_repo); rc=$?
if [ "$rc" -eq 0 ] && [ -n "$out" ]; then
    pass "the same (url, version) in two manifests is not a conflict"
else
    fail "agreeing manifests: rc=$rc out='$out' (expected 0 / a path), stderr='$(stderr_text)'"
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

# --- 10. credentials never reach stderr (and so never reach the report) ------
# Both scripts funnel their failure reasons into a sweep report that is written
# to disk and handed around. A manifest url is not supposed to carry userinfo
# and `WORKSPACE_MANIFEST_GIT_BASE` comes from the ambient environment, so
# "supposed to" is not a guarantee. The redaction is shared (redact.sh) so the
# two cannot drift apart.
( set -uo pipefail
  # shellcheck source=../redact.sh
  source "$REAL_SCRIPTS_DIR/redact.sh"
  [ "$(redact_url 'https://bob:hunter2@github.com/o/r.git')" = 'https://<redacted>@github.com/o/r.git' ] || exit 1
  [ "$(redact_url 'ssh://bob:hunter2@example.com/o/r.git')" = 'ssh://<redacted>@example.com/o/r.git' ] || exit 1
  # scp-form has no password field; leave it readable.
  [ "$(redact_url 'git@github.com:o/r.git')" = 'git@github.com:o/r.git' ] || exit 1
  [ "$(redact_url 'https://github.com/o/r.git')" = 'https://github.com/o/r.git' ] || exit 1
  # Captured git output: the url is mid-sentence, on any line, and there may be
  # more than one. redact_url's whole-string anchor does not match there.
  got=$(redact_text "fatal: could not read from https://bob:hunter2@github.com/o/r.git
remote: denied for ssh://bob:hunter2@example.com/o/r.git")
  case "$got" in *hunter2*) exit 1 ;; esac
  case "$got" in *'<redacted>@github.com'*) ;; *) exit 1 ;; esac
  case "$got" in *'<redacted>@example.com'*) ;; *) exit 1 ;; esac
  # Local paths carry host identity into the same report.
  # shellcheck disable=SC2034  # read by redact_text, in the file sourced above
  REDACT_PATH_PREFIXES=("/home/someone/ws=<workspace>" "/home/someone=~")
  [ "$(redact_text 'failed at /home/someone/ws/.agent/scratchpad/x')" = 'failed at <workspace>/.agent/scratchpad/x' ] || exit 1
  [ "$(redact_text 'failed at /home/someone/elsewhere')" = 'failed at ~/elsewhere' ] || exit 1
  # No prefixes configured, and no url: the string is returned unchanged.
  unset REDACT_PATH_PREFIXES
  [ "$(redact_text 'plain reason, nothing to strip')" = 'plain reason, nothing to strip' ] || exit 1
) && pass "redact_url/redact_text strip userinfo and local path prefixes" \
  || fail "redaction helpers: see .agent/scripts/redact.sh"

# redact.sh executed rather than sourced would define two functions and exit 0
# having done nothing — the silent success both its callers exist to refuse.
out=$(bash "$REAL_SCRIPTS_DIR/redact.sh" 2>"$TMPDIR_ROOT/stderr"); rc=$?
if [ "$rc" -eq 2 ] && [ -z "$out" ] && stderr_text | grep -q "must be sourced"; then
    pass "redact.sh executed → exit 2 with a reason, never a silent 0"
else
    fail "redact.sh source guard: rc=$rc out='$out' (expected 2 / empty), stderr='$(stderr_text)'"
fi

# ...and the resolver actually uses it: a manifest url in no recognised form is
# reported with its userinfo stripped, without ever reaching the network.
root=$(make_root redaction_resolver)
write_manifest "$root" "demo_repo" "ftp://bob:hunter2@example.com/o/r.git"
out=$(run_resolver "$root" demo_repo); rc=$?
if [ "$rc" -eq 6 ] && [ -z "$out" ] && ! stderr_text | grep -q "hunter2" \
   && stderr_text | grep -q "<redacted>@example.com"; then
    pass "an unrecognised manifest url is reported with its credentials redacted"
else
    fail "resolver redaction: rc=$rc out='$out' (expected 6 / empty), stderr='$(stderr_text)'"
fi

# ...and so does the manifest fallback, on the one url it takes from the
# ambient environment. A base carrying `..` is refused before any clone, so
# this stays hermetic while still exercising the message.
root=$(make_root redaction_manifest_fallback)
echo "https://raw.githubusercontent.com/testowner/testmanifest/main/config/bootstrap.yaml" \
    > "$root/configs/project_bootstrap.url"
out=$(WORKSPACE_MANIFEST_GIT_BASE="https://bob:hunter2@example.com/../mirror" \
      "$root/.agent/scripts/resolve_repo_checkout.sh" demo_repo 2>"$TMPDIR_ROOT/stderr"); rc=$?
if [ "$rc" -eq 5 ] && [ -z "$out" ] && ! stderr_text | grep -q "hunter2" \
   && stderr_text | grep -q "WORKSPACE_MANIFEST_GIT_BASE"; then
    pass "a WORKSPACE_MANIFEST_GIT_BASE carrying credentials is reported redacted"
else
    fail "manifest fallback redaction: rc=$rc out='$out' (expected 5 / empty), stderr='$(stderr_text)'"
fi

# --- 10d. git's OWN captured output is scrubbed, and so are local paths ------
# The resolver redacted $REPO_URL and then interpolated git's captured $out
# verbatim — and git echoes the remote url, userinfo included, in many clone
# and fetch errors, so the message leaked through its own second half. The
# same strings carry absolute paths that name the host. Both now go through
# redact_text: git's output here names a repository path under the workspace
# root, which must come back as <workspace>/...
root=$(make_root redaction_captured_output)
write_manifest "$root" "demo_repo" "file://$root/origins/absent.git"
out=$(run_resolver "$root" demo_repo); rc=$?
if [ "$rc" -eq 5 ] && [ -z "$out" ] \
   && stderr_text | grep -q "<workspace>/origins/absent.git" \
   && ! stderr_text | grep -q "$root/origins/absent.git"; then
    pass "git's captured output is scrubbed of local paths before it reaches the report"
else
    fail "captured-output redaction: rc=$rc out='$out', stderr='$(stderr_text)'"
fi

# The same for a message this script composes itself, on the layer branch.
if [ "$(id -u)" -eq 0 ]; then
    echo "⏭️  SKIP: composed-path redaction case (running as root; chmod cannot deny)"
else
root=$(make_root redaction_layer_path)
origin=$(make_origin redaction_layer_path)
write_manifest "$root" "demo_repo" "file://$origin"
mkdir -p "$root/layers/main/demo_ws/src/demo_repo"
touch "$root/layers/main/demo_ws/src/demo_repo/package.xml"
chmod 000 "$root/layers/main/demo_ws/src/demo_repo"
out=$(run_resolver "$root" demo_repo); rc=$?
chmod 755 "$root/layers/main/demo_ws/src/demo_repo"
if [ "$rc" -eq 5 ] && [ -z "$out" ] \
   && stderr_text | grep -q "<workspace>/layers/main/demo_ws/src/demo_repo"; then
    pass "a local path in a composed message is reported as <workspace>/..., not the host's"
else
    fail "layer path redaction: rc=$rc out='$out', stderr='$(stderr_text)'"
fi
fi

echo ""
echo "Passed: $TEST_PASS  Failed: $TEST_FAIL"
[ "$TEST_FAIL" -eq 0 ]
