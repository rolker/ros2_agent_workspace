#!/bin/bash
# test_ci_local.sh — hermetic tests for ci_local.sh (workspace#573).
# Stubs `docker` on PATH (no containers, no network) and exercises argument
# handling, package discovery (COLCON_IGNORE, name validation), dry-run
# planning, pass/fail exit codes, snapshot-vs-live mount selection, and
# attestation-note semantics (clean vs dirty tree, append-not-overwrite,
# partial scope, --no-attest, failure). The upstream.repos path (#577) is
# exercised against a local fixture upstream git repo (file-path URL, so the
# host-side ls-remote resolution runs for real with no network), covering
# dry-run planning, hook/skip-keys detection, injection rejection, the
# upstream-repo: note lines, and the no-upstream byte-compat regression.
set -uo pipefail

TESTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SUT="$TESTS_DIR/../ci_local.sh"

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

PASS=0
FAIL=0
check() {
  local desc="$1"; shift
  if "$@"; then echo "  ok: $desc"; ((++PASS))
  else echo "  FAIL: $desc"; ((++FAIL)); fi
}
contains() { grep -qF -- "$2" <<<"$1"; }
not_contains() { ! grep -qF -- "$2" <<<"$1"; }

# ---- docker stub ------------------------------------------------------------
mkdir -p "$TMP/bin"
export DOCKER_STUB_CALLS="$TMP/docker_calls"
cat > "$TMP/bin/docker" <<'EOF'
#!/bin/bash
case "${1:-}" in
  image)
    echo "sha256:stub-image-id"
    exit 0 ;;
  run)
    echo "docker-run: $*" >> "$DOCKER_STUB_CALLS"
    echo "stub container output"
    exit "${DOCKER_STUB_RUN_RC:-0}" ;;
  *) exit 0 ;;
esac
EOF
chmod +x "$TMP/bin/docker"
export PATH="$TMP/bin:$PATH"
export CI_LOCAL_LOG_DIR="$TMP/logs"

# Run the whole harness with NO ambient git identity, exactly like a CI
# runner or fresh host — the attestation path must not depend on it
# (regression: the #574 Script-tests CI failure). Fixture commits below
# pass identity per-invocation with -c.
export GIT_CONFIG_GLOBAL=/dev/null
export GIT_CONFIG_SYSTEM=/dev/null

# ---- fixture repo -----------------------------------------------------------
REPO="$TMP/demo_repo"
mkdir -p "$REPO/demo_pkg" "$REPO/fixtures/ignored_pkg"
cat > "$REPO/demo_pkg/package.xml" <<'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>demo_pkg</name>
  <version>0.0.1</version>
  <description>fixture</description>
  <maintainer email="t@t">t</maintainer>
  <license>BSD</license>
</package>
EOF
# A package colcon would not discover: under a COLCON_IGNORE subtree.
sed 's/demo_pkg/hidden_fixture_pkg/' "$REPO/demo_pkg/package.xml" > "$REPO/fixtures/ignored_pkg/package.xml"
touch "$REPO/fixtures/COLCON_IGNORE"
git -C "$REPO" init -q
git -C "$REPO" -c user.name=t -c user.email=t@t add -A
git -C "$REPO" -c user.name=t -c user.email=t@t commit -qm fixture
HEAD_SHA="$(git -C "$REPO" rev-parse HEAD)"

note_of() { git -C "$REPO" notes --ref=ci-local show "$1" 2>/dev/null; }

echo "== dry run =="
out=$(bash "$SUT" "$REPO" --dry-run 2>&1); rc=$?
check "exits 0"                    [ "$rc" -eq 0 ]
check "discovers package"          contains "$out" "packages : demo_pkg"
check "skips COLCON_IGNORE subtree" not_contains "$out" hidden_fixture_pkg
check "plans attestation"          contains "$out" "refs/notes/ci-local on $HEAD_SHA"
check "plans snapshot source"      contains "$out" "pristine snapshot of $HEAD_SHA"
check "no container run"           bash -c "! grep -qs docker-run '$DOCKER_STUB_CALLS'"

echo "== --packages narrowing is partial; --no-attest plan =="
out=$(bash "$SUT" "$REPO" --dry-run --packages "demo_pkg extra_pkg" 2>&1); rc=$?
check "narrowed set flagged partial" contains "$out" "(PARTIAL of: demo_pkg)"
out=$(bash "$SUT" "$REPO" --dry-run --no-attest 2>&1); rc=$?
check "no-attest in plan"          contains "$out" "attest   : no"
check "no-attest uses live tree"   contains "$out" "live working tree"

echo "== successful run attests from snapshot =="
out=$(bash "$SUT" "$REPO" 2>&1); rc=$?
check "exits 0"                    [ "$rc" -eq 0 ]
check "container ran"              grep -qs docker-run "$DOCKER_STUB_CALLS"
check "snapshot mounted, not live" bash -c "grep -qs 'ci_local_snap.*:/ci/src/demo_repo:ro' '$DOCKER_STUB_CALLS' && ! grep -qs -- '$REPO:/ci/src/demo_repo:ro' '$DOCKER_STUB_CALLS'"
check "entrypoint bypassed"        grep -qs -- "--entrypoint bash" "$DOCKER_STUB_CALLS"
note=$(note_of "$HEAD_SHA")
check "note says pass"             contains "$note" "ci-local: pass"
check "note scope full"            contains "$note" "scope: full"
check "note has commit"            contains "$note" "commit: $HEAD_SHA"
check "note has log hash"          contains "$note" "log-sha256: "
log_file=$(ls "$CI_LOCAL_LOG_DIR"/demo_repo-*.log 2>/dev/null | head -1)
check "log file written"           [ -n "$log_file" ]
check "log hash matches"           bash -c "[ \"\$(sha256sum '$log_file' | cut -d' ' -f1)\" = \"\$(sed -n 's/^log-sha256: //p' <<<'$note' | head -1)\" ]"

echo "== re-run appends a second record (no overwrite) =="
out=$(bash "$SUT" "$REPO" 2>&1); rc=$?
check "exits 0"                    [ "$rc" -eq 0 ]
check "two pass records"           bash -c "[ \"\$(git -C '$REPO' notes --ref=ci-local show '$HEAD_SHA' | grep -cF 'ci-local: pass')\" -eq 2 ]"
check "still one note object"      bash -c "[ \"\$(git -C '$REPO' notes --ref=ci-local list | wc -l)\" -eq 1 ]"

echo "== partial run recorded as partial =="
git -C "$REPO" -c user.name=t -c user.email=t@t notes --ref=ci-local remove "$HEAD_SHA" >/dev/null 2>&1
out=$(bash "$SUT" "$REPO" --packages "demo_pkg other_pkg" 2>&1); rc=$?
check "exits 0"                    [ "$rc" -eq 0 ]
note=$(note_of "$HEAD_SHA")
check "note marked partial"        contains "$note" "ci-local: pass (partial)"
check "note scope partial"         contains "$note" "scope: partial"
git -C "$REPO" -c user.name=t -c user.email=t@t notes --ref=ci-local remove "$HEAD_SHA" >/dev/null 2>&1

echo "== dirty tree passes but does not attest, uses live tree =="
: > "$DOCKER_STUB_CALLS"
echo x > "$REPO/dirty.txt"
out=$(bash "$SUT" "$REPO" 2>&1); rc=$?
check "exits 0"                    [ "$rc" -eq 0 ]
check "reports unattested"         contains "$out" "PASS (unattested)"
check "live tree mounted"          grep -qs -- "$REPO:/ci/src/demo_repo:ro" "$DOCKER_STUB_CALLS"
check "no note written"            bash -c "! git -C '$REPO' notes --ref=ci-local show '$HEAD_SHA' >/dev/null 2>&1"
rm "$REPO/dirty.txt"

echo "== failing container run =="
out=$(DOCKER_STUB_RUN_RC=3 bash "$SUT" "$REPO" 2>&1); rc=$?
check "exits non-zero"             [ "$rc" -ne 0 ]
check "reports failure"            contains "$out" "CI FAILED"
check "no note written"            bash -c "! git -C '$REPO' notes --ref=ci-local show '$HEAD_SHA' >/dev/null 2>&1"

echo "== --no-attest run =="
out=$(bash "$SUT" "$REPO" --no-attest 2>&1); rc=$?
check "exits 0"                    [ "$rc" -eq 0 ]
check "reports skip"               contains "$out" "attestation skipped"
check "no note written"            bash -c "! git -C '$REPO' notes --ref=ci-local show '$HEAD_SHA' >/dev/null 2>&1"

echo "== malicious package name rejected =="
BADREPO="$TMP/bad_repo"
mkdir -p "$BADREPO/p"
printf '<package><name>evil; touch /tmp/pwned</name></package>\n' > "$BADREPO/p/package.xml"
git -C "$BADREPO" init -q
git -C "$BADREPO" -c user.name=t -c user.email=t@t add -A
git -C "$BADREPO" -c user.name=t -c user.email=t@t commit -qm bad
out=$(bash "$SUT" "$BADREPO" --dry-run 2>&1); rc=$?
check "injection name rejected"    [ "$rc" -ne 0 ]
check "names the bad package"      contains "$out" "invalid package name"
out=$(bash "$SUT" "$REPO" --dry-run --packages 'demo_pkg $(reboot)' 2>&1); rc=$?
check "injection via --packages rejected" [ "$rc" -ne 0 ]

echo "== argument errors =="
out=$(bash "$SUT" 2>&1); rc=$?
check "missing repo rejected"      [ "$rc" -ne 0 ]
out=$(bash "$SUT" "$TMP/nonexistent" 2>&1); rc=$?
check "bad path rejected"          [ "$rc" -ne 0 ]
out=$(bash "$SUT" "$REPO/demo_pkg" 2>&1); rc=$?
check "subdirectory rejected"      [ "$rc" -ne 0 ]
check "explains repo root"         contains "$out" "pass the repo root"
out=$(bash "$SUT" "$REPO" --frobnicate 2>&1); rc=$?
check "unknown option rejected"    [ "$rc" -ne 0 ]
out=$(bash "$SUT" "$REPO" --dry-run --image "evil
ci-local: pass" 2>&1); rc=$?
check "whitespace image rejected"  [ "$rc" -ne 0 ]
mkdir -p "$TMP/bad dir/space_repo"
git -C "$TMP/bad dir/space_repo" init -q
out=$(bash "$SUT" "$TMP/bad dir/space_repo" --dry-run 2>&1); rc=$?
check "whitespace repo path rejected" [ "$rc" -ne 0 ]
check "explains mount hazard"      contains "$out" "unsupported for container mounts"
mkdir -p "$TMP/no_pkgs" && git -C "$TMP/no_pkgs" init -q
out=$(bash "$SUT" "$TMP/no_pkgs" --dry-run 2>&1); rc=$?
check "package-less repo rejected" [ "$rc" -ne 0 ]

# ---- upstream.repos fixtures (#577) -----------------------------------------
# A local git repo serves as the upstream source: its path is the url, so the
# host-side ls-remote branch->SHA resolution runs for real without network.
UPDEP_SRC="$TMP/updep_src"
mkdir -p "$UPDEP_SRC"
echo upstream > "$UPDEP_SRC/marker.txt"
git -C "$UPDEP_SRC" init -q -b jazzy
git -C "$UPDEP_SRC" -c user.name=t -c user.email=t@t add -A
git -C "$UPDEP_SRC" -c user.name=t -c user.email=t@t commit -qm upstream
UPDEP_SHA="$(git -C "$UPDEP_SRC" rev-parse HEAD)"

UPREPO="$TMP/up_repo"
mkdir -p "$UPREPO/up_pkg" "$UPREPO/.agents"
sed 's/demo_pkg/up_pkg/' "$REPO/demo_pkg/package.xml" > "$UPREPO/up_pkg/package.xml"
cat > "$UPREPO/upstream.repos" <<EOF
repositories:
  updep:
    type: git
    url: $UPDEP_SRC
    version: jazzy
EOF
cat > "$UPREPO/.agents/ci_local_upstream_extra.sh" <<'EOF'
#!/bin/bash
touch some_subpkg/COLCON_IGNORE 2>/dev/null || true
EOF
chmod +x "$UPREPO/.agents/ci_local_upstream_extra.sh"
cat > "$UPREPO/.agents/ci_local_rosdep_skip_keys.txt" <<'EOF'
# keys the combined rosdep install must skip
skip_key_a
skip_key_b  # trailing comment
EOF
git -C "$UPREPO" init -q
git -C "$UPREPO" -c user.name=t -c user.email=t@t add -A
git -C "$UPREPO" -c user.name=t -c user.email=t@t commit -qm upfixture
UPREPO_SHA="$(git -C "$UPREPO" rev-parse HEAD)"

echo "== upstream.repos dry run =="
: > "$DOCKER_STUB_CALLS"
out=$(bash "$SUT" "$UPREPO" --dry-run 2>&1); rc=$?
check "exits 0"                       [ "$rc" -eq 0 ]
check "shows parsed upstream entry"   contains "$out" "upstream : updep@jazzy $UPDEP_SRC"
check "steps gains upstream tokens"   contains "$out" "steps    : template+upstream+upstream-hook+rosdep-skip-keys"
check "upstream hook detected"        contains "$out" "upstream-hook : yes"
check "skip keys parsed"              contains "$out" "rosdep-skip-keys: skip_key_a skip_key_b"
check "no container run"              bash -c "! grep -qs docker-run '$DOCKER_STUB_CALLS'"

echo "== upstream.repos successful run attests with resolved SHAs =="
out=$(bash "$SUT" "$UPREPO" 2>&1); rc=$?
check "exits 0"                       [ "$rc" -eq 0 ]
unote=$(git -C "$UPREPO" notes --ref=ci-local show "$UPREPO_SHA" 2>/dev/null)
check "note says pass"                contains "$unote" "ci-local: pass"
check "note scope full"               contains "$unote" "scope: full"
check "note records resolved SHA"     contains "$unote" "upstream-repo: updep@$UPDEP_SHA"
check "note steps include upstream"   contains "$unote" "steps: template+upstream+upstream-hook+rosdep-skip-keys"
check "note records skip keys"        contains "$unote" "rosdep-skip-keys: skip_key_a skip_key_b"

echo "== annotated tag resolves to the peeled commit SHA =="
# The note must record the commit that gets checked out, not the tag object.
git -C "$UPDEP_SRC" -c user.name=t -c user.email=t@t tag -a v_test -m "annotated"
TAG_OBJ="$(git -C "$UPDEP_SRC" rev-parse v_test)"
check "fixture tag is annotated"      bash -c "[ '$TAG_OBJ' != '$UPDEP_SHA' ]"
sed -i 's/version: jazzy/version: v_test/' "$UPREPO/upstream.repos"
git -C "$UPREPO" -c user.name=t -c user.email=t@t commit -qam "pin annotated tag"
TAGPIN_SHA="$(git -C "$UPREPO" rev-parse HEAD)"
out=$(bash "$SUT" "$UPREPO" 2>&1); rc=$?
check "exits 0"                       [ "$rc" -eq 0 ]
tnote=$(git -C "$UPREPO" notes --ref=ci-local show "$TAGPIN_SHA" 2>/dev/null)
check "note records peeled commit"    contains "$tnote" "upstream-repo: updep@$UPDEP_SHA"
check "tag object sha not recorded"   not_contains "$tnote" "$TAG_OBJ"

echo "== upstream.repos injection / validation rejection =="
# Live-tree reads (untracked upstream.repos => dirty tree) let each bad case
# run without committing; every rejection must happen before any container run.
BADUP="$TMP/up_bad"
mkdir -p "$BADUP/p"
sed 's/demo_pkg/badup_pkg/' "$REPO/demo_pkg/package.xml" > "$BADUP/p/package.xml"
git -C "$BADUP" init -q
git -C "$BADUP" -c user.name=t -c user.email=t@t add -A
git -C "$BADUP" -c user.name=t -c user.email=t@t commit -qm badup
: > "$DOCKER_STUB_CALLS"
printf 'repositories:\n  "evil; touch pwned":\n    type: git\n    url: %s\n    version: jazzy\n' "$UPDEP_SRC" > "$BADUP/upstream.repos"
out=$(bash "$SUT" "$BADUP" --dry-run 2>&1); rc=$?
check "metachar dir name rejected"    [ "$rc" -ne 0 ]
check "names the bad dir"             contains "$out" "invalid repo dir name"
printf 'repositories:\n  updep:\n    type: git\n    url: %s\n    version: "jazzy; rm -rf /"\n' "$UPDEP_SRC" > "$BADUP/upstream.repos"
out=$(bash "$SUT" "$BADUP" --dry-run 2>&1); rc=$?
check "metachar version rejected"     [ "$rc" -ne 0 ]
check "names the bad version"         contains "$out" "invalid version"
printf 'repositories:\n  updep:\n    type: git\n    url: "%s evil"\n    version: jazzy\n' "$UPDEP_SRC" > "$BADUP/upstream.repos"
out=$(bash "$SUT" "$BADUP" --dry-run 2>&1); rc=$?
check "whitespace url rejected"       [ "$rc" -ne 0 ]
check "names the bad url"             contains "$out" "invalid url"
printf 'repositories:\n  updep:\n    type: git\n    url: %s\n' "$UPDEP_SRC" > "$BADUP/upstream.repos"
out=$(bash "$SUT" "$BADUP" --dry-run 2>&1); rc=$?
check "missing version rejected"      [ "$rc" -ne 0 ]
check "explains version requirement"  contains "$out" "has no version"
printf 'repositories:\n  updep:\n    url: %s\n    version: jazzy\n' "$UPDEP_SRC" > "$BADUP/upstream.repos"
out=$(bash "$SUT" "$BADUP" --dry-run 2>&1); rc=$?
check "missing type rejected"         [ "$rc" -ne 0 ]
check "explains type requirement"     contains "$out" "has no type"
rm "$BADUP/upstream.repos"
mkdir -p "$BADUP/.agents"
printf 'good_key\nbad key; rm -rf /\n' > "$BADUP/.agents/ci_local_rosdep_skip_keys.txt"
out=$(bash "$SUT" "$BADUP" --dry-run 2>&1); rc=$?
check "bad skip-key rejected"         [ "$rc" -ne 0 ]
check "names the bad key"             contains "$out" "invalid rosdep key"
check "no container run for any bad case" bash -c "! grep -qs docker-run '$DOCKER_STUB_CALLS'"

echo "== unresolvable upstream version fails before the container runs =="
: > "$DOCKER_STUB_CALLS"
printf 'repositories:\n  updep:\n    type: git\n    url: %s\n    version: no_such_branch\n' "$UPDEP_SRC" > "$BADUP/upstream.repos"
rm "$BADUP/.agents/ci_local_rosdep_skip_keys.txt"
out=$(bash "$SUT" "$BADUP" --no-attest 2>&1); rc=$?
check "unresolvable ref rejected"     [ "$rc" -ne 0 ]
check "names the unresolvable ref"    contains "$out" "cannot resolve 'no_such_branch'"
check "no container run"              bash -c "! grep -qs docker-run '$DOCKER_STUB_CALLS'"

echo "== no upstream.repos stays byte-compatible (regression) =="
out=$(bash "$SUT" "$REPO" 2>&1); rc=$?
check "exits 0"                       [ "$rc" -eq 0 ]
note=$(note_of "$HEAD_SHA")
check "no upstream-repo lines"        not_contains "$note" "upstream-repo:"
check "no rosdep-skip-keys line"      not_contains "$note" "rosdep-skip-keys:"
check "steps unchanged"               contains "$note" "steps: template"

echo
echo "$PASS passed, $FAIL failed"
[ "$FAIL" -eq 0 ]
