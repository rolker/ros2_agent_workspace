#!/bin/bash
# test_ci_local.sh — hermetic tests for ci_local.sh (workspace#573).
# Stubs `docker` on PATH (no containers, no network) and exercises argument
# handling, package discovery (COLCON_IGNORE, name validation), dry-run
# planning, pass/fail exit codes, snapshot-vs-live mount selection, and
# attestation-note semantics (clean vs dirty tree, append-not-overwrite,
# partial scope, --no-attest, failure).
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

echo
echo "$PASS passed, $FAIL failed"
[ "$FAIL" -eq 0 ]
