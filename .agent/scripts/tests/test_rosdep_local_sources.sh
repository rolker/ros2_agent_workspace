#!/bin/bash
# .agent/scripts/tests/test_rosdep_local_sources.sh
# Tests for the workspace-local rosdep key mechanism (#654):
#   .agent/scripts/rosdep_local_sources.sh          (aggregation)
#   .agent/scripts/rosdep_local_staleness_check.sh  (enforcement)
#   .agent/scripts/stage_rosdep_manifests.sh        (agent-image staging)
#
# Hermetic: temp workspace fixtures, a fake system sources dir via
# ROSDEP_SYSTEM_SOURCES_DIR, and a stub `rosdep` on PATH. No network, no ROS,
# no /etc access, no touching the real workspace.
set -uo pipefail

TESTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPTS_DIR="$(dirname "$TESTS_DIR")"
AGG="$SCRIPTS_DIR/rosdep_local_sources.sh"
CHECK="$SCRIPTS_DIR/rosdep_local_staleness_check.sh"
STAGE="$SCRIPTS_DIR/stage_rosdep_manifests.sh"

PASS=0; FAIL=0
check() {
    local label="$1"; shift
    if "$@" >/dev/null 2>&1; then echo "  ✅ $label"; PASS=$((PASS+1));
    else echo "  ❌ $label"; FAIL=1; fi
}
contains() { case "$1" in *"$2"*) return 0;; *) return 1;; esac; }
eq() { [ "$1" = "$2" ]; }

TMP="$(mktemp -d -t rosdep_local_test.XXXXXX)"
trap 'rm -rf "$TMP"' EXIT

# ---- fake system sources dir ------------------------------------------------
# Two files, because the real host carries 10-local.list alongside
# 20-default.list and copying only the default would silently drop a source.
SYS="$TMP/etc_sources"
mkdir -p "$SYS"
echo "yaml https://example.invalid/base.yaml" > "$SYS/20-default.list"
echo "# a site-local overlay"                 > "$SYS/10-local.list"
export ROSDEP_SYSTEM_SOURCES_DIR="$SYS"

# ---- fixture workspace ------------------------------------------------------
WS="$TMP/ws"
mkdir -p "$WS/layers/main/a_ws/src/repo_with" \
         "$WS/layers/main/a_ws/src/repo_without" \
         "$WS/layers/main/b_ws/src/repo_other"
cat > "$WS/layers/main/a_ws/src/repo_with/rosdep.yaml" <<'EOF'
# upstream PR owed: https://github.com/ros/rosdistro/pull/00000
snakemake:
  ubuntu: [snakemake]
EOF

echo "=== rosdep_local_sources.sh: aggregation ==="
out="$("$AGG" "$WS" 2>&1)"; rc=$?
OUT_DIR="$WS/.rosdep/sources.list.d"
LOCAL="$OUT_DIR/30-workspace-local.list"
check "exits 0"                        eq "$rc" 0
check "reports the file count"         contains "$out" "Aggregated 1 project rosdep.yaml file(s)"
check "copies every system *.list"     bash -c "[ -f '$OUT_DIR/20-default.list' ] && [ -f '$OUT_DIR/10-local.list' ]"
check "writes the local list"          test -f "$LOCAL"
yaml_lines="$(grep -c '^yaml ' "$LOCAL")"
check "exactly one yaml line"          eq "$yaml_lines" 1
check "line is the absolute file:// path" \
    grep -qxF "yaml file://$WS/layers/main/a_ws/src/repo_with/rosdep.yaml" "$LOCAL"
check "repo without rosdep.yaml absent" bash -c "! grep -q repo_without '$LOCAL'"

echo "=== rosdep_local_sources.sh: idempotent regeneration ==="
first="$(cat "$LOCAL")"
"$AGG" "$WS" >/dev/null 2>&1
check "second run is byte-identical"   eq "$first" "$(cat "$LOCAL")"
# A repo whose rosdep.yaml goes away must not survive in the generated list:
# the script rebuilds from scratch rather than appending.
rm "$WS/layers/main/a_ws/src/repo_with/rosdep.yaml"
"$AGG" "$WS" >/dev/null 2>&1
check "removed rosdep.yaml drops out"  bash -c "! grep -q repo_with '$LOCAL'"
cat > "$WS/layers/main/a_ws/src/repo_with/rosdep.yaml" <<'EOF'
# upstream PR owed: https://github.com/ros/rosdistro/pull/00000
snakemake:
  ubuntu: [snakemake]
EOF
"$AGG" "$WS" >/dev/null 2>&1
check "re-added rosdep.yaml comes back" grep -q repo_with "$LOCAL"

echo "=== rosdep_local_sources.sh: zero rosdep.yaml is a normal state ==="
EMPTY="$TMP/empty_ws"; mkdir -p "$EMPTY/layers/main"
out="$("$AGG" "$EMPTY" 2>&1)"; rc=$?
check "exits 0"                        eq "$rc" 0
check "still copies the system lists"  test -f "$EMPTY/.rosdep/sources.list.d/20-default.list"
check "local list has no yaml lines"   bash -c "! grep -q '^yaml ' '$EMPTY/.rosdep/sources.list.d/30-workspace-local.list'"

echo "=== rosdep_local_sources.sh: failure paths ==="
out="$("$AGG" 2>&1)"; rc=$?
check "no argument is a usage error (2)" eq "$rc" 2
out="$("$AGG" "$TMP/nope" 2>&1)"; rc=$?
check "missing workspace_root is 2"    eq "$rc" 2
# An uninitialized rosdep must refuse: a generated dir holding ONLY the local
# list would shadow the real default sources for every rosdep call on the host.
out="$(ROSDEP_SYSTEM_SOURCES_DIR="$TMP/no_such_etc" "$AGG" "$WS" 2>&1)"; rc=$?
check "uninitialized rosdep is 3"      eq "$rc" 3
check "names bootstrap.sh as the fix"  contains "$out" "bootstrap.sh"
check "refuses without clobbering"     grep -q '^yaml ' "$LOCAL"

echo "=== stage_rosdep_manifests.sh: stages rosdep.yaml for the image bake ==="
"$STAGE" "$WS" "$TMP/stage" >/dev/null 2>&1
check "staged under rosdep-local/"     test -f "$TMP/stage/rosdep-local/repo_with.yaml"
check "named after the repo dir"       bash -c "! test -e '$TMP/stage/rosdep-local/rosdep.yaml'"
check "content preserved"              grep -q snakemake "$TMP/stage/rosdep-local/repo_with.yaml"
check "no package.xml in that dir"     bash -c "! find '$TMP/stage/rosdep-local' -name package.xml | grep -q ."
out="$("$STAGE" "$EMPTY" "$TMP/stage_empty" 2>&1)"
check "reports zero local yamls"       contains "$out" "0 local rosdep.yaml file(s)"

# ---- stub rosdep ------------------------------------------------------------
# The staleness probe must not reach the network. The stub resolves exactly one
# key, standing in for "this one has landed upstream".
STUB="$TMP/bin"; mkdir -p "$STUB"
cat > "$STUB/rosdep" <<'EOF'
#!/bin/bash
case "${1:-}" in
    update)  [ -n "${STUB_UPDATE_FAILS:-}" ] && { echo "ERROR: offline" >&2; exit 1; }; exit 0 ;;
    resolve) [ "${2:-}" = "python3-numpy" ] && exit 0; exit 1 ;;
esac
exit 1
EOF
chmod +x "$STUB/rosdep"

echo "=== rosdep_local_staleness_check.sh: clean workspace ==="
out="$(PATH="$STUB:$PATH" "$CHECK" "$WS" 2>&1)"; rc=$?
check "marked, unresolvable key is clean (0)" eq "$rc" 0
check "says how many keys it checked"  contains "$out" "1 local rosdep key(s)"
out="$(PATH="$STUB:$PATH" "$CHECK" "$EMPTY" 2>&1)"; rc=$?
check "no rosdep.yaml at all is 0"     eq "$rc" 0
check "says there is nothing to check" contains "$out" "nothing to check"

echo "=== rosdep_local_staleness_check.sh: findings ==="
cat > "$WS/layers/main/b_ws/src/repo_other/rosdep.yaml" <<'EOF'
python3-numpy:  # upstream PR owed: ros/rosdistro#11111
  ubuntu: [python3-numpy]
EOF
out="$(PATH="$STUB:$PATH" "$CHECK" "$WS" 2>&1)"; rc=$?
check "key that resolves upstream fails (1)" eq "$rc" 1
check "names the stale key"            contains "$out" "'python3-numpy' now resolves"
check "tells you to delete it"         contains "$out" "delete the local entry"
rm "$WS/layers/main/b_ws/src/repo_other/rosdep.yaml"

cat > "$WS/layers/main/b_ws/src/repo_other/rosdep.yaml" <<'EOF'
mystery-key:
  ubuntu: [mystery]
EOF
out="$(PATH="$STUB:$PATH" "$CHECK" "$WS" 2>&1)"; rc=$?
check "key with no upstream-PR marker fails (1)" eq "$rc" 1
check "names the unmarked key"         contains "$out" "'mystery-key' has no upstream-PR marker"

echo "=== rosdep_local_staleness_check.sh: offline is SKIPPED, never a verdict ==="
rm "$WS/layers/main/b_ws/src/repo_other/rosdep.yaml"
out="$(STUB_UPDATE_FAILS=1 PATH="$STUB:$PATH" "$CHECK" "$WS" 2>&1)"; rc=$?
check "failed probe update is SKIPPED (3)" eq "$rc" 3
check "says SKIPPED"                   contains "$out" "SKIPPED"
check "reports the reason"             contains "$out" "offline"
check "reports no resolution verdict"  bash -c "! grep -q 'now resolves' <<< '$out'"
# A text-only finding stands on its own even when the probe could not run.
cat > "$WS/layers/main/b_ws/src/repo_other/rosdep.yaml" <<'EOF'
mystery-key:
  ubuntu: [mystery]
EOF
out="$(STUB_UPDATE_FAILS=1 PATH="$STUB:$PATH" "$CHECK" "$WS" 2>&1)"; rc=$?
check "unmarked key still fails (1) while skipped" eq "$rc" 1
rm "$WS/layers/main/b_ws/src/repo_other/rosdep.yaml"

echo "=== rosdep_local_staleness_check.sh: malformed yaml ==="
printf 'not: [a\n  mapping\n' > "$WS/layers/main/b_ws/src/repo_other/rosdep.yaml"
out="$(PATH="$STUB:$PATH" "$CHECK" "$WS" 2>&1)"; rc=$?
check "unparseable rosdep.yaml fails (1)" eq "$rc" 1
rm "$WS/layers/main/b_ws/src/repo_other/rosdep.yaml"

out="$("$CHECK" a b 2>&1)"; rc=$?
check "too many arguments is a usage error (2)" eq "$rc" 2

echo ""
echo "$PASS passed, $FAIL failed"
[ "$FAIL" -eq 0 ]
