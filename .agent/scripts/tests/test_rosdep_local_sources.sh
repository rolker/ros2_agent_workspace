#!/bin/bash
# .agent/scripts/tests/test_rosdep_local_sources.sh
# Tests for the workspace-local rosdep key mechanism (#654):
#   .agent/scripts/rosdep_local_sources.sh          (aggregation)
#   .agent/scripts/rosdep_local_staleness_check.sh  (enforcement)
#   .agent/scripts/stage_rosdep_manifests.sh        (agent-image staging)
#   .agent/scripts/rosdep_yaml_validate.sh          (shape gate)
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
VALIDATE="$SCRIPTS_DIR/rosdep_yaml_validate.sh"

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
    update)  [ -n "${STUB_UPDATE_MARKER:-}" ] && touch "$STUB_UPDATE_MARKER"; \
             [ -n "${STUB_UPDATE_FAILS:-}" ] && { echo "ERROR: offline" >&2; exit 1; }; exit 0 ;;
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

echo "=== rosdep_yaml_validate.sh: the shape gate ==="
# These files drive a ROOT-LEVEL `rosdep install` on the dev host, in the
# ci_local container and at image-bake time, and rosdep's own format is wider
# than the policy: a nested mapping expresses pip/npm/gem/source rules, and a
# `source` rule downloads an rdmanifest and runs its install script.
SHAPE="$TMP/shape"; mkdir -p "$SHAPE"
shape_rc() { "$VALIDATE" "$1" >/dev/null 2>&1; echo $?; }

printf 'python3-pystac:  # upstream PR owed: ros/rosdistro#1\n  ubuntu: [python3-pystac]\n  debian: [python3-pystac]\n' > "$SHAPE/good.yaml"
check "the documented list form is accepted"  eq "$(shape_rc "$SHAPE/good.yaml")" 0
: > "$SHAPE/empty.yaml"
check "an empty file declares nothing, so it conforms" eq "$(shape_rc "$SHAPE/empty.yaml")" 0

for rule in pip npm gem source; do
    printf 'k:\n  ubuntu:\n    %s:\n      packages: [x]\n' "$rule" > "$SHAPE/$rule.yaml"
    check "a '$rule' rule is rejected"        eq "$(shape_rc "$SHAPE/$rule.yaml")" 1
done
out="$("$VALIDATE" "$SHAPE/pip.yaml" 2>&1)"
check "the message names the rule"            contains "$out" "'pip' rule(s) are not accepted"
check "the message states the accepted form"  contains "$out" "<os>: [<package>, ...]"

printf 'k:\n  ubuntu:\n    noble: [x]\n' > "$SHAPE/codename.yaml"
check "the nested codename form is rejected too" eq "$(shape_rc "$SHAPE/codename.yaml")" 1
printf 'k:\n  ubuntu: somepkg\n' > "$SHAPE/bare.yaml"
check "a bare string instead of a list is rejected" eq "$(shape_rc "$SHAPE/bare.yaml")" 1
printf 'k:\n  ubuntu: [--allow-anything]\n' > "$SHAPE/flag.yaml"
check "a flag-shaped package name is rejected" eq "$(shape_rc "$SHAPE/flag.yaml")" 1
printf 'k:\n  ubuntu: [https://example.invalid/x.tar.gz]\n' > "$SHAPE/url.yaml"
check "a URL instead of a package name is rejected" eq "$(shape_rc "$SHAPE/url.yaml")" 1
printf -- '- not\n- a mapping\n' > "$SHAPE/seq.yaml"
check "a top-level sequence is rejected"      eq "$(shape_rc "$SHAPE/seq.yaml")" 1
printf 'k: [unclosed\n' > "$SHAPE/broken.yaml"
check "unparseable YAML is rejected"          eq "$(shape_rc "$SHAPE/broken.yaml")" 1
check "no argument is a usage error (2)"      eq "$("$VALIDATE" >/dev/null 2>&1; echo $?)" 2

echo "=== the shape gate is applied BEFORE a key can reach a root install ==="
# Aggregation: a rejected file is excluded from the generated list (not merely
# warned about) and the exit is non-zero, so the Makefile stamp fails the build.
BADWS="$TMP/bad_ws"
mkdir -p "$BADWS/layers/main/a_ws/src/good_repo" "$BADWS/layers/main/a_ws/src/bad_repo"
cp "$SHAPE/good.yaml" "$BADWS/layers/main/a_ws/src/good_repo/rosdep.yaml"
cp "$SHAPE/pip.yaml"  "$BADWS/layers/main/a_ws/src/bad_repo/rosdep.yaml"
out="$("$AGG" "$BADWS" 2>&1)"; rc=$?
BADLIST="$BADWS/.rosdep/sources.list.d/30-workspace-local.list"
check "aggregation exits 4 on a rejected file" eq "$rc" 4
check "the rejected repo is NOT in the source list" \
    bash -c "! grep -q bad_repo '$BADLIST'"
check "the conforming repo still is"          grep -q good_repo "$BADLIST"
check "the system lists are still written"    test -f "$BADWS/.rosdep/sources.list.d/20-default.list"

# Staging for the image bake: same gate, same fail-closed answer.
out="$("$STAGE" "$BADWS" "$TMP/stage_bad" 2>&1)"; rc=$?
check "staging exits 4 on a rejected file"    eq "$rc" 4
check "the rejected yaml is not staged"       bash -c "! ls '$TMP/stage_bad/rosdep-local/' 2>/dev/null | grep -q bad_repo"
check "the conforming yaml is staged"         bash -c "ls '$TMP/stage_bad/rosdep-local/' | grep -q good_repo"

echo "=== rosdep_local_staleness_check.sh: a shape rejection is a finding ==="
out="$(PATH="$STUB:$PATH" "$CHECK" "$BADWS" 2>&1)"; rc=$?
check "make validate fails on a rejected file (1)" eq "$rc" 1
check "it names the file"                     contains "$out" "bad_repo/rosdep.yaml"

echo "=== Makefile \$(STAMP)/rosdep-local.done: recoverable vs. fatal ==="
# The stamp recipe drives the generator in the `make build` chain, so its
# handling of the generator's exit codes is load-bearing: exit 3 (rosdep not
# initialized) is the normal state of a clone that has not run `sudo rosdep
# init`, and must not break the build; a rejected rosdep.yaml (exit 4) must.
# Runs the REAL recipe in a sandbox — MAIN_ROOT falls back to CURDIR — against
# a stub generator and the stub rosdep above.
REPO_ROOT="$(cd "$SCRIPTS_DIR/../.." && pwd)"
MKS="$TMP/mk"
mkdir -p "$MKS/.agent/scripts" "$MKS/.make"
cp "$REPO_ROOT/Makefile" "$MKS/Makefile"
# Prerequisite stamps, back-dated so make treats them as up to date and does
# not try to re-run the bootstrap/manifest recipes.
touch -d '2 hours ago' "$MKS/.make/bootstrap.done"
touch -d '1 hour ago'  "$MKS/.make/manifest.done"

# Runs the stamp recipe with the generator stubbed to exit $1.
# Echoes "<recipe-rc> <stamp|no-stamp> <updated|no-update>".
run_stamp() {
    local grc="$1" rc
    printf '#!/bin/bash\ntouch "%s/gen_ran"\nexit %s\n' "$MKS" "$grc" \
        > "$MKS/.agent/scripts/rosdep_local_sources.sh"
    chmod +x "$MKS/.agent/scripts/rosdep_local_sources.sh"
    rm -f "$MKS/.make/rosdep-local.done" "$MKS/gen_ran" "$TMP/rosdep_update_ran"
    ( cd "$MKS" && PATH="$STUB:$PATH" STUB_UPDATE_MARKER="$TMP/rosdep_update_ran" \
        make "$MKS/.make/rosdep-local.done" >"$TMP/mk.out" 2>&1 )
    rc=$?
    printf '%s %s %s' "$rc" \
        "$([ -f "$MKS/.make/rosdep-local.done" ] && echo stamp || echo no-stamp)" \
        "$([ -f "$TMP/rosdep_update_ran" ] && echo updated || echo no-update)"
}

got="$(run_stamp 0)"
check "generator success stamps and refreshes the cache" eq "$got" "0 stamp updated"
got="$(run_stamp 3)"
check "uninitialized rosdep (3) does not fail make build" contains "$got" "0 "
check "uninitialized rosdep leaves no stamp, so the next build retries" \
    contains "$got" "no-stamp"
check "uninitialized rosdep skips the cache refresh" contains "$got" "no-update"
check "uninitialized rosdep says how to fix it" \
    grep -q "bootstrap.sh" "$TMP/mk.out"
got="$(run_stamp 4)"
check "a rejected rosdep.yaml (4) fails the build" bash -c "[ \"${got%% *}\" != 0 ]"
check "a rejected rosdep.yaml leaves no stamp" contains "$got" "no-stamp"

echo "=== agent-entrypoint.sh: the launch-time refresh must reach the AGENT's cache ==="
# rosdep's cache is per-user ($HOME/.ros/rosdep/sources.cache). The entrypoint
# runs as root and drops CMD to $TARGET_USER, so a bare `rosdep update` there
# populates /root's cache only and an in-session resolve of a workspace-local
# key still fails. Executes the real refresh block, sliced out of the
# entrypoint, against stub `rosdep` and `setpriv`.
ENTRY="$REPO_ROOT/.devcontainer/agent/agent-entrypoint.sh"
BLOCK="$TMP/refresh_block.sh"
sed -n '/^if \[ -n "${ROSDEP_SOURCE_PATH:-}"/,/^fi$/p' "$ENTRY" > "$BLOCK"
check "the refresh block was found in the entrypoint" \
    bash -c "grep -q 'rosdep update' '$BLOCK'"
# TARGET_HOME must be resolved BEFORE the block, not at its old first use in
# step 5 — under `set -u` a late assignment is an unbound-variable abort.
check "TARGET_HOME is assigned before the refresh block" bash -c "
    home_ln=\$(grep -n '^TARGET_HOME=' '$ENTRY' | head -1 | cut -d: -f1)
    blk_ln=\$(grep -n '^if \[ -n \"\\\${ROSDEP_SOURCE_PATH:-}\"' '$ENTRY' | head -1 | cut -d: -f1)
    [ -n \"\$home_ln\" ] && [ -n \"\$blk_ln\" ] && [ \"\$home_ln\" -lt \"\$blk_ln\" ]"

STUB2="$TMP/bin2"; mkdir -p "$STUB2"
cat > "$STUB2/rosdep" <<'EOF'
#!/bin/bash
echo "rosdep $1 HOME=$HOME UID=$(id -u) SRC=${ROSDEP_SOURCE_PATH:-unset}" >> "$REFRESH_LOG"
exit 0
EOF
cat > "$STUB2/setpriv" <<'EOF'
#!/bin/bash
echo "setpriv $*" >> "$REFRESH_LOG"
while [ "$#" -gt 0 ] && [ "$1" != "--" ]; do shift; done
shift
exec "$@"
EOF
chmod +x "$STUB2/rosdep" "$STUB2/setpriv"

ESRC="$TMP/entry_sources"; mkdir -p "$ESRC"
ROS_HOME="$TMP/ros_home"; mkdir -p "$ROS_HOME"
run_refresh() {
    rm -f "$TMP/refresh.log"
    ( uid="$(id -u)"; gid="$(id -g)"
      export TARGET_USER=ros TARGET_UID="$uid" TARGET_GID="$gid" \
             TARGET_HOME="$ROS_HOME" ROSDEP_SOURCE_PATH="$ESRC" \
             REFRESH_LOG="$TMP/refresh.log"
      PATH="$STUB2:$PATH" bash "$BLOCK" >/dev/null 2>&1 )
    cat "$TMP/refresh.log" 2>/dev/null
}

printf '# generated\nyaml file:///ws/layers/main/a_ws/src/r/rosdep.yaml\n' \
    > "$ESRC/30-workspace-local.list"
log="$(run_refresh)"
check "refreshes twice — once per cache"  eq "$(grep -c '^rosdep update' <<< "$log")" 2
check "one refresh runs under the agent's HOME" \
    bash -c "grep -q 'rosdep update HOME=$ROS_HOME' <<< \"\$1\"" _ "$log"
check "that refresh is dropped to the target user via setpriv" \
    bash -c "grep -q 'setpriv .*--reuid=' <<< \"\$1\"" _ "$log"
check "the dropped refresh keeps ROSDEP_SOURCE_PATH" \
    bash -c "grep -q 'rosdep update HOME=$ROS_HOME .*SRC=$ESRC' <<< \"\$1\"" _ "$log"

printf '# only a comment\n' > "$ESRC/30-workspace-local.list"
log="$(run_refresh)"
check "a comment-only local list refreshes nothing" bash -c "[ -z \"\$1\" ]" _ "$log"

echo ""
echo "$PASS passed, $FAIL failed"
[ "$FAIL" -eq 0 ]
