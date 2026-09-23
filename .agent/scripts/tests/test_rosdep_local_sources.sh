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
lacks()    { case "$1" in *"$2"*) return 1;; *) return 0;; esac; }
eq() { [ "$1" = "$2" ]; }

# ---- worktree fixture helpers (#659) ----------------------------------------
# The registration check (wt_is_registered) needs a GENUINE git worktree to
# inspect — a plain directory can't exercise it. These build a throwaway
# "project repo" and register/deregister linked worktrees against it, the
# same shape layers/worktrees/<name>/<layer>_ws/src/<repo>/ has in the real
# workspace.
new_git_repo() {
    local dir="$1"
    mkdir -p "$dir"
    git -C "$dir" init -q -b main
    git -C "$dir" -c user.email=t@t -c user.name=t commit -q --allow-empty -m init
}
add_git_worktree() {
    local repo_dir="$1" wt_path="$2" branch="$3"
    mkdir -p "$(dirname "$wt_path")"
    git -C "$repo_dir" worktree add -q -b "$branch" "$wt_path" >/dev/null
}
# Deregisters a linked worktree WITHOUT touching its working directory or
# files — the same shape a merge_pr.sh exit-3 worktree-removal failure (or an
# interrupted manual removal) leaves: the directory and its rosdep.yaml are
# still on disk, but git no longer knows about the worktree.
deregister_git_worktree() {
    local wt_path="$1"
    local gitdir_line admin_dir
    gitdir_line="$(cat "$wt_path/.git")"
    admin_dir="${gitdir_line#gitdir: }"
    rm -rf "$admin_dir"
}

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

echo "=== rosdep_local_sources.sh: a relative workspace_root still yields absolute URIs ==="
# A file:// URI built from a relative path is unresolvable by whatever process
# later reads the list — never this script's own working directory.
( cd "$TMP" && "$AGG" "ws" >/dev/null 2>&1 )
check "relative root exits 0"          bash -c "( cd '$TMP' && '$AGG' ws >/dev/null 2>&1 )"
check "the yaml line is still absolute" \
    grep -qxF "yaml file://$WS/layers/main/a_ws/src/repo_with/rosdep.yaml" "$LOCAL"
check "no relative file:// URI"        bash -c "! grep -q 'file://[^/]' '$LOCAL'"

echo "=== rosdep_local_sources.sh: the published dir is swapped atomically ==="
# The generated directory is HOST-SHARED (setup.bash exports it as
# ROSDEP_SOURCE_PATH for every shell; two worktrees' `make build` both
# regenerate it). Rebuilding it in place would hand a concurrent reader an
# empty or half-copied sources dir, which is "rosdep has no sources at all".
check "out dir is a symlink"           test -L "$OUT_DIR"
check "it resolves to a directory"     test -d "$OUT_DIR"
slot1="$(readlink "$OUT_DIR")"
check "it points into a slots dir"     contains "$slot1" ".sources.list.d.slots/"
"$AGG" "$WS" >/dev/null 2>&1
slot2="$(readlink "$OUT_DIR")"
check "the slot alternates on rebuild" bash -c "[ '$slot1' != '$slot2' ]"
check "the new slot is complete"       bash -c "[ -f '$OUT_DIR/20-default.list' ] && [ -f '$OUT_DIR/10-local.list' ] && [ -f '$OUT_DIR/30-workspace-local.list' ]"

# A pre-#654-swap generated directory (a REAL dir at the published path) must
# migrate to the symlink form rather than wedging every later run.
rm -rf "$OUT_DIR" "$WS/.rosdep/.sources.list.d.slots"
mkdir -p "$OUT_DIR"
echo "stale" > "$OUT_DIR/99-stale.list"
"$AGG" "$WS" >/dev/null 2>&1
check "a real dir migrates to a link"  test -L "$OUT_DIR"
check "the stale list is gone"         bash -c "! [ -f '$OUT_DIR/99-stale.list' ]"

echo "=== rosdep_local_sources.sh: the migration never unpublishes the path ==="
# The migration used to `rm -rf` the pre-existing REAL directory and only THEN
# rename the symlink in, so the published path was absent for as long as
# removing a whole directory tree takes. setup.bash's `[ -d ]` gate and rosdep
# itself both read that ENOENT as "no workspace sources at all".
#
# Sensitivity matters here: a pre-push review reproduced the gap at only
# 40/2000 reader samples against a 4-file directory. This guard widens the
# window the OLD code would open (the stale directory carries MIG_FILLERS extra
# files, so its removal is a real tree walk) and repeats the migration
# MIG_ROUNDS times, each against a spin-polling reader — tens of thousands of
# samples per round. The fixed code swaps the old directory and the new symlink
# atomically (renameat2 RENAME_EXCHANGE) and deletes the old content only
# afterwards; the rename-aside fallback below is still detected by this same
# guard in every round, which is the evidence that it is sensitive enough.
MIG_FILLERS=500
MIG_ROUNDS=12
MIGWS="$TMP/mig_ws"; mkdir -p "$MIGWS/layers/main"
MIG_OUT="$MIGWS/.rosdep/sources.list.d"
mig_log="$TMP/mig_reader.log"
mig_done="$TMP/mig.done"
mig_bad_rounds=0
mig_round=0
while [ "$mig_round" -lt "$MIG_ROUNDS" ]; do
    mig_round=$((mig_round + 1))
    # A pre-swap generation: a REAL directory at the published path, fat enough
    # that removing it is not instantaneous.
    rm -rf "$MIG_OUT" "$MIGWS/.rosdep/.sources.list.d.slots"
    mkdir -p "$MIG_OUT"
    cp "$SYS"/*.list "$MIG_OUT/"
    : > "$MIG_OUT/30-workspace-local.list"
    i=0
    while [ "$i" -lt "$MIG_FILLERS" ]; do : > "$MIG_OUT/filler-$i.list"; i=$((i + 1)); done
    : > "$mig_log"; rm -f "$mig_done"
    (
        while [ ! -e "$mig_done" ]; do
            if [ ! -d "$MIG_OUT" ]; then
                echo "MISSING" >> "$mig_log"
            elif [ ! -f "$MIG_OUT/20-default.list" ] || [ ! -f "$MIG_OUT/10-local.list" ]; then
                echo "INCOMPLETE" >> "$mig_log"
            fi
        done
    ) &
    mig_reader=$!
    "$AGG" "$MIGWS" >/dev/null 2>&1
    : > "$mig_done"
    wait "$mig_reader" 2>/dev/null
    [ -s "$mig_log" ] && mig_bad_rounds=$((mig_bad_rounds + 1))
done
rm -f "$mig_done"
check "no reader ever lost the published dir" eq "$mig_bad_rounds" 0
check "the migrated path is a symlink"     test -L "$MIG_OUT"
check "it resolves to a complete dir"      bash -c "[ -f '$MIG_OUT/20-default.list' ] && [ -f '$MIG_OUT/10-local.list' ] && [ -f '$MIG_OUT/30-workspace-local.list' ]"
check "the old generation is gone"         bash -c "! [ -e '$MIG_OUT/filler-0.list' ]"
check "no aside copy is left behind"       bash -c "! ls '$MIGWS/.rosdep/'.sources.list.d.aside.* >/dev/null 2>&1"

# The default path swaps the directory and the new symlink atomically
# (renameat2 RENAME_EXCHANGE). Where that call is unsupported the script falls
# back to rename-aside-then-publish, whose gap is one rename(2) — small, but
# the guard above still catches it in every round, which is what shows the
# guard is sensitive enough to have caught the reviewed defect. The fallback is
# exercised here for CORRECTNESS only.
rm -rf "$MIG_OUT" "$MIGWS/.rosdep/.sources.list.d.slots"
mkdir -p "$MIG_OUT"; echo "stale" > "$MIG_OUT/99-stale.list"
ROSDEP_SOURCES_FORCE_FALLBACK=1 "$AGG" "$MIGWS" >/dev/null 2>&1
check "the fallback migration lands too"   test -L "$MIG_OUT"
check "it drops the old generation"        bash -c "! [ -f '$MIG_OUT/99-stale.list' ]"
check "and publishes a complete dir"       bash -c "[ -f '$MIG_OUT/20-default.list' ] && [ -f '$MIG_OUT/30-workspace-local.list' ]"

echo "=== rosdep_local_sources.sh: a lock timeout FAILS, it does not fall through ==="
# A timeout is not the same condition as "flock is not installed": it proves
# another writer holds the lock, and two unserialized writers share one build
# slot, so the swap would publish a mid-mutation directory.
if command -v flock >/dev/null 2>&1; then
    LOCKWS="$TMP/lock_ws"; mkdir -p "$LOCKWS/layers/main"
    "$AGG" "$LOCKWS" >/dev/null 2>&1
    before="$(readlink "$LOCKWS/.rosdep/sources.list.d")"
    LOCK_FILE="$LOCKWS/.rosdep/.sources.list.d.lock"
    flock -x "$LOCK_FILE" -c 'sleep 3' &
    holder=$!
    sleep 0.3
    out="$(ROSDEP_SOURCES_LOCK_TIMEOUT=1 "$AGG" "$LOCKWS" 2>&1)"; rc=$?
    kill "$holder" 2>/dev/null; wait "$holder" 2>/dev/null
    check "a held lock times out with 5"   eq "$rc" 5
    check "it says what timed out"         contains "$out" "timed out"
    check "it refuses to go unserialized"  lacks "$out" "regenerating unserialized"
    check "the published dir is untouched" eq "$before" "$(readlink "$LOCKWS/.rosdep/sources.list.d")"
else
    echo "  ⏭  flock not installed — timeout cases skipped"
fi

echo "=== rosdep_local_sources.sh: flock-absent and lock-unopenable are distinct ==="
# "flock unavailable" for an unopenable lock file misdirects the debugging:
# the .rosdep tree is written by the host uid, by root in the agent entrypoint
# and by the agent user, so a permission failure there is plausible.
FAKEBIN="$TMP/fake_path"; mkdir -p "$FAKEBIN"
for tool in bash cp ln mv rm mkdir dirname basename readlink sort cat python3 grep; do
    tp="$(command -v "$tool" 2>/dev/null)" && ln -sf "$tp" "$FAKEBIN/$tool"
done
NOFLOCKWS="$TMP/noflock_ws"; mkdir -p "$NOFLOCKWS/layers/main"
out="$(PATH="$FAKEBIN" "$AGG" "$NOFLOCKWS" 2>&1)"; rc=$?
check "runs without flock (exit 0)"    eq "$rc" 0
check "says flock is not installed"    contains "$out" "flock is not installed"

UNOPENWS="$TMP/unopen_ws"; mkdir -p "$UNOPENWS/layers/main/.rosdep"
mkdir -p "$UNOPENWS/.rosdep"
chmod 500 "$UNOPENWS/.rosdep"
out="$("$AGG" "$UNOPENWS" 2>&1)"; rc=$?
chmod 700 "$UNOPENWS/.rosdep"
if [ "$(id -u)" -eq 0 ]; then
    echo "  ⏭  running as root — unopenable-lock case skipped"
else
    check "an unopenable lock file says so" contains "$out" "could not open lock file"
    check "it does not blame flock"         lacks "$out" "flock is not installed"
fi

echo "=== rosdep_local_sources.sh: a reader never sees an incomplete dir ==="
# Concurrent regenerations + a reader loop: at no instant may the published
# path be missing, be a non-directory, or be missing one of the lists.
reader_log="$TMP/reader.log"
: > "$reader_log"
(
    end=$((SECONDS + 12))
    while [ "$SECONDS" -lt "$end" ]; do
        if [ ! -d "$OUT_DIR" ]; then
            echo "MISSING" >> "$reader_log"
        elif [ ! -f "$OUT_DIR/20-default.list" ] || [ ! -f "$OUT_DIR/10-local.list" ] \
             || [ ! -f "$OUT_DIR/30-workspace-local.list" ]; then
            echo "INCOMPLETE" >> "$reader_log"
        fi
    done
) &
reader_pid=$!
for _ in 1 2 3 4 5 6 7 8; do
    "$AGG" "$WS" >/dev/null 2>&1 &
done
wait
kill "$reader_pid" 2>/dev/null
wait "$reader_pid" 2>/dev/null
check "reader saw no torn directory"   bash -c "! [ -s '$reader_log' ]"
check "the survivor is still complete" bash -c "[ -f '$OUT_DIR/30-workspace-local.list' ]"
check "and still lists the repo"       grep -q "repo_with" "$LOCAL"

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

echo "=== rosdep_local_sources.sh: worktree discovery (#659) ==="
# A rosdep.yaml added on a feature branch lives under
# layers/worktrees/<name>/<layer>_ws/src/<repo>/ until the PR merges — the
# generator must discover it there too, but ONLY while git still registers
# the worktree.
WTREPO="$TMP/wt_repo"; new_git_repo "$WTREPO"
WTWS="$TMP/wt_ws"; mkdir -p "$WTWS/layers/main"
WT_PKG="$WTWS/layers/worktrees/issue-900/a_ws/src/repo_wt"
add_git_worktree "$WTREPO" "$WT_PKG" wt-900
cat > "$WT_PKG/rosdep.yaml" <<'EOF'
# upstream PR owed: https://github.com/ros/rosdistro/pull/00002
snakemake:
  ubuntu: [snakemake]
EOF
out="$("$AGG" "$WTWS" 2>&1)"; rc=$?
WT_LOCAL="$WTWS/.rosdep/sources.list.d/30-workspace-local.list"
check "exits 0"                              eq "$rc" 0
check "a registered worktree yaml is picked up" \
    grep -qxF "yaml file://$WT_PKG/rosdep.yaml" "$WT_LOCAL"

echo "=== rosdep_local_sources.sh: a leftover/deregistered worktree is excluded ==="
WT_PKG2="$WTWS/layers/worktrees/issue-901/a_ws/src/repo_wt2"
add_git_worktree "$WTREPO" "$WT_PKG2" wt-901
cat > "$WT_PKG2/rosdep.yaml" <<'EOF'
mystery-wt-key:
  ubuntu: [mystery]
EOF
deregister_git_worktree "$WT_PKG2"
out="$("$AGG" "$WTWS" 2>&1)"; rc=$?
check "exits 0 despite the leftover directory" eq "$rc" 0
check "the leftover/deregistered worktree yaml is skipped" \
    bash -c "! grep -q repo_wt2 '$WT_LOCAL'"
check "it says the directory is not git-registered" \
    contains "$out" "not a git-registered worktree"
check "the still-registered worktree yaml survives the same run" \
    grep -qxF "yaml file://$WT_PKG/rosdep.yaml" "$WT_LOCAL"

echo "=== rosdep_local_sources.sh: worktree discovery skips symlinked entries ==="
# worktree_create.sh symlinks untouched siblings/non-target layers straight
# back to layers/main; bash's glob follows those transparently, so without
# the skip they would be rediscovered a second time under layers/worktrees.
SYMWS="$TMP/sym_ws"
mkdir -p "$SYMWS/layers/main/a_ws/src/repo_with"
cat > "$SYMWS/layers/main/a_ws/src/repo_with/rosdep.yaml" <<'EOF'
snakemake:
  ubuntu: [snakemake]
EOF
mkdir -p "$SYMWS/layers/worktrees/issue-902"
ln -s "../../main/a_ws" "$SYMWS/layers/worktrees/issue-902/a_ws"
out="$("$AGG" "$SYMWS" 2>&1)"; rc=$?
SYMLIST="$SYMWS/.rosdep/sources.list.d/30-workspace-local.list"
check "exits 0"                             eq "$rc" 0
check "a symlinked non-target layer is not rediscovered" \
    eq "$(grep -c '^yaml ' "$SYMLIST")" 1

mkdir -p "$SYMWS/layers/worktrees/issue-903/b_ws/src"
ln -s "$SYMWS/layers/main/a_ws/src/repo_with" \
      "$SYMWS/layers/worktrees/issue-903/b_ws/src/repo_with"
out="$("$AGG" "$SYMWS" 2>&1)"; rc=$?
check "exits 0 with a symlinked package entry too" eq "$rc" 0
check "a symlinked package entry is not rediscovered" \
    eq "$(grep -c '^yaml ' "$SYMLIST")" 1

echo "=== rosdep_local_sources.sh: conflicting keys across layers/main and a worktree ==="
CONFREPO="$TMP/conf_repo"; new_git_repo "$CONFREPO"
CONFWS="$TMP/conf_ws"
mkdir -p "$CONFWS/layers/main/a_ws/src/repo_main"
cat > "$CONFWS/layers/main/a_ws/src/repo_main/rosdep.yaml" <<'EOF'
sharedkey:
  ubuntu: [pkg-a]
EOF
CONF_WT="$CONFWS/layers/worktrees/issue-904/a_ws/src/repo_wt"
add_git_worktree "$CONFREPO" "$CONF_WT" wt-904
cat > "$CONF_WT/rosdep.yaml" <<'EOF'
sharedkey:
  ubuntu: [pkg-b]
EOF
out="$("$AGG" "$CONFWS" 2>&1)"; rc=$?
CONFLIST="$CONFWS/.rosdep/sources.list.d/30-workspace-local.list"
check "exits 6 on a main-vs-worktree conflict" eq "$rc" 6
check "names the conflicting key"      contains "$out" "sharedkey"
check "names the layers/main file"     contains "$out" "repo_main"
check "names the worktree file"        contains "$out" "repo_wt"
check "excludes the layers/main file"  bash -c "! grep -q repo_main '$CONFLIST'"
check "excludes the worktree file"     bash -c "! grep -q repo_wt '$CONFLIST'"

echo "=== rosdep_local_sources.sh: conflicting keys worktree-vs-worktree ==="
CONF2REPO="$TMP/conf2_repo"; new_git_repo "$CONF2REPO"
CONFWS2="$TMP/conf_ws2"; mkdir -p "$CONFWS2/layers/main"
CONF2_A="$CONFWS2/layers/worktrees/issue-905/a_ws/src/repo_a"
CONF2_B="$CONFWS2/layers/worktrees/issue-906/a_ws/src/repo_b"
add_git_worktree "$CONF2REPO" "$CONF2_A" wt-905
add_git_worktree "$CONF2REPO" "$CONF2_B" wt-906
cat > "$CONF2_A/rosdep.yaml" <<'EOF'
wtkey:
  ubuntu: [x]
EOF
cat > "$CONF2_B/rosdep.yaml" <<'EOF'
wtkey:
  ubuntu: [y]
EOF
out="$("$AGG" "$CONFWS2" 2>&1)"; rc=$?
check "exits 6 on a worktree-vs-worktree conflict" eq "$rc" 6
check "names the key"                  contains "$out" "wtkey"

echo "=== rosdep_local_sources.sh: an identical declaration in main and a worktree dedupes cleanly ==="
DEDUPREPO="$TMP/dedup_repo"; new_git_repo "$DEDUPREPO"
DEDUPWS="$TMP/dedup_ws"
mkdir -p "$DEDUPWS/layers/main/a_ws/src/repo_main"
cat > "$DEDUPWS/layers/main/a_ws/src/repo_main/rosdep.yaml" <<'EOF'
dupkey:
  ubuntu: [same-pkg]
EOF
DEDUP_WT="$DEDUPWS/layers/worktrees/issue-907/a_ws/src/repo_wt"
add_git_worktree "$DEDUPREPO" "$DEDUP_WT" wt-907
cat > "$DEDUP_WT/rosdep.yaml" <<'EOF'
dupkey:
  ubuntu: [same-pkg]
EOF
out="$("$AGG" "$DEDUPWS" 2>&1)"; rc=$?
check "identical declarations exit 0, no false conflict" eq "$rc" 0
check "no conflict reported"           lacks "$out" "conflict"

echo "=== rosdep_local_sources.sh: a shape rejection (4) wins over a simultaneous conflict (6) ==="
PRECREPO="$TMP/prec_repo"; new_git_repo "$PRECREPO"
PRECWS="$TMP/prec_ws"
mkdir -p "$PRECWS/layers/main/a_ws/src/repo_bad" "$PRECWS/layers/main/a_ws/src/repo_main2"
printf 'k:\n  ubuntu:\n    pip:\n      packages: [x]\n' \
    > "$PRECWS/layers/main/a_ws/src/repo_bad/rosdep.yaml"
cat > "$PRECWS/layers/main/a_ws/src/repo_main2/rosdep.yaml" <<'EOF'
otherkey:
  ubuntu: [b]
EOF
PREC_WT="$PRECWS/layers/worktrees/issue-908/a_ws/src/repo_wt"
add_git_worktree "$PRECREPO" "$PREC_WT" wt-908
cat > "$PREC_WT/rosdep.yaml" <<'EOF'
otherkey:
  ubuntu: [a]
EOF
out="$("$AGG" "$PRECWS" 2>&1)"; rc=$?
check "shape rejection (4) wins over the simultaneous conflict (6)" eq "$rc" 4

echo "=== Makefile: ROSDEP_LOCAL_YAMLS includes the worktree glob (#659) ==="
MFILE="$(cd "$SCRIPTS_DIR/../.." && pwd)/Makefile"
check "the worktree wildcard is present in the Makefile" \
    grep -q 'layers/worktrees/\*/\*_ws/src/\*/rosdep.yaml' "$MFILE"

echo "=== stage_rosdep_manifests.sh: stages rosdep.yaml for the image bake ==="
"$STAGE" "$WS" "$TMP/stage" >/dev/null 2>&1
check "staged under rosdep-local/"     test -f "$TMP/stage/rosdep-local/a_ws__repo_with.yaml"
check "named after the repo dir"       bash -c "! test -e '$TMP/stage/rosdep-local/rosdep.yaml'"
check "content preserved"              grep -q snakemake "$TMP/stage/rosdep-local/a_ws__repo_with.yaml"
# Repo directory names are unique only WITHIN a layer's src/. Keyed by basename
# alone, two same-named repos in different layers overwrote each other while
# the reported count still claimed both.
mkdir -p "$WS/layers/main/b_ws/src/repo_with"
cat > "$WS/layers/main/b_ws/src/repo_with/rosdep.yaml" <<'EOF'
# upstream PR owed: https://github.com/ros/rosdistro/pull/00001
python3-pystac:
  ubuntu: [python3-pystac]
EOF
out="$("$STAGE" "$WS" "$TMP/stage_dup" 2>&1)"
check "two same-named repos both stage"  eq "$(ls "$TMP/stage_dup/rosdep-local" | wc -l)" 2
check "the staged count matches"         contains "$out" "2 local rosdep.yaml file(s)"
check "each layer's copy keeps its keys" bash -c "
    grep -q snakemake '$TMP/stage_dup/rosdep-local/a_ws__repo_with.yaml' &&
    grep -q python3-pystac '$TMP/stage_dup/rosdep-local/b_ws__repo_with.yaml'"
# The aggregation path is keyed by absolute path, so it never collided — pin it.
"$AGG" "$WS" >/dev/null 2>&1
check "aggregation lists both copies"    eq "$(grep -c '^yaml ' "$LOCAL")" 2
rm -rf "$WS/layers/main/b_ws/src/repo_with"
"$AGG" "$WS" >/dev/null 2>&1
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

echo "=== rosdep_local_staleness_check.sh: a worktree-only rosdep.yaml is audited too (#659) ==="
STALEREPO="$TMP/stale_repo"; new_git_repo "$STALEREPO"
STALEWS="$TMP/stale_ws"; mkdir -p "$STALEWS/layers/main"
STALE_WT="$STALEWS/layers/worktrees/issue-909/a_ws/src/repo_wt"
add_git_worktree "$STALEREPO" "$STALE_WT" wt-909
cat > "$STALE_WT/rosdep.yaml" <<'EOF'
mystery-wt-key:
  ubuntu: [mystery]
EOF
out="$(PATH="$STUB:$PATH" "$CHECK" "$STALEWS" 2>&1)"; rc=$?
check "a worktree-only unmarked key fails (1)" eq "$rc" 1
check "names the unmarked worktree key" contains "$out" "'mystery-wt-key' has no upstream-PR marker"

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

# One broken file must not cost the report for every other file: a legitimate
# stale or unmarked key elsewhere used to go unreported until someone fixed the
# broken one.
printf 'not: [a\n  mapping\n' > "$WS/layers/main/b_ws/src/repo_other/rosdep.yaml"
cat > "$WS/layers/main/a_ws/src/repo_without/rosdep.yaml" <<'EOF'
mystery-key:
  ubuntu: [mystery]
EOF
out="$(PATH="$STUB:$PATH" "$CHECK" "$WS" 2>&1)"; rc=$?
check "a broken file is still a failure (1)" eq "$rc" 1
check "it names the broken file"       contains "$out" "repo_other/rosdep.yaml"
check "and still reports the other file's key" \
    contains "$out" "'mystery-key' has no upstream-PR marker"
rm "$WS/layers/main/b_ws/src/repo_other/rosdep.yaml" \
   "$WS/layers/main/a_ws/src/repo_without/rosdep.yaml"

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

echo "=== Makefile \$(STAMP)/rosdep-local.done: a DELETED rosdep.yaml invalidates it ==="
# Deleting the file is the DOCUMENTED end of a local key's lifecycle. A
# $(wildcard) prerequisite list only ever shrinks, which never makes a stamp
# stale, so the generated source list kept a `yaml file://…` line for a file
# that no longer existed. The stamp also depends on a file recording the
# current SET of paths, so a removal moves it.
mkdir -p "$MKS/layers/main/x_ws/src/r"
printf 'k:\n  ubuntu: [p]\n' > "$MKS/layers/main/x_ws/src/r/rosdep.yaml"
printf '#!/bin/bash\ntouch "%s/gen_ran"\nexit 0\n' "$MKS" \
    > "$MKS/.agent/scripts/rosdep_local_sources.sh"
chmod +x "$MKS/.agent/scripts/rosdep_local_sources.sh"
run_stamp_keep() {
    ( cd "$MKS" && PATH="$STUB:$PATH" STUB_UPDATE_MARKER="$TMP/rosdep_update_ran" \
        make "$MKS/.make/rosdep-local.done" >"$TMP/mk.out" 2>&1 )
}
rm -f "$MKS/.make/rosdep-local.done" "$MKS/.make/rosdep-local.list" "$MKS/gen_ran"
run_stamp_keep
check "stamp is created with the yaml present" test -f "$MKS/.make/rosdep-local.done"
rm -f "$MKS/gen_ran"
run_stamp_keep
check "an unchanged set does not re-run it"    bash -c "! [ -f '$MKS/gen_ran' ]"
rm -f "$MKS/layers/main/x_ws/src/r/rosdep.yaml"
run_stamp_keep
check "a deleted rosdep.yaml re-runs it"       test -f "$MKS/gen_ran"

echo "=== Makefile: a stray file named FORCE must not silently disable it ==="
# The FORCE idiom is deliberately not .PHONY (.PHONY targets in this Makefile
# are published as /make_* slash commands), so nothing stopped a real FILE
# named FORCE from making the target up to date — which turns off the
# add/rename/delete detection above, silently, and only for whoever has that
# file. The guard turns that into a parse-time error.
: > "$MKS/FORCE"
out="$( cd "$MKS" && make -n "$MKS/.make/rosdep-local.done" 2>&1 )"; rc=$?
rm -f "$MKS/FORCE"
check "a stray FORCE file stops make"      bash -c "[ $rc -ne 0 ]"
check "it names the file and what it hides" contains "$out" "A file named 'FORCE' exists"
check "it says why that matters"           contains "$out" "rosdep.yaml"
check "make parses again once removed"     bash -c "( cd '$MKS' && make -n '$MKS/.make/rosdep-local.done' >/dev/null 2>&1 )"

echo "=== agent-entrypoint.sh: the launch-time refresh must reach the AGENT's cache ==="
# rosdep's cache is per-user ($HOME/.ros/rosdep/sources.cache). The entrypoint
# runs as root and drops CMD to $TARGET_USER, so a bare `rosdep update` there
# populates /root's cache only and an in-session resolve of a workspace-local
# key still fails. Executes the real refresh block, sliced out of the
# entrypoint, against stub `rosdep` and `setpriv`.
ENTRY="$REPO_ROOT/.devcontainer/agent/agent-entrypoint.sh"
BLOCK="$TMP/refresh_block.sh"
# The slice starts at BAKED_ROSDEP_SOURCES= — the helper the `if` calls is
# defined there, and slicing from the `if` alone would leave it undefined.
sed -n '/^# (overridable so the regression test/,/^fi$/p' "$ENTRY" > "$BLOCK"
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
      [ -n "${BAKED_ROSDEP_SOURCES:-}" ] && export BAKED_ROSDEP_SOURCES
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

# The image bakes its OWN non-empty 30-workspace-local.list. When the mounted
# workspace never generated one, setup.bash leaves ROSDEP_SOURCE_PATH pointing
# at the baked dir — the list is then non-empty and non-comment, and an
# unguarded test paid two rosdep updates per launch to rebuild a cache that was
# already correct, while announcing the image's sources as the workspace's.
BAKED="$TMP/baked_sources"; mkdir -p "$BAKED"
printf '# generated\nyaml file:///opt/rosdep-local/a_ws__r.yaml\n' \
    > "$BAKED/30-workspace-local.list"
cp "$BAKED/30-workspace-local.list" "$ESRC/30-workspace-local.list"
log="$(BAKED_ROSDEP_SOURCES="$BAKED" run_refresh)"
check "the image's own baked list refreshes nothing" bash -c "[ -z \"\$1\" ]" _ "$log"
log="$(BAKED_ROSDEP_SOURCES="$ESRC" run_refresh)"
check "ROSDEP_SOURCE_PATH == the baked dir refreshes nothing" \
    bash -c "[ -z \"\$1\" ]" _ "$log"
printf '# generated\nyaml file:///ws/layers/main/a_ws/src/r/rosdep.yaml\n' \
    > "$ESRC/30-workspace-local.list"
log="$(BAKED_ROSDEP_SOURCES="$BAKED" run_refresh)"
check "a list that differs from the baked one still refreshes" \
    eq "$(grep -c '^rosdep update' <<< "$log")" 2

echo ""
echo "$PASS passed, $FAIL failed"
[ "$FAIL" -eq 0 ]
