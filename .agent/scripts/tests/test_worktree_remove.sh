#!/bin/bash
# .agent/scripts/tests/test_worktree_remove.sh
# Tests for worktree_remove.sh — ROOT resolution (regression for #507)
#
# The bug: worktree_remove.sh derived ROOT from its own on-disk location
# (`dirname dirname SCRIPT_DIR`). When invoked through a worktree's *own* copy of
# .agent/scripts — e.g. merge_pr.sh calling "$SCRIPT_DIR/worktree_remove.sh" from
# inside a worktree — ROOT pointed at the worktree instead of the main tree, so the
# worktree dirs it manages (.workspace-worktrees/, layers/worktrees/) were looked
# for under the worktree and removal failed with "No worktree found".
#
# Note: Not using set -e because we want to continue after test failures.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CREATE_SCRIPT="$SCRIPT_DIR/../worktree_create.sh"
REMOVE_SCRIPT="$SCRIPT_DIR/../worktree_remove.sh"
HELPERS="$SCRIPT_DIR/../_worktree_helpers.sh"
AGG_SCRIPT="$SCRIPT_DIR/../rosdep_local_sources.sh"
VALIDATE_SCRIPT="$SCRIPT_DIR/../rosdep_yaml_validate.sh"
TEST_PASS=0
TEST_FAIL=0

# Build a mock workspace (bare origin + clone) with the create/remove scripts and
# helpers copied into .agent/scripts, then create one workspace worktree for issue
# $1. Leaves cwd at WORKSPACE_DIR. Sets WT_DIR to the created worktree path.
setup_with_worktree() {
    local issue="$1"
    TEST_DIR=$(mktemp -d)
    ORIGIN_DIR="$TEST_DIR/origin.git"
    WORKSPACE_DIR="$TEST_DIR/workspace"

    git init -q --bare "$ORIGIN_DIR"
    git clone -q "$ORIGIN_DIR" "$WORKSPACE_DIR"
    cd "$WORKSPACE_DIR" || return 1
    git config user.email "test@example.com"
    git config user.name "Test User"
    git config commit.gpgsign false
    echo "init" > README.md
    git add README.md
    git commit -q -m "Initial commit"
    git push -q origin HEAD 2>/dev/null

    mkdir -p .workspace-worktrees .agent/scripts configs/manifest
    echo "core" > configs/manifest/layers.txt
    cp "$CREATE_SCRIPT" .agent/scripts/worktree_create.sh
    cp "$REMOVE_SCRIPT" .agent/scripts/worktree_remove.sh
    cp "$HELPERS"       .agent/scripts/_worktree_helpers.sh
    chmod +x .agent/scripts/worktree_create.sh .agent/scripts/worktree_remove.sh

    # Slug auto-detects to "origin" (basename of the bare origin path).
    .agent/scripts/worktree_create.sh --issue "$issue" --type workspace >/dev/null 2>&1
    WT_DIR="$WORKSPACE_DIR/.workspace-worktrees/issue-origin-$issue"
    [ -d "$WT_DIR" ]
}

cleanup() {
    if [ -n "$TEST_DIR" ] && [ -d "$TEST_DIR" ]; then
        cd "$WORKSPACE_DIR" 2>/dev/null && git worktree prune 2>/dev/null
        cd /
        rm -rf "$TEST_DIR"
    fi
}

run_test() {
    local test_name="$1" test_func="$2"
    echo "Test: $test_name"
    if $test_func; then
        echo "  PASS: $test_name"; ((TEST_PASS++))
    else
        echo "  FAIL: $test_name"; ((TEST_FAIL++))
    fi
    echo ""
}

echo "=== Testing worktree_remove.sh ROOT resolution (#507) ==="
echo ""

# Regression: invoke the WORKTREE's own copy of worktree_remove.sh (SCRIPT_DIR is
# inside the worktree), from the main tree. ROOT must resolve to the main tree so
# the worktree is found and removed — not to the worktree itself.
test_remove_via_worktree_copy_resolves_main_root() {
    setup_with_worktree 12321 || { echo "    setup failed"; cleanup; return 1; }

    # Place a copy of the scripts inside the worktree (a real workspace worktree
    # has .agent/scripts checked out; the mock's are untracked, so copy them in).
    mkdir -p "$WT_DIR/.agent/scripts"
    cp "$REMOVE_SCRIPT" "$WT_DIR/.agent/scripts/worktree_remove.sh"
    cp "$HELPERS"       "$WT_DIR/.agent/scripts/_worktree_helpers.sh"
    chmod +x "$WT_DIR/.agent/scripts/worktree_remove.sh"

    # Invoke from the main tree (CALLER_PWD must not be inside the target worktree).
    # --force: the mock worktree has an untracked `layers/` symlink (no .gitignore),
    # which is irrelevant to ROOT resolution. The old bug still prints "No worktree
    # found" even with --force (ROOT mis-resolves, so the dir is never located).
    local output rc
    output=$(cd "$WORKSPACE_DIR" && \
        "$WT_DIR/.agent/scripts/worktree_remove.sh" --issue 12321 --repo-slug origin --force 2>&1)
    rc=$?

    if [[ "$output" == *"No worktree found"* ]]; then
        echo "    ROOT mis-resolved to the worktree (the #507 bug): $output"
        cleanup; return 1
    fi
    if [ $rc -ne 0 ]; then
        echo "    removal exited non-zero (rc=$rc): $output"
        cleanup; return 1
    fi
    if [ -d "$WT_DIR" ]; then
        echo "    worktree dir still present after removal: $WT_DIR"
        cleanup; return 1
    fi
    cleanup; return 0
}
run_test "remove via worktree's own script copy resolves ROOT to main tree (#507)" \
    test_remove_via_worktree_copy_resolves_main_root

# Sanity: the normal main-tree invocation still removes correctly (the new
# git-based resolution must not regress the common path).
test_remove_via_main_tree_copy_still_works() {
    setup_with_worktree 12322 || { echo "    setup failed"; cleanup; return 1; }

    local output rc
    output=$(cd "$WORKSPACE_DIR" && \
        .agent/scripts/worktree_remove.sh --issue 12322 --repo-slug origin --force 2>&1)
    rc=$?

    if [ $rc -ne 0 ] || [ -d "$WT_DIR" ]; then
        echo "    main-tree removal failed (rc=$rc): $output"
        cleanup; return 1
    fi
    cleanup; return 0
}
run_test "remove via main-tree script copy still works" \
    test_remove_via_main_tree_copy_still_works

# ---- #659 round-2: layer-worktree removal regenerates rosdep sources ------
# Build a mock MAIN workspace (a plain git repo, no bare origin needed — these
# tests invoke the main tree's own copy of worktree_remove.sh, never a
# worktree's copy) with a genuine LAYER worktree: a real `git worktree add`
# linked worktree of a `layers/main/<ws>/src/<pkg>` repo, mirroring what
# worktree_create.sh actually produces. Hermetic: ROSDEP_SYSTEM_SOURCES_DIR
# points at a fixture dir, never the real /etc/ros/rosdep/sources.list.d.
LAYER_TEST_DIR=""
setup_with_layer_worktree() {
    local issue="$1" with_rosdep="$2"
    LAYER_TEST_DIR=$(mktemp -d)
    LWORKSPACE_DIR="$LAYER_TEST_DIR/workspace"
    mkdir -p "$LWORKSPACE_DIR"
    git -C "$LWORKSPACE_DIR" init -q -b main
    git -C "$LWORKSPACE_DIR" -c user.email=t@t -c user.name=t commit -q --allow-empty -m init

    mkdir -p "$LWORKSPACE_DIR/.agent/scripts" "$LWORKSPACE_DIR/layers/worktrees" \
             "$LWORKSPACE_DIR/layers/main"
    cp "$REMOVE_SCRIPT"   "$LWORKSPACE_DIR/.agent/scripts/worktree_remove.sh"
    cp "$HELPERS"         "$LWORKSPACE_DIR/.agent/scripts/_worktree_helpers.sh"
    cp "$AGG_SCRIPT"      "$LWORKSPACE_DIR/.agent/scripts/rosdep_local_sources.sh"
    cp "$VALIDATE_SCRIPT" "$LWORKSPACE_DIR/.agent/scripts/rosdep_yaml_validate.sh"
    chmod +x "$LWORKSPACE_DIR/.agent/scripts/"*.sh

    LSYS_DIR="$LAYER_TEST_DIR/etc_sources"; mkdir -p "$LSYS_DIR"
    echo "yaml https://example.invalid/base.yaml" > "$LSYS_DIR/20-default.list"

    LMAIN_PKG_DIR="$LWORKSPACE_DIR/layers/main/a_ws/src/repo_wt"
    mkdir -p "$LMAIN_PKG_DIR"
    git -C "$LMAIN_PKG_DIR" init -q -b main
    git -C "$LMAIN_PKG_DIR" -c user.email=t@t -c user.name=t commit -q --allow-empty -m init
    if [ "$with_rosdep" = "yes" ]; then
        cat > "$LMAIN_PKG_DIR/rosdep.yaml" <<'EOF'
wtkey:
  ubuntu: [wtpkg]
EOF
        git -C "$LMAIN_PKG_DIR" add rosdep.yaml
        git -C "$LMAIN_PKG_DIR" -c user.email=t@t -c user.name=t commit -q -m "add rosdep.yaml"
    fi

    LWT_DIR="$LWORKSPACE_DIR/layers/worktrees/issue-origin-$issue"
    mkdir -p "$LWT_DIR/a_ws/src"
    git -C "$LMAIN_PKG_DIR" worktree add -q -b "feature/issue-$issue" \
        "$LWT_DIR/a_ws/src/repo_wt" >/dev/null

    [ -d "$LWT_DIR" ]
}
layer_cleanup() {
    if [ -n "$LAYER_TEST_DIR" ] && [ -d "$LAYER_TEST_DIR" ]; then
        rm -rf "$LAYER_TEST_DIR"
    fi
    LAYER_TEST_DIR=""
}

test_layer_removal_with_rosdep_regenerates() {
    setup_with_layer_worktree 940 yes || { echo "    setup failed"; layer_cleanup; return 1; }

    local output rc
    output=$(cd "$LWORKSPACE_DIR" && \
        ROSDEP_SYSTEM_SOURCES_DIR="$LSYS_DIR" \
        .agent/scripts/worktree_remove.sh --issue 940 --repo-slug origin --force 2>&1)
    rc=$?
    local local_list="$LWORKSPACE_DIR/.rosdep/sources.list.d/30-workspace-local.list"

    if [ $rc -ne 0 ]; then
        echo "    removal exited non-zero (rc=$rc): $output"; layer_cleanup; return 1
    fi
    if [ -d "$LWT_DIR" ]; then
        echo "    worktree dir still present after removal"; layer_cleanup; return 1
    fi
    if [[ "$output" != *"Regenerating workspace-local rosdep sources"* ]]; then
        echo "    did not announce regeneration: $output"; layer_cleanup; return 1
    fi
    if [ ! -f "$local_list" ]; then
        echo "    regenerated local list was not written"; layer_cleanup; return 1
    fi
    if grep -q "layers/worktrees" "$local_list"; then
        echo "    removed worktree's rosdep.yaml still referenced: $(cat "$local_list")"
        layer_cleanup; return 1
    fi
    if ! grep -qxF "yaml file://$LMAIN_PKG_DIR/rosdep.yaml" "$local_list"; then
        echo "    layers/main's own rosdep.yaml dropped out of the regenerated list"
        layer_cleanup; return 1
    fi
    layer_cleanup; return 0
}
run_test "layer removal with a rosdep.yaml regenerates workspace-local sources (#659)" \
    test_layer_removal_with_rosdep_regenerates

test_layer_removal_without_rosdep_skips_regen() {
    setup_with_layer_worktree 941 no || { echo "    setup failed"; layer_cleanup; return 1; }

    local output rc
    output=$(cd "$LWORKSPACE_DIR" && \
        ROSDEP_SYSTEM_SOURCES_DIR="$LSYS_DIR" \
        .agent/scripts/worktree_remove.sh --issue 941 --repo-slug origin --force 2>&1)
    rc=$?

    if [ $rc -ne 0 ]; then
        echo "    removal exited non-zero (rc=$rc): $output"; layer_cleanup; return 1
    fi
    if [[ "$output" == *"Regenerating workspace-local rosdep sources"* ]]; then
        echo "    regenerated even though the worktree carried no rosdep.yaml: $output"
        layer_cleanup; return 1
    fi
    if [ -e "$LWORKSPACE_DIR/.rosdep" ]; then
        echo "    .rosdep was created even though regeneration should have been skipped"
        layer_cleanup; return 1
    fi
    layer_cleanup; return 0
}
run_test "layer removal without a rosdep.yaml does not regenerate (#659)" \
    test_layer_removal_without_rosdep_skips_regen

test_layer_removal_regens_when_published_list_names_it() {
    # Codex review of PR #661: the rosdep.yaml was published, then deleted
    # and committed on the branch before removal. No file is left to find,
    # but the published list still names the worktree — regeneration must
    # run anyway and drop the dangling line.
    setup_with_layer_worktree 943 yes || { echo "    setup failed"; layer_cleanup; return 1; }
    local local_list="$LWORKSPACE_DIR/.rosdep/sources.list.d/30-workspace-local.list"
    ROSDEP_SYSTEM_SOURCES_DIR="$LSYS_DIR" \
        "$LWORKSPACE_DIR/.agent/scripts/rosdep_local_sources.sh" "$LWORKSPACE_DIR" >/dev/null 2>&1
    if ! grep -q "layers/worktrees/issue-origin-943/" "$local_list" 2>/dev/null; then
        echo "    precondition: worktree's rosdep.yaml was not published"; layer_cleanup; return 1
    fi
    git -C "$LWT_DIR/a_ws/src/repo_wt" rm -q rosdep.yaml
    git -C "$LWT_DIR/a_ws/src/repo_wt" -c user.email=t@t -c user.name=t commit -q -m "drop rosdep.yaml"

    local output rc
    output=$(cd "$LWORKSPACE_DIR" && \
        ROSDEP_SYSTEM_SOURCES_DIR="$LSYS_DIR" \
        .agent/scripts/worktree_remove.sh --issue 943 --repo-slug origin --force 2>&1)
    rc=$?
    if [ $rc -ne 0 ]; then
        echo "    removal exited non-zero (rc=$rc): $output"; layer_cleanup; return 1
    fi
    if [[ "$output" != *"Regenerating workspace-local rosdep sources"* ]]; then
        echo "    did not regenerate for a published-but-deleted rosdep.yaml: $output"
        layer_cleanup; return 1
    fi
    if grep -q "layers/worktrees" "$local_list"; then
        echo "    dangling worktree line survived: $(cat "$local_list")"; layer_cleanup; return 1
    fi
    if [[ "$output" != *"until the next"*"rosdep update"* ]]; then
        echo "    no note that the cache still resolves until rosdep update: $output"
        layer_cleanup; return 1
    fi
    layer_cleanup; return 0
}
run_test "layer removal regenerates when the published list still names the worktree (#659)" \
    test_layer_removal_regens_when_published_list_names_it

test_layer_removal_regen_failure_does_not_fail_removal() {
    setup_with_layer_worktree 942 yes || { echo "    setup failed"; layer_cleanup; return 1; }

    # Swap in a generator stub that always fails, simulating a regeneration
    # error (e.g. a lock timeout on a busy host) — the removal must still
    # report success, loudly warn, and name the exact re-run command.
    cat > "$LWORKSPACE_DIR/.agent/scripts/rosdep_local_sources.sh" <<'EOF'
#!/bin/bash
echo "stub: simulated regeneration failure" >&2
exit 5
EOF
    chmod +x "$LWORKSPACE_DIR/.agent/scripts/rosdep_local_sources.sh"

    local output rc
    output=$(cd "$LWORKSPACE_DIR" && \
        ROSDEP_SYSTEM_SOURCES_DIR="$LSYS_DIR" \
        .agent/scripts/worktree_remove.sh --issue 942 --repo-slug origin --force 2>&1)
    rc=$?

    if [ $rc -ne 0 ]; then
        echo "    a regen failure must not fail the (already-successful) removal (rc=$rc): $output"
        layer_cleanup; return 1
    fi
    if [ -d "$LWT_DIR" ]; then
        echo "    worktree dir still present after removal"; layer_cleanup; return 1
    fi
    if [[ "$output" != *"Warning"*"regeneration exited non-zero"* ]]; then
        echo "    did not warn loudly about the regen failure: $output"; layer_cleanup; return 1
    fi
    # #659 round-3 must-fix: the warning used to report `$?` from the negated
    # `!` test (always 0), not the wrapped command's real exit status, so it
    # always claimed "exit 0" no matter what the generator actually returned.
    # The stub above exits 5 — assert that number, not just the substring the
    # old test settled for, so a regression back to the `$?`-after-`!` bug is
    # caught (it would print "exit 0" instead).
    if [[ "$output" != *"exit 5 — see messages above"* ]]; then
        echo "    did not report the real regeneration exit code (expected 5): $output"
        layer_cleanup; return 1
    fi
    if [[ "$output" != *"rosdep_local_sources.sh $LWORKSPACE_DIR"* ]]; then
        echo "    did not name the exact re-run command: $output"; layer_cleanup; return 1
    fi
    layer_cleanup; return 0
}
run_test "a regeneration failure is loud but does not fail the removal (#659)" \
    test_layer_removal_regen_failure_does_not_fail_removal

echo "========================================"
echo "TEST RESULTS"
echo "========================================"
echo "Passed: $TEST_PASS"
echo "Failed: $TEST_FAIL"
echo "========================================"

if [ $TEST_FAIL -eq 0 ]; then
    echo "All tests passed!"
    exit 0
else
    echo "Some tests failed"
    exit 1
fi
