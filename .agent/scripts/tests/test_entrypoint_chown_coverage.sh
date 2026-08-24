#!/bin/bash
# .agent/scripts/tests/test_entrypoint_chown_coverage.sh
# Regression test for #604: every anonymous volume docker_run_agent.sh adds
# must have a matching chown in the container's startup path.
#
# #602 added the section-4b worktree mounts and no matching ownership loop, so
# the volumes came up root:root and the dropped-privilege agent could not build
# in its own worktree. test_docker_run_mount_args.sh asserts the MOUNT half;
# this asserts the OWNERSHIP half, and their pairing.
#
# Two layers, because neither alone is sufficient:
#
#   A. Static coverage (always runs, Docker-free). Drives --print-mounts over a
#      fabricated tree, collects every mounted *_ws/{build,install,log} path,
#      and asserts each is reachable by one of the loops in
#      fix-volume-ownership.sh — by running that script's real loops against
#      the same fixture (as the current user, with chown stubbed) and comparing
#      the two sets. Comparing sets, rather than grepping the script for loop
#      headers, means a reformatted loop doesn't cause a mystery failure while
#      a genuinely uncovered mount still does.
#
#   B. Container-side smoke test (skips cleanly with no Docker/image). The
#      property that actually broke is container-side: docker initializes an
#      anonymous volume from the IMAGE at that path, not from the host dir
#      underneath it, so it comes up root-owned no matter what the host did.
#      Only a real container can observe that. This one runs the working
#      tree's fix-volume-ownership.sh inside a container over real anonymous
#      volumes and asserts the target uid can then write into them — so it
#      tests the script under review, not whatever is baked into the image.
#
# Run: bash .agent/scripts/tests/test_entrypoint_chown_coverage.sh

set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPTS_DIR="$(dirname "$SCRIPT_DIR")"
ROOT_DIR="$(dirname "$(dirname "$SCRIPTS_DIR")")"   # .agent/scripts/tests -> repo root
DRA="$SCRIPTS_DIR/docker_run_agent.sh"
OWNERSHIP_SH="$ROOT_DIR/.devcontainer/agent/fix-volume-ownership.sh"
IMAGE="ros2-agent-workspace-agent:latest"

TEST_PASS=0
TEST_FAIL=0
pass() { echo "PASS: $1"; TEST_PASS=$((TEST_PASS + 1)); }
fail() { echo "FAIL: $1"; TEST_FAIL=$((TEST_FAIL + 1)); }
skip() { echo "SKIP: $1"; }

if [ ! -f "$OWNERSHIP_SH" ]; then
    fail "fix-volume-ownership.sh not found at $OWNERSHIP_SH"
    echo "entrypoint chown-coverage tests: $TEST_PASS passed, $TEST_FAIL failed"
    exit 1
fi

# ---------- Fixture ----------
# Same shape as test_docker_run_mount_args.sh: a layer worktree with one REAL
# *_ws and one SYMLINKED sibling pointing back into layers/main.
make_fixture() {
    local root="$1" slug="$2" issue="$3"
    mkdir -p "$root/layers/main/nav_ws"
    local lwt="$root/layers/worktrees/issue-$slug-$issue"
    mkdir -p "$lwt/ui_ws"
    ln -s "$root/layers/main/nav_ws" "$lwt/nav_ws"
}

SLUG="chowncov"
ISSUE=999604

# ---------- A. Static coverage: mounted set ⊆ chowned set ----------
ROOT="$(mktemp -d /tmp/chown_cov.XXXXXX)"
cleanup_a() { rm -rf "$ROOT"; }
trap cleanup_a EXIT
make_fixture "$ROOT" "$SLUG" "$ISSUE"

mount_out=$(DRA_ROOT_DIR_OVERRIDE="$ROOT" "$DRA" --issue "$ISSUE" --repo-slug "$SLUG" --print-mounts 2>&1)
if [ $? -ne 0 ]; then
    fail "--print-mounts exits 0 (out=$mount_out)"
fi

# Every mounted anonymous volume under the fixture that looks like a layer
# workspace artifact dir. Anonymous volumes are bare paths (no colon); bind
# mounts are "src:dst" and are excluded by the colon test.
mounted=$(printf '%s\n' "$mount_out" \
    | grep -E "^$ROOT/.*_ws/(build|install|log)$" \
    | grep -v ':' | sort -u)

if [ -z "$mounted" ]; then
    fail "fixture produced no *_ws anonymous volumes to check"
fi

# Run the REAL ownership loops over the same fixture, as the current user.
# chown/mkdir are stubbed to just record the paths the loops reach, so the
# script's own control flow (including the [ ! -L ] guard) decides the set.
STUB_BIN="$(mktemp -d /tmp/chown_stub.XXXXXX)"
cleanup_a() { rm -rf "$ROOT" "$STUB_BIN"; }
RECORD="$STUB_BIN/reached.txt"
: > "$RECORD"
cat > "$STUB_BIN/chown" <<STUB
#!/bin/bash
for arg in "\$@"; do
    case "\$arg" in -*|*:*) continue ;; esac
    echo "\$arg" >> "$RECORD"
done
STUB
chmod +x "$STUB_BIN/chown"

# The loops mkdir any mountpoint docker did not create; let that happen for
# real inside the fixture so the chown branch is exercised the same way.
PATH="$STUB_BIN:$PATH" bash "$OWNERSHIP_SH" \
    "$(id -u)" "$(id -g)" "$ROOT" "$ROOT/layers/worktrees/issue-$SLUG-$ISSUE" \
    >/dev/null 2>&1
own_rc=$?
if [ "$own_rc" -ne 0 ]; then
    fail "fix-volume-ownership.sh exits 0 over the fixture (rc=$own_rc)"
else
    pass "fix-volume-ownership.sh exits 0 over the fixture"
fi

chowned=$(sort -u "$RECORD")

uncovered=$(comm -23 <(printf '%s\n' "$mounted") <(printf '%s\n' "$chowned"))
if [ -z "$uncovered" ]; then
    pass "every anonymous *_ws volume the launcher mounts is chowned"
else
    fail "anonymous volumes mounted but never chowned (this is #604):
$uncovered"
fi

# The worktree's REAL ui_ws must be in both sets — proves the check has teeth
# rather than passing because both sets are empty or narrowed to layers/main.
LWT="$ROOT/layers/worktrees/issue-$SLUG-$ISSUE"
if printf '%s\n' "$mounted" | grep -qxF "$LWT/ui_ws/build" \
   && printf '%s\n' "$chowned" | grep -qxF "$LWT/ui_ws/build"; then
    pass "worktree ui_ws/build is both mounted and chowned"
else
    fail "worktree ui_ws/build missing from mounted and/or chowned set"
fi

# The symlinked sibling must be in NEITHER set — both sides skip it, and the
# layers/main target is covered by the first loop.
if printf '%s\n' "$chowned" | grep -qF "$LWT/nav_ws/"; then
    fail "symlinked worktree nav_ws was chowned ([ ! -L ] guard missing)"
else
    pass "symlinked worktree nav_ws is skipped by the ownership loops"
fi

# ---------- B. Container-side smoke test ----------
# The property under test only exists inside a container. Skips cleanly when
# Docker or the agent image is unavailable, keeping run_script_tests.sh
# hermetic in CI.
if ! command -v docker >/dev/null 2>&1 || ! docker info >/dev/null 2>&1; then
    skip "container smoke test (no usable Docker daemon)"
elif ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    skip "container smoke test (image $IMAGE not built — run 'make agent-build')"
else
    CROOT="$(mktemp -d /tmp/chown_smoke.XXXXXX)"
    cleanup_a() { rm -rf "$ROOT" "$STUB_BIN" "$CROOT"; }
    make_fixture "$CROOT" "$SLUG" "$ISSUE"
    CLWT="$CROOT/layers/worktrees/issue-$SLUG-$ISSUE"
    mkdir -p "$CLWT/ui_ws/build" "$CROOT/layers/main/nav_ws/build"

    # Anonymous volumes over both a layers/main and a worktree artifact dir,
    # exactly as docker_run_agent.sh sections 4 and 4b do. The bind mount of
    # $CROOT reproduces the launcher's section-1 workspace mount underneath
    # them. The working tree's ownership script is mounted in, so this tests
    # the script under review rather than the copy baked into the image.
    # --entrypoint bash bypasses agent-entrypoint.sh: this test targets the
    # ownership step alone, not the ROS sourcing and rosdep pass that follow it.
    smoke_out=$(docker run --rm \
        --entrypoint bash \
        -u 0 \
        -v "$CROOT:$CROOT" \
        -v "$OWNERSHIP_SH:/tmp/fix-volume-ownership.sh:ro" \
        -v "$CROOT/layers/main/nav_ws/build" \
        -v "$CLWT/ui_ws/build" \
        "$IMAGE" \
        -c "
            set -e
            bash /tmp/fix-volume-ownership.sh 1000 1000 '$CROOT' '$CLWT'
            for d in '$CROOT/layers/main/nav_ws/build' '$CLWT/ui_ws/build'; do
                setpriv --reuid=1000 --regid=1000 --clear-groups \
                    touch \"\$d/.probe\" || { echo \"UNWRITABLE \$d\"; exit 1; }
            done
            echo SMOKE_OK
        " 2>&1)
    smoke_rc=$?

    if [ "$smoke_rc" -eq 0 ] && printf '%s\n' "$smoke_out" | grep -q SMOKE_OK; then
        pass "container: dropped-privilege user can write to both anonymous volumes"
    else
        fail "container smoke test (rc=$smoke_rc):
$smoke_out"
    fi
fi

echo
echo "entrypoint chown-coverage tests: $TEST_PASS passed, $TEST_FAIL failed"
[ "$TEST_FAIL" -eq 0 ]
