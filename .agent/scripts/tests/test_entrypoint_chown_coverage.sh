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
#      What layer A CANNOT see is the call site: it invokes
#      fix-volume-ownership.sh itself, with both roots. Deleting the
#      "${WORKTREE_ROOT:-}" argument from agent-entrypoint.sh — #604 exactly —
#      leaves layer A green.
#
#   B. Container-side end-to-end (skips cleanly with no local Docker/image).
#      Runs the working tree's agent-entrypoint.sh as the image ENTRYPOINT over
#      real anonymous volumes, and asserts the dropped-privilege user it hands
#      off to can write into both of them. This exercises the whole chain the
#      bug lived in — entrypoint call site, argument list, ownership loops,
#      privilege drop — against the property that only exists inside a
#      container: docker initializes an anonymous volume from the IMAGE at that
#      path, not from the host dir underneath it, so it comes up root-owned no
#      matter what the host did.
#
#      Mutation-checked both ways: deleting the "${WORKTREE_ROOT:-}" argument
#      at the entrypoint call site, and deleting loop 2 of
#      fix-volume-ownership.sh, each make this layer FAIL.
#
# Run: bash .agent/scripts/tests/test_entrypoint_chown_coverage.sh

set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPTS_DIR="$(dirname "$SCRIPT_DIR")"
ROOT_DIR="$(dirname "$(dirname "$SCRIPTS_DIR")")"   # .agent/scripts/tests -> repo root
DRA="$SCRIPTS_DIR/docker_run_agent.sh"
OWNERSHIP_SH="$ROOT_DIR/.devcontainer/agent/fix-volume-ownership.sh"
ENTRYPOINT_SH="$ROOT_DIR/.devcontainer/agent/agent-entrypoint.sh"

TEST_PASS=0
TEST_FAIL=0
pass() { echo "PASS: $1"; TEST_PASS=$((TEST_PASS + 1)); }
fail() { echo "FAIL: $1"; TEST_FAIL=$((TEST_FAIL + 1)); }
skip() { echo "SKIP: $1"; }
summarize_and_exit() {
    echo
    echo "entrypoint chown-coverage tests: $TEST_PASS passed, $TEST_FAIL failed"
    [ "$TEST_FAIL" -eq 0 ]
    exit
}

# The image name is the launcher's, read from the launcher — hardcoding it here
# means a rename downgrades layer B to a permanent silent SKIP instead of a
# failure.
IMAGE_NAME=$(sed -n 's/^IMAGE_NAME="\([^"]*\)".*/\1/p' "$DRA" | head -1)
IMAGE_TAG=$(sed -n 's/^IMAGE_TAG="\([^"]*\)".*/\1/p' "$DRA" | head -1)
IMAGE="$IMAGE_NAME:$IMAGE_TAG"

for f in "$OWNERSHIP_SH" "$ENTRYPOINT_SH"; do
    if [ ! -f "$f" ]; then
        fail "startup script not found at $f"
        summarize_and_exit
    fi
done
if [ -z "$IMAGE_NAME" ] || [ -z "$IMAGE_TAG" ]; then
    fail "could not read IMAGE_NAME/IMAGE_TAG from $DRA (renamed?)"
    summarize_and_exit
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
STUB_BIN="$(mktemp -d /tmp/chown_stub.XXXXXX)"
CROOT=""
cleanup() { rm -rf "$ROOT" "$STUB_BIN" ${CROOT:+"$CROOT"}; }
trap cleanup EXIT
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

# An empty set would make every comparison below vacuously true, so stop here
# rather than reporting passes that assert nothing.
if [ -z "$mounted" ]; then
    fail "fixture produced no *_ws anonymous volumes to check"
    summarize_and_exit
fi

# Run the REAL ownership loops over the same fixture, as the current user.
# Only `chown` is stubbed — it just records the paths the loops reach. `mkdir`
# runs for real inside the fixture so the loops' create-then-chown branch is
# exercised the same way it is in a container. The script's own control flow
# (including the [ ! -L ] guard) therefore decides the recorded set.
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

# ---------- A2. Workspace worktree: loop 2 is a clean no-op ----------
# A workspace worktree (.workspace-worktrees/issue-workspace-<N>) has no *_ws
# directories of its own. The script header claims loop 2 is a no-op there;
# assert it rather than trusting the comment.
WWT="$ROOT/.workspace-worktrees/issue-workspace-$ISSUE"
mkdir -p "$WWT/docs"
: > "$RECORD"
PATH="$STUB_BIN:$PATH" bash "$OWNERSHIP_SH" \
    "$(id -u)" "$(id -g)" "$ROOT" "$WWT" >/dev/null 2>&1
ws_rc=$?
ws_reached=$(grep -F "$WWT" "$RECORD" || true)
if [ "$ws_rc" -eq 0 ] && [ -z "$ws_reached" ]; then
    pass "workspace worktree: ownership loops touch nothing inside it"
else
    fail "workspace worktree fixture (rc=$ws_rc) reached:
$ws_reached"
fi

# ---------- A3. Bad roots fail loud ----------
# A stale or typo'd root would otherwise make its loop iterate over nothing —
# root-owned volumes and no diagnostic, which is #604's failure mode.
if bash "$OWNERSHIP_SH" "$(id -u)" "$(id -g)" "$ROOT/nope" >/dev/null 2>&1; then
    fail "a non-existent workspace root is accepted silently"
else
    pass "a non-existent workspace root fails loud"
fi
if bash "$OWNERSHIP_SH" "$(id -u)" "$(id -g)" "$ROOT" "$ROOT/nope" >/dev/null 2>&1; then
    fail "a non-existent worktree root is accepted silently (loop 2 no-ops)"
else
    pass "a non-existent worktree root fails loud"
fi

# ---------- B. Container-side end-to-end ----------
# Skips cleanly when Docker or the agent image is unavailable, keeping
# run_script_tests.sh hermetic in CI.
#
# A REMOTE/rootless daemon is also skipped: the bind mounts below resolve on
# the daemon's filesystem, not this one, so a remote daemon would fail on paths
# it cannot see — a spurious failure, not a regression.
docker_is_local() {
    local host="${DOCKER_HOST:-}"
    if [ -z "$host" ]; then
        host=$(docker context inspect --format '{{.Endpoints.docker.Host}}' 2>/dev/null || true)
    fi
    case "$host" in
        ""|unix://*) return 0 ;;
        *) return 1 ;;
    esac
}

if ! command -v docker >/dev/null 2>&1 || ! docker info >/dev/null 2>&1; then
    skip "container end-to-end test (no usable Docker daemon)"
elif ! docker_is_local; then
    skip "container end-to-end test (non-local Docker daemon: ${DOCKER_HOST:-remote context})"
elif ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    skip "container end-to-end test (image $IMAGE not built — run 'make agent-build')"
else
    CROOT="$(mktemp -d /tmp/chown_e2e.XXXXXX)"
    make_fixture "$CROOT" "$SLUG" "$ISSUE"
    CLWT="$CROOT/layers/worktrees/issue-$SLUG-$ISSUE"
    mkdir -p "$CLWT/ui_ws/build" "$CROOT/layers/main/nav_ws/build" "$CROOT/.agent/scripts"

    # agent-entrypoint.sh sources the workspace's setup.bash. The fixture is not
    # a real workspace, so stand in a no-op: this test is about the ownership
    # step and the privilege drop, not about ROS. layers/main/nav_ws has no src/,
    # so the entrypoint's rosdep loop skips it and no apt work happens either.
    cat > "$CROOT/.agent/scripts/setup.bash" <<'STUBSETUP'
# test stub — the real one sources ROS 2 and the layer chain
:
STUBSETUP

    # Anonymous volumes over both a layers/main and a worktree artifact dir,
    # exactly as docker_run_agent.sh sections 4 and 4b do, plus the section-1
    # workspace bind mount underneath them and the two env vars the launcher
    # exports. Both startup scripts are mounted over their baked copies so this
    # tests the working tree's versions, and the container runs the IMAGE'S
    # ENTRYPOINT — so the entrypoint's call into fix-volume-ownership.sh, with
    # its argument list, is part of what is under test.
    e2e_out=$(docker run --rm \
        -e "ROS2_AGENT_WORKSPACE_ROOT=$CROOT" \
        -e "WORKTREE_ROOT=$CLWT" \
        -v "$CROOT:$CROOT" \
        -v "$ENTRYPOINT_SH:/usr/local/bin/agent-entrypoint.sh:ro" \
        -v "$OWNERSHIP_SH:/usr/local/bin/fix-volume-ownership.sh:ro" \
        -v "$CROOT/layers/main/nav_ws/build" \
        -v "$CLWT/ui_ws/build" \
        "$IMAGE" \
        bash -c "
            set -e
            echo \"probe uid=\$(id -u)\"
            for d in '$CROOT/layers/main/nav_ws/build' '$CLWT/ui_ws/build'; do
                touch \"\$d/.probe\" || { echo \"UNWRITABLE \$d\"; exit 1; }
            done
            echo E2E_OK
        " 2>&1)
    e2e_rc=$?

    if [ "$e2e_rc" -eq 0 ] && printf '%s\n' "$e2e_out" | grep -q E2E_OK; then
        pass "container: entrypoint hands off a user that can write to both anonymous volumes"
    else
        fail "container end-to-end test (rc=$e2e_rc):
$e2e_out"
    fi

    # The probe must NOT have run as root, or "writable" proves nothing.
    if printf '%s\n' "$e2e_out" | grep -q '^probe uid=0$'; then
        fail "container: CMD ran as root — the privilege drop did not happen"
    elif printf '%s\n' "$e2e_out" | grep -q '^probe uid='; then
        pass "container: CMD ran as the dropped-privilege user"
    fi

    # The staleness marker must describe what is actually baked (#604). Compare
    # the image's label against a digest of its own baked scripts — computed in
    # a container with nothing mounted over them.
    baked_label=$(docker image inspect "$IMAGE" \
        --format '{{index .Config.Labels "org.ros2-agent.startup-scripts-sha"}}' 2>/dev/null || true)
    if [ -z "$baked_label" ]; then
        skip "baked-scripts digest check (image carries no startup-scripts label)"
    else
        baked_sha=$(docker run --rm --entrypoint bash "$IMAGE" -c \
            'cd /usr/local/bin && cat agent-entrypoint.sh fix-volume-ownership.sh | sha256sum | cut -d" " -f1' 2>&1)
        if [ "$baked_sha" = "$baked_label" ]; then
            pass "image label matches a digest of the scripts actually baked into it"
        else
            fail "image label ($baked_label) does not match its baked scripts ($baked_sha)"
        fi
    fi
fi

summarize_and_exit
