#!/bin/bash
# .agent/scripts/tests/test_agent_image_build_paths.sh
# Guards the SINGLE build path for the agent image (#604).
#
# The image carries a staleness marker — a combined digest of the startup
# scripts (agent-entrypoint.sh + fix-volume-ownership.sh) stamped at build time
# and compared at every launch. That comparison is only meaningful if every
# build path computes the digest the same way over the same directory. When
# `make agent-build` ran its own `docker build` with an inline cwd-relative
# formula while docker_run_agent.sh hashed its own $ROOT_DIR (rewound out of
# worktrees), a build from a worktree stamped a digest the launcher could never
# match — a permanent, unfixable "stale" warning.
#
# The fix is structural: docker_run_agent.sh --build-only is the only builder,
# and `make agent-build` delegates to it. These assertions keep it that way.
#
# Run: bash .agent/scripts/tests/test_agent_image_build_paths.sh

set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPTS_DIR="$(dirname "$SCRIPT_DIR")"
ROOT_DIR="$(dirname "$(dirname "$SCRIPTS_DIR")")"
DRA="$SCRIPTS_DIR/docker_run_agent.sh"
MAKEFILE="$ROOT_DIR/Makefile"

TEST_PASS=0
TEST_FAIL=0
pass() { echo "PASS: $1"; TEST_PASS=$((TEST_PASS + 1)); }
fail() { echo "FAIL: $1"; TEST_FAIL=$((TEST_FAIL + 1)); }

# The agent-build recipe: from the target line to the next blank-line-separated
# target (recipe lines are tab-indented or `@`-prefixed comments).
recipe=$(awk '/^agent-build:/{f=1;next} f && /^[^\t]/{exit} f' "$MAKEFILE")

if [ -z "$recipe" ]; then
    fail "Makefile has an agent-build recipe"
else
    pass "Makefile has an agent-build recipe"

    if printf '%s\n' "$recipe" | grep -q -- '--build-only'; then
        pass "make agent-build delegates to docker_run_agent.sh --build-only"
    else
        fail "make agent-build does not delegate to docker_run_agent.sh --build-only"
    fi

    # A second builder is the defect this test exists for: it would carry its
    # own --build-arg STARTUP_SCRIPTS_SHA over its own directory.
    # Comment lines (`@#`/`#`) are stripped: the recipe's own comment explains
    # why it must not run `docker build`, and would otherwise trip the check.
    recipe_code=$(printf '%s\n' "$recipe" | grep -vE '^[[:space:]]*@?#')
    if printf '%s\n' "$recipe_code" | grep -qE 'docker build|sha256sum'; then
        fail "make agent-build runs its own docker build / digest formula (#604):
$recipe_code"
    else
        pass "make agent-build carries no second docker build or digest formula"
    fi
fi

# One digest formula, in the launcher, full stop.
sha_sites=$(grep -c 'sha256sum' "$DRA")
if [ "$sha_sites" -eq 1 ]; then
    pass "docker_run_agent.sh computes the startup-scripts digest in exactly one place"
else
    fail "docker_run_agent.sh has $sha_sites sha256sum sites (expected 1)"
fi

# The Dockerfile must consume the build arg and stamp the label the launcher
# reads back; a rename on either side silently disables the whole check.
DOCKERFILE="$ROOT_DIR/.devcontainer/agent/Dockerfile"
label=$(grep -oE 'org\.ros2-agent\.startup-scripts-sha' "$DOCKERFILE" | head -1)
if [ -n "$label" ] && grep -q "$label" "$DRA" && grep -q 'ARG STARTUP_SCRIPTS_SHA' "$DOCKERFILE"; then
    pass "Dockerfile ARG/LABEL and the launcher agree on the marker name"
else
    fail "Dockerfile and launcher disagree on the startup-scripts marker (ARG/LABEL/name)"
fi

# --build-only must parse without --issue and short-circuit before any worktree
# lookup. Paired with --print-mounts it needs no Docker daemon.
FIX_ROOT="$(mktemp -d /tmp/build_paths.XXXXXX)"
trap 'rm -rf "$FIX_ROOT"' EXIT
mkdir -p "$FIX_ROOT/layers/main/nav_ws"
if out=$(DRA_ROOT_DIR_OVERRIDE="$FIX_ROOT" "$DRA" --build-only --print-mounts 2>&1); then
    pass "--build-only exits 0 without --issue (no worktree required)"
else
    fail "--build-only without --issue failed: $out"
fi

echo
echo "agent image build-path tests: $TEST_PASS passed, $TEST_FAIL failed"
[ "$TEST_FAIL" -eq 0 ]
