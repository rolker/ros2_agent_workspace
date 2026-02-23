#!/usr/bin/env bash
# .agent/scripts/tests/test_generate_make_skills.sh
# Tests for the generate_make_skills.sh skill generator

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GENERATOR="$SCRIPT_DIR/../generate_make_skills.sh"
TEST_PASS=0
TEST_FAIL=0

echo "=== Testing generate_make_skills.sh ==="
echo ""

# --- Helpers ---

setup_workspace() {
    TEST_DIR=$(mktemp -d /tmp/test_gen_skills.XXXXXX)
    # Mimic workspace layout: Makefile at root, .claude/skills/ for output
    mkdir -p "$TEST_DIR/.claude/skills"
    mkdir -p "$TEST_DIR/.agent/scripts"
    # Copy the generator into the fake workspace so its WORKSPACE_ROOT resolves correctly
    cp "$GENERATOR" "$TEST_DIR/.agent/scripts/generate_make_skills.sh"
    chmod +x "$TEST_DIR/.agent/scripts/generate_make_skills.sh"
}

cleanup_workspace() {
    rm -rf "$TEST_DIR"
}

run_generator() {
    "$TEST_DIR/.agent/scripts/generate_make_skills.sh" 2>&1
}

# --- Tests ---

# Test 1: Basic skill generation from .PHONY targets
echo "Test 1: Generate skills for .PHONY targets"
setup_workspace
cat > "$TEST_DIR/Makefile" << 'MAKEFILE'
.PHONY: help build test lint

help:
	@echo "  build  - Build all layers"
	@echo "  test   - Run all tests"
	@echo "  lint   - Run linters"

build:
	@echo "building"

test:
	@echo "testing"

lint:
	@echo "linting"
MAKEFILE
output=$(run_generator)
# Should create make_build, make_test, make_lint but NOT make_help
if [[ -f "$TEST_DIR/.claude/skills/make_build/SKILL.md" ]] && \
   [[ -f "$TEST_DIR/.claude/skills/make_test/SKILL.md" ]] && \
   [[ -f "$TEST_DIR/.claude/skills/make_lint/SKILL.md" ]]; then
    echo "  ✅ PASS: Expected skill files created"
    TEST_PASS=$((TEST_PASS + 1))
else
    echo "  ❌ FAIL: Missing expected skill files"
    echo "  Output: $output"
    TEST_FAIL=$((TEST_FAIL + 1))
fi
cleanup_workspace
echo ""

# Test 2: 'help' target is excluded
echo "Test 2: 'help' target is excluded"
setup_workspace
cat > "$TEST_DIR/Makefile" << 'MAKEFILE'
.PHONY: help build

help:
	@echo "usage"

build:
	@echo "building"
MAKEFILE
run_generator > /dev/null
if [[ ! -d "$TEST_DIR/.claude/skills/make_help" ]]; then
    echo "  ✅ PASS: 'help' target correctly excluded"
    TEST_PASS=$((TEST_PASS + 1))
else
    echo "  ❌ FAIL: 'help' target should not generate a skill"
    TEST_FAIL=$((TEST_FAIL + 1))
fi
cleanup_workspace
echo ""

# Test 3: Pruning removes stale skills
echo "Test 3: Pruning removes skills for deleted targets"
setup_workspace
cat > "$TEST_DIR/Makefile" << 'MAKEFILE'
.PHONY: help build test

help:
	@echo "usage"

build:
	@echo "building"

test:
	@echo "testing"
MAKEFILE
run_generator > /dev/null
# Verify both exist
if [[ ! -f "$TEST_DIR/.claude/skills/make_build/SKILL.md" ]] || \
   [[ ! -f "$TEST_DIR/.claude/skills/make_test/SKILL.md" ]]; then
    echo "  ❌ FAIL: Initial generation did not create expected skills"
    TEST_FAIL=$((TEST_FAIL + 1))
    cleanup_workspace
else
    # Remove 'test' from .PHONY and re-run
    cat > "$TEST_DIR/Makefile" << 'MAKEFILE'
.PHONY: help build

help:
	@echo "usage"

build:
	@echo "building"
MAKEFILE
    run_generator > /dev/null
    if [[ -f "$TEST_DIR/.claude/skills/make_build/SKILL.md" ]] && \
       [[ ! -d "$TEST_DIR/.claude/skills/make_test" ]]; then
        echo "  ✅ PASS: Stale skill pruned, remaining skill kept"
        TEST_PASS=$((TEST_PASS + 1))
    else
        echo "  ❌ FAIL: Pruning did not work correctly"
        echo "  make_build exists: $(test -f "$TEST_DIR/.claude/skills/make_build/SKILL.md" && echo yes || echo no)"
        echo "  make_test exists: $(test -d "$TEST_DIR/.claude/skills/make_test" && echo yes || echo no)"
        TEST_FAIL=$((TEST_FAIL + 1))
    fi
    cleanup_workspace
fi
echo ""

# Test 4: Skill content includes target name and description
echo "Test 4: Skill content includes target name and description"
setup_workspace
cat > "$TEST_DIR/Makefile" << 'MAKEFILE'
.PHONY: help build

help:
	@echo "  build  - Build all layers"

build:
	@echo "building"
MAKEFILE
run_generator > /dev/null
content=$(cat "$TEST_DIR/.claude/skills/make_build/SKILL.md")
if echo "$content" | grep -q "make_build" && echo "$content" | grep -q "Build all layers"; then
    echo "  ✅ PASS: Skill content has correct name and description"
    TEST_PASS=$((TEST_PASS + 1))
else
    echo "  ❌ FAIL: Skill content missing expected name or description"
    echo "  Content: $content"
    TEST_FAIL=$((TEST_FAIL + 1))
fi
cleanup_workspace
echo ""

# Test 5: Idempotent re-run reports unchanged
echo "Test 5: Idempotent re-run reports 0 created, N unchanged"
setup_workspace
cat > "$TEST_DIR/Makefile" << 'MAKEFILE'
.PHONY: help build test

help:
	@echo "usage"

build:
	@echo "building"

test:
	@echo "testing"
MAKEFILE
run_generator > /dev/null
output=$(run_generator)
if echo "$output" | grep -q "0 created" && echo "$output" | grep -q "unchanged"; then
    echo "  ✅ PASS: Second run reports no changes"
    TEST_PASS=$((TEST_PASS + 1))
else
    echo "  ❌ FAIL: Second run should report 0 created"
    echo "  Output: $output"
    TEST_FAIL=$((TEST_FAIL + 1))
fi
cleanup_workspace
echo ""

# Test 6: No .PHONY line gives friendly error (tests || true fix)
echo "Test 6: Missing .PHONY line gives friendly error"
setup_workspace
cat > "$TEST_DIR/Makefile" << 'MAKEFILE'
build:
	@echo "building"
MAKEFILE
output=$(run_generator 2>&1 || true)
if echo "$output" | grep -q "No .PHONY targets found"; then
    echo "  ✅ PASS: Friendly error shown for missing .PHONY"
    TEST_PASS=$((TEST_PASS + 1))
else
    echo "  ❌ FAIL: Expected 'No .PHONY targets found' error message"
    echo "  Output: $output"
    TEST_FAIL=$((TEST_FAIL + 1))
fi
cleanup_workspace
echo ""

# Test 7: No Makefile gives error
echo "Test 7: Missing Makefile gives error"
setup_workspace
# Don't create a Makefile
output=$(run_generator 2>&1 || true)
if echo "$output" | grep -q "Makefile not found"; then
    echo "  ✅ PASS: Error shown for missing Makefile"
    TEST_PASS=$((TEST_PASS + 1))
else
    echo "  ❌ FAIL: Expected 'Makefile not found' error message"
    echo "  Output: $output"
    TEST_FAIL=$((TEST_FAIL + 1))
fi
cleanup_workspace
echo ""

# --- Summary ---

echo "========================================"
echo "TEST RESULTS"
echo "========================================"
echo "Passed: $TEST_PASS"
echo "Failed: $TEST_FAIL"
echo "========================================"

if [ $TEST_FAIL -eq 0 ]; then
    echo "✅ All tests passed!"
    exit 0
else
    echo "❌ Some tests failed"
    exit 1
fi
