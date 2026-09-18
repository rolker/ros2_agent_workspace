#!/bin/bash
# .agent/scripts/tests/test_planning_doc_probe.sh
# Tests for probe_vision(), probe_roadmap(), probe_decision(), probe_health(),
# probe_all() and the CLI form in planning_doc_probe.sh.
#
# Builds throwaway repos under a mktemp -d root (no real repo is touched —
# the probe reads the filesystem only, so a fabricated directory tree is a
# faithful stand-in). Covers every case listed in the work plan for #634,
# including the explicit graceful-absence case, which instantiates the
# design draft's "absence is never a finding" rule as an automated test.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT="$SCRIPT_DIR/../planning_doc_probe.sh"
TEST_PASS=0
TEST_FAIL=0

TMPDIR_ROOT=$(mktemp -d /tmp/test_planning_doc_probe.XXXXXX)
cleanup() {
    rm -rf "$TMPDIR_ROOT"
}
trap cleanup EXIT

# Source the script under test
# shellcheck source=../planning_doc_probe.sh
source "$SCRIPT"

new_repo() {
    local repo
    repo=$(mktemp -d "$TMPDIR_ROOT/repo.XXXXXX")
    echo "$repo"
}

assert_eq() {
    local desc="$1" expected="$2" actual="$3"
    if [ "$expected" == "$actual" ]; then
        echo "✅ PASS: $desc"
        TEST_PASS=$((TEST_PASS + 1))
    else
        echo "❌ FAIL: $desc — expected '$expected', got '$actual'"
        TEST_FAIL=$((TEST_FAIL + 1))
    fi
}

# ---------------------------------------------------------------------------
echo "=== probe_vision: heading variants ==="

REPO=$(new_repo)
{
    echo "# Some Project"
    echo ""
    echo "## Vision"
    echo ""
    echo "We do things."
} > "$REPO/README.md"
assert_eq "bare '## Vision' heading -> present" "present	README.md" "$(probe_vision "$REPO")"

REPO=$(new_repo)
{
    echo "# Some Project"
    echo ""
    echo "## Vision and Goals"
} > "$REPO/README.md"
assert_eq "'## Vision and Goals' (trailing text) -> present" "present	README.md" "$(probe_vision "$REPO")"

REPO=$(new_repo)
{
    echo "# Some Project"
    echo ""
    echo "### Vision"
} > "$REPO/README.md"
assert_eq "'### Vision' (wrong heading level) -> absent" "absent	" "$(probe_vision "$REPO")"

REPO=$(new_repo)
{
    echo "# Some Project"
    echo ""
    echo "## Our Vision"
} > "$REPO/README.md"
assert_eq "'## Our Vision' (does not start with Vision) -> absent" "absent	" "$(probe_vision "$REPO")"

REPO=$(new_repo)
{
    echo "# Some Project"
    echo ""
    echo "## About"
} > "$REPO/README.md"
assert_eq "README.md exists, no Vision heading -> absent" "absent	" "$(probe_vision "$REPO")"

REPO=$(new_repo)
assert_eq "no README.md at all -> absent" "absent	" "$(probe_vision "$REPO")"

REPO=$(new_repo)
{
    echo "# Some Project"
    echo ""
    echo "## Visionary Roadmap"
} > "$REPO/README.md"
assert_eq "'## Visionary Roadmap' (same-prefix word, not a boundary) -> absent" "absent	" "$(probe_vision "$REPO")"

if [ "$(id -u)" -ne 0 ]; then
    REPO=$(new_repo)
    {
        echo "# Some Project"
        echo ""
        echo "## Vision"
    } > "$REPO/README.md"
    chmod 000 "$REPO/README.md"
    STDERR_FILE=$(mktemp "$TMPDIR_ROOT/stderr.XXXXXX")
    ACTUAL=$(probe_vision "$REPO" 2>"$STDERR_FILE")
    assert_eq "unreadable README.md -> absent (not a false present)" "absent	" "$ACTUAL"
    if grep -q "permission denied" "$STDERR_FILE"; then
        echo "✅ PASS: unreadable README.md emits a distinct permission-denied stderr diagnostic"
        TEST_PASS=$((TEST_PASS + 1))
    else
        echo "❌ FAIL: unreadable README.md did not emit a permission-denied diagnostic: $(cat "$STDERR_FILE")"
        TEST_FAIL=$((TEST_FAIL + 1))
    fi
    chmod 644 "$REPO/README.md"
else
    echo "⚠️  SKIP: unreadable-README.md case (running as root — permission bits are not enforced)"
fi
echo ""

# ---------------------------------------------------------------------------
echo "=== probe_roadmap ==="

REPO=$(new_repo)
echo "# Roadmap" > "$REPO/ROADMAP.md"
assert_eq "ROADMAP.md present -> present" "present	ROADMAP.md" "$(probe_roadmap "$REPO")"

REPO=$(new_repo)
assert_eq "no ROADMAP.md -> absent" "absent	" "$(probe_roadmap "$REPO")"

REPO=$(new_repo)
ln -s "./nonexistent-target-$$" "$REPO/ROADMAP.md"
assert_eq "ROADMAP.md is a broken symlink -> absent" "absent	" "$(probe_roadmap "$REPO")"

if [ "$(id -u)" -ne 0 ]; then
    REPO=$(new_repo)
    echo "# Roadmap" > "$REPO/ROADMAP.md"
    chmod 100 "$REPO"   # search-only: a stat by known name needs x, not r
    ACTUAL=$(probe_roadmap "$REPO" 2>/dev/null)
    chmod 755 "$REPO"
    assert_eq "search-only (chmod 100) repo_path with ROADMAP.md -> present (r on the parent is not required)" "present	ROADMAP.md" "$ACTUAL"
fi

if [ "$(id -u)" -ne 0 ]; then
    REPO=$(new_repo)
    echo "# Roadmap" > "$REPO/ROADMAP.md"
    chmod 000 "$REPO"
    STDERR_FILE=$(mktemp "$TMPDIR_ROOT/stderr.XXXXXX")
    ACTUAL=$(probe_roadmap "$REPO" 2>"$STDERR_FILE")
    chmod 755 "$REPO"
    assert_eq "unlistable repo_path -> ROADMAP.md absent (not a false present)" "absent	" "$ACTUAL"
    if grep -q "permission denied" "$STDERR_FILE"; then
        echo "✅ PASS: unlistable repo_path emits a distinct permission-denied stderr diagnostic (probe_roadmap)"
        TEST_PASS=$((TEST_PASS + 1))
    else
        echo "❌ FAIL: unlistable repo_path did not emit a permission-denied diagnostic (probe_roadmap): $(cat "$STDERR_FILE")"
        TEST_FAIL=$((TEST_FAIL + 1))
    fi
else
    echo "⚠️  SKIP: unlistable-repo_path case for probe_roadmap (running as root — permission bits are not enforced)"
fi
echo ""

# ---------------------------------------------------------------------------
echo "=== probe_decision ==="

REPO=$(new_repo)
mkdir -p "$REPO/docs/decisions"
echo "# ADR 1" > "$REPO/docs/decisions/0001-example.md"
assert_eq "docs/decisions with an ADR -> present" "present	docs/decisions" "$(probe_decision "$REPO")"

REPO=$(new_repo)
mkdir -p "$REPO/docs/decisions"
assert_eq "docs/decisions empty -> absent" "absent	" "$(probe_decision "$REPO")"

REPO=$(new_repo)
mkdir -p "$REPO/docs/decisions"
touch "$REPO/docs/decisions/.gitkeep"
assert_eq "docs/decisions with only .gitkeep -> absent" "absent	" "$(probe_decision "$REPO")"

REPO=$(new_repo)
assert_eq "no docs/decisions at all -> absent" "absent	" "$(probe_decision "$REPO")"

REPO=$(new_repo)
mkdir -p "$REPO/docs/decisions"
ln -s "./nonexistent-target-$$" "$REPO/docs/decisions/broken-link"
assert_eq "docs/decisions with only a broken symlink -> absent" "absent	" "$(probe_decision "$REPO")"

REPO=$(new_repo)
mkdir -p "$REPO/docs/decisions"
touch "$REPO/docs/decisions/.gitkeep"
echo "# ADR 1" > "$REPO/docs/decisions/0001-example.md"
assert_eq "docs/decisions with .gitkeep AND a real entry -> present" "present	docs/decisions" "$(probe_decision "$REPO")"

REPO=$(new_repo)
mkdir -p "$REPO/real-decisions"
echo "# ADR 1" > "$REPO/real-decisions/0001-example.md"
mkdir -p "$REPO/docs"
ln -s "../real-decisions" "$REPO/docs/decisions"
assert_eq "docs/decisions itself a symlink to a populated dir -> present" "present	docs/decisions" "$(probe_decision "$REPO")"

if [ "$(id -u)" -ne 0 ]; then
    REPO=$(new_repo)
    mkdir -p "$REPO/docs/decisions"
    echo "# ADR 1" > "$REPO/docs/decisions/0001-example.md"
    chmod 000 "$REPO/docs/decisions"
    STDERR_FILE=$(mktemp "$TMPDIR_ROOT/stderr.XXXXXX")
    ACTUAL=$(probe_decision "$REPO" 2>"$STDERR_FILE")
    assert_eq "unlistable docs/decisions -> absent (not a false present)" "absent	" "$ACTUAL"
    if grep -q "permission denied" "$STDERR_FILE"; then
        echo "✅ PASS: unlistable docs/decisions emits a distinct permission-denied stderr diagnostic"
        TEST_PASS=$((TEST_PASS + 1))
    else
        echo "❌ FAIL: unlistable docs/decisions did not emit a permission-denied diagnostic: $(cat "$STDERR_FILE")"
        TEST_FAIL=$((TEST_FAIL + 1))
    fi
    chmod 755 "$REPO/docs/decisions"
else
    echo "⚠️  SKIP: unlistable-docs/decisions case (running as root — permission bits are not enforced)"
fi
echo ""

# ---------------------------------------------------------------------------
echo "=== probe_health ==="

REPO=$(new_repo)
mkdir -p "$REPO/docs"
echo "# Health" > "$REPO/docs/health.md"
assert_eq "docs/health.md present -> present" "present	docs/health.md" "$(probe_health "$REPO")"

REPO=$(new_repo)
assert_eq "no docs/health.md -> absent" "absent	" "$(probe_health "$REPO")"

REPO=$(new_repo)
mkdir -p "$REPO/docs"
ln -s "./nonexistent-target-$$" "$REPO/docs/health.md"
assert_eq "docs/health.md is a broken symlink -> absent" "absent	" "$(probe_health "$REPO")"

if [ "$(id -u)" -ne 0 ]; then
    REPO=$(new_repo)
    mkdir -p "$REPO/docs"
    echo "# Health" > "$REPO/docs/health.md"
    chmod 100 "$REPO/docs"   # search-only: a stat by known name needs x, not r
    ACTUAL=$(probe_health "$REPO" 2>/dev/null)
    chmod 755 "$REPO/docs"
    assert_eq "search-only (chmod 100) docs/ with health.md -> present (r on the parent is not required)" "present	docs/health.md" "$ACTUAL"
fi

if [ "$(id -u)" -ne 0 ]; then
    REPO=$(new_repo)
    mkdir -p "$REPO/docs"
    echo "# Health" > "$REPO/docs/health.md"
    chmod 000 "$REPO/docs"
    STDERR_FILE=$(mktemp "$TMPDIR_ROOT/stderr.XXXXXX")
    ACTUAL=$(probe_health "$REPO" 2>"$STDERR_FILE")
    chmod 755 "$REPO/docs"
    assert_eq "unlistable docs/ -> docs/health.md absent (not a false present)" "absent	" "$ACTUAL"
    if grep -q "permission denied" "$STDERR_FILE"; then
        echo "✅ PASS: unlistable docs/ emits a distinct permission-denied stderr diagnostic (probe_health)"
        TEST_PASS=$((TEST_PASS + 1))
    else
        echo "❌ FAIL: unlistable docs/ did not emit a permission-denied diagnostic (probe_health): $(cat "$STDERR_FILE")"
        TEST_FAIL=$((TEST_FAIL + 1))
    fi
else
    echo "⚠️  SKIP: unlistable-docs/ case for probe_health (running as root — permission bits are not enforced)"
fi
echo ""

# ---------------------------------------------------------------------------
echo "=== probe_all: each kind present alone ==="

REPO=$(new_repo)
{
    echo "# Some Project"
    echo ""
    echo "## Vision"
} > "$REPO/README.md"
EXPECTED=$'vision\tpresent\tREADME.md\nroadmap\tabsent\t\ndecision\tabsent\t\nhealth\tabsent\t'
assert_eq "vision present alone" "$EXPECTED" "$(probe_all "$REPO")"

REPO=$(new_repo)
echo "# Roadmap" > "$REPO/ROADMAP.md"
EXPECTED=$'vision\tabsent\t\nroadmap\tpresent\tROADMAP.md\ndecision\tabsent\t\nhealth\tabsent\t'
assert_eq "roadmap present alone" "$EXPECTED" "$(probe_all "$REPO")"

REPO=$(new_repo)
mkdir -p "$REPO/docs/decisions"
echo "# ADR 1" > "$REPO/docs/decisions/0001-example.md"
EXPECTED=$'vision\tabsent\t\nroadmap\tabsent\t\ndecision\tpresent\tdocs/decisions\nhealth\tabsent\t'
assert_eq "decision present alone" "$EXPECTED" "$(probe_all "$REPO")"

REPO=$(new_repo)
mkdir -p "$REPO/docs"
echo "# Health" > "$REPO/docs/health.md"
EXPECTED=$'vision\tabsent\t\nroadmap\tabsent\t\ndecision\tabsent\t\nhealth\tpresent\tdocs/health.md'
assert_eq "health present alone" "$EXPECTED" "$(probe_all "$REPO")"
echo ""

# ---------------------------------------------------------------------------
echo "=== probe_all: the graceful-absence case (named explicitly) ==="
# A repo with NONE of the four paths. This is the automated instantiation of
# the issue's headline requirement: absence is never a finding, never an
# error, and never a non-zero exit — the probe just reports four absences.

REPO=$(new_repo)
EXPECTED=$'vision\tabsent\t\nroadmap\tabsent\t\ndecision\tabsent\t\nhealth\tabsent\t'
ACTUAL="$(probe_all "$REPO")"
assert_eq "empty repo -> four absent lines" "$EXPECTED" "$ACTUAL"

CLI_STDERR=$(mktemp "$TMPDIR_ROOT/stderr.XXXXXX")
CLI_OUT=$(bash "$SCRIPT" "$REPO" 2>"$CLI_STDERR")
CLI_STATUS=$?
if [ "$CLI_STATUS" -eq 0 ]; then
    echo "✅ PASS: CLI on an all-absent repo exits 0"
    TEST_PASS=$((TEST_PASS + 1))
else
    echo "❌ FAIL: CLI on an all-absent repo exited $CLI_STATUS"
    TEST_FAIL=$((TEST_FAIL + 1))
fi
assert_eq "CLI on an all-absent repo: stdout matches probe_all" "$EXPECTED" "$CLI_OUT"
if [ -s "$CLI_STDERR" ]; then
    echo "❌ FAIL: CLI on an all-absent repo wrote to stderr: $(cat "$CLI_STDERR")"
    TEST_FAIL=$((TEST_FAIL + 1))
else
    echo "✅ PASS: CLI on an all-absent repo — stderr is empty"
    TEST_PASS=$((TEST_PASS + 1))
fi
echo ""

# ---------------------------------------------------------------------------
echo "=== probe_all: all four present together ==="

REPO=$(new_repo)
{
    echo "# Some Project"
    echo ""
    echo "## Vision"
} > "$REPO/README.md"
echo "# Roadmap" > "$REPO/ROADMAP.md"
mkdir -p "$REPO/docs/decisions"
echo "# ADR 1" > "$REPO/docs/decisions/0001-example.md"
echo "# Health" > "$REPO/docs/health.md"
EXPECTED=$'vision\tpresent\tREADME.md\nroadmap\tpresent\tROADMAP.md\ndecision\tpresent\tdocs/decisions\nhealth\tpresent\tdocs/health.md'
assert_eq "all four present" "$EXPECTED" "$(probe_all "$REPO")"

CLI_OUT=$(bash "$SCRIPT" "$REPO")
CLI_STATUS=$?
assert_eq "CLI exit status on all-present repo" "0" "$CLI_STATUS"
assert_eq "CLI stdout on all-present repo matches probe_all" "$EXPECTED" "$CLI_OUT"
echo ""

# ---------------------------------------------------------------------------
echo "=== CLI usage/error cases ==="

set +e
bash "$SCRIPT" >/dev/null 2>&1
STATUS=$?
set -e
assert_eq "CLI with no args -> exit 2" "2" "$STATUS"

set +e
bash "$SCRIPT" "$REPO" extra-arg >/dev/null 2>&1
STATUS=$?
set -e
assert_eq "CLI with extra arg -> exit 2" "2" "$STATUS"

set +e
bash "$SCRIPT" "/nonexistent/path/$$" >/dev/null 2>&1
STATUS=$?
set -e
assert_eq "CLI on nonexistent path -> exit 3" "3" "$STATUS"

NOT_A_DIR=$(mktemp "$TMPDIR_ROOT/not-a-dir.XXXXXX")
set +e
bash "$SCRIPT" "$NOT_A_DIR" >/dev/null 2>&1
STATUS=$?
set -e
assert_eq "CLI on a path that is a file, not a directory -> exit 3" "3" "$STATUS"

if [ "$(id -u)" -ne 0 ]; then
    UNSEARCHABLE=$(new_repo)
    chmod 600 "$UNSEARCHABLE"
    set +e
    bash "$SCRIPT" "$UNSEARCHABLE" >/dev/null 2>&1
    STATUS=$?
    set -e
    assert_eq "CLI on a readable-but-unsearchable (no x bit) repo_path -> exit 3" "3" "$STATUS"
    chmod 755 "$UNSEARCHABLE"
else
    echo "⚠️  SKIP: unsearchable-repo_path case (running as root — permission bits are not enforced)"
fi
echo ""

echo "=== Results ==="
echo "Passed: $TEST_PASS"
echo "Failed: $TEST_FAIL"
exit "$TEST_FAIL"
