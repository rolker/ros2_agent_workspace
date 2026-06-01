#!/bin/bash
# .agent/scripts/tests/run_script_tests.sh
# Runs every script-level test and aggregates the result:
#   - test_*.sh  → executed with bash (each self-reports and exits non-zero on failure).
#                  Covers both .agent/scripts/tests/ and the few sibling tests that
#                  live directly in .agent/scripts/ (e.g. test_check_commit_identity.sh).
#   - test_*.py  → run via pytest, which collects both unittest-style
#                  (test_progress_read.py) and pytest-style (test_build_report_generator.py)
# Exits non-zero if any test file fails. The tests are hermetic (temp sandboxes,
# stubbed `gh`, no network, no ROS), so this is safe to run in lightweight CI.
#
# PYTHON env var selects the interpreter for pytest (default: python3); it must
# have pytest installed. Not on PATH — run by absolute path or via `make test-scripts`.
set -uo pipefail

TESTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPTS_DIR="$(dirname "$TESTS_DIR")"
PYTHON="${PYTHON:-python3}"
fail=0
shopt -s nullglob

echo "=== Shell tests ==="
# Sibling tests in scripts/ first, then the tests/ dir. The non-recursive globs
# don't overlap (tests/ is a subdir, not matched by "$SCRIPTS_DIR"/test_*.sh).
for f in "$SCRIPTS_DIR"/test_*.sh "$TESTS_DIR"/test_*.sh; do
    name=$(basename "$f")
    if out=$(bash "$f" 2>&1); then
        echo "  ✅ $name"
    else
        echo "  ❌ $name"
        echo "$out" | sed 's/^/      /'
        fail=1
    fi
done

echo ""
echo "=== Python tests (pytest) ==="
py_tests=("$TESTS_DIR"/test_*.py)
if [ "${#py_tests[@]}" -gt 0 ]; then
    if ! "$PYTHON" -m pytest "$TESTS_DIR" -q; then
        fail=1
    fi
else
    echo "  (none)"
fi

echo ""
if [ "$fail" -eq 0 ]; then
    echo "✅ All script tests passed."
else
    echo "❌ Some script tests failed."
fi
exit "$fail"
