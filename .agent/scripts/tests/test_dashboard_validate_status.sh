#!/bin/bash
# Regression test for dashboard.sh's handling of validate_workspace.py's exit
# codes (#609).
#
# validate_workspace.py gained a third outcome: exit 3 means "no repos are
# configured, so nothing was validated" (un-bootstrapped clone, or a workspace
# worktree, which has no configs/manifest symlink). dashboard.sh used to treat
# every non-zero code as drift, which would send the operator to `make
# validate` — the same empty answer they just got. This pins the three-way
# branch by running the real dashboard.sh in a sandbox whose
# validate_workspace.py is a stub returning a chosen exit code.
#
# Hermetic: no network, no ROS, no access to the real workspace's layers/.
set -uo pipefail

TESTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPTS_DIR="$(dirname "$TESTS_DIR")"
fail=0
pass_count=0

sandbox=$(mktemp -d)
trap 'rm -rf "$sandbox"' EXIT

mkdir -p "$sandbox/.agent/scripts"
cp "$SCRIPTS_DIR/dashboard.sh" "$sandbox/.agent/scripts/dashboard.sh"

# Run dashboard.sh --quick with a validate_workspace.py stub that exits $1,
# and echo the dashboard's line about workspace validation.
run_with_rc() {
    printf '#!/usr/bin/env python3\nimport sys\nsys.exit(%s)\n' "$1" \
        > "$sandbox/.agent/scripts/validate_workspace.py"
    bash "$sandbox/.agent/scripts/dashboard.sh" --quick 2>&1 \
        | grep -iE "\.repos configuration|drift detected|nothing validated" || true
}

check() {
    local label="$1" rc="$2" expect="$3" out
    out=$(run_with_rc "$rc")
    if [[ "$out" == *"$expect"* ]]; then
        echo "  ✅ $label"
        pass_count=$((pass_count + 1))
    else
        echo "  ❌ $label"
        echo "     expected to contain: $expect"
        echo "     got: ${out:-<no matching line>}"
        fail=1
    fi
}

check "exit 0 reports a match" 0 "Workspace matches .repos configuration"
check "exit 1 reports drift" 1 "Workspace drift detected"
check "exit 3 reports an unconfigured workspace, not drift" 3 "No repos configured"
check "exit 3 points at setup, not at make validate" 3 "make setup-all"

# The distinction is the whole point: 3 must not be reported as drift.
out=$(run_with_rc 3)
if [[ "$out" == *"drift detected"* ]]; then
    echo "  ❌ exit 3 must not be reported as drift"
    fail=1
else
    echo "  ✅ exit 3 is not reported as drift"
    pass_count=$((pass_count + 1))
fi

# An unexpected code must still be surfaced, not silently treated as a pass.
out=$(run_with_rc 4)
if [[ "$out" == *"Workspace matches"* || -z "$out" ]]; then
    echo "  ❌ an unexpected exit code must not read as a pass"
    fail=1
else
    echo "  ✅ an unexpected exit code falls through to the drift warning"
    pass_count=$((pass_count + 1))
fi

echo ""
if [ "$fail" -eq 0 ]; then
    echo "✅ test_dashboard_validate_status.sh: $pass_count checks passed."
else
    echo "❌ test_dashboard_validate_status.sh failed."
fi
exit "$fail"
