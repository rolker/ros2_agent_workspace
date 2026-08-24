#!/bin/bash
# Regression test for the `make validate` recipe (#609).
#
# `validate` is two commands: validate_workspace.py, then test_layer_sourcing.sh
# — ADR-0016's named enforcement path for runtime layer chaining. Written as two
# plain recipe lines, make aborts the recipe at the first non-zero status. That
# was harmless while validate_workspace.py only ever exited 0 or 1 on a
# configured workspace; #609 gave it exit 3 for "no repos configured at all",
# which is the *normal* state of every workspace worktree and every
# un-bootstrapped clone. The layer-sourcing guard silently stopped running
# exactly where agents work.
#
# Pins two properties by running the real recipe against stubbed scripts:
#   1. test_layer_sourcing.sh runs even when validate_workspace.py fails.
#   2. The recipe still reports the first failure, and still passes when both do.
#
# Hermetic: a sandbox with the real Makefile and two stub scripts. No network,
# no ROS, no access to the real workspace's layers/.
set -uo pipefail

TESTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$TESTS_DIR/../../.." && pwd)"
fail=0
pass_count=0

sandbox=$(mktemp -d)
trap 'rm -rf "$sandbox"' EXIT

mkdir -p "$sandbox/.agent/scripts"
cp "$ROOT_DIR/Makefile" "$sandbox/Makefile"

# Run `make validate` with both scripts stubbed to the given exit codes.
# Echoes "<recipe-rc> <marker>", where the marker records whether the
# layer-sourcing guard ran at all.
run_validate() {
    local vrc="$1" lrc="$2" rc
    printf '#!/usr/bin/env python3\nimport sys\nsys.exit(%s)\n' "$vrc" \
        > "$sandbox/.agent/scripts/validate_workspace.py"
    printf '#!/bin/bash\ntouch "%s/layer_ran"\nexit %s\n' "$sandbox" "$lrc" \
        > "$sandbox/.agent/scripts/test_layer_sourcing.sh"
    chmod +x "$sandbox/.agent/scripts/test_layer_sourcing.sh"
    rm -f "$sandbox/layer_ran"
    (cd "$sandbox" && make validate >/dev/null 2>&1)
    rc=$?
    if [[ -f "$sandbox/layer_ran" ]]; then echo "$rc ran"; else echo "$rc skipped"; fi
}

check() {
    local label="$1" expect="$2" got="$3"
    if [[ "$got" == "$expect" ]]; then
        echo "  ✅ $label"
        pass_count=$((pass_count + 1))
    else
        echo "  ❌ $label (expected '$expect', got '$got')"
        fail=1
    fi
}

echo "=== make validate: both checks always run (#609) ==="

# make flattens every non-zero recipe status to its own 2 — asserted below, and
# the reason the AGENTS.md rows and the `make help` line describe the codes as
# the *scripts'*, not make's.
check "unconfigured workspace (exit 3) still reaches the layer-sourcing guard" \
    "2 ran" "$(run_validate 3 0)"
check "drift (exit 1) still reaches the layer-sourcing guard" \
    "2 ran" "$(run_validate 1 0)"
check "a layer-sourcing failure alone still fails the recipe" \
    "2 ran" "$(run_validate 0 1)"
check "both passing is a clean recipe" \
    "0 ran" "$(run_validate 0 0)"

echo ""
if [[ "$fail" -eq 0 ]]; then
    echo "✅ make validate recipe: $pass_count checks passed"
else
    echo "❌ make validate recipe: failures above"
fi
exit "$fail"
