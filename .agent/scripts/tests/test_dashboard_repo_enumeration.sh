#!/bin/bash
# Regression test for dashboard.sh's handling of a FAILED repo enumeration (#609).
#
# list_overlay_repos.py now exits 1 when a .repos file will not parse, instead
# of printing the repos that happened to survive. dashboard.sh read it through
# `2>/dev/null` and emptied its repo list on failure, which turned the
# untracked-repo check off silently — every repo on disk then read as tracked,
# and the GitHub section printed a clean table over an empty repo list. Same
# defect class as the fix above it: an absence read as an all-clear.
#
# Hermetic: runs the real dashboard.sh in a sandbox with stubbed
# list_overlay_repos.py, validate_workspace.py, vcs and git. No network, no ROS.
set -uo pipefail

TESTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPTS_DIR="$(dirname "$TESTS_DIR")"
fail=0
pass_count=0

sandbox=$(mktemp -d)
trap 'rm -rf "$sandbox"' EXIT

mkdir -p "$sandbox/.agent/scripts" "$sandbox/layers/main/core_ws/src" "$sandbox/bin"
cp "$SCRIPTS_DIR/dashboard.sh" "$sandbox/.agent/scripts/dashboard.sh"
printf '#!/usr/bin/env python3\nimport sys\nsys.exit(0)\n' \
    > "$sandbox/.agent/scripts/validate_workspace.py"

# `vcs custom` reports one repo on disk that no manifest declares.
cat > "$sandbox/bin/vcs" << 'STUB'
#!/bin/bash
echo "=== ./orphan_repo (git) ==="
echo "## main...origin/main"
STUB
chmod +x "$sandbox/bin/vcs"

# Report whatever exit code / output the test asks of the enumeration.
write_enumeration() {
    local rc="$1" out="$2" err="$3"
    cat > "$sandbox/.agent/scripts/list_overlay_repos.py" << STUB
#!/usr/bin/env python3
import sys
if """$out""".strip():
    print("""$out""".strip())
if """$err""".strip():
    print("""$err""".strip(), file=sys.stderr)
sys.exit($rc)
STUB
}

run_dashboard() {
    PATH="$sandbox/bin:$PATH" bash "$sandbox/.agent/scripts/dashboard.sh" --quick 2>&1
}

check_contains() {
    local label="$1" needle="$2" haystack="$3"
    if [[ "$haystack" == *"$needle"* ]]; then
        echo "  ✅ $label"
        pass_count=$((pass_count + 1))
    else
        echo "  ❌ $label"
        echo "     expected to contain: $needle"
        fail=1
    fi
}

check_absent() {
    local label="$1" needle="$2" haystack="$3"
    if [[ "$haystack" == *"$needle"* ]]; then
        echo "  ❌ $label"
        echo "     must NOT contain: $needle"
        fail=1
    else
        echo "  ✅ $label"
        pass_count=$((pass_count + 1))
    fi
}

# --- A failed enumeration is named, and its consequence is stated ---
write_enumeration 1 "" "ERROR: cannot parse core.repos: mapping values not allowed here"
out=$(run_dashboard)
check_contains "a failed enumeration is reported" "Could not read the configured repo list" "$out"
check_contains "the reason is passed through" "core.repos" "$out"
check_contains "the disabled check is named" "untracked-repo detection is OFF" "$out"
check_absent "a repo is not silently called tracked" "orphan_repo, Untracked" "$out"

# --- False-RED direction: a healthy enumeration says none of that ---
write_enumeration 0 '["declared_repo"]' ""
out=$(run_dashboard)
check_absent "a healthy enumeration warns about nothing" "Could not read the configured repo list" "$out"
check_absent "and does not claim tracking is unknown" "Tracking unknown" "$out"

echo ""
if [ "$fail" -eq 0 ]; then
    echo "✅ test_dashboard_repo_enumeration.sh: $pass_count checks passed."
else
    echo "❌ test_dashboard_repo_enumeration.sh: failures above."
fi
exit "$fail"
