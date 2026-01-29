#!/bin/bash
# .agent/scripts/verify_change.sh
# Verification tool for agents to run targeted tests (Unit, Lint, etc.)
# Usage: ./verify_change.sh --package <package_name> [--type <unit|lint|all>]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")" # Go up to workspace root

PKG=""
TYPE="all"

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --package) PKG="$2"; shift ;;
        --type) TYPE="$2"; shift ;;
        -h|--help)
            echo "Usage: $0 --package <package_name> [--type <unit|lint|all>]"
            echo "Runs colcon test for the specified package with optional type filtering."
            exit 0
            ;;
        *) echo "Unknown parameter: $1"; exit 1 ;;
    esac
    shift
done

if [ -z "$PKG" ]; then
    echo "Error: --package argument is required."
    echo "Usage: $0 --package <package_name> [--type <unit|lint|all>]"
    exit 1
fi

echo "========================================"
echo "Verifying Change: $PKG"
echo "Type: $TYPE"
echo "========================================"

# Auto-detect layer
FOUND_LAYER=""
if [ -d "$ROOT_DIR/layers" ]; then
    for layer in "$ROOT_DIR/layers/"*; do
        if [ -d "$layer/src" ]; then
            # Simple check: is there a directory with the package name?
            # Using find to handle nested structures
            if [ -n "$(find "$layer/src" -name "$PKG" -type d -print -quit 2>/dev/null)" ]; then
                FOUND_LAYER="$layer"
                break
            fi
        fi
    done
fi

if [ -n "$FOUND_LAYER" ]; then
    echo "📂 Located package in: $FOUND_LAYER"
    cd "$FOUND_LAYER" || exit 1
    
    # Source install if available to ensure environment is ready
    if [ -f "install/setup.bash" ]; then
        source "install/setup.bash"
    fi
else
    echo "❌ Error: Could not locate layer for '$PKG'."
    echo "Search locations checked:"
    echo "  - $ROOT_DIR/layers/*/src"
    exit 1
fi

# Paranoid check against running in root if logic matched root somehow
if [ "$(pwd)" == "$ROOT_DIR" ]; then
    echo "❌ Error: Attempted to run test in workspace root. This is forbidden."
    exit 1
fi

# Not forcing build before test, assuming agent handles build separately 
# or colcon test builds if needed (it usually doesn't, test depends on build).
# Ideally user should run ./scripts/build.sh first or agent does it.

CMD=(colcon test --packages-select "$PKG" --event-handlers console_direct+ --return-code-on-test-failure)

if [ "$TYPE" == "unit" ]; then
    CMD+=("--ctest-args" "-L" "unit")
elif [ "$TYPE" == "lint" ]; then
    CMD+=("--ctest-args" "-L" "lint|copyright|flake8|pep8|cpplint")
fi

echo "Running: ${CMD[*]}"
"${CMD[@]}"
EXIT_CODE=$?

if [ $EXIT_CODE -eq 0 ]; then
    echo "✅ Verification Passed ($TYPE) for $PKG"
else
    echo "❌ Verification Failed ($TYPE) for $PKG"
fi

exit $EXIT_CODE
