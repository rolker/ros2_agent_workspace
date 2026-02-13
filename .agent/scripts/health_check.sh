#!/bin/bash
# scripts/health_check.sh
# Comprehensive health check for ROS2 Agent Workspace
#
# Usage: ./scripts/health_check.sh
#
# This script checks:
# - ROS2 installation
# - Required tools (vcs, colcon, rosdep)
# - Workspace structure
# - Git repository status
# - Dev tools (pre-commit hooks)
# - Configuration file validity

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

echo "========================================"
echo "ROS2 Agent Workspace Health Check"
echo "========================================"
echo ""

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

check_pass() {
    echo -e "${GREEN}✅ $1${NC}"
}

check_fail() {
    echo -e "${RED}❌ $1${NC}"
}

check_warn() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

FAILED_CHECKS=0

# Check 1: ROS2 Installation
echo "1. Checking ROS2 Installation..."
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    check_pass "ROS2 Jazzy found at /opt/ros/jazzy"
else
    check_fail "ROS2 Jazzy not found. Run: ./.agent/scripts/bootstrap.sh"
    ((FAILED_CHECKS++))
fi
echo ""

# Check 2: Required Tools
echo "2. Checking Required Tools..."

if command -v vcs &> /dev/null; then
    VCS_VERSION=$(vcs --version 2>&1 | head -1)
    check_pass "vcs tool found: $VCS_VERSION"
else
    check_fail "vcs tool not found. Install: pip install vcstool"
    ((FAILED_CHECKS++))
fi

if command -v colcon &> /dev/null; then
    check_pass "colcon found"
else
    check_fail "colcon not found. Install: sudo apt install python3-colcon-common-extensions"
    ((FAILED_CHECKS++))
fi

if command -v rosdep &> /dev/null; then
    check_pass "rosdep found"
else
    check_fail "rosdep not found. Should be installed with ROS2"
    ((FAILED_CHECKS++))
fi

if command -v python3 &> /dev/null; then
    PYTHON_VERSION=$(python3 --version)
    check_pass "Python found: $PYTHON_VERSION"
else
    check_fail "Python3 not found"
    ((FAILED_CHECKS++))
fi

echo ""

# Check 3: Dev Tools (pre-commit)
echo "3. Checking Dev Tools..."
if [ -x "$ROOT_DIR/.venv/bin/pre-commit" ]; then
    check_pass "pre-commit installed in .venv"

    GIT_DIR=$(git -C "$ROOT_DIR" rev-parse --git-dir 2>/dev/null)
    HOOK_FILE="$GIT_DIR/hooks/pre-commit"
    if [ -f "$HOOK_FILE" ]; then
        # Extract the INSTALL_PYTHON path from the hook and verify it exists
        INSTALL_PYTHON=$(sed -n 's/^INSTALL_PYTHON=//p' "$HOOK_FILE" | tr -d "'" | tr -d '"')
        if [ -n "$INSTALL_PYTHON" ] && [ -x "$INSTALL_PYTHON" ]; then
            check_pass "pre-commit hook installed (python: $INSTALL_PYTHON)"
        else
            check_warn "pre-commit hook has invalid INSTALL_PYTHON. Run: make setup-dev"
        fi
    else
        check_warn "pre-commit hook not installed. Run: make setup-dev"
    fi
else
    check_warn "pre-commit not found in .venv. Run: make setup-dev"
fi
echo ""

# Check 4: Python Dependencies
echo "4. Checking Python Dependencies..."
if python3 -c "import yaml" 2>/dev/null; then
    check_pass "PyYAML installed"

    # Verify validate_repos.py can run
    if python3 "$SCRIPT_DIR/validate_repos.py" --help &>/dev/null; then
        check_pass "validate_repos.py is functional"
    else
        check_warn "validate_repos.py may have issues"
    fi
else
    check_warn "PyYAML not found. Install: sudo apt install python3-yaml"
fi
echo ""

# Check 5: Workspace Structure
echo "5. Checking Workspace Structure..."

if [ -d "$ROOT_DIR/configs" ]; then
    REPOS_COUNT=$(ls -1 "$ROOT_DIR/configs"/*.repos 2>/dev/null | wc -l)

    # Check for manifest repo .repos files via symlink
    MANIFEST_REPOS_DIR="$ROOT_DIR/configs/manifest/repos"
    if [ -d "$MANIFEST_REPOS_DIR" ]; then
        MANIFEST_COUNT=$(ls -1 "$MANIFEST_REPOS_DIR"/*.repos 2>/dev/null | wc -l)
        REPOS_COUNT=$((REPOS_COUNT + MANIFEST_COUNT))
    fi

    check_pass "configs/ directory exists with $REPOS_COUNT .repos files"
else
    check_fail "configs/ directory not found"
    ((FAILED_CHECKS++))
fi

if [ -d "$ROOT_DIR/.agent/scripts" ]; then
    SCRIPT_COUNT=$(ls -1 "$ROOT_DIR/.agent/scripts"/*.sh 2>/dev/null | wc -l)
    check_pass "scripts/ directory exists in .agent/scripts with $SCRIPT_COUNT shell scripts"
else
    check_fail ".agent/scripts/ directory not found"
    ((FAILED_CHECKS++))
fi

if [ -d "$ROOT_DIR/.agent" ]; then
    check_pass ".agent/ directory exists"
else
    check_warn ".agent/ directory not found"
fi

echo ""

# Check 6: Validate Configuration Files
echo "6. Validating Configuration Files..."
if [ -f "$SCRIPT_DIR/validate_repos.py" ]; then
    VALIDATION_PASSED=true

    # Validate configs/ (bootstrap)
    if [ -d "$ROOT_DIR/configs" ] && ls "$ROOT_DIR/configs"/*.repos &>/dev/null; then
        if ! python3 "$SCRIPT_DIR/validate_repos.py" --configs-dir "$ROOT_DIR/configs" --strict &>/dev/null; then
             VALIDATION_PASSED=false
             check_fail "Validation failed for $ROOT_DIR/configs"
        fi
    fi

    # Validate manifest repos (if symlink exists)
    MANIFEST_REPOS_DIR="$ROOT_DIR/configs/manifest/repos"
    if [ -d "$MANIFEST_REPOS_DIR" ]; then
        if ! python3 "$SCRIPT_DIR/validate_repos.py" --configs-dir "$MANIFEST_REPOS_DIR" --strict &>/dev/null; then
             VALIDATION_PASSED=false
             check_fail "Validation failed for $MANIFEST_REPOS_DIR"
        fi
    fi

    if [ "$VALIDATION_PASSED" = true ]; then
        check_pass "All existing .repos files are valid"
    else
        check_fail "Some .repos files have errors."
        ((FAILED_CHECKS++))
    fi
else
    check_warn "validate_repos.py not found"
fi
echo ""

# Check 7: Validate Workspace Matches Configuration
echo "7. Validating Workspace Matches Configuration..."
if [ -f "$SCRIPT_DIR/validate_workspace.py" ]; then
    if python3 "$SCRIPT_DIR/validate_workspace.py" &>/dev/null; then
        check_pass "Workspace matches .repos configuration"
    else
        check_warn "Workspace does not match .repos configuration. Run: make validate"
    fi
else
    check_warn "validate_workspace.py not found"
fi
echo ""

# Check 8: Git Status
echo "8. Checking Git Repository..."
cd "$ROOT_DIR"
if git rev-parse --git-dir > /dev/null 2>&1; then
    check_pass "Git repository initialized"

    BRANCH=$(git branch --show-current)
    if [ -n "$BRANCH" ]; then
        check_pass "Current branch: $BRANCH"
    fi

    if [ -n "$(git status --porcelain)" ]; then
        check_warn "Working directory has uncommitted changes"
    else
        check_pass "Working directory is clean"
    fi
else
    check_fail "Not a git repository"
    ((FAILED_CHECKS++))
fi
echo ""

# Check 9: Workspace Layers
echo "9. Checking Workspace Layers..."
if [ -d "$ROOT_DIR/layers/main" ]; then
    LAYER_COUNT=$(ls -d "$ROOT_DIR/layers/main"/*_ws 2>/dev/null | wc -l)
    if [ "$LAYER_COUNT" -gt 0 ]; then
        check_pass "Found $LAYER_COUNT layer(s)"

        for layer_dir in "$ROOT_DIR/layers/main"/*_ws; do
            if [ -d "$layer_dir" ]; then
                layer_name=$(basename "$layer_dir")
                if [ -d "$layer_dir/src" ]; then
                    repo_count=$(find "$layer_dir/src" -mindepth 1 -maxdepth 1 -type d | wc -l)
                    echo "  - $layer_name: $repo_count repositories"

                    if [ -f "$layer_dir/install/setup.bash" ]; then
                        echo "    ✓ Built"
                    else
                        echo "    ⚠ Not built"
                    fi
                fi
            fi
        done
    else
        check_warn "No layers set up. Run: ./.agent/scripts/setup.sh <layer>"
    fi
else
    check_warn "layers/main/ directory not found (will be created on setup)"
fi
echo ""

# Check 10: Lock Status
echo "10. Checking Workspace Lock..."
LOCK_FILE="$ROOT_DIR/.agent/scratchpad/workspace.lock"
if [ -f "$LOCK_FILE" ]; then
    check_warn "Workspace is LOCKED:"
    cat "$LOCK_FILE"
    echo "    Run: ./.agent/scripts/unlock.sh to unlock"
else
    check_pass "Workspace is unlocked"
fi
echo ""

# Summary
echo "========================================"
echo "Health Check Summary"
echo "========================================"
if [ $FAILED_CHECKS -eq 0 ]; then
    check_pass "All critical checks passed!"
    echo ""
    echo "Your workspace is healthy and ready to use."
    exit 0
else
    check_fail "$FAILED_CHECKS critical check(s) failed"
    echo ""
    echo "Please address the failed checks above before proceeding."
    exit 1
fi
