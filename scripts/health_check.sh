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
# - Configuration file validity

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"

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
    check_fail "ROS2 Jazzy not found. Run: ./scripts/bootstrap.sh"
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

# Check 3: Python Dependencies
echo "3. Checking Python Dependencies..."
if python3 -c "import yaml" 2>/dev/null; then
    check_pass "PyYAML installed"
else
    check_warn "PyYAML not found. Install: pip install -r requirements.txt"
fi
echo ""

# Check 4: Workspace Structure
echo "4. Checking Workspace Structure..."

if [ -d "$ROOT_DIR/configs" ]; then
    REPOS_COUNT=$(ls -1 "$ROOT_DIR/configs"/*.repos 2>/dev/null | wc -l)
    check_pass "configs/ directory exists with $REPOS_COUNT .repos files"
else
    check_fail "configs/ directory not found"
    ((FAILED_CHECKS++))
fi

if [ -d "$ROOT_DIR/scripts" ]; then
    SCRIPT_COUNT=$(ls -1 "$ROOT_DIR/scripts"/*.sh 2>/dev/null | wc -l)
    check_pass "scripts/ directory exists with $SCRIPT_COUNT shell scripts"
else
    check_fail "scripts/ directory not found"
    ((FAILED_CHECKS++))
fi

if [ -d "$ROOT_DIR/.agent" ]; then
    check_pass ".agent/ directory exists"
else
    check_warn ".agent/ directory not found"
fi

echo ""

# Check 5: Validate Configuration Files
echo "5. Validating Configuration Files..."
if [ -f "$SCRIPT_DIR/validate_repos.py" ]; then
    if python3 "$SCRIPT_DIR/validate_repos.py" &>/dev/null; then
        check_pass "All .repos files are valid"
    else
        check_fail "Some .repos files have errors. Run: python3 scripts/validate_repos.py"
        ((FAILED_CHECKS++))
    fi
else
    check_warn "validate_repos.py not found"
fi
echo ""

# Check 6: Git Status
echo "6. Checking Git Repository..."
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

# Check 7: Workspace Layers
echo "7. Checking Workspace Layers..."
if [ -d "$ROOT_DIR/workspaces" ]; then
    WORKSPACE_COUNT=$(ls -d "$ROOT_DIR/workspaces"/*_ws 2>/dev/null | wc -l)
    if [ "$WORKSPACE_COUNT" -gt 0 ]; then
        check_pass "Found $WORKSPACE_COUNT workspace layer(s)"
        
        for ws_dir in "$ROOT_DIR/workspaces"/*_ws; do
            if [ -d "$ws_dir" ]; then
                ws_name=$(basename "$ws_dir")
                if [ -d "$ws_dir/src" ]; then
                    repo_count=$(find "$ws_dir/src" -mindepth 1 -maxdepth 1 -type d | wc -l)
                    echo "  - $ws_name: $repo_count repositories"
                    
                    if [ -f "$ws_dir/install/setup.bash" ]; then
                        echo "    ✓ Built"
                    else
                        echo "    ⚠ Not built"
                    fi
                fi
            fi
        done
    else
        check_warn "No workspace layers set up. Run: ./scripts/setup.sh <layer>"
    fi
else
    check_warn "workspaces/ directory not found (will be created on setup)"
fi
echo ""

# Check 8: Lock Status
echo "8. Checking Workspace Lock..."
LOCK_FILE="$ROOT_DIR/ai_workspace/workspace.lock"
if [ -f "$LOCK_FILE" ]; then
    check_warn "Workspace is LOCKED:"
    cat "$LOCK_FILE"
    echo "    Run: ./scripts/unlock.sh to unlock"
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
