#!/bin/bash
set -e

# Script: .agent/scripts/checkout_default_branch.sh
# Purpose: Automatically detect and switch to the remote default branch.
#          Safety: Exits if uncommitted changes are detected.

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "🔍 Detecting default branch..."

# 1. Detect Remote Default Branch
# Try to get it from local git config first (fast)
REMOTE_HEAD=$(git symbolic-ref refs/remotes/origin/HEAD 2>/dev/null)

if [ -n "$REMOTE_HEAD" ]; then
    # refs/remotes/origin/main -> main
    DEFAULT_BRANCH=${REMOTE_HEAD#refs/remotes/origin/}
else
    # Try querying remote (slow)
    echo "🔎 origin/HEAD not found locally. Querying remote..."
    DEFAULT_BRANCH=$(git remote show origin | grep "HEAD branch" | cut -d: -f2 | xargs)

    if [ -z "$DEFAULT_BRANCH" ]; then
         echo -e "${YELLOW}⚠️  Could not determine origin/HEAD. Falling back to 'main'.${NC}"
         DEFAULT_BRANCH="main"
    fi
fi

echo -e "✅ Default branch detected: ${GREEN}${DEFAULT_BRANCH}${NC}"

# 2. Check for Uncommitted Changes (including untracked files)
if [ -n "$(git status --porcelain)" ]; then
    echo -e "${RED}❌ Uncommitted changes detected.${NC}"
    echo "   Please commit or stash your changes before switching branches."
    echo "   Aborting auto-checkout."
    exit 1
fi

# 3. Checkout and Pull
echo "🔄 Switching to ${DEFAULT_BRANCH}..."
if git checkout "${DEFAULT_BRANCH}"; then
    echo "📥 Pulling latest changes..."
    if git pull; then
        echo -e "${GREEN}✅ Successfully checked out and updated ${DEFAULT_BRANCH}.${NC}"
    else
        echo -e "${RED}❌ Failed to pull latest changes for ${DEFAULT_BRANCH}.${NC}"
        exit 1
    fi
else
    echo -e "${RED}❌ Failed to checkout ${DEFAULT_BRANCH}.${NC}"
    exit 1
fi
