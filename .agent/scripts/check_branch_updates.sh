#!/bin/bash
# Script: .agent/scripts/check_branch_updates.sh
# Purpose: Check if the default branch has new commits and provide merge/rebase recommendations
# Usage: check_branch_updates.sh [--strict]
#
# Exit codes:
#   0 - No updates needed or on default branch
#   1 - Script should never exit 1 in informational mode (reserved for strict mode)
#   2 - Error occurred

set -e

# Colors
RED='\033[0;31m'
YELLOW='\033[1;33m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Parse arguments
STRICT_MODE=false
if [[ "$1" == "--strict" ]]; then
    STRICT_MODE=true
fi

# Get current branch
CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD) || {
    echo -e "${RED}✗ Error:${NC} Could not determine current Git branch. Are you in a Git repository with a checked-out branch?"
    exit 2
}

# Detect default branch
REMOTE_HEAD=$(git symbolic-ref refs/remotes/origin/HEAD 2>/dev/null || echo "")
if [ -n "$REMOTE_HEAD" ]; then
    DEFAULT_BRANCH=${REMOTE_HEAD#refs/remotes/origin/}
else
    # Fallback: query remote (slower)
    DEFAULT_BRANCH=$(git remote show origin 2>/dev/null | grep "HEAD branch" | cut -d: -f2 | xargs || echo "main")
fi

# If we're on the default branch, no need to check
if [ "$CURRENT_BRANCH" == "$DEFAULT_BRANCH" ]; then
    echo -e "${GREEN}✓${NC} On default branch ($DEFAULT_BRANCH). No update check needed."
    exit 0
fi

# Fetch latest from origin (quietly)
# For shallow clones, we may need to unshallow or fetch with depth
echo "🔍 Checking for updates in default branch ($DEFAULT_BRANCH)..."

# Check if this is a shallow clone
IS_SHALLOW=$(git rev-parse --is-shallow-repository 2>/dev/null || echo "false")

if [ "$IS_SHALLOW" == "true" ]; then
    # For shallow clones, fetch with depth to get enough history
    if ! git fetch origin "$DEFAULT_BRANCH" --depth=50 --quiet 2>/dev/null; then
        echo -e "${YELLOW}⚠️  Warning: Could not fetch from remote${NC}"
        exit 0  # Don't block on fetch failures
    fi
else
    if ! git fetch origin "$DEFAULT_BRANCH" --quiet 2>/dev/null; then
        echo -e "${YELLOW}⚠️  Warning: Could not fetch from remote${NC}"
        exit 0  # Don't block on fetch failures
    fi
fi

# Get commit counts
COMMITS_BEHIND=$(git rev-list --count HEAD..origin/"$DEFAULT_BRANCH" 2>/dev/null || echo "0")
COMMITS_AHEAD=$(git rev-list --count origin/"$DEFAULT_BRANCH"..HEAD 2>/dev/null || echo "0")

# Check merge base to detect divergence
# In shallow clones, this might not work, so we handle gracefully
MERGE_BASE=$(git merge-base HEAD origin/"$DEFAULT_BRANCH" 2>/dev/null || echo "")
if [ -z "$MERGE_BASE" ]; then
    # Merge base not found - likely shallow clone or unrelated histories
    # We can still provide useful info based on commit counts
    if [ "$COMMITS_BEHIND" -eq 0 ] && [ "$COMMITS_AHEAD" -eq 0 ]; then
        echo -e "${GREEN}✓${NC} Branch appears to be in sync with $DEFAULT_BRANCH"
        exit 0
    fi
    # Continue with limited information
    echo -e "${YELLOW}⚠️  Note: Limited history available (shallow clone)${NC}"
fi

# Display status
if [ "$COMMITS_BEHIND" -eq 0 ]; then
    echo -e "${GREEN}✓${NC} Feature branch is up-to-date with $DEFAULT_BRANCH"
    exit 0
fi

# Branch is behind
echo ""
echo -e "${YELLOW}⚠️  Default branch has new commits!${NC}"
echo ""
echo -e "  Current branch:  ${BLUE}$CURRENT_BRANCH${NC}"
echo -e "  Default branch:  ${BLUE}$DEFAULT_BRANCH${NC}"
echo -e "  Commits behind:  ${YELLOW}$COMMITS_BEHIND${NC}"
echo -e "  Commits ahead:   ${GREEN}$COMMITS_AHEAD${NC}"
echo ""

# Check if branches have diverged
DIVERGED=false
if [ "$COMMITS_AHEAD" -gt 0 ] && [ "$COMMITS_BEHIND" -gt 0 ]; then
    DIVERGED=true
    echo -e "${YELLOW}📊 Branches have diverged${NC}"
    echo ""
fi

# Provide recommendations
echo -e "${BLUE}📋 Recommendations:${NC}"
echo ""

if [ "$DIVERGED" == "true" ]; then
    echo "  Your branch has diverged from $DEFAULT_BRANCH."
    echo "  You have two options:"
    echo ""
    echo "  1️⃣  MERGE (Recommended for collaborative work):"
    echo "     ${GREEN}git merge origin/$DEFAULT_BRANCH${NC}"
    echo "     • Preserves complete history"
    echo "     • Creates a merge commit"
    echo "     • Safe and reversible"
    echo ""
    echo "  2️⃣  REBASE (For cleaner history):"
    echo "     ${YELLOW}git rebase origin/$DEFAULT_BRANCH${NC}"
    echo "     • Replays your commits on top of latest $DEFAULT_BRANCH"
    echo "     • Creates linear history"
    echo "     • May require force-push if already pushed"
    echo "     • ⚠️  Use with caution on shared branches"
else
    # Only behind, not ahead - fast-forward possible
    echo "  Your branch is behind but has no unique commits."
    echo ""
    echo "  Fast-forward update:"
    echo "     ${GREEN}git merge origin/$DEFAULT_BRANCH${NC}"
    echo "     • Will fast-forward (no merge commit needed)"
    echo "     • Safe and automatic"
fi

echo ""
echo -e "${BLUE}💡 After updating:${NC}"
echo "   • Re-run your tests to ensure compatibility"
echo "   • Review any merge conflicts carefully"
echo "   • Consider rebasing only if you haven't pushed yet"
echo ""

# Show recent commits in default branch
echo -e "${BLUE}📝 Recent commits in $DEFAULT_BRANCH:${NC}"
git log --oneline --max-count=5 origin/"$DEFAULT_BRANCH" --not HEAD 2>/dev/null || echo "  (Could not retrieve commits)"
echo ""

# Exit with appropriate code
if [ "$STRICT_MODE" == "true" ]; then
    echo -e "${RED}❌ Strict mode: Blocking due to outdated branch${NC}"
    echo "   Run one of the recommended commands above and try again."
    exit 1
fi

echo -e "${YELLOW}ℹ️  Informational: You may want to update your branch${NC}"
echo ""
# In informational mode, we don't block the commit
exit 0
