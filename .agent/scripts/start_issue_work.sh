#!/bin/bash
# Start work on a GitHub issue with visibility workflow
# Usage: start_issue_work.sh <issue_number> [agent_name] [--worktree layer|workspace]
#
# Options:
#   --worktree layer      Create a layer worktree (for ROS package development)
#   --worktree workspace  Create a workspace worktree (for infrastructure work)
#   (no --worktree)       Create a feature branch in the main workspace

set -e

# Parse arguments
ISSUE_NUMBER=""
AGENT_NAME="AI Agent"
WORKTREE_TYPE=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --worktree)
            WORKTREE_TYPE="$2"
            shift 2
            ;;
        *)
            if [ -z "$ISSUE_NUMBER" ]; then
                ISSUE_NUMBER="$1"
            else
                AGENT_NAME="$1"
            fi
            shift
            ;;
    esac
done

if [ -z "$ISSUE_NUMBER" ]; then
    echo "Usage: $0 <issue_number> [agent_name] [--worktree layer|workspace]"
    echo ""
    echo "Examples:"
    echo "  $0 42 'Copilot CLI Agent'              # Create feature branch"
    echo "  $0 42 'Copilot CLI Agent' --worktree layer    # Create layer worktree"
    echo "  $0 42 'Copilot CLI Agent' --worktree workspace # Create workspace worktree"
    exit 1
fi

# Get repository root
REPO_ROOT="$(git rev-parse --show-toplevel)"
cd "$REPO_ROOT"

# Ensure we're on default branch and clean
DEFAULT_BRANCH=$(git symbolic-ref refs/remotes/origin/HEAD | sed 's@^refs/remotes/origin/@@')
CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)

if [ "$CURRENT_BRANCH" != "$DEFAULT_BRANCH" ]; then
    echo "⚠️  Not on default branch '$DEFAULT_BRANCH'. Switching..."
    git checkout "$DEFAULT_BRANCH"
fi

# Check for uncommitted changes
if ! git diff-index --quiet HEAD --; then
    echo "❌ Uncommitted changes detected. Please commit or stash before starting new work."
    exit 1
fi

# Pull latest changes
echo "🔄 Pulling latest changes..."
git pull

# Fetch issue details from GitHub
echo "📥 Fetching issue #$ISSUE_NUMBER from GitHub..."

# Check for required dependencies
if ! command -v gh &> /dev/null; then
    echo "❌ Error: 'gh' (GitHub CLI) is not installed or not in PATH"
    echo "   Install from: https://cli.github.com/"
    exit 1
fi

if ! command -v jq &> /dev/null; then
    echo "❌ Error: 'jq' is not installed or not in PATH"
    echo "   Install with: sudo apt-get install jq (or brew install jq)"
    exit 1
fi

# Check GitHub authentication
if ! gh auth status &> /dev/null; then
    echo "❌ Error: GitHub CLI is not authenticated"
    echo "   Run: gh auth login"
    exit 1
fi

ISSUE_JSON=$(gh issue view "$ISSUE_NUMBER" --json title,body,number)
ISSUE_TITLE=$(echo "$ISSUE_JSON" | jq -r '.title')
ISSUE_BODY=$(echo "$ISSUE_JSON" | jq -r '.body // ""')

# Create slug from title (lowercase, replace spaces/special chars with hyphens)
SLUG=$(echo "$ISSUE_TITLE" | tr '[:upper:]' '[:lower:]' | sed 's/[^a-z0-9]/-/g' | sed 's/--*/-/g' | sed 's/^-//' | sed 's/-$//' | cut -c1-50)

BRANCH_NAME="feature/ISSUE-${ISSUE_NUMBER}-${SLUG}"

# Handle worktree vs branch creation
if [ -n "$WORKTREE_TYPE" ]; then
    # Create worktree using worktree_create.sh
    echo "🌲 Creating $WORKTREE_TYPE worktree for issue #$ISSUE_NUMBER..."
    
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    "$SCRIPT_DIR/worktree_create.sh" --issue "$ISSUE_NUMBER" --type "$WORKTREE_TYPE" --branch "$BRANCH_NAME"
    
    if [ "$WORKTREE_TYPE" = "layer" ]; then
        WORKTREE_PATH="$REPO_ROOT/layers/worktrees/issue-$ISSUE_NUMBER"
    else
        WORKTREE_PATH="$REPO_ROOT/.workspace-worktrees/issue-$ISSUE_NUMBER"
    fi
    
    echo ""
    echo "✅ Worktree created! Next steps:"
    echo "   1. Enter the worktree: source .agent/scripts/worktree_enter.sh $ISSUE_NUMBER"
    echo "   2. Work in isolation without affecting the main workspace"
    echo "   3. When done, commit and push from within the worktree"
    echo "   4. Create draft PR: gh pr create --draft --title 'feat: <description>' --body 'Closes #$ISSUE_NUMBER'"
    echo ""
    echo "💡 To list all worktrees: .agent/scripts/worktree_list.sh"
else
    # Traditional branch workflow
    echo "🌿 Creating branch: $BRANCH_NAME"
    git checkout -b "$BRANCH_NAME"
    
    # Create work plan from template
    PLAN_FILE=".agent/work-plans/PLAN_ISSUE-${ISSUE_NUMBER}.md"
    START_DATE=$(date +%Y-%m-%d)
    
    echo "📝 Generating work plan: $PLAN_FILE"
    
    # Copy template and replace placeholders
    cp .agent/templates/ISSUE_PLAN.md "$PLAN_FILE"
    sed -i "s/{ISSUE_NUMBER}/$ISSUE_NUMBER/g" "$PLAN_FILE"
    sed -i "s/{ISSUE_TITLE}/$ISSUE_TITLE/g" "$PLAN_FILE"
    sed -i "s/{AGENT_NAME}/$AGENT_NAME/g" "$PLAN_FILE"
    sed -i "s/{START_DATE}/$START_DATE/g" "$PLAN_FILE"
    
    echo ""
    echo "✅ Plan created! Next steps:"
    echo "   1. Edit $PLAN_FILE with your approach and tasks"
    echo "   2. Commit the plan: git add $PLAN_FILE && git commit -m 'docs: Add work plan for Issue #$ISSUE_NUMBER'"
    echo "   3. Push the branch: git push -u origin $BRANCH_NAME"
    echo "   4. Create draft PR: gh pr create --draft --title 'feat: <description>' --body \$'See .agent/work-plans/PLAN_ISSUE-${ISSUE_NUMBER}.md\\n\\nCloses #${ISSUE_NUMBER}\\n\\n---\\n**🤖 Authored-By**: \\`$AGENT_NAME\\`'"
    echo ""
    echo "💡 Or use the combined helper: .agent/scripts/update_issue_plan.sh $ISSUE_NUMBER"
fi
