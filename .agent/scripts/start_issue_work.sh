#!/bin/bash
# Start work on a GitHub issue with visibility workflow
# Usage: start_issue_work.sh <issue_number> [agent_name]

set -e

ISSUE_NUMBER="$1"
AGENT_NAME="${2:-AI Agent}"

if [ -z "$ISSUE_NUMBER" ]; then
    echo "Usage: $0 <issue_number> [agent_name]"
    echo "Example: $0 42 'Copilot CLI Agent'"
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
ISSUE_JSON=$(gh issue view "$ISSUE_NUMBER" --json title,body,number)
ISSUE_TITLE=$(echo "$ISSUE_JSON" | jq -r '.title')
ISSUE_BODY=$(echo "$ISSUE_JSON" | jq -r '.body // ""')

# Create slug from title (lowercase, replace spaces/special chars with hyphens)
SLUG=$(echo "$ISSUE_TITLE" | tr '[:upper:]' '[:lower:]' | sed 's/[^a-z0-9]/-/g' | sed 's/--*/-/g' | sed 's/^-//' | sed 's/-$//' | cut -c1-50)

# Create feature branch
BRANCH_NAME="feature/ISSUE-${ISSUE_NUMBER}-${SLUG}"
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
echo "   4. Create draft PR: gh pr create --draft --title 'feat: <description>' --body 'See .agent/work-plans/PLAN_ISSUE-${ISSUE_NUMBER}.md\n\nCloses #${ISSUE_NUMBER}\n\n---\n**🤖 Authored-By**: \`$AGENT_NAME\`'"
echo ""
echo "💡 Or use the combined helper: .agent/scripts/update_issue_plan.sh $ISSUE_NUMBER"
