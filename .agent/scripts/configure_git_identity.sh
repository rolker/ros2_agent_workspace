#!/bin/bash
# .agent/scripts/configure_git_identity.sh
# Configures git identity for AI agents across workspace and all cloned repositories
#
# Usage: ./configure_git_identity.sh "<Agent Name>" "<email>"
# Example: ./configure_git_identity.sh "Copilot CLI Agent" "roland+copilot-cli@ccom.unh.edu"


if [ $# -ne 2 ]; then
    echo "Usage: $0 \"<Agent Name>\" \"<email>\""
    echo ""
    echo "Example:"
    echo "  $0 \"Copilot CLI Agent\" \"roland+copilot-cli@ccom.unh.edu\""
    echo "  $0 \"Antigravity Agent\" \"roland+antigravity@ccom.unh.edu\""
    echo ""
    echo "This configures git identity in:"
    echo "  - The workspace repository (ros2_agent_workspace)"
    echo "  - All repositories under workspaces/*/src/*"
    exit 1
fi

AGENT_NAME="$1"
AGENT_EMAIL="$2"

echo "Configuring git identity for: $AGENT_NAME <$AGENT_EMAIL>"
echo ""

# Configure workspace repository
echo "Configuring workspace repository..."
git config user.name "$AGENT_NAME"
git config user.email "$AGENT_EMAIL"
echo "  ✓ Workspace repository configured"

# Configure all repositories in workspaces/

if [ -d "workspaces" ]; then
    REPO_COUNT=0
    # Use process substitution with find to be robust against weird filenames
    # and avoid subshell exit issues with set -e
    while IFS= read -r git_dir; do
        repo_dir="$(dirname "$git_dir")"
        echo "Configuring $repo_dir..."
        
        # Capture failure but don't crash whole script immediately if one fails
        if ! git -C "$repo_dir" config user.name "$AGENT_NAME" || \
           ! git -C "$repo_dir" config user.email "$AGENT_EMAIL"; then
            echo "  ✗ Failed to configure $repo_dir" >&2
            # Decide if we want to fail hard or continue. 
            # Continuing sees more errors.
            continue
        fi
        ((REPO_COUNT++))
    done < <(find workspaces -type d -name ".git")
    
    if [ $REPO_COUNT -gt 0 ]; then
        echo "  ✓ Configured $REPO_COUNT repositories in workspaces/"
    else
        echo "  ℹ No repositories found in workspaces/ yet"
    fi
else
    echo "  ℹ No workspaces/ directory found yet"
fi

echo ""
echo "✅ Git identity configuration complete!"
echo ""
echo "Verification:"
echo "  Workspace: $(git config user.name) <$(git config user.email)>"
