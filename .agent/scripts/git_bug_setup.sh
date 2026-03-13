#!/bin/bash
# Configure git-bug identity and GitHub bridge for the workspace repo.
#
# Idempotent: safe to re-run. Failures warn but don't exit non-zero
# (git-bug is optional infrastructure).
#
# Steps:
#   1. Check git-bug is installed (exit 0 if not)
#   2. Create/adopt identity from git config
#   3. Configure GitHub bridge using gh auth token
#   4. Run initial sync (git bug pull)

if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
    echo "Error: This script should be executed, not sourced."
    echo "  Run: ${BASH_SOURCE[0]} $*"
    return 1
fi

# --- Resolve workspace root ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

# 1. Skip in CI or if git-bug is not installed
if [ -n "${CI:-}" ]; then
    echo "CI environment — skipping git-bug setup."
    exit 0
fi

if ! command -v git-bug &>/dev/null; then
    echo "git-bug not installed — skipping setup."
    exit 0
fi

echo "Configuring git-bug..."

# 2. Identity setup — create if none exists, adopt if not already adopted
cd "$ROOT_DIR" || exit 0

# Check if any identity exists
if ! git bug user ls &>/dev/null 2>&1 || [ -z "$(git bug user ls 2>/dev/null)" ]; then
    GIT_NAME=$(git config user.name 2>/dev/null || echo "")
    GIT_EMAIL=$(git config user.email 2>/dev/null || echo "")
    if [ -z "$GIT_NAME" ] || [ -z "$GIT_EMAIL" ]; then
        echo "⚠️  No git identity configured — cannot create git-bug identity."
        exit 0
    fi
    echo "Creating git-bug identity: $GIT_NAME <$GIT_EMAIL>"
    if ! git bug user new --name "$GIT_NAME" --email "$GIT_EMAIL" --non-interactive 2>/dev/null; then
        echo "⚠️  Could not create git-bug identity — skipping."
        exit 0
    fi
else
    echo "git-bug identity already exists."
fi

# Adopt identity if not already adopted
if ! git bug user adopt &>/dev/null 2>&1; then
    # List identities and adopt the first one
    FIRST_ID=$(git bug user ls 2>/dev/null | head -1 | awk '{print $1}')
    if [ -n "$FIRST_ID" ]; then
        git bug user adopt "$FIRST_ID" 2>/dev/null || true
    fi
fi

# 3. GitHub bridge setup
BRIDGE_EXISTS=$(git bug bridge list 2>/dev/null | grep -c "github" || true)
if [ "$BRIDGE_EXISTS" -eq 0 ]; then
    # Need gh CLI for auth token
    if ! command -v gh &>/dev/null; then
        echo "⚠️  gh CLI not available — cannot configure GitHub bridge."
        exit 0
    fi

    GH_TOKEN=$(gh auth token 2>/dev/null || echo "")
    if [ -z "$GH_TOKEN" ]; then
        echo "⚠️  No GitHub token available — cannot configure bridge."
        exit 0
    fi

    # Determine repo owner and name from remote
    REMOTE_URL=$(git remote get-url origin 2>/dev/null || echo "")
    if [ -z "$REMOTE_URL" ]; then
        echo "⚠️  No origin remote — cannot configure bridge."
        exit 0
    fi

    REPO_SLUG=$(echo "$REMOTE_URL" | sed -E 's#.*github\.com[:/]##' | sed 's/\.git$//')
    OWNER="${REPO_SLUG%%/*}"
    REPO="${REPO_SLUG##*/}"

    if [ -z "$OWNER" ] || [ -z "$REPO" ]; then
        echo "⚠️  Could not parse owner/repo from remote — cannot configure bridge."
        exit 0
    fi

    echo "Configuring GitHub bridge for $OWNER/$REPO..."
    if git bug bridge new \
        --name github \
        --target github \
        --owner "$OWNER" \
        --project "$REPO" \
        --token "$GH_TOKEN" \
        --non-interactive 2>/dev/null; then
        echo "✅ GitHub bridge configured."
    else
        echo "⚠️  Failed to configure GitHub bridge — continuing without it."
        exit 0
    fi
else
    echo "GitHub bridge already configured."
fi

# 4. Initial sync
echo "Pulling issues from GitHub..."
if git bug pull 2>/dev/null; then
    echo "✅ git-bug sync complete."
else
    echo "⚠️  git bug pull failed — issues may not be up to date."
fi
