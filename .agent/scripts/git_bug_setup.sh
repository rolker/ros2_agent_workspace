#!/bin/bash
# Configure git-bug identity for the workspace repo and every overlay project
# repo, and (where applicable) a GitHub bridge.
#
# Idempotent: safe to re-run. Failures warn but don't exit non-zero
# (git-bug is optional infrastructure).
#
# Per repo:
#   1. Create/adopt identity from git config user.name/user.email
#      (no GitHub credentials needed)
#   2. Configure GitHub bridge using `gh auth token`
#      — skipped when origin is not github.com or `gh` is unavailable
#   3. Run initial sync (`git bug pull`) — works over plain git transport
#      against any origin once an identity exists

if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
    echo "Error: This script should be executed, not sourced."
    echo "  Run: ${BASH_SOURCE[0]} $*"
    return 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

if [ -n "${CI:-}" ]; then
    echo "CI environment — skipping git-bug setup."
    exit 0
fi

if ! command -v git-bug &>/dev/null; then
    echo "git-bug not installed — skipping setup."
    exit 0
fi

# ---- Per-repo setup ---------------------------------------------------------
setup_repo_gitbug() {
    local repo_dir="$1"
    local repo_label="$2"

    # Must be a git working tree (regular .git dir or worktree pointer file)
    if [ ! -e "$repo_dir/.git" ]; then
        return 0
    fi

    echo ""
    echo "── $repo_label  ($repo_dir)"

    # 1. Identity ------------------------------------------------------------
    local existing_user
    existing_user=$(cd "$repo_dir" && git bug user 2>/dev/null)
    if [ -z "$existing_user" ]; then
        local git_name git_email
        git_name=$(cd "$repo_dir" && git config user.name 2>/dev/null || echo "")
        git_email=$(cd "$repo_dir" && git config user.email 2>/dev/null || echo "")
        if [ -z "$git_name" ] || [ -z "$git_email" ]; then
            echo "⚠️  No git identity configured here — skipping."
            return 0
        fi
        echo "Creating git-bug identity: $git_name <$git_email>"
        if ! (cd "$repo_dir" && git bug user new \
                --name "$git_name" --email "$git_email" --non-interactive) 2>/dev/null; then
            echo "⚠️  Could not create git-bug identity — skipping."
            return 0
        fi
    else
        echo "git-bug identity already exists."
    fi

    # Adopt the first identity if none is adopted yet
    local adopted
    adopted=$(cd "$repo_dir" && git config git-bug.identity 2>/dev/null || echo "")
    if [ -z "$adopted" ]; then
        local first_id
        first_id=$(cd "$repo_dir" && git bug user 2>/dev/null | head -1 | awk '{print $1}')
        if [ -n "$first_id" ]; then
            (cd "$repo_dir" && git bug user adopt "$first_id") 2>/dev/null || true
        fi
    fi

    # 2. GitHub bridge (GitHub origin + gh CLI required) ---------------------
    local remote_url
    remote_url=$(cd "$repo_dir" && git remote get-url origin 2>/dev/null || echo "")
    if [ -z "$remote_url" ]; then
        echo "ℹ️  No origin remote — skipping bridge + pull."
        return 0
    fi

    if [[ "$remote_url" == *"github.com"* ]]; then
        local bridge_count
        bridge_count=$(cd "$repo_dir" && git bug bridge 2>/dev/null | grep -c "github" || true)
        if [ "$bridge_count" -eq 0 ]; then
            if ! command -v gh &>/dev/null; then
                echo "⚠️  gh CLI not available — skipping GitHub bridge."
            else
                local gh_token
                gh_token=$(gh auth token 2>/dev/null || echo "")
                if [ -z "$gh_token" ]; then
                    echo "⚠️  No GitHub token — skipping bridge."
                else
                    local repo_slug owner repo
                    repo_slug=$(echo "$remote_url" | sed -E 's#.*github\.com[:/]##' | sed 's/\.git$//')
                    owner="${repo_slug%%/*}"
                    repo="${repo_slug##*/}"
                    if [ -z "$owner" ] || [ -z "$repo" ]; then
                        echo "⚠️  Could not parse owner/repo from $remote_url — skipping bridge."
                    else
                        echo "Configuring GitHub bridge for $owner/$repo..."
                        if (cd "$repo_dir" && git bug bridge new \
                                --name github --target github \
                                --owner "$owner" --project "$repo" \
                                --token "$gh_token" --non-interactive) 2>/dev/null; then
                            echo "✅ GitHub bridge configured."
                        else
                            echo "⚠️  Failed to configure GitHub bridge."
                        fi
                    fi
                fi
            fi
        else
            echo "GitHub bridge already configured."
        fi
    else
        echo "ℹ️  Non-GitHub origin — skipping bridge step."
    fi

    # 3. Initial sync --------------------------------------------------------
    echo "Pulling git-bug data from origin..."
    if (cd "$repo_dir" && git bug pull) >/dev/null 2>&1; then
        echo "✅ git-bug sync complete."
    else
        echo "⚠️  git bug pull failed — issues may not be up to date."
    fi
}

# ---- Drive: workspace + each overlay project repo ---------------------------
echo "Configuring git-bug for workspace + overlay repos..."

setup_repo_gitbug "$ROOT_DIR" "ros2_agent_workspace"

shopt -s nullglob
for repo_dir in "$ROOT_DIR"/layers/main/*_ws/src/*/; do
    repo_dir="${repo_dir%/}"
    setup_repo_gitbug "$repo_dir" "$(basename "$repo_dir")"
done
shopt -u nullglob

# ---- Smoke test on workspace ------------------------------------------------
echo ""
if (cd "$ROOT_DIR" && git bug bug --format json 2>/dev/null | head -c 1 | grep -q '\['); then
    _COUNT=$(cd "$ROOT_DIR" && git bug bug "status:open" 2>/dev/null | grep -c . || echo "0")
    echo "✅ git-bug smoke test passed in workspace ($_COUNT open issues cached)"
else
    echo "⚠️  git-bug smoke test failed — 'git bug bug --format json' returned unexpected output"
    echo "   Commands may use wrong syntax for this git-bug version ($(git-bug version 2>/dev/null | head -1))"
fi
