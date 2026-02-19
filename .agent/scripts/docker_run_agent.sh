#!/bin/bash
# .agent/scripts/docker_run_agent.sh
# Host-side launcher for the sandboxed agent DevContainer.
#
# Orchestrates: build image → validate worktree → generate mounts → launch container
# After exit: check for pending push requests and run push gateway.
#
# Usage:
#   .agent/scripts/docker_run_agent.sh --issue <N>
#   .agent/scripts/docker_run_agent.sh --issue <N> --build
#   .agent/scripts/docker_run_agent.sh --issue <N> --shell

set -euo pipefail

# ---------- Constants ----------

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

# If running from inside a worktree, resolve to the main workspace root.
# Worktree directories (.workspace-worktrees/, layers/worktrees/) live there.
if [[ "$ROOT_DIR" == *"/.workspace-worktrees/"* ]]; then
    ROOT_DIR="${ROOT_DIR%%/.workspace-worktrees/*}"
elif [[ "$ROOT_DIR" == *"/layers/worktrees/"* ]]; then
    ROOT_DIR="${ROOT_DIR%%/layers/worktrees/*}"
fi

IMAGE_NAME="ros2-agent-workspace-agent"
IMAGE_TAG="latest"
CONTAINER_PREFIX="ros2-agent"

# ---------- Argument parsing ----------

ISSUE=""
BUILD_IMAGE=false
SHELL_MODE=false

show_usage() {
    cat <<'EOF'
Usage: docker_run_agent.sh --issue <N> [OPTIONS]

Launch a sandboxed agent container for a worktree.

Required:
  --issue <N>       Issue number (worktree must exist on host)

Options:
  --build           Build/rebuild the Docker image before launch
  --shell           Drop into bash instead of Claude Code (debugging)
  -h, --help        Show this help

Prerequisites:
  - Worktree must be created on host first:
      .agent/scripts/worktree_create.sh --issue <N> --type workspace
  - ANTHROPIC_API_KEY must be set

Workflow:
  1. Create worktree on host
  2. Run this script to launch container
  3. Interact with Claude Code inside container
  4. On exit, script checks for push requests and prompts to push
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --issue)
            if [[ $# -lt 2 ]]; then
                echo "ERROR: --issue requires a value." >&2
                show_usage >&2
                exit 1
            fi
            ISSUE="$2"; shift 2 ;;
        --build)
            BUILD_IMAGE=true; shift ;;
        --shell)
            SHELL_MODE=true; shift ;;
        -h|--help)
            show_usage; exit 0 ;;
        *)
            echo "Unknown option: $1" >&2
            show_usage >&2
            exit 1 ;;
    esac
done

if [ -z "$ISSUE" ]; then
    echo "ERROR: --issue <N> is required." >&2
    show_usage >&2
    exit 1
fi

# ---------- Validation ----------

# Check for authentication (API key or subscription credentials)
if [ -z "${ANTHROPIC_API_KEY:-}" ] && [ ! -f "$HOME/.claude/.credentials.json" ] && [ "$SHELL_MODE" = false ]; then
    echo "ERROR: No authentication found." >&2
    echo "Either:" >&2
    echo "  - Export ANTHROPIC_API_KEY for API key auth" >&2
    echo "  - Run 'claude' on the host and /login for subscription auth" >&2
    exit 1
fi

# Find worktree for this issue
WORKTREE_PATH=""
for candidate in \
    "$ROOT_DIR/.workspace-worktrees/issue-workspace-$ISSUE" \
    "$ROOT_DIR/layers/worktrees/issue-"*"-$ISSUE"; do
    if [ -d "$candidate" ]; then
        WORKTREE_PATH="$candidate"
        break
    fi
done

if [ -z "$WORKTREE_PATH" ]; then
    echo "ERROR: No worktree found for issue #$ISSUE." >&2
    echo "Create one first:" >&2
    echo "  .agent/scripts/worktree_create.sh --issue $ISSUE --type workspace" >&2
    exit 1
fi

WORKTREE_ID="$(basename "$WORKTREE_PATH")"
echo "Using worktree: $WORKTREE_PATH (id: $WORKTREE_ID)"

# ---------- Build image (if requested or missing) ----------

DOCKERFILE_DIR="$ROOT_DIR/.devcontainer/agent"

if ! docker image inspect "$IMAGE_NAME:$IMAGE_TAG" >/dev/null 2>&1; then
    echo "Image $IMAGE_NAME:$IMAGE_TAG not found — building..."
    BUILD_IMAGE=true
fi

if [ "$BUILD_IMAGE" = true ]; then
    echo "Building agent image..."
    docker build \
        --build-arg USER_UID="$(id -u)" \
        --build-arg USER_GID="$(id -g)" \
        -t "$IMAGE_NAME:$IMAGE_TAG" \
        -f "$DOCKERFILE_DIR/Dockerfile" \
        "$DOCKERFILE_DIR"
    echo "Image built: $IMAGE_NAME:$IMAGE_TAG"
fi

# ---------- Generate mount arguments ----------

MOUNT_ARGS=()

# 1. Bind mount the entire workspace root at the same absolute path
#    (preserves git worktree .git file absolute paths)
MOUNT_ARGS+=(-v "$ROOT_DIR:$ROOT_DIR")

# 2. Read-only overlay for .agent/ (scripts, configs, templates)
MOUNT_ARGS+=(-v "$ROOT_DIR/.agent:$ROOT_DIR/.agent:ro")

# 3. Read-write override for .agent/scratchpad/ (push requests, temp files)
mkdir -p "$ROOT_DIR/.agent/scratchpad/push-requests"
MOUNT_ARGS+=(-v "$ROOT_DIR/.agent/scratchpad:$ROOT_DIR/.agent/scratchpad")

# 4. Anonymous volumes for build/install/log in each layer workspace
#    (container gets its own build artifacts, doesn't pollute host)
for ws_dir in "$ROOT_DIR"/layers/main/*_ws; do
    [ -d "$ws_dir" ] || continue
    for subdir in build install log; do
        MOUNT_ARGS+=(-v "$ws_dir/$subdir")
    done
done

# 5. Read-write: worktree directories (where agent makes changes)
MOUNT_ARGS+=(-v "$ROOT_DIR/layers/worktrees:$ROOT_DIR/layers/worktrees")
MOUNT_ARGS+=(-v "$ROOT_DIR/.workspace-worktrees:$ROOT_DIR/.workspace-worktrees")

# 6. Claude Code config (skips onboarding, inherits auth + preferences)
#    Mount to staging paths — the entrypoint copies into user-owned locations
#    so Claude Code can create sibling directories (plugins/, debug/, etc.)
#    ~/.claude.json          — onboarding state (hasCompletedOnboarding flag)
#    ~/.claude/settings.json — user preferences
#    ~/.claude/.credentials.json — Anthropic subscription auth (NOT GitHub creds)
CLAUDE_STATE="$HOME/.claude.json"
CLAUDE_SETTINGS="$HOME/.claude/settings.json"
CLAUDE_CREDS="$HOME/.claude/.credentials.json"
if [ -f "$CLAUDE_STATE" ]; then
    MOUNT_ARGS+=(-v "$CLAUDE_STATE:/tmp/claude-state.json:ro")
fi
if [ -f "$CLAUDE_SETTINGS" ]; then
    MOUNT_ARGS+=(-v "$CLAUDE_SETTINGS:/tmp/claude-settings.json:ro")
fi
if [ -f "$CLAUDE_CREDS" ]; then
    MOUNT_ARGS+=(-v "$CLAUDE_CREDS:/tmp/claude-credentials.json:ro")
fi

# ---------- Container environment ----------

ENV_ARGS=(
    ${ANTHROPIC_API_KEY:+-e "ANTHROPIC_API_KEY"}
    -e "CLAUDE_CODE_ENTRYPOINT=1"
    -e "ROS2_AGENT_WORKSPACE_ROOT=$ROOT_DIR"
    -e "WORKTREE_ROOT=$WORKTREE_PATH"
    -e "WORKTREE_ID=$WORKTREE_ID"
    -e "WORKTREE_ISSUE=$ISSUE"
    -e "GIT_EDITOR=true"
)

# ---------- Determine CMD ----------

CONTAINER_CMD=()
if [ "$SHELL_MODE" = true ]; then
    CONTAINER_CMD=(/bin/bash)
else
    CONTAINER_CMD=(claude --dangerously-skip-permissions)
fi

# ---------- Launch container ----------

CONTAINER_NAME="${CONTAINER_PREFIX}-${WORKTREE_ID}"

echo ""
echo "========================================="
echo "  Launching Sandboxed Agent Container"
echo "========================================="
echo "  Issue:     #$ISSUE"
echo "  Worktree:  $WORKTREE_PATH"
echo "  Container: $CONTAINER_NAME"
echo "  Mode:      $([ "$SHELL_MODE" = true ] && echo 'shell (debug)' || echo 'Claude Code (YOLO)')"
echo "========================================="
echo ""

# Allow non-zero exit (user Ctrl+C, Claude exit, etc.) — we still check push requests
EXIT_CODE=0
docker run -it --rm \
    --name "$CONTAINER_NAME" \
    --hostname "agent-$ISSUE" \
    --security-opt no-new-privileges:true \
    -w "$WORKTREE_PATH" \
    "${MOUNT_ARGS[@]}" \
    "${ENV_ARGS[@]}" \
    "$IMAGE_NAME:$IMAGE_TAG" \
    "${CONTAINER_CMD[@]}" \
    || EXIT_CODE=$?

# ---------- Post-exit: check for outbox items ----------

echo ""
HAS_PENDING=false

# Check push requests
PUSH_REQUEST="$ROOT_DIR/.agent/scratchpad/push-requests/$WORKTREE_ID.json"
if [ -f "$PUSH_REQUEST" ]; then
    STATUS=$(jq -r '.status // "pending"' "$PUSH_REQUEST" 2>/dev/null || echo "unknown")
    if [ "$STATUS" = "pending" ]; then
        HAS_PENDING=true
    fi
fi

# Check issue requests
ISSUE_DIR="$ROOT_DIR/.agent/scratchpad/issue-requests/$WORKTREE_ID"
if [ -d "$ISSUE_DIR" ]; then
    for req in "$ISSUE_DIR"/*.json; do
        [ -f "$req" ] || continue
        STATUS=$(jq -r '.status // "pending"' "$req" 2>/dev/null || echo "unknown")
        if [ "$STATUS" = "pending" ]; then
            HAS_PENDING=true
            break
        fi
    done
fi

if [ "$HAS_PENDING" = true ]; then
    if [ -x "$SCRIPT_DIR/push_gateway.sh" ]; then
        "$SCRIPT_DIR/push_gateway.sh" --worktree-id "$WORKTREE_ID"
    else
        echo "Pending outbox items found. Run manually:"
        echo "  .agent/scripts/push_gateway.sh --worktree-id $WORKTREE_ID"
    fi
else
    echo "No pending outbox items. Container exited with code $EXIT_CODE."
fi
