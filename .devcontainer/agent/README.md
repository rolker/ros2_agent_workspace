# Sandboxed Agent DevContainer

Run Claude Code in YOLO mode inside a Docker container, using container filesystem
isolation as the security boundary. No GitHub credentials enter the container — commits
happen inside, pushes and PR creation happen from the host via the push gateway.

## Quick Start

```bash
# 1. Create worktree on the host (before container launch)
.agent/scripts/worktree_create.sh --issue 42 --type workspace

# 2. Build the agent image (first time only, or after Dockerfile changes)
make agent-build

# 3. Launch the container
make agent-run ISSUE=42
#   → Container starts, entrypoint configures env
#   → Claude Code launches in YOLO mode
#   → You chat with Claude normally: give instructions, answer questions
#   → Claude edits/builds/tests/commits freely (no permission prompts)
#   → When done, Claude runs push_request.sh to signal readiness
#   → You exit Claude (Ctrl+C or /exit) → container exits

# 4. Launcher detects pending push request and prompts:
#   "Push feature/issue-42 and create PR? [y/n/diff]"
#   → Confirm → git push + gh pr create happens on host
```

## Building the Image

```bash
# Via Makefile
make agent-build

# Or directly
.agent/scripts/docker_run_agent.sh --issue <N> --build
```

The image is based on `ros:jazzy-perception` and includes:
- ROS 2 Jazzy dev tools, rosdep, vcstool
- Node.js 22.x + Claude Code CLI
- Git (for local commits only — no SSH keys or `gh` auth)

The build passes your host UID/GID to match file ownership.

## Running

```bash
# Standard: launches Claude Code in YOLO mode
make agent-run ISSUE=42

# Debug: drop into bash inside the container
.agent/scripts/docker_run_agent.sh --issue 42 --shell

# Build + run in one step
.agent/scripts/docker_run_agent.sh --issue 42 --build
```

### Prerequisites

- Docker installed and running
- `ANTHROPIC_API_KEY` environment variable set
- Worktree created on host before launch

## Mount Strategy

The container mounts the workspace at the **same absolute path** as the host. This is
required because git worktree `.git` files contain absolute paths to the parent repo's
object store — mounting at a different path would break git operations.

```
Host Path                           Container Mount     Access
────────────────────────────────    ──────────────────  ──────
<workspace>/                        same path           rw (base)
<workspace>/.agent/                 same path           ro (overlay)
<workspace>/.agent/scratchpad/      same path           rw (override)
<workspace>/layers/main/*_ws/build  anonymous volume    rw (isolated)
<workspace>/layers/main/*_ws/install anonymous volume   rw (isolated)
<workspace>/layers/main/*_ws/log   anonymous volume     rw (isolated)
<workspace>/layers/worktrees/       same path           rw
<workspace>/.workspace-worktrees/   same path           rw
```

**Mount ordering matters**: Docker processes mounts in order. Later mounts overlay
earlier ones at the same path. So `.agent/` (ro) overlays the base (rw), and
`.agent/scratchpad/` (rw) overrides the `.agent/` (ro) overlay.

## Security Model

The container has **zero network authentication**:

- No SSH keys (`~/.ssh/` not mounted)
- No GitHub CLI auth (`~/.config/gh/` not mounted)
- No `GITHUB_TOKEN` environment variable
- `git commit` works (local operation)
- `git push` fails (no credentials) — by design

All pushes and PR creation happen on the host via the push gateway, where the user
has full visibility and control.

The `.agent/` directory is mounted read-only to prevent the agent from modifying
workspace infrastructure scripts. The exception is `.agent/scratchpad/`, which is
read-write for push request signal files and temporary work.

## Push Gateway Workflow

When the agent finishes work inside the container:

1. **Agent** runs `push_request.sh --title "PR title"` to write a signal file
2. **User** exits Claude (`/exit` or Ctrl+C) → container exits
3. **Launcher** detects the pending push request and runs the push gateway
4. **User** reviews: `[y]es push` / `[d]iff` to review / `[s]kip` / `[c]ancel`
5. On confirmation: `git push` + `gh pr create` runs on the host

Signal files are stored in `.agent/scratchpad/push-requests/<issue>.json`.

### Manual push gateway

```bash
# Process all pending requests
.agent/scripts/push_gateway.sh

# Process specific issue
.agent/scripts/push_gateway.sh --issue 42
```

## Troubleshooting

### UID mismatch / permission denied

If files inside the container are owned by a different user:

```bash
# Rebuild with your UID
make agent-build
# Or:
docker build --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g) \
    -t ros2-agent-workspace-agent:latest .devcontainer/agent/
```

### Volume ownership issues

The entrypoint automatically fixes ownership of anonymous volumes (build/install/log).
If you see permission errors during `colcon build`, check that the entrypoint ran:

```bash
# Debug: run with shell to inspect
.agent/scripts/docker_run_agent.sh --issue <N> --shell
ls -la layers/main/core_ws/build/
```

### Git worktree path errors

Git worktree `.git` files contain absolute paths. The container mounts the workspace
at the same absolute path as the host to avoid path mismatches. If you see errors like
"not a git repository", verify the mount:

```bash
# Inside container
cat .git  # Should show gitdir with a valid absolute path
ls -la $(cat .git | awk '{print $2}')  # Should be accessible
```

### rosdep failures

Some rosdep dependencies may not be available in the container. The entrypoint runs
rosdep install with best-effort (`-y` flag) and continues on failures. If a specific
package fails to build due to missing dependencies, install them manually:

```bash
# Inside container (shell mode)
sudo apt-get update && sudo apt-get install -y <package>
```

### Container won't start

Check that:
1. Docker is running: `docker info`
2. Image exists: `docker images | grep ros2-agent`
3. API key is set: `echo $ANTHROPIC_API_KEY | head -c 10`
4. Worktree exists: `.agent/scripts/worktree_list.sh`
