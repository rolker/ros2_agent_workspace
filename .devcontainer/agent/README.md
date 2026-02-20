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
- GitHub CLI (`gh`) for read-only access (see [Read-Only GitHub Access](#read-only-github-access))
- Git (for local commits only — no SSH keys)

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

The container has **no write-level network authentication**:

- No SSH keys (`~/.ssh/` not mounted)
- No GitHub CLI write auth (`~/.config/gh/` not mounted)
- Optional read-only `GH_TOKEN` for `gh` CLI read operations (see [Read-Only GitHub Access](#read-only-github-access))
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

## Read-Only GitHub Access

Container agents can optionally have read-only `gh` CLI access for viewing issues, PRs,
and code. This uses a fine-grained Personal Access Token (PAT) with read-only permissions,
passed as `GH_TOKEN` into the container.

### Setup

1. Create a fine-grained PAT at https://github.com/settings/personal-access-tokens/new:
   - **Token name**: `ros2-agent-readonly` (or similar)
   - **Expiration**: choose an appropriate duration
   - **Repository access**: select the repos your agents work with
   - **Permissions** (read-only):
     - Issues: Read
     - Pull requests: Read
     - Contents: Read
     - Metadata: Read (auto-selected)

2. Save the token to the config file:

```bash
mkdir -p ~/.config/ros2-agent
echo "ghp_yourTokenHere" > ~/.config/ros2-agent/gh-readonly-token
chmod 600 ~/.config/ros2-agent/gh-readonly-token
```

3. The launcher picks it up automatically on next run. Verify in the launch banner:

```
  GitHub:    read-only token
```

### Alternative: environment variable

Instead of the config file, export `AGENT_GH_TOKEN` before launching:

```bash
export AGENT_GH_TOKEN="ghp_yourTokenHere"
make agent-run ISSUE=42
```

### What agents can do with read-only access

- `gh issue view`, `gh issue list` — read issues and comments
- `gh pr view`, `gh pr list`, `gh pr diff` — read pull requests
- `gh api` — read-only API calls
- `gh search` — search code, issues, PRs

Agents still **cannot** push, create PRs, or create issues from inside the container.
Those actions go through the push gateway on the host.

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
package fails to build due to missing dependencies, report them via `issue_request.sh`
or note them for the host user. The container runs as a non-root user without sudo.

### Container won't start

Check that:
1. Docker is running: `docker info`
2. Image exists: `docker images | grep ros2-agent`
3. API key is set: `echo $ANTHROPIC_API_KEY | head -c 10`
4. Worktree exists: `.agent/scripts/worktree_list.sh`
