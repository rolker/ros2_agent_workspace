# Agent Dashboard

Local web dashboard for monitoring concurrent agent sessions.

## Quick Start

```bash
make dashboard-ui
# or: python .agent/tools/dashboard/server.py [--port 3000] [--bind 127.0.0.1]
# Open http://localhost:3000
```

## Requirements

- Python 3.10+
- tmux (for terminal capture)
- `worktree_list.sh --json` (#399)
- `gh` CLI (optional, for issue data)
- `git-bug` (optional, for local issue data)

## Architecture

The dashboard is a zero-dependency Python stdlib web server that:

1. Discovers sessions by correlating `worktree_list.sh --json` with `tmux list-panes`
2. Serves a three-panel UI: terminal, plan, and context
3. Pushes real-time status updates via Server-Sent Events (SSE)

There are two independent polling mechanisms:

- **Session poller** (backend, 10s interval): SSE endpoint runs `discover_sessions()`
  every 10 seconds to detect added/removed/status-changed sessions. Intentionally
  conservative — session changes are infrequent and `worktree_list.sh` is expensive.
- **Terminal poller** (frontend, 1.5s interval): After opening a terminal panel the
  browser polls `GET /api/terminal/:id` every 1.5 seconds to refresh the captured
  tmux output. This is independent of the SSE session poller.

See [#398](https://github.com/rolker/ros2_agent_workspace/issues/398) for the
full design document.
