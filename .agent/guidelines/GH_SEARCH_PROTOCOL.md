# GitHub Search Protocol

## Purpose
To ensure AI agents consistently find ALL relevant Pull Requests and Issues when querying GitHub.

## The Problem
Agents often default to "lazy" searches like `user:rolker` to find "my tasks". This fails because:
1.  It returns results for ALL repos owned by the user, clogging the context.
2.  It misses PRs created by *other* users in the workspace's repositories (e.g., Dependabot, other contributors).
3.  It relies on the agent's assumption of who "the user" is.

## The Protocol

### 1. Repository-First Search
**ALWAYS** scope searches to specific repositories.

❌ **Bad**: `user:rolker is:pr`
✅ **Good**: `repo:rolker/ros2_agent_workspace repo:rolker/camp is:pr`

### 2. Finding the Repository List
Before searching, you must know WHAT to search.
- **Root**: `rolker/ros2_agent_workspace` (or current root remote).
- **Overlays**: Run `python3 .agent/scripts/list_overlay_repos.py` to get the list of active repositories.

### 3. Batching
If there are many repositories (> 5), batch them using `OR`:
`repo:A OR repo:B OR repo:C is:issue is:open`

### 4. Inclusion
- **Root Repo**: Always include the workspace root repo in your checks.
- **Underlay**: Don't ignore underlay repos if they are forked/owned by the user.

## Example Workflow
1.  Read `.agent/scripts/list_overlay_repos.py` output.
2.  Add `rolker/ros2_agent_workspace` to the list.
3.  Construct query: `repo:rolker/ros2_agent_workspace OR repo:rolker/camp ...`
4.  Execute `search_pull_requests`.
