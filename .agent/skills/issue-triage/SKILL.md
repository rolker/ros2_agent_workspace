---
name: Issue Triage
description: Scan overlay repositories for GitHub issues and triage them into the workspace roadmap.
---

# Issue Triage Skill

Use this skill when asked to "check for issues", "scan the repos", or "triage bugs".

## Scope
This skill operates on **Overlay Repositories** only (excluding the `underlay.repos`). It connects upstream GitHub issues to the local task tracking system.

**Note**: Per `.agent/WORKFORCE_PROTOCOL.md`, GitHub Issues are the **Source of Truth**. This skill helps synchronize upstream issues with local tracking if needed.

## Prerequisites
-   **GitHub MCP**: Must be configured and active.
-   **Helper Script**: `.agent/scripts/list_overlay_repos.py` must exist.

## Core Operations

### 1. Scan Issues
**Trigger**: "Scan for new issues" or "Triage issues"
**Procedure**:
1.  Run `python3 .agent/scripts/list_overlay_repos.py` to get a JSON list of `{name, url, version}`.
2.  For each repository:
    -   Parse the owner/repo from the URL.
    -   Use the GitHub MCP tool `search_issues` (or `list_issues`) to find **open** issues.
    -   *Filter*: Check if the issue is already tracked in this workspace's GitHub Issues (search for the issue URL or number).
3.  **Report**: Present a list of *new, untracked* issues to the User.

### 2. Triage (Interactive)
**Trigger**: (Continuation of Scan)
**Procedure**:
1.  Ask the User: "Which of these should be tracked in this workspace?"
2.  For selected issues:
    -   **Primary**: Create a GitHub Issue in this workspace to track the upstream issue. **MANDATORY**: Append the AI Signature (see `.agent/rules/common/ai-signature.md`) to the issue body.
    -   **Optional**: Use the **Project Management Skill** to also add to `.agent/ROADMAP.md` if local file tracking is needed for offline or quick reference.
    -   Title format: `[Upstream] <Issue Title> (Link: <url>)`

## Example Output
"I found 3 new issues in `depthai_marine`:
1.  `Crash on startup when no camera connected` (#42)
2.  `Add graceful shutdown` (#45)

Do you want to create GitHub Issues in this workspace to track these? (I will automatically append my AI Signature to any issues created)."
