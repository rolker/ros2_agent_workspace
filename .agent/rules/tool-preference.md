---
trigger: always_on
---

# Tool Preference: GitHub Interactions

**Rule**: Agents MUST prefer using the `github-mcp-server` tools over the `gh` CLI whenever possible.

## Reasoning
1.  **Reliability**: The MCP server is part of the agent's defined environment and is guaranteed to be available. The `gh` CLI depends on the host environment and may be missing or unauthenticated.
2.  **Safety**: MCP tools provide reduced, structured capabilities that prevent accidental destructive actions (like deleting arbitrary repos or secrets).
3.  **Structured Data**: MCP tools return JSON, which is less prone to parsing errors than raw CLI text output.

## Guidelines
*   **Agent Actions**: When you (the agent) need to check issues, create PRs, or search code, use the `github-mcp-server`.
*   **User Scripts**: When writing scripts for the *user* to run (e.g., setup scripts, CI pipelines), use standard tools like `gh` or `git`, as the user may not have the MCP server. Always check for tool existence in `bash` scripts (e.g., `command -v gh`).
