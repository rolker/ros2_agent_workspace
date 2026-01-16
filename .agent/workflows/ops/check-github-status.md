---
description: Check for open Pull Requests and Issues in overlay repositories using GitHub MCP
---

1. Get List of Repositories
```bash
python3 scripts/list_overlay_repos.py
```

2. Fetch GitHub Data
// turbo-all
For each repository in the JSON output from step 1:
   - Call `github-mcp-server` tool `search_pull_requests` with query: `repo:<owner>/<name> is:pr is:open`
   - Call `github-mcp-server` tool `search_issues` with query: `repo:<owner>/<name> is:issue is:open`

3. Report Summary
Synthesize a report listing:
   - **Open Pull Requests**: Repo, Title, Author, Number.
   - **Open Issues**: Repo, Title, Number.
   - Highlight any items assigned to `rolker` or tagged `help wanted`.
