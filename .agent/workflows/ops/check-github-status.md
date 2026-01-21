---
description: Check for open Pull Requests and Issues in overlay repositories using GitHub MCP
---

1. Get List of Repositories
```bash
python3 scripts/list_overlay_repos.py
```

2. Fetch GitHub Data
// turbo-all
Group repositories into batches (e.g., 5-10 repositories per batch) to avoid rate limits.
For each batch:
   - Construct a query string: `repo:owner1/name1 OR repo:owner2/name2 ... is:pr is:open`
   - Call `github-mcp-server` tool `search_pull_requests` with the combined query.
   - Construct a query string: `repo:owner1/name1 OR repo:owner2/name2 ... is:issue is:open`
   - Call `github-mcp-server` tool `search_issues` with the combined query.

3. Report Summary
Synthesize a report listing:
   - **Open Pull Requests**: Repo, Title, Author, Number, Link.
   - **Open Issues**: Repo, Title, Number, Link.
   - Highlight any items assigned to `rolker` or tagged `help wanted`.
