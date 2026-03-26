# Plan: Sync workspace repos to a secondary git server

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/422

## Context

The workspace has 37 repos across 7 layers, all with `origin` pointing at GitHub.
Field robots pull from a private Forgejo instance. We manually proved the workflow:
add a named remote per repo, `git push <remote> --all`. Now we need a script to
automate this as a single command with parameterized remote name and URL prefix.

The existing `sync_repos.py` (pull from origin) and `workspace.py` library
(`get_overlay_repos`) already solve the repo-discovery problem. The new script
should reuse `workspace.py` for repo enumeration and follow the same patterns.

## Approach

1. **Create `push_repos.py`** in `.agent/scripts/` — a new script (not extending
   `sync_repos.py`, which is pull-oriented). It will:
   - Accept required args: `--remote-name` (e.g., `gitcloud`) and `--url-prefix`
     (e.g., `git@gitcloud:field/`)
   - Accept optional args: `--manifest` to filter by `.repos` file name(s)
     (e.g., `--manifest core,platforms`), `--include-underlay` flag, `--dry-run`
   - Always include the workspace repo itself (like `sync_repos.py` does)
   - For each repo: add the named remote if it doesn't exist, then `git push <remote> --all`
   - Report success/failure per repo with a summary at the end

2. **Add a Makefile target** — `make push-repos` that invokes the script.
   Require `REMOTE_NAME` and `URL_PREFIX` as make variables so the command
   is self-documenting: `make push-repos REMOTE_NAME=gitcloud URL_PREFIX=git@gitcloud:field/`

3. **Document usage** in the script's docstring and Makefile help target.

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/push_repos.py` | New script — remote management and push logic |
| `Makefile` | Add `push-repos` phony target |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Workspace vs. project separation | Script is generic infrastructure — no project-specific content. Remote name and URL are parameters, not hardcoded. |
| Only what's needed | Single script + Makefile target. No config files, no new dependencies. |
| Enforcement over documentation | Makefile target makes the command discoverable; required args prevent misconfiguration. |
| A change includes its consequences | Makefile help text updated; script docstring serves as docs. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0003 (project-agnostic) | Yes | Remote name and URL prefix are parameters, not hardcoded to any server. The script works for any secondary git server. |
| ADR-0007 (retain Make) | Yes | Adding a Makefile target follows the established pattern. |
| ADR-0009 (Python packages) | No | Only uses stdlib + PyYAML (already a workspace dependency via workspace.py). |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Add Makefile target | `make help` output | Yes — .PHONY + help text |
| Add new script | Script reference in AGENTS.md | Yes — add to table |

## Open Questions

- Should `--all` (push all branches) be default, or should we default to pushing
  only the manifest-declared branch (e.g., `jazzy`) and require `--all-branches`
  to push everything? Pushing all branches is what we did manually and matches
  a full-mirror use case, but for field robots only the default branch may matter.
- Should we also push tags? (`git push <remote> --tags`)

## Estimated Scope

Single PR.
