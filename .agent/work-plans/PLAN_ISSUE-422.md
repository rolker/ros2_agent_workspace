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

---

## Revised Approach (2026-03-27)

After discussion, the design has changed significantly from the original approach above.

### Key Design Decisions

1. **No config file, no URL prefix argument for routine use.** Remotes are assumed
   to already exist in each repo (added once manually or via a one-time helper).
   The scripts just take `--remote <name>` and operate on repos where that remote
   exists, skipping others with a warning.

2. **Two separate scripts** (leave `sync_repos.py` unchanged):
   - **`push_remote.py --remote <name>`** — Push to a named remote
   - **`pull_remote.py --remote <name>`** — Pull/fetch from a named remote

3. **Push default branch + tags by default.** The manifest-declared branch
   (e.g., `jazzy`) is pushed, plus all tags. `--all-branches` opt-in for full mirror.

4. **Pull = fetch + report.** Fetch from the named remote and report which repos
   have new commits. Don't merge or create branches automatically — some repos
   require PRs on GitHub, so the user decides how to integrate changes. Branch
   creation or PR automation can be added later once patterns are clear.

### Revised Script Designs

#### `push_remote.py`

```
push_remote.py --remote <name> [--all-branches] [--manifest <name,...>]
                               [--include-underlay] [--dry-run]
```

- Iterates all workspace repos (via `list_overlay_repos` + workspace root)
- For each repo: if the named remote exists, push the manifest-declared branch + tags
- If `--all-branches`: push all branches instead of just the default
- Skip repos where the remote doesn't exist (warn)
- Report per-repo success/failure with summary

#### `pull_remote.py`

```
pull_remote.py --remote <name> [--manifest <name,...>]
                               [--include-underlay] [--dry-run]
```

- Iterates all workspace repos
- For each repo: if the named remote exists, `git fetch <remote>`
- Compare the remote's default branch against the local default branch
- Report repos with new commits (ahead/behind counts)
- No automatic merge — user decides how to integrate (manual merge, PR, etc.)

### Remote Setup (One-Time)

Remotes are added manually or via a future `add_remote.py` helper:

```bash
# Manual (per repo)
cd layers/main/core_ws/src/some_repo
git remote add gitcloud git@gitcloud:field/some_repo.git

# Future helper (all repos at once — not in this PR)
# add_remote.py --remote gitcloud --url-prefix git@gitcloud:field/
```

The URL prefix helper could be a follow-up PR if the manual approach proves too
tedious for 37 repos. It would derive the repo name from the origin URL
(via `extract_github_owner_repo`) rather than the directory name.

### Makefile Targets

```makefile
make push-remote REMOTE=gitcloud              # push default branches + tags
make push-remote REMOTE=gitcloud ALL=1        # push all branches + tags
make pull-remote REMOTE=gitcloud              # fetch + report
```

### Files to Change (Revised)

| File | Change |
|------|--------|
| `.agent/scripts/push_remote.py` | New — push to named remote |
| `.agent/scripts/pull_remote.py` | New — fetch from named remote + report |
| `Makefile` | Add `push-remote` and `pull-remote` targets |
| `AGENTS.md` | Add scripts to reference table |

### Open Questions (Revised)

- Should the one-time `add_remote.py` helper be included in this PR or deferred?
  For 37 repos, manual setup is tedious. But keeping this PR focused on push/pull
  is cleaner.
- For `pull_remote.py`, should it suggest the next step per repo (e.g., "run
  `git merge gitcloud/jazzy`" or "create a PR")? Or keep output minimal?
