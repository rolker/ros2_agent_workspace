# Plan: Fix git-bug command syntax for v0.10.1 and add fallback warnings

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/418

## Context

git-bug v0.10.1 is installed, bridge-synced, and wired into agent scripts — but
every git-bug call silently fails because the scripts use pre-v0.10 command syntax.
In v0.10.1, `select`, `show`, `deselect`, and `new` are subcommands of `git bug bug`,
not top-level commands. Additionally, `git bug bug select` takes an internal git-bug
ID (hex prefix), not a GitHub issue number. The `2>/dev/null` error suppression makes
the failure invisible — scripts always fall back to `gh` API.

A secondary problem: the bridge sync appears stalled at issue #354 (175 of ~210 issues).
This should be investigated but may be a separate fix.

## Approach

### 1. Create a helper function for GitHub-issue-number → git-bug lookup

Rather than fixing `select`/`show` in every script independently, extract a shared
helper that maps a GitHub issue number to git-bug metadata. This avoids the
select/show/deselect dance entirely — use `git bug bug --format json` with jq filtering
instead, which is simpler and avoids statefulness.

Add to a new file `.agent/scripts/gitbug_helpers.sh` (sourced by scripts that need it):

```bash
# gitbug_lookup_issue <issue_num> <field>
# Returns title, status, or human_id for a GitHub issue number.
# Uses git-bug JSON output filtered by github-url metadata.
# Returns empty string on failure (caller handles fallback).
```

This is more robust than `select`/`show` because:
- No statefulness (no select/deselect needed)
- Direct field extraction via `--field` or JSON
- Single command, not a multi-step pipeline

### 2. Fix each affected script

**`worktree_enter.sh`** (~lines 237–244):
- Replace `git bug select` + `git bug show` + `git bug deselect` with
  `gitbug_lookup_issue "$ISSUE_NUM" title`
- Add fallback warning when git-bug fails and `gh` takes over

**`worktree_create.sh`** (~lines 565–579):
- Replace `git bug select` + `git bug show` with `gitbug_lookup_issue` for
  title and state
- Add fallback warning

**`gh_create_issue.sh`** (~lines 153–190):
- Fix `git bug new` → `git bug bug new` (the `--title`/`--message` flags are the same)
- Fix `git bug push` (this one is already correct — `push` is top-level)

**`dashboard.sh`** (~lines 502–510):
- Fix `git bug bridge list` → `git bug bridge` (check output for bridge name)
- Fix `git bug ls status:open` → `git bug bug status:open`

**`git_bug_setup.sh`** (line 66):
- Fix `git bug bridge list` → `git bug bridge`
- Fix `git bug user ls` → `git bug user` (already top-level, but verify)
- Fix `git bug user new` / `git bug user adopt` (verify these are correct)
- Add a smoke test at the end that runs `git bug bug --format json | head -c 1`
  to confirm the command actually works

### 3. Add fallback warnings

In each script where git-bug falls back to `gh`, emit a visible warning:

```bash
echo "⚠️  git-bug lookup failed for #$ISSUE_NUM, falling back to gh API" >&2
```

Use stderr so it's visible but doesn't pollute captured output. Keep it concise —
one line, not a stack trace.

### 4. Investigate and fix bridge sync gap

The bridge stopped syncing at #354. Check if a fresh `git bug pull` picks up
newer issues. If not, this may need a bridge reconfigure (delete + recreate).
Document findings but don't block the main fix on this.

### 5. Test

- Run `worktree_create.sh` and `worktree_enter.sh` with a known issue and
  verify git-bug is used (no fallback warning)
- Run with git-bug temporarily removed from PATH and verify graceful fallback
  with warning message
- Run `dashboard.sh --quick` and verify issue counts come from git-bug

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/gitbug_helpers.sh` | **New**: shared helper for GitHub-issue → git-bug lookup |
| `.agent/scripts/worktree_enter.sh` | Replace select/show/deselect with helper; add fallback warning |
| `.agent/scripts/worktree_create.sh` | Replace select/show/deselect with helper; add fallback warning |
| `.agent/scripts/gh_create_issue.sh` | Fix `git bug new` → `git bug bug new` |
| `.agent/scripts/dashboard.sh` | Fix `bridge list` → `bridge`; fix `bug ls` → `bug bug` |
| `.agent/scripts/git_bug_setup.sh` | Fix `bridge list` and `user` subcommands; add smoke test |
| `AGENTS.md` | Add `gitbug_helpers.sh` to script reference table |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Adding fallback warnings directly improves transparency — agents and users will see when git-bug isn't working |
| Enforcement over documentation | The fix is mechanical (correct commands), not a new doc-only rule |
| A change includes its consequences | AGENTS.md script table updated; smoke test catches future regressions |
| Only what's needed | One small helper file avoids duplicating the lookup logic in 4 scripts; no over-abstraction |
| Improve incrementally | Fixes the existing integration rather than redesigning it |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0010 (git-bug) | Yes | Directly implements ADR's "scripts try git-bug first, fall back to gh" — this fix makes that actually work |
| ADR-0007 (Retain Make) | No | No Makefile changes needed |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Scripts in `.agent/scripts/` | Script reference table in `AGENTS.md` | Yes |
| git-bug command patterns | Any documentation referencing git-bug commands | Yes (will check) |

## Open Questions

- **Bridge sync gap**: Why did sync stop at #354? Is this a one-time issue or
  recurring? May need a separate fix if bridge reconfiguration is required.
- **Performance**: `git bug bug --format json` loads all issues into memory.
  For 200+ issues this should be fast, but if the workspace grows to thousands,
  we may want `git bug bug --format json "github-url:*issues/NNN"` filtering
  (if supported) or a cached index.

## Estimated Scope

Single PR. All changes are in `.agent/scripts/` plus one `AGENTS.md` table entry.
