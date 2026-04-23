# Field Mode Hotfix Walkthrough

Concrete walkthrough for making a hotfix on a field machine and getting the
change back to GitHub via the import skill. Reference implementation of the
field-mode carve-out documented in [`AGENTS.md § Field
Mode`](../../AGENTS.md#field-mode-origin-not-githubcom).

## When This Applies

Use this flow only when **both** of these are true:

1. The repo you're editing has a non-github origin (`field_mode.sh --describe`
   prints `field mode`).
2. You are on a field machine — the boat, an operator station, or another
   environment where the field remote (gitcloud, Forgejo, etc.) is the
   authoritative copy.

Neither the workspace repo nor github-origin project repos use this flow; they
stay on the worktree + PR workflow regardless of where the machine physically
sits.

## Scenario

You're on BizzyBoat. The `unh_echoboats_project11` repo on this machine has
origin `git@gitcloud:field/unh_echoboats_project11.git`. Operator tmux is
failing to start a pane because a launch file references a missing config
key. The fix is one line in a config file. The next pier session is in an
hour.

## Step-by-Step

### 1. Confirm field mode

```bash
cd layers/main/platforms_ws/src/unh_echoboats_project11
.agent/scripts/field_mode.sh --describe
# → field mode  (origin: git@gitcloud:field/unh_echoboats_project11.git)
```

If the output says `dev mode`, stop — you're on a github-origin repo and
this flow does not apply. Use a worktree + PR.

### 2. Check the default branch

Project repos may use `jazzy` or another branch, not `main`:

```bash
git branch --show-current
# → jazzy
```

This is the branch you'll commit to directly.

### 3. Make the fix

Edit the tracked file in place. No worktree needed:

```bash
$EDITOR config/operator_tmux.yaml
```

### 4. Commit — hooks and AI signature still required

Field mode relaxes the worktree/PR requirement. It does **not** relax any
other quality gate. Pre-commit hooks still run, the AI signature is still
required, commits stay atomic:

```bash
git add config/operator_tmux.yaml
git commit -m "$(cat <<'EOF'
fix(operator_tmux): restore missing pane_order config key

Pane startup failed because operator_tmux.yaml was missing the
pane_order list. Added a default ordering matching the previous
session's layout.

Field hotfix — landing direct on jazzy per AGENTS.md Field Mode.

Co-Authored-By: <your model> <noreply@anthropic.com>
EOF
)"
```

### 5. Push to the field remote

Push goes to `origin` (gitcloud), not github:

```bash
git push origin jazzy
```

No PR is opened — there is no GitHub on this side. The commit is live on the
field branch immediately.

### 6. Verify the fix on the boat

Restart whatever was broken and confirm behavior:

```bash
tmux kill-session -t operator 2>/dev/null
scripts/start_operator_tmux.sh
```

### 7. (Later, on the dev machine) Import back to GitHub

When a dev machine next has connectivity and time to do review work, run the
import skill from the workspace root:

```bash
/import-field-changes
```

The skill:

- Fetches from the configured field remote (gitcloud)
- Detects that `unh_echoboats_project11` is ahead of `origin` (github)
- Creates a GitHub issue describing the incoming commit(s)
- Opens a draft PR with pre-review notes (tests? topic names? idempotency?)
- Leaves review + merge to a human

The field hotfix goes through the normal review gate — just delayed until
there's bandwidth for it, rather than blocking the boat.

## Gotchas

**Config file in `.agent/project_config.yaml` is required for the import
skill.** It needs `field_remote: <name>` to know where to pull from. This
file is created during field-machine bootstrap, but verify it exists on any
dev machine that runs the import.

**Don't cherry-pick field commits manually.** The import skill handles the
branch/PR setup to avoid perturbing the main tree HEAD. Manual cherry-picks
lose the pre-review and risk violating the dev-mode worktree rule.

**Diverged repos need human merge.** If both sides have commits since they
last synced, the import skill flags the repo and lets you resolve in a
dedicated worktree. Don't force-push either direction.

**Hotfix commits should still be atomic.** Multiple unrelated fixes in one
commit make later review painful. If the boat needs five things fixed, that's
five commits.

**Field mode does NOT apply in the workspace repo.** Even if you clone the
workspace from gitcloud on a field machine, workspace infrastructure is
never hotfixed in the field. File a git-bug bug report instead
(`gh_create_issue.sh` with `GITBUG_CREATE=1`) — it will sync back to GitHub
via the git-bug bridge when the machine reconnects.

## Related

- [`AGENTS.md § Field Mode`](../../AGENTS.md#field-mode-origin-not-githubcom) — canonical rule
- [`.agent/scripts/field_mode.sh`](../scripts/field_mode.sh) — mode detection helper
- [`.claude/skills/import-field-changes/content.md`](../../.claude/skills/import-field-changes/content.md) — the import skill
- Issue [#445](https://github.com/rolker/ros2_agent_workspace/issues/445) — field-mode design and decisions
- Issue [#247](https://github.com/rolker/ros2_agent_workspace/issues/247) — dev-mode worktree hardening
