# Sandboxed Agent DevContainer

Run Claude Code in YOLO mode inside a Docker container. Be precise about what
that contains: the container isolates the **OS/dependency state** and the
**build artifacts**, and it is configured without GitHub **write** auth — so
commits happen inside, and pushes and PR creation happen from the host via the
push gateway. It does **not** isolate the workspace files, which are
bind-mounted read-write at the same absolute path (see
[Mount Strategy](#mount-strategy)), and it is not credential-free: it inherits
the host's Claude Code authentication, and may carry an optional GitHub token
whose read-only-ness is a convention, not a checked scope (see
[Security Model](#security-model)).

## Quick Start

```bash
# 1. Create worktree on the host (before container launch)
.agent/scripts/worktree_create.sh --issue 42 --type workspace

# 2. Build the agent image (first time only; also after editing the Dockerfile
#    or the startup scripts — only the latter drift is detected for you)
make agent-build

# 3. Launch the container
make agent-run ISSUE=42
#   → Container starts, entrypoint configures env
#   → Claude Code launches in YOLO mode
#   → You chat with Claude normally: give instructions, answer questions
#   → Claude edits/builds/tests/commits freely (no permission prompts)
#   → When done, Claude runs push_request.sh to signal readiness
#   → You exit Claude (Ctrl+C or /exit) → container exits

# 4. Launcher detects pending push request and prompts:
#   "Push feature/issue-42 and create PR? [y/n/diff]"
#   → Confirm → git push + gh pr create happens on host
```

## Building the Image

```bash
# Via Makefile
make agent-build

# Or directly
.agent/scripts/docker_run_agent.sh --issue <N> --build
```

The launcher only builds automatically when the image is **missing**, so a
change to `agent-entrypoint.sh` or `fix-volume-ownership.sh` does not reach an
existing image on its own — the container keeps running the baked copies. The
build stamps those two scripts' combined hash into the image
(`org.ros2-agent.startup-scripts-sha`), and `docker_run_agent.sh` compares it at
every launch, warning when it has drifted **or when the image carries no marker
at all** — an unmarked image predates the check and most likely bakes an older
entrypoint (#604). The warning never blocks the launch; rebuild when you see it.

Only the two startup scripts are hashed. Other changes to the image — the
Dockerfile, the baked rosdep set — are **not** detected; rebuild deliberately
after those.

`make agent-build` delegates to `docker_run_agent.sh --build-only`, which is the
single build path: one digest formula over one directory, so the launcher can
never read a freshly built image as stale. Note that the launcher resolves the
**main workspace root** even when invoked from inside a worktree, so a worktree's
edits to the startup scripts are not what gets baked — merge them, or build from
a checkout whose main tree carries them.

The image is based on `ros:jazzy-perception` and includes:
- ROS 2 Jazzy dev tools, rosdep, vcstool
- Node.js 22.x + Claude Code CLI
- GitHub CLI (`gh`) for read-only access (see [Read-Only GitHub Access](#read-only-github-access))
- git-bug for local issue access (reads/comments without network, see [ADR-0010](../../docs/decisions/0010-adopt-git-bug-for-local-issue-tracking.md))
- Git (for local commits only — no SSH keys)

The build passes your host UID/GID to match file ownership.

### Dependency Baking

The build **bakes the workspace's layer system-dependencies into the image**
(#520, #522). Before `docker build`, `stage_rosdep_manifests.sh` gathers every
layer `package.xml` (recursively — multi-package repos nest manifests in
subdirs; `COLCON_IGNORE`/`AMENT_IGNORE`/`CATKIN_IGNORE` packages are skipped so
non-built packages can't poison the bake) into a staging dir
(`.rosdep-manifests/`, gitignored) under the build context; the Dockerfile
`COPY`s those manifests and runs `rosdep install` so the deps land in a
read-only image layer. The source itself stays mounted at runtime, never copied
into the image.

All layers are installed in a single rosdep pass so `--ignore-src` correctly
ignores local packages across layers (an overlay package may depend on a local
package in a lower layer). Because `rosdep install` aborts entirely if *any*
key is unresolvable, the bake first computes the genuinely-unresolvable external
keys (`rosdep check`) and passes them as `--skip-keys` — so one bad key skips
only itself rather than nuking the whole bake (#522).

This is what makes the launch-time `rosdep check` in `agent-entrypoint.sh`
actually skip: with deps already baked, each launch installs only the *delta*
(deps added to a `package.xml` since the last image build) instead of
re-running the full apt install on every ephemeral (`--rm`) container start.

**Rebuild the image when layer deps drift** — i.e. after adding a dependency to
a `package.xml`, or pulling layer repos that introduce new packages. Builds are
slower and the image larger; launches are much faster. The bake is best-effort:
a dep that can't resolve at build time is logged and falls through to the
launch-time install path rather than failing the build.

> Build through `docker_run_agent.sh --build` / `--build-only`, or
> `make agent-build`, which delegates to `--build-only`. That build block is the
> single build path (#604): it is the only caller of
> `.agent/scripts/stage_rosdep_manifests.sh`, and the only thing that stamps the
> startup-scripts staleness marker.
>
> A **manual** `docker build .devcontainer/agent/` bypasses both. It has no
> `.rosdep-manifests/`, so the `COPY` step fails; and even after staging by
> hand it passes no `--build-arg STARTUP_SCRIPTS_SHA`, leaving an image whose
> marker is empty — which the launcher warns about at every launch until it is
> rebuilt through the real path. Don't build by hand.
>
> The baked deps are a snapshot of whatever layers are checked out at build
> time, so the image is **not** project-agnostic at runtime — it's a local,
> per-workspace artifact, rebuilt when deps drift (cf. ADR-0003).

**Architecture**: the image builds for the host's architecture (no
`--platform` is set in `make agent-build`). Supported: `amd64` and
`armhf` (32-bit ARM). Unsupported: `arm64` — git-bug v0.10.1 has no
upstream arm64 binary, so the build fails explicitly at the git-bug
install step on any arm64 host (Apple Silicon, AWS Graviton, 64-bit
Raspberry Pi, etc.).

## Running

```bash
# Standard: launches Claude Code in YOLO mode
make agent-run ISSUE=42

# Debug: drop into bash inside the container
.agent/scripts/docker_run_agent.sh --issue 42 --shell

# Build + run in one step
.agent/scripts/docker_run_agent.sh --issue 42 --build
```

### Prerequisites

- Docker installed and running
- Claude Code authentication reachable by the launcher — any one of
  `CLAUDE_CODE_OAUTH_TOKEN` (recommended: `claude setup-token`, saved to
  `~/.config/ros2-agent/claude-oauth-token`), `ANTHROPIC_API_KEY` (API billing),
  or a host `~/.claude/.credentials.json` from `/login`. With none of the three
  the launcher exits with an error before starting the container
  (`docker_run_agent.sh:317-328`) — except under `--shell`, which the same guard
  exempts (`[ "$SHELL_MODE" = false ]`), so an interactive shell launches
  uncredentialed and Claude Code cannot authenticate inside it. For **headless dispatch** use the long-lived
  token: mounted `.credentials.json` OAuth tokens cannot refresh in the sandbox
  (`:333-339`).
- Worktree created on host before launch

## Mount Strategy

The container mounts the workspace at the **same absolute path** as the host. This is
required because git worktree `.git` files contain absolute paths to the parent repo's
object store — mounting at a different path would break git operations.

```
Host Path / Source                  Container Mount          Access
────────────────────────────────    ───────────────────────  ──────
<workspace>/                        same path                rw (base)
<workspace>/.agent/                 same path                ro (overlay)
<workspace>/.agent/scratchpad/      same path                rw (override)
<workspace>/layers/main/*_ws/build  anonymous volume         rw (isolated)
<workspace>/layers/main/*_ws/install anonymous volume        rw (isolated)
<workspace>/layers/main/*_ws/log   anonymous volume          rw (isolated)
<workspace>/layers/worktrees/       same path                rw
<workspace>/.workspace-worktrees/   same path                rw
~/.claude.json, settings, creds     /tmp staging (copied)    ro
named: ros2-agent-precommit-cache   /home/ros/.cache/pre-commit  rw (isolated)
named: ros2-agent-claude-plugins    /home/ros/.claude/plugins    rw (isolated)
```

The last two are **named Docker volumes**, not host binds. They amortize
the pre-commit hook-environment install and the Claude plugin-marketplace
clone across container runs without touching the host's own
`~/.cache/pre-commit` or `~/.claude/plugins` — see Security Model. Reset
either with `docker volume rm ros2-agent-precommit-cache` /
`docker volume rm ros2-agent-claude-plugins` to force a fresh rebuild.

**Mount ordering matters**: Docker processes mounts in order. Later mounts overlay
earlier ones at the same path. So `.agent/` (ro) overlays the base (rw), and
`.agent/scratchpad/` (rw) overrides the `.agent/` (ro) overlay.

## Security Model

The container is **configured** without write-level network authentication:

- No SSH keys (`~/.ssh/` not mounted)
- No GitHub CLI write auth (`~/.config/gh/` not mounted)
- Optional read-only `GH_TOKEN` for `gh` CLI read operations (see [Read-Only GitHub Access](#read-only-github-access))
- `git commit` works (local operation)
- `git push` fails (no credentials) — by design

"Configured" is the operative word: the launcher forwards `AGENT_GH_TOKEN` (or
the `~/.config/ros2-agent/gh-readonly-token` file) into the container as
`GH_TOKEN` (`docker_run_agent.sh:651-665`, `:690`) **without validating its
scopes** — the read-only-ness comes from the filename and from how you minted
the PAT, not from anything the launcher checks. Put a write-capable PAT there
and the container has write auth. This is a configuration to hold to, not a
boundary the container enforces.

Nor is the container credential-free in general. It inherits the host's Claude
Code authentication: `~/.claude/.credentials.json`, `~/.claude.json` and
`~/.claude/settings.json` are mounted (`docker_run_agent.sh:599-616`), and the
long-lived `CLAUDE_CODE_OAUTH_TOKEN` is forwarded at `:688`.

What is genuinely withheld is narrower than "GitHub write credentials", and it
holds only for the read-only configuration above. `git push` fails
unconditionally: no SSH keys, no `~/.config/gh`, no credential helper and no
`gh auth setup-git` anywhere in the launcher or the entrypoint (verified), so
there is no transport for git to authenticate over. `gh` is a separate matter —
it authenticates from `GH_TOKEN` alone, so a write-capable PAT left in
`AGENT_GH_TOKEN` makes `gh pr create` and `gh api -X POST` succeed from inside
the container. Mint that token read-only and the whole boundary holds; the
launcher will not check it for you.

Pushes and PR creation are therefore meant to happen on the host via the push
gateway, where the user has full visibility and control — `git push` cannot
happen anywhere else, and `gh` publication stays on the host as long as the
forwarded token is read-only.

The `.agent/` directory is mounted read-only to prevent the agent from modifying
workspace infrastructure scripts. The exception is `.agent/scratchpad/`, which is
read-write for push request signal files and temporary work.

**The host's `~/.cache/pre-commit` and `~/.claude/plugins` are deliberately
NOT bind-mounted.** Both hold *executable* content (hook-environment
interpreters/shims; plugin code). A read-write host bind would let a
`--dangerously-skip-permissions` agent overwrite binaries the **host**
later executes (pre-commit runs hook envs at every host-side commit) —
a sandbox-escape path that bypasses PR review. Instead, named volumes
(`ros2-agent-precommit-cache`, `ros2-agent-claude-plugins`) give the
same cross-run amortization while staying isolated from the host.

MCP servers are disabled in-container via `--strict-mcp-config`: the
claude.ai Gmail/Drive/Calendar servers (and any user-scoped servers
inherited from the mounted `~/.claude.json`) are host-only and
credential-less here, so they would only fail-fast on startup. The
workspace defines no project `.mcp.json` servers, so nothing the sandbox
needs is lost; local skills still resolve via `/skill-name`.

## Push Gateway Workflow

When the agent finishes work inside the container:

1. **Agent** runs `push_request.sh --title "PR title"` to write a signal file
2. **User** exits Claude (`/exit` or Ctrl+C) → container exits
3. **Launcher** detects the pending push request and runs the push gateway
4. **User** reviews: `[y]es push` / `[d]iff` to review / `[s]kip` / `[c]ancel`
5. On confirmation: `git push` + `gh pr create` runs on the host

Signal files are stored in `.agent/scratchpad/push-requests/<issue>.json`.

### Manual push gateway

```bash
# Process all pending requests
.agent/scripts/push_gateway.sh

# Process specific issue
.agent/scripts/push_gateway.sh --issue 42
```

## Read-Only GitHub Access

Container agents can optionally have read-only `gh` CLI access for viewing issues, PRs,
and code. This uses a fine-grained Personal Access Token (PAT) with read-only permissions,
passed as `GH_TOKEN` into the container.

### Setup

1. Create a fine-grained PAT at https://github.com/settings/personal-access-tokens/new:
   - **Token name**: `ros2-agent-readonly` (or similar)
   - **Expiration**: choose an appropriate duration
   - **Repository access**: select the repos your agents work with
   - **Permissions** (read-only):
     - Issues: Read
     - Pull requests: Read
     - Contents: Read
     - Metadata: Read (auto-selected)

2. Save the token to the config file:

```bash
mkdir -p ~/.config/ros2-agent
echo "ghp_yourTokenHere" > ~/.config/ros2-agent/gh-readonly-token
chmod 600 ~/.config/ros2-agent/gh-readonly-token
```

3. The launcher picks it up automatically on next run. Verify in the launch banner:

```
  GitHub:    read-only token
```

### Alternative: environment variable

Instead of the config file, export `AGENT_GH_TOKEN` before launching:

```bash
export AGENT_GH_TOKEN="ghp_yourTokenHere"
make agent-run ISSUE=42
```

### What agents can do with read-only access

- `gh issue view`, `gh issue list` — read issues and comments
- `gh pr view`, `gh pr list`, `gh pr diff` — read pull requests
- `gh api` — read-only API calls
- `gh search` — search code, issues, PRs

With a genuinely read-only PAT, agents **cannot** push, create PRs, or create
issues from inside the container; those actions go through the push gateway on
the host. That holds because of how you minted the token — the launcher does not
verify it (see [Security Model](#security-model)) — so mint it read-only.

## Troubleshooting

### UID mismatch / permission denied

If files inside the container are owned by a different user:

```bash
# Rebuild with your UID
make agent-build
```

`make agent-build` already passes your `USER_UID`/`USER_GID` — it delegates to
`docker_run_agent.sh --build-only`, which reads them from `id -u` / `id -g`.

Do **not** substitute a bare `docker build` here. It is not the single build
path (#604): it stages no rosdep manifests, and it passes no
`--build-arg STARTUP_SCRIPTS_SHA`, so the image is stamped with an empty
startup-scripts marker — which the launcher warns about at *every* launch, with
no way to clear it short of rebuilding through the real path.

### Volume ownership issues

The launcher declares anonymous volumes over every `build/install/log` directory
in **two** places: `layers/main/*_ws` and the dispatched worktree's own `*_ws`.
Docker initializes each from the *image* at that path — not from the host
directory bind-mounted underneath it — so every one comes up empty and
`root:root` regardless of what the host did. `fix-volume-ownership.sh`, run by
the entrypoint as root before it drops privileges, chowns both sets to the
target user. Missing the second set was [#604](https://github.com/rolker/ros2_agent_workspace/issues/604):
the agent could build in `layers/main` but not in its own worktree.

So a `Permission denied` from `colcon build` points at that chown. Check which
directory it is — a worktree path and a `layers/main` path implicate different
loops:

```bash
# Debug: run with shell to inspect
.agent/scripts/docker_run_agent.sh --issue <N> --shell
ls -ld layers/main/core_ws/build/           # loop 1
ls -ld "$WORKTREE_ROOT"/*_ws/build/         # loop 2 (the #604 gap)
```

The `--shell` session above is the dropped-privilege `ros` user (the entrypoint
hands off via `setpriv`), the image ships no `sudo`, and the container runs with
`--security-opt no-new-privileges:true` — so the chown cannot be re-run from
inside that session. Re-run it from the **host**, as root in the same container:

```bash
# Host, in a second terminal while the --shell session is still up.
# The launcher prints the name on its "Container:" line; this finds it again.
# With several agent containers up, name it explicitly instead.
c=$(docker ps --filter name=^/ros2-agent- --format '{{.Names}}' | head -1)
docker exec -u 0 "$c" \
    /usr/local/bin/fix-volume-ownership.sh \
    "$(docker exec "$c" id -u ros)" \
    "$(docker exec "$c" id -g ros)" \
    "$(docker exec "$c" printenv ROS2_AGENT_WORKSPACE_ROOT)" \
    "$(docker exec "$c" printenv WORKTREE_ROOT)"
```

Every argument is read back out of the container, because none of it exists in
the host shell: the launcher *passes* `ROS2_AGENT_WORKSPACE_ROOT` and
`WORKTREE_ROOT` into the container, it does not export them to its caller, so
expanding them host-side would hand the script two empty strings. (The same two
paths are also printed in the container's startup banner and the `Using
worktree:` line if you would rather paste them literally.)

A root-owned volume with an image that predates the fix is the likely cause —
check for the staleness warning at launch and rebuild. `bash
.agent/scripts/tests/test_entrypoint_chown_coverage.sh` reproduces the whole
chain locally.

### Git worktree path errors

Git worktree `.git` files contain absolute paths. The container mounts the workspace
at the same absolute path as the host to avoid path mismatches. If you see errors like
"not a git repository", verify the mount:

```bash
# Inside container
cat .git  # Should show gitdir with a valid absolute path
ls -la $(cat .git | awk '{print $2}')  # Should be accessible
```

### rosdep failures

Most layer deps are baked into the image at build time (see [Dependency
Baking](#dependency-baking)), so the launch-time check normally skips. If a
launch still installs many deps via apt, the image is stale relative to the
current layer `package.xml` set — rebuild it (`make agent-build`). Force a
launch-time refresh with `FORCE_DEPS_REFRESH=1`.

Some rosdep dependencies may not be available at all. Both the bake and the
entrypoint run `rosdep install` best-effort and continue on failures. If a
specific package fails to build due to missing dependencies, report them via
`issue_request.sh` or note them for the host user. The container runs as a
non-root user without sudo (the bake runs as root at build time, where apt is
available).

### Container won't start

Check that:
1. Docker is running: `docker info`
2. Image exists: `docker images | grep ros2-agent`
3. Authentication is available — the launcher accepts any one of three sources
   and errors out with none — unless `--shell` was passed, which that guard
   exempts (`docker_run_agent.sh:317-328`). Test for
   *presence*; never echo the value, not even a prefix — these are long-lived
   credentials and terminal scrollback and screenshots outlive the check:

   ```bash
   [ -n "$CLAUDE_CODE_OAUTH_TOKEN" ] && echo "CLAUDE_CODE_OAUTH_TOKEN: set"
   [ -n "$ANTHROPIC_API_KEY" ]       && echo "ANTHROPIC_API_KEY: set"
   [ -f ~/.claude/.credentials.json ] && echo "~/.claude/.credentials.json: present"
   ```

4. Worktree exists: `.agent/scripts/worktree_list.sh`
