#!/bin/bash
# .agent/scripts/docker_run_agent.sh
# Host-side launcher for the sandboxed agent DevContainer.
#
# Orchestrates: build image → validate worktree → generate mounts → launch container
# After exit: check for pending push requests and run push gateway.
#
# Usage:
#   .agent/scripts/docker_run_agent.sh --issue <N>
#   .agent/scripts/docker_run_agent.sh --issue <N> --build
#   .agent/scripts/docker_run_agent.sh --issue <N> --shell
#   .agent/scripts/docker_run_agent.sh --issue <N> --print-mounts
#   .agent/scripts/docker_run_agent.sh --build-only        (what `make agent-build` runs)

set -euo pipefail

# ---------- Constants ----------

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# ROOT_DIR is derived from the script's own location. A test can point the
# mount logic at a fabricated worktree tree via DRA_ROOT_DIR_OVERRIDE (used by
# --print-mounts; see tests/test_docker_run_mount_args.sh).
#
# The override is deliberately purpose-named rather than honouring an ambient
# ROOT_DIR: this value decides every docker bind mount, so an inherited one
# would silently repoint them and widen what the container can reach. That is
# not hypothetical — .agent/scripts/agent:19-20 already exports ROOT_DIR, and
# dispatch_subagent.sh invokes this script as a child, so a bare
# ${ROOT_DIR:=...} would consume it. The two happen to compute the same path
# today; a purpose-named knob means they cannot diverge into a surprise.
ROOT_DIR="${DRA_ROOT_DIR_OVERRIDE:-$(dirname "$(dirname "$SCRIPT_DIR")")}"

# If running from inside a worktree, resolve to the main workspace root.
# Worktree directories (.workspace-worktrees/, layers/worktrees/) live there.
if [[ "$ROOT_DIR" == *"/.workspace-worktrees/"* ]]; then
    ROOT_DIR="${ROOT_DIR%%/.workspace-worktrees/*}"
elif [[ "$ROOT_DIR" == *"/layers/worktrees/"* ]]; then
    ROOT_DIR="${ROOT_DIR%%/layers/worktrees/*}"
fi

IMAGE_NAME="ros2-agent-workspace-agent"
IMAGE_TAG="latest"
CONTAINER_PREFIX="ros2-agent"

# ---------- Argument parsing ----------

ISSUE=""
REPO_SLUG=""           # --repo-slug: disambiguate a layer worktree (issue-<slug>-<N>) (#526)
BUILD_IMAGE=false
BUILD_ONLY=false       # --build-only: build the image and exit (no worktree, no container)
SHELL_MODE=false
PROMPT=""              # kickoff prompt text (dispatch mode); empty → interactive
PROMPT_SET=false       # tracks whether a prompt source (--prompt/--prompt-file) was given
OUTPUT_FORMAT="stream-json"  # claude -p --output-format in dispatch mode
MODEL=""               # claude --model (alias like 'opus'/'sonnet', or full id); empty → claude default
PRINT_MOUNTS=false     # --print-mounts: dry-run, dump docker -v args and exit (no Docker)

show_usage() {
    cat <<'EOF'
Usage: docker_run_agent.sh --issue <N> [OPTIONS]

Launch a sandboxed agent container for a worktree.

Required:
  --issue <N>       Issue number (worktree must exist on host).
                    Not required with --build-only.

Options:
  --repo-slug <slug>    Disambiguate a layer worktree (issue-<slug>-<N>) when
                        multiple repos share the issue number (#526)
  --build               Build/rebuild the Docker image before launch
  --build-only          Build the image and exit — no worktree, no container.
                        The single build path: `make agent-build` runs this, so
                        both entry points bake and stamp identically (#604).
  --shell               Drop into bash instead of Claude Code (debugging)
  --prompt <text>       Dispatch mode: run a headless `claude -p` with this
                        kickoff prompt and exit (non-interactive). Mutually
                        exclusive with --shell.
  --prompt-file <path>  Dispatch mode: read the kickoff prompt from a file
                        (preferred for multi-line prompts). Mutually exclusive
                        with --prompt and --shell.
  --output-format <fmt> Dispatch-mode claude output format: stream-json
                        (default), json, or text. Ignored without a prompt.
  --model <id>          claude --model (prefer an alias like 'opus'/'sonnet'
                        over a pinned id). Empty => claude's default. An
                        unavailable model makes claude exit non-zero.
  --print-mounts        Dry run: print the docker -v mount arguments that would
                        be used and exit, without building the image or running
                        a container. Requires no Docker daemon or auth; used by
                        tests/test_docker_run_mount_args.sh. Combined with
                        --build or --build-only it reports what WOULD be built
                        and builds nothing.
  -h, --help            Show this help

Prerequisites:
  - Worktree must be created on host first:
      .agent/scripts/worktree_create.sh --issue <N> --type workspace
  - ANTHROPIC_API_KEY or subscription credentials must be available

Modes:
  - Interactive (default): a TTY Claude Code session you drive.
  - Dispatch (--prompt / --prompt-file): a headless `claude -p` run that
    executes the kickoff and exits. The push-gateway post-exit step is
    skipped; the caller (e.g. dispatch_subagent.sh) reads the worktree's
    progress.md for the outcome.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --issue)
            if [[ $# -lt 2 ]]; then
                echo "ERROR: --issue requires a value." >&2
                show_usage >&2
                exit 1
            fi
            ISSUE="$2"; shift 2 ;;
        --repo-slug)
            if [[ $# -lt 2 ]]; then
                echo "ERROR: --repo-slug requires a value." >&2
                show_usage >&2
                exit 1
            fi
            REPO_SLUG="$2"; shift 2 ;;
        --build)
            BUILD_IMAGE=true; shift ;;
        --build-only)
            BUILD_IMAGE=true; BUILD_ONLY=true; shift ;;
        --shell)
            SHELL_MODE=true; shift ;;
        --prompt)
            if [[ $# -lt 2 ]]; then
                echo "ERROR: --prompt requires a value." >&2
                show_usage >&2
                exit 1
            fi
            if [ "$PROMPT_SET" = true ]; then
                echo "ERROR: --prompt and --prompt-file are mutually exclusive." >&2
                exit 1
            fi
            [ -n "$2" ] || { echo "ERROR: --prompt is empty." >&2; exit 1; }
            PROMPT="$2"; PROMPT_SET=true; shift 2 ;;
        --prompt-file)
            if [[ $# -lt 2 ]]; then
                echo "ERROR: --prompt-file requires a path." >&2
                show_usage >&2
                exit 1
            fi
            if [ "$PROMPT_SET" = true ]; then
                echo "ERROR: --prompt and --prompt-file are mutually exclusive." >&2
                exit 1
            fi
            if [ ! -f "$2" ]; then
                echo "ERROR: --prompt-file not found: $2" >&2
                exit 1
            fi
            [ -s "$2" ] || { echo "ERROR: --prompt-file is empty: $2" >&2; exit 1; }
            PROMPT="$(cat "$2")"; PROMPT_SET=true; shift 2 ;;
        --output-format)
            if [[ $# -lt 2 ]]; then
                echo "ERROR: --output-format requires a value." >&2
                show_usage >&2
                exit 1
            fi
            case "$2" in
                stream-json|json|text) : ;;
                *) echo "ERROR: --output-format must be one of stream-json|json|text (got '$2')." >&2; exit 1 ;;
            esac
            OUTPUT_FORMAT="$2"; shift 2 ;;
        --model)
            if [[ $# -lt 2 ]]; then
                echo "ERROR: --model requires a value." >&2
                show_usage >&2
                exit 1
            fi
            MODEL="$2"; shift 2 ;;
        --print-mounts)
            PRINT_MOUNTS=true; shift ;;
        -h|--help)
            show_usage; exit 0 ;;
        *)
            echo "Unknown option: $1" >&2
            show_usage >&2
            exit 1 ;;
    esac
done

if [ -z "$ISSUE" ] && [ "$BUILD_ONLY" = false ]; then
    echo "ERROR: --issue <N> is required." >&2
    show_usage >&2
    exit 1
fi
if [ -n "$ISSUE" ] && ! [[ "$ISSUE" =~ ^[0-9]+$ ]]; then
    echo "ERROR: --issue must be a number (got '$ISSUE')." >&2
    exit 1
fi
# Sanitize --repo-slug to match worktree_create.sh dir naming (non [A-Za-z0-9_]
# -> _), so a hyphenated slug still resolves (#526).
[ -n "$REPO_SLUG" ] && REPO_SLUG="${REPO_SLUG//[^A-Za-z0-9_]/_}"

# Dispatch mode is active when a prompt source was explicitly provided. Gate on
# PROMPT_SET (the intent), not [ -n "$PROMPT" ] (the value): an empty prompt is
# already rejected at parse time, but keying on intent keeps a future empty
# prompt from silently falling back to an interactive `-it` container.
DISPATCH_MODE=false
[ "$PROMPT_SET" = true ] && DISPATCH_MODE=true

# --shell and dispatch mode are mutually exclusive (one wants a TTY, the
# other is headless).
if [ "$SHELL_MODE" = true ] && [ "$DISPATCH_MODE" = true ]; then
    echo "ERROR: --shell and --prompt/--prompt-file are mutually exclusive." >&2
    exit 1
fi

# ---------- Resolve the worktree (fail fast) ----------
# Deliberately ahead of the credential read and the image build: a bad or
# missing --issue must error in under a second, not after a 10-minute rosdep
# bake. --build-only has no worktree at all, so it skips this entirely.
WORKTREE_PATH=""
WORKTREE_MATCHES=0
MATCHED=()
if [ "$BUILD_ONLY" = false ]; then
    if [ -n "$REPO_SLUG" ]; then
        # --repo-slug: resolve the layer worktree exactly (issue-<slug>-<N>) — no
        # glob, no cross-repo issue-# collision (#526).
        candidate="$ROOT_DIR/layers/worktrees/issue-$REPO_SLUG-$ISSUE"
        if [ -d "$candidate" ]; then WORKTREE_PATH="$candidate"; WORKTREE_MATCHES=1; MATCHED=("$candidate"); fi
    else
        for candidate in \
            "$ROOT_DIR/.workspace-worktrees/issue-workspace-$ISSUE" \
            "$ROOT_DIR/layers/worktrees/issue-"*"-$ISSUE"; do
            if [ -d "$candidate" ]; then
                MATCHED+=("$candidate")
                WORKTREE_MATCHES=$((WORKTREE_MATCHES + 1))
                [ -z "$WORKTREE_PATH" ] && WORKTREE_PATH="$candidate"
            fi
        done
    fi

    if [ -z "$WORKTREE_PATH" ]; then
        echo "ERROR: No worktree found for issue #$ISSUE${REPO_SLUG:+ (repo-slug '$REPO_SLUG')}." >&2
        echo "Create one first:" >&2
        echo "  .agent/scripts/worktree_create.sh --issue $ISSUE --type workspace" >&2
        exit 1
    fi
    if [ "$WORKTREE_MATCHES" -gt 1 ]; then
        # FAIL LOUD (#526) — was warn-and-proceed; multiple repos can share an issue
        # number and guessing the first match ran a container against the wrong repo.
        {
            echo "ERROR: $WORKTREE_MATCHES worktrees match issue #$ISSUE — refusing to guess."
            echo "       Disambiguate with --repo-slug <slug>. Candidates:"
            printf '         %s\n' "${MATCHED[@]}"
        } >&2
        exit 1
    fi

    WORKTREE_ID="$(basename "$WORKTREE_PATH")"
    echo "Using worktree: $WORKTREE_PATH (id: $WORKTREE_ID)"
fi

# ---------- Subscription token (file fallback) ----------
# CLAUDE_CODE_OAUTH_TOKEN is the long-lived subscription token from
# `claude setup-token`. Prefer an already-exported env var; otherwise read it
# from a 600-perm file. The file path mirrors the gh-readonly-token pattern
# and keeps the secret out of the launching agent's environment dumps. We
# export it (rather than passing -e VAR=value) so it forwards via the bare
# `-e CLAUDE_CODE_OAUTH_TOKEN` below without ever appearing in `ps`/argv.
#
# Skipped entirely for --build-only: that path launches no container and is
# exempt from the credential check below, so reading the token would only pull
# the subscription secret into the environment `docker build` inherits — a
# credential-free build should stay credential-free.
CLAUDE_OAUTH_TOKEN_FILE="$HOME/.config/ros2-agent/claude-oauth-token"
if [ "$BUILD_ONLY" = false ] \
   && [ -z "${CLAUDE_CODE_OAUTH_TOKEN:-}" ] && [ -f "$CLAUDE_OAUTH_TOKEN_FILE" ]; then
    # Warn (don't fail) if the secret file is group/world-readable.
    PERM=$(stat -c '%a' "$CLAUDE_OAUTH_TOKEN_FILE" 2>/dev/null || echo "")
    case "$PERM" in
        600|400) : ;;
        "") : ;;  # stat unavailable; skip the check
        *) echo "⚠️  $CLAUDE_OAUTH_TOKEN_FILE is mode $PERM — recommend 'chmod 600'." >&2 ;;
    esac
    IFS= read -r CLAUDE_CODE_OAUTH_TOKEN < "$CLAUDE_OAUTH_TOKEN_FILE" || true
    export CLAUDE_CODE_OAUTH_TOKEN
fi

# ---------- Validation ----------

# Check for authentication. Three sources, in order of robustness for a
# headless/sandboxed run:
#   1. CLAUDE_CODE_OAUTH_TOKEN — long-lived (1yr) subscription token from
#      `claude setup-token`. Subscription-native (counts against Max/Pro
#      limits, not API billing), purpose-built for CI/headless. Best for
#      dispatch mode.
#   2. ANTHROPIC_API_KEY — pay-as-you-go API billing.
#   3. ~/.claude/.credentials.json — interactive subscription OAuth. The
#      access token is short-lived and a transplanted stale token can't
#      reliably refresh inside the sandbox, so it's unreliable for headless
#      dispatch (works for interactive sessions where a fresh login is at
#      hand).
# --build-only never launches a container, so it needs no credentials.
if [ "$PRINT_MOUNTS" = false ] && [ "$BUILD_ONLY" = false ] \
   && [ -z "${CLAUDE_CODE_OAUTH_TOKEN:-}" ] && [ -z "${ANTHROPIC_API_KEY:-}" ] \
   && [ ! -f "$HOME/.claude/.credentials.json" ] && [ "$SHELL_MODE" = false ]; then
    echo "ERROR: No authentication found." >&2
    echo "Pick one:" >&2
    echo "  - CLAUDE_CODE_OAUTH_TOKEN  (recommended; 'claude setup-token' in a real" >&2
    echo "                              terminal, then save to $CLAUDE_OAUTH_TOKEN_FILE" >&2
    echo "                              [chmod 600], or export the env var)" >&2
    echo "  - ANTHROPIC_API_KEY        (API billing, not subscription)" >&2
    echo "  - 'claude' + /login on the host for interactive subscription auth" >&2
    exit 1
fi

# Dispatch mode is headless, so transplanted OAuth credentials.json can't
# refresh — steer the user to the long-lived token unless they've set one
# (or an API key).
if [ "$DISPATCH_MODE" = true ] && [ -z "${CLAUDE_CODE_OAUTH_TOKEN:-}" ] && [ -z "${ANTHROPIC_API_KEY:-}" ]; then
    echo "⚠️  Dispatch (headless) mode with no CLAUDE_CODE_OAUTH_TOKEN / ANTHROPIC_API_KEY." >&2
    echo "    Mounted ~/.claude/.credentials.json OAuth tokens expire and cannot refresh in" >&2
    echo "    the sandbox — the run will likely fail with 'Not logged in'. Generate a" >&2
    echo "    long-lived subscription token with 'claude setup-token' and save it to" >&2
    echo "    $CLAUDE_OAUTH_TOKEN_FILE (chmod 600) before dispatching." >&2
fi

# ---------- Build image (if requested or missing) ----------

DOCKERFILE_DIR="$ROOT_DIR/.devcontainer/agent"

# Combined hash of the startup scripts baked into the image (#604). Stamped
# into the image at build time and compared at launch: `docker build` is only
# triggered when the image is MISSING, so without this an image built before a
# change to either script keeps running the old copy — exactly how a fix to
# agent-entrypoint.sh can silently fail to reach an existing user. Hashed by
# content in a fixed order from DOCKERFILE_DIR, so the digest is independent of
# the checkout path.
#
# THIS IS THE ONLY IMPLEMENTATION of the digest, and this script is the only
# build path: `make agent-build` delegates here via --build-only rather than
# running its own `docker build`. A second formula elsewhere could hash a
# different directory (a worktree's copy vs. the main tree's) and stamp a
# digest the launcher would then read as permanently "stale".
# tests/test_agent_image_build_paths.sh enforces the single build path.
STARTUP_SCRIPTS_LABEL="org.ros2-agent.startup-scripts-sha"
STARTUP_SCRIPTS=(agent-entrypoint.sh fix-volume-ownership.sh)

# Print the combined digest, or return 1 printing NOTHING when any script is
# missing. Returning empty rather than hashing what happens to be there is
# deliberate: a partial `cat` produces a well-formed but wrong digest, which
# would read as a confident "stale"/"current" verdict about scripts that
# aren't even present.
startup_scripts_sha() {
    local f
    for f in "${STARTUP_SCRIPTS[@]}"; do
        [ -f "$DOCKERFILE_DIR/$f" ] || return 1
    done
    (cd "$DOCKERFILE_DIR" && cat "${STARTUP_SCRIPTS[@]}") | sha256sum | cut -d' ' -f1
}

# Warn (never block) when the image's baked startup scripts are not the ones
# this checkout would bake. Contract: this function must always succeed — it is
# called from the main flow under `set -euo pipefail`, so every substitution
# that can fail carries `|| true` and the function ends with `return 0`.
#
# An ABSENT label is warned about too, not treated as "unknown, stay quiet":
# every image without the label was built before this marker existed, and that
# is exactly the population carrying the pre-#604 entrypoint. Staying quiet
# would withhold the warning from precisely the users who need it.
warn_if_startup_scripts_stale() {
    local baked current
    current="$(startup_scripts_sha || true)"
    if [ -z "$current" ]; then
        echo "⚠️  Cannot check image startup-script staleness: missing" >&2
        echo "   ${STARTUP_SCRIPTS[*]} under $DOCKERFILE_DIR." >&2
        return 0
    fi
    baked="$(docker image inspect "$IMAGE_NAME:$IMAGE_TAG" \
        --format "{{index .Config.Labels \"$STARTUP_SCRIPTS_LABEL\"}}" 2>/dev/null || true)"
    if [ -z "$baked" ]; then
        echo "⚠️  Image $IMAGE_NAME:$IMAGE_TAG carries no startup-scripts marker." >&2
        echo "   It predates the marker (#604) or was built by a bare 'docker build'," >&2
        echo "   so it most likely bakes an OLDER agent-entrypoint.sh. Rebuild:" >&2
        echo "       make agent-build   (or: $0 --issue <N> --build)" >&2
        return 0
    fi
    if [ "$baked" != "$current" ]; then
        echo "⚠️  Image $IMAGE_NAME:$IMAGE_TAG was built from older startup scripts." >&2
        echo "   ${STARTUP_SCRIPTS[*]} have changed since." >&2
        echo "   The container will run the BAKED copies. Rebuild to pick them up:" >&2
        echo "       make agent-build   (or: $0 --issue <N> --build)" >&2
    fi
    return 0
}

# --print-mounts is a Docker-free dry run: skip the image existence check
# (a docker call) so the mount logic can be exercised without a daemon.
if [ "$PRINT_MOUNTS" = false ] && ! docker image inspect "$IMAGE_NAME:$IMAGE_TAG" >/dev/null 2>&1; then
    echo "Image $IMAGE_NAME:$IMAGE_TAG not found — building..."
    BUILD_IMAGE=true
fi

if [ "$BUILD_IMAGE" = false ] && [ "$PRINT_MOUNTS" = false ]; then
    warn_if_startup_scripts_stale
fi

if [ "$BUILD_IMAGE" = true ] && [ "$PRINT_MOUNTS" = true ]; then
    # --print-mounts is a Docker-free dry run, so it must not build. Say so
    # rather than skipping in silence: `--build-only --print-mounts` would
    # otherwise be a wordless rc-0 no-op, and this is the form the build-path
    # test uses to exercise the parse without touching the real :latest tag.
    echo "[dry run] --print-mounts: would build $IMAGE_NAME:$IMAGE_TAG from $DOCKERFILE_DIR (no build performed)"
fi

if [ "$BUILD_IMAGE" = true ] && [ "$PRINT_MOUNTS" = false ]; then
    # The digest must be computable or the image would bake an incomplete
    # startup path — fail loud rather than stamping a wrong/empty marker.
    if ! BAKE_SHA="$(startup_scripts_sha)"; then
        echo "ERROR: cannot build — missing ${STARTUP_SCRIPTS[*]} under $DOCKERFILE_DIR." >&2
        exit 1
    fi

    # ---------- Stage layer manifests for the rosdep bake (#520) ----------
    # The agent image bakes the workspace's layer system-deps at build time so
    # each launch installs only the delta. The build context (.devcontainer/
    # agent/) has no layer source — layers/ is gitignored and mounted at
    # runtime, never copied — so gather just the package.xml manifests here,
    # host-side where layers/ exists, into a staging dir the Dockerfile COPYs.
    # The gather logic lives in stage_rosdep_manifests.sh (shared with
    # `make agent-build`) so both build entry points stage identically.
    STAGE_DIR="$DOCKERFILE_DIR/.rosdep-manifests"

    # Serialize concurrent builds. STAGE_DIR is a FIXED path — the Dockerfile's
    # `COPY .rosdep-manifests/` is build-context-relative, so it cannot be
    # randomized per invocation — and it is removed by the EXIT trap below. Two
    # builders racing (a dispatch auto-build and `make agent-build`, say) would
    # otherwise have one delete the other's build context mid-build. The lock
    # lives in TMPDIR, keyed by user and by the context path, so it pollutes
    # neither the working tree nor another user's build. (cksum, not sha256sum:
    # the launcher must carry exactly ONE sha256sum site — the startup-scripts
    # digest formula — and test_agent_image_build_paths.sh enforces that.)
    # fd 9 is released when
    # this process exits. Where flock is unavailable, proceed unserialized
    # rather than failing the build outright — the race is rare and the
    # alternative is no build at all.
    STAGE_LOCK="${TMPDIR:-/tmp}/ros2-agent-build-$(id -u)-$(printf '%s' "$DOCKERFILE_DIR" | cksum | cut -d' ' -f1).lock"
    if command -v flock >/dev/null 2>&1 && exec 9>"$STAGE_LOCK"; then
        flock 9 || echo "⚠️  could not acquire $STAGE_LOCK — building unserialized." >&2
    fi

    # Clean the staging dir on any exit from here through the build — including
    # a `set -e` abort on a failed `docker build` — so it never lingers in the
    # working tree (workspace-cleanliness rule). Cleared after the build below.
    trap 'rm -rf "$STAGE_DIR"' EXIT
    "$SCRIPT_DIR/stage_rosdep_manifests.sh" "$ROOT_DIR" "$STAGE_DIR"

    echo "Building agent image from $DOCKERFILE_DIR..."
    docker build \
        --build-arg USER_UID="$(id -u)" \
        --build-arg USER_GID="$(id -g)" \
        --build-arg STARTUP_SCRIPTS_SHA="$BAKE_SHA" \
        -t "$IMAGE_NAME:$IMAGE_TAG" \
        -f "$DOCKERFILE_DIR/Dockerfile" \
        "$DOCKERFILE_DIR"
    echo "Image built: $IMAGE_NAME:$IMAGE_TAG"
    # Build succeeded — remove the staged manifests and clear the cleanup trap.
    rm -rf "$STAGE_DIR"
    trap - EXIT
fi

# --build-only (what `make agent-build` runs): the image is the whole job —
# no worktree, no auth, no container.
if [ "$BUILD_ONLY" = true ]; then
    exit 0
fi

# ---------- Generate mount arguments ----------

MOUNT_ARGS=()

# 1. Bind mount the entire workspace root at the same absolute path
#    (preserves git worktree .git file absolute paths)
MOUNT_ARGS+=(-v "$ROOT_DIR:$ROOT_DIR")

# 2. Read-only overlay for .agent/ (scripts, configs, templates)
MOUNT_ARGS+=(-v "$ROOT_DIR/.agent:$ROOT_DIR/.agent:ro")

# 3. Read-write override for .agent/scratchpad/ (push requests, temp files)
# Skipped for --print-mounts: a dry run must be inert (#602 review).
# Written as an `if` rather than `[ ... ] && mkdir -p ...`. At this position
# either form is safe. The difference is the block's exit status: the `&&` form
# evaluates to the guard's status, so it is false whenever the guard is. That
# only matters if this block ever ends a function (`set -e` then aborts the
# caller) or ends a script (its status becomes the script's, with or without
# `set -e`). The `if` has no such coupling and matches the two `if ! mkdir`
# guards below.
if [ "$PRINT_MOUNTS" = false ]; then
    mkdir -p "$ROOT_DIR/.agent/scratchpad/push-requests"
fi
MOUNT_ARGS+=(-v "$ROOT_DIR/.agent/scratchpad:$ROOT_DIR/.agent/scratchpad")

# 4. Anonymous volumes for build/install/log in each layer workspace
#    (container gets its own build artifacts, doesn't pollute host)
#    Pre-create each mountpoint as the invoking user: docker creates missing
#    mountpoint sources as root:root, which breaks subsequent host-side builds
#    (EACCES) — e.g. a container start racing the #559 clean-rebuild heal.
#    Docker never chowns an existing dir, so mkdir -p here prevents the hazard
#    in normal use (a concurrent rm between mkdir and docker run remains
#    possible but self-inflicted) and is a no-op when the dirs exist (#566).
#    Fail loud if mkdir fails — proceeding would let docker recreate the
#    mountpoint root-owned, silently defeating the fix.
for ws_dir in "$ROOT_DIR"/layers/main/*_ws; do
    [ -d "$ws_dir" ] || continue
    for subdir in build install log; do
        if [ "$PRINT_MOUNTS" = false ] && ! mkdir -p "$ws_dir/$subdir"; then
            echo "❌ Error: cannot create mountpoint $ws_dir/$subdir (would be" >&2
            echo "   created root-owned by docker — see #566). Fix and retry." >&2
            exit 1
        fi
        MOUNT_ARGS+=(-v "$ws_dir/$subdir")
    done
done

# 4b. Anonymous volumes for build/install/log in the DISPATCHED WORKTREE's
#     layer workspaces. The section-4 loop only shields layers/main/*_ws; a
#     dispatched worktree has its own *_ws build artifacts at a distinct
#     absolute path, and the workspace bind mount (section 1) otherwise shares
#     that path straight through to the host. Without this shield, container
#     and host builds write objects compiled against different ROS package sets
#     to the same path — the contamination #602 fixes.
#
#     Scope to the already-resolved $WORKTREE_PATH ONLY, never a glob of all
#     worktrees: globbing every worktree path matches hundreds of entries (many
#     stale/removed) and would mount scratch space this launch never touches.
#
#     Guard on [ -d ] AND [ ! -L ]: inside a layer worktree only the --layer
#     target is a real directory; its sibling *_ws entries are symlinks into
#     layers/main, which the section-4 loop already shields. [ -d ] follows
#     symlinks (so it does NOT filter them) — the explicit [ ! -L ] does,
#     preventing mkdir -p from reaching through a symlink into the
#     already-shielded layers/main tree. Inside a workspace worktree no real
#     *_ws dirs exist, so the loop is a clean no-op. Mirror section 4's
#     mkdir-as-invoking-user precaution and fail-loud handling (#566).
#
#     THE MOUNT IS ONLY HALF THE MECHANISM (#604). The mkdir below is the
#     #566 precaution about the *host* mountpoint; it has NO effect on the
#     volume's ownership inside the container, because docker initializes an
#     anonymous volume from the IMAGE at that path, not from the host dir
#     underneath it. The volume therefore comes up root-owned, and the
#     entrypoint chown is what makes it writable by the dropped-privilege
#     agent. #602 added this loop without the matching chown, and dispatched
#     agents could not build in their own worktree. ANY new anonymous volume
#     added here needs a matching loop in
#     .devcontainer/agent/fix-volume-ownership.sh; that pairing is enforced
#     by tests/test_entrypoint_chown_coverage.sh.
for ws_dir in "$WORKTREE_PATH"/*_ws; do
    [ -d "$ws_dir" ] || continue
    [ ! -L "$ws_dir" ] || continue
    for subdir in build install log; do
        if [ "$PRINT_MOUNTS" = false ] && ! mkdir -p "$ws_dir/$subdir"; then
            echo "❌ Error: cannot create mountpoint $ws_dir/$subdir (would be" >&2
            echo "   created root-owned by docker — see #566). Fix and retry." >&2
            exit 1
        fi
        MOUNT_ARGS+=(-v "$ws_dir/$subdir")
    done
done

# 5. Read-write: worktree directories (where agent makes changes)
MOUNT_ARGS+=(-v "$ROOT_DIR/layers/worktrees:$ROOT_DIR/layers/worktrees")
MOUNT_ARGS+=(-v "$ROOT_DIR/.workspace-worktrees:$ROOT_DIR/.workspace-worktrees")

# 6. Claude Code config (skips onboarding, inherits auth + preferences)
#    Mount to staging paths — the entrypoint copies into user-owned locations
#    so Claude Code can create sibling directories (plugins/, debug/, etc.)
#    ~/.claude.json          — onboarding state (hasCompletedOnboarding flag)
#    ~/.claude/settings.json — user preferences
#    ~/.claude/.credentials.json — Anthropic subscription auth (NOT GitHub creds)
CLAUDE_STATE="$HOME/.claude.json"
CLAUDE_SETTINGS="$HOME/.claude/settings.json"
CLAUDE_CREDS="$HOME/.claude/.credentials.json"
if [ -f "$CLAUDE_STATE" ]; then
    MOUNT_ARGS+=(-v "$CLAUDE_STATE:/tmp/claude-state.json:ro")
fi
if [ -f "$CLAUDE_SETTINGS" ]; then
    MOUNT_ARGS+=(-v "$CLAUDE_SETTINGS:/tmp/claude-settings.json:ro")
fi
if [ -f "$CLAUDE_CREDS" ]; then
    MOUNT_ARGS+=(-v "$CLAUDE_CREDS:/tmp/claude-credentials.json:ro")
fi

# 7. Pre-commit hook-env cache — amortize the ~30s hook-environment
#    install across container sessions via a NAMED VOLUME, deliberately
#    NOT a bind of the host's ~/.cache/pre-commit. A read-write host bind
#    would let a --dangerously-skip-permissions agent overwrite the
#    host's hook-environment binaries (interpreters, console-script
#    shims), which the human then executes at the next host-side commit —
#    a sandbox-escape / persistence vector outside the PR review path.
#    A named volume stays writable (pre-commit can install missing envs)
#    and amortizes across container runs, with no route back to the
#    host's own cache. First run pays the one-time install; the entrypoint
#    chowns the root-owned mountpoint to the ros user.
MOUNT_ARGS+=(-v "ros2-agent-precommit-cache:/home/ros/.cache/pre-commit")

# 8. Persist the Claude Code plugin marketplace across runs via a named
#    volume, so each container start doesn't re-clone claude-plugins-official.
#    A named volume (not a host bind) keeps it isolated from the host's own
#    ~/.claude/plugins. The entrypoint's recursive chown of ~/.claude fixes
#    the root-owned mountpoint docker creates on first use.
MOUNT_ARGS+=(-v "ros2-agent-claude-plugins:/home/ros/.claude/plugins")

# ---------- Dry-run short-circuit (--print-mounts) ----------
# Dump the assembled docker -v arguments and exit before any Docker
# interaction. Placed after all MOUNT_ARGS assembly (through section 8) and
# before the container launch, so tests/test_docker_run_mount_args.sh can
# inspect the mount plan in CI without a daemon or credentials.
if [ "$PRINT_MOUNTS" = true ]; then
    printf '%s\n' "${MOUNT_ARGS[@]}"
    exit 0
fi

# ---------- Read-only GitHub token (optional) ----------
# Provides container agents with read-only gh CLI access (view issues, PRs, code search).
# Token sources (first match wins):
#   1. AGENT_GH_TOKEN env var (explicit override)
#   2. ~/.config/ros2-agent/gh-readonly-token file (persistent config)
# If neither is set, gh inside the container has no auth (existing behavior).

AGENT_GH_TOKEN="${AGENT_GH_TOKEN:-}"
GH_TOKEN_FILE="$HOME/.config/ros2-agent/gh-readonly-token"

if [ -z "$AGENT_GH_TOKEN" ] && [ -f "$GH_TOKEN_FILE" ]; then
    IFS= read -r AGENT_GH_TOKEN < "$GH_TOKEN_FILE" || true
fi

# Export the read-only GH token so it forwards via the bare `-e GH_TOKEN` below
# (value-in-env, not value-in-argv) — keeping it out of the host's `ps`/cmdline,
# matching the CLAUDE_CODE_OAUTH_TOKEN handling above.
[ -n "$AGENT_GH_TOKEN" ] && export GH_TOKEN="$AGENT_GH_TOKEN"

# ---------- Container environment ----------

# Forward the agent git identity into the container. No
# `configure_git_identity.sh --detect` runs in the container anymore (that
# entrypoint step was removed). Container commit identity now flows two ways:
#   (a) the dispatch handoff prompt's per-invocation `git -c user.name=… -c
#       user.email=… commit` literals (the load-bearing path); and
#   (b) agent-entrypoint.sh exports GIT_AUTHOR_*/GIT_COMMITTER_* from
#       AGENT_NAME/AGENT_EMAIL before sourcing setup.bash, so setup.bash's
#       detect-and-set fallback is skipped and any `git commit` that omits the
#       `-c` literals still uses the *forwarded* identity, not a re-detected one.
# AGENT_NAME/AGENT_EMAIL are forwarded so (b) and `gh` signatures have them.
# Warn if they're missing so the env-fallback identity doesn't go unset.
if [ "$SHELL_MODE" = false ] && { [ -z "${AGENT_NAME:-}" ] || [ -z "${AGENT_EMAIL:-}" ]; }; then
    echo "⚠️  AGENT_NAME / AGENT_EMAIL not set — the container's env-fallback git" >&2
    echo "    identity will be unset (commits then rely solely on the handoff's" >&2
    echo "    'git -c' literals). Source .agent/scripts/set_git_identity_env.sh" >&2
    echo "    before launching to set them." >&2
fi

ENV_ARGS=(
    ${CLAUDE_CODE_OAUTH_TOKEN:+-e "CLAUDE_CODE_OAUTH_TOKEN"}
    ${ANTHROPIC_API_KEY:+-e "ANTHROPIC_API_KEY"}
    ${AGENT_GH_TOKEN:+-e GH_TOKEN}
    ${AGENT_NAME:+-e "AGENT_NAME=$AGENT_NAME"}
    ${AGENT_EMAIL:+-e "AGENT_EMAIL=$AGENT_EMAIL"}
    ${AGENT_MODEL:+-e "AGENT_MODEL=$AGENT_MODEL"}
    ${AGENT_FRAMEWORK:+-e "AGENT_FRAMEWORK=$AGENT_FRAMEWORK"}
    -e "CLAUDE_CODE_ENTRYPOINT=1"
    -e "ROS2_AGENT_WORKSPACE_ROOT=$ROOT_DIR"
    -e "WORKTREE_ROOT=$WORKTREE_PATH"
    -e "WORKTREE_ID=$WORKTREE_ID"
    -e "WORKTREE_ISSUE=$ISSUE"
    -e "GIT_EDITOR=true"
)

# ---------- Determine CMD ----------

# --strict-mcp-config: use only MCP servers from --mcp-config (none
# passed) → all MCP servers disabled. Silences the claude.ai
# Gmail/Drive/Calendar servers (and any other user-scoped MCP servers
# inherited via the mounted ~/.claude.json) that fail-fast with auth
# errors on every start in the credential-less sandbox — they're
# host-only by design (see #481 non-goals). The workspace defines no
# project/.mcp.json servers, so nothing the sandbox legitimately needs
# is lost; local skills still resolve via /skill-name. Removes the
# startup-log noise and a round-trip per server.
# --model is optional. Prefer an alias ('opus'/'sonnet') over a pinned id so
# it survives model version bumps; an unavailable model makes claude exit
# non-zero (hard-fail, surfaced by the caller's exit-contract — no silent
# downgrade). Empty MODEL => claude's built-in default.
MODEL_FLAGS=()
[ -n "$MODEL" ] && MODEL_FLAGS=(--model "$MODEL")

CONTAINER_CMD=()
if [ "$SHELL_MODE" = true ]; then
    CONTAINER_CMD=(/bin/bash)
elif [ "$DISPATCH_MODE" = true ]; then
    # Headless kickoff. `-p` takes the prompt as a single argv element
    # (passed through the entrypoint's `exec ... -- "$@"` unchanged, so no
    # shell re-splitting/quoting issue). stream-json output requires
    # --verbose; for other formats --verbose is harmless.
    CONTAINER_CMD=(claude -p "$PROMPT"
        --output-format "$OUTPUT_FORMAT" --verbose
        "${MODEL_FLAGS[@]}"
        --dangerously-skip-permissions --strict-mcp-config)
else
    CONTAINER_CMD=(claude "${MODEL_FLAGS[@]}" --dangerously-skip-permissions --strict-mcp-config)
fi

# TTY allocation: interactive/shell modes want a TTY; headless dispatch
# must NOT allocate one (no stdin TTY in an orchestrated/CI context, and
# stream-json is meant to be piped).
TTY_FLAGS=(-it)
[ "$DISPATCH_MODE" = true ] && TTY_FLAGS=()

# ---------- Launch container ----------

CONTAINER_NAME="${CONTAINER_PREFIX}-${WORKTREE_ID}"

echo ""
echo "========================================="
echo "  Launching Sandboxed Agent Container"
echo "========================================="
echo "  Issue:     #$ISSUE"
echo "  Worktree:  $WORKTREE_PATH"
echo "  Container: $CONTAINER_NAME"
if [ "$SHELL_MODE" = true ]; then
    RUN_MODE="shell (debug)"
elif [ "$DISPATCH_MODE" = true ]; then
    RUN_MODE="dispatch (headless claude -p, $OUTPUT_FORMAT)"
else
    RUN_MODE="Claude Code (YOLO)"
fi
echo "  Mode:      $RUN_MODE"
echo "  Model:     ${MODEL:-(claude default)}"
echo "  GitHub:    $([ -n "$AGENT_GH_TOKEN" ] && echo 'read-only token' || echo 'no token')"
echo "========================================="
echo ""

# Allow non-zero exit (user Ctrl+C, Claude exit, etc.) — we still check push requests
EXIT_CODE=0
docker run "${TTY_FLAGS[@]}" --rm \
    --name "$CONTAINER_NAME" \
    --hostname "agent-$ISSUE" \
    --security-opt no-new-privileges:true \
    -w "$WORKTREE_PATH" \
    "${MOUNT_ARGS[@]}" \
    "${ENV_ARGS[@]}" \
    "$IMAGE_NAME:$IMAGE_TAG" \
    "${CONTAINER_CMD[@]}" \
    || EXIT_CODE=$?

# ---------- Post-exit ----------
# Dispatch mode uses progress.md as the outcome channel (the caller reads
# the worktree's last entry), so skip the interactive push-gateway flow
# entirely and just surface the container exit code.
if [ "$DISPATCH_MODE" = true ]; then
    echo ""
    echo "Dispatch run exited with code $EXIT_CODE."
    echo "Outcome is in the worktree's progress.md (read the last entry)."
    exit "$EXIT_CODE"
fi

# ---------- Post-exit: check for outbox items (interactive mode) ----------

echo ""
HAS_PENDING=false

# Check push requests
PUSH_REQUEST="$ROOT_DIR/.agent/scratchpad/push-requests/$WORKTREE_ID.json"
if [ -f "$PUSH_REQUEST" ]; then
    STATUS=$(jq -r '.status // "pending"' "$PUSH_REQUEST" 2>/dev/null || echo "unknown")
    if [ "$STATUS" = "pending" ]; then
        HAS_PENDING=true
    fi
fi

# Check issue requests
ISSUE_DIR="$ROOT_DIR/.agent/scratchpad/issue-requests/$WORKTREE_ID"
if [ -d "$ISSUE_DIR" ]; then
    for req in "$ISSUE_DIR"/*.json; do
        [ -f "$req" ] || continue
        STATUS=$(jq -r '.status // "pending"' "$req" 2>/dev/null || echo "unknown")
        if [ "$STATUS" = "pending" ]; then
            HAS_PENDING=true
            break
        fi
    done
fi

if [ "$HAS_PENDING" = true ]; then
    if [ -x "$SCRIPT_DIR/push_gateway.sh" ]; then
        "$SCRIPT_DIR/push_gateway.sh" --worktree-id "$WORKTREE_ID"
    else
        echo "Pending outbox items found. Run manually:"
        echo "  .agent/scripts/push_gateway.sh --worktree-id $WORKTREE_ID"
    fi
else
    echo "No pending outbox items. Container exited with code $EXIT_CODE."
fi
