#!/bin/bash
# .agent/scripts/manifest_fallback.sh
# Make the repo manifests readable on a host that has no `layers/`.
#
# Why: `configs/manifest` is a SYMLINK into the layer tree
# (`layers/main/<layer>_ws/src/<manifest-repo>/config` in this workspace), and
# `layers/` is gitignored — absent in every fresh clone and container. So a
# caller that "does not assume `layers/` exists" still could not enumerate a
# single repo there: the manifests themselves live behind the thing that is
# missing. The manifest is the first thing "it clones what it needs" has to
# cover.
#
# What this does: derive the manifest repo and branch from the TRACKED
# `configs/project_bootstrap.url` pointer, shallow-clone it into the scratch
# cache, and print the config directory to read `.repos` files from. Callers
# pass that to `list_overlay_repos.py --config-dir <dir>`, which ADDS it to the
# normal search path rather than replacing it — a workspace that has a real
# `configs/manifest` never takes this path at all.
#
# Usage (sourced — this file defines functions and runs nothing on its own):
#   source /path/to/.agent/scripts/manifest_fallback.sh
#   extra_config_dir=$(manifest_config_dir "$ROOT"); rc=$?
#   case "$rc" in
#       0) ;;  # usable: EMPTY string means "the workspace's own manifest is
#              # present, no override needed"; non-empty is a --config-dir value
#       3) ;;  # no manifest AND no usable bootstrap pointer — nothing to read
#       5) ;;  # the manifest repo could not be USED (reason on stderr): the
#              # clone or the cache/lock failed, the derived git base was
#              # refused, or the pointer and the manifest repo disagree
#   esac
#
# Environment:
#   BOOTSTRAP_URL                  overrides the pointer file (same precedence
#                                  as setup_layers.sh, where it is source 1)
#   WORKSPACE_MANIFEST_GIT_BASE    base for the derived clone url; defaults to
#                                  https://github.com. Must be a
#                                  `<scheme>://<host>/<path>` url (http(s), ssh,
#                                  git, file) — anything else is refused at
#                                  exit 5 rather than handed to `git clone`.
#                                  Exists so the hermetic tests can point at a
#                                  local `file://` fixture — no test ever
#                                  reaches the network.
#
# The clone is a cache, not state: `rm -rf .agent/scratchpad/manifest-repo` at
# any time and the next run re-creates it.

# Sourced, not executed — the mirror image of resolve_repo_checkout.sh's guard.
# Run directly, this file would define two functions and exit 0 having done
# nothing at all: a silent success, from a script whose whole job is to refuse
# to report success it did not earn.
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "manifest_fallback.sh must be sourced, not executed: source ${BASH_SOURCE[0]} && manifest_config_dir <workspace-root>" >&2
    exit 2
fi

# manifest_config_dir <workspace_root>
# See the exit-code table above.
manifest_config_dir() {
    local root="${1:-$PWD}"
    local pointer_file="$root/configs/project_bootstrap.url"
    local bootstrap_url owner repo branch config_path git_url cache clone_dir

    # The normal case: the workspace has its manifest on disk. Print nothing.
    if [[ -d "$root/configs/manifest/repos" ]]; then
        return 0
    fi

    bootstrap_url="${BOOTSTRAP_URL:-}"
    if [[ -z "$bootstrap_url" && -f "$pointer_file" ]]; then
        bootstrap_url=$(tr -d '[:space:]' < "$pointer_file")
    fi
    if [[ -z "$bootstrap_url" ]]; then
        echo "manifest_fallback: no configs/manifest and no configs/project_bootstrap.url under $root — nothing to enumerate repos from" >&2
        return 3
    fi

    # Only the raw.githubusercontent form is derivable without fetching the
    # bootstrap.yaml itself (which would need the network before we know
    # whether we can reach it at all, and would put a YAML parser in the
    # failure path). Anything else is reported as unsupported rather than
    # guessed at.
    if [[ ! "$bootstrap_url" =~ ^https://raw\.githubusercontent\.com/([^/]+)/([^/]+)/([^/]+)/(.+)/bootstrap\.yaml$ ]]; then
        echo "manifest_fallback: '$bootstrap_url' is not a raw.githubusercontent.com/<owner>/<repo>/<branch>/<path>/bootstrap.yaml url — this fallback cannot derive a git url from it; run 'make setup-all' on a host that can" >&2
        return 3
    fi
    owner="${BASH_REMATCH[1]}"
    repo="${BASH_REMATCH[2]}"
    branch="${BASH_REMATCH[3]}"
    config_path="${BASH_REMATCH[4]}"
    repo="${repo%.git}"

    # Each of these is interpolated into a url, a `git clone --branch`, or a
    # path. A branch with a `/` in it is indistinguishable from the config path
    # in a raw url, so it is refused rather than split on a guess.
    if [[ ! "$owner" =~ ^[A-Za-z0-9_][A-Za-z0-9._-]*$ ]] \
       || [[ ! "$repo" =~ ^[A-Za-z0-9_][A-Za-z0-9._-]*$ ]] \
       || [[ ! "$branch" =~ ^[A-Za-z0-9_][A-Za-z0-9._-]*$ ]] \
       || [[ "$config_path" == /* || "$config_path" == *..* ]]; then
        echo "manifest_fallback: '$bootstrap_url' does not parse into a safe <owner>/<repo>/<branch>/<path> — refusing to guess" >&2
        return 3
    fi

    # The base reaches `git clone`, and it is the root of the whole trust chain
    # (manifest repo -> the repo list -> every repo the sweep clones). It comes
    # from the ambient environment, so require a recognised url form — the same
    # shape resolve_repo_checkout.sh requires of a manifest url — so it can
    # never be read as a git OPTION or as a stray local path.
    local git_base="${WORKSPACE_MANIFEST_GIT_BASE:-https://github.com}"
    if [[ ! "$git_base" =~ ^(https?|ssh|git|file)://[^[:space:]]+$ ]] || [[ "$git_base" == *..* ]]; then
        echo "manifest_fallback: WORKSPACE_MANIFEST_GIT_BASE '$git_base' is not a <scheme>://<host>/<path> url — refusing to hand it to git clone" >&2
        return 5
    fi
    git_url="${git_base%/}/$owner/$repo.git"
    cache="$root/.agent/scratchpad/manifest-repo"
    clone_dir="$cache/$repo"

    if ! mkdir -p "$cache" 2>/dev/null; then
        echo "manifest_fallback: cannot create the manifest cache at $cache" >&2
        return 5
    fi

    # Same shared-cache hazard as the repo clone cache, same remedy: a per-repo
    # advisory lock with a bounded wait, held for the clone/refresh. Absent
    # flock the run proceeds unlocked and says so.
    if command -v flock >/dev/null 2>&1; then
        if ! { exec 8>"$cache/.$repo.lock" && flock -w 120 8; }; then
            echo "manifest_fallback: could not lock the manifest cache for $repo within 120s" >&2
            return 5
        fi
    else
        echo "manifest_fallback: flock not available — the manifest cache is unlocked for this run" >&2
    fi

    local out cached_url
    if [[ -d "$clone_dir/.git" ]]; then
        # The cache is keyed on the repo NAME alone, so a cached clone is only
        # reusable once its origin is confirmed to be the url the CURRENT
        # bootstrap pointer derives — repoint the pointer at a different
        # owner's manifest and the old clone would otherwise be returned at
        # exit 0, and the whole rotation would enumerate from a manifest this
        # workspace no longer points at. `resolve_repo_checkout.sh` performs
        # exactly this guard one level down.
        cached_url=$(git -C "$clone_dir" remote get-url origin 2>/dev/null)
        if [[ "$cached_url" != "$git_url" ]]; then
            echo "manifest_fallback: cached manifest clone at $clone_dir points at '${cached_url:-<none>}', the bootstrap pointer derives '$git_url' — re-cloning" >&2
            rm -rf "$clone_dir"
        fi
    fi

    if [[ -d "$clone_dir/.git" ]]; then
        # A refresh failure leaves a usable (if possibly stale) manifest. Say
        # so and carry on — reporting "no manifest configured" over a manifest
        # that is right there would be the worse answer.
        if ! out=$(_manifest_fallback_git git -C "$clone_dir" fetch --depth 1 origin -- "$branch" 2>&1) \
           || ! out=$(git -C "$clone_dir" reset --hard FETCH_HEAD 2>&1); then
            echo "manifest_fallback: could not refresh the cached manifest repo at $clone_dir ($out) — using the cached copy, which may be stale" >&2
        fi
    else
        rm -rf "$clone_dir"
        if ! out=$(_manifest_fallback_git git clone --depth 1 --branch "$branch" -- "$git_url" "$clone_dir" 2>&1); then
            echo "manifest_fallback: could not clone the manifest repo $git_url at '$branch': $out" >&2
            rm -rf "$clone_dir"
            return 5
        fi
    fi

    if [[ ! -d "$clone_dir/$config_path/repos" ]]; then
        echo "manifest_fallback: $git_url ($branch) has no '$config_path/repos' directory — the bootstrap pointer and the manifest repo disagree" >&2
        return 5
    fi

    printf '%s\n' "$clone_dir/$config_path/repos"
    return 0
}

# Non-interactive, wall-clock-bounded git, for the same reasons
# resolve_repo_checkout.sh documents: git's own prompt suppression does not
# cover ssh, and only `timeout` covers a server that stalls after connecting.
_manifest_fallback_git() {
    local -a pre=(env GIT_TERMINAL_PROMPT=0 GIT_ASKPASS=/bin/true
                  "GIT_SSH_COMMAND=${GIT_SSH_COMMAND:-ssh -o BatchMode=yes -o StrictHostKeyChecking=accept-new -o ConnectTimeout=15}")
    if command -v timeout >/dev/null 2>&1; then
        timeout 300 "${pre[@]}" "$@"
    else
        "${pre[@]}" "$@"
    fi
}
