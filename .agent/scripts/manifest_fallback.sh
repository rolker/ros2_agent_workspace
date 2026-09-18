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
# cache, read the now-local `bootstrap.yaml` the pointer names — the same three
# keys `setup_layers.sh` reads — and print the config directory to read `.repos`
# files from. Only `config_path:` is HONOURED (it decides where the `.repos`
# are read from); `git_url:` and `branch:` are hard-failing cross-checks
# against what the clone already used, since a disagreement there cannot be
# honoured by rewriting a variable — it means this clone is the wrong repo. Callers
# pass that to `list_overlay_repos.py --config-dir <dir>`, which ADDS it to the
# normal search path rather than replacing it — a workspace that has a real
# `configs/manifest` never takes this path at all.
#
# Usage (sourced — this file defines functions and runs nothing on its own, and
# it sources `redact.sh` beside it, which must be present). The `source` itself
# can fail (exit 5, below), so its status is checked rather than assumed:
#   source /path/to/.agent/scripts/manifest_fallback.sh || exit 5   # see below
#   extra_config_dir=$(manifest_config_dir "$ROOT"); rc=$?
#   case "$rc" in
#       0) ;;  # usable: EMPTY string means "the workspace's own manifest is
#              # present, no override needed"; non-empty is a --config-dir value
#       3) ;;  # no manifest AND no usable bootstrap pointer — nothing to read
#       5) ;;  # the manifest repo could not be USED (reason on stderr): the
#              # clone or the cache/lock failed, the derived git base was
#              # refused, or the pointer and the manifest repo disagree — the
#              # cloned `bootstrap.yaml` names a different repo or branch, its
#              # `config_path` is unsafe, or there is no `<config_path>/repos`.
#              # Also returned by the SOURCE of this file when `redact.sh`
#              # beside it is missing or will not load — the same "a required
#              # local piece is unusable" answer resolve_repo_checkout.sh
#              # gives that condition, and deliberately NOT 2, which means
#              # "you executed this file instead of sourcing it": one is a
#              # caller bug, the other an unusable install, and a caller that
#              # reads 2 for both reports the wrong remedy
#       6) ;;  # a CACHED manifest clone could not be refreshed (reason on
#              # stderr). Its own outcome, not a warning on top of rc 0: the
#              # cached copy may be arbitrarily stale, and a caller that
#              # enumerated from it while reporting success would be claiming
#              # a state it could not verify. Callers report it as
#              # FAILED(manifest refresh: <reason>)
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

# Exit codes at SOURCE time (manifest_config_dir's own are in the table above):
#   2  executed rather than sourced — a caller bug
#   5  `redact.sh` beside this file is missing or would not load, so the
#      diagnostics this helper prints into a report could not be redacted
#
# Sourced, not executed — the mirror image of resolve_repo_checkout.sh's guard.
# Run directly, this file would define two functions and exit 0 having done
# nothing at all: a silent success, from a script whose whole job is to refuse
# to report success it did not earn.
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "manifest_fallback.sh must be sourced, not executed: source ${BASH_SOURCE[0]} && manifest_config_dir <workspace-root>" >&2
    exit 2
fi

# Credentials must never reach the sweep report, and this helper prints urls
# that came from the ambient environment (`WORKSPACE_MANIFEST_GIT_BASE`) into
# exactly that report. The redaction is shared with resolve_repo_checkout.sh
# rather than duplicated — two copies of the regex drift, and the one that
# drifts is the one that leaks. A caller that already sourced redact.sh (the
# resolver does) keeps its definitions; a missing helper is refused rather than
# printed around, because the failure mode of "carry on" is a leak.
_manifest_fallback_redact_rc=0
if ! declare -F redact_url >/dev/null 2>&1 || ! declare -F redact_text >/dev/null 2>&1; then
    _manifest_fallback_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    if [[ -f "$_manifest_fallback_dir/redact.sh" ]]; then
        # A `source` that FAILED (a truncated or edited-in-place redact.sh) can
        # still leave a half-defined shell: the rc is checked rather than
        # inferred from whether the two names happen to exist.
        # shellcheck source=redact.sh
        source "$_manifest_fallback_dir/redact.sh" || _manifest_fallback_redact_rc=$?
    else
        _manifest_fallback_redact_rc=1
    fi
    unset _manifest_fallback_dir
fi
if [[ "$_manifest_fallback_redact_rc" -ne 0 ]] \
   || ! declare -F redact_url >/dev/null 2>&1 || ! declare -F redact_text >/dev/null 2>&1; then
    unset _manifest_fallback_redact_rc
    echo "manifest_fallback.sh: cannot load redact.sh beside it — refusing to source a helper that would print urls unredacted (exit 5)" >&2
    return 5
fi
unset _manifest_fallback_redact_rc

# One funnel for every diagnostic, so a new message cannot forget to redact.
# Paths are rewritten only when the CALLER has set $REDACT_PATH_PREFIXES
# (resolve_repo_checkout.sh does, from its own $MAIN_ROOT/$HOME); sourced from
# a skill that has not, the messages keep their absolute paths and lose only
# credentials.
_manifest_fallback_say() { echo "manifest_fallback: $(redact_text "$1")" >&2; }

# manifest_config_dir <workspace_root>
# See the exit-code table above.
manifest_config_dir() {
    local root="${1:-$PWD}"
    local pointer_file="$root/configs/project_bootstrap.url"
    local bootstrap_url owner repo branch config_path git_url cache clone_dir
    local lock_open

    # The normal case: the workspace has its manifest on disk. Print nothing.
    #
    # This test must recognise EVERY layout `get_overlay_repos` reads, or a
    # workable on-disk manifest is bypassed for a network clone whose failure
    # then hard-fails the caller (resolve_repo_checkout.sh turns rc 5 into
    # exit 5) before the manifest right there is ever opened — a false RED over
    # a working state. `lib/workspace.py` searches BOTH `configs/manifest/repos`
    # (the symlink into the layer tree) and `configs/` itself, so both count.
    # `underlay.repos` does not: it is on that search's ignore list, so a
    # workspace holding only that one still enumerates zero overlay repos and
    # does need the fallback.
    if [[ -d "$root/configs/manifest/repos" ]]; then
        return 0
    fi
    local repos_file
    for repos_file in "$root"/configs/*.repos; do
        [[ -e "$repos_file" ]] || continue
        [[ "$(basename "$repos_file")" == "underlay.repos" ]] && continue
        return 0
    done

    bootstrap_url="${BOOTSTRAP_URL:-}"
    if [[ -z "$bootstrap_url" && -f "$pointer_file" ]]; then
        bootstrap_url=$(tr -d '[:space:]' < "$pointer_file")
    fi
    if [[ -z "$bootstrap_url" ]]; then
        _manifest_fallback_say "no configs/manifest and no configs/project_bootstrap.url under $root — nothing to enumerate repos from"
        return 3
    fi

    # Only the raw.githubusercontent form is derivable without fetching the
    # bootstrap.yaml itself (which would need the network before we know
    # whether we can reach it at all, and would put a YAML parser in the
    # failure path). Anything else is reported as unsupported rather than
    # guessed at.
    if [[ ! "$bootstrap_url" =~ ^https://raw\.githubusercontent\.com/([^/]+)/([^/]+)/([^/]+)/(.+)/bootstrap\.yaml$ ]]; then
        _manifest_fallback_say "'$(redact_url "$bootstrap_url")' is not a raw.githubusercontent.com/<owner>/<repo>/<branch>/<path>/bootstrap.yaml url — this fallback cannot derive a git url from it; run 'make setup-all' on a host that can"
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
        _manifest_fallback_say "'$(redact_url "$bootstrap_url")' does not parse into a safe <owner>/<repo>/<branch>/<path> — refusing to guess"
        return 3
    fi

    # The base reaches `git clone`, and it is the root of the whole trust chain
    # (manifest repo -> the repo list -> every repo the sweep clones). It comes
    # from the ambient environment, so require a recognised url form — the same
    # shape resolve_repo_checkout.sh requires of a manifest url — so it can
    # never be read as a git OPTION or as a stray local path.
    local git_base="${WORKSPACE_MANIFEST_GIT_BASE:-https://github.com}"
    if [[ ! "$git_base" =~ ^(https?|ssh|git|file)://[^[:space:]]+$ ]] || [[ "$git_base" == *..* ]]; then
        _manifest_fallback_say "WORKSPACE_MANIFEST_GIT_BASE '$(redact_url "$git_base")' is not a <scheme>://<host>/<path> url — refusing to hand it to git clone"
        return 5
    fi
    git_url="${git_base%/}/$owner/$repo.git"
    cache="$root/.agent/scratchpad/manifest-repo"
    clone_dir="$cache/$repo"

    if ! mkdir -p "$cache" 2>/dev/null; then
        _manifest_fallback_say "cannot create the manifest cache at $cache"
        return 5
    fi

    # Same shared-cache hazard as the repo clone cache, same remedy: a per-repo
    # advisory lock with a bounded wait, held for the clone/refresh. Absent
    # flock the run proceeds unlocked and says so.
    if command -v flock >/dev/null 2>&1; then
        # A failed `exec 8>...` (e.g. an unwritable cache directory) prints
        # bash's own error text straight to the real stderr, naming the
        # absolute lock path — bypassing the `_manifest_fallback_say`
        # redaction funnel entirely. Same fix as resolve_repo_checkout.sh's
        # lock open: save the real stderr first (fd 6 here — distinct from
        # resolve_repo_checkout.sh's fd 7, in case the two are ever nested in
        # one process) and point fd 2 at /dev/null for the open attempt, then
        # restore before deciding how to report. A trailing `2>/dev/null` on
        # the `exec` line itself would be too late — bash applies
        # redirections left to right, so the open (and its error) happens
        # before a later redirection on the same line takes effect.
        exec 6>&2
        # Signal window: between the line above and the restore below, fd 2 is
        # /dev/null. Only `trap cleanup EXIT` is installed, and an exiting shell
        # needs no restore; a future NON-exiting trap would lose its output here
        # and must restore fd 2 itself.
        exec 2>/dev/null
        if exec 8>"$cache/.$repo.lock"; then
            lock_open=1
        else
            lock_open=0
        fi
        exec 2>&6 6>&-
        if [[ "$lock_open" -ne 1 ]]; then
            _manifest_fallback_say "could not open the lock file for $repo (permissions?) — refusing to proceed unlocked"
            return 5
        fi
        if ! flock -w 120 8; then
            _manifest_fallback_say "could not lock the manifest cache for $repo within 120s"
            return 5
        fi
    else
        _manifest_fallback_say "flock not available — the manifest cache is unlocked for this run"
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
        #
        # The url is the only key needed: the refresh below fetches and hard
        # resets to the CURRENT pointer's `$branch` on every run, so a cached
        # clone made at a different branch of the same repo is corrected
        # rather than reused as-is.
        cached_url=$(git -C "$clone_dir" remote get-url origin 2>/dev/null 8>&-)
        if [[ "$cached_url" != "$git_url" ]]; then
            _manifest_fallback_say "cached manifest clone at $clone_dir points at '$(redact_url "${cached_url:-<none>}")', the bootstrap pointer derives '$(redact_url "$git_url")' — re-cloning"
            rm -rf "$clone_dir"
        fi
    fi

    if [[ -d "$clone_dir/.git" ]]; then
        # A refresh that failed is its OWN outcome, not a warning printed over
        # a success. The cached copy may be arbitrarily stale — the manifest
        # that decides which repos exist, at which versions — and every caller
        # captures stdout only, so returning 0 here handed back a config dir
        # with nothing in the return value to distinguish it from a refreshed
        # one. A sweep would then report "4 of 4 completed" over a repo list it
        # could not verify: exactly the report-level false green the callers'
        # status contract exists to prevent. Callers map 6 to
        # FAILED(manifest refresh: <reason>).
        if ! out=$(_manifest_fallback_git git -C "$clone_dir" fetch --depth 1 origin -- "$branch" 2>&1 8>&-) \
           || ! out=$(git -C "$clone_dir" reset --hard FETCH_HEAD 2>&1 8>&-); then
            _manifest_fallback_say "could not refresh the cached manifest repo at $clone_dir ($out) — the cached copy may be stale, so this is a failure, not a fallback"
            return 6
        fi
    else
        rm -rf "$clone_dir"
        if ! out=$(_manifest_fallback_git git clone --depth 1 --branch "$branch" -- "$git_url" "$clone_dir" 2>&1 8>&-); then
            _manifest_fallback_say "could not clone the manifest repo $(redact_url "$git_url") at '$branch': $out"
            rm -rf "$clone_dir"
            return 5
        fi
    fi

    # The raw pointer's path can only say WHERE bootstrap.yaml sits; the file
    # itself is authoritative about everything else, and `setup_layers.sh`
    # treats it that way (`git_url:`/`branch:`/`config_path:`, the last
    # defaulting to `config`). Deriving all three from the path instead meant a
    # bootstrap whose `config_path` differs from where bootstrap.yaml lives
    # looked for the repos in the wrong place and blamed the operator for a
    # "disagreement" that was really this derivation's limit. Now that the
    # clone is local, reading it costs no network and no YAML parser — the same
    # `grep | cut | awk` setup_layers.sh uses.
    local bootstrap_file="$clone_dir/$config_path/bootstrap.yaml"
    local declared_url declared_branch declared_config_path
    if [[ -f "$bootstrap_file" ]]; then
        declared_url=$(grep "^git_url:" "$bootstrap_file" | cut -d '#' -f 1 | awk '{print $2}')
        declared_branch=$(grep "^branch:" "$bootstrap_file" | cut -d '#' -f 1 | awk '{print $2}')
        declared_config_path=$(grep "^config_path:" "$bootstrap_file" | cut -d '#' -f 1 | awk '{print $2}')
        # setup_layers.sh defaults an absent config_path to `config` because
        # the url is all it has. Here the pointer already told us where
        # bootstrap.yaml sits, which is a better inference than a constant —
        # so an absent key keeps the derived path rather than overriding it
        # with `config` (identical on any layout where bootstrap.yaml is at
        # `config/`, which is every one in use).
        declared_config_path="${declared_config_path:-$config_path}"

        if [[ "$declared_config_path" == /* || "$declared_config_path" == *..* ]]; then
            _manifest_fallback_say "$bootstrap_file declares config_path '$declared_config_path', which is absolute or contains '..' — refusing to read repos from it"
            return 5
        fi
        # The url and branch are what the clone ALREADY used, so a mismatch
        # cannot be honoured by rewriting a variable — it means this clone is
        # the wrong repo or the wrong branch, and the operator needs to be told
        # which two values disagree rather than sent to look for a missing
        # directory. The HOST is exempt when WORKSPACE_MANIFEST_GIT_BASE is
        # set: redirecting the host is that variable's entire purpose (a
        # mirror, or the hermetic tests), so only the <owner>/<repo> it points
        # at has to agree. Note that this is also the one condition under which
        # the cross-check is NARROWER precisely because the trust root was
        # redirected — a known property of pointing the base elsewhere, not an
        # oversight.
        #
        # Both comparisons go through _manifest_fallback_url_key, so url FORM
        # never decides the answer. Comparing the strings raw (which is what
        # the unset branch used to do) hard-failed at exit 5 on every url form
        # `setup_layers.sh` accepts and this derivation does not emit —
        # scp-form `git@host:owner/repo.git`, a url with no `.git`, a trailing
        # `/` — reporting a workspace whose bootstrap agrees perfectly as "not
        # the one its own bootstrap names".
        if [[ -n "$declared_branch" && "$declared_branch" != "$branch" ]]; then
            _manifest_fallback_say "the bootstrap pointer names branch '$branch', but $config_path/bootstrap.yaml in that repo declares branch '$declared_branch' — the pointer is stale, or it points into the wrong branch"
            return 5
        fi
        if [[ -n "$declared_url" ]]; then
            local want have
            want=$(_manifest_fallback_url_key "$declared_url")
            have=$(_manifest_fallback_url_key "$git_url")
            if [[ "$want" != "$have" ]]; then
                _manifest_fallback_say "the bootstrap pointer derives '$(redact_url "$git_url")', but $config_path/bootstrap.yaml in that repo declares git_url '$(redact_url "$declared_url")' — the manifest repo this workspace points at is not the one its own bootstrap names"
                return 5
            fi
        fi
        config_path="$declared_config_path"
    fi

    if [[ ! -d "$clone_dir/$config_path/repos" ]]; then
        if [[ -f "$bootstrap_file" ]]; then
            _manifest_fallback_say "$(redact_url "$git_url") ($branch) has no '$config_path/repos' directory, the config_path its own bootstrap.yaml declares — the manifest repo's layout and its bootstrap disagree"
        else
            _manifest_fallback_say "$(redact_url "$git_url") ($branch) has no '$config_path/repos' directory, and no '$config_path/bootstrap.yaml' to declare a different config_path — the bootstrap pointer and the manifest repo disagree"
        fi
        return 5
    fi

    printf '%s\n' "$clone_dir/$config_path/repos"
    return 0
}

# The comparison key for two git urls that should name the same manifest repo.
# Normalised through _manifest_fallback_repo_path so url FORM never decides the
# answer. The HOST is part of the key unless WORKSPACE_MANIFEST_GIT_BASE is set
# — redirecting the host is that variable's entire purpose, and comparing hosts
# then would refuse every mirror it exists to allow.
_manifest_fallback_url_key() {
    local path host
    path=$(_manifest_fallback_repo_path "$1")
    if [[ -n "${WORKSPACE_MANIFEST_GIT_BASE:-}" ]]; then
        printf '%s' "$path"
    else
        host=$(_manifest_fallback_url_host "$1")
        printf '%s/%s' "$host" "$path"
    fi
}

# The host of a git url, lowercased, with any `user[:pass]@` userinfo and any
# `:port` removed. Handles `scheme://[userinfo@]host[:port]/...` and the
# scp-style `[user@]host:owner/repo`; anything else has no host and answers
# empty, which compares equal only to another url with no host.
_manifest_fallback_url_host() {
    local u="$1" hostport=""
    if [[ "$u" =~ ^[A-Za-z][A-Za-z0-9+.-]*://([^/]*)(/|$) ]]; then
        hostport="${BASH_REMATCH[1]}"
    elif [[ "$u" == *:* ]]; then
        hostport="${u%%:*}"
    fi
    hostport="${hostport##*@}"
    hostport="${hostport%%:*}"
    printf '%s' "${hostport,,}"
}

# The <owner>/<repo> tail of a git url, for comparing two urls that may name
# different HOSTS legitimately (WORKSPACE_MANIFEST_GIT_BASE redirects the host
# on purpose). Handles both `scheme://host/owner/repo[.git]` and the scp-style
# `git@host:owner/repo[.git]`.
_manifest_fallback_repo_path() {
    local u="${1%/}"
    u="${u%.git}"
    u="${u//:/\/}"
    local repo="${u##*/}"
    u="${u%/*}"
    local owner="${u##*/}"
    printf '%s/%s' "$owner" "$repo"
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
