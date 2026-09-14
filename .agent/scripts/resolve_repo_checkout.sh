#!/bin/bash
# .agent/scripts/resolve_repo_checkout.sh
# Resolve a project repo to a usable on-disk checkout, without assuming that
# the layer tree (`layers/`) exists.
#
# Why: skills that audit a project repo (`audit-project`, and the
# `janitor-sweep` that chains it) located repos with
# `find layers/main/*/src/<repo>` — a hard dependency on a fully set-up layer
# tree. `layers/` is gitignored and absent in every worktree, fresh clone and
# container, so those callers silently had nothing to audit. This script gives
# them one resolution rule: use the layer checkout when there is one, otherwise
# shallow-clone the URL the workspace manifests already declare, at the version
# those manifests pin. Where the manifests themselves are missing — they live
# behind the same `configs/manifest` -> `layers/...` symlink — the manifest
# repo is cloned first, from the tracked `configs/project_bootstrap.url`
# pointer (see manifest_fallback.sh).
#
# Usage:
#   .agent/scripts/resolve_repo_checkout.sh <repo-name>
#
# Output (stdout, on success only):
#   <absolute-path><TAB><layer|clone>
#
# Nothing is ever printed to stdout on a failure path — a caller that reads
# stdout gets a path or nothing at all, never an empty string at exit 0.
#
# The mode field matters to callers: `clone` means there is no layer checkout,
# so layer-dependent checks (a `colcon test` run, "is it in the expected
# layer?") must report SKIPPED rather than OK.
#
# And the two modes carry different guarantees, which a report naming the mode
# is also naming:
#   clone  — the manifests decided it. The url is the one they declare, the
#            tree is at the `version:` they pin (verified after the fact for a
#            SHA pin), and a cached clone whose origin no longer matches is
#            re-cloned.
#   layer  — the operator's working tree, accepted AS IS. Its origin and its
#            checked-out ref are NOT compared against the manifests, so a
#            renamed or re-pointed repo, or one sitting at an old pin or on a
#            feature branch, is audited under the manifest name. That is
#            deliberate, not an oversight: a layer checkout is the code the
#            operator actually has, auditing anything else would be auditing
#            the wrong tree, and its remote legitimately differs in FORM from
#            the manifest url (ssh vs https for the same repo) while its branch
#            legitimately differs from the pin during any feature work — so a
#            comparison here would produce false findings on healthy
#            workspaces. What the caller owes the reader is the mode, so the
#            reader knows which of these two the finding came from.
#
# Exit codes — every failure is named, and none of them is an empty success
# (#609's false-green lesson):
#   0  resolved
#   2  usage error (no argument, or a repo name that is not a single path
#      segment)
#   3  no repo manifest configured AND none derivable from the tracked
#      `configs/project_bootstrap.url` pointer — so zero repos are
#      enumerable. list_overlay_repos.py prints an empty list at exit 0 in
#      this state, which would otherwise read as "repo not found". Note this
#      is NOT a worktree: the script resolves against the main root, where
#      configs/manifest does live. A host with neither (a container, a fresh
#      clone) reaches this only when the bootstrap pointer is missing or in a
#      form the fallback cannot derive a git url from — when it IS derivable,
#      the manifest repo is shallow-cloned into the scratch cache and the
#      repos enumerate from there (see manifest_fallback.sh). A manifest
#      clone that was attempted and FAILED is exit 5, never exit 3.
#   4  repo not listed in any manifest that WAS read
#   5  could not produce a checkout: a clone or refresh failed (including the
#      manifest repo's own clone), a layer checkout exists but cannot be read,
#      or the local scaffolding the clone needs could not be set up (temp dir,
#      cache directory, the per-repo lock, or the sibling `redact.sh` this
#      script's diagnostics are routed through)
#   6  manifest unreadable, or the repo's manifest entry is malformed (no
#      `url:` key, a URL in no recognised form, or a `version:` that is
#      neither a full commit SHA nor a ref-safe branch/tag name) — distinct
#      from 4, which means the manifests were fine and simply do not name
#      this repo
#   7  the repo is declared in more than one manifest with a different url OR a
#      different `version:` pin — ambiguous, so which one to audit is the
#      operator's call, not a silent first-match. The pin counts because two
#      manifests agreeing on the url and disagreeing on the version describe
#      two different trees, and picking one of them silently audits code the
#      operator did not name
#
# Clones land in <main-workspace-root>/.agent/scratchpad/janitor-repos/<repo>
# (gitignored). The cache is anchored at the MAIN workspace root, not the
# current worktree, so every worktree on a host shares one cache — and is
# therefore guarded by a per-repo `flock` so two concurrent runs cannot race
# on the same tree (see "Concurrency" below).
#
# What the lock does and does not buy: it serialises the CLONE/REFRESH of one
# repo, which is where `rm -rf` + `clone` + `reset --hard` run. It is released
# when this script exits, so the caller reads $TARGET UNLOCKED — a concurrent
# run that re-clones the same repo can still delete a tree an audit is reading.
# Closing that would take a per-run directory (or a lock the caller holds for
# the audit's duration), which is a change to the caller's contract, not to
# this script; until then, concurrent audits of the SAME repo on one host are
# not safe, and different repos are.
#
# Run from a worktree, this is the worktree's copy of the script and it sources
# the worktree's copy of `manifest_fallback.sh` — but it executes the MAIN
# checkout's `list_overlay_repos.py`, because that is where `configs/` lives.
# Deliberate, and worth knowing when testing a change: a worktree that edits
# both this script and `list_overlay_repos.py` only exercises the first half
# until the branch is merged.

set -uo pipefail

# Executed, not sourced.
if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
    echo "resolve_repo_checkout.sh must be executed, not sourced" >&2
    return 2 2>/dev/null || exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# .agent/scripts/ -> .agent/ -> workspace root
TREE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Failure messages name the url so the operator can see which remote failed,
# and the caller funnels this stderr into a report that must stay free of
# credentials. A manifest url is not supposed to carry userinfo, but "supposed
# to" is not a guarantee — `redact_url` strips any `user[:password]@` from a
# url, and `redact_text` does the same anywhere inside a captured command
# output. Shared with manifest_fallback.sh (which prints into the same report)
# rather than duplicated, and its absence is a failure: a run that carried on
# without it would print the very thing the helper exists to remove.
if [[ ! -f "$SCRIPT_DIR/redact.sh" ]]; then
    echo "resolve_repo_checkout.sh: cannot find $SCRIPT_DIR/redact.sh — refusing to run, since its failure messages would print urls and captured git output unredacted" >&2
    exit 5
fi
# shellcheck source=redact.sh
source "$SCRIPT_DIR/redact.sh"

# One funnel for every diagnostic, so a new message cannot forget to redact.
# Path rewriting joins in once $MAIN_ROOT is known (below); until then
# redact_text still strips credentials out of anything printed.
say() { echo "resolve_repo_checkout.sh: $(redact_text "$1")" >&2; }

usage() {
    echo "usage: resolve_repo_checkout.sh <repo-name>" >&2
    echo "  <repo-name> is a single path segment, as it appears in a .repos manifest" >&2
}

if [[ $# -ne 1 ]]; then
    usage
    exit 2
fi

# The name is interpolated into a cache path that is handed to `rm -rf`, and
# vcstool manifest keys are *paths* while `--repos` is operator input. Constrain
# it to one path segment before it can mean anything else.
REPO_NAME="$1"
if [[ ! "$REPO_NAME" =~ ^[A-Za-z0-9_][A-Za-z0-9._+-]*$ ]] || [[ "$REPO_NAME" == "." || "$REPO_NAME" == ".." ]]; then
    say "'$REPO_NAME' is not a valid repo name (one path segment: letters, digits, . _ + -, not starting with '.' or '-')"
    usage
    exit 2
fi

TMP_DIR=$(mktemp -d "${TMPDIR:-/tmp}/resolve_repo_checkout.XXXXXX") || {
    say "could not create a temp dir"
    exit 5
}
cleanup() { rm -rf "$TMP_DIR"; }
trap cleanup EXIT


# Unattended network git, precisely:
#   - GIT_TERMINAL_PROMPT=0 / GIT_ASKPASS=/bin/true stop git's own username and
#     password prompts. They do NOT cover ssh, which reads its host-key and
#     key-passphrase prompts straight from /dev/tty — and the workspace
#     manifests carry `git@github.com:` urls, so ssh is a live path here.
#   - GIT_SSH_COMMAND adds ssh's own equivalents: BatchMode=yes refuses every
#     interactive prompt (passphrase, password, keyboard-interactive) instead
#     of asking, and StrictHostKeyChecking=accept-new accepts a first-seen host
#     key without asking while still REFUSING a changed one. ConnectTimeout
#     bounds the TCP handshake.
#   - `timeout` bounds the whole operation in wall-clock time, which is the
#     only thing that covers a server that accepts the connection and then
#     stalls. Set GIT_SSH_COMMAND in the environment to override the ssh
#     options (the value below is a default, not a policy).
# Together these mean a network operation FAILS rather than hanging. Where
# `timeout` is missing the run is still non-interactive, but only git's and
# ssh's own timeouts bound it — so the script says so rather than implying a
# guarantee it cannot make.
GIT_NET=(env GIT_TERMINAL_PROMPT=0 GIT_ASKPASS=/bin/true
         "GIT_SSH_COMMAND=${GIT_SSH_COMMAND:-ssh -o BatchMode=yes -o StrictHostKeyChecking=accept-new -o ConnectTimeout=15}")
if command -v timeout >/dev/null 2>&1; then
    GIT_NET=(timeout 300 "${GIT_NET[@]}")
else
    say "timeout(1) not available — network git operations are non-interactive but not wall-clock bounded"
fi

# Main workspace root: from a worktree, the common git dir points at the main
# checkout's .git. Falling back to the tree root covers a non-git sandbox and
# a plain (non-worktree) clone alike.
MAIN_ROOT="$TREE_ROOT"
if common_dir=$(git -C "$TREE_ROOT" rev-parse --path-format=absolute --git-common-dir 2>/dev/null); then
    candidate="$(dirname "$common_dir")"
    [[ -d "$candidate" ]] && MAIN_ROOT="$candidate"
fi

# Everything this script prints is a reason string the caller funnels into a
# sweep report — one the sweep's own contract requires to be free of host
# identity — so absolute paths are rewritten to placeholders and git's captured
# output is scrubbed of credentials. git echoes the FULL remote url, userinfo
# included, in many clone and fetch errors, so a message that redacts $REPO_URL
# and then interpolates $out verbatim leaks through its own second half. Most
# specific prefix first: the worktree root sits UNDER the main root.
REDACT_PATH_PREFIXES=()
[[ "$TREE_ROOT" != "$MAIN_ROOT" ]] && REDACT_PATH_PREFIXES+=("$TREE_ROOT=<worktree>")
REDACT_PATH_PREFIXES+=("$MAIN_ROOT=<workspace>")
[[ -n "${HOME:-}" && "$HOME" != "/" && "$HOME" != "$MAIN_ROOT" ]] && REDACT_PATH_PREFIXES+=("$HOME=~")

# --- (a) an existing layer checkout wins -------------------------------------
# A bare `-d` test is not enough: `vcs import` leaves an empty `src/<repo>`
# directory behind when it fails partway, and resolving that as mode `layer`
# hands the caller a directory with nothing in it to audit — an empty success
# wearing a valid path. An empty candidate falls through to the clone path
# instead, with a note, so the caller still gets a real checkout.
for candidate in "$MAIN_ROOT"/layers/main/*/src/"$REPO_NAME"; do
    [[ -d "$candidate" ]] || continue
    # Unreadable is not empty. `ls -A` returns nothing for both, so testing
    # emptiness first would quietly downgrade a permissions fault to "fall back
    # to a clone" and audit a different tree than the operator has on disk.
    if [[ ! -r "$candidate" || ! -x "$candidate" ]]; then
        say "the layer checkout at $candidate is not readable (permissions?)"
        exit 5
    fi
    if [[ -z "$(ls -A "$candidate" 2>/dev/null)" ]]; then
        say "$candidate exists but is empty (partial 'vcs import'?) — falling back to a clone"
        continue
    fi
    # `cd` can still fail on an unreadable directory; its status must reach the
    # exit code rather than being swallowed by a command substitution inside
    # printf's format argument.
    if ! abs_path=$(cd "$candidate" && pwd); then
        say "cannot enter the layer checkout at $candidate (permissions?)"
        exit 5
    fi
    printf '%s\t%s\n' "$abs_path" "layer"
    exit 0
done

# --- (b) otherwise resolve the URL from the workspace manifests ---------------
LIST_SCRIPT="$MAIN_ROOT/.agent/scripts/list_overlay_repos.py"
if [[ ! -f "$LIST_SCRIPT" ]]; then
    say "cannot find $LIST_SCRIPT"
    exit 6
fi

# `configs/manifest` is a symlink INTO the layer tree, so a host with no
# `layers/` has no manifests at all — the manifest is the first thing a
# resolver that "clones what it needs" has to be able to clone. The shared
# helper derives the manifest repo from the tracked
# `configs/project_bootstrap.url` pointer, clones it into the scratch cache,
# and hands back a directory to read `.repos` from; it prints nothing (rc 0)
# when the workspace has its own manifest, which is the normal case.
EXTRA_CONFIG=()
if [[ -f "$SCRIPT_DIR/manifest_fallback.sh" ]]; then
    # shellcheck source=manifest_fallback.sh
    source "$SCRIPT_DIR/manifest_fallback.sh"
    if fallback_dir=$(manifest_config_dir "$MAIN_ROOT"); then
        [[ -n "$fallback_dir" ]] && EXTRA_CONFIG=(--config-dir "$fallback_dir")
    else
        fallback_rc=$?
        # A manifest clone that FAILED is not "no manifest configured": the
        # pointer named one and we could not get it. That is exit 5, with the
        # helper's reason already on stderr. rc 6 — a cached manifest clone
        # that could not be REFRESHED — is the same answer here: this script's
        # exit 5 already covers "a clone or refresh failed", and resolving a
        # repo out of a manifest we could not verify is the false green the
        # exit vocabulary exists to prevent.
        [[ "$fallback_rc" -eq 5 || "$fallback_rc" -eq 6 ]] && exit 5
        # rc 3 — no manifest and no usable pointer. Fall through: if nothing is
        # enumerable after that, the EMPTY verdict below reports exit 3.
    fi
else
    say "cannot find $SCRIPT_DIR/manifest_fallback.sh — no manifest fallback for a host without layers/"
fi

# stdout and stderr are kept apart: folding them together turns any benign
# stderr noise on a *successful* run into unparseable JSON, i.e. a readable
# manifest reported as exit 6.
if ! repos_json=$(python3 "$LIST_SCRIPT" --format json "${EXTRA_CONFIG[@]}" 2>"$TMP_DIR/list.err"); then
    say "could not read the repo manifests: $(tr '\n' ' ' < "$TMP_DIR/list.err")"
    exit 6
fi

# One lookup, one verdict. Zero repos enumerated is NOT "repo not found" — it
# means nothing is configured to look through; a repo declared twice with two
# different URLs is not a first-match; and an entry with no `url:` is a broken
# manifest, not an absent repo.
lookup=$(printf '%s' "$repos_json" | python3 -c '
import json, sys

name = sys.argv[1]
try:
    repos = json.load(sys.stdin)
except (ValueError, TypeError) as exc:
    print("BADJSON\t%s" % exc)
    raise SystemExit(0)
if not isinstance(repos, list):
    print("BADJSON\tmanifest listing is not a list")
    raise SystemExit(0)
if not repos:
    print("EMPTY\t0")
    raise SystemExit(0)

matches = [r for r in repos if r.get("name") == name]
if not matches:
    print("NOTFOUND\t%d" % len(repos))
    raise SystemExit(0)

# The url alone is not the identity of a checkout: two manifests naming the
# same repo at the same url but DIFFERENT `version:` pins describe two
# different trees, and taking matches[0] in sorted-manifest order would clone
# whichever one sorted first and audit the wrong branch or commit — silently,
# where the conflicting-url case is loud. Same asymmetry, same answer.
pins = {((r.get("url") or ""), (r.get("version") or "")) for r in matches}
if len(pins) > 1:
    print("AMBIGUOUS\t%s" % " | ".join(
        "%s at '%s' (%s)" % (r.get("url") or "<no url>",
                             r.get("version") or "<no version>",
                             r.get("source_file") or "?") for r in matches))
    raise SystemExit(0)

url = matches[0].get("url") or ""
if not url:
    print("NOURL\t%s" % (matches[0].get("source_file") or "?"))
    raise SystemExit(0)
print("OK\t%s\t%s" % (url, matches[0].get("version") or ""))
' "$REPO_NAME" 2>"$TMP_DIR/lookup.err")

if [[ -z "$lookup" ]]; then
    say "could not interpret the manifest listing: $(tr '\n' ' ' < "$TMP_DIR/lookup.err")"
    exit 6
fi

verdict=${lookup%%$'\t'*}
detail=${lookup#*$'\t'}

case "$verdict" in
    BADJSON)
        say "manifest listing was not valid JSON: $detail"
        exit 6
        ;;
    EMPTY)
        say "no repo manifest configured under $MAIN_ROOT/configs, and none could be derived from configs/project_bootstrap.url — run 'make setup-all'"
        exit 3
        ;;
    NOTFOUND)
        say "'$REPO_NAME' is not listed in any of the $detail configured repos (underlay.repos is excluded from this search by list_overlay_repos.py — a repo declared only there is expected to land here)"
        exit 4
        ;;
    AMBIGUOUS)
        say "'$REPO_NAME' is declared with conflicting urls or version pins: $detail — resolve the manifests, do not guess"
        exit 7
        ;;
    NOURL)
        say "'$REPO_NAME' is listed in $detail with no 'url:' key — the manifest entry is malformed"
        exit 6
        ;;
    OK) ;;
    *)
        say "unexpected manifest lookup result '$verdict'"
        exit 6
        ;;
esac

REPO_URL=${detail%%$'\t'*}
REPO_VERSION=${detail#*$'\t'}
[[ "$REPO_VERSION" == "$detail" ]] && REPO_VERSION=""

# The URL reaches `git clone`; require a recognised form so it can never be
# read as an option or a stray local path.
if [[ ! "$REPO_URL" =~ ^(https?|ssh|git|file)://[^[:space:]]+$ ]] \
   && [[ ! "$REPO_URL" =~ ^[A-Za-z0-9._-]+@[A-Za-z0-9._-]+:[^[:space:]]+$ ]] \
   && [[ ! "$REPO_URL" =~ ^/[^[:space:]]*$ ]]; then
    say "'$REPO_NAME' has an unrecognised url '$(redact_url "$REPO_URL")' — the manifest entry is malformed"
    exit 6
fi

# `version:` reaches `git clone --branch` and `git fetch <refspec>` and is just
# as much manifest input as the url — an unvalidated one starting with `-` is
# read as an OPTION (`--upload-pack=<script>` executes it), and because a fetch
# that consumed its argument as a flag still succeeds, the run would exit 0
# reporting mode `clone` with the pin silently unhonoured. Require a value that
# is either a full commit SHA or a ref-safe branch/tag name.
if [[ -n "$REPO_VERSION" ]]; then
    version_ok=0
    if [[ "$REPO_VERSION" =~ ^[0-9a-fA-F]{40}$ ]]; then
        version_ok=1
    elif [[ "$REPO_VERSION" != -* ]] \
         && [[ "$REPO_VERSION" != *..* ]] \
         && [[ "$REPO_VERSION" =~ ^[A-Za-z0-9._/+-]+$ ]] \
         && git check-ref-format --branch "$REPO_VERSION" >/dev/null 2>&1; then
        # The character-class test is deliberately narrower than
        # check-ref-format: it also excludes whitespace, control characters and
        # the revision-expression punctuation (`{}`, `~`, `^`, `@`) that
        # check-ref-format's --branch form would resolve against the *current*
        # repo rather than treat as a literal name.
        version_ok=1
    fi
    if [[ "$version_ok" -ne 1 ]]; then
        say "'$REPO_NAME' pins version '$REPO_VERSION', which is neither a full commit SHA nor a ref-safe branch/tag name — the manifest entry is malformed"
        exit 6
    fi
fi

# --- (c) shallow clone / refresh ---------------------------------------------
# --depth 1 only: a --filter=blob:none clone that still checks out a working
# tree refetches every blob immediately, so the filter would buy nothing here.
CACHE_DIR="$MAIN_ROOT/.agent/scratchpad/janitor-repos"
TARGET="$CACHE_DIR/$REPO_NAME"

if ! mkdir -p "$CACHE_DIR" 2>/dev/null; then
    say "cannot create clone cache at $CACHE_DIR"
    exit 5
fi

# Concurrency: the cache is shared by every worktree on the host, and the
# refresh path runs `rm -rf` + `clone` + `reset --hard` over it. Two runs
# racing there can delete a tree the other is reading. A per-repo advisory
# lock with a bounded wait, held for the whole clone/refresh, serialises them.
# `flock` is
# util-linux and present on every host this workspace targets; where it is
# absent the script proceeds unlocked rather than failing, and says so.
if command -v flock >/dev/null 2>&1; then
    # Bounded wait: an untimed `flock` lets one wedged run (a stalled clone, a
    # process stopped under a debugger) block every other run on the host
    # forever, with no output to say why. 300s is longer than the network
    # timeout above, so a healthy holder always finishes first; past that the
    # honest answer is a named failure, not an indefinite hang.
    # RESOLVE_LOCK_TIMEOUT overrides the wait, in seconds — the tests use it to
    # exercise the timeout path without waiting five minutes for it.
    lock_wait="${RESOLVE_LOCK_TIMEOUT:-300}"
    if exec 9>"$CACHE_DIR/.$REPO_NAME.lock" && flock -w "$lock_wait" 9; then
        :
    else
        say "could not lock the clone cache for $REPO_NAME within ${lock_wait}s — another run may be wedged on $CACHE_DIR/.$REPO_NAME.lock"
        exit 5
    fi
else
    say "flock not available — the clone cache is unlocked for this run"
fi

# The pinned ref, as a fetch refspec. Empty `version:` means "whatever the
# remote's HEAD is", which `HEAD` expresses exactly.
FETCH_REF="${REPO_VERSION:-HEAD}"

clone_fresh() {
    rm -rf "$TARGET"
    local out
    if [[ -n "$REPO_VERSION" ]]; then
        # Branch or tag: one shot.
        if out=$("${GIT_NET[@]}" git clone --depth 1 --branch "$REPO_VERSION" -- "$REPO_URL" "$TARGET" 2>&1); then
            return 0
        fi
        # A `version:` may also be a commit SHA, which --branch cannot take.
        # `--` before the refspec keeps a ref from being read as an option, and
        # the SHA is re-checked against HEAD afterwards: a pin that did not
        # take must never reach the caller as a successful resolve.
        rm -rf "$TARGET"
        if out=$("${GIT_NET[@]}" git clone --depth 1 -- "$REPO_URL" "$TARGET" 2>&1) \
           && out=$("${GIT_NET[@]}" git -C "$TARGET" fetch --depth 1 origin -- "$REPO_VERSION" 2>&1) \
           && out=$(git -C "$TARGET" checkout --detach FETCH_HEAD 2>&1) \
           && out=$(verify_pin); then
            return 0
        fi
        say "clone of $(redact_url "$REPO_URL") at version '$REPO_VERSION' failed: $out"
        rm -rf "$TARGET"
        return 1
    fi
    if out=$("${GIT_NET[@]}" git clone --depth 1 -- "$REPO_URL" "$TARGET" 2>&1); then
        return 0
    fi
    say "clone of $(redact_url "$REPO_URL") failed: $out"
    rm -rf "$TARGET"
    return 1
}

# A full-SHA pin is verifiable after the fact, and verifying it is what closes
# the gap between "the fetch command returned 0" and "the tree is at the pinned
# commit". A branch/tag pin is honoured by `clone --branch` itself.
verify_pin() {
    [[ "$REPO_VERSION" =~ ^[0-9a-fA-F]{40}$ ]] || return 0
    local head
    head=$(git -C "$TARGET" rev-parse HEAD 2>/dev/null)
    if [[ "${head,,}" != "${REPO_VERSION,,}" ]]; then
        echo "checked out '${head:-<none>}', not the pinned '$REPO_VERSION'"
        return 1
    fi
    return 0
}

if [[ -d "$TARGET/.git" ]]; then
    # The cache is keyed on repo name alone, so a cached tree is only reusable
    # once its origin is confirmed to be the URL this manifest declares —
    # otherwise a renamed or re-homed repo would be audited from the old
    # remote's code under the new remote's name.
    cached_url=$(git -C "$TARGET" remote get-url origin 2>/dev/null)
    if [[ "$cached_url" != "$REPO_URL" ]]; then
        say "cached clone of $REPO_NAME points at '$(redact_url "${cached_url:-<none>}")', manifest says '$(redact_url "$REPO_URL")' — re-cloning"
        clone_fresh || exit 5
    elif ! refresh_out=$("${GIT_NET[@]}" git -C "$TARGET" fetch --depth 1 origin -- "$FETCH_REF" 2>&1) \
         || ! refresh_out=$(git -C "$TARGET" reset --hard FETCH_HEAD 2>&1) \
         || ! refresh_out=$(verify_pin); then
        # A shallow fetch of a pinned SHA is not served by every host; a
        # re-clone is the honest recovery, and only its failure is exit 5.
        say "could not refresh the existing clone at $TARGET ($refresh_out) — re-cloning"
        clone_fresh || exit 5
    fi
else
    clone_fresh || exit 5
fi

printf '%s\t%s\n' "$TARGET" "clone"
exit 0
