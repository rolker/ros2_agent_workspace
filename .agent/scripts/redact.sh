#!/bin/bash
# .agent/scripts/redact.sh
# Strip credentials and local-path identity out of diagnostic strings.
#
# Why this is shared rather than a private helper: the janitor rotation funnels
# every failure reason it prints into a sweep report that is written to disk and
# handed around, and the manifests it reads carry urls from the ambient
# environment (`WORKSPACE_MANIFEST_GIT_BASE`) and from repo manifests that are
# *not supposed to* carry userinfo — "not supposed to" being the operative
# phrase. `resolve_repo_checkout.sh` and `manifest_fallback.sh` both print urls
# and both feed that report, so they must redact identically; two copies of a
# regex drift, and the one that drifts is the one that leaks.
#
# Usage (sourced — this file defines functions and runs nothing on its own):
#   source /path/to/.agent/scripts/redact.sh
#   echo "clone of $(redact_url "$url") failed: $(redact_text "$git_output")" >&2
#
# Optional, for redact_text only:
#   REDACT_PATH_PREFIXES  an array of `<prefix>=<replacement>` entries applied
#                         in order, longest/most specific first. Callers set it
#                         to strip host identity out of absolute paths:
#                             REDACT_PATH_PREFIXES=("$MAIN_ROOT=<workspace>" "$HOME=~")
#                         Unset, no path rewriting happens.
#
# These are hygiene, not a security boundary: a password that a remote echoes
# back in some form neither function anticipates still reaches the report. The
# defence in depth is not putting credentials in manifest urls.

# Sourced, not executed — run directly this file would define two functions and
# exit 0 having done nothing, the silent success its callers exist to refuse.
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "redact.sh must be sourced, not executed: source ${BASH_SOURCE[0]} && redact_url <url>" >&2
    exit 2
fi

# redact_url <url> — a whole string that IS a url. Replaces the `user[:pass]@`
# userinfo of a `scheme://` url with `<redacted>`. An scp-style
# `git@github.com:org/repo` url has no password field and is left alone.
# It rewrites ONE url, the one the string starts with: a second full
# `scheme://user:pass@` url embedded inside the first (say, in its query
# string) is not touched. Anything that may carry more than one url — a
# captured error message, a command's output — goes through redact_text,
# which rewrites every occurrence.
redact_url() {
    local url="$1"
    # Greedy up to the LAST `@` before the host, not the first: a password
    # containing a literal `@` (`user:p@ss@host`) previously matched only the
    # shortest run (`[^/@]*` stops at the first `@`), leaving the password's
    # own `@`-suffix — `ss@host` — unredacted in the output.
    if [[ "$url" =~ ^([A-Za-z][A-Za-z0-9+.-]*://)[^/]*@(.*)$ ]]; then
        printf '%s<redacted>@%s' "${BASH_REMATCH[1]}" "${BASH_REMATCH[2]}"
    else
        printf '%s' "$url"
    fi
}

# redact_text <text> — a string that merely CONTAINS urls and paths, such as
# git's own captured stderr. git echoes the full remote url, userinfo included,
# in many clone and fetch errors, and it does so mid-sentence where redact_url's
# whole-string anchor does not match — so this rewrites every `scheme://userinfo@`
# occurrence anywhere in the string, on every line, and then applies
# $REDACT_PATH_PREFIXES.
redact_text() {
    local text="$1"
    # Same last-`@` widening as redact_url, and for the same reason: a
    # password containing a literal `@` left its own tail unredacted when the
    # class excluded `@` and stopped at the first one. Every `scheme://`
    # prefix contains a literal `/`, which the widened class still excludes,
    # so this cannot merge two distinct urls' credentials on one line — the
    # greedy run for one url's userinfo still halts at the next url's `://`
    # (or at any `/` in the first url's own path).
    text=$(printf '%s' "$text" \
           | sed -E 's#([A-Za-z][A-Za-z0-9+.-]*://)[^/[:space:]]+@#\1<redacted>@#g')
    local spec prefix replacement
    for spec in ${REDACT_PATH_PREFIXES[@]+"${REDACT_PATH_PREFIXES[@]}"}; do
        [[ -z "$spec" || "$spec" != *=* ]] && continue
        # A `<path>=<label>` spec is ambiguous whenever EITHER side contains
        # `=`, and splitting on a fixed `=` only moves the leak: the first `=`
        # corrupts a path containing `=`, the last corrupts a label containing
        # `=` — either way the path is left unmatched and leaks whole, silently.
        # Every label callers use is `~` or `<...>`, so parse the label by that
        # SHAPE first: the trailing `=~` or `=<...>` is the label, everything
        # before it the path, whatever `=` either contains. Only a spec with a
        # label of neither shape falls back to splitting on the last `=`.
        if [[ "$spec" =~ ^(.+)=(~|<[^<>]*>)$ ]]; then
            prefix=${BASH_REMATCH[1]}
            replacement=${BASH_REMATCH[2]}
        else
            prefix=${spec%=*}
            replacement=${spec##*=}
        fi
        # A bare `/` prefix would rewrite every path separator in the string.
        [[ -z "$prefix" || "$prefix" == "/" ]] && continue
        text=${text//"$prefix"/"$replacement"}
    done
    printf '%s' "$text"
}
