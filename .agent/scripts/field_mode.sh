#!/bin/bash
# .agent/scripts/field_mode.sh
# Detect whether the current repo is in "field mode" vs. "dev mode".
#
# Field mode = the repo's origin host is NOT on the GitHub allowlist
# (currently `github.com` and `ssh.github.com` — see is_field_url
# below). Examples: gitcloud, a private Forgejo, another non-GitHub
# remote. In field mode, agents may edit tracked files in the main
# tree and commit directly to the default branch without opening a
# PR. See issue #445 for rationale.
#
# Dev mode = the repo's origin host is on the GitHub allowlist. Dev
# mode enforces the worktree + PR workflow.
#
# Usage:
#   # CLI with no args — exit status mirrors is_field_mode:
#   #   0 = field mode, 1 = dev mode or error (no git repo / no origin)
#   .agent/scripts/field_mode.sh [<repo_dir>]
#
#   # CLI with --describe — prints a human-readable summary.
#   # Exit 0 unless origin is missing (then exit 1).
#   .agent/scripts/field_mode.sh --describe [<repo_dir>]
#
#   # Sourced (script must be reachable by path — not on PATH by default):
#   source /path/to/.agent/scripts/field_mode.sh
#   if is_field_mode; then                 # checks $PWD
#     echo "field mode — direct main-tree edits permitted"
#   fi
#   if is_field_mode "/path/to/some/repo"; then  # check a specific repo
#     ...
#   fi
#   if is_field_url "$url"; then           # classify a URL with no checkout
#     ...
#   fi
#
# Two entry points, one allowlist. `is_field_mode` answers for a checkout on
# disk; `is_field_url` answers for a bare URL, which is what a caller has
# before anything is cloned — `janitor-sweep` builds its repo rotation from
# the `.repos` manifests, where no checkout exists yet. `is_field_mode` is
# implemented on top of `is_field_url` so the host allowlist (ADR-0011,
# AGENTS.md § Field Mode) exists exactly once.

# is_field_url <url>
# Exit 0 if the given git URL is a field-mode origin, else 1. THE allowlist —
# `is_field_mode` and every caller that only has a URL both go through here,
# so there is never a second copy of the host list to drift.
#
# Host is extracted from the URL by stripping the scheme (`*://`), then any
# user prefix (`*@`), then anything from the first `:` or `/` onwards. The
# remaining host is lowercased and compared against the GitHub allowlist
# (`github.com`, `ssh.github.com`). Any host not in the allowlist — including
# `mygithub.com`, subdomains like `github.company.internal`, and paths like
# `https://example.com/github.com/foo` — is field mode.
#
# An empty or host-less URL returns 1 (dev mode), matching is_field_mode's
# "no origin → the safer default" behaviour. A caller that must distinguish
# "not a field origin" from "could not classify this at all" has to check the
# URL is non-empty itself — this function answers one question.
is_field_url() {
    local url="${1:-}"
    local host

    host="${url#*://}"          # strip scheme (if any)
    host="${host#*@}"           # strip user@ (if any)
    host="${host%%[:/]*}"       # take up to first : or /
    host="${host,,}"            # lowercase (DNS is case-insensitive)

    # Dev-mode allowlist: GitHub's canonical git hosts. `ssh.github.com` is
    # GitHub's SSH-over-443 fallback for users behind firewalls. Extend this
    # list only for hosts GitHub itself documents for git clone.
    case "$host" in
        github.com|ssh.github.com) return 1 ;;  # dev mode
    esac

    [[ -n "$host" ]]            # field mode iff we extracted a host
}

# is_field_mode [repo_dir]
# Exit 0 if the given repo (default: $PWD) is in field mode, else 1.
# Reads the checkout's `origin` URL and classifies it with is_field_url; a
# repo with no origin (or no repo at all) is dev mode, the safer default.
is_field_mode() {
    local repo_dir="${1:-$PWD}"
    local origin_url
    origin_url=$(git -C "$repo_dir" remote get-url origin 2>/dev/null) || return 1

    is_field_url "$origin_url"
}

# describe_mode [repo_dir]
# Print a human-readable mode summary to stdout.
#
# Uses `|| true` on the git call so that callers running with `set -e`
# don't exit before the empty-string check; we want to print a message
# and return 1 in that case, not silently abort the parent shell.
describe_mode() {
    local repo_dir="${1:-$PWD}"
    local origin_url
    origin_url=$(git -C "$repo_dir" remote get-url origin 2>/dev/null || true)
    if [ -z "$origin_url" ]; then
        echo "not a git repo or no 'origin' remote: $repo_dir"
        return 1
    fi

    if is_field_mode "$repo_dir"; then
        echo "field mode  (origin: $origin_url)"
    else
        echo "dev mode    (origin: $origin_url)"
    fi
}

# If invoked directly (not sourced), run as CLI
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    case "${1:-}" in
        --describe)
            shift
            describe_mode "$@"
            ;;
        *)
            is_field_mode "$@"
            ;;
    esac
fi
