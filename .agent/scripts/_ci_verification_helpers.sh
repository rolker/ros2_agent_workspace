#!/bin/bash
# .agent/scripts/_ci_verification_helpers.sh
# Shared helpers for reading ci_local attestations (ADR-0018) as merge evidence.
#
# Source this file (it defines functions only; it runs nothing):
#   source "$SCRIPT_DIR/_ci_verification_helpers.sh"
#
# ADR-0018 accepts a full-scope `refs/notes/ci-local` record on a PR's head
# commit as merge verification for PROJECT-repo PRs. The workspace repo is
# exempt (decision 4) — that gate lives in the caller (merge_pr.sh), not here.
#
# Written for `set -eo pipefail` callers: every command whose failure is a
# legitimate signal is guarded, so sourcing this into a strict script is safe.

CI_LOCAL_NOTES_REF="ci-local"
# Scratch ref for the fetch fallback. NEVER fetch onto refs/notes/ci-local
# itself: that would risk clobbering a local, not-yet-pushed attestation
# ci_local.sh just wrote (it treats the ref as append-only; the read side must
# too).
CI_LOCAL_SCRATCH_REF="refs/notes/ci-local-merge-check"

# Parse an upstream.repos YAML blob (on stdin) into one repository key per line.
# Same parse shape ci_local.sh uses (yaml.safe_load over the `repositories`
# mapping) — deliberately a re-implementation of the shape, not an import of
# ci_local.sh, which is an executable driver, not a library.
_ci_local_upstream_entries() {
    python3 -c '
import sys, yaml
try:
    data = yaml.safe_load(sys.stdin.read())
except Exception:
    sys.exit(1)
repos = (data or {}).get("repositories")
if not isinstance(repos, dict) or not repos:
    sys.exit(1)
for name in repos:
    print(name)
' 2>/dev/null
}

# Does any record in the note satisfy ADR-0018 decisions 1, 2 and the #577
# upstream.repos completeness rule?
#
#   $1  note body (may hold several `---`-separated appended records)
#   $2  newline-separated upstream.repos entry names ("" when the repo has none)
#
# Echoes a one-word reason on stdout; returns 0 when some record qualifies.
# A note is a sequence of records because ci_local.sh APPENDS on re-runs — so
# every block must be considered, not just the first or last.
_ci_local_note_accepts() {
    local note="$1" upstream_entries="$2"
    local block reason="no-passing-record"
    local -a blocks=()
    # Split on the `---` separator ci_local.sh writes between appended records.
    local acc=""
    while IFS= read -r line; do
        if [[ "$line" == "---" ]]; then
            blocks+=("$acc"); acc=""
        else
            acc+="$line"$'\n'
        fi
    done <<<"$note"
    blocks+=("$acc")

    for block in "${blocks[@]}"; do
        # Exact matches only: `ci-local: pass (partial)` is NOT a pass, and
        # `scope: partial` is not full scope (ADR-0018 decision 2).
        grep -qx 'ci-local: pass' <<<"$block" || continue
        grep -qx 'scope: full'    <<<"$block" || continue
        if [[ -n "$upstream_entries" ]]; then
            local entry missing=false
            while IFS= read -r entry; do
                [[ -z "$entry" ]] && continue
                # `upstream-repo: <dir>@<sha>` — one line per upstream.repos
                # entry. A record missing any entry does not describe the
                # environment that was actually verified (ADR-0018 / #577).
                grep -qE "^upstream-repo: ${entry}@[0-9a-f]+$" <<<"$block" || missing=true
            done <<<"$upstream_entries"
            if [[ "$missing" == true ]]; then
                reason="incomplete-upstream-repos"
                continue
            fi
        fi
        return 0
    done
    echo "$reason"
    return 1
}

# ci_local_attestation_status <repo_path> <head_sha>
#
# Verdict on whether <head_sha> carries a full-scope ci_local attestation.
#   return 0  → attested        (a qualifying record exists on THAT commit)
#   return 1  → no-attestation  (anything else: absent, partial, ancestor-only,
#                                incomplete upstream coverage, unreadable)
# Echoes a one-line human explanation on stdout either way; the caller prints it.
#
# Notes are looked up LOCAL-FIRST. That matters and is not an optimization
# detail: a git worktree shares its ref store with the main checkout, so a note
# ci_local.sh wrote inside the feature worktree is already visible here — the
# common case needs no fetch at all.
#
# The lookup is EXACT-HEAD by design. `git notes show <sha>` is keyed by the
# sha path, so it resolves even when the commit object is absent locally, and
# no ancestor can be matched by accident. An attestation on an ancestor is not
# evidence for a head that may carry unverified changes since.
ci_local_attestation_status() {
    local repo="$1" head_sha="$2"
    local note="" ref_used="" reason=""

    if [[ -z "$repo" || -z "$head_sha" ]]; then
        echo "internal error: ci_local_attestation_status needs <repo_path> <head_sha>"
        return 1
    fi

    note=$(git -C "$repo" notes --ref="$CI_LOCAL_NOTES_REF" show "$head_sha" 2>/dev/null || true)
    if [[ -n "$note" ]]; then
        ref_used="local"
    else
        # Fetch fallback into a scratch ref. Both the up-front delete and the
        # FORCED refspec (`+`) are load-bearing: a leftover scratch ref from an
        # interrupted run makes the next fetch fail non-fast-forward, which
        # would yield a false "no attestation" — i.e. a merge on a warning
        # instead of on evidence.
        git -C "$repo" update-ref -d "$CI_LOCAL_SCRATCH_REF" 2>/dev/null || true
        if git -C "$repo" fetch -q origin \
                "+refs/notes/$CI_LOCAL_NOTES_REF:$CI_LOCAL_SCRATCH_REF" 2>/dev/null; then
            note=$(git -C "$repo" notes --ref="$CI_LOCAL_SCRATCH_REF" show "$head_sha" 2>/dev/null || true)
            [[ -n "$note" ]] && ref_used="origin"
        fi
        git -C "$repo" update-ref -d "$CI_LOCAL_SCRATCH_REF" 2>/dev/null || true
    fi

    if [[ -z "$note" ]]; then
        echo "no ci-local note on ${head_sha:0:12} (checked local + origin)"
        return 1
    fi

    # upstream.repos completeness (ADR-0018 / #577). WHICH COPY is read matters:
    # the caller passes the owning repo's MAIN checkout, typically sitting on
    # the default branch, so its working-tree file may not be the PR head's.
    # Read it at the head commit. Unlike the note lookup, `git show` needs the
    # commit object locally; when it is absent we cannot verify completeness and
    # must NOT pass by default.
    local upstream_entries=""
    local head_present=false
    git -C "$repo" cat-file -e "${head_sha}^{commit}" 2>/dev/null && head_present=true || true
    if [[ "$head_present" == true ]]; then
        if git -C "$repo" cat-file -e "${head_sha}:upstream.repos" 2>/dev/null; then
            upstream_entries=$(git -C "$repo" show "${head_sha}:upstream.repos" 2>/dev/null \
                | _ci_local_upstream_entries || true)
            if [[ -z "$upstream_entries" ]]; then
                echo "upstream.repos at ${head_sha:0:12} could not be parsed — refusing to treat the note as evidence"
                return 1
            fi
        fi
    elif grep -q '^upstream-repo: ' <<<"$note"; then
        # The note says this repo has upstream sources, but we cannot read
        # upstream.repos at the head to check the coverage is complete.
        echo "note for ${head_sha:0:12} records upstream-repo entries but the commit is not local — run 'git -C $repo fetch origin' and re-run"
        return 1
    fi

    if reason=$(_ci_local_note_accepts "$note" "$upstream_entries"); then
        echo "full-scope ci-local attestation on ${head_sha:0:12} (from $ref_used note)"
        return 0
    fi
    case "$reason" in
        incomplete-upstream-repos)
            echo "ci-local note on ${head_sha:0:12} passes but omits an upstream-repo: line for every upstream.repos entry (ADR-0018 #577) — not valid evidence" ;;
        *)
            echo "ci-local note on ${head_sha:0:12} has no 'ci-local: pass' + 'scope: full' record (partial/dirty runs are not merge evidence)" ;;
    esac
    return 1
}
