#!/bin/bash
# .agent/scripts/planning_doc_probe.sh
# Probe a repo for the four planning-document kinds named in the
# expected-location table (docs/design/planning_document_vocabulary.md,
# "Discovery by convention: the expected-location table"):
#
#   | Kind     | Expected location                              |
#   |----------|-------------------------------------------------|
#   | vision   | a `## Vision` section in `README.md` at the root |
#   | roadmap  | `ROADMAP.md` at the repo root                    |
#   | decision | `docs/decisions/`                                |
#   | health   | `docs/health.md`                                 |
#
# This is a published EXPECTATION, not a requirement (the draft is explicit:
# "a project that keeps a document somewhere else is not in violation,
# produces no error, and is flagged by no check"). This script only probes
# these four exact paths — no externally-hosted-document fallback (org
# Project board, `homepage` field, README link matching); that is
# https://github.com/rolker/ros2_agent_workspace/issues/643, deliberately out
# of scope here.
#
# The script reads the filesystem only. It writes nothing, caches nothing,
# and creates no marker/cache file anywhere (ADR-0017's rejection of a
# per-repo declaration file extends to this reader, per the design draft's
# "no per-repo declaration file and no schema" decision, 2026-09-17).
#
# `## Vision` heading match: exact-level prefix match on the heading text,
# `^## Vision`. This matches a bare `## Vision` (this workspace's own
# README.md) and also a longer heading that starts with "Vision", such as
# `## Vision and Goals` — a project repo may reasonably phrase the heading
# with trailing words and the draft names no exact wording, only the
# section's topic. It does NOT match `### Vision` (wrong heading level) or
# `## Our Vision` (does not start with the word), and it does not match the
# word "Vision" appearing anywhere but at the start of a `##`-level heading.
# See test_planning_doc_probe.sh's "Vision heading variants" cases.
#
# Usage:
#   # CLI — probes all four kinds, prints TSV to stdout, one line per kind:
#   #   <kind>\t<present|absent>\t<relative-path-or-empty>
#   .agent/scripts/planning_doc_probe.sh <repo_path>
#
#   # Exit codes:
#   #   0 = ran successfully — presence or absence of any/all four kinds is
#   #       NEVER a non-zero exit (absence is never a finding, per the draft)
#   #   2 = usage error (missing or extra arguments)
#   #   3 = repo_path does not exist or is not a readable directory — a real
#   #       error (the probe could not run at all), not an absence finding
#
#   # Sourced (script must be reachable by path — not on PATH by default):
#   source /path/to/.agent/scripts/planning_doc_probe.sh
#   probe_vision "$repo_path"    # prints "present\t<rel-path>" or "absent\t"
#   probe_roadmap "$repo_path"
#   probe_decision "$repo_path"
#   probe_health "$repo_path"
#   probe_all "$repo_path"       # all four, one TSV line per kind, to stdout

# probe_vision <repo_path>
# Present iff README.md exists at the repo root and contains a line matching
# ^## Vision (see the heading-match note above). Prints
# "present\tREADME.md" or "absent\t".
probe_vision() {
    local repo_path="$1"
    local readme="$repo_path/README.md"
    if [ -f "$readme" ] && grep -q '^## Vision' "$readme"; then
        printf 'present\tREADME.md\n'
    else
        printf 'absent\t\n'
    fi
}

# probe_roadmap <repo_path>
# Present iff ROADMAP.md exists at the repo root (plain file existence — the
# draft names no required heading for this kind).
probe_roadmap() {
    local repo_path="$1"
    if [ -f "$repo_path/ROADMAP.md" ]; then
        printf 'present\tROADMAP.md\n'
    else
        printf 'absent\t\n'
    fi
}

# probe_decision <repo_path>
# Present iff docs/decisions/ exists and contains at least one non-dotfile
# entry. A directory that exists but is empty (or holds only dotfiles, e.g. a
# tracked .gitkeep placeholder) reads as absent — the kind is "a decision was
# recorded here", not "the directory was created".
probe_decision() {
    local repo_path="$1"
    local dir="$repo_path/docs/decisions"
    local entry
    if [ -d "$dir" ]; then
        for entry in "$dir"/*; do
            # Glob with no match expands to the literal pattern; skip that.
            [ -e "$entry" ] || continue
            printf 'present\tdocs/decisions\n'
            return
        done
    fi
    printf 'absent\t\n'
}

# probe_health <repo_path>
# Present iff docs/health.md exists.
probe_health() {
    local repo_path="$1"
    if [ -f "$repo_path/docs/health.md" ]; then
        printf 'present\tdocs/health.md\n'
    else
        printf 'absent\t\n'
    fi
}

# probe_all <repo_path>
# Runs all four probes and prints one TSV line per kind to stdout:
#   <kind>\t<present|absent>\t<relative-path-or-empty>
# This is the machine-readable contract audit-project (and, through its
# per-repo embed, janitor-sweep) reads.
probe_all() {
    local repo_path="$1"
    local kind result status path
    for kind in vision roadmap decision health; do
        result=$("probe_$kind" "$repo_path")
        status="${result%%$'\t'*}"
        path="${result#*$'\t'}"
        printf '%s\t%s\t%s\n' "$kind" "$status" "$path"
    done
}

# If invoked directly (not sourced), run as CLI.
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    if [ "$#" -ne 1 ]; then
        echo "Usage: $0 <repo_path>" >&2
        exit 2
    fi
    repo_path="$1"
    if [ ! -d "$repo_path" ] || [ ! -r "$repo_path" ]; then
        echo "planning_doc_probe.sh: not a readable directory: $repo_path" >&2
        exit 3
    fi
    probe_all "$repo_path"
    exit 0
fi
