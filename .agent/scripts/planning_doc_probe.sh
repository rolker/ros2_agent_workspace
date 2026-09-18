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
# `^## Vision([[:space:]]|$)`. This matches a bare `## Vision` (this workspace's own
# README.md) and also a longer heading that starts with "Vision", such as
# `## Vision and Goals` — a project repo may reasonably phrase the heading
# with trailing words and the draft names no exact wording, only the
# section's topic. It does NOT match `### Vision` (wrong heading level),
# `## Our Vision` (does not start with the word), or `## Visionary Roadmap`
# (the word-boundary check after "Vision" — a space or end-of-line — stops a
# same-prefix word like "Visionary" from false-positiving). It does not match
# the word "Vision" appearing anywhere but at the start of a `##`-level
# heading. See test_planning_doc_probe.sh's "Vision heading variants" cases.
#
# Markdown-context-aware matching (fenced code blocks, blockquotes) is
# deliberately NOT implemented: a `## Vision` line inside ``` ``` or a `>`
# blockquote would still match. Handling that correctly needs a stateful
# line scanner (or a markdown parser) to track fence/quote state, which is a
# lot of complexity for a self-inflicted edge case — nobody puts a real
# "## Vision" heading inside a code fence on purpose. Since the Planning
# Documents section this feeds is descriptive-only (absence is never a
# finding), the only thing a false match could get wrong is reporting
# "present" for a document with no real Vision section, which is a harmless,
# non-blocking cosmetic error, not something worth the parser.
#
# Permission-denied paths: all four probes emit a distinct stderr diagnostic
# rather than silently reporting absent-and-nothing-else, whenever the
# permission problem is one the probe can actually detect. probe_vision and
# probe_decision detect it on the file/directory they read CONTENTS from
# (grep'ing README.md, listing docs/decisions/); probe_roadmap and
# probe_health detect it on the immediate parent directory of the file they
# check for (repo_path itself for ROADMAP.md, docs/ for docs/health.md) —
# they never read the target file's own contents, so an unreadable-but-
# listable target file is still correctly reported present (existence alone
# is all that kind needs). In every case, a permission problem the probe
# cannot detect at all (e.g. repo_path itself unsearchable) still degrades
# to a silent absent — the CLI's own top-level `[ -x ]` guard on repo_path
# (see the exit-3 case above) is what catches that one before any probe runs.
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
#   #   3 = repo_path does not exist or is not a readable AND searchable
#   #       directory (missing read or `x`/search permission) — a real error
#   #       (the probe could not run at all), not an absence finding. Without
#   #       the `x` bit nothing inside repo_path can be stat'd or opened, so
#   #       every probe would silently read as "absent" instead of surfacing
#   #       the real "could not run" condition.
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
# ^## Vision([[:space:]]|$) (see the heading-match note above). Prints
# "present\tREADME.md" or "absent\t". A README.md that exists but cannot be
# read (permission denied) is reported absent, same as a genuinely missing
# one, but with a distinct stderr diagnostic so the two are not confused by
# anyone reading the probe's own diagnostics (the TSV output contract is
# unaffected — see the permission-denied note above).
# True iff the resolved path of $2 lies inside the resolved path of $1.
_inside_repo() {
    local root target
    root=$(realpath -e -- "$1" 2>/dev/null) || return 1
    target=$(realpath -e -- "$2" 2>/dev/null) || return 1
    [ "$target" = "$root" ] || [ "${target#"$root"/}" != "$target" ]
}

# _outside <repo_path> <path> <label>: true iff <path> is a symlink resolving
# outside <repo_path>; prints the diagnostic. The probe never reads outside
# the repo it was given, so such a file reads as absent.
_outside() {
    if [ -L "$2" ] && ! _inside_repo "$1" "$2"; then
        echo "planning_doc_probe.sh: $2 is a symlink resolving outside $1 — reporting $3 absent (the probe never reads outside the repo)" >&2
        return 0
    fi
    return 1
}

probe_vision() {
    local repo_path="$1"
    local readme="$repo_path/README.md"
    if [ -f "$readme" ]; then
        if _outside "$repo_path" "$readme" "README.md"; then
            :
        elif [ ! -r "$readme" ]; then
            echo "planning_doc_probe.sh: $readme exists but is not readable (permission denied) — reporting absent" >&2
        elif grep -qE '^## Vision([[:space:]]|$)' "$readme"; then
            printf 'present\tREADME.md\n'
            return
        fi
    fi
    printf 'absent\t\n'
}

# probe_roadmap <repo_path>
# Present iff ROADMAP.md exists at the repo root (plain file existence — the
# draft names no required heading for this kind).
#
# A repo_path that exists but is not searchable (permission denied) is
# reported absent, same as a genuinely missing ROADMAP.md, but with a
# distinct stderr diagnostic — same rationale as probe_vision above. (The
# file's own read permission does not matter here — this probe only checks
# existence, never content — so an unreadable-but-present ROADMAP.md is
# still correctly reported present.) Only SEARCH permission (x) on the parent
# is required for a stat by known name; read permission (r) is not, so a
# search-only parent with the file present still reports present.
probe_roadmap() {
    local repo_path="$1"
    if [ -d "$repo_path" ] && [ ! -x "$repo_path" ]; then
        echo "planning_doc_probe.sh: $repo_path exists but is not searchable (permission denied) — reporting ROADMAP.md absent" >&2
    elif [ -f "$repo_path/ROADMAP.md" ]; then
        if ! _outside "$repo_path" "$repo_path/ROADMAP.md" "ROADMAP.md"; then
            printf 'present\tROADMAP.md\n'
            return
        fi
    fi
    printf 'absent\t\n'
}

# probe_decision <repo_path>
# Present iff docs/decisions/ exists and contains at least one non-dotfile
# entry. A directory that exists but is empty (or holds only dotfiles, e.g. a
# tracked .gitkeep placeholder) reads as absent — the kind is "a decision was
# recorded here", not "the directory was created".
#
# Dotfile exclusion uses `find ... ! -name '.*'` rather than a shell glob, so
# it does not depend on the caller's `dotglob` shell option being unset (this
# script is sourced into whatever shell state the caller already has — a
# glob-based exclusion would silently start counting dotfiles if dotglob were
# ever on).
#
# A directory that exists but cannot be listed (permission denied) is
# reported absent, same as a genuinely empty one, but with a distinct stderr
# diagnostic — same rationale as probe_vision above.
#
# A directory holding only a broken symlink also reads as absent: `[ -e ]`
# follows a symlink and a broken one fails that test. This is deliberate, not
# a gap to close — the kind means "a decision resolves here", not "some
# directory entry, valid or not, was placed here" — but it is a real,
# documented side effect: an entry can exist (`find` lists it) while still
# reading as absent.
#
# The same inside-the-repo rule applies to README.md, ROADMAP.md and
# docs/health.md: a symlink at any of those paths resolving outside repo_path
# reads as absent with a diagnostic.
#
# `docs/decisions` itself may be a symlink to a populated directory — `find
# -L` is used so listing descends through that symlink (plain `find`, without
# -L, does not descend into a symlinked start path). A symlinked
# `docs/decisions` is followed only if it resolves INSIDE repo_path; one
# pointing outside the repo reads as absent with a diagnostic, because the
# probe never reads outside the repo it was given; the same rule applies to
# each ENTRY: a symlinked entry whose target resolves outside repo_path is
# not counted. This still correctly
# reads a broken-symlink ENTRY inside the directory as absent, per the
# paragraph above, since `find -L` still lists a broken symlink (it just
# cannot resolve it) and the `[ -e ]` guard still drops it.
probe_decision() {
    local repo_path="$1"
    local dir="$repo_path/docs/decisions"
    local entry
    if [ -d "$repo_path/docs" ] && [ ! -x "$repo_path/docs" ]; then
        # The parent is unsearchable, so [ -d "$dir" ] below cannot even stat
        # it: distinguish that from a genuinely missing docs/decisions.
        echo "planning_doc_probe.sh: $repo_path/docs exists but is not searchable (permission denied) — reporting docs/decisions absent" >&2
    elif [ -d "$dir" ]; then
        if [ ! -r "$dir" ] || [ ! -x "$dir" ]; then
            echo "planning_doc_probe.sh: $dir exists but is not listable (permission denied) — reporting absent" >&2
        elif [ -L "$dir" ] && ! _inside_repo "$repo_path" "$dir"; then
            echo "planning_doc_probe.sh: $dir is a symlink resolving outside $repo_path — reporting absent (the probe never reads outside the repo)" >&2
        else
            while IFS= read -r -d '' entry; do
                [ -e "$entry" ] || continue  # broken symlink: exists as an entry, but reads as absent (see note above)
                if [ -L "$entry" ] && ! _inside_repo "$repo_path" "$entry"; then
                    echo "planning_doc_probe.sh: $entry is a symlink resolving outside $repo_path — not counted (the probe never reads outside the repo)" >&2
                    continue
                fi
                printf 'present\tdocs/decisions\n'
                return
            done < <(find -L "$dir" -mindepth 1 -maxdepth 1 ! -name '.*' -print0 2>/dev/null)
        fi
    fi
    printf 'absent\t\n'
}

# probe_health <repo_path>
# Present iff docs/health.md exists.
#
# A docs/ directory that exists but is not searchable (permission denied) is
# reported absent, same as a genuinely missing docs/health.md, but with a
# distinct stderr diagnostic — same rationale as probe_vision above. (As in
# probe_roadmap, the file's own read permission does not matter — this probe
# only checks existence, never content.)
probe_health() {
    local repo_path="$1"
    local dir="$repo_path/docs"
    if [ -d "$dir" ] && [ ! -x "$dir" ]; then
        echo "planning_doc_probe.sh: $dir exists but is not searchable (permission denied) — reporting docs/health.md absent" >&2
    elif [ -f "$dir/health.md" ]; then
        if ! _outside "$repo_path" "$dir/health.md" "docs/health.md"; then
            printf 'present\tdocs/health.md\n'
            return
        fi
    fi
    printf 'absent\t\n'
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
    if [ ! -d "$repo_path" ] || [ ! -r "$repo_path" ] || [ ! -x "$repo_path" ]; then
        echo "planning_doc_probe.sh: not a readable/searchable directory: $repo_path" >&2
        exit 3
    fi
    probe_all "$repo_path"
    exit 0
fi
