#!/bin/bash
# .agent/scripts/rosdep_local_staleness_check.sh
# Enforcement for the local-rosdep-key policy (#654).
#
# A key in a project repo's root rosdep.yaml is always a TEMPORARY stand-in for
# an upstream ros/rosdistro entry. Two things go wrong silently if nobody looks:
#
#   1. The upstream PR merges and the local entry is never deleted — the
#      workspace then keeps shadowing an upstream definition it no longer needs,
#      and quietly diverges from it the day upstream's changes.
#   2. A key is added and no upstream PR is ever opened — the workspace carries
#      a private dependency database forever, and nothing says so.
#
# This checks both, and is wired into `make validate` so it nags rather than
# waits to be remembered.
#
# THE PROBE MUST BE ABLE TO VOUCH FOR ITS OWN ANSWER. "Does this key resolve
# with default sources only?" is asked in a subshell with ROSDEP_SOURCE_PATH
# UNSET and HOME pointed at a throwaway directory — rosdep's cache lives at
# $HOME/.ros/rosdep/sources.cache, so probing under the caller's HOME would
# rewrite the very cache the local override populates — and with its own
# `rosdep update` run FIRST. If that update fails (offline, no network in CI,
# rosdep not installed, running as root), every key is reported SKIPPED. A
# resolves/does-not-resolve verdict read off a cache state the probe did not
# create would be worse than no verdict: it flags a live key for deletion, or
# hides a merged upstream entry.
#
# The "upstream PR owed" test is an advisory TEXT heuristic — it looks for `PR`
# or a `rosdistro` URL in the comment on, or immediately above, the key. There
# is no structured field for this in the rosdistro format. It catches the
# forgotten-bookkeeping case; it cannot verify that the PR exists.
#
# Usage:
#   rosdep_local_staleness_check.sh [<workspace_root>]
#     workspace_root  optional — default: workspace_root.sh (the MAIN checkout)
#
# Exit codes (distinct on purpose — `make validate` treats 3 as a notice):
#   0  clean, INCLUDING "no rosdep.yaml files found". A workspace where no
#      project repo has opted in must never fail this check.
#   1  a local key now resolves upstream (delete it), or a key carries no
#      upstream-PR marker
#   2  usage error
#   3  SKIPPED — the probe could not vouch for an answer (offline / no rosdep)

set -uo pipefail

if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
    echo "Error: execute this script, don't source it." >&2
    return 1 2>/dev/null || exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ "$#" -gt 1 ]; then
    echo "Usage: rosdep_local_staleness_check.sh [<workspace_root>]" >&2
    exit 2
fi

if [ "$#" -eq 1 ]; then
    ROOT_DIR="$1"
else
    if ! ROOT_DIR="$("$SCRIPT_DIR/workspace_root.sh")"; then
        echo "rosdep-local: cannot resolve the workspace root — nothing to check." >&2
        exit 2
    fi
fi

if [ ! -d "$ROOT_DIR" ]; then
    echo "Error: workspace_root '$ROOT_DIR' is not a directory." >&2
    exit 2
fi

shopt -s nullglob
yamls=("$ROOT_DIR"/layers/main/*_ws/src/*/rosdep.yaml)
if [ "${#yamls[@]}" -eq 0 ]; then
    echo "rosdep-local: no rosdep.yaml files found — nothing to check."
    exit 0
fi

# ---- parse: file<TAB>key<TAB>has-upstream-PR-marker ------------------------
# yaml.safe_load for the key set (the same module ci_local.sh parses
# upstream.repos with), plus a raw-text pass for the comments, which the loader
# discards. A key's marker may sit on its own line (trailing comment) or in the
# contiguous comment block directly above it.
KEYS_TSV="$(python3 - "${yamls[@]}" <<'PY'
import io, re, sys, yaml

rc = 0
for path in sys.argv[1:]:
    raw = io.open(path, encoding="utf-8").read()
    try:
        data = yaml.safe_load(raw)
    except yaml.YAMLError as e:
        sys.stderr.write("%s: not valid YAML: %s\n" % (path, e))
        rc = 1
        continue
    if data is None:
        continue
    if not isinstance(data, dict):
        sys.stderr.write("%s: top level is not a mapping of rosdep keys\n" % path)
        rc = 1
        continue
    lines = raw.splitlines()
    for key in data:
        marker = ""
        for i, line in enumerate(lines):
            if not re.match(r"^%s\s*:" % re.escape(str(key)), line):
                continue
            ctx = [line[line.index("#"):]] if "#" in line else []
            j = i - 1
            while j >= 0 and lines[j].lstrip().startswith("#"):
                ctx.append(lines[j])
                j -= 1
            marker = " ".join(ctx)
            break
        has = "yes" if re.search(r"\bPR\b|rosdistro", marker) else "no"
        print("%s\t%s\t%s" % (path, key, has))
sys.exit(rc)
PY
)" || { echo "rosdep-local: could not parse a rosdep.yaml (see above)." >&2; exit 1; }

if [ -z "$KEYS_TSV" ]; then
    echo "rosdep-local: ${#yamls[@]} rosdep.yaml file(s) found, but no keys declared."
    exit 0
fi

# ---- probe: default sources only, isolated cache ---------------------------
PROBE_HOME="$(mktemp -d -t rosdep_probe.XXXXXX)"
trap 'rm -rf "$PROBE_HOME"' EXIT

probe_ok=1
probe_why=""
if ! command -v rosdep >/dev/null 2>&1; then
    probe_ok=0
    probe_why="rosdep is not installed"
elif ! probe_out="$(env -u ROSDEP_SOURCE_PATH HOME="$PROBE_HOME" rosdep update 2>&1)"; then
    probe_ok=0
    probe_why="$(printf '%s' "$probe_out" | tail -n 1)"
fi

resolves_upstream() {
    env -u ROSDEP_SOURCE_PATH HOME="$PROBE_HOME" rosdep resolve "$1" >/dev/null 2>&1
}

# ---- report ----------------------------------------------------------------
rc=0
stale=0
unmarked=0
skipped=0
while IFS=$'\t' read -r file key marked; do
    [ -z "$key" ] && continue
    rel="${file#"$ROOT_DIR"/}"
    if [ "$marked" = "no" ]; then
        echo "❌ $rel: key '$key' has no upstream-PR marker in its comment."
        echo "   A local key is a stand-in for an upstream ros/rosdistro entry — note the"
        echo "   owed or open PR beside it, e.g.  # upstream PR owed: ros/rosdistro#NNNNN"
        unmarked=$((unmarked + 1))
        rc=1
    fi
    if [ "$probe_ok" = 0 ]; then
        skipped=$((skipped + 1))
        continue
    fi
    if resolves_upstream "$key"; then
        echo "❌ $rel: key '$key' now resolves against the default rosdep sources."
        echo "   The upstream entry has landed — delete the local entry (and the file,"
        echo "   if it was its last key)."
        stale=$((stale + 1))
        rc=1
    fi
done <<< "$KEYS_TSV"

total="$(printf '%s\n' "$KEYS_TSV" | grep -c .)"
if [ "$probe_ok" = 0 ]; then
    echo "⏭️  rosdep-local: SKIPPED the upstream-resolution probe for $skipped key(s)."
    echo "   Reason: $probe_why"
    echo "   No resolves/does-not-resolve verdict is reported from a cache this check"
    echo "   did not build. Re-run online to get one."
    # An unmarked key is a text-only finding and stands on its own; report it
    # as a real failure even when the probe could not run.
    [ "$rc" -ne 0 ] && exit 1
    exit 3
fi

if [ "$rc" -eq 0 ]; then
    echo "✅ rosdep-local: $total local rosdep key(s) across ${#yamls[@]} file(s) — all still needed, all marked."
else
    echo "rosdep-local: $stale stale key(s), $unmarked unmarked key(s), of $total checked."
fi
exit "$rc"
