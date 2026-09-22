#!/bin/bash
# .agent/scripts/rosdep_local_sources.sh
# Generate a workspace-owned rosdep sources.list.d directory that layers every
# project repo's local `rosdep.yaml` on top of the system default sources (#654).
#
# WHY: some Ubuntu-shipped packages have no rosdep key upstream yet. Rather than
# editing /etc (root-owned, invisible to CI, and undoable only by hand), rosdep
# honours ROSDEP_SOURCE_PATH — it REPLACES sources.list.d wholesale, so the
# generated directory must carry a copy of the system lists as well as the local
# ones. Every *.list under the system directory is copied, not just
# 20-default.list: a host may legitimately carry others (10-local.list, a
# site-local overlay), and copying only the default would silently drop a source
# the caller already had.
#
# A project repo opts in by adding a `rosdep.yaml` at ITS OWN ROOT, in upstream
# ros/rosdistro format:
#
#     python3-pystac:          # upstream PR owed: https://github.com/ros/rosdistro/pull/NNNNN
#       ubuntu: [python3-pystac]
#       debian: [python3-pystac]
#
# The comment naming the owed upstream PR is what rosdep_local_staleness_check.sh
# looks for — a local key is always a temporary stand-in for an upstream entry.
#
# This script is deliberately generic: it globs, it never names a repo. The
# `rosdep.yaml` files themselves live in the consuming project repos.
#
# Usage:
#   rosdep_local_sources.sh <workspace_root> [<out_dir>]
#     workspace_root  required — path whose layers/main/*_ws/src/ holds the repos
#     out_dir         optional — default <workspace_root>/.rosdep/sources.list.d
#
# Anchor <workspace_root> at the MAIN workspace root so every worktree on the
# host shares one generated directory (setup.bash resolves it that way).
#
# Because that directory IS shared — setup.bash exports it as
# ROSDEP_SOURCE_PATH for every shell on the host, and two worktrees' `make
# build` both regenerate it — the rebuild is never done in place. The new
# content is built in a sibling slot directory and published by an ATOMIC
# symlink swap, so <out_dir> always resolves to a COMPLETE source directory:
# a reader mid-rebuild sees either the old generation or the new one, never a
# half-copied dir and never a missing one. Concurrent regenerations are
# serialized by a bounded flock (300s, ROSDEP_SOURCES_LOCK_TIMEOUT), following
# docker_run_agent.sh's precedent; where flock is unavailable the run proceeds
# unserialized and says so — the swap still keeps readers consistent.
#
# Callers: bootstrap.sh (step 4), the Makefile's $(STAMP)/rosdep-local.done
# stamp, and anyone regenerating by hand. Consumers read it via
# ROSDEP_SOURCE_PATH, which setup.bash exports when the directory exists.
#
# Prints the number of rosdep.yaml files aggregated. Exit codes:
#   0  generated (including the common zero-rosdep.yaml case — an empty local
#      list is valid, and the system lists are still copied)
#   2  usage error / workspace_root is not a directory
#   3  rosdep is not initialized (no *.list under the system sources dir) —
#      run bootstrap.sh (`sudo rosdep init`) first. Refusing is deliberate: a
#      generated directory holding ONLY the local list would shadow the real
#      default sources and break every rosdep call on the host.
#   4  one or more project rosdep.yaml files were REJECTED by the shape gate
#      (rosdep_yaml_validate.sh) or could not be validated at all. Those files
#      are left OUT of the generated list — the rest of the directory is still
#      written and usable — and the exit is non-zero because a key feeding a
#      root-level `rosdep install` must not arrive unvalidated.

set -euo pipefail

if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
    echo "Error: execute this script, don't source it." >&2
    return 1 2>/dev/null || exit 1
fi

SYSTEM_SOURCES_DIR="${ROSDEP_SYSTEM_SOURCES_DIR:-/etc/ros/rosdep/sources.list.d}"

if [ "$#" -lt 1 ] || [ -z "${1:-}" ]; then
    echo "Usage: rosdep_local_sources.sh <workspace_root> [<out_dir>]" >&2
    exit 2
fi

ROOT_DIR="$1"
OUT_DIR="${2:-$ROOT_DIR/.rosdep/sources.list.d}"

if [ ! -d "$ROOT_DIR" ]; then
    echo "Error: workspace_root '$ROOT_DIR' is not a directory." >&2
    exit 2
fi

# Canonicalize before anything derives from it. The generated list holds
# `yaml file://<path>` URIs, and a file:// URI built from a RELATIVE path is
# not resolvable by whatever process later reads the list — which is never
# this script's own working directory. Every current caller happens to pass an
# absolute path; nothing made that a requirement, so make it one here instead
# of documenting a trap. `CDPATH=''` keeps `cd` from printing or hopping.
ROOT_DIR="$(CDPATH='' cd -- "$ROOT_DIR" && pwd)"
case "$OUT_DIR" in
    /*) ;;
    *)  OUT_DIR="$PWD/$OUT_DIR" ;;
esac

shopt -s nullglob
system_lists=("$SYSTEM_SOURCES_DIR"/*.list)
if [ "${#system_lists[@]}" -eq 0 ]; then
    echo "Error: no *.list found under $SYSTEM_SOURCES_DIR — rosdep is not initialized." >&2
    echo "       Run .agent/scripts/bootstrap.sh (or: sudo rosdep init) first." >&2
    exit 3
fi

# ---- serialize, then build into a slot (never in place) ---------------------
# See the header: <out_dir> is host-shared, so it is published by an atomic
# symlink swap rather than rebuilt under readers' feet.
OUT_PARENT="$(dirname "$OUT_DIR")"
OUT_BASE="$(basename "$OUT_DIR")"
SLOT_ROOT_NAME=".$OUT_BASE.slots"
SLOT_ROOT="$OUT_PARENT/$SLOT_ROOT_NAME"
LOCK_FILE="$OUT_PARENT/.$OUT_BASE.lock"
LOCK_TIMEOUT="${ROSDEP_SOURCES_LOCK_TIMEOUT:-300}"

mkdir -p "$OUT_PARENT"

# The lock is held for the whole build AND the swap: the slot chosen below is
# "the one the symlink does not point at", which is only safe while no other
# writer is choosing at the same time. fd 9 is released when this process
# exits.
if command -v flock >/dev/null 2>&1 && exec 9>"$LOCK_FILE"; then
    flock -w "$LOCK_TIMEOUT" 9 \
        || echo "Warning: could not take $LOCK_FILE within ${LOCK_TIMEOUT}s —" \
                "regenerating unserialized." >&2
else
    echo "Warning: flock unavailable — regenerating unserialized." >&2
fi

# Two slots, alternating: the one currently published is left untouched until
# the swap, so a reader that already resolved the symlink keeps a readable
# directory. Anything else at <out_dir> (a real directory from before this
# scheme, a stray file) is replaced by the symlink at swap time.
current_slot=""
if [ -L "$OUT_DIR" ]; then
    current_slot="$(readlink "$OUT_DIR")"
fi
case "$current_slot" in
    */a) slot="b" ;;
    *)   slot="a" ;;
esac
BUILD_DIR="$SLOT_ROOT/$slot"

# Rebuild the slot from scratch so a removed repo's entry cannot survive
# (idempotent).
rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"

for list in "${system_lists[@]}"; do
    cp "$list" "$BUILD_DIR/$(basename "$list")"
done

# Sorted for determinism: the generated file is compared by the Makefile stamp
# and read by humans; glob order must not churn it.
local_yamls=("$ROOT_DIR"/layers/main/*_ws/src/*/rosdep.yaml)
LOCAL_LIST="$BUILD_DIR/30-workspace-local.list"
{
    echo "# Generated by .agent/scripts/rosdep_local_sources.sh — do not edit."
    echo "# One line per project repo carrying a root rosdep.yaml (#654)."
} > "$LOCAL_LIST"

# Shape gate (#654): these files drive a root-level `rosdep install`, and
# rosdep's own format also accepts pip/npm/gem/source rules. A rejected file is
# EXCLUDED here — never merely warned about — and the script exits 4 so the
# caller can decide (the Makefile stamp fails the build; bootstrap.sh notes it).
# A validator that cannot run at all (exit 3) is treated the same way: fail
# closed, because "unvalidated" and "invalid" reach root identically.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VALIDATOR="$SCRIPT_DIR/rosdep_yaml_validate.sh"

yaml_count=0
rejected_count=0
if [ "${#local_yamls[@]}" -gt 0 ]; then
    while IFS= read -r yaml; do
        if [ ! -x "$VALIDATOR" ]; then
            echo "Error: $VALIDATOR is missing — cannot validate '$yaml'." >&2
            rejected_count=$((rejected_count + 1))
            continue
        fi
        if ! "$VALIDATOR" "$yaml"; then
            echo "   excluded from $LOCAL_LIST" >&2
            rejected_count=$((rejected_count + 1))
            continue
        fi
        echo "yaml file://$yaml" >> "$LOCAL_LIST"
        yaml_count=$((yaml_count + 1))
    done < <(printf '%s\n' "${local_yamls[@]}" | LC_ALL=C sort)
fi

# ---- publish: atomic symlink swap -------------------------------------------
# rename(2) over an existing symlink is atomic, so <out_dir> never transiently
# vanishes or appears half-built. A pre-existing real directory at <out_dir>
# (an older generation of this script, or a hand-made one) cannot be renamed
# over, so it is removed first — a one-time migration, inside the lock.
TMP_LINK="$OUT_PARENT/.$OUT_BASE.link.$$"
rm -f "$TMP_LINK"
ln -s "$SLOT_ROOT_NAME/$slot" "$TMP_LINK"
if [ -e "$OUT_DIR" ] && [ ! -L "$OUT_DIR" ]; then
    rm -rf "$OUT_DIR"
fi
mv -T "$TMP_LINK" "$OUT_DIR"

echo "Aggregated $yaml_count project rosdep.yaml file(s) into $OUT_DIR" \
     "(over ${#system_lists[@]} system source list(s))"
if [ "$yaml_count" -gt 0 ]; then
    echo "Run 'ROSDEP_SOURCE_PATH=$OUT_DIR rosdep update' to refresh the cache."
fi
if [ "$rejected_count" -gt 0 ]; then
    echo "Error: $rejected_count project rosdep.yaml file(s) rejected (see above)." >&2
    exit 4
fi
