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
# DISCOVERY (#659): a repo's rosdep.yaml is picked up under layers/main AND
# under any registered layer worktree (layers/worktrees/*/*_ws/src/*/), via
# _worktree_helpers.sh's wt_discover_local_rosdep_yamls — see that function
# for the symlink-skip and git-worktree-registration rules. Both globs feed
# ONE host-shared directory (ROSDEP_SOURCE_PATH is exported once, for every
# shell on the host, by setup.bash), so a key a branch changes is visible to
# every build on the host until its worktree is removed.
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
# docker_run_agent.sh's precedent; where flock is not installed (or its lock
# file cannot be opened) the run proceeds unserialized and says which of the
# two it was — the swap still keeps readers consistent. A flock TIMEOUT is
# different and is a hard failure (exit 5): it proves another writer holds the
# lock, so proceeding would put two writers in one build slot.
#
# The FIRST run over a pre-existing REAL directory at <out_dir> (an older
# generation of this script) has to migrate it, and a symlink cannot be renamed
# over a directory. That path swaps the two atomically with
# renameat2(RENAME_EXCHANGE) and deletes the old content afterwards; where that
# call is unsupported it falls back to rename-aside-then-publish in a single
# process, whose residual gap is one rename(2) — never the `rm -rf` of a whole
# tree that it replaced. Steady state (symlink over symlink) is a single
# rename and is never absent.
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
#   5  timed out waiting for the regeneration lock (ROSDEP_SOURCES_LOCK_TIMEOUT,
#      default 300s) — another regeneration still holds it. Nothing was
#      written; the previously published directory is untouched. Retry.
#   6  one or more rosdep keys are declared with DIFFERENT package lists by
#      two or more discovered files (layers/main vs. a worktree, or worktree
#      vs. worktree) (#659). rosdep itself does not error on a duplicate key —
#      it silently merges sources first-loaded-wins — so this script instead
#      EXCLUDES every file touched by a conflicting key from the generated
#      list (never picks a side) and reports each conflict. An IDENTICAL
#      declaration in two places is not a conflict and dedupes cleanly.
#      Exit-code precedence when a shape rejection (4) and a conflict (6)
#      both occur in the same run: 4 wins — it is the stricter, pre-existing
#      gate, and keeping it load-bearing preserves today's `make build` /
#      bootstrap.sh failure semantics for the already-shipped case.
#   7  one or more discovered rosdep.yaml files that PASSED the shape gate
#      became unreadable (typically: deleted) before this run finished with
#      them (#659 round-2 pre-push review). Worktree teardown racing this
#      generator is the routine trigger, not a theoretical layers/main race:
#      a `worktree_remove.sh` can delete a package directory while a
#      concurrent `make build` in another shell is midway through generating
#      this host-shared directory. Two points are checked and both exclude
#      the file rather than default it in as "declares no keys" or publish a
#      dangling `yaml file://` line: (a) the conflict-detection subprocess
#      reports a read failure explicitly instead of silently skipping the
#      file (a silent skip would let a stale key from a DIFFERENT file in
#      the same conflict group win uncontested, which is the same
#      first-loaded-wins hazard exit 6 exists to prevent); (b) the publish
#      loop re-checks the file's existence immediately before writing its
#      line, closing the remaining window between conflict-check and
#      publish. Exit-code precedence: 4 (shape) wins over 6 (conflict) wins
#      over 7 (vanished) — each is a stricter, earlier-established gate than
#      the one after it, and a run that hit more than one still reports the
#      strictest so `make build` / bootstrap.sh keep today's failure
#      semantics for the already-shipped cases (4, 6). Like 4 and 6, the
#      directory is still written, minus the excluded file(s), so a
#      transient worktree-removal race never costs every OTHER key on the
#      host — only the raced one, until the next regeneration. Treated as
#      transient/soft by the Makefile stamp (like exit 3 — a note, stamp
#      left stale, next `make build` retries) and by bootstrap.sh
#      (#659 round-3 pre-push review).
#
# Test-only hooks (ROSDEP_LOCAL_SOURCES_TEST_VANISH,
# ROSDEP_LOCAL_SOURCES_TEST_VANISH_AFTER_CONFLICT), #659 round-3: reproduce
# the exit-7 race deterministically WITHOUT deleting anything — shipped
# code must never `rm` an env-supplied path. Each hook only makes this
# script TREAT the named path as missing at its checkpoint (conflict-check
# read / publish existence recheck), and only when that path is one of
# this run's own discovered files; any other value is ignored, with a
# stderr note. Both are no-ops when unset, which is every real invocation.

set -euo pipefail

if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
    echo "Error: execute this script, don't source it." >&2
    return 1 2>/dev/null || exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_worktree_helpers.sh
source "$SCRIPT_DIR/_worktree_helpers.sh"   # wt_discover_local_rosdep_yamls

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
# Three outcomes, deliberately distinguished — they are not the same condition
# and they do not get the same response:
#   flock not installed   -> proceed unserialized (the swap still keeps readers
#                            consistent; this is the documented degraded mode)
#   lock file unopenable  -> proceed unserialized, but say WHY (the .rosdep tree
#                            is written by the host uid, by root in the agent
#                            entrypoint and by the agent user, so a
#                            permission/ownership failure here is plausible and
#                            "flock unavailable" would misdirect the debugging)
#   flock TIMED OUT       -> FAIL (exit 5). A timeout means another writer
#                            demonstrably holds the lock; falling through would
#                            put two writers on the same published slot, both
#                            rm -rf'ing and cp'ing into one BUILD_DIR, and the
#                            swap would then publish a mid-mutation directory.
#                            Unserialized is only safe when nobody else is
#                            writing, which is exactly what a timeout disproves.
if ! command -v flock >/dev/null 2>&1; then
    echo "Warning: flock is not installed — regenerating unserialized." >&2
elif ! exec 9>"$LOCK_FILE"; then
    echo "Warning: could not open lock file $LOCK_FILE (check ownership and" \
         "permissions on $OUT_PARENT) — regenerating unserialized." >&2
elif ! flock -w "$LOCK_TIMEOUT" 9; then
    echo "Error: timed out after ${LOCK_TIMEOUT}s waiting for $LOCK_FILE —" \
         "another regeneration is still holding it." >&2
    echo "       Refusing to regenerate unserialized: a second writer would" \
         "share the build slot and publish a half-written directory." >&2
    echo "       Retry, or raise ROSDEP_SOURCES_LOCK_TIMEOUT." >&2
    exit 5
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

# Discover under layers/main AND registered layer worktrees (#659). Sorted
# for determinism: the generated file is compared by the Makefile stamp and
# read by humans; discovery order must not churn it.
mapfile -t local_yamls < <(wt_discover_local_rosdep_yamls "$ROOT_DIR" | LC_ALL=C sort)
LOCAL_LIST="$BUILD_DIR/30-workspace-local.list"
{
    echo "# Generated by .agent/scripts/rosdep_local_sources.sh — do not edit."
    echo "# One line per project repo carrying a root rosdep.yaml (#654, #659)."
} > "$LOCAL_LIST"

# Shape gate (#654): these files drive a root-level `rosdep install`, and
# rosdep's own format also accepts pip/npm/gem/source rules. A rejected file is
# EXCLUDED here — never merely warned about — and the script exits 4 so the
# caller can decide (the Makefile stamp fails the build; bootstrap.sh notes it).
# A validator that cannot run at all (exit 3) is treated the same way: fail
# closed, because "unvalidated" and "invalid" reach root identically. Runs
# regardless of which glob (layers/main or a worktree) a file came from.
VALIDATOR="$SCRIPT_DIR/rosdep_yaml_validate.sh"

rejected_count=0
passed_yamls=()
for yaml in "${local_yamls[@]}"; do
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
    passed_yamls+=("$yaml")
done

# Test-only hook (#659, simulation-only per round-3 pre-push review — never
# deletes): lets the regression suite deterministically reproduce "a
# shape-gate-passed file vanishes before the conflict-check subprocess reads
# it" without racing a real background deletion against this script's own
# execution speed, and WITHOUT an `rm` in shipped code — an unconfined
# `rm -f` of an env-supplied path is the same risk class as the unapproved
# deletion AGENTS.md's Never section rules out, just reached through a code
# path instead of an operator action. Instead of deleting, the named path is
# treated as missing by the conflict-check read below (via
# ROSDEP_LOCAL_SOURCES_SIMULATE_VANISH, passed to the python subprocess) —
# and only when it is one of THIS run's own discovered files
# (passed_yamls); any other value is ignored, with a note, since simulating
# a vanish for a path this run never touched would be meaningless. Unset in
# every real run; harmless no-op then.
declare -A simulate_vanish_conflict=()
if [ -n "${ROSDEP_LOCAL_SOURCES_TEST_VANISH:-}" ]; then
    _tv="$ROSDEP_LOCAL_SOURCES_TEST_VANISH"
    _tv_is_own=0
    for yaml in "${passed_yamls[@]}"; do
        if [ "$yaml" = "$_tv" ]; then
            _tv_is_own=1
            break
        fi
    done
    if [ "$_tv_is_own" -eq 1 ]; then
        simulate_vanish_conflict["$_tv"]=1
    else
        echo "Note: ROSDEP_LOCAL_SOURCES_TEST_VANISH='$_tv' is not one of" \
             "this run's own discovered files — ignored." >&2
    fi
fi

# Conflict detection (#659): rosdep's own root-level merge across multiple
# sources is first-loaded-wins and does NOT error on a duplicate key (checked
# against the installed rosdep2 source — RosdepDefinition.reverse_merge). A
# key declared with DIFFERENT package lists by two or more files (layers/main
# vs. a worktree, or worktree vs. worktree) must not resolve to whichever
# file happens to sort first — that would be the same silent last-wins the
# owner's checkpoint decision explicitly rejected. So: parse every
# shape-gate-passed file's top-level keys, canonicalize each key's value
# (OS-name-sorted, package-list-sorted, so ordering differences alone never
# manufacture a false conflict), and group by key across ALL files. A key
# with more than one distinct canonical value is a conflict; every file that
# declared ANY conflicting key is excluded from the generated list (the same
# per-file exclusion granularity the shape gate already uses — a file loses
# a co-located non-conflicting key too, same trade-off). A key with exactly
# one distinct value across every file that declares it is NOT a conflict —
# it dedupes cleanly (e.g. an in-progress merge: a worktree's copy and the
# already-merged layers/main copy are the same content).
conflict_key_count=0
vanished_count=0
declare -A excluded_by_conflict=()
declare -A vanished_files=()
if [ "${#passed_yamls[@]}" -gt 0 ]; then
    conflict_report="$(ROSDEP_LOCAL_SOURCES_SIMULATE_VANISH="$(printf '%s\n' "${!simulate_vanish_conflict[@]}")" \
        python3 - "${passed_yamls[@]}" <<'PY'
import os, sys, yaml, json
from collections import defaultdict

# #659 round-3: simulation-only test hook, never a real deletion — see the
# bash-side comment above this subprocess call. A path listed here is
# treated as unreadable without ever touching the filesystem entry itself.
simulated_vanish = set(
    p for p in os.environ.get("ROSDEP_LOCAL_SOURCES_SIMULATE_VANISH", "").split("\n") if p
)

key_entries = defaultdict(list)
unreadable = []
for path in sys.argv[1:]:
    if path in simulated_vanish:
        unreadable.append(path)
        continue
    try:
        with open(path, encoding="utf-8") as f:
            data = yaml.safe_load(f)
    except Exception:
        # #659: the shape gate accepted this file, but it can still vanish
        # (or become otherwise unreadable — permissions, a worktree teardown
        # racing this run) before THIS read. That is a real, routine race now
        # that worktree churn feeds this generator, not just a theoretical
        # layers/main one. Report it explicitly and let the bash side
        # exclude it — silently treating a read failure as "declares no
        # keys" would let a DIFFERENT file's declaration of the same key win
        # uncontested, the same first-loaded-wins hazard the conflict check
        # exists to prevent, and would never surface the file's own
        # (unread) declaration as a name to investigate.
        unreadable.append(path)
        continue
    if not isinstance(data, dict):
        continue
    for key, val in data.items():
        canon = {}
        if isinstance(val, dict):
            for os_name, pkgs in val.items():
                # Dedupe before sorting: two semantically-identical
                # declarations (`[foo]` vs. `[foo, foo]`) must canonicalize
                # to the same value, or an accidental in-file duplicate could
                # spuriously read as a conflict against a clean declaration
                # elsewhere.
                canon[os_name] = sorted(set(pkgs)) if isinstance(pkgs, list) else pkgs
        key_entries[key].append((path, json.dumps(canon, sort_keys=True)))

excluded = set()
lines = []
for key in sorted(key_entries):
    entries = key_entries[key]
    values = sorted(set(v for _, v in entries))
    if len(values) > 1:
        files = sorted(set(f for f, _ in entries))
        excluded.update(files)
        lines.append("CONFLICT\t%s\t%s\t%s" % (key, ";".join(files), " | ".join(values)))

for line in lines:
    print(line)
for f in sorted(excluded):
    print("EXCLUDE\t%s" % f)
for f in sorted(unreadable):
    print("UNREADABLE\t%s" % f)
PY
)"
    if [ -n "$conflict_report" ]; then
        while IFS=$'\t' read -r tag a b c; do
            case "$tag" in
                CONFLICT)
                    echo "Error: rosdep key '$a' is declared with conflicting package lists:" >&2
                    echo "   files: $b" >&2
                    echo "   values: $c" >&2
                    conflict_key_count=$((conflict_key_count + 1))
                    ;;
                EXCLUDE)
                    excluded_by_conflict["$a"]=1
                    ;;
                UNREADABLE)
                    # #659 round-2: this file passed the shape gate but the
                    # conflict-detection subprocess could not read it a
                    # moment later (deleted — routinely a worktree teardown
                    # racing this generator, not just a theoretical race).
                    # Exclude it and say so, rather than defaulting it in as
                    # "declares no keys": a silent skip here would let a
                    # DIFFERENT file's declaration of the same key win
                    # uncontested, undermining the conflict check itself.
                    vanished_files["$a"]=1
                    vanished_count=$((vanished_count + 1))
                    echo "Error: '$a' became unreadable during conflict" \
                         "detection (likely deleted) — excluded from" \
                         "$LOCAL_LIST (#659 race)." >&2
                    ;;
            esac
        done <<< "$conflict_report"
        for f in "${!excluded_by_conflict[@]}"; do
            echo "Error: '$f' excluded from $LOCAL_LIST — conflicting key(s), see above." >&2
        done
    fi
fi

# Test-only hook (#659, simulation-only per round-3 pre-push review — never
# deletes): lets the regression suite deterministically reproduce "a file
# survives conflict-check but vanishes before publish" — the second half of
# the race window this exit code closes — without an `rm` in shipped code.
# The named path is treated as missing by the publish loop's existence
# re-check below, and only when it is one of THIS run's own discovered
# files (passed_yamls); any other value is ignored, with a note. Unset in
# every real run; harmless no-op then.
declare -A simulate_vanish_publish=()
if [ -n "${ROSDEP_LOCAL_SOURCES_TEST_VANISH_AFTER_CONFLICT:-}" ]; then
    _tv2="$ROSDEP_LOCAL_SOURCES_TEST_VANISH_AFTER_CONFLICT"
    _tv2_is_own=0
    for yaml in "${passed_yamls[@]}"; do
        if [ "$yaml" = "$_tv2" ]; then
            _tv2_is_own=1
            break
        fi
    done
    if [ "$_tv2_is_own" -eq 1 ]; then
        simulate_vanish_publish["$_tv2"]=1
    else
        echo "Note: ROSDEP_LOCAL_SOURCES_TEST_VANISH_AFTER_CONFLICT='$_tv2'" \
             "is not one of this run's own discovered files — ignored." >&2
    fi
fi

yaml_count=0
for yaml in "${passed_yamls[@]}"; do
    if [ -n "${excluded_by_conflict[$yaml]+x}" ]; then
        continue
    fi
    if [ -n "${vanished_files[$yaml]+x}" ]; then
        continue
    fi
    # Re-check existence immediately before writing the publish line — the
    # remaining half of the race window (#659 round-2): a file can pass
    # every earlier check and still vanish before THIS write. Never publish
    # a `yaml file://` line for a path that is not there right now. The
    # simulate_vanish_publish check is the test-only hook above — it never
    # deletes, it only makes this recheck behave as if the file were gone.
    if [ -n "${simulate_vanish_publish[$yaml]+x}" ] || [ ! -f "$yaml" ]; then
        vanished_count=$((vanished_count + 1))
        echo "Error: '$yaml' vanished before it could be published to" \
             "$LOCAL_LIST — excluded (#659 race)." >&2
        continue
    fi
    echo "yaml file://$yaml" >> "$LOCAL_LIST"
    yaml_count=$((yaml_count + 1))
done

# ---- publish: atomic symlink swap -------------------------------------------
# rename(2) over an existing symlink is atomic, so <out_dir> never transiently
# vanishes or appears half-built. The one-time migration of a pre-existing REAL
# directory is handled below, inside the lock.
TMP_LINK="$OUT_PARENT/.$OUT_BASE.link.$$"
rm -f "$TMP_LINK"
ln -s "$SLOT_ROOT_NAME/$slot" "$TMP_LINK"
if [ -e "$OUT_DIR" ] && [ ! -L "$OUT_DIR" ]; then
    # MIGRATION: a real directory (an older generation of this script, or a
    # hand-made one) sits at the published path. A symlink cannot be renamed
    # over a directory, so the two have to trade places. It is NOT `rm -rf`'d
    # first: that left the published path absent for as long as removing a
    # whole directory tree takes, and setup.bash's `[ -d ]` gate and rosdep
    # itself both read that ENOENT as "no workspace sources at all".
    #
    # renameat2(RENAME_EXCHANGE) swaps the two paths in one atomic step, so a
    # reader sees either the old directory or the new symlink and never a gap.
    # Where the kernel, the C library or the filesystem does not support it,
    # the fallback renames the old directory ASIDE and publishes immediately
    # after — two renames in ONE process, so the residual gap is a single
    # rename(2) rather than the fork+exec of a second `mv`. Either way the old
    # content is deleted only AFTER the new symlink is published.
    ASIDE="$OUT_PARENT/.$OUT_BASE.aside.$$"
    rm -rf "$ASIDE"
    if command -v python3 >/dev/null 2>&1; then
        ROSDEP_SOURCES_FORCE_FALLBACK="${ROSDEP_SOURCES_FORCE_FALLBACK:-0}" \
        python3 - "$TMP_LINK" "$OUT_DIR" "$ASIDE" <<'PY'
import ctypes, os, sys

new_link, published, aside = sys.argv[1], sys.argv[2], sys.argv[3]

def exchange():
    """Atomically swap `new_link` and `published` (renameat2 RENAME_EXCHANGE).

    Returns True on success. Every failure is a fallback, never a crash: the
    call is unavailable on pre-2.28 glibc, on pre-3.15 kernels and on some
    filesystems, and each of those reports it differently.
    """
    if os.environ.get("ROSDEP_SOURCES_FORCE_FALLBACK") == "1":
        return False
    try:
        libc = ctypes.CDLL("libc.so.6", use_errno=True)
        renameat2 = libc.renameat2
    except (OSError, AttributeError):
        return False
    AT_FDCWD, RENAME_EXCHANGE = -100, 2
    rc = renameat2(AT_FDCWD, os.fsencode(new_link),
                   AT_FDCWD, os.fsencode(published), RENAME_EXCHANGE)
    return rc == 0

if exchange():
    # `new_link` now holds the old real directory; the caller removes it.
    os.rename(new_link, aside)
else:
    os.rename(published, aside)
    os.rename(new_link, published)
PY
    else
        echo "Warning: python3 not found — migrating with two mv(1) calls;" \
             "$OUT_DIR is briefly absent." >&2
        mv -T "$OUT_DIR" "$ASIDE"
        mv -T "$TMP_LINK" "$OUT_DIR"
    fi
    rm -rf "$ASIDE"
else
    mv -T "$TMP_LINK" "$OUT_DIR"
fi

echo "Aggregated $yaml_count project rosdep.yaml file(s) into $OUT_DIR" \
     "(over ${#system_lists[@]} system source list(s))"
if [ "$yaml_count" -gt 0 ]; then
    echo "Run 'ROSDEP_SOURCE_PATH=$OUT_DIR rosdep update' to refresh the cache."
fi
if [ "$rejected_count" -gt 0 ]; then
    echo "Error: $rejected_count project rosdep.yaml file(s) rejected (see above)." >&2
    exit 4
fi
if [ "$conflict_key_count" -gt 0 ]; then
    echo "Error: $conflict_key_count rosdep key(s) conflict across files (see above)." >&2
    exit 6
fi
if [ "$vanished_count" -gt 0 ]; then
    echo "Error: $vanished_count project rosdep.yaml file(s) vanished mid-run" \
         "and were excluded (see above, #659)." >&2
    exit 7
fi
