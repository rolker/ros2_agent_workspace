#!/bin/bash
# .agent/scripts/stage_rosdep_manifests.sh
# Gather the workspace's layer package.xml manifests into a staging dir under
# the agent-image build context, so the Dockerfile can COPY them and bake their
# rosdep dependencies at build time (#520).
#
# The build context (.devcontainer/agent/) holds no layer source — layers/ is
# gitignored and mounted at runtime, never copied — so the manifests must be
# staged here, host-side, where layers/ exists. The gather is RECURSIVE: most
# project repos are multi-package and nest manifests in subdirs
# (marine_control/marine_control_interfaces/package.xml, …), so a shallow
# src/*/package.xml glob would miss ~80% of them. Each manifest keeps its path
# relative to the workspace root so it lands in its own directory
# (`rosdep install --from-paths` reads one package.xml per directory).
#
# This is the single source of truth for the gather. It has exactly ONE caller:
# docker_run_agent.sh's build block — the workspace's single image build path
# since #604, which `make agent-build` reaches via --build-only. The CALLER owns
# removing the staging dir after the build (it knows when the build is done);
# this script only (re)populates it idempotently.
#
# It ALSO stages each project repo's root rosdep.yaml — keys that have no
# upstream ros/rosdistro entry yet (#654) — into <stage_dir>/rosdep-local/.
# Without them, a package.xml <depend> on such a key lands on the Dockerfile's
# skip_keys list and is silently omitted from the image. They go inside the same
# staged tree rather than a second build-context path so the caller keeps one
# lock, one trap and one COPY; the directory holds no package.xml, so
# `rosdep install --from-paths` walks straight past it. The Dockerfile turns
# them into a ROSDEP_SOURCE_PATH overlay. The workspace-side equivalent is
# rosdep_local_sources.sh; see .agent/knowledge/dependency_policy.md.
#
# Usage:
#   stage_rosdep_manifests.sh <workspace_root> [<stage_dir>]
#     workspace_root  required — path whose layers/main/*_ws/src/ holds the repos
#     stage_dir       optional — default <workspace_root>/.devcontainer/agent/.rosdep-manifests
#
# Prints the staged manifest count. Exits 0 even when zero manifests are found
# (e.g. layers not yet checked out) — the Dockerfile COPY of an empty dir
# succeeds and the bake degrades to a no-op, deferring deps to launch time.
# Exit 4: one or more rosdep.yaml files were rejected by the shape gate
# (rosdep_yaml_validate.sh) or could not be validated. They are NOT staged, and
# the non-zero exit stops the image build — these keys drive a root-level
# `rosdep install` inside the bake, so an unvalidated one must not get there.

set -euo pipefail

if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
    echo "Error: execute this script, don't source it." >&2
    return 1 2>/dev/null || exit 1
fi

if [ "$#" -lt 1 ] || [ -z "${1:-}" ]; then
    echo "Usage: stage_rosdep_manifests.sh <workspace_root> [<stage_dir>]" >&2
    exit 2
fi

ROOT_DIR="$1"
STAGE_DIR="${2:-$ROOT_DIR/.devcontainer/agent/.rosdep-manifests}"

if [ ! -d "$ROOT_DIR" ]; then
    echo "Error: workspace_root '$ROOT_DIR' is not a directory." >&2
    exit 2
fi

# Canonicalize: the `rel` paths reported below are computed by stripping
# "$ROOT_DIR/" off a matched path, which only works when both sides are in the
# same (absolute) form. Same requirement as rosdep_local_sources.sh.
ROOT_DIR="$(CDPATH='' cd -- "$ROOT_DIR" && pwd)"
case "$STAGE_DIR" in
    /*) ;;
    *)  STAGE_DIR="$PWD/$STAGE_DIR" ;;
esac

rm -rf "$STAGE_DIR"
mkdir -p "$STAGE_DIR"

manifest_count=0
skipped_count=0
for src_dir in "$ROOT_DIR"/layers/main/*_ws/src; do
    [ -d "$src_dir" ] || continue
    while IFS= read -r -d '' pkgxml; do
        # Skip packages that colcon/ament/catkin ignore. rosdep does NOT honor
        # COLCON_IGNORE, so a non-built package (e.g. a leftover ROS1 package
        # with deps that aren't rosdep keys) would otherwise be staged and
        # abort the whole bake — `rosdep install --from-paths` fails if ANY
        # key across the set is unresolvable. Walk from the package dir up to
        # src_dir looking for an ignore marker.
        pkgdir="$(dirname "$pkgxml")"
        ignored=0
        d="$pkgdir"
        while :; do
            if [ -e "$d/COLCON_IGNORE" ] || [ -e "$d/AMENT_IGNORE" ] || [ -e "$d/CATKIN_IGNORE" ]; then
                ignored=1
                break
            fi
            [ "$d" = "$src_dir" ] && break
            d="$(dirname "$d")"
        done
        if [ "$ignored" = 1 ]; then
            skipped_count=$((skipped_count + 1))
            continue
        fi
        rel="${pkgxml#"$ROOT_DIR"/}"
        dest="$STAGE_DIR/$rel"
        mkdir -p "$(dirname "$dest")"
        cp "$pkgxml" "$dest"
        manifest_count=$((manifest_count + 1))
    done < <(find "$src_dir" -name package.xml -type f -print0)
done

# Project-repo rosdep.yaml files (#654). Shallow glob: one per repo root, the
# declaration location rosdep_local_sources.sh also uses. Named after the repo
# directory so the Dockerfile's generated source list is readable, and sorted so
# a build context (and therefore the image layer's cache key) is deterministic.
LOCAL_DIR="$STAGE_DIR/rosdep-local"
STAGE_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VALIDATOR="$STAGE_SCRIPT_DIR/rosdep_yaml_validate.sh"
local_count=0
rejected_count=0
shopt -s nullglob
local_yamls=("$ROOT_DIR"/layers/main/*_ws/src/*/rosdep.yaml)
if [ "${#local_yamls[@]}" -gt 0 ]; then
    mkdir -p "$LOCAL_DIR"
    while IFS= read -r yaml; do
        # Shape gate (#654) — fail closed, exactly as rosdep_local_sources.sh
        # does: a file that is invalid, or that could not be validated at all,
        # is not staged.
        if [ ! -x "$VALIDATOR" ]; then
            echo "Error: $VALIDATOR is missing — cannot validate '$yaml'." >&2
            rejected_count=$((rejected_count + 1))
            continue
        fi
        if ! "$VALIDATOR" "$yaml"; then
            echo "   not staged for the image bake" >&2
            rejected_count=$((rejected_count + 1))
            continue
        fi
        # Named <layer_ws>__<repo>.yaml, not <repo>.yaml: repo directory names
        # are only unique WITHIN a layer's src/, so two same-named repos under
        # different *_ws/src/ trees silently overwrote each other here while
        # local_count still reported both — one repo's keys would vanish from
        # the image with nothing said.
        repo_dir="$(basename "$(dirname "$yaml")")"
        rel="${yaml#"$ROOT_DIR"/layers/main/}"
        layer_ws="${rel%%/*}"
        cp "$yaml" "$LOCAL_DIR/${layer_ws}__${repo_dir}.yaml"
        local_count=$((local_count + 1))
    done < <(printf '%s\n' "${local_yamls[@]}" | LC_ALL=C sort)
fi

echo "Staged $manifest_count layer package.xml manifest(s) into $STAGE_DIR" \
     "(skipped $skipped_count ignored package(s);" \
     "$local_count local rosdep.yaml file(s))"
if [ "$rejected_count" -gt 0 ]; then
    echo "Error: $rejected_count project rosdep.yaml file(s) rejected (see above)." >&2
    exit 4
fi
