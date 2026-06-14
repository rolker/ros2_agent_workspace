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
# This is the single source of truth for the gather, called by both build
# entry points: docker_run_agent.sh --build and `make agent-build`. The CALLER
# owns removing the staging dir after the build (it knows when the build is
# done); this script only (re)populates it idempotently.
#
# Usage:
#   stage_rosdep_manifests.sh <workspace_root> [<stage_dir>]
#     workspace_root  required — path whose layers/main/*_ws/src/ holds the repos
#     stage_dir       optional — default <workspace_root>/.devcontainer/agent/.rosdep-manifests
#
# Prints the staged manifest count. Exits 0 even when zero manifests are found
# (e.g. layers not yet checked out) — the Dockerfile COPY of an empty dir
# succeeds and the bake degrades to a no-op, deferring deps to launch time.

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

rm -rf "$STAGE_DIR"
mkdir -p "$STAGE_DIR"

manifest_count=0
for src_dir in "$ROOT_DIR"/layers/main/*_ws/src; do
    [ -d "$src_dir" ] || continue
    while IFS= read -r -d '' pkgxml; do
        rel="${pkgxml#"$ROOT_DIR"/}"
        dest="$STAGE_DIR/$rel"
        mkdir -p "$(dirname "$dest")"
        cp "$pkgxml" "$dest"
        manifest_count=$((manifest_count + 1))
    done < <(find "$src_dir" -name package.xml -type f -print0)
done

echo "Staged $manifest_count layer package.xml manifest(s) into $STAGE_DIR"
