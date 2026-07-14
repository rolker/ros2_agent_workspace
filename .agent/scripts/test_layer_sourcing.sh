#!/bin/bash
# .agent/scripts/test_layer_sourcing.sh
# Regression guard for runtime layer chaining (ADR-0016, issue #559).
#
# Checks:
#   1. STATIC (hard fail, runs everywhere incl. CI): no workspace script
#      sources a layer's baked chained install/setup.bash — runtime chaining
#      must use install/local_setup.bash.
#   2. RUNTIME ORDER (hard fail, skipped when no layers are built): sourcing
#      .agent/scripts/setup.bash in a clean shell must yield canonical overlay
#      precedence in AMENT_PREFIX_PATH — topmost layer first, underlay just
#      above /opt/ros/jazzy (i.e., layers.txt order reversed).
#   3. CHAIN PURITY (warning only): each built layer's baked parent chain
#      (COLCON_CURRENT_PREFIX lines in install/setup.sh) should reference only
#      /opt/ros/jazzy and strictly-lower layers. Pollution means the layer was
#      built with higher layers sourced (the pre-#559 build.sh bug) and needs
#      the one-time bottom-up clean rebuild to heal. Warning-only because
#      pre-existing installs are expected to be polluted until healed.
#
# Exit codes: 0 = pass (or skipped), 1 = check failed.

if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
    echo "Error: This script should be executed, not sourced."
    echo "  Run: ${BASH_SOURCE[0]} $*"
    return 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"
# LAYERS_BASE is resolved (worktree-aware) alongside LAYERS_CONFIG below.

FAILURES=0

echo "=== Layer sourcing regression guard (ADR-0016 / #559) ==="

# --- Check 1: static — no chained install/setup.bash sourcing in scripts ---
STATIC_TARGETS=(
    "$SCRIPT_DIR/setup.bash"
    "$SCRIPT_DIR/build.sh"
    "$SCRIPT_DIR/test.sh"
    "$SCRIPT_DIR/verify_change.sh"
    "$SCRIPT_DIR/worktree_create.sh"
)
STATIC_BAD=0
for f in "${STATIC_TARGETS[@]}"; do
    [ -f "$f" ] || continue
    # Match actual source statements (incl. heredoc-generated ones), not comments.
    if grep -nE '^[^#]*source[^#]*install/setup\.bash' "$f" > /dev/null; then
        echo "❌ Check 1: $f sources a baked chained install/setup.bash:"
        grep -nE '^[^#]*source[^#]*install/setup\.bash' "$f" | sed 's/^/     /'
        STATIC_BAD=1
    fi
done
if [ "$STATIC_BAD" -eq 0 ]; then
    echo "✅ Check 1: no baked-chain sourcing in workspace scripts"
else
    echo "   Layer sourcing must use install/local_setup.bash (see ADR-0016)."
    FAILURES=$((FAILURES + 1))
fi

# --- Resolve layer list + built-layer base (worktree-aware) ---
# LAYERS_CONFIG and LAYERS_BASE must resolve to the SAME workspace root. In a
# worktree, configs/ (gitignored) is absent and built layers live under the
# MAIN root's layers/main (a workspace worktree symlinks it; a layer worktree
# does not). Derive one MAIN_ROOT and use it for both, so Checks 2-3 run
# against the real built layers instead of silently skipping in a layer
# worktree (and so the /layers/main/ sed below stays correct — see Check 2).
MAIN_ROOT="$ROOT_DIR"
if [ ! -f "$ROOT_DIR/configs/manifest/layers.txt" ]; then
    if [[ "$ROOT_DIR" == *"/.workspace-worktrees/"* ]]; then
        MAIN_ROOT="$(dirname "$(dirname "$ROOT_DIR")")"
    elif [[ "$ROOT_DIR" == *"/layers/worktrees/"* ]]; then
        MAIN_ROOT="$(dirname "$(dirname "$(dirname "$ROOT_DIR")")")"
    fi
fi
LAYERS_CONFIG="$MAIN_ROOT/configs/manifest/layers.txt"
LAYERS_BASE="$MAIN_ROOT/layers/main"
if [ ! -f "$LAYERS_CONFIG" ]; then
    echo "⏭️  Checks 2-3 skipped: no layers.txt at $LAYERS_CONFIG"
    exit "$((FAILURES > 0 ? 1 : 0))"
fi
mapfile -t LAYERS < <(grep -v '^[[:space:]]*$' "$LAYERS_CONFIG" | grep -v '^#' | sed 's/[[:space:]]*$//')

BUILT=()
for layer in "${LAYERS[@]}"; do
    [ -f "$LAYERS_BASE/${layer}_ws/install/local_setup.bash" ] && BUILT+=("$layer")
done
if [ ${#BUILT[@]} -eq 0 ]; then
    echo "⏭️  Checks 2-3 skipped: no built layers under $LAYERS_BASE (expected in CI)"
    exit "$((FAILURES > 0 ? 1 : 0))"
fi

# --- Check 2: runtime precedence order ---
# Source in a scrubbed shell so a pre-sourced caller environment can't skew
# the order, then print first-occurrence layer names from AMENT_PREFIX_PATH.
# Pass ROS2_LAYERS_BASE so setup.bash sources the SAME layers we detected as
# BUILT: in a layer worktree it would otherwise default LAYERS_BASE to the
# worktree path and its AMENT entries wouldn't be under /layers/main/ (the
# path the sed below keys on). setup.bash honors this override and won't
# clobber it (see setup.bash's ROS2_LAYERS_BASE guard).
ACTUAL_ORDER=$(env -i HOME="$HOME" PATH="/usr/local/bin:/usr/bin:/bin" TERM=dumb \
    ROS2_LAYERS_BASE="$LAYERS_BASE" \
    bash --noprofile --norc -c \
    "source '$SCRIPT_DIR/setup.bash' > /dev/null 2>&1
     echo \"\$AMENT_PREFIX_PATH\" | tr ':' '\n' \
       | sed -E 's|.*/layers/main/([a-z0-9_]+)_ws/.*|\1|; s|^/opt/ros/[a-z]+.*|ROS_BASE|' \
       | uniq | tr '\n' ' '")
ACTUAL_ORDER="${ACTUAL_ORDER% }"

EXPECTED_ORDER=""
for ((i=${#BUILT[@]}-1; i>=0; i--)); do
    EXPECTED_ORDER+="${BUILT[$i]} "
done
EXPECTED_ORDER+="ROS_BASE"

if [ "$ACTUAL_ORDER" == "$EXPECTED_ORDER" ]; then
    echo "✅ Check 2: canonical overlay precedence ($ACTUAL_ORDER)"
else
    echo "❌ Check 2: AMENT_PREFIX_PATH layer order is not canonical."
    echo "     expected: $EXPECTED_ORDER"
    echo "     actual:   $ACTUAL_ORDER"
    echo "   Overlays must outrank underlays (topmost layer first)."
    FAILURES=$((FAILURES + 1))
fi

# --- Check 3: baked-chain purity (warning only) ---
POLLUTED=()
for ((i=0; i<${#LAYERS[@]}; i++)); do
    layer="${LAYERS[$i]}"
    chain_file="$LAYERS_BASE/${layer}_ws/install/setup.sh"
    [ -f "$chain_file" ] || continue
    bad_refs=""
    while IFS= read -r prefix; do
        case "$prefix" in
            /opt/ros/*) continue ;;
            *'$'*) continue ;;  # self-reference via shell variable, not a parent
        esac
        parent_ok=0
        for ((j=0; j<i; j++)); do
            if [[ "$prefix" == *"/${LAYERS[$j]}_ws/install" ]]; then
                parent_ok=1
                break
            fi
        done
        if [ "$parent_ok" -eq 0 ]; then
            ref_name=$(basename "$(dirname "$prefix")")
            bad_refs="$bad_refs ${ref_name%_ws}"
        fi
    done < <(grep -oE '^COLCON_CURRENT_PREFIX="[^"]+"' "$chain_file" | cut -d'"' -f2)
    [ -n "$bad_refs" ] && POLLUTED+=("$layer: non-parent chain refs →$bad_refs")
done
if [ ${#POLLUTED[@]} -eq 0 ]; then
    echo "✅ Check 3: baked parent chains are clean (jazzy + lower layers only)"
else
    echo "⚠️  Check 3 (warning): polluted baked parent chains detected:"
    printf '     %s\n' "${POLLUTED[@]}"
    echo "   These layers were built with higher layers sourced (pre-#559"
    echo "   build.sh). Heal with a one-time bottom-up clean rebuild:"
    echo "     rm -rf layers/main/*_ws/{build,install,log} && make build"
    echo "   (Warning only — runtime chaining ignores baked chains, but other"
    echo "   consumers sourcing install/setup.bash directly inherit them.)"
fi

if [ "$FAILURES" -gt 0 ]; then
    echo "=== FAILED: $FAILURES check(s) ==="
    exit 1
fi
echo "=== PASSED ==="
exit 0
