#!/bin/bash
# scripts/generate_knowledge.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
KNOWLEDGE_DIR="$ROOT_DIR/.agent/knowledge"

# Ensure directory exists
mkdir -p "$KNOWLEDGE_DIR"

# Helper function to link
link_doc() {
    local src_path="$1"
    local link_name="$2"
    
    local full_src="$ROOT_DIR/$src_path"
    local full_link="$KNOWLEDGE_DIR/$link_name"
    
    if [ -e "$full_src" ]; then
        # Create relative path for symlink
        # We know knowledge dir is .agent/knowledge (2 levels deep)
        # So we need ../../$src_path
        ln -sf "../../$src_path" "$full_link"
        echo "Linked: $link_name -> $src_path"
    else
        echo "Skipped: $src_path (Not found)"
    fi
}

echo "Generating workspace knowledge links..."

# --- Definition of Knowledge Links ---

# System
link_doc "layers/core_ws/src/unh_marine_autonomy/marine_autonomy/README.md" "system__autonomy_overview.md"

# Components
link_doc "layers/core_ws/src/unh_marine_autonomy/mission_manager/mission_manager/README.md" "component__mission_manager.md"
link_doc "layers/core_ws/src/unh_marine_autonomy/helm_manager/README.md" "component__helm_manager.md"
link_doc "layers/core_ws/src/marine_ais/marine_ais_msgs/README.md" "component__marine_ais.md"

# UI
link_doc "layers/ui_ws/src/camp/README.md" "component__camp_ui.md"
link_doc "layers/ui_ws/src/camp/docs" "architecture__camp_docs"

# Simulation
link_doc "layers/simulation_ws/src/unh_marine_simulation/README.md" "component__simulation.md"

echo "Done."
