#!/bin/bash
# .agent/scripts/lib/scratchpad_helpers.sh
# Helper functions for safe scratchpad file operations in multi-agent scenarios

# Get the scratchpad directory path
get_scratchpad_dir() {
    echo ".agent/scratchpad"
}

# Generate a unique filename in the scratchpad directory
# Args:
#   $1: base_name (e.g., "issue_body", "build_report")
#   $2: extension (optional, defaults to ".txt")
# Returns: Full path to unique file in scratchpad
#
# Example:
#   BODY_FILE=$(scratchpad_file "issue_body" ".md")
#   cat > "$BODY_FILE" << 'EOF'
#   My content
#   EOF
#   gh issue create --body-file "$BODY_FILE"
#   rm "$BODY_FILE"  # Clean up after use
scratchpad_file() {
    local base_name="$1"
    local extension="${2:-.txt}"
    local agent_id="${AGENT_ID:-unknown}"
    local timestamp=$(date +%s%N 2>/dev/null || date +%s)  # nanoseconds if available, else seconds
    local pid=$$
    
    # Include PID for additional uniqueness
    echo ".agent/scratchpad/${agent_id}_${base_name}_${timestamp}_${pid}${extension}"
}

# Generate a unique filename with agent subdirectory isolation
# Args:
#   $1: base_name (e.g., "issue_body", "analysis")
#   $2: extension (optional, defaults to ".txt")
# Returns: Full path to unique file in agent-specific subdirectory
#
# Example:
#   REPORT_FILE=$(scratchpad_file_namespaced "analysis" ".md")
#   cat > "$REPORT_FILE" << 'EOF'
#   Analysis results
#   EOF
scratchpad_file_namespaced() {
    local base_name="$1"
    local extension="${2:-.txt}"
    local agent_id="${AGENT_ID:-unknown}"
    local timestamp=$(date +%s%N 2>/dev/null || date +%s)
    
    # Create agent-specific subdirectory
    local agent_dir=".agent/scratchpad/${agent_id}"
    mkdir -p "$agent_dir"
    
    echo "${agent_dir}/${base_name}_${timestamp}${extension}"
}

# Clean up files created by this agent
# Args:
#   $1: age_in_seconds (optional, only delete files older than this)
# Example:
#   scratchpad_cleanup 3600  # Clean up files older than 1 hour
scratchpad_cleanup() {
    local age_in_seconds="${1:-0}"
    local agent_id="${AGENT_ID:-unknown}"
    
    if [ "$agent_id" = "unknown" ]; then
        echo "⚠️  Warning: AGENT_ID not set, cannot safely clean up scratchpad"
        return 1
    fi
    
    if [ "$age_in_seconds" -gt 0 ]; then
        echo "🧹 Cleaning up scratchpad files for agent '$agent_id' older than $age_in_seconds seconds..."
        find .agent/scratchpad -name "${agent_id}_*" -type f -mmin "+$(($age_in_seconds / 60))" -delete 2>/dev/null || true
        # Also clean up agent-specific directory if it exists
        if [ -d ".agent/scratchpad/${agent_id}" ]; then
            find ".agent/scratchpad/${agent_id}" -type f -mmin "+$(($age_in_seconds / 60))" -delete 2>/dev/null || true
            # Remove directory if empty
            rmdir ".agent/scratchpad/${agent_id}" 2>/dev/null || true
        fi
    else
        echo "🧹 Cleaning up all scratchpad files for agent '$agent_id'..."
        rm -f .agent/scratchpad/${agent_id}_* 2>/dev/null || true
        # Also clean up agent-specific directory if it exists
        if [ -d ".agent/scratchpad/${agent_id}" ]; then
            rm -rf ".agent/scratchpad/${agent_id}"
        fi
    fi
}

# List files created by this agent
scratchpad_list_mine() {
    local agent_id="${AGENT_ID:-unknown}"
    
    echo "Files in scratchpad for agent '$agent_id':"
    ls -lh .agent/scratchpad/${agent_id}_* 2>/dev/null || echo "  (none)"
    
    if [ -d ".agent/scratchpad/${agent_id}" ]; then
        echo ""
        echo "Files in agent subdirectory:"
        ls -lhR ".agent/scratchpad/${agent_id}" 2>/dev/null || echo "  (none)"
    fi
}

# Create a temporary file that will be automatically cleaned up
# This is useful for short-lived files within a single script
# Args:
#   $1: base_name
#   $2: extension (optional)
# Returns: Full path to temporary file
# Note: File must be manually deleted, but will be cleaned up by scratchpad_cleanup
scratchpad_temp() {
    scratchpad_file "$@"
}

# Check if scratchpad directory exists, create if not
ensure_scratchpad() {
    mkdir -p .agent/scratchpad
}
