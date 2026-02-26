#!/bin/bash
# discover_governance.sh — Scan workspace and project repos for governance documents.
#
# Outputs a TSV inventory: path<tab>type<tab>size<tab>scope
#   scope: "workspace" or the project repo directory name
#   type:  principles | adr | agent-guide | architecture | agents-config | workspace-context
#   size:  bytes for files, file count for workspace-context directories
#
# Usage:
#   .agent/scripts/discover_governance.sh [--json]
#
# With --json, outputs JSON lines instead of TSV.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

OUTPUT_JSON=false
if [[ "${1:-}" == "--json" ]]; then
    OUTPUT_JSON=true
fi

emit() {
    local path="$1" type="$2" size="$3" scope="$4"
    local relpath="${path#"$ROOT_DIR"/}"
    if $OUTPUT_JSON; then
        printf '{"path":"%s","type":"%s","size":%s,"scope":"%s"}\n' \
            "$relpath" "$type" "$size" "$scope"
    else
        printf '%s\t%s\t%s\t%s\n' "$relpath" "$type" "$size" "$scope"
    fi
}

check_file() {
    local path="$1" type="$2" scope="$3"
    if [[ -f "$path" ]]; then
        local size
        size=$(stat -c%s "$path" 2>/dev/null || stat -f%z "$path" 2>/dev/null || echo 0)
        emit "$path" "$type" "$size" "$scope"
    fi
}

check_dir() {
    local dir="$1" type="$2" scope="$3" pattern="${4:-*.md}"
    if [[ -d "$dir" ]]; then
        while IFS= read -r -d '' file; do
            local size
            size=$(stat -c%s "$file" 2>/dev/null || stat -f%z "$file" 2>/dev/null || echo 0)
            emit "$file" "$type" "$size" "$scope"
        done < <(find "$dir" -maxdepth 1 -name "$pattern" -type f -print0 2>/dev/null)
    fi
}

check_directory() {
    local dir="$1" type="$2" scope="$3"
    if [[ -d "$dir" ]]; then
        local count
        count=$(find "$dir" -type f 2>/dev/null | wc -l)
        emit "$dir" "$type" "$count" "$scope"
    fi
}

scan_scope() {
    local dir="$1" scope="$2"

    check_file "$dir/PRINCIPLES.md"       principles    "$scope"
    check_file "$dir/docs/PRINCIPLES.md"  principles    "$scope"
    check_file "$dir/ARCHITECTURE.md"     architecture  "$scope"
    check_file "$dir/AGENTS.md"           agents-config "$scope"
    check_file "$dir/.agents/README.md"   agent-guide   "$scope"
    check_dir  "$dir/docs/decisions"      adr           "$scope"
    check_directory "$dir/.agents/workspace-context" workspace-context "$scope"
}

# Header (TSV only)
if ! $OUTPUT_JSON; then
    printf '%s\t%s\t%s\t%s\n' "path" "type" "size" "scope"
fi

# Scan workspace root
scan_scope "$ROOT_DIR" "workspace"

# Scan project repos under layers/main/*/src/
if [[ -d "$ROOT_DIR/layers/main" ]]; then
    for ws_dir in "$ROOT_DIR"/layers/main/*_ws; do
        [[ -d "$ws_dir/src" ]] || continue
        for repo_dir in "$ws_dir"/src/*/; do
            [[ -d "$repo_dir" ]] || continue
            repo_name="$(basename "$repo_dir")"
            scan_scope "${repo_dir%/}" "$repo_name"
        done
    done
fi
