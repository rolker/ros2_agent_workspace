#!/usr/bin/env bash
# generate_make_skills.sh — Generate Claude Code slash commands from Makefile targets
#
# Parses the Makefile's .PHONY line and help text to create a /make_<target> skill
# for each target. Generated skills live in .claude/skills/make_<target>/SKILL.md
# and are gitignored (only hand-crafted skills are committed).
#
# Usage:
#   .agent/scripts/generate_make_skills.sh          # from workspace root
#   make generate-skills                             # via Makefile target
#
# Re-run after adding/removing Makefile targets to keep skills in sync.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
MAKEFILE="$WORKSPACE_ROOT/Makefile"
SKILLS_DIR="$WORKSPACE_ROOT/.claude/skills"

if [[ ! -f "$MAKEFILE" ]]; then
    echo "Error: Makefile not found at $MAKEFILE" >&2
    exit 1
fi

# Extract .PHONY targets, skip "help" (it's the default target, not useful as a skill)
PHONY_TARGETS=$(grep -m1 '^\.PHONY:' "$MAKEFILE" | sed 's/^\.PHONY://' | tr ' ' '\n' | grep -v '^$' | grep -v '^help$' | sort || true)

if [[ -z "$PHONY_TARGETS" ]]; then
    echo "Error: No .PHONY targets found in Makefile" >&2
    exit 1
fi

# Extract description for a target from the Makefile help text
# Looks for lines like: @echo "  target_name  - Description here"
get_description() {
    local target="$1"
    local desc
    # Match: "  target_name ... - Description" (extra text like "ISSUE=<number>" may appear before the dash)
    desc=$(grep -oP "\"  ${target}\b[^-]*- \K[^\"]*" "$MAKEFILE" 2>/dev/null || true)
    if [[ -z "$desc" ]]; then
        # Fallback: use the target name itself
        desc="Run 'make ${target}' in the workspace."
    fi
    echo "$desc"
}

created=0
unchanged=0
removed=0

# Generate skills for each target
for target in $PHONY_TARGETS; do
    skill_name="make_${target}"
    skill_dir="$SKILLS_DIR/$skill_name"
    skill_file="$skill_dir/SKILL.md"
    desc=$(get_description "$target")

    # Build the SKILL.md content
    if [[ "$target" == "revert-feature" ]]; then
        content=$(cat <<HEREDOC
---
name: ${skill_name}
description: "${desc}"
disable-model-invocation: true
allowed-tools: Bash
argument-hint: "<issue-number>"
---

Run \`make revert-feature ISSUE=\$ARGUMENTS\` from the workspace root (\`${WORKSPACE_ROOT}\`).
Report the result to the user.
HEREDOC
)
    else
        content=$(cat <<HEREDOC
---
name: ${skill_name}
description: "${desc}"
disable-model-invocation: true
allowed-tools: Bash
---

Run \`make ${target}\` from the workspace root (\`${WORKSPACE_ROOT}\`).
Report the result to the user.
HEREDOC
)
    fi

    # Check if skill already exists with identical content
    if [[ -f "$skill_file" ]] && [[ "$(cat "$skill_file")" == "$content" ]]; then
        unchanged=$((unchanged + 1))
        continue
    fi

    mkdir -p "$skill_dir"
    echo "$content" > "$skill_file"
    echo "  Created: /make_${target}"
    created=$((created + 1))
done

# Prune skills that no longer have matching Makefile targets
for skill_dir in "$SKILLS_DIR"/make_*/; do
    [[ -d "$skill_dir" ]] || continue
    skill_name=$(basename "$skill_dir")
    target="${skill_name#make_}"

    if ! echo "$PHONY_TARGETS" | grep -qx "$target"; then
        rm -rf "$skill_dir"
        echo "  Removed: /${skill_name} (target '${target}' no longer in Makefile)"
        removed=$((removed + 1))
    fi
done

echo ""
echo "Done: ${created} created, ${unchanged} unchanged, ${removed} removed"
