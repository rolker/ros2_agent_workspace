---
trigger: always_on
---

# Keep Workspace Root Clean

## Golden Rule
**The workspace root and `layers/*/src/` must remain clean.** All agent-generated files must be isolated in designated temporary locations.

## Build Discipline
- **No Builds in Root**: Do NOT run `colcon build` or similar build commands from the workspace root. Always build within specific layer directories (e.g., `layers/main/core_ws`).
- **No Builds in Source Dirs**: Do NOT run builds inside `layers/*/src/`. Use the corresponding workspace layer directory instead.

## Temporary Files Policy

### Designated Scratchpad: `.agent/scratchpad/`
Use this for all persistent temporary files created during agent sessions:
- Analysis artifacts (CSVs, JSON reports, markdown documents)
- Build logs and diagnostics
- Test results and coverage reports
- Session-specific metadata

**⚠️ Multi-Agent Safety**: When multiple agents work concurrently, you MUST use unique filenames to prevent collisions.

**Recommended: Use mktemp**:
```bash
# ✅ CORRECT - Collision-safe with mktemp
output_file=$(mktemp .agent/scratchpad/build_report.XXXXXX.json)
analysis_result=$(mktemp .agent/scratchpad/analysis.XXXXXX.md)

# Use the files
echo "data" > "$output_file"
echo "analysis" > "$analysis_result"

# Clean up when done
rm "$output_file" "$analysis_result"
```

**Alternative: Timestamp-Based Naming**:
```bash
# ✅ ACCEPTABLE - Collision-safe with timestamps
output_file=".agent/scratchpad/build_report_$(date +%s%N).json"
analysis_result=".agent/scratchpad/analysis_$(date +%s%N).md"

# ❌ INCORRECT - Static filenames cause collisions!
output_file=".agent/scratchpad/report.json"  # Multiple agents will overwrite each other
temp_file="layers/main/core_ws/temp_analysis"  # Never in source dirs!
```

### When to Use System `/tmp`
Use `/tmp` (Linux/Mac) or `%TEMP%` (Windows) only for:
- Ephemeral files cleaned up within the same command
- Files that should not persist across sessions
- Temporary downloads or caches

### Cleanup Responsibilities
- **Session-Level**: When your session ends (after calling final tools), clean up `.agent/scratchpad/` unless artifacts are explicitly needed for review
- **Cross-Session**: Files in `.agent/scratchpad/` persist; don't assume they're temporary across sessions
- **Before Commit**: Always verify no untracked files exist in `layers/*/src/` before committing

## Prevention & Enforcement

### Pre-Commit Warnings
If you attempt to stage files in `layers/*/src/` that are untracked (temp artifacts), the pre-commit hook will warn you. Do not bypass this—investigate and clean up instead.

### Git Status Check
Before finishing work:
```bash
git status
vcs status layers/*/src
```

If you see **modified** files in source directories that you didn't touch, stop and alert the user.

## Examples

**❌ Problematic Pattern (Old)**:
```bash
# Agent creates file in source tree
cd layers/main/core_ws/src/marine_autonomy
echo "debug info" > debug_output.txt
# Forgets to clean up; later agent commits it
```

**✅ Correct Pattern (New)**:
```bash
# Agent creates file in scratchpad with unique name using mktemp
debug_file=$(mktemp .agent/scratchpad/debug_output.XXXXXX.txt)
echo "debug info" > "$debug_file"
# File is ignored by git; never accidentally committed
# Can be manually reviewed later if needed
# Clean up when done
rm "$debug_file"
```
