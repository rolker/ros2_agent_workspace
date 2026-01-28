---
trigger: always_on
---

# Keep Workspace Root Clean

## Golden Rule
**The workspace root and `workspaces/*/src/` must remain clean.** All agent-generated files must be isolated in designated temporary locations.

## Build Discipline
- **No Builds in Root**: Do NOT run `colcon build` or similar build commands from the workspace root. Always build within specific layer directories (e.g., `workspaces/core_ws`).
- **No Builds in Source Dirs**: Do NOT run builds inside `workspaces/*/src/`. Use the corresponding workspace layer directory instead.

## Temporary Files Policy

### Designated Scratchpad: `.agent/scratchpad/`
Use this for all persistent temporary files created during agent sessions:
- Analysis artifacts (CSVs, JSON reports, markdown documents)
- Build logs and diagnostics
- Test results and coverage reports
- Session-specific metadata

**Example**:
```bash
# ✅ CORRECT
output_file=".agent/scratchpad/build_report_$(date +%s).json"
analysis_result=".agent/scratchpad/analysis.md"

# ❌ INCORRECT
temp_file="workspaces/core_ws/temp_analysis" # Never in source dirs!
```

### When to Use System `/tmp`
Use `/tmp` (Linux/Mac) or `%TEMP%` (Windows) only for:
- Ephemeral files cleaned up within the same command
- Files that should not persist across sessions
- Temporary downloads or caches

### Cleanup Responsibilities
- **Session-Level**: When your session ends (after calling final tools), clean up `.agent/scratchpad/` unless artifacts are explicitly needed for review
- **Cross-Session**: Files in `.agent/scratchpad/` persist; don't assume they're temporary across sessions
- **Before Commit**: Always verify no untracked files exist in `workspaces/*/src/` before committing

## Prevention & Enforcement

### Pre-Commit Warnings
If you attempt to stage files in `workspaces/*/src/` that are untracked (temp artifacts), the pre-commit hook will warn you. Do not bypass this—investigate and clean up instead.

### Git Status Check
Before finishing work:
```bash
git status
vcs status workspaces/*/src
```

If you see **modified** files in source directories that you didn't touch, stop and alert the user.

## Examples

**❌ Problematic Pattern (Old)**:
```bash
# Agent creates file in source tree
cd workspaces/core_ws/src/marine_autonomy
echo "debug info" > debug_output.txt
# Forgets to clean up; later agent commits it
```

**✅ Correct Pattern (New)**:
```bash
# Agent creates file in scratchpad
echo "debug info" > .agent/scratchpad/debug_output_$(date +%s).txt
# File is ignored by git; never accidentally committed
# Can be manually reviewed later if needed
```
