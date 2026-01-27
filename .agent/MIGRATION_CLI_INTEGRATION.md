# Migration Guide - AI CLI Integration (Issue #46)

This guide helps existing agents and users adapt to the new consolidated documentation structure.

## What Changed?

### Documentation Structure

**Before** (Duplicated):
- `.github/copilot-instructions.md` - Full instructions
- `.agent/AGENT_ONBOARDING.md` - Full instructions
- Rules scattered across multiple files

**After** (Consolidated):
- `.agent/AI_RULES.md` - **Universal rules** (single source of truth)
- `.agent/AI_CLI_QUICKSTART.md` - **Fast path for CLI agents**
- `.agent/CLI_COMMANDS.md` - **Command mapping**
- `.github/copilot-instructions.md` - **Reference layer** (links to AI_RULES.md)
- `.agent/instructions/<framework>.instructions.md` - **Framework overlays**
- `.agent/AGENT_ONBOARDING.md` - **Specialized agents only**

### Identity Configuration

**Before**:
```bash
source .agent/scripts/set_git_identity_env.sh "Copilot CLI Agent" "roland+copilot-cli@ccom.unh.edu"
```

**After** (backward compatible + new options):
```bash
# Option 1: Same as before (still works)
source .agent/scripts/set_git_identity_env.sh "Copilot CLI Agent" "roland+copilot-cli@ccom.unh.edu"

# Option 2: Use framework preset (NEW)
source .agent/scripts/set_git_identity_env.sh --agent copilot

# Option 3: Auto-detect (NEW)
source .agent/scripts/set_git_identity_env.sh --detect
```

### Workflows

**Before**:
- `/check-status` used basic git/vcs commands only

**After**:
- `/check-status` detects CLI framework and uses GitHub API when available
- New `/setup-environment` workflow for one-command setup
- Workflows check for ROS environment and source if needed

## Migration Path for Agents

### For Copilot CLI

1. **Update your entry point**:
   - **Old**: Read `.github/copilot-instructions.md` directly
   - **New**: Start with `.agent/AI_CLI_QUICKSTART.md`, reference `.agent/AI_RULES.md` for details

2. **Use new identity shortcuts**:
   ```bash
   source .agent/scripts/set_git_identity_env.sh --agent copilot
   ```

3. **No other changes required** - All existing workflows still work

### For Gemini CLI

1. **New framework support**:
   - Use `.agent/AI_CLI_QUICKSTART.md` for onboarding
   - Framework-specific guide: `.agent/instructions/gemini-cli.instructions.md`

2. **Identity configuration**:
   ```bash
   source .agent/scripts/set_git_identity_env.sh --agent gemini
   ```

### For Antigravity and Custom Agents

1. **Entry point unchanged**:
   - Continue using `.agent/AGENT_ONBOARDING.md`
   - AGENT_ONBOARDING now focuses on specialized/container agents

2. **Reference AI_RULES.md for universal rules**:
   - No need to read duplicated instructions
   - Universal workflow documented in one place

## Migration Path for Documentation Contributors

### When Adding New Framework Support

**Before**: Copy entire instruction set, create duplication

**After**: 
1. Add framework to lookup tables in:
   - `.agent/scripts/configure_git_identity.sh`
   - `.agent/scripts/set_git_identity_env.sh`
2. Create minimal overlay file in `.agent/instructions/<framework>.instructions.md`:
   - Link to `AI_RULES.md` for universal rules
   - Add only framework-specific features
3. Test framework detection in `.agent/scripts/detect_cli_env.sh`

### When Updating Universal Rules

**Before**: Update multiple files (.github/copilot-instructions.md, AGENT_ONBOARDING.md, etc.)

**After**:
1. Update `.agent/AI_RULES.md` (single source of truth)
2. Framework-specific files automatically benefit
3. No duplication to maintain

## Breaking Changes

**None** - All changes are backward compatible:
- ✅ Old identity configuration syntax still works
- ✅ Existing workflows unchanged
- ✅ `.github/copilot-instructions.md` still exists (now a reference layer)
- ✅ `.agent/AGENT_ONBOARDING.md` still exists (refocused on specialized agents)

## Deprecations

None at this time. Old patterns continue to work but new patterns are recommended.

## Testing Your Migration

### For CLI Agents

1. **Clone fresh repository**
2. **Follow AI_CLI_QUICKSTART.md**:
   ```bash
   source .agent/scripts/env.sh
   source .agent/scripts/set_git_identity_env.sh --agent copilot
   .agent/scripts/status_report.sh
   ```
3. **Verify identity**:
   ```bash
   git config user.name   # Should show: Copilot CLI Agent
   git config user.email  # Should show: roland+copilot-cli@ccom.unh.edu
   ```
4. **Test framework detection**:
   ```bash
   .agent/scripts/tests/test_detect_cli_env.sh
   ```

### For Specialized Agents

1. **Clone fresh repository**
2. **Follow AGENT_ONBOARDING.md** as before
3. **Reference AI_RULES.md** when needed
4. **No other changes required**

## Benefits of New Structure

### For CLI Users
- ⚡ Faster onboarding (5 min vs 15+ min)
- 🔍 Better workflow discoverability (CLI_COMMANDS.md)
- 🤖 Auto-detection of framework
- 📦 One-command setup (/setup-environment)

### For Documentation Maintainers
- 🎯 Single source of truth (no duplication)
- 🔌 Easy to add new frameworks (overlay pattern)
- ✅ Easier to keep consistent (update one file)
- 📊 Clear ownership and structure

### For All Agents
- 📚 Clear navigation (README → Quickstart → Rules → Commands)
- 🔄 Universal workflow applies to everyone
- 🛠️ Framework-specific optimizations when available
- 🚀 Future-proof (new frameworks just add overlay)

## Questions?

- **"Where do I start now?"** - See `.agent/AI_CLI_QUICKSTART.md` (CLI) or `.agent/AGENT_ONBOARDING.md` (specialized)
- **"What are the universal rules?"** - See `.agent/AI_RULES.md`
- **"How do I use workflows?"** - See `.agent/CLI_COMMANDS.md`
- **"My framework isn't listed?"** - Use manual identity configuration, or ask for framework support to be added

## Related Documentation

- [AI_CLI_QUICKSTART.md](AI_CLI_QUICKSTART.md) - Fast path for CLI agents
- [AI_RULES.md](AI_RULES.md) - Universal agent rules
- [CLI_COMMANDS.md](CLI_COMMANDS.md) - Workflow discovery
- [AGENT_ONBOARDING.md](AGENT_ONBOARDING.md) - Specialized agents
- [CONTRIBUTING.md](../CONTRIBUTING.md) - Documentation maintenance

---

**Last Updated**: 2026-01-27  
**Related Issue**: #46  
**Migration Support**: Ask questions in issue comments or PR discussions
