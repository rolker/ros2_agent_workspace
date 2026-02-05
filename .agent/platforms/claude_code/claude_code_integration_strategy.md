# Claude Code Integration Strategy for ROS2 Agent Workspace

## Executive Summary

Your ROS2 Agent Workspace is well-architected for multi-agent collaboration with Gemini CLI, Antigravity, and GitHub Copilot CLI. This strategy extends it to support Claude Code while preserving platform agnosticism through standardized interfaces and configuration patterns.

## Key Principles

1. **Platform-Agnostic Design**: All agents share common workflows and interfaces
2. **Configuration Over Code**: Agent-specific behavior through config files, not hardcoded logic
3. **Minimal Duplication**: Extend existing patterns rather than creating parallel systems
4. **Safe Defaults**: Claude Code should work out-of-the-box with conservative settings

---

## 1. Directory Structure Extension

### Recommended Structure

```
.agent/
├── platforms/              # NEW: Platform-specific configurations
│   ├── common/            # Shared by all platforms
│   │   ├── workflows/     # Universal workflow templates
│   │   └── schemas/       # Validation schemas
│   ├── copilot/          # GitHub Copilot specific
│   ├── gemini/           # Gemini CLI specific
│   ├── antigravity/      # Antigravity specific
│   └── claude_code/      # NEW: Claude Code specific
│       ├── config.yaml
│       ├── prompts/
│       └── mcp_servers/  # Model Context Protocol servers
├── workflows/            # Keep existing universal workflows
├── scripts/              # Keep existing helper scripts
└── AI_RULES.md          # Keep existing universal rules
```

### Why This Works

- Existing agents continue using `.agent/workflows/` for universal patterns
- Platform-specific customizations go in `.agent/platforms/<name>/`
- No breaking changes to current setup

---

## 2. Claude Code Configuration File

### `.agent/platforms/claude_code/config.yaml`

```yaml
# Claude Code Platform Configuration
platform:
  name: claude_code
  version: "1.0"
  
# Context and behavior
context:
  max_tokens: 100000  # Conservative for ROS2 workspace
  include_patterns:
    - "*.md"
    - "*.repos"
    - "*.yaml"
    - "*.sh"
    - "package.xml"
    - "CMakeLists.txt"
  exclude_patterns:
    - "build/**"
    - "install/**"
    - "log/**"
    - "*.bag"
    
# Model Context Protocol (MCP) integration
mcp:
  enabled: true
  servers:
    - name: "filesystem"
      type: "builtin"
    - name: "git"
      type: "builtin"
    - name: "ros2_workspace"
      type: "custom"
      config: ".agent/platforms/claude_code/mcp_servers/ros2_workspace.json"

# Workflow mapping
workflows:
  # Map universal workflows to Claude Code patterns
  setup_layer: ".agent/scripts/setup.sh"
  build_all: ".agent/scripts/build_all.sh"
  status_check: ".agent/scripts/status_report.sh"
  
# Safety and constraints
safety:
  auto_commit: false  # Never auto-commit
  auto_push: false    # Never auto-push
  confirm_destructive: true  # Always confirm rm, clean, etc.
  max_file_edits: 10  # Safety limit per session
  
# Integration with existing tools
git:
  identity_script: ".agent/scripts/set_git_identity_env.sh"
  identity_flag: "--agent claude_code"
```

---

## 3. Custom MCP Server for ROS2 Context

Claude Code uses Model Context Protocol (MCP) to access external tools. Create a custom MCP server for ROS2-specific operations:

### `.agent/platforms/claude_code/mcp_servers/ros2_workspace.json`

```json
{
  "name": "ros2_workspace",
  "description": "MCP server for ROS2 workspace operations",
  "version": "1.0",
  "commands": [
    {
      "name": "list_layers",
      "description": "List all workspace layers",
      "command": ".agent/scripts/list_layers.sh"
    },
    {
      "name": "layer_status",
      "description": "Get status of a specific layer",
      "command": ".agent/scripts/layer_status.sh",
      "args": ["layer_name"]
    },
    {
      "name": "check_dependencies",
      "description": "Check ROS dependencies for a package",
      "command": "rosdep check",
      "args": ["--from-paths", "src"]
    },
    {
      "name": "package_info",
      "description": "Get package.xml info",
      "command": ".agent/scripts/package_info.sh",
      "args": ["package_name"]
    }
  ],
  "resources": [
    {
      "name": "build_logs",
      "path": "layers/*/build/*/stdout_stderr.log"
    },
    {
      "name": "test_results",
      "path": "layers/*/build/test_results/**/*.xml"
    }
  ]
}
```

---

## 4. Helper Scripts for Claude Code

### `.agent/scripts/list_layers.sh`

```bash
#!/bin/bash
# List all configured layers

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
CONFIGS_DIR="$WORKSPACE_ROOT/configs"

echo "Available layers:"
for repos_file in "$CONFIGS_DIR"/*.repos; do
    if [ -f "$repos_file" ]; then
        layer_name=$(basename "$repos_file" .repos)
        echo "  - $layer_name"
    fi
done
```

### `.agent/scripts/layer_status.sh`

```bash
#!/bin/bash
# Get detailed status for a layer

LAYER_NAME="$1"
if [ -z "$LAYER_NAME" ]; then
    echo "Usage: layer_status.sh <layer_name>"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
LAYER_PATH="$WORKSPACE_ROOT/layers/main/${LAYER_NAME}_ws"

if [ ! -d "$LAYER_PATH" ]; then
    echo "Layer '$LAYER_NAME' not found at $LAYER_PATH"
    exit 1
fi

echo "Layer: $LAYER_NAME"
echo "Path: $LAYER_PATH"
echo ""

if [ -d "$LAYER_PATH/build" ]; then
    echo "Build status: Built"
    echo "Build date: $(stat -c %y "$LAYER_PATH/build" 2>/dev/null || stat -f %Sm "$LAYER_PATH/build" 2>/dev/null)"
else
    echo "Build status: Not built"
fi

if [ -d "$LAYER_PATH/src" ]; then
    echo "Packages in src/:"
    find "$LAYER_PATH/src" -name "package.xml" -type f | wc -l
fi
```

---

## 5. Platform-Agnostic Workflow Interface

### `.agent/workflows/interface.yaml`

Create a universal workflow interface that all platforms implement:

```yaml
# Universal Workflow Interface
# All platforms must support these core operations

workflows:
  setup:
    description: "Initialize a workspace layer"
    parameters:
      - name: "layer_name"
        type: "string"
        required: true
    platforms:
      copilot: "npm run setup"
      gemini: "gemini setup"
      antigravity: "ag setup"
      claude_code: ".agent/scripts/setup.sh"
  
  build:
    description: "Build one or all layers"
    parameters:
      - name: "layer_name"
        type: "string"
        required: false
    platforms:
      copilot: "npm run build"
      gemini: "gemini build"
      antigravity: "ag build"
      claude_code: ".agent/scripts/build.sh"
  
  status:
    description: "Get workspace status"
    platforms:
      copilot: "npm run status"
      gemini: "gemini status"
      antigravity: "ag status"
      claude_code: ".agent/scripts/status_report.sh"
  
  add_repo:
    description: "Add a repository to a layer"
    parameters:
      - name: "layer_name"
        type: "string"
        required: true
      - name: "repo_url"
        type: "string"
        required: true
    platforms:
      copilot: ".agent/workflows/dev/add-repo.md"
      gemini: ".agent/workflows/dev/add-repo.md"
      antigravity: ".agent/workflows/dev/add-repo.md"
      claude_code: ".agent/workflows/dev/add-repo.md"
```

---

## 6. Claude Code Onboarding Guide

### `.agent/platforms/claude_code/ONBOARDING.md`

```markdown
# Claude Code Quick Start

## Setup (One-time)

1. **Configure git identity:**
   ```bash
   source .agent/scripts/set_git_identity_env.sh --agent claude_code
   ```

2. **Source ROS environment:**
   ```bash
   source .agent/scripts/env.sh
   ```

3. **Verify configuration:**
   ```bash
   .agent/scripts/status_report.sh
   ```

## Available Commands

Claude Code can invoke these operations:

- `list_layers` - See all available workspace layers
- `layer_status <name>` - Check if a layer is built
- `setup <layer>` - Initialize a layer (e.g., setup core)
- `build [layer]` - Build specific or all layers
- `status` - Comprehensive workspace report

## Common Tasks

### Add a new repository
"Add repository https://github.com/org/repo to the core layer"

### Build workspace
"Build all layers" or "Build the core layer"

### Check status
"What's the current status of the workspace?"

## Safety Features

- No auto-commit or auto-push
- Confirmation required for destructive operations
- Limited to 10 file edits per session
- Build artifacts excluded from context
```

---

## 7. Migration Path for Existing Agents

To ensure other agents aren't disrupted:

### Phase 1: Additive Only (Week 1)
- Add `.agent/platforms/` directory
- Add Claude Code configuration
- Keep all existing paths working

### Phase 2: Gradual Adoption (Week 2-3)
- Update documentation to reference platform-agnostic patterns
- Add interface.yaml for workflow discovery
- Test with each platform

### Phase 3: Optimization (Week 4+)
- Consider consolidating duplicate configs
- Collect feedback from agent usage
- Refine MCP server capabilities

---

## 8. Testing Strategy

### Integration Tests

Create `.agent/platforms/claude_code/tests/integration_test.sh`:

```bash
#!/bin/bash
# Test Claude Code integration

set -e

echo "Testing Claude Code integration..."

# Test 1: List layers
echo "Test 1: Can list layers"
.agent/scripts/list_layers.sh

# Test 2: Setup a layer
echo "Test 2: Can setup core layer"
.agent/scripts/setup.sh core

# Test 3: Get layer status
echo "Test 3: Can get layer status"
.agent/scripts/layer_status.sh core

# Test 4: Build layer
echo "Test 4: Can build layer"
cd layers/main/core_ws && colcon build --symlink-install

echo "All tests passed!"
```

### Validation Checklist

- [ ] Claude Code can discover available layers
- [ ] Claude Code can setup a layer
- [ ] Claude Code can build layers
- [ ] Claude Code respects safety constraints
- [ ] Existing agents (Copilot, Gemini, Antigravity) still work
- [ ] Git identity configured correctly
- [ ] MCP server responds to queries

---

## 9. Documentation Updates

### Update README.md

Add a new section:

```markdown
### For Claude Code Users

**Quick Setup:**
```bash
# 1. Configure Claude Code
source .agent/platforms/claude_code/ONBOARDING.md

# 2. Source environment
source .agent/scripts/env.sh

# 3. Verify
.agent/scripts/status_report.sh
```

**Common Commands:**
- Ask: "List all workspace layers"
- Ask: "Setup the core layer"
- Ask: "Build all layers and show me any errors"
```

---

## 10. Next Steps

### Immediate (Day 1)
1. Create `.agent/platforms/claude_code/` directory structure
2. Add `config.yaml` with conservative settings
3. Create basic `ONBOARDING.md`
4. Test with a simple "list layers" command

### Short-term (Week 1)
1. Implement MCP server for ROS2 operations
2. Add helper scripts (list_layers.sh, layer_status.sh)
3. Create integration tests
4. Update main README

### Medium-term (Month 1)
1. Collect usage data from Claude Code sessions
2. Refine MCP server based on actual needs
3. Add platform-agnostic workflow interface
4. Document lessons learned

### Long-term (Quarter 1)
1. Consider standardizing all agents on MCP pattern
2. Build shared agent observability dashboard
3. Create cross-platform workflow library
4. Publish as reference architecture

---

## Appendix: Why This Approach Works

### Platform Agnosticism Through Abstraction

```
┌─────────────────────────────────────┐
│     Universal Interface Layer       │
│  (interface.yaml, AI_RULES.md)      │
└──────────────┬──────────────────────┘
               │
       ┌───────┴────────┬──────────┬──────────┐
       ▼                ▼          ▼          ▼
┌──────────┐    ┌──────────┐  ┌──────────┐  ┌──────────┐
│ Copilot  │    │  Gemini  │  │Antigravity│  │  Claude  │
│ Platform │    │ Platform │  │ Platform  │  │   Code   │
└──────────┘    └──────────┘  └──────────┘  └──────────┘
       │                │          │          │
       └────────────────┴──────────┴──────────┘
                        │
                        ▼
               ┌─────────────────┐
               │  ROS2 Workspace │
               └─────────────────┘
```

### Key Benefits

1. **No Lock-in**: Each platform has equal access to workspace
2. **Incremental Adoption**: Add Claude Code without breaking existing agents
3. **Single Source of Truth**: Workflows defined once, executed many ways
4. **Testable**: Each platform can be validated independently
5. **Maintainable**: Platform logic isolated from core workspace logic

---

## Questions or Issues?

If Claude Code encounters issues:

1. Check `.agent/platforms/claude_code/config.yaml` for constraints
2. Review safety settings (auto_commit, max_file_edits)
3. Validate MCP server is responding: `cat .agent/platforms/claude_code/mcp_servers/ros2_workspace.json`
4. Compare with working platform (e.g., Copilot) for patterns

---

## Summary

This integration strategy:
- ✅ Extends your workspace for Claude Code
- ✅ Maintains platform agnosticism
- ✅ Doesn't break existing agents
- ✅ Uses industry patterns (MCP, YAML configs)
- ✅ Provides clear migration path
- ✅ Includes testing and validation

Start with Phase 1 (additive changes only) and iterate based on real usage!
