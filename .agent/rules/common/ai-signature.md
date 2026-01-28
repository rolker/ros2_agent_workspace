---
trigger: always_on
---

# AI Agent Signature Protocol

**Rule**: All AI agents must append a structured signature to the body of any GitHub Issue, Pull Request, or Comment they create via the API.

## Reasoning
This ensures that content created by agents (which may technically be authored by the user's Personal Access Token) is clearly distinguishable from content written by the human user.

## Critical: Do NOT Copy Example Model Names

**⚠️ WARNING**: The examples below show model names like "GPT-4o" or "Gemini 2.0 Flash". These are **EXAMPLES ONLY**.

**DO NOT copy these model names directly.** Instead:

1. **Check the source of truth**: Read your identity from `.agent/.identity` file
2. **Auto-detect your identity**: Run `.agent/scripts/detect_agent_identity.sh`
3. **Use the correct values**:
   - `AGENT_NAME` for the **🤖 Authored-By** field
   - `AGENT_MODEL` for the **🧠 Model** field
4. **If identity is unknown**: Use generic fallback values:
   - Agent Name: "AI Agent"
   - Model: "Unknown Model"
   - Then ask the user to configure your identity

**Copying example values leads to mislabeled signatures** where different agents incorrectly claim to be the same model.

## How to Get Your Identity

### Option 1: Use Environment Variables (Recommended)

The most reliable way is to use environment variables set during initialization:

```bash
# After running set_git_identity_env.sh, these are available:
echo "Agent: $AGENT_NAME"
echo "Model: $AGENT_MODEL"
```

These variables are always current for your active shell session and won't become stale if configuration changes.

### Option 2: Read from .agent/.identity file (if it exists)

```bash
# Source the identity file if it exists
if [ -f .agent/.identity ]; then
    source .agent/.identity
    echo "Agent: $AGENT_NAME"
    echo "Model: $AGENT_MODEL"
else
    echo "Identity file not found. Run: .agent/scripts/detect_agent_identity.sh --write"
fi
```

**Note**: The `.agent/.identity` file is runtime-generated and may become stale if model/config changes during a session. Prefer environment variables when possible.

### Option 3: Auto-detect at runtime

```bash
# Run detection script
.agent/scripts/detect_agent_identity.sh --export

# Variables are now available:
# $AGENT_NAME, $AGENT_EMAIL, $AGENT_MODEL, $AGENT_FRAMEWORK
```

## Signature Format

The signature must be placed at the very end of the content body, separated by a horizontal rule.

```markdown

---
**🤖 Authored-By**: `<Your Actual Agent Name>`
**🧠 Model**: `<Your Actual Model Name>`
```

### Examples (DO NOT COPY - Determine Your Own Values)

**For Antigravity Agent** (example only):
```markdown
This PR updates the dependency list.

---
**🤖 Authored-By**: `Antigravity Agent`
**🧠 Model**: `Gemini 2.5 Pro`
```

**For Copilot CLI** (example only):
```markdown
I have fixed the bug in the navigation stack.

---
**🤖 Authored-By**: `Copilot CLI Agent`
**🧠 Model**: `GPT-4o`
```

**For Gemini CLI** (example only):
```markdown
Created new feature as requested.

---
**🤖 Authored-By**: `Gemini CLI Agent`
**🧠 Model**: `Gemini 2.0 Flash`
```

**If identity is unknown**:
```markdown
Completed the task as specified.

---
**🤖 Authored-By**: `AI Agent`
**🧠 Model**: `Unknown Model`
```

## Reminder

- ✅ DO read your identity from `.agent/.identity` or auto-detect
- ✅ DO use your actual runtime model name
- ✅ DO use fallback values if identity cannot be determined
- ❌ DO NOT copy example model names from this document
- ❌ DO NOT guess or assume your model name
- ❌ DO NOT hardcode model names based on examples
