---
trigger: always_on
---

# AI Agent Signature Protocol

**Rule**: All AI agents must append a structured signature to the body of any GitHub Issue, Pull Request, or Comment they create via the API.

## Reasoning
This ensures that content created by agents (which may technically be authored by the user's Personal Access Token) is clearly distinguishable from content written by the human user.

## Signature Format
The signature must be placed at the very end of the content body, separated by a horizontal rule.

```markdown

---
**🤖 Authored-By**: `<Agent Name>`
**🧠 Model**: `<Model Name>`
```

### Examples

**For Antigravity Agent:**
```markdown
This PR updates the dependency list.

---
**🤖 Authored-By**: `Antigravity Agent`
**🧠 Model**: `Gemini 2.0 Flash`
```

**For Copilot CLI:**
```markdown
I have fixed the bug in the navigation stack.

---
**🤖 Authored-By**: `Copilot CLI Agent`
**🧠 Model**: `GPT-4o`
```
