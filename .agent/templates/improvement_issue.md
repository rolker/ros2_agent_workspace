# Improvement Issue Template

Use this template when reporting infrastructure friction through the Continuous Improvement workflow.

---

## Problem

[Describe what friction was encountered. Be specific about what didn't work, was confusing, or required a workaround.]

## Example Friction

```bash
# What the agent tried:
[Show the actual command, code, or process that failed or was confusing]

# What should have worked (or what eventually worked):
[Show the correct approach, or what you had to figure out]
```

## Proposed Solution

[Provide a specific, actionable fix. Examples:
- Update section X in file Y to clarify Z
- Add new script .agent/scripts/do_thing.sh that automates X
- Update framework instruction files (CLAUDE.md, copilot-instructions.md, etc.)]

## Impact

Who benefits from this fix? Quantify if possible:

- ✅ **Benefit 1**: [e.g., "All agents save ~10 min per session"]
- ✅ **Benefit 2**: [e.g., "Prevents confusion for ROS Developers"]
- ✅ **Benefit 3**: [e.g., "Reduces support questions"]

## Priority

[Choose one: 🔴 High / 🟡 Medium / 🟢 Low]

**Justification**: [Why this priority? Consider frequency, impact, and blast radius]

## Session Evidence

**Session**: [session-id or date]

Specific examples from session:
- [What happened in the session that revealed this friction?]
- [What was the workaround or time spent?]
- [Link to commit, file, or checkpoint if available]

## Additional Context

[Optional: Any other relevant information, related issues, or constraints]

---

**🤖 Reported-By**: [Agent Name] ([Model - e.g., Claude 3.5 Sonnet, GPT-4o])

---

## Issue Metadata (for gh issue create)

**Labels**: `enhancement`, [add others: `documentation`, `bug`, `agent-infrastructure`, `good-first-issue`]

**Relates to**: [Link any related issues with #123 format]
