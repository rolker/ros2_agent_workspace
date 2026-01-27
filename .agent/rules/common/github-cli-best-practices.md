# Protocol: GitHub CLI Best Practices

**Rule**: Agents MUST use the "File-First" approach when passing multiline content to the GitHub CLI (`gh`).

## Context
Passing complex, multiline strings directly via command-line arguments (e.g., `gh issue create --body "line 1\nline 2"`) is unreliable. Shell environments and execution layers often escape newline characters (`\n` -> `\\n`), causing the content to render as a single block of literal text on GitHub.

## The Protocol

### 1. ❌ Forbidden Pattern
Do NOT pass multiline strings directly to the `--body` flag.
```bash
# BAD: This will often fail to render newlines correctly
gh issue create --title "My Issue" --body "Line 1\nLine 2"
```

### 2. ✅ Required Pattern (File-First)
ALWAYS write the content to a temporary file first, then use `--body-file`.

**Steps**:
1.  Create a temporary markdown file in `.agent/scratchpad/`.
2.  Write your full, formatted content to that file.
3.  Pass that file path to the `gh` command.

```bash
# GOOD
# 1. Prepare content
cat <<EOF > .agent/scratchpad/issue_body.md
# My Issue Title

Here is the first paragraph.

- List item 1
- List item 2

---
**🤖 Authored-By**: \`Agent Name\`
EOF

# 2. Execute command
gh issue create --title "My Issue" --body-file .agent/scratchpad/issue_body.md
```

## Applicability
This rule applies to:
- `gh issue create`
- `gh pr create`
- `gh issue comment`
- `gh pr comment`
- Any other command accepting a `--body` or `--notes` argument.
