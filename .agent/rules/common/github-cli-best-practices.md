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
1.  Ensure the scratchpad directory exists: `mkdir -p .agent/scratchpad/`
2.  Create a **unique filename** to prevent collisions with other concurrent agents
3.  Write your full, formatted content to that unique file.
4.  Pass that file path to the `gh` command.
5.  Clean up the file after use.

```bash
# GOOD - Using helper functions (recommended)
source .agent/scripts/lib/scratchpad_helpers.sh
BODY_FILE=$(scratchpad_file "issue_body" ".md")

cat <<EOF > "$BODY_FILE"
# My Issue Title

Here is the first paragraph.

- List item 1
- List item 2

---
**🤖 Authored-By**: \`Agent Name\`
**🧠 Model**: \`Model Name\`
EOF

gh issue create --title "My Issue" --body-file "$BODY_FILE"
rm "$BODY_FILE"  # Clean up after use
```

**Alternative - Manual timestamp-based naming**:
```bash
# GOOD - Manual unique naming
mkdir -p .agent/scratchpad/
BODY_FILE=".agent/scratchpad/issue_body_$(date +%s%N).md"

cat <<EOF > "$BODY_FILE"
# My Issue Title

Here is the first paragraph.

- List item 1
- List item 2

---
**🤖 Authored-By**: \`Agent Name\`
**🧠 Model**: \`Model Name\`
EOF

gh issue create --title "My Issue" --body-file "$BODY_FILE"
rm "$BODY_FILE"  # Clean up after use
```

⚠️ **Critical**: Never use static filenames like `issue_body.md` - multiple agents will overwrite each other's files!

## Applicability
This rule applies to:
- `gh issue create`
- `gh pr create`
- `gh issue comment`
- `gh pr comment`
- Any other command accepting a `--body` or `--notes` argument.
