#!/usr/bin/env python3
"""PreToolUse hook: nudge AskUserQuestion toward a repo-qualified header.

Warn-only and always-on (operator decision, #592 plan checkpoint 2026-07-31).
When an `AskUserQuestion` tool call fires and *no* question opens with a
`<repo>#<N>`-style token, this hook emits a non-blocking nudge reminding the
model to add the re-orientation header defined in run-issue § Checkpoints
(`<repo>#<N> (PR #M): <title> — phase X of Y (<phase>); <state>`). Concurrent
sessions mean the operator answers a checkpoint while thinking about a
different repo; a bare `#24` is unadjudicable from the dialog alone.

Design contract:
- **Warn-only** — this hook NEVER blocks or denies. It always exits 0 and never
  emits a `deny`/`block` decision. A false positive costs a nudge, nothing else,
  so it can run always-on without the `$WORKTREE_ISSUE` gate the first plan drew
  (that gate fired in *any* worktree session, over-firing on non-orchestration
  work — the same can't-distinguish failure mode that sinks the Bash hook below).
- **Fail SAFE** — any parse error, unexpected schema, missing key, or internal
  exception exits 0 silently. A wrong assumption about the stdin JSON degrades to
  "no nudge," never a broken checkpoint. The whole point of a warn-only hook is
  that it can never be the reason a tool call fails.
- **Nudge channel** — the nudge rides `hookSpecificOutput.additionalContext`
  (injected into the model's context so subsequent AskUserQuestion calls carry
  the header) plus a `systemMessage` the operator sees. Neither field can block
  the call; emitting them with exit 0 is always non-blocking per the Claude Code
  hooks contract.

Why no Bash-`description` hook (recorded here per #592):
  The same context failure recurs in Bash `description` fields during
  orchestration — a permission prompt shows only the description. A PreToolUse
  hook on `Bash` cannot reliably distinguish orchestration commands (which should
  carry `<repo>#<N> <phase>:`) from the flood of ordinary Bash calls (builds,
  greps, git) that legitimately should not. Enforcing the format on all Bash
  would be pervasive false positives; scoping to "orchestration only" has no
  reliable signal in the tool-call JSON. So the Bash surface is covered by
  run-issue § Checkpoints prose, not a hook. The issue comment recording this
  conclusion is posted by the host at close time (this dispatch has no `gh` auth).
"""

import json
import re
import sys

# A repo-qualified token: <repo>#<N>, e.g. "project11_navigation#466". The `#`
# must be immediately preceded by non-whitespace, so a bare "PR #24" (space
# before `#`) does NOT satisfy it — that is the ambiguous form we nudge against.
REPO_ISSUE_TOKEN = re.compile(r"\S+#\d+")

# How far into the question text the header token must appear. The re-orientation
# header opens the question, so only the leading slice is inspected — a `#N`
# buried deep in prose is not a header.
HEADER_WINDOW = 120

NUDGE = (
    "AskUserQuestion checkpoint is missing its re-orientation header. Open the "
    "`question` text with `<repo>#<N> (PR #M): <issue title> — phase X of Y "
    "(<phase name>); <one-line state>` so the operator — who juggles concurrent "
    "sessions across repos — can adjudicate it from the dialog alone. A bare "
    "issue/PR number is ambiguous across project repos. See run-issue "
    "§ Checkpoints for the canonical format."
)


def leading_slice(question):
    """Return the header region of a question: up to the first newline or window."""
    first_line = question.split("\n", 1)[0]
    return first_line[:HEADER_WINDOW]


def any_question_conforms(questions):
    """True if at least one question opens with a <repo>#<N> token."""
    for q in questions:
        if not isinstance(q, dict):
            continue
        text = q.get("question")
        if not isinstance(text, str):
            continue
        if REPO_ISSUE_TOKEN.search(leading_slice(text)):
            return True
    return False


def emit_nudge():
    """Print a non-blocking warn payload. Never carries a deny/block decision."""
    payload = {
        "systemMessage": NUDGE,
        "hookSpecificOutput": {
            "hookEventName": "PreToolUse",
            "additionalContext": NUDGE,
        },
    }
    print(json.dumps(payload))


def main():
    # Fail SAFE: read + parse under a blanket guard; any surprise → silent allow.
    try:
        raw = sys.stdin.read()
        data = json.loads(raw)
        questions = data.get("tool_input", {}).get("questions")
        # No questions list, empty, or wrong shape → nothing to check, allow.
        if not isinstance(questions, list) or not questions:
            return 0
        if any_question_conforms(questions):
            return 0
        emit_nudge()
    except Exception:
        # Warn-only hook: never break the tool call. Swallow everything.
        return 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
