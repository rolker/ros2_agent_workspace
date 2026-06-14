#!/bin/bash
# .agent/scripts/dispatch_subagent.sh
# Dispatch a workflow skill (or a raw kickoff prompt) into a fresh-context
# sub-agent — either in-process (Agent tool, via a stdout handoff block the
# host pastes) or in a container (headless docker via docker_run_agent.sh).
#
# Part of #481 phase C / #490. The sub-agent's exit contract is convention-only
# (no enforcement, per ADR-0004/0005): it writes a final progress.md entry; the
# host reads the last entry for the outcome + next action.
#
# Usage:
#   dispatch_subagent.sh --mode in-process|container --issue <N> \
#       (--skill <name> | --prompt-file <path>) \
#       [--entry-type <type>] [--output-format <fmt>]
#
# Examples:
#   dispatch_subagent.sh --mode in-process --issue 490 --skill review-code
#   dispatch_subagent.sh --mode container  --issue 490 --prompt-file /tmp/task.md --entry-type 'Local Review'

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"
# Resolve to the main workspace root if invoked from inside a worktree.
if [[ "$ROOT_DIR" == *"/.workspace-worktrees/"* ]]; then
    ROOT_DIR="${ROOT_DIR%%/.workspace-worktrees/*}"
elif [[ "$ROOT_DIR" == *"/layers/worktrees/"* ]]; then
    ROOT_DIR="${ROOT_DIR%%/layers/worktrees/*}"
fi

# ---------- Defaults ----------

MODE=""
ISSUE=""
SKILL=""
PROMPT_FILE=""
ENTRY_TYPE=""
OUTPUT_FORMAT="stream-json"

# Identity literals embedded in the handoff (AGENTS.md § Agent Commit Identity).
# Prefer the session's exported identity; fall back to the Claude default.
DISPATCH_AGENT_NAME="${AGENT_NAME:-Claude Code Agent}"
DISPATCH_AGENT_EMAIL="${AGENT_EMAIL:-roland+claude-code@ccom.unh.edu}"

# Skill -> expected progress.md entry type (ADR-0013 vocabulary). Used for the
# exit-contract "a newer entry of the expected type" check when --entry-type is
# not given explicitly.
skill_entry_type() {
    case "$1" in
        review-issue)    echo "Issue Review" ;;
        plan-task)       echo "Plan Authored" ;;
        review-plan)     echo "Plan Review" ;;
        review-code)     echo "Local Review" ;;
        triage-reviews)  echo "Integrated Review" ;;
        *)               echo "" ;;
    esac
}

usage() {
    sed -n '2,18p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

# ---------- Argument parsing ----------

while [[ $# -gt 0 ]]; do
    case "$1" in
        --mode)          MODE="${2:-}"; shift 2 ;;
        --issue)         ISSUE="${2:-}"; shift 2 ;;
        --skill)         SKILL="${2:-}"; shift 2 ;;
        --prompt-file)   PROMPT_FILE="${2:-}"; shift 2 ;;
        --entry-type)    ENTRY_TYPE="${2:-}"; shift 2 ;;
        --output-format) OUTPUT_FORMAT="${2:-}"; shift 2 ;;
        -h|--help)       usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
    esac
done

# ---------- Validation ----------

case "$MODE" in
    in-process|container) : ;;
    *) echo "ERROR: --mode must be 'in-process' or 'container'." >&2; usage >&2; exit 1 ;;
esac
if [ -z "$ISSUE" ]; then
    echo "ERROR: --issue <N> is required." >&2; exit 1
fi
if [ -n "$SKILL" ] && [ -n "$PROMPT_FILE" ]; then
    echo "ERROR: pass exactly one of --skill or --prompt-file, not both." >&2; exit 1
fi
if [ -z "$SKILL" ] && [ -z "$PROMPT_FILE" ]; then
    echo "ERROR: pass one of --skill <name> or --prompt-file <path>." >&2; exit 1
fi
if [ -n "$PROMPT_FILE" ] && [ ! -f "$PROMPT_FILE" ]; then
    echo "ERROR: --prompt-file not found: $PROMPT_FILE" >&2; exit 1
fi

# Resolve the expected entry type: explicit --entry-type wins, else derive from
# the skill. May be empty for a raw prompt with no declared type.
if [ -z "$ENTRY_TYPE" ] && [ -n "$SKILL" ]; then
    ENTRY_TYPE="$(skill_entry_type "$SKILL")"
fi

# ---------- Resolve the worktree + its progress.md ----------

WORKTREE_PATH=""
for candidate in \
    "$ROOT_DIR/.workspace-worktrees/issue-workspace-$ISSUE" \
    "$ROOT_DIR/layers/worktrees/issue-"*"-$ISSUE"; do
    if [ -d "$candidate" ]; then WORKTREE_PATH="$candidate"; break; fi
done
if [ -z "$WORKTREE_PATH" ]; then
    echo "ERROR: no worktree found for issue #$ISSUE (create it first)." >&2; exit 1
fi
PROGRESS_FILE="$WORKTREE_PATH/.agent/work-plans/issue-$ISSUE/progress.md"

# ---------- Build the handoff prompt ----------

if [ -n "$PROMPT_FILE" ]; then
    TASK_BODY="$(cat "$PROMPT_FILE")"
else
    TASK_BODY="Run the \`/$SKILL\` workflow skill for issue #$ISSUE: read \`.claude/skills/$SKILL/SKILL.md\` and follow its procedure to completion."
fi

if [ -n "$ENTRY_TYPE" ]; then
    ENTRY_CLAUSE="append your final \`## $ENTRY_TYPE\` entry to"
else
    ENTRY_CLAUSE="append your final typed entry (per ADR-0013) to"
fi

HANDOFF="$(cat <<EOF
$TASK_BODY

---
## Sub-agent handoff contract (#490)

You are a fresh-context sub-agent dispatched for **issue #$ISSUE**. Work only
within this issue's worktree; do not touch other issues.

**Commit identity** — every \`git commit\` MUST carry per-invocation identity
literals (AGENTS.md § Agent Commit Identity):

    git -c user.name="$DISPATCH_AGENT_NAME" -c user.email="$DISPATCH_AGENT_EMAIL" commit -m "..."

Never use \`--no-verify\`. Do **not** \`git push\` — the host performs pushes
with its own credentials.

**Exit contract** — before you finish, $ENTRY_CLAUSE:

    .agent/work-plans/issue-$ISSUE/progress.md

The host reads the **last** entry to learn the outcome and the next step. If you
cannot finish, still write an entry with \`**Status**: partial\` (or \`failed\`)
recording what was done and what remains — a missing entry reads as "died
before reporting".
EOF
)"

# ---------- in-process mode: emit the handoff for a fresh Agent call ----------

if [ "$MODE" = "in-process" ]; then
    echo "=== DISPATCH (in-process) — issue #$ISSUE${SKILL:+, skill /$SKILL} ==="
    echo "Paste the block between the markers into a fresh Agent tool call (no"
    echo "inherited context). Expected entry type: ${ENTRY_TYPE:-<any>}."
    echo "---------------8<--------------- BEGIN HANDOFF ---------------8<---------------"
    printf '%s\n' "$HANDOFF"
    echo "---------------8<---------------- END HANDOFF ----------------8<---------------"
    exit 0
fi

# ---------- container mode: launch headless docker, then read the contract ----------

# Count progress.md entries (optionally of a given type) — used to require a
# *newer* entry after dispatch. Missing file => 0.
entry_count() {
    [ -f "$PROGRESS_FILE" ] || { echo 0; return 0; }
    local out
    if [ -n "$ENTRY_TYPE" ]; then
        out="$(python3 "$SCRIPT_DIR/progress_read.py" --type "$ENTRY_TYPE" "$PROGRESS_FILE" 2>/dev/null || true)"
    else
        out="$(python3 "$SCRIPT_DIR/progress_read.py" "$PROGRESS_FILE" 2>/dev/null || true)"
    fi
    printf '%s' "$out" | python3 -c 'import sys,json
try: print(len(json.load(sys.stdin).get("entries", [])))
except Exception: print(0)' 2>/dev/null || echo 0
}

PRE_COUNT="$(entry_count)"

PROMPT_TMP="$(mktemp /tmp/dispatch_handoff.XXXXXX.md)"
trap 'rm -f "$PROMPT_TMP"' EXIT
printf '%s\n' "$HANDOFF" > "$PROMPT_TMP"

echo "Dispatching issue #$ISSUE into a container (headless)…" >&2
RC=0
"$SCRIPT_DIR/docker_run_agent.sh" \
    --issue "$ISSUE" \
    --output-format "$OUTPUT_FORMAT" \
    --prompt-file "$PROMPT_TMP" || RC=$?

# --- Exit-contract read ---
echo ""
echo "===== Dispatch outcome (issue #$ISSUE) ====="
if [ "$RC" -ne 0 ]; then
    echo "  Result: FAILED — container exited $RC (claude error / killed)."
    echo "  Any partial work is in the worktree: $WORKTREE_PATH"
    exit "$RC"
fi

POST_COUNT="$(entry_count)"
if [ "$POST_COUNT" -le "$PRE_COUNT" ]; then
    echo "  Result: FAILED — no new \`## ${ENTRY_TYPE:-<typed>}\` progress.md entry"
    echo "          (count ${PRE_COUNT} -> ${POST_COUNT}); sub-agent likely died before reporting."
    echo "  Inspect the worktree: $WORKTREE_PATH"
    exit 1
fi

echo "  Result: OK — sub-agent wrote a new entry. Last entry:"
python3 "$SCRIPT_DIR/progress_read.py" "$PROGRESS_FILE" 2>/dev/null | python3 -c 'import sys,json
d=json.load(sys.stdin); e=d.get("entries", [])
if e:
    x=e[-1]
    print(f"    type:   {x.get(\"type\")}")
    print(f"    status: {x.get(\"status\")}")
    print(f"    by:     {x.get(\"by\")}")
    print(f"    when:   {x.get(\"when\")}")
' || echo "    (could not parse progress.md)"
echo "  Next: the host reviews the entry and runs the next phase / does the push."
