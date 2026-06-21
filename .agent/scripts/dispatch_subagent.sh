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
#       [--entry-type <type>] [--output-format <fmt>] [--model <id>]
#       [--repo-slug <slug>]   # disambiguate a layer worktree on issue-# collision (#526)
#       [--context-file <path>] # host-fetched issue/PR body spliced into the
#                               # handoff so a no-GitHub-auth container phase can
#                               # read it instead of `gh issue view` (#552);
#                               # composable with --skill (unlike --prompt-file)
#   dispatch_subagent.sh --check    # container-auth preflight (#532), then exit
#
#   --model defaults per-skill (Opus for review/implement, Sonnet otherwise),
#   or per --entry-type for a raw --prompt-file dispatch (#539); pass an alias
#   ('opus'/'sonnet') to override. Container forwards it; in-process recommends
#   it. --check verifies the container auth tokens and exits.
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
MODEL=""   # --model override; empty => derive from the skill (see skill_model)
CHECK=""   # --check: run the container-auth preflight and exit (#532)
REPO_SLUG="" # --repo-slug: disambiguate a layer worktree (issue-<slug>-<N>) (#526)
CONTEXT_FILE="" # --context-file: host-fetched issue/PR body to splice into the
                # handoff (#552); composable with --skill

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
        address-findings) echo "Implementation" ;;
        *)               echo "" ;;
    esac
}

# Skill -> model (per-phase, #490). Reasoning-heavy phases get Opus; lighter
# ones (and the default) get Sonnet for quota headroom. Aliases, not pinned
# ids, so they survive model version bumps; an unavailable model hard-fails
# loudly (claude exits non-zero -> exit-contract reports FAILED). `--model`
# overrides any cell. Adjust the mapping here as needs change.
skill_model() {
    case "$1" in
        review-code|review-plan|triage-reviews)  echo "opus" ;;
        implement|address-findings)              echo "opus" ;;
        review-issue|plan-task)                  echo "sonnet" ;;
        *)                                       echo "sonnet" ;;
    esac
}

# Entry-type -> model, for raw-prompt (--prompt-file) dispatches that carry no
# --skill to map. Without this a custom-prompt dispatch silently defaulted to
# Sonnet even for an Implementation / review pass (#539).
#
# The tier is NOT duplicated here: this maps the entry type to the skill that
# writes it and defers to skill_model() — the single source of truth — so adding
# an Opus tier in skill_model can't silently downgrade an entry type. Only the
# entry-type -> skill map below is local; test_dispatch_model.sh asserts it.
entry_type_model() {
    local skill
    case "$1" in
        "Local Review"|"Local Review (Pre-Push)") skill="review-code" ;;
        "Integrated Review")                       skill="triage-reviews" ;;
        Implementation)                            skill="address-findings" ;;
        "Plan Review")                             skill="review-plan" ;;
        "Issue Review")                            skill="review-issue" ;;
        "Plan Authored")                           skill="plan-task" ;;
        *)                                         skill="" ;;  # unknown -> skill_model default (sonnet)
    esac
    skill_model "$skill"
}

# Model alias -> version-less display name for the sub-agent's progress.md
# `By:` line (#540). The alias resolves to the current release inside the
# container, so a sub-agent must NOT invent a version (e.g. "Opus 4.6"); the
# dispatcher hands it this exact string to stamp.
model_display() {
    case "$1" in
        opus)   echo "Claude Opus" ;;
        sonnet) echo "Claude Sonnet" ;;
        haiku)  echo "Claude Haiku" ;;
        *)      echo "$1" ;;
    esac
}

# Resolve the worktree's real progress.md, with a worktree-root fallback.
# Workspace worktrees keep it at depth 4 ($wt/.agent/work-plans/issue-N/
# progress.md), but LAYER worktrees nest the project repo at <layer>_ws/src/
# <repo>/, putting progress.md at depth 7. find -maxdepth 7 reaches the nested
# case (maxdepth 6 falls short — confirmed off-by-one); -quit stops at the first
# hit. When no file exists yet (the first-phase pre-creation case), fall back to
# the root path so a PRE count reads "absent -> 0". Callers MUST re-resolve per
# invocation: on the first phase the file is absent pre-dispatch, so only a
# fresh post-dispatch resolve discovers the freshly-written nested file.
# Assumes a single progress.md per issue per worktree (one issue == one project
# repo); -print -quit returns the first hit, whose order is filesystem-dependent,
# so a second match would make resolution nondeterministic. That invariant holds
# for the worktrees worktree_create.sh produces.
resolve_progress_file() {
    local wt="$1" issue="$2" f
    f="$(find "$wt" -maxdepth 7 -type f \
        -path "*/.agent/work-plans/issue-$issue/progress.md" \
        -print -quit 2>/dev/null)"
    if [ -n "$f" ]; then printf '%s\n' "$f"
    else printf '%s\n' "$wt/.agent/work-plans/issue-$issue/progress.md"; fi
}

# Return the LAST matching entry's `when|status` signature (Gap 2, #552). Uses
# the SAME match predicate as entry_count (full type == want, base_type == want,
# OR type startswith want) so the freshness gate keys on the same entry the
# count gate does. Empty string when the file is absent or no entry matches.
# Lives above the source-guard so the regression test can source + call it
# without docker.
#
# The signature is `when|status` (operator decision, #552: `by` dropped). A
# same-minute/same-status re-dispatch yields an IDENTICAL signature and so reads
# as not-fresh => FAILED. That is FAIL-CLOSED (the safe direction: a genuinely
# stuck re-dispatch is the far more likely cause of a same-minute/same-status
# flat count than a real success), and is the accepted trade-off.
last_entry_signature() {
    local progress_file="$1" want="${2:-}"
    [ -f "$progress_file" ] || { printf '%s' ""; return 0; }
    python3 "$SCRIPT_DIR/progress_read.py" "$progress_file" 2>/dev/null \
        | ENTRY_TYPE="$want" python3 -c 'import os,sys,json
want = os.environ.get("ENTRY_TYPE", "")
try: entries = json.load(sys.stdin).get("entries", [])
except Exception: print(""); sys.exit(0)
def match(e):
    t = e.get("type") or ""
    return (not want) or t == want or e.get("base_type") == want or t.startswith(want)
m = [e for e in entries if match(e)]
if m:
    e = m[-1]
    print("%s|%s" % ((e.get("when") or ""), (e.get("status") or "")))
else:
    print("")' 2>/dev/null || printf '%s' ""
}

# Pure freshness test (Gap 2, #552): is POST a genuine new/updated entry vs PRE?
# Fresh iff:
#   - POST_COUNT > PRE_COUNT                                  (append — new entry)
#   - OR (POST_COUNT == PRE_COUNT AND POST_SIG non-empty
#         AND POST_SIG != PRE_SIG)                            (replace — the typed
#                                                              entry's when|status
#                                                              changed in place)
# A flat count with an identical (or empty) signature is NOT fresh => the gate
# reports FAILED ("died before reporting"). No side effects, no globals — the
# regression test exercises all three branches without docker. Returns 0 (true)
# when fresh, 1 otherwise. Callers MUST invoke it inside an `if`/`! ` so the
# false return doesn't trip `set -e`.
is_fresh_entry() {
    local pre_count="$1" post_count="$2" pre_sig="$3" post_sig="$4"
    if [ "$post_count" -gt "$pre_count" ]; then
        return 0
    fi
    if [ "$post_count" -eq "$pre_count" ] && [ -n "$post_sig" ] && [ "$post_sig" != "$pre_sig" ]; then
        return 0
    fi
    return 1
}

# Canonical container-auth token paths (read by docker_run_agent.sh). Documented
# here too so they're discoverable without grepping that script (#532).
CLAUDE_TOKEN_FILE="$HOME/.config/ros2-agent/claude-oauth-token"
GH_TOKEN_FILE="$HOME/.config/ros2-agent/gh-readonly-token"

# Container-dispatch auth preflight (#532). Verifies the long-lived Claude
# subscription token (env or file) and reports the optional read-only GH token.
# Prints the canonical paths + a ready/not-ready verdict. Returns non-zero when
# the Claude token is missing. In-process mode uses the host session and needs
# neither token, so this is a container-mode concern.
preflight_check() {
    local ok=0
    echo "Container-dispatch auth preflight (#532)"
    echo "  Claude subscription token:"
    if [ -n "${CLAUDE_CODE_OAUTH_TOKEN:-}" ]; then
        echo "    ok  \$CLAUDE_CODE_OAUTH_TOKEN set in env"
    elif [ -s "$CLAUDE_TOKEN_FILE" ]; then
        echo "    ok  $CLAUDE_TOKEN_FILE"
    elif [ -n "${ANTHROPIC_API_KEY:-}" ]; then
        echo "    ok  \$ANTHROPIC_API_KEY set (API billing, not subscription)"
    elif [ -f "$HOME/.claude/.credentials.json" ]; then
        echo "    ok  ~/.claude/.credentials.json (mounted OAuth — accepted by"
        echo "        docker_run_agent.sh, but short-lived and can't refresh"
        echo "        in-container; 'claude setup-token' is more robust)"
    else
        echo "    MISSING — generate with 'claude setup-token', then save to:"
        echo "             $CLAUDE_TOKEN_FILE   (chmod 600)"
        ok=1
    fi
    echo "  GitHub token (optional — container reads only; the host publishes):"
    if [ -n "${AGENT_GH_TOKEN:-}" ]; then
        echo "    ok  \$AGENT_GH_TOKEN set in env"
    elif [ -s "$GH_TOKEN_FILE" ]; then
        echo "    ok  $GH_TOKEN_FILE"
    else
        echo "    none — container can't post to GitHub; that is expected (the host"
        echo "          publishes review comments / PRs — local-first, see #532)."
    fi
    if [ "$ok" -eq 0 ]; then
        echo "  => READY for container dispatch."
    else
        echo "  => NOT READY (Claude token missing)."
    fi
    return "$ok"
}

usage() {
    # Keep the end line in sync with the header Usage block (currently ends at
    # line 20 — the --model/--check note); a too-small range truncates --help.
    sed -n '2,21p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

# Source-guard: when this script is `source`d (e.g. by the regression test to
# unit-test resolve_progress_file), stop here so only the function definitions
# above load — no dispatch runs. Direct execution ($0 == this file) falls
# through unchanged. The `[[ ]] && return` is an AND-OR list, so the false
# branch is exempt from `set -e`.
[[ "${BASH_SOURCE[0]}" != "${0}" ]] && return 0

# ---------- Argument parsing ----------

while [[ $# -gt 0 ]]; do
    case "$1" in
        --mode)          MODE="${2:-}"; shift 2 ;;
        --issue)         ISSUE="${2:-}"; shift 2 ;;
        --skill)         SKILL="${2:-}"; shift 2 ;;
        --prompt-file)   PROMPT_FILE="${2:-}"; shift 2 ;;
        --entry-type)    ENTRY_TYPE="${2:-}"; shift 2 ;;
        --model)         MODEL="${2:-}"; shift 2 ;;
        --output-format) OUTPUT_FORMAT="${2:-}"; shift 2 ;;
        --repo-slug)     REPO_SLUG="${2:-}"; shift 2 ;;
        --context-file)  CONTEXT_FILE="${2:-}"; shift 2 ;;
        --check)         CHECK=1; shift ;;
        -h|--help)       usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
    esac
done

# ---------- Auth preflight (#532): `--check` runs standalone and exits ----------

if [ -n "$CHECK" ]; then
    # Explicit if/else: under `set -e` a bare `preflight_check` returning
    # non-zero would exit before `exit $?` ran (making it partly dead code).
    if preflight_check; then exit 0; else exit 1; fi
fi

# ---------- Validation ----------

case "$MODE" in
    in-process|container) : ;;
    *) echo "ERROR: --mode must be 'in-process' or 'container'." >&2; usage >&2; exit 1 ;;
esac
if [ -z "$ISSUE" ]; then
    echo "ERROR: --issue <N> is required." >&2; exit 1
fi
if ! [[ "$ISSUE" =~ ^[0-9]+$ ]]; then
    echo "ERROR: --issue must be a number (got '$ISSUE')." >&2; exit 1
fi
# Sanitize --repo-slug to match how worktree_create.sh names dirs (non
# [A-Za-z0-9_] -> _), so a hyphenated slug still resolves (#526).
[ -n "$REPO_SLUG" ] && REPO_SLUG="${REPO_SLUG//[^A-Za-z0-9_]/_}"
if [ -n "$SKILL" ] && [ -n "$PROMPT_FILE" ]; then
    echo "ERROR: pass exactly one of --skill or --prompt-file, not both." >&2; exit 1
fi
if [ -z "$SKILL" ] && [ -z "$PROMPT_FILE" ]; then
    echo "ERROR: pass one of --skill <name> or --prompt-file <path>." >&2; exit 1
fi
if [ -n "$PROMPT_FILE" ] && [ ! -f "$PROMPT_FILE" ]; then
    echo "ERROR: --prompt-file not found: $PROMPT_FILE" >&2; exit 1
fi
# --context-file is COMPOSABLE with --skill (unlike --prompt-file): it injects a
# host-fetched issue/PR body into the handoff while the skill keeps its auto
# entry-type + auto model (#552). Validate existence the same way as
# --prompt-file; no mutual-exclusion check.
if [ -n "$CONTEXT_FILE" ] && [ ! -f "$CONTEXT_FILE" ]; then
    echo "ERROR: --context-file not found: $CONTEXT_FILE" >&2; exit 1
fi
case "$OUTPUT_FORMAT" in
    stream-json|json|text) : ;;
    *) echo "ERROR: --output-format must be one of stream-json|json|text (got '$OUTPUT_FORMAT')." >&2; exit 1 ;;
esac

# Resolve the expected entry type: explicit --entry-type wins, else derive from
# the skill. May be empty for a raw prompt with no declared type.
if [ -z "$ENTRY_TYPE" ] && [ -n "$SKILL" ]; then
    ENTRY_TYPE="$(skill_entry_type "$SKILL")"
fi

# Resolve the model: explicit --model wins; else derive from the skill; else,
# for a raw --prompt-file dispatch (no skill), derive from the declared
# --entry-type so an Implementation / review pass isn't silently downgraded to
# Sonnet (#539); else the default ('sonnet').
if [ -z "$MODEL" ]; then
    if [ -n "$SKILL" ]; then MODEL="$(skill_model "$SKILL")"
    elif [ -n "$ENTRY_TYPE" ]; then MODEL="$(entry_type_model "$ENTRY_TYPE")"
    else MODEL="sonnet"; fi
fi
MODEL_DISPLAY="$(model_display "$MODEL")"

# ---------- Resolve the worktree (progress.md is resolved per-call below) ----------

WORKTREE_PATH=""
WORKTREE_MATCHES=0
MATCHED=()
if [ -n "$REPO_SLUG" ]; then
    # --repo-slug: resolve the layer worktree exactly (issue-<slug>-<N>), like
    # worktree_remove.sh. No glob -> no cross-repo collision (#526).
    candidate="$ROOT_DIR/layers/worktrees/issue-$REPO_SLUG-$ISSUE"
    if [ -d "$candidate" ]; then WORKTREE_PATH="$candidate"; WORKTREE_MATCHES=1; MATCHED=("$candidate"); fi
else
    for candidate in \
        "$ROOT_DIR/.workspace-worktrees/issue-workspace-$ISSUE" \
        "$ROOT_DIR/layers/worktrees/issue-"*"-$ISSUE"; do
        if [ -d "$candidate" ]; then
            MATCHED+=("$candidate")
            WORKTREE_MATCHES=$((WORKTREE_MATCHES + 1))
            [ -z "$WORKTREE_PATH" ] && WORKTREE_PATH="$candidate"
        fi
    done
fi
if [ -z "$WORKTREE_PATH" ]; then
    echo "ERROR: no worktree found for issue #$ISSUE${REPO_SLUG:+ (repo-slug '$REPO_SLUG')} (create it first)." >&2; exit 1
fi
if [ "$WORKTREE_MATCHES" -gt 1 ]; then
    # FAIL LOUD (#526) — was warn-and-proceed. Multiple repos can share an issue
    # number; guessing the first match alphabetically once ran a review against
    # the wrong repo. Refuse, and tell the operator how to disambiguate.
    {
        echo "ERROR: $WORKTREE_MATCHES worktrees match issue #$ISSUE — refusing to guess."
        echo "       Disambiguate with --repo-slug <slug>. Candidates:"
        printf '         %s\n' "${MATCHED[@]}"
    } >&2
    exit 1
fi

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

# ---------- Optional host-injected read context (Gap 1, #552) ----------
# When --context-file is set, splice the host-fetched GitHub context (issue/PR
# body) into the handoff AFTER the task body and BEFORE the handoff contract, so
# a container phase with no GitHub read auth reads the injected section instead
# of running `gh issue view`. CONTEXT_SECTION is empty (and the handoff is
# byte-identical to the no-context case) when the flag is unset.
CONTEXT_SECTION=""
if [ -n "$CONTEXT_FILE" ]; then
    CONTEXT_BODY="$(cat "$CONTEXT_FILE")"
    CONTEXT_SECTION="$(cat <<EOF

## Injected GitHub context (issue/PR body, fetched host-side)

$CONTEXT_BODY

Use the injected context above **instead of** \`gh issue view\` / \`gh pr view\`
— this dispatch has no GitHub read auth. Treat it as the canonical issue/PR body.
EOF
)"
    # Transparency (#552, human-control principle): log WHAT went into the handoff
    # — byte size + first non-empty line — never the raw contents (which could
    # carry tokens/secrets in pathological inputs).
    ctx_bytes="$(wc -c < "$CONTEXT_FILE" | tr -d ' ')"
    ctx_heading="$(grep -m1 '[^[:space:]]' "$CONTEXT_FILE" 2>/dev/null || true)"
    echo "Injected host-side context from $CONTEXT_FILE (${ctx_bytes} bytes; first line: ${ctx_heading:-<empty>})" >&2
fi

HANDOFF="$(cat <<EOF
$TASK_BODY
$CONTEXT_SECTION
---
## Sub-agent handoff contract (#490)

You are a fresh-context sub-agent dispatched for **issue #$ISSUE**. Work only
within this issue's worktree; do not touch other issues.

**Commit identity** — every \`git commit\` MUST carry per-invocation identity
literals (AGENTS.md § Agent Commit Identity):

    git -c user.name="$DISPATCH_AGENT_NAME" -c user.email="$DISPATCH_AGENT_EMAIL" commit -m "..."

Never use \`--no-verify\`. Do **not** \`git push\` — the host performs pushes
with its own credentials. Never write credentials/tokens (e.g.
\`\$CLAUDE_CODE_OAUTH_TOKEN\`) into files, commits, or output.

**Your runtime model** is **$MODEL_DISPLAY** (dispatched with \`--model $MODEL\`).
Stamp **exactly that string** in the \`**By**:\` line of your progress.md entry —
do not add to or embellish it (never expand "Claude Opus" into a guessed
"Opus 4.6"; if a pinned model id was given, stamp it verbatim).

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
    echo "Recommended model: $MODEL (in-process can't set it — pick it when you"
    echo "spawn the Agent; container mode passes it automatically)."
    echo "---------------8<--------------- BEGIN HANDOFF ---------------8<---------------"
    printf '%s\n' "$HANDOFF"
    echo "---------------8<---------------- END HANDOFF ----------------8<---------------"
    exit 0
fi

# ---------- container mode: launch headless docker, then read the contract ----------

# Count progress.md entries (optionally of a given type) — used to require a
# *newer* entry after dispatch. Missing file => 0.
#
# Type matching is done here, NOT via progress_read.py's `--type` filter,
# because that filter matches `type`/`base_type` exactly and `Local Review
# (Pre-Push)` is itself a canonical type (its `base_type` is the full
# `Local Review (Pre-Push)`, not the stripped `Local Review`). review-code in
# pre-push mode resolves ENTRY_TYPE="Local Review" but writes a
# `## Local Review (Pre-Push)` heading, so an exact filter misses it and the
# count stays flat → a false "died before reporting". We therefore count an
# entry when its full type equals ENTRY_TYPE, its base_type equals ENTRY_TYPE,
# OR its type begins with ENTRY_TYPE (catching the parenthetical variant).
entry_count() {
    # Re-resolve every call: on the first phase the file is absent pre-dispatch,
    # so only a fresh post-dispatch resolve finds the freshly-written (possibly
    # layer-nested) file. PRE: absent -> 0; POST: nested file found -> 1.
    local PROGRESS_FILE
    PROGRESS_FILE="$(resolve_progress_file "$WORKTREE_PATH" "$ISSUE")"
    [ -f "$PROGRESS_FILE" ] || { echo 0; return 0; }
    python3 "$SCRIPT_DIR/progress_read.py" "$PROGRESS_FILE" 2>/dev/null \
        | ENTRY_TYPE="$ENTRY_TYPE" python3 -c 'import os,sys,json
want = os.environ.get("ENTRY_TYPE", "")
try: entries = json.load(sys.stdin).get("entries", [])
except Exception: print(0); sys.exit(0)
def match(e):
    t = e.get("type") or ""
    return (not want) or t == want or e.get("base_type") == want or t.startswith(want)
print(sum(1 for e in entries if match(e)))' 2>/dev/null || echo 0
}

PRE_COUNT="$(entry_count)"
# Fail closed: a non-numeric PRE_COUNT becomes -1 so any valid POST_COUNT
# (>= 0) still requires a real new entry rather than spuriously passing.
[[ "$PRE_COUNT" =~ ^[0-9]+$ ]] || PRE_COUNT=-1
# Capture the PRE signature alongside the count so the freshness gate (Gap 2,
# #552) can detect a same-count REPLACE (re-dispatch overwriting the typed entry).
PRE_SIG="$(last_entry_signature "$(resolve_progress_file "$WORKTREE_PATH" "$ISSUE")" "$ENTRY_TYPE")"

PROMPT_TMP="$(mktemp /tmp/dispatch_handoff.XXXXXX.md)"
trap 'rm -f "$PROMPT_TMP"' EXIT
printf '%s\n' "$HANDOFF" > "$PROMPT_TMP"

# Soft auth preflight (#532): warn early and actionably if the Claude token is
# absent, rather than failing opaquely deep inside docker_run_agent.sh.
if ! preflight_check >/dev/null 2>&1; then
    echo "⚠️  Container auth not ready — run '${BASH_SOURCE[0]} --check' for details." >&2
fi

echo "Dispatching issue #$ISSUE into a container (headless, model=$MODEL)…" >&2
RC=0
"$SCRIPT_DIR/docker_run_agent.sh" \
    --issue "$ISSUE" \
    ${REPO_SLUG:+--repo-slug "$REPO_SLUG"} \
    --output-format "$OUTPUT_FORMAT" \
    --model "$MODEL" \
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
# Fail closed: a non-numeric POST_COUNT becomes 0 so the integer comparisons in
# is_fresh_entry below can't error (status 2) into the success branch and print
# a false "Result: OK".
[[ "$POST_COUNT" =~ ^[0-9]+$ ]] || POST_COUNT=0
POST_SIG="$(last_entry_signature "$(resolve_progress_file "$WORKTREE_PATH" "$ISSUE")" "$ENTRY_TYPE")"
# Gate on last-entry FRESHNESS, not raw count delta (Gap 2, #552). A re-dispatch
# that REPLACES a typed entry (e.g. a prior `failed ## Issue Review` -> a
# `complete` one) keeps the count flat (1->1); the old count-delta gate read
# that as a false FAILED. is_fresh_entry treats a flat count with a changed
# `when|status` signature as fresh (replace), and only a flat count AND an
# identical/empty signature as FAILED. The existing fail-closed numeric guards
# above are preserved. NOTE: a same-minute/same-status re-dispatch yields an
# identical signature => FAILED — fail-closed, accepted (see last_entry_signature).
if ! is_fresh_entry "$PRE_COUNT" "$POST_COUNT" "$PRE_SIG" "$POST_SIG"; then
    echo "  Result: FAILED — no fresh \`## ${ENTRY_TYPE:-<typed>}\` entry"
    echo "          (count ${PRE_COUNT} -> ${POST_COUNT}, signature unchanged);"
    echo "          sub-agent died before reporting, or wrote no new/updated entry."
    echo "  Inspect the worktree: $WORKTREE_PATH"
    exit 1
fi

# A new entry exists. Read its status (last entry matching the gate, same
# type/base_type/prefix predicate as entry_count, so the displayed entry is the
# one the count gate keyed on) and make the headline track it — #492 greps
# "Result: …". Emits STATUS on the first line and the display fields after.
PROGRESS_FILE="$(resolve_progress_file "$WORKTREE_PATH" "$ISSUE")"
LAST_ENTRY="$(
    python3 "$SCRIPT_DIR/progress_read.py" "$PROGRESS_FILE" 2>/dev/null \
    | ENTRY_TYPE="$ENTRY_TYPE" python3 -c '
import os, sys, json
want = os.environ.get("ENTRY_TYPE", "")
try:
    entries = json.load(sys.stdin).get("entries", [])
except Exception:
    entries = []
def match(e):
    t = e.get("type") or ""
    return (not want) or t == want or e.get("base_type") == want or t.startswith(want)
e = [x for x in entries if match(x)]
if e:
    x = e[-1]
    print((x.get("status") or "").strip().lower())
    for k in ("type", "status", "by", "when"):
        print("    %-7s %s" % (k + ":", x.get(k)))
'
)"
LAST_STATUS="$(printf '%s\n' "$LAST_ENTRY" | head -n1)"
ENTRY_DISPLAY="$(printf '%s\n' "$LAST_ENTRY" | tail -n +2)"

case "$LAST_STATUS" in
    failed)
        echo "  Result: FAILED — sub-agent reported \`**Status**: failed\`. Last entry:" ;;
    partial)
        echo "  Result: PARTIAL — sub-agent reported \`**Status**: partial\`. Last entry:" ;;
    complete)
        echo "  Result: OK — sub-agent wrote a \`complete\` entry. Last entry:" ;;
    *)
        echo "  Result: OK — sub-agent wrote a new entry (status '${LAST_STATUS:-unknown}'). Last entry:" ;;
esac
if [ -n "$ENTRY_DISPLAY" ]; then
    printf '%s\n' "$ENTRY_DISPLAY"
else
    echo "    (could not parse progress.md)"
fi
case "$LAST_STATUS" in
    failed)  echo "  Next: the host inspects the failure in $WORKTREE_PATH."; exit 1 ;;
    partial) echo "  Next: the host inspects the partial work in $WORKTREE_PATH."; exit 1 ;;
    *)       echo "  Next: the host reviews the entry and runs the next phase / does the push." ;;
esac
