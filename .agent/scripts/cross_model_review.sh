#!/usr/bin/env bash
# Cross-model adversarial review via external CLI agents
#
# Runs one or more external CLI agents to provide independent adversarial
# reviews of a PR or local branch. Writes prompt and findings to
# .agent/work-plans/issue-<N>/ alongside the work plan — or to a /tmp dir
# under --no-progress. These files are not committed (gitignored under
# .agent/work-plans/, outside the repo under --no-progress). Regenerated
# each run, not part of the audit trail (durable findings live in
# progress.md). See #193 for the recursive-bloat failure mode that
# motivated this.
#
# Supported agents: gemini, codex, claude, copilot
# (the gemini agent runs via the `agy` binary — see AGENT_BINS — through
# the _agy_review.sh helper, which feeds the prompt over stdin and
# validates agy's result event; issues #274, #288)
#
# No agent CLI is invoked from this script directly. Gemini goes through
# _agy_review.sh; codex, claude and copilot go through _cli_review.sh
# (issue #313, folding in #212). Each helper owns its findings file:
# it truncates the file first, runs its CLI with the prompt on stdin,
# and writes either the review text or a failure reason — so an empty
# response, a quota / rate-limit / auth error or a missing result is a
# failed review here, not review-looking text in the findings file.
#
# The embedded diff excludes .agent/work-plans/** (plan.md, progress.md,
# review artifacts): review bookkeeping, not code under review, and the
# main source of oversized prompts (#312).
#
# Execution model (rolker/agent_workspace ADR-0015, issue #206): every
# agent runs synchronously in
# its own background job, all selected agents in parallel, and the script
# blocks until the last one finishes. There is no tmux mode any more —
# reviews run headless, so the interactive session it provided had no
# remaining value, and the sandboxed callers it silently downgraded to
# sequential runs get parallelism back. `--sync` (the old opt-out) is
# rejected as removed. Live observation: `tail -f <findings-file>`.
#
# Every agent is bounded so one hung CLI cannot hang the call. Codex,
# claude and copilot run under `timeout "$AGENT_TIMEOUT"` (coreutils
# duration, env-overridable, default 1800), applied to _cli_review.sh,
# which forwards the signal to its CLI child. Gemini's primary bound stays
# _agy_review.sh's own --print-timeout (`AGY_PRINT_TIMEOUT`), because that
# path reports the expiry with a reason; it also gets an outer
# `timeout "$GEMINI_BACKSTOP"` derived to sit ABOVE the print-timeout
# (print-timeout + GEMINI_BACKSTOP_MARGIN, default 300s) so a helper or
# agy that wedges without ever honouring its own timeout is still cut off.
# The margin is what keeps the backstop from racing agy's
# timeout-then-partial-response contract (#288): under normal operation
# the print-timeout always fires first and the backstop never triggers.
#
# Usage:
#   .agent/scripts/cross_model_review.sh --pr <N>                              # gemini (default)
#   .agent/scripts/cross_model_review.sh --pr <N> --agent codex                # one specific agent
#   .agent/scripts/cross_model_review.sh --pr <N> --agents gemini,codex,copilot # several, in parallel
#   .agent/scripts/cross_model_review.sh --branch --agents gemini,codex        # local pre-push review
#   .agent/scripts/cross_model_review.sh --pr <N> --repo owner/repo            # explicit repo target
#   .agent/scripts/cross_model_review.sh --pr <N> --work-dir /path/to/worktree # explicit artifact dir
#
# --agent and --agents are mutually exclusive. --agents takes a comma list:
# entries are trimmed and lowercased, empty entries (stray commas) are
# rejected, exact duplicates collapse to one run, unknown names exit 2.
#
# The script runs in whichever repo worktree it's invoked from.
# Workspace issues run in workspace worktrees, project issues in project worktrees.
#
# Output (stdout), --agent <X> (single-agent, unchanged contract):
#   MODE=sync                        (machine-parseable)
#   AGENT=<agent-key>                (machine-parseable)
#   FINDINGS_FILE=<path-to-findings> (machine-parseable)
#   followed by informational lines for human consumption
#
# Output (stdout), --agents <list> (one entry or more):
#   MODE=parallel-sync               (once)
#   AGENT=<agent-key>                (one triplet per agent, after all
#   FINDINGS_FILE=<path-to-findings>  agents have finished; EXIT= is that
#   EXIT=<n>                          agent job's exit status, 0 = success.
#                                     A failed review is 1 from its helper
#                                     — the CLI's own status is named in
#                                     the findings file's reason — or 124
#                                     when the outer timeout cut it off.)
#   followed by informational lines for human consumption
#
# Whenever the agents actually run, each findings file ends with
# `--- Review complete ---` or `--- Review failed ---` (reason on the lines
# above), written by the agent's own job the moment it finishes — a slow
# agent never delays a fast agent's marker. Two paths end without such a
# marker: the shared-prompt abort (exit 3, no triplets) truncates every
# selected findings file to a `--- Review error: ... ---` marker instead,
# and an interrupted run (SIGINT/SIGTERM) can leave a file with no marker
# at all. Readers must handle a marker-less file rather than block on one.
#
# Informational lines naming each agent's findings file are printed
# BEFORE the agents launch (so `tail -f` works while they run); the
# triplets follow once every agent has finished. Callers parse by line
# prefix, not by position.
#
# Interrupting the script (SIGINT/SIGTERM) or any early exit kills every
# agent job it started; nothing is left running in the background.
#
# Exit codes:
#   0 — every selected agent completed successfully
#   1 — missing dependencies: gh (PR mode), or no selected agent has a
#       usable CLI. Nothing is written and no triplet is printed; each
#       unavailable agent is named on stderr. Distinct from exit 3.
#   2 — invalid arguments
#   3 — failed to build the prompt (no AGENT= triplets printed under
#       --agents: the shared diff fetch failed or was empty and every
#       selected findings file carries the `--- Review error: ... ---`
#       marker), OR at least one agent failed (triplets printed — read
#       EXIT= per agent). The presence of triplets is the disambiguator.
#   4 — wrong worktree / invalid environment (see _resolve_work_plans_dir.sh)

set -euo pipefail

# --- Agent configuration ---
# Binary name to search for in PATH and fallback locations.
# The Gemini CLI migrated to the `agy` binary (issue #223); the agent key
# stays "gemini" so existing callers (--agent gemini, review-code skill
# mapping, artifact filenames) keep working.
declare -A AGENT_BINS=(
    ["gemini"]="agy"
    ["codex"]="codex"
    ["claude"]="claude"
    ["copilot"]="copilot"
)

# Timeout for agy print mode (Go duration format). agy's own default is
# 0s = wait until the turn completes; the explicit cap keeps a hung
# review from blocking the caller forever. On expiry agy exits 0 with a
# partial response — _agy_review.sh treats that as a failed review (#288).
# Env-overridable so tests can inject a small value.
AGY_PRINT_TIMEOUT="${AGY_PRINT_TIMEOUT:-30m}"

# Outer per-agent bound for codex/claude/copilot (coreutils `timeout`
# duration: a number of seconds, or with an s/m/h suffix). Env-overridable
# so tests can inject a small value. 124 is timeout's own "expired"
# status and counts as that agent's failure. A CLI that ignores the
# SIGTERM gets SIGKILL after AGENT_KILL_AFTER, so a stuck process cannot
# outlive the bound.
AGENT_TIMEOUT="${AGENT_TIMEOUT:-1800}"
AGENT_KILL_AFTER="${AGENT_KILL_AFTER:-10}"

# Seconds added to AGY_PRINT_TIMEOUT to derive gemini's outer backstop.
GEMINI_BACKSTOP_MARGIN="${GEMINI_BACKSTOP_MARGIN:-300}"

# Convert a coreutils `timeout` duration (`90`, `90s`, `30m`, `1.5h`, `1d`)
# to whole seconds on stdout. Returns 1 on a malformed value.
duration_to_seconds() {
    [[ "$1" =~ ^([0-9]+(\.[0-9]+)?)([smhd]?)$ ]] || return 1
    local number="${BASH_REMATCH[1]}" unit="${BASH_REMATCH[3]}" mult=1
    case "$unit" in
        m) mult=60 ;;
        h) mult=3600 ;;
        d) mult=86400 ;;
        *) mult=1 ;;
    esac
    awk -v n="$number" -v m="$mult" 'BEGIN { printf "%.0f", n * m }'
}

# Validate one duration knob up front, because every way a bad value can
# fail later is worse than exiting 2 here: a shape coreutils `timeout`
# rejects surfaces as a bare exit 125 from every agent job (reads as "the
# CLI failed"), and a zero silently removes a bound rather than setting a
# short one.
#   $3 allow_zero  — true only for AGENT_KILL_AFTER, where 0 legitimately
#                    means "send SIGKILL immediately after the SIGTERM"
#   $4 go_shape    — true for a value handed to agy's --print-timeout
#   $5 zero_reason — why zero is wrong for this particular knob
validate_duration_knob() {
    local name="$1" value="$2" allow_zero="$3" go_shape="$4" zero_reason="$5" seconds
    # Go's time.ParseDuration (agy's --print-timeout parser) requires an
    # explicit unit and has no `d`, so the coreutils shapes `90` and `1d`
    # would pass a `timeout`-only check and then fail inside agy at
    # runtime — after the prompt is built and the job has launched.
    if [[ "$go_shape" == true && ! "$value" =~ ^[0-9]+(\.[0-9]+)?[smh]$ ]]; then
        echo "ERROR: ${name} value '${value}' is not a valid Go duration. It is passed straight to agy's --print-timeout, which needs an explicit s/m/h unit (e.g. 90s, 30m, 1.5h): a bare number and a 'd' suffix are both rejected there." >&2
        exit 2
    fi
    if ! seconds=$(duration_to_seconds "$value"); then
        echo "ERROR: ${name} value '${value}' is not a valid duration (a positive number with an optional s/m/h/d suffix, e.g. 1800, 30m, 1.5h)" >&2
        exit 2
    fi
    if [[ "$allow_zero" != true && "$seconds" -le 0 ]]; then
        echo "ERROR: ${name} value '${value}' must be greater than zero (whole seconds after rounding). ${zero_reason}" >&2
        exit 2
    fi
}

validate_duration_knob AGENT_TIMEOUT "$AGENT_TIMEOUT" false false \
    "coreutils 'timeout 0' imposes no limit at all, which would leave the agent unbounded — the opposite of what ADR-0015 §3 guarantees."
validate_duration_knob AGENT_KILL_AFTER "$AGENT_KILL_AFTER" true false ""
validate_duration_knob AGY_PRINT_TIMEOUT "$AGY_PRINT_TIMEOUT" false true \
    "agy reads 0 as 'wait until the turn completes', which is the unbounded review this cap exists to prevent."
validate_duration_knob GEMINI_BACKSTOP_MARGIN "$GEMINI_BACKSTOP_MARGIN" false false \
    "a zero margin collapses the backstop onto AGY_PRINT_TIMEOUT, reintroducing the race with the helper's timeout-then-partial-response handling (#288)."

# Gemini's outer backstop, deliberately ABOVE AGY_PRINT_TIMEOUT: the
# helper's own print-timeout must always be the path that fires first, so
# a real expiry is reported with its reason and any partial response is
# handled by _agy_review.sh (#288). The backstop only catches the case
# that contract cannot cover — a helper or agy wedged so hard it never
# honours its own timeout — so an outer SIGTERM never races the normal
# path. A margin of 0 would reintroduce that race, which is why the
# validator above refuses it.
GEMINI_BACKSTOP=$(( $(duration_to_seconds "$AGY_PRINT_TIMEOUT") + $(duration_to_seconds "$GEMINI_BACKSTOP_MARGIN") ))

# How long an interrupted run waits for its agent jobs before dropping
# the shared temp root (cleanup_jobs). It has to outlast the escalation a
# helper legitimately spends killing a CLI that ignored SIGTERM
# (REVIEW_KILL_ESCALATION, read here only to size this budget — the
# helpers own the knob), plus a margin for that helper to exit. A
# hard-coded value (it was 8s) silently breaks as soon as the escalation
# is raised above it (#313 round 3), so the default is derived; an
# explicit CLEANUP_REAP_TIMEOUT is honoured but must clear the same bar.
REVIEW_KILL_ESCALATION="${REVIEW_KILL_ESCALATION:-5}"
validate_duration_knob REVIEW_KILL_ESCALATION "$REVIEW_KILL_ESCALATION" true false ""
CLEANUP_REAP_MARGIN="${CLEANUP_REAP_MARGIN:-3}"
validate_duration_knob CLEANUP_REAP_MARGIN "$CLEANUP_REAP_MARGIN" false false \
    "the parent needs a moment after a helper's own SIGKILL escalation to see that helper exit."
ESCALATION_SECONDS=$(duration_to_seconds "$REVIEW_KILL_ESCALATION")
CLEANUP_REAP_TIMEOUT="${CLEANUP_REAP_TIMEOUT:-$(( ESCALATION_SECONDS + $(duration_to_seconds "$CLEANUP_REAP_MARGIN") ))}"
validate_duration_knob CLEANUP_REAP_TIMEOUT "$CLEANUP_REAP_TIMEOUT" false false \
    "a zero reap budget would drop the shared temp root while the helpers are still writing into it."
CLEANUP_REAP_SECONDS=$(duration_to_seconds "$CLEANUP_REAP_TIMEOUT")
if [[ "$CLEANUP_REAP_SECONDS" -le "$ESCALATION_SECONDS" ]]; then
    echo "ERROR: CLEANUP_REAP_TIMEOUT (${CLEANUP_REAP_TIMEOUT}) must exceed REVIEW_KILL_ESCALATION (${REVIEW_KILL_ESCALATION}): the shared temp root would be removed while a helper is still escalating to SIGKILL on a CLI that is writing into it." >&2
    exit 2
fi

# Helpers that own the agent invocations. A missing helper makes the
# agents it serves unavailable (those agents fail; others still run).
SCRIPT_SELF_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AGY_REVIEW_HELPER="${SCRIPT_SELF_DIR}/_agy_review.sh"
CLI_REVIEW_HELPER="${SCRIPT_SELF_DIR}/_cli_review.sh"

# Run one agent to completion. Args: agent_key, bin_path, prompt_file,
# findings_file. Exits with the helper's status (124 on timeout).
# Always called in a background subshell; `exec` makes that subshell's
# PID the helper's own (via timeout, which forwards signals to it, and
# the helper in turn forwards them to its CLI child), so killing the PID
# the parent holds really stops the agent. Agents read the prompt from
# stdin, so prompt size is not bounded by argv.
#
# Both helpers own their findings file, so there is no stdout redirect
# here (#274, #288, #313): a helper truncates the file first and writes
# either the review or a failure reason into it.
# TMPDIR points at the parent-owned scratch root so a SIGKILLed helper's
# temp dir is still swept by this script's EXIT trap.
run_agent_sync() {
    local agent="$1" bin="$2" prompt="$3" findings="$4"

    case "$agent" in
        # Gemini's outer timeout is the backstop ABOVE the helper's own
        # print-timeout, not a replacement for it.
        # AGENT_KILL_AFTER is exported so the helper can check that its
        # own SIGKILL escalation fits inside this grace (#313 round 2).
        gemini)  exec env TMPDIR="$AGENT_TMP_ROOT" AGENT_KILL_AFTER="$AGENT_KILL_AFTER" timeout -k "$AGENT_KILL_AFTER" "$GEMINI_BACKSTOP" "$AGY_REVIEW_HELPER" "$bin" "$prompt" "$findings" "$AGY_PRINT_TIMEOUT" ;;
        # codex, claude and copilot (and anything that somehow reaches
        # here — _cli_review.sh rejects an unknown agent with a readable
        # reason in the findings file rather than running a CLI blind).
        # AGENT_TIMEOUT is passed through as an informational label so
        # the helper's failure reasons can name the bound they ran under.
        *)       exec env TMPDIR="$AGENT_TMP_ROOT" AGENT_KILL_AFTER="$AGENT_KILL_AFTER" timeout -k "$AGENT_KILL_AFTER" "$AGENT_TIMEOUT" "$CLI_REVIEW_HELPER" "$agent" "$bin" "$prompt" "$findings" "$AGENT_TIMEOUT" ;;
    esac
}

# Resolve an agent's CLI: PATH first, then common install locations.
# Prints the path on stdout (empty when not found); never exits.
resolve_agent_bin() {
    local agent="$1"
    local name="${AGENT_BINS[$agent]}"
    if command -v "$name" &>/dev/null; then
        command -v "$name"
        return 0
    fi
    local candidate
    for candidate in \
        "${HOME}/.nvm/versions/node"/*/bin/"${name}" \
        "${HOME}/.local/bin/${name}" \
        "${HOME}/.npm-global/bin/${name}" \
        /usr/local/bin/"${name}"; do
        if [[ -x "$candidate" ]]; then
            echo "INFO: ${name} not in PATH, found at: ${candidate}" >&2
            echo "$candidate"
            return 0
        fi
    done
    echo ""
}

# --- Argument parsing ---
PR_NUMBER=""
BRANCH_MODE=false
BRANCH_BASE=""
NO_PROGRESS=false
CLI_ISSUE_NUMBER=""
SINGLE_AGENT=""
AGENTS_LIST=""
EXPLICIT_REPO=""
EXPLICIT_WORK_DIR=""
CLI_WORK_PLANS_DIR=""

USAGE="Usage: $0 (--pr <N> | --branch [<ref>]) [--issue <N>] [--agent <name> | --agents <a,b,...>] [--repo owner/repo] [--work-dir <path>] [--work-plans-dir <path>] [--no-progress]"

# Helper: treat a missing value OR a value that looks like another long
# flag (`--foo`) as "missing value." Narrower than `-*` so negative
# integers and dash-prefixed paths fall through to their dedicated
# validators (per review feedback on #149).
require_value() {
    local flag="$1"
    local value="$2"
    if [[ -z "$value" || "$value" == --* ]]; then
        echo "ERROR: Missing value for $flag" >&2
        echo "$USAGE" >&2
        exit 2
    fi
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --pr)
            require_value "--pr" "${2:-}"
            PR_NUMBER="$2"
            shift 2
            ;;
        --branch)
            BRANCH_MODE=true
            # --branch takes an optional value. Treat the next token as
            # the base ref only if it doesn't look like another flag and
            # isn't empty. Otherwise leave BRANCH_BASE empty so the
            # default-branch resolver runs.
            if [[ -n "${2:-}" && "${2}" != --* ]]; then
                BRANCH_BASE="$2"
                shift 2
            else
                shift 1
            fi
            ;;
        --no-progress)
            NO_PROGRESS=true
            shift 1
            ;;
        --issue)
            require_value "--issue" "${2:-}"
            CLI_ISSUE_NUMBER="$2"
            shift 2
            ;;
        --agent)
            require_value "--agent" "${2:-}"
            SINGLE_AGENT="${2,,}"  # lowercase
            shift 2
            ;;
        --agents)
            require_value "--agents" "${2:-}"
            AGENTS_LIST="$2"
            shift 2
            ;;
        --repo|-R)
            require_value "--repo" "${2:-}"
            EXPLICIT_REPO="$2"
            shift 2
            ;;
        --work-dir)
            require_value "--work-dir" "${2:-}"
            EXPLICIT_WORK_DIR="$2"
            shift 2
            ;;
        --work-plans-dir)
            require_value "--work-plans-dir" "${2:-}"
            CLI_WORK_PLANS_DIR="$2"
            shift 2
            ;;
        --sync)
            echo "ERROR: --sync was removed (#206, ADR-0015): synchronous parallel dispatch is the only mode now; drop the flag." >&2
            echo "$USAGE" >&2
            exit 2
            ;;
        *)
            echo "ERROR: Unknown argument: $1" >&2
            echo "$USAGE" >&2
            exit 2
            ;;
    esac
done

if [[ -n "$PR_NUMBER" && "$BRANCH_MODE" == true ]]; then
    echo "ERROR: --pr and --branch are mutually exclusive." >&2
    echo "" >&2
    echo "  Use --pr <N> for PR mode (post-push review)" >&2
    echo "  or --branch [<base>] for branch mode (local pre-push review)." >&2
    exit 2
fi
if [[ -z "$PR_NUMBER" && "$BRANCH_MODE" != true ]]; then
    echo "ERROR: one of --pr <N> or --branch [<ref>] is required" >&2
    echo "$USAGE" >&2
    exit 2
fi

# --- Agent selection ---
# --agent <X> (or neither flag: gemini) is the single-agent contract;
# --agents <list> is the multi-agent contract, even with one entry.
if [[ -n "$SINGLE_AGENT" && -n "$AGENTS_LIST" ]]; then
    echo "ERROR: --agent and --agents are mutually exclusive." >&2
    echo "$USAGE" >&2
    exit 2
fi

MULTI_AGENT=false
AGENTS_TO_RUN=()
declare -A SEEN_AGENT=()
if [[ -n "$AGENTS_LIST" ]]; then
    MULTI_AGENT=true
    # A leading or trailing comma is an empty entry too; `read -a` would
    # silently drop a trailing one, so check the raw string first.
    if [[ "$AGENTS_LIST" == ,* || "$AGENTS_LIST" == *, ]]; then
        echo "ERROR: --agents has an empty entry (stray comma?): '${AGENTS_LIST}'" >&2
        exit 2
    fi
    IFS=',' read -r -a raw_agents <<< "$AGENTS_LIST"
    for raw in "${raw_agents[@]}"; do
        # Trim surrounding whitespace, lowercase.
        entry="${raw#"${raw%%[![:space:]]*}"}"
        entry="${entry%"${entry##*[![:space:]]}"}"
        entry="${entry,,}"
        if [[ -z "$entry" ]]; then
            echo "ERROR: --agents has an empty entry (stray comma?): '${AGENTS_LIST}'" >&2
            exit 2
        fi
        if [[ -z "${AGENT_BINS[$entry]+x}" ]]; then
            echo "ERROR: Unknown agent '${entry}' in --agents" >&2
            echo "Supported agents: ${!AGENT_BINS[*]}" >&2
            exit 2
        fi
        # Exact duplicates collapse to one run: two jobs for the same
        # agent would race on one prompt/findings filename.
        if [[ -z "${SEEN_AGENT[$entry]+x}" ]]; then
            SEEN_AGENT["$entry"]=1
            AGENTS_TO_RUN+=("$entry")
        fi
    done
    # A list that was entirely empty entries cannot reach here (each is
    # rejected above), so AGENTS_TO_RUN has at least one element.
else
    SINGLE_AGENT="${SINGLE_AGENT:-gemini}"
    if [[ -z "${AGENT_BINS[$SINGLE_AGENT]+x}" ]]; then
        echo "ERROR: Unknown agent '${SINGLE_AGENT}'" >&2
        echo "Supported agents: ${!AGENT_BINS[*]}" >&2
        exit 2
    fi
    AGENTS_TO_RUN=("$SINGLE_AGENT")
fi

# Validate --repo slug (before dependency checks so bad input always exits 2)
if [[ -n "$EXPLICIT_REPO" && ! "$EXPLICIT_REPO" =~ ^[^/[:space:]]+/[^/[:space:]]+$ ]]; then
    echo "ERROR: --repo value '${EXPLICIT_REPO}' is not a valid owner/repo slug" >&2
    exit 2
fi

# Validate --pr is a bare positive integer. Without this, a typo'd value
# reaches `gh pr view` and comes back as a generic retrieval failure that
# blames auth or the network instead of the argument.
if [[ -n "$PR_NUMBER" && ! "$PR_NUMBER" =~ ^[1-9][0-9]*$ ]]; then
    echo "ERROR: --pr value '${PR_NUMBER}' is not a positive integer" >&2
    exit 2
fi

# Validate --issue is a bare positive integer (same contract as the
# per-issue work-plans resolver — see _resolve_work_plans_dir.sh).
if [[ -n "$CLI_ISSUE_NUMBER" && ! "$CLI_ISSUE_NUMBER" =~ ^[1-9][0-9]*$ ]]; then
    echo "ERROR: --issue value '${CLI_ISSUE_NUMBER}' is not a positive integer" >&2
    exit 2
fi

# --- Dependency checks ---
# gh is required for PR mode (PR body/diff retrieval) but optional for
# branch mode (offline pre-push review uses local git only).
if [[ "$BRANCH_MODE" != true ]] && ! command -v gh &>/dev/null; then
    echo "WARNING: GitHub CLI (gh) not installed — required for PR metadata" >&2
    exit 1
fi

# Resolve repo slug for explicit -R targeting (prevents misrouting in
# nested repos). Prefer `gh repo view` over parsing `git remote get-url`
# — gh handles SSH host aliases (~/.ssh/config), GitHub Enterprise, and
# custom remote names correctly, where the regex approach produced
# garbage or silently fell back (issue #150). Skipped in branch mode
# unless gh is available, since branch mode never calls gh -R.
GH_REPO_SLUG=""
if [[ -n "$EXPLICIT_REPO" ]]; then
    GH_REPO_SLUG="$EXPLICIT_REPO"
elif [[ "$BRANCH_MODE" != true ]] && command -v gh &>/dev/null; then
    # PR mode only — branch mode never uses GH_REPO_ARGS so the
    # network call is wasted work.
    GH_REPO_SLUG=$(gh repo view --json nameWithOwner --jq '.nameWithOwner' 2>/dev/null || echo "")
fi
GH_REPO_ARGS=()
if [[ -n "$GH_REPO_SLUG" && "$GH_REPO_SLUG" =~ ^[^/[:space:]]+/[^/[:space:]]+$ ]]; then
    GH_REPO_ARGS=("-R" "$GH_REPO_SLUG")
fi

# --- Per-agent CLI resolution ---
# One missing CLI never aborts the others: the agent is recorded as
# unavailable with a reason, gets a failed marker once the artifact dir
# exists, and the run continues. Only "no selected agent is usable"
# is a dependency error (exit 1).
declare -A AGENT_BIN_FOR=()
declare -A AGENT_UNAVAILABLE_REASON=()
USABLE_AGENTS=0
for agent in "${AGENTS_TO_RUN[@]}"; do
    bin=$(resolve_agent_bin "$agent")
    if [[ -z "$bin" ]]; then
        AGENT_UNAVAILABLE_REASON["$agent"]="${AGENT_BINS[$agent]} CLI not found (PATH searched: ${PATH}; also ~/.nvm/versions/node/*/bin/, ~/.local/bin/, ~/.npm-global/bin/, /usr/local/bin/)"
    elif [[ "$agent" == "gemini" && ! -x "$AGY_REVIEW_HELPER" ]]; then
        AGENT_UNAVAILABLE_REASON["$agent"]="${AGY_REVIEW_HELPER} is missing or not executable"
    elif [[ "$agent" == "claude" ]] && ! command -v jq >/dev/null 2>&1; then
        # claude's result is JSON and the helper parses it with jq, so
        # without jq this agent cannot produce a validated review at all
        # — name that here rather than letting every claude run fail.
        AGENT_UNAVAILABLE_REASON["$agent"]="jq is required to parse claude's JSON result and is not installed (see bootstrap.sh)"
    elif [[ "$agent" == "codex" || "$agent" == "claude" || "$agent" == "copilot" ]] && [[ ! -x "$CLI_REVIEW_HELPER" ]]; then
        # Scoped to the three agents that helper serves, the way the
        # gemini check above is scoped: a missing _cli_review.sh must not
        # mark a gemini-only run unavailable.
        AGENT_UNAVAILABLE_REASON["$agent"]="${CLI_REVIEW_HELPER} is missing or not executable"
    else
        AGENT_BIN_FOR["$agent"]="$bin"
        USABLE_AGENTS=$((USABLE_AGENTS + 1))
    fi
done

if [[ "$USABLE_AGENTS" -eq 0 ]]; then
    for agent in "${AGENTS_TO_RUN[@]}"; do
        echo "WARNING: ${agent} adversarial review unavailable — ${AGENT_UNAVAILABLE_REASON[$agent]}" >&2
    done
    exit 1
fi

# --- Resolve issue number ---
# Order: explicit --issue flag wins; otherwise mode-specific resolution.
# PR mode: require a GitHub closure keyword in the PR body. The loose
# "first standalone #N" fallback was removed in #149 — it silently
# routed artifacts to unrelated issues and, post-#147, caused the
# work-plans resolver to abort with a confusing wrong-issue message.
# Branch mode: parse the current branch name (AGENTS.md branch-naming
# rule: feature/issue-<N> or feature/ISSUE-<N>-<desc>). On parse
# failure, hard error unless --no-progress was passed.
if [[ -n "$CLI_ISSUE_NUMBER" ]]; then
    ISSUE_NUMBER="$CLI_ISSUE_NUMBER"
elif [[ "$BRANCH_MODE" == true ]]; then
    CURRENT_BRANCH=$(git branch --show-current 2>/dev/null || echo "")
    # Tightened regex: requires a separator after digits (so
    # `feature/issue-3foo` doesn't silently capture `3`) and rejects
    # leading zeros (so `feature/issue-03` doesn't bypass the
    # --issue validator's positive-integer check). #149 lesson:
    # silent issue-number misrouting is the failure mode to prevent.
    if [[ "$CURRENT_BRANCH" =~ ^feature/[Ii][Ss][Ss][Uu][Ee]-([1-9][0-9]*)(-|$) ]]; then
        ISSUE_NUMBER="${BASH_REMATCH[1]}"
    elif [[ "$NO_PROGRESS" == true ]]; then
        # Sentinel used for the findings filename; the per-issue
        # artifact dir is replaced with a tmp dir below.
        ISSUE_NUMBER="noprogress"
    else
        {
            echo "ERROR: cannot resolve issue number."
            echo ""
            echo "  Current branch '${CURRENT_BRANCH}' does not match feature/issue-<N>."
            echo ""
            echo "  Fix one of:"
            echo "    --issue <N>     point to a specific issue"
            echo "    --no-progress   skip progress.md persistence"
            echo "                    (skill worktrees, one-off branches)"
            echo "    rename branch   if this should be tracked"
        } >&2
        exit 2
    fi
else
    # PR mode: parse Closes/Fixes/Resolves keyword from PR body.
    # Capture gh's exit status distinctly from "retrieved an empty body"
    # so auth/network/permission failures produce the right remediation
    # (per review feedback on #149).
    if ! PR_BODY=$(gh pr view "$PR_NUMBER" "${GH_REPO_ARGS[@]}" --json body --jq '.body' 2>/dev/null); then
        {
            echo "ERROR: Failed to retrieve body for PR #${PR_NUMBER}."
            echo ""
            echo "  Verify GitHub authentication, repository permissions,"
            echo "  and network connectivity, then try again."
            echo "  Alternatively, pass --issue <N> to skip PR-body extraction."
        } >&2
        exit 2
    fi

    # Match GitHub closure keywords (case-insensitive): Closes #N,
    # Fixes #N, Resolves #N. Requires a word boundary before the keyword
    # to avoid matching "encloses", "prefixes", etc. Also accepts the
    # cross-repo form "Closes owner/repo#N" (just extracts N).
    ISSUE_REF=$(printf '%s\n' "$PR_BODY" \
        | grep -ioE '(^|[^[:alnum:]_])(closes|fixes|resolves)[[:space:]]+([a-zA-Z0-9._-]+/[a-zA-Z0-9._-]+)?#[0-9]+' \
        | head -n1 || true)
    ISSUE_NUMBER=$(printf '%s\n' "$ISSUE_REF" | grep -oE '[0-9]+$' || true)

    if [[ -z "$ISSUE_NUMBER" ]]; then
        {
            echo "ERROR: PR #${PR_NUMBER} body has no 'Closes|Fixes|Resolves #N' keyword."
            echo ""
            echo "  The loose '#N' fallback was removed in #149 because it routed"
            echo "  artifacts to unrelated issues. Two ways to proceed:"
            echo "    1. Pass --issue <N> to set the issue number explicitly, or"
            echo "    2. Edit the PR body to include a closure keyword"
            echo "       (e.g. 'Closes #123')."
        } >&2
        exit 2
    fi
fi

# --- Set up artifact directory ---
# Refuse to run outside the matching worktree (issue #147) unless the
# caller explicitly overrides the location via --work-plans-dir (exact
# path), --work-dir (repo root), or --no-progress (mktemp -d for
# ephemeral artifacts when there's no issue to track). Each override
# routes through $WORK_PLANS_DIR_OVERRIDE so the resolver treats them
# uniformly.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_resolve_work_plans_dir.sh
source "${SCRIPT_DIR}/_resolve_work_plans_dir.sh"
# shellcheck source=_resolve_default_branch.sh
source "${SCRIPT_DIR}/_resolve_default_branch.sh"


if [[ -n "$CLI_WORK_PLANS_DIR" ]]; then
    export WORK_PLANS_DIR_OVERRIDE="$CLI_WORK_PLANS_DIR"
elif [[ -n "$EXPLICIT_WORK_DIR" ]]; then
    if [[ ! -d "$EXPLICIT_WORK_DIR" ]]; then
        echo "ERROR: --work-dir is not an existing directory: ${EXPLICIT_WORK_DIR}" >&2
        exit 2
    fi
    # Resolve to absolute path so the findings paths printed are usable
    # from any cwd.
    EXPLICIT_WORK_DIR=$(cd "$EXPLICIT_WORK_DIR" && pwd)
    export WORK_PLANS_DIR_OVERRIDE="${EXPLICIT_WORK_DIR}/.agent/work-plans/issue-${ISSUE_NUMBER}"
elif [[ "$NO_PROGRESS" == true ]]; then
    # --no-progress: ephemeral artifact dir. Findings remain readable
    # for the session but aren't tied to a per-issue directory and won't
    # be picked up by progress.md commit conventions. Note that the
    # branch-name regex may still resolve a real ISSUE_NUMBER (e.g.
    # `feature/issue-3` with --no-progress); the artifact dir override
    # always wins when --no-progress is set, even if the issue number
    # was resolvable. Tmpdir is left in /tmp for the session; the OS
    # cleans it up on next boot. Adding a `trap` to remove on exit was
    # considered and declined: users frequently want to inspect
    # findings after the script finishes.
    NO_PROGRESS_TMP_DIR=$(mktemp -d -t "cross-model-review.XXXXXX")
    echo "INFO: --no-progress: artifacts going to ${NO_PROGRESS_TMP_DIR}" >&2
    export WORK_PLANS_DIR_OVERRIDE="${NO_PROGRESS_TMP_DIR}"
fi

WORK_PLANS_DIR=$(resolve_work_plans_dir "$ISSUE_NUMBER") || exit 4
mkdir -p "$WORK_PLANS_DIR"

prompt_file_for()   { echo "${WORK_PLANS_DIR}/review-$1-prompt.md"; }
findings_file_for() { echo "${WORK_PLANS_DIR}/review-$1-findings.md"; }

# Write one error marker into every selected agent's findings file
# (truncating) and exit 3 — the shared prompt could not be built, so no
# agent ran and no AGENT= triplet is printed.
abort_all_agents() {
    local marker="$1"
    local agent
    for agent in "${AGENTS_TO_RUN[@]}"; do
        echo "$marker" > "$(findings_file_for "$agent")"
    done
    exit 3
}

# --- Get review-target metadata ---
# PR mode: query gh for the PR's title and URL.
# Branch mode: resolve base ref via the helper, capture HEAD sha and
# branch name for the prompt header.
if [[ "$BRANCH_MODE" == true ]]; then
    BRANCH_NAME=$(git branch --show-current 2>/dev/null || echo "(detached)")
    HEAD_SHA=$(git rev-parse --short HEAD 2>/dev/null || echo "unknown")

    if [[ -n "$BRANCH_BASE" ]]; then
        # Validate and normalize: prefer the local ref, fall back to
        # origin/<ref> when only the remote-tracking branch exists.
        # Without normalization, `git diff develop...HEAD` would fail
        # on a fresh clone where only `origin/develop` is reachable.
        if git rev-parse --verify --quiet "$BRANCH_BASE" >/dev/null 2>&1; then
            BASE_REF="$BRANCH_BASE"
        elif git rev-parse --verify --quiet "origin/$BRANCH_BASE" >/dev/null 2>&1; then
            BASE_REF="origin/$BRANCH_BASE"
        else
            echo "ERROR: --branch base '${BRANCH_BASE}' is not a known ref (locally or as origin/${BRANCH_BASE})" >&2
            exit 2
        fi
    else
        BASE_REF=$(resolve_default_branch) || exit 2
    fi

    PR_TITLE="Local branch ${BRANCH_NAME} at ${HEAD_SHA}"
    PR_URL=""
else
    PR_TITLE=$(gh pr view "$PR_NUMBER" "${GH_REPO_ARGS[@]}" --json title --jq '.title' 2>/dev/null || echo "PR #${PR_NUMBER}")
    PR_URL=$(gh pr view "$PR_NUMBER" "${GH_REPO_ARGS[@]}" --json url --jq '.url' 2>/dev/null || echo "")
fi

# Human-readable label for status messages. Mode-aware so branch-mode
# logs read sensibly without "PR #" prefixes that don't apply. Computed
# after the metadata block so BRANCH_NAME is in scope under `set -u`.
if [[ "$BRANCH_MODE" == true ]]; then
    if [[ "$ISSUE_NUMBER" == "noprogress" ]]; then
        TARGET_LABEL="branch ${BRANCH_NAME} (no-progress mode)"
    else
        TARGET_LABEL="branch ${BRANCH_NAME} (issue #${ISSUE_NUMBER})"
    fi
else
    TARGET_LABEL="PR #${PR_NUMBER} (issue #${ISSUE_NUMBER})"
fi

# Temp paths owned by cleanup_jobs. Declared empty here, and only filled
# in AFTER the EXIT/INT/TERM/HUP traps below are registered: a failure or
# signal between a `mktemp` and the trap would otherwise leak the file.
# cleanup_jobs skips whichever of them is still empty.
SHARED_PROMPT=""
SHARED_DIFF=""
AGENT_TMP_ROOT=""

# Cleanup on every exit path: stop any agent job still running (each
# job's own TERM trap forwards to its CLI), then drop the temp prompt.
# The INT/TERM traps turn a signal into an exit so the EXIT trap fires
# — otherwise an interrupted run would leave the CLIs running for up to
# AGENT_TIMEOUT, burning quota on an abandoned review.
declare -A AGENT_PID=()

# Is this job over? A dead-but-unreaped child still answers `kill -0`,
# so that alone would report every finished job as alive and burn the
# whole reap budget. Two liveness reads, in order of availability:
#   * bash's own job table — `jobs -pr` lists only jobs still RUNNING, so
#     a job absent from it has exited whether or not it was reaped. No
#     /proc, works on macOS/BSD and in stripped containers.
#   * /proc's process state (Z = exited, not yet reaped) as a
#     cross-check where /proc exists.
#
# proc_state prints the state letter from /proc/<pid>/stat. Field 2 is
# the parenthesised comm, which may itself contain spaces (or a `)`), so
# a fixed whitespace field such as awk's $3 lands on the wrong word for
# a comm like "a Z b". The state is the first word after the LAST `)`.
proc_state() {
    local stat
    [[ -r "/proc/$1/stat" ]] || return 1
    IFS= read -r stat < "/proc/$1/stat" 2>/dev/null || return 1
    stat=${stat##*) }
    printf '%s' "${stat%% *}"
}

job_finished() {
    local jpid="$1" state running
    kill -0 "$jpid" 2>/dev/null || return 0
    running=$(jobs -pr 2>/dev/null || true)
    if [[ -n "$running" ]]; then
        grep -qx -- "$jpid" <<< "$running" || return 0
    elif state=$(proc_state "$jpid"); then
        [[ "$state" == "Z" ]] && return 0
    fi
    return 1
}

cleanup_jobs() {
    local pid waited=0 finished
    for pid in "${AGENT_PID[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    # Reap BEFORE removing AGENT_TMP_ROOT (#313 round 2). Each helper may
    # legitimately still be waiting out its own escalation window for a
    # CLI that ignored SIGTERM, and that CLI is still writing into a temp
    # dir under this root: removing it here would pull the ground out
    # from under a live process.
    for pid in "${AGENT_PID[@]}"; do
        finished=false
        while (( waited < CLEANUP_REAP_SECONDS * 10 )); do
            if job_finished "$pid"; then
                finished=true
                break
            fi
            sleep 0.1
            waited=$((waited + 1))
        done
        if job_finished "$pid"; then
            finished=true
        fi
        if [[ "$finished" == true ]]; then
            # Only now: `wait` on a job that is still running would block
            # past the bound and hang the exit path forever (#313 round 3).
            wait "$pid" 2>/dev/null || true
        else
            # Budget spent and the job is still alive. SIGKILL it and do
            # NOT wait: cleaning up beats blocking, and the job's CLI was
            # already signalled twice over by this point.
            echo "WARNING: agent job ${pid} did not finish within CLEANUP_REAP_TIMEOUT=${CLEANUP_REAP_TIMEOUT}s; killing it and removing the shared temp root anyway" >&2
            kill -9 "$pid" 2>/dev/null || true
        fi
    done
    # Any of these may still be empty: an early exit or signal can land
    # before (or between) the mktemp calls below, and `rm ""` is an error.
    [[ -z "$SHARED_PROMPT" ]] || rm -f "$SHARED_PROMPT"
    [[ -z "$SHARED_DIFF" ]] || rm -f "$SHARED_DIFF"
    [[ -z "$AGENT_TMP_ROOT" ]] || rm -rf "$AGENT_TMP_ROOT"
}
trap cleanup_jobs EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

# --- Write the shared prompt ---
# The header, metadata, diff and output-format footer are identical for
# every agent, so they are built once into a temp file and copied into
# each agent's prompt file; only the per-agent tool-use footer differs.
# Use a quoted heredoc for the static header to prevent shell expansion,
# then stream the diff from gh/git into a staging file (never a variable,
# which could hit shell limits for large Deep-tier PRs) before fencing it
# into the prompt.
SHARED_PROMPT=$(mktemp -t "cross-model-review-prompt.XXXXXX")
# The diff is staged here before it is fenced into the prompt: its outer
# fence length depends on the longest backtick run in it (outer_fence_for).
SHARED_DIFF=$(mktemp -t "cross-model-review-diff.XXXXXX")

# Scratch root handed to the agent jobs as their TMPDIR. _agy_review.sh
# makes its own `mktemp -d` under it and removes it on every exit path it
# can trap — but `timeout -k` finishes a wedged helper with SIGKILL, which
# no trap survives. Owning the parent directory here means that one
# untrappable path still gets cleaned up, by this script's EXIT trap.
AGENT_TMP_ROOT=$(mktemp -d -t "cross-model-review-tmp.XXXXXX")

cat > "$SHARED_PROMPT" << 'PROMPT_HEADER'
# Adversarial Code Review

## Your Role

You are an independent adversarial reviewer. Your job is to find issues that
other reviewers missed: edge cases, security implications, incorrect
assumptions, subtle bugs, and logic errors.

Review the diff below with fresh eyes. Do not assume previous reviewers caught
everything. Focus on:

- **Edge cases**: What inputs or states could break this code?
- **Security**: Are there injection, auth, or data exposure risks?
- **Assumptions**: What does the code assume that might not hold?
- **Subtle bugs**: Off-by-one, race conditions, resource leaks, null/undefined
- **Logic errors**: Does the code actually do what the PR title claims?

## PR Under Review

PROMPT_HEADER

# Append review-target metadata (needs expansion).
# Branch-mode prompt emits Branch + Base + HEAD instead of PR Number/URL
# so the agent reviewing local pre-push work has the right framing.
if [[ "$BRANCH_MODE" == true ]]; then
    printf '**Title**: %s\n**Branch**: %s\n**Base**: %s\n**HEAD**: %s\n\n' \
        "$PR_TITLE" "$BRANCH_NAME" "$BASE_REF" "$HEAD_SHA" >> "$SHARED_PROMPT"
else
    printf '**Title**: %s\n**URL**: %s\n**PR Number**: #%s\n\n' \
        "$PR_TITLE" "$PR_URL" "$PR_NUMBER" >> "$SHARED_PROMPT"
fi

# Drop .agent/work-plans/** file sections from a unified diff (#312).
# Reads the diff on stdin. A section starts at `diff --git a/<p> b/<p>`
# and runs to the next such header. Only the b/ (post-image) path
# decides: a deleted file still carries its b/ path on that line, and a
# file renamed OUT of work-plans into the codebase is new code that
# must stay in review. Git quotes paths with unusual characters
# (`"b/..."`), hence the optional quote. Everything else passes through
# byte-for-byte.
filter_work_plans_diff() {
    awk '
        /^diff --git / {
            skip = ($0 ~ / "?b\/\.agent\/work-plans\//)
        }
        !skip { print }
    '
}

# Print the backtick fence that safely wraps the text on stdin: one
# backtick longer than the longest backtick run anywhere in it, minimum 3
# (#320). Under CommonMark nothing inside a fence that long can close it,
# and its own closer — printed at column 0, with no trailing CR — always
# does, whatever the content holds: fences with info strings or longer
# markers, fences in list items, indented code, tildes, CRLF line endings,
# or a fence cut in half. Used for every block of embedded content in the
# prompt (the diff, the plan context); never write a fixed ``` around
# content this script does not control.
outer_fence_for() {
    awk '
        {
            line = $0
            while (match(line, /`+/)) {
                if (RLENGTH > max) max = RLENGTH
                line = substr(line, RSTART + RLENGTH)
            }
        }
        END {
            n = (max >= 3) ? max + 1 : 3
            s = ""
            for (i = 0; i < n; i++) s = s "`"
            print s
        }
    '
}

# Stage the diff through the work-plans filter, then fence it into the
# shared prompt. Branch mode uses local `git diff <base>...HEAD`; PR mode
# uses `gh pr diff <N>`. The pipeline sits inside `if !` so `set -e` does
# not abort the script before the error branch runs; with `pipefail` the
# pipeline's status is the last non-zero stage's (bash's rule), so any
# failing stage still makes the whole pipeline fail — a failed gh/git call
# is not masked by the filter succeeding on empty input, and a filter
# dying mid-stream cannot leave a truncated diff looking complete. The
# diff goes to a file rather than a variable (shell limits on large
# Deep-tier PRs) and is read once more for its fence length.
if [[ "$BRANCH_MODE" == true ]]; then
    # Explicit a/ b/ prefixes so a diff.noprefix / diff.mnemonicPrefix
    # config cannot defeat the work-plans filter.
    if ! git diff --src-prefix=a/ --dst-prefix=b/ "${BASE_REF}...HEAD" 2>/dev/null | filter_work_plans_diff > "$SHARED_DIFF"; then
        echo "ERROR: Could not produce diff for ${BRANCH_NAME} against ${BASE_REF}" >&2
        abort_all_agents '--- Review error: failed to produce branch diff ---'
    fi
else
    if ! gh pr diff "$PR_NUMBER" "${GH_REPO_ARGS[@]}" 2>/dev/null | filter_work_plans_diff > "$SHARED_DIFF"; then
        echo "ERROR: Could not retrieve diff for PR #${PR_NUMBER}" >&2
        abort_all_agents '--- Review error: failed to retrieve diff ---'
    fi
fi

# Guard: if diff is empty (before or after the work-plans filter), abort
# with a clear error instead of launching agents with no content to
# review.
if [[ "$(wc -l < "$SHARED_DIFF")" -eq 0 ]]; then
    if [[ "$BRANCH_MODE" == true ]]; then
        echo "ERROR: branch '${BRANCH_NAME}' has no reviewable changes against '${BASE_REF}' — nothing to review" >&2
        echo "  Either the branch is up-to-date with the base, the base ref is wrong," >&2
        echo "  or every changed file is under .agent/work-plans/ (excluded from review, #312)." >&2
        abort_all_agents '--- Review error: diff was empty (branch matches base, or only .agent/work-plans/ changed) ---'
    else
        echo "ERROR: PR #${PR_NUMBER} diff is empty — nothing to review" >&2
        echo "  This usually means the PR was not found in the target repo, or every" >&2
        echo "  changed file is under .agent/work-plans/ (excluded from review, #312)." >&2
        echo "  Try passing --repo <owner/repo> explicitly." >&2
        abort_all_agents '--- Review error: diff was empty (PR not found, no changes, or only .agent/work-plans/ changed) ---'
    fi
fi
DIFF_FENCE=$(outer_fence_for < "$SHARED_DIFF")
{
    printf '## Diff\n\n%sdiff\n' "$DIFF_FENCE"
    cat "$SHARED_DIFF"
    # A diff whose last line lacks a newline ("\ No newline at end of
    # file" covers the file contents, not the stream) must not glue the
    # closer onto it.
    [[ -z "$(tail -c 1 "$SHARED_DIFF")" ]] || printf '\n'
    printf '%s\n\n' "$DIFF_FENCE"
} >> "$SHARED_PROMPT"
rm -f "$SHARED_DIFF"

# --- Plan context (#320) ---
# The diff still excludes `.agent/work-plans/**` (#312), but a reviewer
# that never sees the plan cannot flag divergence from it. The plan's
# `## Approach` section — and only that section — is re-admitted here as
# labelled context outside the diff fence, capped so it cannot reintroduce
# the prompt bloat #312 removed. Built once, into the shared prompt, so
# every agent copy carries it.
#
# The section is omitted entirely — never a fallback to the whole plan,
# never an empty heading — when the plan file is absent, carries no
# `## Approach`, or that section is blank. Also omitted under
# --no-progress: that mode has no issue bookkeeping to draw on (its
# artifact dir is a fresh mktemp -d, so a plan.md can only exist in the
# --no-progress + --work-plans-dir combination, where this guard is what
# keeps the mode's promise).
PLAN_CONTEXT_MAX_LINES=200
PLAN_CONTEXT_FILE="${WORK_PLANS_DIR}/plan.md"
if [[ "$NO_PROGRESS" != true && -f "$PLAN_CONTEXT_FILE" ]]; then
    # The section is located by _plan_approach.py, a CommonMark parser
    # (markdown-it-py) rather than a line scanner: it starts after the first
    # top-level `## Approach` heading (never one inside a fenced example)
    # and ends at the next H1/H2 heading — ATX, indented ATX or setext — or
    # thematic break; a fence that never closes is cut at the first
    # boundary after its opener instead of running to EOF. The script's
    # docstring has the rules and the exit codes.
    #
    # Interpreter: the workspace .venv's python3 first (that is where
    # requirements.txt installs markdown-it-py, ADR-0009), then python3 on
    # PATH. The venv belongs to the main checkout, never a worktree (#272),
    # so it is found through git's common dir, as the Makefile does. An
    # interpreter that lacks the library (exit 4) passes to the next one.
    # Any other code, including python's own 2 for a missing or unreadable
    # _plan_approach.py, is an extractor error.
    # If none can import it, or the extractor fails, the review still runs:
    # one warning, and the Plan Context block is omitted as if there were
    # no plan.
    PLAN_APPROACH=""
    PLAN_APPROACH_RC=4
    PLAN_APPROACH_ERR="${AGENT_TMP_ROOT}/plan-approach.err"
    PLAN_APPROACH_PYTHONS=()
    PLAN_APPROACH_COMMON=$(git -C "$SCRIPT_SELF_DIR" rev-parse --path-format=absolute --git-common-dir 2>/dev/null || true)
    if [[ -n "$PLAN_APPROACH_COMMON" && -x "${PLAN_APPROACH_COMMON%/.git}/.venv/bin/python3" ]]; then
        PLAN_APPROACH_PYTHONS+=("${PLAN_APPROACH_COMMON%/.git}/.venv/bin/python3")
    fi
    if command -v python3 > /dev/null 2>&1; then
        PLAN_APPROACH_PYTHONS+=("$(command -v python3)")
    fi
    for plan_python in "${PLAN_APPROACH_PYTHONS[@]}"; do
        PLAN_APPROACH_RC=0
        PLAN_APPROACH=$("$plan_python" "${SCRIPT_SELF_DIR}/_plan_approach.py" \
            "$PLAN_CONTEXT_FILE" 2> "$PLAN_APPROACH_ERR") || PLAN_APPROACH_RC=$?
        (( PLAN_APPROACH_RC == 4 )) || break
    done
    case "$PLAN_APPROACH_RC" in
        0) ;;
        1) PLAN_APPROACH="" ;;
        4)
            PLAN_APPROACH=""
            if (( ${#PLAN_APPROACH_PYTHONS[@]} == 0 )); then
                echo "WARNING: plan context omitted: no python3 found to run _plan_approach.py" >&2
            else
                echo "WARNING: plan context omitted: markdown-it-py is not importable by ${PLAN_APPROACH_PYTHONS[*]} (run 'make setup' to install requirements.txt into the workspace .venv)" >&2
            fi
            ;;
        *)
            PLAN_APPROACH=""
            echo "WARNING: plan context omitted: _plan_approach.py failed (exit ${PLAN_APPROACH_RC}): $(head -n 1 "$PLAN_APPROACH_ERR" 2>/dev/null)" >&2
            ;;
    esac
    rm -f "$PLAN_APPROACH_ERR"

    # Whitespace-only counts as empty.
    if [[ -n "${PLAN_APPROACH//[[:space:]]/}" ]]; then
        # Here-strings, never `printf ... | head`: under `set -o pipefail`
        # head exits as soon as it has its lines, printf takes SIGPIPE on
        # an Approach larger than the pipe buffer, and the whole script
        # dies with 141 before a single agent is dispatched.
        PLAN_APPROACH_LINES=$(wc -l <<< "$PLAN_APPROACH")
        if (( PLAN_APPROACH_LINES > PLAN_CONTEXT_MAX_LINES )); then
            PLAN_CONTEXT_BODY=$(head -n "$PLAN_CONTEXT_MAX_LINES" <<< "$PLAN_APPROACH")
            PLAN_CONTEXT_TRUNCATED=$(( PLAN_APPROACH_LINES - PLAN_CONTEXT_MAX_LINES ))
        else
            PLAN_CONTEXT_BODY="$PLAN_APPROACH"
            PLAN_CONTEXT_TRUNCATED=0
        fi

        # Wrap the excerpt in ONE outer fence (outer_fence_for): nothing in
        # the plan can close it early and its closer always ends it, so the
        # footer below can never be swallowed. The heading and the framing
        # text stay outside the fence; the truncation marker follows the
        # closer.
        PLAN_CONTEXT_FENCE=$(outer_fence_for <<< "$PLAN_CONTEXT_BODY")

        {
            printf '## Plan Context\n\n'
            printf 'Below is the `## Approach` section of the plan this change is meant\n'
            printf 'to implement. It is context, not the subject of the review: flag\n'
            printf 'divergences between the diff and this plan, but do not review the\n'
            printf 'plan itself.\n\n'
            printf '%smarkdown\n' "$PLAN_CONTEXT_FENCE"
            printf '%s\n' "$PLAN_CONTEXT_BODY"
            printf '%s\n' "$PLAN_CONTEXT_FENCE"
            if (( PLAN_CONTEXT_TRUNCATED > 0 )); then
                printf '\n_[truncated: %d more lines]_\n' "$PLAN_CONTEXT_TRUNCATED"
            fi
            printf '\n'
        } >> "$SHARED_PROMPT"
    fi
fi

# Append output format instructions (quoted heredoc, no expansion)
cat >> "$SHARED_PROMPT" << 'PROMPT_FOOTER'
## Output Format

Write your findings to this exact format so they can be parsed:

### Findings

| # | Severity | File | Line | Finding |
|---|----------|------|------|---------|
| 1 | must-fix / suggestion | `path/to/file` | line number | Description of the issue |

If you find no issues, write:

### Findings

No issues found.

### Summary

Write a 1-3 sentence overall assessment after the findings table.
PROMPT_FOOTER

# --- Per-agent prompt files ---
# Gemini only: headless agy auto-denies tool calls it cannot prompt for
# (shell commands, and on real branches file reads too) and then returns
# an empty response (#288, #336). So the reviewer is told to call no tools
# at all and to work from the embedded diff and Plan Context alone, and to
# keep its answer short: large Deep-tier prompts otherwise draw a reply
# that hits agy's output-token limit (#336). Not added for
# codex/claude/copilot — codex reads files through the shell, so the
# restriction would cost it context.
for agent in "${AGENTS_TO_RUN[@]}"; do
    prompt_file=$(prompt_file_for "$agent")
    cp "$SHARED_PROMPT" "$prompt_file"
    if [[ "$agent" == "gemini" ]]; then
        cat >> "$prompt_file" << 'PROMPT_TOOL_USE'

## Tool Use

The diff above is the complete set of code changes under review; files
under `.agent/work-plans/` (plan and progress bookkeeping) are deliberately
excluded **from the diff**. Where a `## Plan Context` section appears above,
it is the plan's Approach quoted as context only — not part of the change
under review. The diff and the Plan Context are the only material available
to you: there is no file-reading tool in this session. Do not call any
tools — file reads and shell commands are both denied in this headless
session, and a denied call ends the review with no output.
Do NOT run shell commands. If you need context the diff does not show, say
so in a `suggestion`-severity finding or in the `### Summary`; never invent
that context, and never write as if you had read a file.

Keep the answer concise: the findings table and a short summary only.
Report every finding you have; keep each row short. Do not restate the
diff, and do not quote large spans of it back; cite file:line instead.
PROMPT_TOOL_USE
    fi
done

# --- Run reviews: one background job per agent, all in parallel ---
# Each job runs its agent, then appends that agent's completion marker
# itself, so a slow agent never delays a fast agent's marker and the
# parent only has to collect exit statuses. Jobs write nothing to stdout,
# keeping the machine-parseable block contiguous.
run_agent_job() {
    local agent="$1"
    local prompt_file findings_file rc child
    prompt_file=$(prompt_file_for "$agent")
    findings_file=$(findings_file_for "$agent")

    if [[ -n "${AGENT_UNAVAILABLE_REASON[$agent]+x}" ]]; then
        {
            echo "${agent} adversarial review unavailable."
            echo ""
            echo "Reason: ${AGENT_UNAVAILABLE_REASON[$agent]}"
            echo '--- Review failed ---'
        } > "$findings_file"
        return 1
    fi

    # The CLI runs as this job's child (run_agent_sync execs into it) so
    # a TERM from the parent's cleanup reaches the CLI, not just this
    # shell. `wait` is interruptible by the trap; a plain foreground
    # command would defer it until the CLI finished on its own.
    # Trap armed BEFORE the spawn so a TERM landing in the launch window
    # cannot leave the CLI running behind a dead job shell.
    # TERM only: bash makes background children of a non-interactive shell
    # ignore SIGINT, and an ignored signal cannot be trapped, so an INT
    # trap here would never run. The parent's INT trap turns Ctrl-C into
    # an exit, and its EXIT cleanup TERMs these jobs — that is the live
    # path for an interrupt.
    # The trap waits for the child before exiting (#313 round 2): the
    # helper under `timeout` legitimately spends its own escalation
    # window killing a CLI that ignored SIGTERM, and it is still writing
    # into AGENT_TMP_ROOT while it does. Exiting here at once would tell
    # the parent's cleanup that this job is finished, and the temp root
    # would be removed under a live CLI. The parent's reap is bounded, so
    # a helper that never returns still cannot hang the exit path.
    child=""
    trap 'if [[ -n "$child" ]]; then kill "$child" 2>/dev/null; wait "$child" 2>/dev/null; fi; exit 143' TERM
    run_agent_sync "$agent" "${AGENT_BIN_FOR[$agent]}" "$prompt_file" "$findings_file" &
    child=$!
    rc=0
    wait "$child" || rc=$?
    if [[ "$rc" -eq 124 ]]; then
        if [[ "$agent" == "gemini" ]]; then
            # The backstop firing means the helper never reported its own
            # print-timeout — say which bound cut the run so the reader
            # doesn't look for a reason the helper never wrote.
            printf '\n%s review hit the outer backstop (GEMINI_BACKSTOP=%ss, above AGY_PRINT_TIMEOUT=%s): the helper never returned, so its own timeout handling did not run.\n' \
                "$agent" "$GEMINI_BACKSTOP" "$AGY_PRINT_TIMEOUT" >> "$findings_file"
        else
            # No partial output to point at: _cli_review.sh truncates the
            # findings file first and only writes once the turn has
            # produced a result, so a killed run leaves it empty.
            printf '\n%s review timed out (AGENT_TIMEOUT=%s) and the CLI was killed; no result was produced.\n' \
                "$agent" "$AGENT_TIMEOUT" >> "$findings_file"
        fi
    fi
    if [[ "$rc" -eq 0 ]]; then
        echo '--- Review complete ---' >> "$findings_file"
    else
        echo '--- Review failed ---' >> "$findings_file"
    fi
    return "$rc"
}

if [[ "$MULTI_AGENT" == true ]]; then
    echo "MODE=parallel-sync"
    echo ""
    echo "Running ${#AGENTS_TO_RUN[@]} adversarial review(s) in parallel for ${TARGET_LABEL}..."
    for agent in "${AGENTS_TO_RUN[@]}"; do
        echo "  ${agent}: $(findings_file_for "$agent")"
    done
    echo "  Live: tail -f <findings-file>; per-agent AGENT=/FINDINGS_FILE=/EXIT= lines follow when all have finished."
else
    echo "MODE=sync"
    echo "AGENT=${AGENTS_TO_RUN[0]}"
    echo "FINDINGS_FILE=$(findings_file_for "${AGENTS_TO_RUN[0]}")"
    echo ""
    echo "Running ${AGENTS_TO_RUN[0]} adversarial review synchronously for ${TARGET_LABEL}..."
    echo "  Prompt:  $(prompt_file_for "${AGENTS_TO_RUN[0]}")"
    echo "  Results: $(findings_file_for "${AGENTS_TO_RUN[0]}")"
fi

for agent in "${AGENTS_TO_RUN[@]}"; do
    run_agent_job "$agent" &
    AGENT_PID["$agent"]=$!
done

# Collect in selection order. `wait` returns the job's own status; the
# `|| rc=$?` keeps set -e from aborting the loop on the first failed
# agent before the remaining ones are collected.
declare -A AGENT_EXIT=()
ANY_FAILED=false
for agent in "${AGENTS_TO_RUN[@]}"; do
    rc=0
    wait "${AGENT_PID[$agent]}" || rc=$?
    # Reaped: drop the PID so the exit cleanup cannot signal a recycled
    # PID belonging to an unrelated process.
    unset "AGENT_PID[$agent]"
    AGENT_EXIT["$agent"]=$rc
    [[ "$rc" -eq 0 ]] || ANY_FAILED=true
done

if [[ "$MULTI_AGENT" == true ]]; then
    for agent in "${AGENTS_TO_RUN[@]}"; do
        echo "AGENT=${agent}"
        echo "FINDINGS_FILE=$(findings_file_for "$agent")"
        echo "EXIT=${AGENT_EXIT[$agent]}"
    done
    echo ""
    for agent in "${AGENTS_TO_RUN[@]}"; do
        if [[ "${AGENT_EXIT[$agent]}" -eq 0 ]]; then
            echo "  ${agent}: complete — $(findings_file_for "$agent")"
        else
            echo "  ${agent}: FAILED (exit ${AGENT_EXIT[$agent]}) — $(findings_file_for "$agent")"
        fi
    done
    if [[ "$ANY_FAILED" == true ]]; then
        echo "ERROR: at least one agent failed — see EXIT= per agent above" >&2
        exit 3
    fi
else
    if [[ "$ANY_FAILED" == true ]]; then
        echo "ERROR: ${AGENTS_TO_RUN[0]} CLI exited with an error" >&2
        exit 3
    fi
    echo ""
    echo "Review complete. Results: $(findings_file_for "${AGENTS_TO_RUN[0]}")"
fi
