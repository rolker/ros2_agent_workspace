#!/usr/bin/env bash
# Tests for cross_model_review.sh
#
# Tests argument parsing, issue extraction, artifact path resolution, and
# empty diff guard. Uses mock gh/agent binaries to avoid real API calls.
#
# Run: bash .agent/scripts/tests/test_cross_model_review.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_UNDER_TEST="${SCRIPT_DIR}/../cross_model_review.sh"

PASS=0
FAIL=0
TMPDIR_BASE=""

# One sandbox for the whole run, created at top level (not inside $()) so
# the trap actually fires — see issue #297. No hardcoded /tmp template:
# `mktemp -d` honors TMPDIR, so the run stays inside whatever temp root the
# caller set. Each test's setup/teardown still carves and drops its own
# TMPDIR_BASE under it; the trap is the backstop for an abort.
SANDBOX="$(mktemp -d)"
trap 'rm -rf "$SANDBOX"' EXIT

# The codex arm runs under an environment allowlist (#660), which would
# strip the mocks' MOCK_* controls. Pass exactly those through the knob
# the helper provides for this, rather than widening the allowlist.
CROSS_MODEL_ENV_PASSTHROUGH="MOCK_ARGV_DIR MOCK_STDIN_DIR MOCK_TIMES_DIR MOCK_PREAMBLE MOCK_CODEX_ECHO_FULL MOCK_CODEX_EMPTY MOCK_CODEX_ERRMARK MOCK_CODEX_EXIT MOCK_CODEX_IGNORE_TERM MOCK_CODEX_ORPHAN_PIDFILE MOCK_CODEX_SLEEP MOCK_CODEX_STDERR MOCK_CODEX_ENV_DUMP"
export CROSS_MODEL_ENV_PASSTHROUGH

setup() {
    TMPDIR_BASE=$(mktemp -d -p "$SANDBOX")

    # Create a mock git repo so git rev-parse works
    MOCK_REPO="${TMPDIR_BASE}/repo"
    mkdir -p "${MOCK_REPO}"
    git -C "${MOCK_REPO}" init -q
    git -C "${MOCK_REPO}" -c user.name="Test" -c user.email="test@test" commit --allow-empty -m "init" -q

    # Create mock bin directory
    MOCK_BIN="${TMPDIR_BASE}/bin"
    mkdir -p "${MOCK_BIN}"

    # Mock agy CLI (the gemini agent's binary post-#223), implementing the
    # stream-json contract _agy_review.sh drives (#274, #288):
    #   * argv is recorded to MOCK_AGY_LOG when set (one arg per line);
    #   * the prompt arrives on stdin as one NDJSON line
    #     {"event":"user","message":{"role":"user","content":...}};
    #   * stdout is an NDJSON event stream ending in a `result` event
    #     whose `response` echoes the prompt content back.
    # Knobs (env):
    #   MOCK_AGY_DENY=1     empty response + denied_actions, exit 0 (#288)
    #   MOCK_AGY_DENY_ACTION=<n>  display_name of that denied action
    #                       (default RunCommand; #336 saw ViewFile)
    #   MOCK_AGY_TIMEOUT=1  SUCCESS result + the print-timeout stderr marker
    #   MOCK_AGY_EXIT=<n>   exit <n> after printing "boom" on stderr
    #   MOCK_AGY_ERROR=<m>  status ERROR with an object-valued `error`
    #                       whose message is <m>, exit 0 (API failure)
    #   MOCK_AGY_ERROR_RESPONSE=<r>  with MOCK_AGY_ERROR: the partial
    #                       `response` text sent alongside it (default "")
    #   MOCK_AGY_ERROR_STDERR=<l>    with MOCK_AGY_ERROR: also print <l>
    #                       on stderr before the result event
    #   MOCK_AGY_ERROR_STRING=1      with MOCK_AGY_ERROR: send `error` as
    #                       the plain string <m> (the live cutoff shape)
    #                       instead of the {code,message} object
    #   MOCK_AGY_DENY_PARTIAL=1  normal response PLUS one denied action
    #   MOCK_AGY_SLEEP=<s>  sleep <s> before answering normally
    #   MOCK_AGY_STALL=1    read the prompt, then never answer (sleep 60):
    #                       agy wedged past its own --print-timeout, which
    #                       only the caller's outer backstop can cut off
    #   MOCK_TIMES_DIR=<d>  write agy.pid at start, agy.end on completion
    # Every run also prints a non-JSON banner line on stdout first, as a
    # real CLI may (update notice), so the parser must skip it.
    cat > "${MOCK_BIN}/agy" << 'MOCK_EOF'
#!/usr/bin/env bash
# MOCK_AGY_IGNORE_TERM=1: an agy that ignores SIGTERM, so only the
# helper's escalation to SIGKILL can end it (#313).
[[ -n "${MOCK_AGY_IGNORE_TERM:-}" ]] && trap '' TERM HUP
# MOCK_AGY_ORPHAN_PIDFILE=<f>: start a child that ignores SIGTERM and
# outlives this mock unless its whole process group is killed; its pid
# goes to <f> (#660). Its fds are detached so it holds no caller pipe.
if [[ -n "${MOCK_AGY_ORPHAN_PIDFILE:-}" ]]; then
    ( trap '' TERM HUP; echo "$BASHPID" > "${MOCK_AGY_ORPHAN_PIDFILE}"
      for ((j = 0; j < 300; j++)); do sleep 0.1; done ) </dev/null >/dev/null 2>&1 &
    for ((j = 0; j < 50; j++)); do [[ -s "${MOCK_AGY_ORPHAN_PIDFILE}" ]] && break; sleep 0.05; done
fi
# Sleep in 0.1s slices so a SIGKILLed mock leaves no long-lived orphan
# sleep behind. Whole seconds only.
mock_sleep() { local n="$1" i; for ((i = 0; i < n * 10; i++)); do sleep 0.1; done; }
if [[ -n "${MOCK_AGY_LOG:-}" ]]; then
    printf '%s\n' "$@" >> "${MOCK_AGY_LOG}"
fi
[[ -n "${MOCK_TIMES_DIR:-}" ]] && echo $$ > "${MOCK_TIMES_DIR}/agy.pid"
if [[ -n "${MOCK_AGY_EXIT:-}" ]]; then
    echo "boom" >&2
    exit "${MOCK_AGY_EXIT}"
fi
# Stream-json contract: exactly one NDJSON message per line. A
# pretty-printed (multi-line) message is a contract violation even if a
# lenient JSON reader would accept it, so the mock refuses it.
input=$(cat)
if [[ "$(printf '%s\n' "$input" | wc -l)" -ne 1 ]]; then
    echo "mock agy: stdin is not a single NDJSON line" >&2
    exit 9
fi
if [[ -n "${MOCK_AGY_STALL:-}" ]]; then
    # Wedged: the prompt was consumed, but no result event and no
    # --print-timeout handling ever happens. Only an outer bound ends it.
    # `exec` so the recorded pid IS the sleep: killing it leaves no
    # orphaned child behind. With IGNORE_TERM the trap above must stay
    # installed, so this process sleeps in slices instead of exec'ing.
    if [[ -n "${MOCK_AGY_IGNORE_TERM:-}" ]]; then
        mock_sleep 60
        exit 0
    fi
    exec sleep 60
fi
[[ -n "${MOCK_AGY_SLEEP:-}" ]] && mock_sleep "${MOCK_AGY_SLEEP}"
echo 'agy: a newer version is available (mock banner, not JSON)'
echo '{"event":"init","init":{"tools":[]}}'
if [[ -n "${MOCK_AGY_DENY:-}" ]]; then
    echo 'jetski: no output produced — a tool required the "command" permission that headless mode cannot prompt for, so it was auto-denied.' >&2
    jq -cn --arg a "${MOCK_AGY_DENY_ACTION:-RunCommand}" '{event:"result",result:{status:"SUCCESS",response:"",denied_actions:[{action:"command",display_name:$a}]}}'
    exit 0
fi
if [[ -n "${MOCK_AGY_ERROR:-}" ]]; then
    [[ -n "${MOCK_AGY_ERROR_STDERR:-}" ]] && echo "${MOCK_AGY_ERROR_STDERR}" >&2
    if [[ -n "${MOCK_AGY_ERROR_STRING:-}" ]]; then
        jq -cn --arg m "$MOCK_AGY_ERROR" --arg r "${MOCK_AGY_ERROR_RESPONSE:-}" \
            '{event:"result",result:{status:"ERROR",response:$r,error:$m}}'
    else
        jq -cn --arg m "$MOCK_AGY_ERROR" --arg r "${MOCK_AGY_ERROR_RESPONSE:-}" \
            '{event:"result",result:{status:"ERROR",response:$r,error:{code:429,message:$m}}}'
    fi
    exit 0
fi
if [[ -n "${MOCK_AGY_TIMEOUT:-}" ]]; then
    echo '[agy] print timeout after 1s with turn in progress; returning partial output' >&2
    echo '{"event":"result","result":{"status":"SUCCESS","response":"partial text"}}'
    exit 0
fi
# Echo the prompt back as the response. Streamed, never a shell variable
# or argv: the whole point of the stdin contract is prompts larger than
# the kernel's per-argument limit, and the mock must not reintroduce it.
if [[ -n "${MOCK_AGY_DENY_PARTIAL:-}" ]]; then
    printf '%s\n' "$input" | jq -r 'select(.event == "user") | .message.content' \
        | jq -c -Rs '{event:"result",result:{status:"SUCCESS",response:.,denied_actions:[{"action":"command","display_name":"RunCommand"}]}}'
    exit 0
fi
printf '%s\n' "$input" | jq -r 'select(.event == "user") | .message.content' \
    | jq -c -Rs '{event:"result",result:{status:"SUCCESS",response:.}}'
MOCK_EOF
    chmod +x "${MOCK_BIN}/agy"

    # Mock gh: valid PR body, non-empty diff. MOCK_GH_DIFF_FILE (env)
    # substitutes a prepared diff for `gh pr diff`.
    cat > "${MOCK_BIN}/gh" << 'GH_EOF'
#!/usr/bin/env bash
if [[ "$1" == "pr" && "$2" == "view" ]]; then
    shift 2; PR="$1"; shift
    [[ "${1:-}" == "-R" ]] && shift 2
    if [[ "$1" == "--json" && "$2" == "body" ]]; then
        echo "Closes #42"
    elif [[ "$1" == "--json" && "$2" == "title" ]]; then
        echo "Test PR"
    elif [[ "$1" == "--json" && "$2" == "url" ]]; then
        echo "https://github.com/test/repo/pull/99"
    fi
elif [[ "$1" == "pr" && "$2" == "diff" ]]; then
    if [[ -n "${MOCK_GH_DIFF_FILE:-}" ]]; then
        cat "${MOCK_GH_DIFF_FILE}"
    else
        echo "diff --git a/file.txt b/file.txt"
        echo "--- a/file.txt"
        echo "+++ b/file.txt"
        echo "@@ -1 +1 @@"
        echo "-old"
        echo "+new"
    fi
fi
exit 0
GH_EOF
    chmod +x "${MOCK_BIN}/gh"
}

teardown() {
    [[ -n "$TMPDIR_BASE" ]] && rm -rf "$TMPDIR_BASE"
}

assert_eq() {
    local label="$1" expected="$2" actual="$3"
    if [[ "$expected" == "$actual" ]]; then
        echo "  PASS: $label"
        PASS=$((PASS + 1))
    else
        echo "  FAIL: $label"
        echo "    expected: $expected"
        echo "    actual:   $actual"
        FAIL=$((FAIL + 1))
    fi
}

assert_contains() {
    local label="$1" pattern="$2" text="$3"
    if echo "$text" | grep -qE "$pattern"; then
        echo "  PASS: $label"
        PASS=$((PASS + 1))
    else
        echo "  FAIL: $label"
        echo "    pattern not found: $pattern"
        echo "    in: $text"
        FAIL=$((FAIL + 1))
    fi
}

assert_not_contains() {
    local label="$1" pattern="$2" text="$3"
    if echo "$text" | grep -qE "$pattern"; then
        echo "  FAIL: $label"
        echo "    unexpected pattern found: $pattern"
        echo "    in: $text"
        FAIL=$((FAIL + 1))
    else
        echo "  PASS: $label"
        PASS=$((PASS + 1))
    fi
}

assert_exit_code() {
    local label="$1" expected="$2" actual="$3"
    if [[ "$expected" == "$actual" ]]; then
        echo "  PASS: $label (exit $actual)"
        PASS=$((PASS + 1))
    else
        echo "  FAIL: $label"
        echo "    expected exit: $expected"
        echo "    actual exit:   $actual"
        FAIL=$((FAIL + 1))
    fi
}

# ---- Test: --repo flag is accepted and overrides auto-detection ----
test_repo_flag_accepted() {
    echo "TEST: --repo flag is accepted"
    setup

    # Mock gh that records arguments and returns direct values for --jq-style
    # queries used by the test.
    cat > "${MOCK_BIN}/gh" << 'GH_EOF'
#!/usr/bin/env bash
echo "$@" >> "${MOCK_GH_LOG}"
# Detect which gh subcommand
if [[ "$1" == "pr" && "$2" == "view" ]]; then
    shift 2  # consume "pr view"
    PR_NUM="$1"; shift
    # Consume -R flag if present
    if [[ "${1:-}" == "-R" ]]; then
        echo "REPO_FLAG=$2" >> "${MOCK_GH_LOG}"
        shift 2
    fi
    if [[ "$1" == "--json" && "$2" == "body" ]]; then
        echo "Closes #42"
        exit 0
    elif [[ "$1" == "--json" && "$2" == "title" ]]; then
        echo "Test PR"
        exit 0
    elif [[ "$1" == "--json" && "$2" == "url" ]]; then
        echo "https://github.com/test/repo/pull/99"
        exit 0
    fi
elif [[ "$1" == "pr" && "$2" == "diff" ]]; then
    shift 2
    PR_NUM="$1"; shift
    if [[ "${1:-}" == "-R" ]]; then
        echo "REPO_FLAG=$2" >> "${MOCK_GH_LOG}"
        shift 2
    fi
    echo "diff --git a/file.txt b/file.txt"
    echo "--- a/file.txt"
    echo "+++ b/file.txt"
    echo "@@ -1 +1 @@"
    echo "-old"
    echo "+new"
    exit 0
fi
exit 0
GH_EOF
    chmod +x "${MOCK_BIN}/gh"

    export MOCK_GH_LOG="${TMPDIR_BASE}/gh_calls.log"
    true > "$MOCK_GH_LOG"

    # Run the script with --repo. Set
    # WORKTREE_ISSUE=42 (matching the mock PR body's "Closes #42") so the
    # work-plans-dir resolver (issue #147) accepts the invocation instead
    # of aborting with "not in matching worktree."
    cd "${MOCK_REPO}"
    PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 bash "${SCRIPT_UNDER_TEST}" \
        --pr 99 --repo test/repo  >/dev/null 2>&1 || true

    # Verify gh was called with -R test/repo
    if grep -q "REPO_FLAG=test/repo" "$MOCK_GH_LOG"; then
        echo "  PASS: --repo flag passed through to gh as -R"
        PASS=$((PASS + 1))
    else
        echo "  FAIL: --repo flag not passed through to gh"
        echo "    gh log: $(cat "$MOCK_GH_LOG")"
        FAIL=$((FAIL + 1))
    fi

    teardown
}

# ---- Test: issue extraction from PR body ----
# Helper: mirrors the extraction logic from cross_model_review.sh.
# Post-#149: keyword-only — no loose "#N anywhere" fallback.
extract_issue() {
    local body="$1"
    local ref num
    ref=$(printf '%s\n' "$body" \
        | grep -ioE '(^|[^[:alnum:]_])(closes|fixes|resolves)[[:space:]]+([a-zA-Z0-9._-]+/[a-zA-Z0-9._-]+)?#[0-9]+' \
        | head -n1 || true)
    num=$(printf '%s\n' "$ref" | grep -oE '[0-9]+$' || true)
    printf '%s' "${num:-}"
}

test_issue_extraction() {
    echo "TEST: issue number extraction (keyword-only, post-#149)"

    # Positive cases — keyword match wins
    assert_eq "Closes #42 -> 42" "42" "$(extract_issue 'Some text. Closes #42. More text.')"
    assert_eq "fixes #123 -> 123" "123" "$(extract_issue 'fixes #123')"
    assert_eq "Resolves owner/repo#77 -> 77" "77" "$(extract_issue 'Resolves owner/repo#77')"
    assert_eq "CLOSES #5 -> 5" "5" "$(extract_issue 'CLOSES #5')"
    # A real keyword later in the body wins over substring false positives
    assert_eq "encloses #42, Closes #99 -> 99" "99" "$(extract_issue 'encloses #42 but Closes #99')"

    # Post-#149: no keyword means empty — no loose "#N anywhere" fallback
    assert_eq "encloses #42 (substring only) -> empty" "" "$(extract_issue 'encloses #42')"
    assert_eq "prefixes #7 (substring only) -> empty" "" "$(extract_issue 'prefixes #7')"
    assert_eq "no keyword, '#N' in body -> empty" "" "$(extract_issue 'Related to #10 and #20')"
    assert_eq "No issue ref -> empty" "" "$(extract_issue 'No issue reference here')"
}

# ---- Test: --work-dir controls artifact placement ----
test_work_dir_flag() {
    echo "TEST: --work-dir controls artifact placement"
    setup

    local custom_dir="${TMPDIR_BASE}/custom_workdir"
    mkdir -p "$custom_dir"

    # Mock gh
    cat > "${MOCK_BIN}/gh" << 'GH_EOF'
#!/usr/bin/env bash
if [[ "$1" == "pr" && "$2" == "view" ]]; then
    shift 2; PR="$1"; shift
    [[ "${1:-}" == "-R" ]] && shift 2
    if [[ "$1" == "--json" && "$2" == "body" ]]; then
        echo "Closes #42"
    elif [[ "$1" == "--json" && "$2" == "title" ]]; then
        echo "Test PR"
    elif [[ "$1" == "--json" && "$2" == "url" ]]; then
        echo "https://github.com/test/repo/pull/99"
    fi
elif [[ "$1" == "pr" && "$2" == "diff" ]]; then
    echo "diff --git a/file.txt b/file.txt"
    echo "--- a/file.txt"
    echo "+++ b/file.txt"
    echo "@@ -1 +1 @@"
    echo "-old"
    echo "+new"
fi
exit 0
GH_EOF
    chmod +x "${MOCK_BIN}/gh"

    cd "${MOCK_REPO}"
    PATH="${MOCK_BIN}:${PATH}" bash "${SCRIPT_UNDER_TEST}" \
        --pr 99 --work-dir "${custom_dir}"  >/dev/null 2>&1 || true

    # Check that artifacts were written under custom_dir, not repo root
    if [[ -d "${custom_dir}/.agent/work-plans/issue-42" ]]; then
        echo "  PASS: artifacts written under --work-dir"
        PASS=$((PASS + 1))
    else
        echo "  FAIL: artifacts not found under --work-dir"
        echo "    expected dir: ${custom_dir}/.agent/work-plans/issue-42"
        echo "    ls custom_dir: $(find "${custom_dir}" -type f 2>/dev/null || echo 'empty')"
        FAIL=$((FAIL + 1))
    fi

    # Also verify artifacts are NOT under the repo root
    if [[ -d "${MOCK_REPO}/.agent/work-plans/issue-42" ]]; then
        echo "  FAIL: artifacts leaked to repo root despite --work-dir"
        FAIL=$((FAIL + 1))
    else
        echo "  PASS: no artifacts in repo root"
        PASS=$((PASS + 1))
    fi

    teardown
}

# ---- Test: empty diff guard ----
test_empty_diff_guard() {
    echo "TEST: empty diff guard exits with error"
    setup

    # Mock gh that returns empty diff
    cat > "${MOCK_BIN}/gh" << 'GH_EOF'
#!/usr/bin/env bash
if [[ "$1" == "pr" && "$2" == "view" ]]; then
    shift 2; PR="$1"; shift
    [[ "${1:-}" == "-R" ]] && shift 2
    if [[ "$1" == "--json" && "$2" == "body" ]]; then
        echo "Closes #42"
    elif [[ "$1" == "--json" && "$2" == "title" ]]; then
        echo "Test PR"
    elif [[ "$1" == "--json" && "$2" == "url" ]]; then
        echo "https://github.com/test/repo/pull/99"
    fi
elif [[ "$1" == "pr" && "$2" == "diff" ]]; then
    # Return empty diff (no output)
    true
fi
exit 0
GH_EOF
    chmod +x "${MOCK_BIN}/gh"

    # WORKTREE_ISSUE=42 matches the mock PR's "Closes #42" so the resolver
    # (issue #147) accepts the invocation; this test exercises the empty-
    # diff guard, not the worktree check.
    cd "${MOCK_REPO}"
    local exit_code=0
    STDERR=$(PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 bash "${SCRIPT_UNDER_TEST}" \
        --pr 99  2>&1) || exit_code=$?

    assert_exit_code "empty diff exits 3" "3" "$exit_code"
    assert_contains "error message mentions empty diff" "diff is empty" "$STDERR"

    # Check that an error marker was written to findings file
    local findings_file="${MOCK_REPO}/.agent/work-plans/issue-42/review-gemini-findings.md"
    if [[ -f "$findings_file" ]]; then
        local content
        content=$(cat "$findings_file")
        assert_contains "findings file has error marker" "Review error" "$content"
    else
        echo "  FAIL: findings file not created for error marker"
        FAIL=$((FAIL + 1))
    fi

    teardown
}

# ---- Test: missing --pr flag ----
test_missing_pr_flag() {
    echo "TEST: missing --pr flag exits 2"

    local exit_code=0
    bash "${SCRIPT_UNDER_TEST}" --agent gemini 2>/dev/null || exit_code=$?
    assert_exit_code "missing --pr exits 2" "2" "$exit_code"
}

# ---- Test: unknown argument ----
test_unknown_argument() {
    echo "TEST: unknown argument exits 2"

    local exit_code=0
    bash "${SCRIPT_UNDER_TEST}" --pr 1 --bogus 2>/dev/null || exit_code=$?
    assert_exit_code "unknown arg exits 2" "2" "$exit_code"
}

# ---- Test: --repo with invalid slug ----
test_invalid_repo_slug() {
    echo "TEST: --repo with invalid slug exits 2"
    setup

    local exit_code=0
    STDERR=$(PATH="${MOCK_BIN}:${PATH}" bash "${SCRIPT_UNDER_TEST}" --pr 1 --repo "not-a-slug" 2>&1) || exit_code=$?
    assert_exit_code "invalid slug exits 2" "2" "$exit_code"
    assert_contains "error mentions invalid slug" "not a valid owner/repo" "$STDERR"

    teardown
}

# ---- Test: resolver refuses outside matching worktree ----
test_resolver_refuses_without_worktree_issue() {
    echo "TEST: resolver refuses when WORKTREE_ISSUE unset / mismatched"
    setup

    # Mock gh returns a PR body with "Closes #42"
    cat > "${MOCK_BIN}/gh" << 'GH_EOF'
#!/usr/bin/env bash
if [[ "$1" == "pr" && "$2" == "view" ]]; then
    shift 2; PR="$1"; shift
    [[ "${1:-}" == "-R" ]] && shift 2
    if [[ "$1" == "--json" && "$2" == "body" ]]; then
        echo "Closes #42"
    fi
fi
exit 0
GH_EOF
    chmod +x "${MOCK_BIN}/gh"

    cd "${MOCK_REPO}"

    # Case 1: WORKTREE_ISSUE unset -> resolver rule 3 aborts with exit 4.
    local exit_code=0
    STDERR=$(unset WORKTREE_ISSUE; PATH="${MOCK_BIN}:${PATH}" \
        bash "${SCRIPT_UNDER_TEST}" --pr 99  2>&1) || exit_code=$?
    assert_exit_code "unset WORKTREE_ISSUE exits 4" "4" "$exit_code"
    assert_contains "error mentions worktree" "worktree" "$STDERR"

    # Case 2: WORKTREE_ISSUE mismatched -> same abort, different message.
    exit_code=0
    STDERR=$(PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=100 \
        bash "${SCRIPT_UNDER_TEST}" --pr 99  2>&1) || exit_code=$?
    assert_exit_code "mismatched WORKTREE_ISSUE exits 4" "4" "$exit_code"
    assert_contains "error names the mismatch" "'100', not '42'" "$STDERR"

    teardown
}

# ---- Test: flag-as-value is rejected ----
test_flag_as_value_rejected() {
    echo "TEST: --flag --other-flag pattern is rejected"

    local exit_code=0
    STDERR=$(bash "${SCRIPT_UNDER_TEST}" --work-plans-dir --no-progress 2>&1) || exit_code=$?
    assert_exit_code "--work-plans-dir --no-progress exits 2" "2" "$exit_code"
    assert_contains "error mentions missing value" "Missing value for --work-plans-dir" "$STDERR"

    exit_code=0
    STDERR=$(bash "${SCRIPT_UNDER_TEST}" --pr --no-progress 2>&1) || exit_code=$?
    assert_exit_code "--pr --no-progress exits 2" "2" "$exit_code"
    assert_contains "error mentions missing value" "Missing value for --pr" "$STDERR"
}

# ---- Test: gh repo view resolves SSH host alias / Enterprise URLs (#150) ----
#
# Before #150, GH_REPO_SLUG was extracted via a sed pipeline on `git
# remote get-url origin` that assumed a literal `github.com` hostname.
# SSH host aliases (`git@github-work:owner/repo.git`) and Enterprise
# hostnames (`git@github.mycorp.com:owner/repo.git`) produced garbage
# slugs that were either silently dropped or misrouted to the wrong repo.
#
# Post-#150, the script defers to `gh repo view --json nameWithOwner`,
# which uses gh's own repo-resolution (reads ~/.ssh/config, respects
# GH_HOST, etc.). This test mocks a git remote using an SSH alias and a
# `gh repo view` response that returns the intended slug, then asserts
# the `-R` flag forwarded to downstream `gh pr view` matches.
test_gh_repo_view_resolves_alias() {
    echo "TEST: gh repo view resolves SSH alias / Enterprise URLs (#150)"
    setup

    # Point the mock repo's origin at an SSH host alias from the old
    # sed pipeline would have mangled.
    git -C "${MOCK_REPO}" remote add origin "git@github-work:real-owner/real-repo.git" 2>/dev/null \
        || git -C "${MOCK_REPO}" remote set-url origin "git@github-work:real-owner/real-repo.git"

    export MOCK_GH_LOG="${TMPDIR_BASE}/gh_calls.log"
    true > "$MOCK_GH_LOG"

    # Mock gh: `repo view` returns the intended slug (as real gh would
    # via ~/.ssh/config); `pr view` / `pr diff` record their -R args.
    cat > "${MOCK_BIN}/gh" << 'GH_EOF'
#!/usr/bin/env bash
echo "$@" >> "${MOCK_GH_LOG}"
if [[ "$1" == "repo" && "$2" == "view" ]]; then
    # Respond only when asked for nameWithOwner (what the script wants)
    if [[ " $* " == *" --json nameWithOwner "* ]]; then
        echo "real-owner/real-repo"
        exit 0
    fi
    exit 0
elif [[ "$1" == "pr" && "$2" == "view" ]]; then
    shift 2; PR="$1"; shift
    if [[ "${1:-}" == "-R" ]]; then
        echo "REPO_FLAG=$2" >> "${MOCK_GH_LOG}"
        shift 2
    fi
    if [[ "$1" == "--json" && "$2" == "body" ]]; then
        echo "Closes #42"
    elif [[ "$1" == "--json" && "$2" == "title" ]]; then
        echo "Test PR"
    elif [[ "$1" == "--json" && "$2" == "url" ]]; then
        echo "https://github.com/real-owner/real-repo/pull/99"
    fi
elif [[ "$1" == "pr" && "$2" == "diff" ]]; then
    echo "diff --git a/file.txt b/file.txt"
    echo "--- a/file.txt"
    echo "+++ b/file.txt"
    echo "@@ -1 +1 @@"
    echo "-old"
    echo "+new"
fi
exit 0
GH_EOF
    chmod +x "${MOCK_BIN}/gh"

    cd "${MOCK_REPO}"
    PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 bash "${SCRIPT_UNDER_TEST}" \
        --pr 99  >/dev/null 2>&1 || true

    # Assert `gh repo view --json nameWithOwner` was called.
    if grep -q "^repo view --json nameWithOwner" "$MOCK_GH_LOG"; then
        echo "  PASS: gh repo view --json nameWithOwner was called"
        PASS=$((PASS + 1))
    else
        echo "  FAIL: gh repo view was not invoked for slug resolution"
        echo "    gh log:"
        sed 's/^/      /' "$MOCK_GH_LOG"
        FAIL=$((FAIL + 1))
    fi

    # Assert downstream pr view received the resolved slug as -R.
    if grep -q "REPO_FLAG=real-owner/real-repo" "$MOCK_GH_LOG"; then
        echo "  PASS: resolved slug forwarded to downstream gh as -R"
        PASS=$((PASS + 1))
    else
        echo "  FAIL: resolved slug not forwarded; would have misrouted"
        echo "    gh log:"
        sed 's/^/      /' "$MOCK_GH_LOG"
        FAIL=$((FAIL + 1))
    fi

    teardown
}

# ---- Test: gh repo view failure falls back cleanly (no -R, no abort) ----
#
# If the cwd isn't a recognized gh repo (no remote, or a non-github
# remote), `gh repo view --json nameWithOwner` exits non-zero. The
# script should treat this as "no explicit slug" and omit -R, letting
# downstream gh calls do their own resolution rather than aborting.
test_gh_repo_view_failure_falls_back() {
    echo "TEST: gh repo view failure => no -R, script continues"
    setup

    export MOCK_GH_LOG="${TMPDIR_BASE}/gh_calls.log"
    true > "$MOCK_GH_LOG"

    cat > "${MOCK_BIN}/gh" << 'GH_EOF'
#!/usr/bin/env bash
echo "$@" >> "${MOCK_GH_LOG}"
if [[ "$1" == "repo" && "$2" == "view" ]]; then
    # Simulate "not a github repo" — exit non-zero, no output.
    exit 1
elif [[ "$1" == "pr" && "$2" == "view" ]]; then
    shift 2; PR="$1"; shift
    if [[ "${1:-}" == "-R" ]]; then
        echo "REPO_FLAG=$2" >> "${MOCK_GH_LOG}"
        shift 2
    fi
    if [[ "$1" == "--json" && "$2" == "body" ]]; then
        echo "Closes #42"
    elif [[ "$1" == "--json" && "$2" == "title" ]]; then
        echo "Test PR"
    elif [[ "$1" == "--json" && "$2" == "url" ]]; then
        echo "https://github.com/fallback/repo/pull/99"
    fi
elif [[ "$1" == "pr" && "$2" == "diff" ]]; then
    echo "diff --git a/file.txt b/file.txt"
    echo "--- a/file.txt"
    echo "+++ b/file.txt"
    echo "@@ -1 +1 @@"
    echo "-old"
    echo "+new"
fi
exit 0
GH_EOF
    chmod +x "${MOCK_BIN}/gh"

    cd "${MOCK_REPO}"
    PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 bash "${SCRIPT_UNDER_TEST}" \
        --pr 99  >/dev/null 2>&1 || true

    # -R should NOT have been passed since slug resolution failed.
    if grep -q "REPO_FLAG=" "$MOCK_GH_LOG"; then
        echo "  FAIL: -R was passed despite gh repo view failing"
        echo "    gh log:"
        sed 's/^/      /' "$MOCK_GH_LOG"
        FAIL=$((FAIL + 1))
    else
        echo "  PASS: no -R when gh repo view fails"
        PASS=$((PASS + 1))
    fi

    # Script should still have proceeded to pr view (graceful fallback).
    if grep -q "^pr view" "$MOCK_GH_LOG"; then
        echo "  PASS: script proceeded to pr view after slug-resolve failure"
        PASS=$((PASS + 1))
    else
        echo "  FAIL: script did not proceed past slug resolution"
        FAIL=$((FAIL + 1))
    fi

    teardown
}

# ---- Test: --issue flag overrides PR-body extraction (#149) ----
#
# When --issue <N> is passed, the script must honour it verbatim without
# consulting the PR body. This is the escape hatch for PRs that don't
# use Closes/Fixes/Resolves keywords (rollup PRs, long-running
# investigations, etc.).
test_issue_flag_overrides_extraction() {
    echo "TEST: --issue overrides PR-body extraction (#149)"
    setup

    export MOCK_GH_LOG="${TMPDIR_BASE}/gh_calls.log"
    true > "$MOCK_GH_LOG"

    # Mock gh: PR body has NO closure keyword — extraction would fail
    # without --issue. With --issue the body shouldn't even be queried
    # for body (but we still need view for title/url; returning body
    # anyway is harmless because the script skips extraction).
    cat > "${MOCK_BIN}/gh" << 'GH_EOF'
#!/usr/bin/env bash
echo "$@" >> "${MOCK_GH_LOG}"
if [[ "$1" == "pr" && "$2" == "view" ]]; then
    shift 2; PR="$1"; shift
    [[ "${1:-}" == "-R" ]] && shift 2
    if [[ "$1" == "--json" && "$2" == "body" ]]; then
        echo "A PR body with no closure keyword. See also #42."
    elif [[ "$1" == "--json" && "$2" == "title" ]]; then
        echo "Test PR"
    elif [[ "$1" == "--json" && "$2" == "url" ]]; then
        echo "https://github.com/test/repo/pull/99"
    fi
elif [[ "$1" == "pr" && "$2" == "diff" ]]; then
    echo "diff --git a/file.txt b/file.txt"
    echo "--- a/file.txt"
    echo "+++ b/file.txt"
    echo "@@ -1 +1 @@"
    echo "-old"
    echo "+new"
fi
exit 0
GH_EOF
    chmod +x "${MOCK_BIN}/gh"

    cd "${MOCK_REPO}"
    # --issue 123 matches WORKTREE_ISSUE so the resolver accepts it; the
    # wrong match (#42 from the loose fallback) would have been picked
    # before #149 and broken this test.
    PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=123 bash "${SCRIPT_UNDER_TEST}" \
        --pr 99 --issue 123  >/dev/null 2>&1 || true

    # Artifacts should land under issue-123 (from --issue), not issue-42
    # (from the PR body).
    if [[ -d "${MOCK_REPO}/.agent/work-plans/issue-123" ]]; then
        echo "  PASS: --issue value used as issue number"
        PASS=$((PASS + 1))
    else
        echo "  FAIL: --issue value not used"
        echo "    ls work-plans: $(ls "${MOCK_REPO}/.agent/work-plans/" 2>/dev/null || echo 'empty')"
        FAIL=$((FAIL + 1))
    fi
    if [[ -d "${MOCK_REPO}/.agent/work-plans/issue-42" ]]; then
        echo "  FAIL: loose fallback still used — routed to #42"
        FAIL=$((FAIL + 1))
    else
        echo "  PASS: PR-body #42 not used"
        PASS=$((PASS + 1))
    fi

    # Also assert the script skipped the PR-body extraction entirely
    # when --issue was supplied (no `gh pr view ... --json body` call).
    # Tightens the test per review feedback on PR #154.
    if grep -qE "^pr view .* --json body" "$MOCK_GH_LOG"; then
        echo "  FAIL: gh pr view --json body was called despite --issue"
        echo "    gh log:"
        sed 's/^/      /' "$MOCK_GH_LOG"
        FAIL=$((FAIL + 1))
    else
        echo "  PASS: PR-body extraction skipped when --issue is set"
        PASS=$((PASS + 1))
    fi

    teardown
}

# ---- Test: missing closure keyword aborts with guidance (#149) ----
#
# Before #149 the script silently routed artifacts to the first '#N'
# found anywhere in the PR body, or fell back to the PR number. Both
# behaviors hid real errors. Now: no keyword + no --issue => exit 2.
test_missing_keyword_aborts() {
    echo "TEST: missing closure keyword without --issue aborts (#149)"
    setup

    # Mock gh: PR body deliberately has only a loose '#N' reference and
    # a substring like "encloses #42" that should not be picked up.
    cat > "${MOCK_BIN}/gh" << 'GH_EOF'
#!/usr/bin/env bash
if [[ "$1" == "pr" && "$2" == "view" ]]; then
    shift 2; PR="$1"; shift
    [[ "${1:-}" == "-R" ]] && shift 2
    if [[ "$1" == "--json" && "$2" == "body" ]]; then
        echo "Related to #42 (encloses #7). No closure keyword here."
    elif [[ "$1" == "--json" && "$2" == "title" ]]; then
        echo "Test PR"
    elif [[ "$1" == "--json" && "$2" == "url" ]]; then
        echo "https://github.com/test/repo/pull/99"
    fi
fi
exit 0
GH_EOF
    chmod +x "${MOCK_BIN}/gh"

    cd "${MOCK_REPO}"
    local exit_code=0
    local stderr
    stderr=$(PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 bash "${SCRIPT_UNDER_TEST}" \
        --pr 99  2>&1) || exit_code=$?

    assert_exit_code "missing keyword exits 2" "2" "$exit_code"
    # Pattern avoids `|` (which grep -E would treat as alternation and
    # accept a partial match). Testing an unambiguous fragment of the
    # error message instead — per review feedback on PR #154.
    assert_contains "error mentions missing keyword" \
        "body has no 'Closes" "$stderr"
    # Pattern must not start with "--" so grep -E doesn't treat it as a flag.
    assert_contains "error suggests --issue flag" "Pass --issue" "$stderr"

    # No artifacts should have been written (abort before resolver).
    if [[ -d "${MOCK_REPO}/.agent/work-plans/issue-42" ]] || \
       [[ -d "${MOCK_REPO}/.agent/work-plans/issue-7" ]]; then
        echo "  FAIL: artifacts leaked from the loose fallback"
        FAIL=$((FAIL + 1))
    else
        echo "  PASS: no artifacts written when extraction fails"
        PASS=$((PASS + 1))
    fi


    # With --no-progress there is nothing to file under, so a keyword-less
    # PR is reviewed into a temp dir instead of refused — what review-code
    # passes when its own step 1 found no closing issue (#660).
    exit_code=0
    stderr=$(PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 bash "${SCRIPT_UNDER_TEST}" \
        --pr 99 --no-progress 2>&1) || exit_code=$?
    assert_not_contains "--no-progress: no missing-keyword refusal" "body has no 'Closes" "$stderr"
    if [[ -e "${MOCK_REPO}/.agent/work-plans/issue-noprogress" ]]; then
        echo "  FAIL: --no-progress wrote an issue-noprogress dir"; FAIL=$((FAIL + 1))
    else
        echo "  PASS: --no-progress wrote nothing into the repo"; PASS=$((PASS + 1))
    fi

    teardown
}

# ---- Test: --issue validates positive integer shape (#149) ----
test_issue_flag_validates_integer() {
    echo "TEST: --issue rejects non-integer values (#149)"

    local exit_code=0
    local stderr
    stderr=$(bash "${SCRIPT_UNDER_TEST}" --pr 99 --issue not-a-number 2>&1) || exit_code=$?
    assert_exit_code "non-integer --issue exits 2" "2" "$exit_code"
    assert_contains "error mentions integer contract" \
        "not a positive integer" "$stderr"

    exit_code=0
    stderr=$(bash "${SCRIPT_UNDER_TEST}" --pr 99 --issue 0 2>&1) || exit_code=$?
    assert_exit_code "--issue 0 rejected" "2" "$exit_code"

    exit_code=0
    stderr=$(bash "${SCRIPT_UNDER_TEST}" --pr 99 --issue -5 2>&1) || exit_code=$?
    assert_exit_code "--issue -5 rejected" "2" "$exit_code"
    # Post-review: require_value was narrowed from -* to --*, so -5
    # now reaches the integer validator instead of being caught as a
    # "missing value" flag. Both paths exit 2; the integer message is
    # more accurate.
    assert_contains "error mentions integer contract for -5" \
        "not a positive integer" "$stderr"
}

# ---- Test: gh pr view failure produces a retrieval-specific error (#149) ----
#
# Regression test for the review fix: when gh fails (auth/permissions/
# network), the script must NOT emit the "no closure keyword" guidance,
# which would point users at the wrong remediation.
test_gh_pr_view_failure_distinct_error() {
    echo "TEST: gh pr view failure produces a distinct error (#149)"
    setup

    # Mock gh that fails on `pr view --json body` (exit non-zero).
    cat > "${MOCK_BIN}/gh" << 'GH_EOF'
#!/usr/bin/env bash
if [[ "$1" == "pr" && "$2" == "view" ]]; then
    shift 2; PR="$1"; shift
    [[ "${1:-}" == "-R" ]] && shift 2
    if [[ "$1" == "--json" && "$2" == "body" ]]; then
        # Simulate auth/network/permission failure
        echo "gh: authentication required" >&2
        exit 1
    fi
fi
exit 0
GH_EOF
    chmod +x "${MOCK_BIN}/gh"

    cd "${MOCK_REPO}"
    local exit_code=0
    local stderr
    stderr=$(PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 bash "${SCRIPT_UNDER_TEST}" \
        --pr 99  2>&1) || exit_code=$?

    assert_exit_code "gh failure exits 2" "2" "$exit_code"
    assert_contains "error mentions retrieval failure" \
        "Failed to retrieve body" "$stderr"
    # Must NOT fall through to the no-keyword remediation — that would
    # be misleading when the real problem is auth/network.
    assert_not_contains "no-keyword guidance suppressed on gh failure" \
        "body has no 'Closes" "$stderr"

    teardown
}

# ---- Gemini/agy tests (#223, #274, #288, #312) ----
#
# The Gemini CLI migrated to the `agy` binary (#223). cross_model_review.sh
# drives it through _agy_review.sh, which feeds the prompt over stdin as a
# stream-json message (#274: no argv size limit) and validates the result
# event (#288: a headless permission denial exits 0 with an empty response).
# The mock agy in setup() implements that contract; see its knobs there.

# Run the script (single-agent gemini) for PR 99 (issue 42) and echo the exit code.
run_gemini_sync() {
    cd "${MOCK_REPO}"
    local exit_code=0
    PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 bash "${SCRIPT_UNDER_TEST}" \
        --pr 99  < /dev/null >/dev/null 2>&1 || exit_code=$?
    echo "$exit_code"
}

FINDINGS_REL=".agent/work-plans/issue-42/review-gemini-findings.md"
PROMPT_REL=".agent/work-plans/issue-42/review-gemini-prompt.md"

test_agy_stdin_invocation() {
    echo "TEST: gemini agent feeds agy the prompt over stdin, argv carries only flags (#274)"
    setup

    export MOCK_AGY_LOG="${TMPDIR_BASE}/agy_calls.log"
    true > "$MOCK_AGY_LOG"
    local exit_code
    exit_code=$(run_gemini_sync)
    unset MOCK_AGY_LOG

    assert_exit_code "review completes (exit 0)" "0" "$exit_code"

    local agy_log
    agy_log=$(cat "${TMPDIR_BASE}/agy_calls.log")
    assert_contains "agy received --input-format=stream-json" "^--input-format=stream-json$" "$agy_log"
    assert_contains "agy received --output-format=stream-json" "^--output-format=stream-json$" "$agy_log"
    assert_contains "agy received --print-timeout" "^--print-timeout$" "$agy_log"
    assert_contains "agy received --disable-slash-commands" "^--disable-slash-commands$" "$agy_log"
    assert_contains "agy received an empty -p=" "^-p=$" "$agy_log"
    assert_not_contains "prompt content is NOT on argv" "Adversarial Code Review" "$agy_log"

    local content
    content=$(cat "${MOCK_REPO}/${FINDINGS_REL}")
    assert_contains "findings file holds agy's response (prompt echoed via stdin)" \
        "Adversarial Code Review" "$content"
    assert_contains "findings file has completion marker" "Review complete" "$content"
    assert_not_contains "no denial note on a clean run" "denied in headless mode" "$content"

    teardown
}

test_agy_large_prompt() {
    echo "TEST: a >200 KiB prompt reaches agy (argv limit no longer applies, #274)"
    setup

    # ~256 KiB of diff: well past MAX_ARG_STRLEN (128 KiB on Linux). An
    # argv regression fails here with E2BIG on a real kernel.
    local big="${TMPDIR_BASE}/big.diff"
    {
        echo "diff --git a/big.txt b/big.txt"
        echo "--- a/big.txt"
        echo "+++ b/big.txt"
        echo "@@ -0,0 +1,4096 @@"
        local i
        for ((i = 0; i < 4096; i++)); do
            printf '+%063d\n' "$i"
        done
    } > "$big"

    export MOCK_GH_DIFF_FILE="$big"
    local exit_code
    exit_code=$(run_gemini_sync)
    unset MOCK_GH_DIFF_FILE

    assert_exit_code "large prompt review completes (exit 0)" "0" "$exit_code"
    local size
    size=$(wc -c < "${MOCK_REPO}/${PROMPT_REL}")
    if [[ "$size" -gt 200000 ]]; then
        echo "  PASS: prompt file is >200 KiB (${size} bytes)"
        PASS=$((PASS + 1))
    else
        echo "  FAIL: prompt file only ${size} bytes — test did not exercise the limit"
        FAIL=$((FAIL + 1))
    fi
    assert_contains "findings file has completion marker" "Review complete" \
        "$(tail -n 1 "${MOCK_REPO}/${FINDINGS_REL}")"

    teardown
}

test_agy_denial_is_failure() {
    echo "TEST: headless permission denial is reported as a failed review (#288)"
    setup

    export MOCK_AGY_DENY=1
    local exit_code
    exit_code=$(run_gemini_sync)
    unset MOCK_AGY_DENY

    assert_exit_code "denied review exits 3" "3" "$exit_code"
    local content
    content=$(cat "${MOCK_REPO}/${FINDINGS_REL}")
    assert_contains "findings file has failed marker" "Review failed" "$content"
    assert_not_contains "findings file has NO complete marker" "Review complete" "$content"
    assert_contains "findings file names the denied action" "RunCommand" "$content"
    assert_contains "findings file explains the denial" "auto-denied in headless mode" "$content"

    teardown
}

test_agy_timeout_is_failure() {
    echo "TEST: print-timeout expiry is a failed review, not a partial success (#288)"
    setup

    export MOCK_AGY_TIMEOUT=1
    local exit_code
    exit_code=$(run_gemini_sync)
    unset MOCK_AGY_TIMEOUT

    assert_exit_code "timed-out review exits 3" "3" "$exit_code"
    local content
    content=$(cat "${MOCK_REPO}/${FINDINGS_REL}")
    assert_contains "findings file has failed marker" "Review failed" "$content"
    assert_contains "findings file names the timeout" "print timeout" "$content"
    assert_not_contains "partial response is discarded" "partial text" "$content"

    teardown
}

test_agy_partial_denial_is_noted() {
    echo "TEST: a response with a denied action completes but carries a note"
    setup

    export MOCK_AGY_DENY_PARTIAL=1
    local exit_code
    exit_code=$(run_gemini_sync)
    unset MOCK_AGY_DENY_PARTIAL

    assert_exit_code "partial-denial review completes (exit 0)" "0" "$exit_code"
    local content
    content=$(cat "${MOCK_REPO}/${FINDINGS_REL}")
    assert_contains "response is kept" "Adversarial Code Review" "$content"
    assert_contains "denial note appended" "1 tool action\(s\) were denied in headless mode \(RunCommand\)" "$content"
    assert_contains "findings file has completion marker" "Review complete" "$content"

    teardown
}

test_diff_fetch_failure_is_marked() {
    echo "TEST: a failing diff fetch writes the error marker and exits 3 (not a bare set -e abort)"
    setup

    # gh: PR metadata fine, `gh pr diff` fails after emitting one line.
    cat > "${MOCK_BIN}/gh" << 'GH_EOF'
#!/usr/bin/env bash
if [[ "$1" == "pr" && "$2" == "view" ]]; then
    shift 2; shift
    [[ "${1:-}" == "-R" ]] && shift 2
    case "$2" in
        body) echo "Closes #42" ;;
        title) echo "Test PR" ;;
        url) echo "https://github.com/test/repo/pull/99" ;;
    esac
    exit 0
elif [[ "$1" == "pr" && "$2" == "diff" ]]; then
    echo "diff --git a/file.txt b/file.txt"
    echo "gh: connection reset" >&2
    exit 1
fi
exit 0
GH_EOF
    chmod +x "${MOCK_BIN}/gh"

    cd "${MOCK_REPO}"
    local exit_code=0 stderr
    stderr=$(PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 bash "${SCRIPT_UNDER_TEST}" \
        --pr 99  < /dev/null 2>&1 >/dev/null) || exit_code=$?

    assert_exit_code "failed diff fetch exits 3" "3" "$exit_code"
    assert_contains "error message names the diff retrieval" "Could not retrieve diff" "$stderr"
    local content
    content=$(cat "${MOCK_REPO}/${FINDINGS_REL}" 2>/dev/null || echo "MISSING")
    assert_contains "findings file carries the error marker" "Review error: failed to retrieve diff" "$content"

    teardown
}

test_agy_api_error_message_kept() {
    echo "TEST: a non-SUCCESS result keeps agy's error message, even as an object (#288)"
    setup

    export MOCK_AGY_ERROR="quota exceeded for model"
    local exit_code
    exit_code=$(run_gemini_sync)
    unset MOCK_AGY_ERROR

    assert_exit_code "API error exits 3" "3" "$exit_code"
    local content
    content=$(cat "${MOCK_REPO}/${FINDINGS_REL}")
    assert_contains "reason names the status" "result status ERROR" "$content"
    assert_contains "reason carries the error message" "quota exceeded for model" "$content"
    assert_contains "findings file has failed marker" "Review failed" "$content"

    teardown
}

test_agy_viewfile_denial_is_failure() {
    echo "TEST: a denied ViewFile read (#336's live signature) fails with the no-tools reason"
    setup

    local exit_code
    exit_code=$(MOCK_AGY_DENY=1 MOCK_AGY_DENY_ACTION=ViewFile run_gemini_sync)

    assert_exit_code "ViewFile-denied review exits 3" "3" "$exit_code"
    local content
    content=$(cat "${MOCK_REPO}/${FINDINGS_REL}")
    assert_contains "findings file has failed marker" "Review failed" "$content"
    assert_contains "findings file names the denied ViewFile action" "\(ViewFile\)" "$content"
    assert_contains "reason states the no-tools policy" \
        "must not call any tools \(file reads and shell commands are both denied headlessly\)" "$content"
    assert_not_contains "reason no longer blames shell commands alone" \
        "it must not run shell commands" "$content"

    teardown
}

# The live cutoff (captured in #336 round 2) puts the text in `.error` as
# a plain string; test_agy_live_cutoff_fixture replays it verbatim. The
# tests here keep exercising the object-valued `.error` and the stderr
# channel, which the helper also checks.
test_agy_output_token_cutoff_is_named() {
    echo "TEST: agy's output-token cutoff is a failure with a precise reason; the partial response is dropped (#336)"
    setup

    local phrase="response was cut off because it exceeded the output token limit"
    local exit_code content
    exit_code=$(MOCK_AGY_ERROR="The ${phrase}." MOCK_AGY_ERROR_RESPONSE="PARTIAL REVIEW TEXT" run_gemini_sync)
    assert_exit_code "cut-off review exits 3" "3" "$exit_code"
    content=$(cat "${MOCK_REPO}/${FINDINGS_REL}")
    assert_contains "reason names the cutoff precisely" \
        "response was cut off because it exceeded the output token limit \(status ERROR\); the prompt or response was too large" "$content"
    assert_not_contains "partial response is not recorded as a review" "PARTIAL REVIEW TEXT" "$content"
    assert_contains "findings file has failed marker" "Review failed" "$content"

    # Same condition reported on stderr instead, with an unrelated .error.
    exit_code=$(MOCK_AGY_ERROR="internal error" MOCK_AGY_ERROR_STDERR="[agy] Response was cut off: it exceeded the output token limit" \
        MOCK_AGY_ERROR_RESPONSE="PARTIAL REVIEW TEXT" run_gemini_sync)
    assert_exit_code "stderr-reported cutoff exits 3" "3" "$exit_code"
    content=$(cat "${MOCK_REPO}/${FINDINGS_REL}")
    assert_contains "stderr-reported cutoff is named" "exceeded the output token limit \(status ERROR\)" "$content"
    assert_not_contains "partial response is not recorded (stderr case)" "PARTIAL REVIEW TEXT" "$content"

    teardown
}

# Verbatim live capture from #336 round 2: `.error` is a plain string
# carrying agy's three-line cutoff message.
test_agy_live_cutoff_fixture() {
    echo "TEST: agy's live cutoff shape (string .error, verbatim) is named as a cutoff (#336)"
    setup

    local live_error=$'Your previous response was cut off because it exceeded the output token limit\nPlease continue from where you left off, keeping your response shorter\nRetries remaining: 3'
    local exit_code content
    exit_code=$(MOCK_AGY_ERROR="$live_error" MOCK_AGY_ERROR_STRING=1 \
        MOCK_AGY_ERROR_RESPONSE="PARTIAL REVIEW TEXT" run_gemini_sync)
    assert_exit_code "live cutoff exits 3" "3" "$exit_code"
    content=$(cat "${MOCK_REPO}/${FINDINGS_REL}")
    assert_contains "live cutoff reason is named" \
        "response was cut off because it exceeded the output token limit \(status ERROR\); the prompt or response was too large" "$content"
    assert_contains "agy's own error text is kept" "Retries remaining: 3" "$content"
    assert_not_contains "partial response is not recorded" "PARTIAL REVIEW TEXT" "$content"
    assert_contains "findings file has failed marker" "Review failed" "$content"

    teardown
}

test_agy_cutoff_phrase_in_response_not_misread() {
    echo "TEST: the cutoff phrase in the model's own response never relabels an unrelated error (#336)"
    setup

    # Status ERROR with an unrelated message, and a partial response that
    # quotes the cutoff phrase (as a review of a diff mentioning it would).
    # Only ERROR_MSG / stderr may drive the cutoff reason.
    local exit_code content
    exit_code=$(MOCK_AGY_ERROR="quota exceeded for model" \
        MOCK_AGY_ERROR_RESPONSE="| 1 | major | x.sh:3 | response was cut off because it exceeded the output token limit |" \
        run_gemini_sync)
    assert_exit_code "unrelated ERROR exits 3" "3" "$exit_code"
    content=$(cat "${MOCK_REPO}/${FINDINGS_REL}")
    assert_contains "reason is the generic status line with the real error" \
        "result status ERROR: .*quota exceeded for model" "$content"
    assert_not_contains "no cutoff reason line" "\(status ERROR\); the prompt or response was too large" "$content"
    assert_not_contains "the partial response is not recorded" "x.sh:3" "$content"

    teardown
}

test_agy_cutoff_halves_on_separate_lines_not_misread() {
    echo "TEST: 'cut off' and 'output token limit' on separate lines is not a cutoff (#336)"
    setup

    # An unrelated "connection was cut off" error plus a separate stderr
    # line naming the token limit: still a failure, but the generic one.
    local exit_code content
    exit_code=$(MOCK_AGY_ERROR="connection was cut off by the server" \
        MOCK_AGY_ERROR_STDERR="[agy] model output token limit: 8192" \
        MOCK_AGY_ERROR_RESPONSE="PARTIAL REVIEW TEXT" run_gemini_sync)
    assert_exit_code "split-phrase failure still exits 3" "3" "$exit_code"
    content=$(cat "${MOCK_REPO}/${FINDINGS_REL}")
    assert_contains "reason is the generic status line with the real error" \
        "result status ERROR: .*connection was cut off by the server" "$content"
    assert_not_contains "no cutoff reason line" "\(status ERROR\); the prompt or response was too large" "$content"
    assert_not_contains "the partial response is not recorded" "PARTIAL REVIEW TEXT" "$content"
    assert_contains "findings file has failed marker" "Review failed" "$content"

    teardown
}

test_agy_findings_truncated() {
    echo "TEST: a failed run never leaves the previous run's findings in place (#288)"
    setup

    mkdir -p "${MOCK_REPO}/.agent/work-plans/issue-42"
    echo "STALE FINDINGS FROM LAST RUN" > "${MOCK_REPO}/${FINDINGS_REL}"

    export MOCK_AGY_EXIT=7
    local exit_code
    exit_code=$(run_gemini_sync)
    unset MOCK_AGY_EXIT

    assert_exit_code "crashed agy exits 3" "3" "$exit_code"
    local content
    content=$(cat "${MOCK_REPO}/${FINDINGS_REL}")
    assert_not_contains "stale findings are gone" "STALE FINDINGS" "$content"
    assert_contains "reason names the exit status" "agy exited 7" "$content"
    assert_contains "reason carries agy stderr" "boom" "$content"
    assert_contains "findings file has failed marker" "Review failed" "$content"

    teardown
}

test_agy_no_temp_leak() {
    echo "TEST: the helper leaves no temp files behind on success or failure"
    setup

    # Point TMPDIR at a private dir so only the helper's mktemp lands there;
    # the mock repo and findings live under TMPDIR_BASE, outside it.
    local leak_dir="${TMPDIR_BASE}/leakcheck"
    mkdir -p "$leak_dir"

    cd "${MOCK_REPO}"
    TMPDIR="$leak_dir" PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 \
        bash "${SCRIPT_UNDER_TEST}" --pr 99  < /dev/null >/dev/null 2>&1 || true
    MOCK_AGY_DENY=1 TMPDIR="$leak_dir" PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 \
        bash "${SCRIPT_UNDER_TEST}" --pr 99  < /dev/null >/dev/null 2>&1 || true
    MOCK_AGY_EXIT=3 TMPDIR="$leak_dir" PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 \
        bash "${SCRIPT_UNDER_TEST}" --pr 99  < /dev/null >/dev/null 2>&1 || true

    local leftovers
    leftovers=$(ls -A "$leak_dir")
    assert_eq "no temp files left after success + denial + crash" "" "$leftovers"

    teardown
}

test_shared_temp_no_leak_on_early_abort() {
    echo "TEST: an abort among the shared mktemp calls leaves no temp files (#320)"
    setup

    local leak_dir="${TMPDIR_BASE}/leakcheck" fail_bin="${TMPDIR_BASE}/failbin"
    local real_mktemp
    real_mktemp=$(command -v mktemp)
    mkdir -p "$leak_dir" "$fail_bin"
    # A mktemp that fails for the template named in MOCK_MKTEMP_FAIL and
    # defers to the real one otherwise, so the script dies part-way through
    # creating its shared temp files.
    cat > "${fail_bin}/mktemp" << MKTEMP_EOF
#!/usr/bin/env bash
if [[ -n "\${MOCK_MKTEMP_FAIL:-}" && "\$*" == *"\${MOCK_MKTEMP_FAIL}"* ]]; then
    echo "mock mktemp: refusing \$*" >&2
    exit 1
fi
exec "${real_mktemp}" "\$@"
MKTEMP_EOF
    chmod +x "${fail_bin}/mktemp"

    cd "${MOCK_REPO}"
    local template exit_code stderr leftovers
    # The last shared mktemp fails after the prompt and diff files exist;
    # the first fails before any exists (cleanup must cope with empty paths).
    for template in cross-model-review-tmp. cross-model-review-prompt.; do
        rm -rf "$leak_dir"
        mkdir -p "$leak_dir"
        exit_code=0
        stderr=$(MOCK_MKTEMP_FAIL="$template" TMPDIR="$leak_dir" \
            PATH="${fail_bin}:${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 \
            bash "${SCRIPT_UNDER_TEST}" --pr 99 < /dev/null 2>&1 >/dev/null) || exit_code=$?
        if [[ "$exit_code" -ne 0 ]]; then
            echo "  PASS: run aborts when mktemp '${template}' fails"
            PASS=$((PASS + 1))
        else
            echo "  FAIL: run should abort when mktemp '${template}' fails"
            FAIL=$((FAIL + 1))
        fi
        assert_contains "the abort is the mocked mktemp (${template})" \
            "mock mktemp: refusing" "$stderr"
        assert_not_contains "cleanup does not trip on an unset path (${template})" \
            "unbound variable|cannot remove" "$stderr"
        leftovers=$(ls -A "$leak_dir")
        assert_eq "no shared temp files left after '${template}' fails" "" "$leftovers"
    done

    teardown
}

test_prompt_tool_use_guidance() {
    echo "TEST: no-tools + concise-output guidance is gemini-only (#288, #336)"
    setup

    run_gemini_sync >/dev/null
    local gemini_prompt
    gemini_prompt=$(cat "${MOCK_REPO}/${PROMPT_REL}")
    assert_contains "gemini prompt tells the model not to run commands" \
        "Do NOT run shell commands" "$gemini_prompt"
    assert_contains "gemini prompt tells the model to call no tools" \
        "Do not call any" "$gemini_prompt"
    assert_contains "gemini prompt says there is no file-reading tool" \
        "there is no file-reading tool in this session" "$gemini_prompt"
    assert_contains "gemini prompt asks for a concise answer" \
        "Keep the answer concise" "$gemini_prompt"
    assert_contains "gemini prompt keeps every finding, brevity per row" \
        "Report every finding you have; keep each row short" "$gemini_prompt"
    assert_contains "gemini prompt says not to restate the diff" \
        "Do not restate the" "$gemini_prompt"
    assert_contains "gemini prompt asks for file:line citations" \
        "cite file:line instead" "$gemini_prompt"
    assert_not_contains "gemini prompt drops the lost-review threat" \
        "whole review is lost" "$gemini_prompt"
    assert_not_contains "gemini prompt no longer invites file reads" \
        "You may read files" "$gemini_prompt"

    # codex: a mock that consumes stdin and prints something.
    cat > "${MOCK_BIN}/codex" << 'CODEX_EOF'
#!/usr/bin/env bash
cat > /dev/null
echo "codex ran"
CODEX_EOF
    chmod +x "${MOCK_BIN}/codex"
    cd "${MOCK_REPO}"
    PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 bash "${SCRIPT_UNDER_TEST}" \
        --pr 99  --agent codex < /dev/null >/dev/null 2>&1 || true
    local codex_prompt
    codex_prompt=$(cat "${MOCK_REPO}/.agent/work-plans/issue-42/review-codex-prompt.md")
    assert_not_contains "codex prompt has no tool-use paragraph" \
        "Do NOT run shell commands" "$codex_prompt"
    assert_not_contains "codex prompt has no no-tools instruction" \
        "Do not call any|no file-reading tool" "$codex_prompt"
    assert_not_contains "codex prompt has no concise-output instruction" \
        "Keep the answer concise" "$codex_prompt"

    make_mock_agent claude
    local out="${TMPDIR_BASE}/out.txt"
    run_agents "$out" "claude" >/dev/null
    local claude_prompt
    claude_prompt=$(cat "${MOCK_REPO}/.agent/work-plans/issue-42/review-claude-prompt.md")
    assert_contains "claude prompt was generated" "### Findings" "$claude_prompt"
    assert_not_contains "claude prompt has no tool-use paragraph" \
        "Do NOT run shell commands" "$claude_prompt"
    assert_not_contains "claude prompt has no no-tools instruction" \
        "Do not call any|no file-reading tool" "$claude_prompt"
    assert_not_contains "claude prompt has no concise-output instruction" \
        "Keep the answer concise" "$claude_prompt"

    teardown
}

test_work_plans_excluded_from_diff() {
    echo "TEST: .agent/work-plans/** sections are stripped from the embedded diff (#312)"
    setup

    local mixed="${TMPDIR_BASE}/mixed.diff"
    cat > "$mixed" << 'DIFF_EOF'
diff --git a/src/code.py b/src/code.py
--- a/src/code.py
+++ b/src/code.py
@@ -1 +1 @@
-old code
+new code
diff --git a/.agent/work-plans/issue-42/plan.md b/.agent/work-plans/issue-42/plan.md
new file mode 100644
--- /dev/null
+++ b/.agent/work-plans/issue-42/plan.md
@@ -0,0 +1 @@
+PLAN BOOKKEEPING
diff --git a/.agent/work-plans/issue-42/progress.md b/.agent/work-plans/issue-42/progress.md
--- a/.agent/work-plans/issue-42/progress.md
+++ b/.agent/work-plans/issue-42/progress.md
@@ -1 +1 @@
-PROGRESS OLD
+PROGRESS NEW
diff --git a/src/after.py b/src/after.py
--- a/src/after.py
+++ b/src/after.py
@@ -1 +1 @@
-x
+y
diff --git a/.agent/work-plans/issue-42/tool.sh b/src/tool.sh
similarity index 90%
rename from .agent/work-plans/issue-42/tool.sh
rename to src/tool.sh
--- a/.agent/work-plans/issue-42/tool.sh
+++ b/src/tool.sh
@@ -1 +1 @@
-RENAMED OUT old
+RENAMED OUT new
diff --git "a/.agent/work-plans/issue-42/odd name.md" "b/.agent/work-plans/issue-42/odd name.md"
--- "a/.agent/work-plans/issue-42/odd name.md"
+++ "b/.agent/work-plans/issue-42/odd name.md"
@@ -1 +1 @@
-q
+QUOTED BOOKKEEPING
DIFF_EOF

    export MOCK_GH_DIFF_FILE="$mixed"
    local exit_code
    exit_code=$(run_gemini_sync)
    unset MOCK_GH_DIFF_FILE

    assert_exit_code "mixed diff review completes" "0" "$exit_code"
    local prompt
    prompt=$(cat "${MOCK_REPO}/${PROMPT_REL}")
    assert_contains "code file before the bookkeeping is kept" "\+new code" "$prompt"
    assert_contains "code file after the bookkeeping is kept" "diff --git a/src/after.py" "$prompt"
    assert_not_contains "plan.md section is dropped" "PLAN BOOKKEEPING" "$prompt"
    assert_not_contains "progress.md section is dropped" "PROGRESS NEW" "$prompt"
    assert_contains "file renamed OUT of work-plans stays in review (b/ path decides)" \
        "RENAMED OUT new" "$prompt"
    assert_not_contains "quoted work-plans path is dropped" "QUOTED BOOKKEEPING" "$prompt"
    assert_not_contains "no work-plans post-image header survives" \
        " \"?b/.agent/work-plans" "$prompt"

    # All-bookkeeping diff: nothing to review.
    local only="${TMPDIR_BASE}/only.diff"
    cat > "$only" << 'DIFF_EOF'
diff --git a/.agent/work-plans/issue-42/plan.md b/.agent/work-plans/issue-42/plan.md
--- a/.agent/work-plans/issue-42/plan.md
+++ b/.agent/work-plans/issue-42/plan.md
@@ -1 +1 @@
-a
+b
DIFF_EOF
    export MOCK_GH_DIFF_FILE="$only"
    cd "${MOCK_REPO}"
    local stderr exit2=0
    stderr=$(PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 bash "${SCRIPT_UNDER_TEST}" \
        --pr 99  < /dev/null 2>&1 >/dev/null) || exit2=$?
    unset MOCK_GH_DIFF_FILE
    assert_exit_code "all-bookkeeping diff exits 3" "3" "$exit2"
    assert_contains "error names the work-plans exclusion" "work-plans" "$stderr"

    teardown
}

test_branch_mode_filter_survives_noprefix() {
    echo "TEST: branch mode filters work-plans even with diff.noprefix=true (#312)"
    setup

    # Feature branch off the mock repo's default branch with one code
    # file and one work-plans file; diff.noprefix set to defeat a naive
    # a/ b/ match.
    local base
    base=$(git -C "${MOCK_REPO}" branch --show-current)
    git -C "${MOCK_REPO}" checkout -q -b feature/issue-42
    mkdir -p "${MOCK_REPO}/src" "${MOCK_REPO}/.agent/work-plans/issue-42"
    echo "BRANCH CODE" > "${MOCK_REPO}/src/code.py"
    echo "BRANCH BOOKKEEPING" > "${MOCK_REPO}/.agent/work-plans/issue-42/plan.md"
    git -C "${MOCK_REPO}" add -A
    git -C "${MOCK_REPO}" -c user.name="Test" -c user.email="test@test" commit -q -m "feature"
    git -C "${MOCK_REPO}" config diff.noprefix true

    cd "${MOCK_REPO}"
    local exit_code=0
    PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 bash "${SCRIPT_UNDER_TEST}" \
        --branch "$base"  < /dev/null >/dev/null 2>&1 || exit_code=$?

    assert_exit_code "branch review completes" "0" "$exit_code"
    local prompt
    prompt=$(cat "${MOCK_REPO}/${PROMPT_REL}")
    assert_contains "code file kept" "BRANCH CODE" "$prompt"
    assert_not_contains "work-plans file dropped despite diff.noprefix" "BRANCH BOOKKEEPING" "$prompt"

    teardown
}

# ---- Diff fence tests (#320) ----
#
# The diff is wrapped in the same longest-backtick-run-plus-one outer
# fence as the plan context. A fixed ``` fence is closed early by a diff
# context line of one space plus three backticks (valid CommonMark: up to
# 3 spaces of indent), after which the rest of the diff — and the
# footer — is read as markdown instead of code.

# Assert the diff block of prompt file $1 is well-formed: the footer and
# a marker line inside the diff sit where they should.
assert_diff_fence_well_formed() {
    local prompt_file="$1" label="$2" inside_marker="$3"
    assert_eq "${label}: a line after the fence-shaped context line is still inside the diff fence" \
        "inside" "$(fence_state_at "$prompt_file" "$inside_marker" '^## Diff$')"
    assert_eq "${label}: the output-format footer is outside every fence" \
        "outside" "$(fence_state_at "$prompt_file" '^## Output Format$' '^## Diff$')"
}

test_diff_fence_context_line_pr_mode() {
    echo "TEST: a diff context line of space + \`\`\` cannot close the diff fence (PR mode, #320)"
    setup

    local diff="${TMPDIR_BASE}/fence-context.diff"
    printf '%s\n' \
        'diff --git a/README.md b/README.md' \
        '--- a/README.md' \
        '+++ b/README.md' \
        '@@ -1,4 +1,4 @@' \
        ' ```bash' \
        '-echo old' \
        '+echo new' \
        ' ```' \
        '+AFTER CONTEXT FENCE' > "$diff"

    local exit_code
    exit_code=$(MOCK_GH_DIFF_FILE="$diff" run_gemini_sync)
    assert_exit_code "review completes" "0" "$exit_code"

    local prompt_file="${MOCK_REPO}/${PROMPT_REL}"
    assert_contains "diff fence is 4 backticks (longest run 3, plus 1)" \
        '^````diff$' "$(cat "$prompt_file")"
    assert_diff_fence_well_formed "$prompt_file" "context line" '^[+]AFTER CONTEXT FENCE$'

    teardown
}

test_diff_fence_four_backtick_run_pr_mode() {
    echo "TEST: a 4-backtick run in the diff gets a 5-backtick diff fence (PR mode, #320)"
    setup

    # Also ends without a trailing newline: the closer must still land on
    # its own line.
    local diff="${TMPDIR_BASE}/fence-four.diff"
    printf '%s\n' \
        'diff --git a/doc.md b/doc.md' \
        '--- a/doc.md' \
        '+++ b/doc.md' \
        '@@ -1,3 +1,5 @@' \
        ' ````markdown' \
        ' ```' \
        '+INSIDE FOUR' \
        ' ```' > "$diff"
    printf '%s' ' ````' >> "$diff"

    local exit_code
    exit_code=$(MOCK_GH_DIFF_FILE="$diff" run_gemini_sync)
    assert_exit_code "review completes" "0" "$exit_code"

    local prompt_file="${MOCK_REPO}/${PROMPT_REL}"
    assert_contains "diff fence is 5 backticks" '^`````diff$' "$(cat "$prompt_file")"
    assert_contains "the closer is on its own line after an unterminated last line" \
        '^`````$' "$(cat "$prompt_file")"
    assert_diff_fence_well_formed "$prompt_file" "4-backtick run" '^[+]INSIDE FOUR$'

    teardown
}

test_diff_fence_branch_mode() {
    echo "TEST: branch mode fences a diff with fence-shaped context lines safely (#320)"
    setup

    local base
    base=$(git -C "${MOCK_REPO}" branch --show-current)
    printf '%s\n' 'Intro' '' '```bash' 'echo old' '```' '' 'Outro' > "${MOCK_REPO}/guide.md"
    git -C "${MOCK_REPO}" add guide.md
    git -C "${MOCK_REPO}" -c user.name="Test" -c user.email="test@test" commit -q -m "base doc"
    git -C "${MOCK_REPO}" checkout -q -b feature/issue-42
    printf '%s\n' 'Intro' '' '```bash' 'echo new' '```' '' '````' 'BRANCH FOUR RUN' '````' 'Outro' \
        > "${MOCK_REPO}/guide.md"
    git -C "${MOCK_REPO}" -c user.name="Test" -c user.email="test@test" commit -q -am "feature"

    cd "${MOCK_REPO}"
    local exit_code=0
    PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 bash "${SCRIPT_UNDER_TEST}" \
        --branch "$base" < /dev/null >/dev/null 2>&1 || exit_code=$?
    assert_exit_code "branch review completes" "0" "$exit_code"

    local prompt_file="${MOCK_REPO}/${PROMPT_REL}"
    assert_contains "the diff carries the space + backtick context line" '^ ```$' "$(cat "$prompt_file")"
    assert_contains "diff fence is 5 backticks" '^`````diff$' "$(cat "$prompt_file")"
    assert_diff_fence_well_formed "$prompt_file" "branch mode" '^[+]BRANCH FOUR RUN$'

    teardown
}

# ---- Plan-context tests (#320) ----
#
# The plan's `## Approach` section is re-admitted to the prompt as
# labelled context outside the diff fence (the diff itself still excludes
# `.agent/work-plans/**`, #312). Every fixture plan.md is written into the
# *resolved* WORK_PLANS_DIR — the same directory the review-<agent>-prompt.md
# files land in — because a plan written anywhere else would make the
# "present" case fail and the "absent" cases pass vacuously.
PLAN_REL=".agent/work-plans/issue-42/plan.md"

# The extractor (_plan_approach.py) needs markdown-it-py. The script tries
# the workspace .venv's python3 (the main checkout's, found through git's
# common dir) and then python3 on PATH; the tests that check what it
# extracts need one of them to have the library. Locally, without it they
# are skipped with a reason rather than failed: the script then omits the
# Plan Context block by design (test_plan_context_parser_unavailable).
# Under CI (GitHub sets CI=true) a missing parser is a failure instead:
# pre-commit hides a passing hook's output, so a skip there would silently
# drop every extractor test.
PLAN_PARSER_PYTHON=""
plan_parser_common=$(git -C "$SCRIPT_DIR" rev-parse --path-format=absolute --git-common-dir 2>/dev/null || true)
for plan_parser_py in "${plan_parser_common%/.git}/.venv/bin/python3" "$(command -v python3 || true)"; do
    if [[ -n "$plan_parser_common" || "$plan_parser_py" != "/.venv/bin/python3" ]] \
        && [[ -x "$plan_parser_py" ]] && "$plan_parser_py" -c 'import markdown_it' 2>/dev/null; then
        PLAN_PARSER_PYTHON="$plan_parser_py"
        break
    fi
done
unset plan_parser_common plan_parser_py

require_plan_parser() {
    [[ -n "$PLAN_PARSER_PYTHON" ]] && return 0
    case "${CI:-}" in
        "" | false | 0)
            echo "  SKIP: markdown-it-py is importable by neither the workspace .venv python3 nor python3 (run 'make setup')"
            ;;
        *)
            echo "  FAIL: markdown-it-py is importable by neither the workspace .venv python3 nor python3, and CI is set, so the extractor tests must run (install requirements.txt)"
            FAIL=$((FAIL + 1))
            ;;
    esac
    return 1
}

# Write $2 as the fixture plan.md at the resolved work-plans dir ($1).
write_plan_fixture() {
    mkdir -p "$(dirname "$1")"
    printf '%s\n' "$2" > "$1"
}

# Echo the `## Plan Context` block of prompt file $1, up to (excluding)
# the `## Output Format` footer.
plan_context_block() {
    awk '/^## Plan Context$/ { in_block = 1 }
         in_block && /^## Output Format$/ { exit }
         in_block { print }' "$1"
}

# Walk prompt file $1 from the first line matching regex $3 (default
# `## Plan Context`) with CommonMark fence rules (opener: 0-3 spaces, 3+
# backticks or tildes, a backtick opener's info string holds no backtick;
# closer: same char, run >= opener's, whitespace only after; a trailing CR
# is a line ending, not content) and print "inside" or "outside" for the
# first line matching regex $2 after that point — "missing" if none does.
# Written independently of the script, so it is an oracle, not a copy.
fence_state_at() {
    awk -v target="$2" -v start="${3:-^## Plan Context$}" '
        { sub(/\r$/, "") }
        !go && $0 ~ start { go = 1; next }
        !go { next }
        $0 ~ target { print (open_n ? "inside" : "outside"); found = 1; exit }
        {
            ind = 0
            while (substr($0, ind + 1, 1) == " ") ind++
            if (ind > 3) next
            c = substr($0, ind + 1, 1)
            if (c != "`" && c != "~") next
            n = 0
            while (substr($0, ind + n + 1, 1) == c) n++
            if (n < 3) next
            rest = substr($0, ind + n + 1)
            if (!open_n) {
                if (c == "`" && rest ~ /`/) next
                open_c = c; open_n = n
            } else if (c == open_c && n >= open_n && rest ~ /^[ \t]*$/) {
                open_n = 0
            }
        }
        END { if (!found) print "missing" }
    ' "$1"
}

footer_fence_state() {
    fence_state_at "$1" '^## Output Format$'
}

# Write an Approach of $2 filler lines followed by the text in $3 (which
# should straddle the 200-line cap) to plan.md, then a Files section.
write_straddling_plan() {
    local path="$1" filler="$2" tail="$3" i
    mkdir -p "$(dirname "$path")"
    {
        printf '# Plan: straddle\n\n## Approach\n\n'
        for ((i = 1; i <= filler; i++)); do printf 'FILLER %d\n' "$i"; done
        printf '%s\n' "$tail"
        for ((i = 1; i <= 100; i++)); do printf 'PAST CAP %d\n' "$i"; done
        printf '\n## Files to Change\n\nTAIL SECTION\n'
    } > "$path"
}

test_plan_context_present() {
    echo "TEST: the plan's ## Approach is appended as labelled plan context (#320)"
    require_plan_parser || return 0
    setup

    write_plan_fixture "${MOCK_REPO}/${PLAN_REL}" "# Plan: something

## Context

CONTEXT SECTION BODY

## Approach

1. APPROACH STEP ONE
2. APPROACH STEP TWO

## Files to Change

FILES SECTION BODY"

    local exit_code
    exit_code=$(run_gemini_sync)
    assert_exit_code "review completes with a plan present" "0" "$exit_code"

    local prompt
    prompt=$(cat "${MOCK_REPO}/${PROMPT_REL}")
    assert_contains "plan context heading is present" "^## Plan Context$" "$prompt"
    assert_contains "Approach body is included" "APPROACH STEP ONE" "$prompt"
    assert_contains "Approach body runs to the section end" "APPROACH STEP TWO" "$prompt"
    assert_contains "framed as context, not as the subject of review" \
        "do not review the" "$prompt"
    assert_contains "reviewer is told to flag divergences" "divergences" "$prompt"
    assert_not_contains "sections before Approach are not included" \
        "CONTEXT SECTION BODY" "$prompt"
    assert_not_contains "sections after Approach are not included" \
        "FILES SECTION BODY" "$prompt"
    assert_not_contains "no truncation marker on a short Approach" \
        "truncated: " "$prompt"
    assert_contains "gemini tool-use footer says excluded from the diff" \
        "excluded \*\*from the diff\*\*" "$prompt"

    teardown
}

test_plan_context_absent_no_plan() {
    echo "TEST: no plan.md => no ## Plan Context section at all (#320)"
    setup

    local exit_code
    exit_code=$(run_gemini_sync)
    assert_exit_code "review completes without a plan" "0" "$exit_code"

    local prompt
    prompt=$(cat "${MOCK_REPO}/${PROMPT_REL}")
    assert_not_contains "no plan context heading" "^## Plan Context$" "$prompt"
    assert_contains "output format footer still present" "^## Output Format$" "$prompt"

    teardown
}

test_plan_context_absent_no_approach_section() {
    echo "TEST: plan.md with no (or empty) ## Approach => section omitted, not emptied (#320)"
    setup

    # (a) No `## Approach` heading at all.
    write_plan_fixture "${MOCK_REPO}/${PLAN_REL}" "# Plan: something

## Context

NO APPROACH HERE

## Files to Change

FILES ONLY"

    local exit_code
    exit_code=$(run_gemini_sync)
    assert_exit_code "review completes with an Approach-less plan" "0" "$exit_code"

    local prompt
    prompt=$(cat "${MOCK_REPO}/${PROMPT_REL}")
    assert_not_contains "no plan context heading without ## Approach" \
        "^## Plan Context$" "$prompt"
    assert_not_contains "never falls back to the rest of the plan" \
        "NO APPROACH HERE" "$prompt"

    # (b) `## Approach` present but empty (whitespace only).
    write_plan_fixture "${MOCK_REPO}/${PLAN_REL}" "# Plan: something

## Approach



## Files to Change

EMPTY APPROACH PLAN"

    exit_code=$(run_gemini_sync)
    assert_exit_code "review completes with an empty Approach" "0" "$exit_code"
    prompt=$(cat "${MOCK_REPO}/${PROMPT_REL}")
    assert_not_contains "empty Approach emits no heading" "^## Plan Context$" "$prompt"
    assert_not_contains "empty Approach pulls in no later section" \
        "EMPTY APPROACH PLAN" "$prompt"

    teardown
}

test_plan_context_truncated() {
    echo "TEST: an over-long ## Approach is capped at 200 lines with a visible marker (#320)"
    require_plan_parser || return 0
    setup

    local body="" i
    for ((i = 1; i <= 250; i++)); do
        body+="APPROACH LINE ${i}"$'\n'
    done
    write_plan_fixture "${MOCK_REPO}/${PLAN_REL}" "# Plan: long

## Approach

${body}
## Files to Change

TAIL SECTION"

    local exit_code
    exit_code=$(run_gemini_sync)
    assert_exit_code "review completes with a long plan" "0" "$exit_code"

    local prompt
    prompt=$(cat "${MOCK_REPO}/${PROMPT_REL}")
    assert_contains "plan context heading is present" "^## Plan Context$" "$prompt"
    assert_contains "an early Approach line survives" "^APPROACH LINE 1$" "$prompt"
    # The section body starts with the blank line after the heading, so
    # line 200 of the extraction is APPROACH LINE 199.
    assert_contains "the line at the cap survives" "^APPROACH LINE 199$" "$prompt"
    assert_not_contains "the line past the cap is cut" "^APPROACH LINE 201$" "$prompt"
    assert_contains "truncation is marked in the prompt" "truncated: [0-9]+ more lines" "$prompt"
    assert_not_contains "the following section is still excluded" "TAIL SECTION" "$prompt"

    teardown
}

test_plan_context_no_progress() {
    echo "TEST: --no-progress omits plan context even when a plan.md exists (#320)"
    setup

    # --no-progress alone gets a fresh mktemp -d where no plan.md can
    # exist; the only combination that can exercise the guard is
    # --no-progress with an explicit --work-plans-dir holding a plan.
    local wp_dir="${TMPDIR_BASE}/explicit-work-plans"
    write_plan_fixture "${wp_dir}/plan.md" "# Plan: skipped

## Approach

NO PROGRESS APPROACH BODY"

    cd "${MOCK_REPO}"
    local exit_code=0
    PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 bash "${SCRIPT_UNDER_TEST}" \
        --pr 99 --no-progress --work-plans-dir "$wp_dir" \
        < /dev/null >/dev/null 2>&1 || exit_code=$?

    assert_exit_code "review completes under --no-progress" "0" "$exit_code"
    local prompt
    prompt=$(cat "${wp_dir}/review-gemini-prompt.md")
    assert_not_contains "no plan context heading under --no-progress" \
        "^## Plan Context$" "$prompt"
    assert_not_contains "plan body never reaches the prompt" \
        "NO PROGRESS APPROACH BODY" "$prompt"

    teardown
}

test_plan_context_large_approach() {
    echo "TEST: an Approach far larger than the pipe buffer does not kill the script (#320)"
    require_plan_parser || return 0
    setup

    # ~400 KB of Approach: well past the 64 KiB pipe buffer. A
    # `printf | head` implementation under `set -o pipefail` takes
    # SIGPIPE here and aborts the whole run with exit 141 before any
    # agent is dispatched.
    local big="${TMPDIR_BASE}/big-approach.md"
    {
        printf '# Plan: huge\n\n## Approach\n\n'
        local i
        for ((i = 1; i <= 5000; i++)); do
            printf 'APPROACH LINE %d %s\n' "$i" \
                "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
        done
        printf '\n## Files to Change\n\nTAIL SECTION\n'
    } > "$big"
    mkdir -p "$(dirname "${MOCK_REPO}/${PLAN_REL}")"
    cp "$big" "${MOCK_REPO}/${PLAN_REL}"

    local exit_code
    exit_code=$(run_gemini_sync)
    assert_exit_code "review survives a >64 KiB Approach (no SIGPIPE, no 141)" "0" "$exit_code"

    local prompt
    prompt=$(cat "${MOCK_REPO}/${PROMPT_REL}")
    assert_contains "plan context heading is present" "^## Plan Context$" "$prompt"
    assert_contains "truncation is marked" "truncated: [0-9]+ more lines" "$prompt"
    assert_contains "the output-format footer still follows it" "^## Output Format$" "$prompt"
    assert_not_contains "the cap still holds" "^APPROACH LINE 1000 " "$prompt"

    teardown
}

# Every outer-fence test ends with the same three checks: the footer and
# (when present) the truncation marker are outside every fence, and the
# framing text sits outside the outer fence.
assert_plan_context_well_formed() {
    local prompt_file="$1" label="$2"
    assert_eq "${label}: the output-format footer is outside every fence" \
        "outside" "$(footer_fence_state "$prompt_file")"
    assert_eq "${label}: the framing text is outside the outer fence" \
        "outside" "$(fence_state_at "$prompt_file" '^plan itself[.]$')"
    if grep -q 'truncated: ' "$prompt_file"; then
        assert_eq "${label}: the truncation marker is outside every fence" \
            "outside" "$(fence_state_at "$prompt_file" 'truncated: [0-9]+ more lines')"
    fi
}

test_plan_context_outer_fence_basic() {
    echo "TEST: the plan excerpt is wrapped in one outer fence longer than any run inside (#320)"
    require_plan_parser || return 0
    setup

    write_plan_fixture "${MOCK_REPO}/${PLAN_REL}" '# Plan: fenced

## Approach

Run it like this:

```bash
echo FENCED COMMAND
```

Then stop.

## Files to Change

TAIL SECTION'

    local exit_code
    exit_code=$(run_gemini_sync)
    assert_exit_code "review completes with a fenced Approach" "0" "$exit_code"

    local prompt_file="${MOCK_REPO}/${PROMPT_REL}" block
    block=$(plan_context_block "$prompt_file")
    assert_contains "outer opener is 4 backticks (longest inner run 3, plus 1)" \
        '^````markdown$' "$block"
    assert_contains "outer closer matches the opener" '^````$' "$block"
    assert_contains "fenced content is carried through unchanged" '^```bash$' "$block"
    assert_eq "the content sits inside the outer fence" \
        "inside" "$(fence_state_at "$prompt_file" '^Then stop\.$')"
    assert_plan_context_well_formed "$prompt_file" "basic"

    teardown
}

test_plan_context_outer_fence_no_backticks() {
    echo "TEST: an excerpt with no backticks still gets a 3-backtick outer fence (#320)"
    require_plan_parser || return 0
    setup

    write_plan_fixture "${MOCK_REPO}/${PLAN_REL}" '## Approach

PLAIN APPROACH'

    local exit_code
    exit_code=$(run_gemini_sync)
    assert_exit_code "review completes" "0" "$exit_code"
    local block
    block=$(plan_context_block "${MOCK_REPO}/${PROMPT_REL}")
    assert_contains "outer opener is the 3-backtick minimum" '^```markdown$' "$block"
    assert_plan_context_well_formed "${MOCK_REPO}/${PROMPT_REL}" "no backticks"

    teardown
}

test_plan_context_outer_fence_info_string_inside() {
    echo "TEST: a \`\`\`js line inside a plan fence cannot break the outer fence (#320)"
    require_plan_parser || return 0
    setup

    write_plan_fixture "${MOCK_REPO}/${PLAN_REL}" '# Plan: info string

## Approach

```bash
echo BEFORE
```js
echo AFTER
```

Prose after the fence.

## Files to Change

TAIL SECTION'

    local exit_code
    exit_code=$(run_gemini_sync)
    assert_exit_code "review completes" "0" "$exit_code"
    assert_contains "prose after the inner fence is kept" "Prose after the fence" \
        "$(plan_context_block "${MOCK_REPO}/${PROMPT_REL}")"
    assert_plan_context_well_formed "${MOCK_REPO}/${PROMPT_REL}" "info string"

    teardown
}

test_plan_context_outer_fence_four_backticks() {
    echo "TEST: a 4-backtick plan fence holding a 3-backtick line gets a 5-backtick outer fence (#320)"
    require_plan_parser || return 0
    setup

    # Extraction line 1 is the blank after the heading, so fillers take
    # lines 2-196 and the tail 197-200: the cap cuts the 4-backtick fence
    # open, just after a 3-backtick line inside it.
    write_straddling_plan "${MOCK_REPO}/${PLAN_REL}" 195 '````markdown
```bash
echo INNER
```'

    local exit_code
    exit_code=$(run_gemini_sync)
    assert_exit_code "review completes" "0" "$exit_code"

    local block
    block=$(plan_context_block "${MOCK_REPO}/${PROMPT_REL}")
    assert_contains "the cut happened" "truncated: [0-9]+ more lines" "$block"
    assert_contains "outer fence is 5 backticks" '^`````markdown$' "$block"
    assert_plan_context_well_formed "${MOCK_REPO}/${PROMPT_REL}" "4-backtick"

    teardown
}

test_plan_context_outer_fence_list_item_cut() {
    echo "TEST: a list-item fence cut by the 200-line cap cannot swallow the footer (#320)"
    require_plan_parser || return 0
    setup

    # An indented fence inside a list item, opened on line 199 and cut.
    write_straddling_plan "${MOCK_REPO}/${PLAN_REL}" 196 '1. Run the migration:
   ```bash
   echo LIST ITEM CODE'

    local exit_code
    exit_code=$(run_gemini_sync)
    assert_exit_code "review completes" "0" "$exit_code"

    local block
    block=$(plan_context_block "${MOCK_REPO}/${PROMPT_REL}")
    assert_contains "the cut happened" "truncated: [0-9]+ more lines" "$block"
    assert_contains "the list-item fence line is kept" '^   ```bash$' "$block"
    assert_plan_context_well_formed "${MOCK_REPO}/${PROMPT_REL}" "list item"

    teardown
}

test_plan_context_outer_fence_indented_code() {
    echo "TEST: a 4-space-indented backtick run (indented code, not a fence) is harmless (#320)"
    require_plan_parser || return 0
    setup

    write_plan_fixture "${MOCK_REPO}/${PLAN_REL}" '## Approach

Indented code that shows a fence marker:

    ```
    not a fence

After the indented block.

## Files to Change

TAIL SECTION'

    local exit_code
    exit_code=$(run_gemini_sync)
    assert_exit_code "review completes" "0" "$exit_code"
    local block
    block=$(plan_context_block "${MOCK_REPO}/${PROMPT_REL}")
    assert_contains "text after the indented block is kept" "After the indented block" "$block"
    assert_not_contains "the next section is still excluded" "TAIL SECTION" "$block"
    assert_plan_context_well_formed "${MOCK_REPO}/${PROMPT_REL}" "indented code"

    teardown
}

test_plan_context_outer_fence_crlf() {
    echo "TEST: a CRLF plan keeps the prompt well-formed and ends at the next section (#320)"
    require_plan_parser || return 0
    setup

    mkdir -p "$(dirname "${MOCK_REPO}/${PLAN_REL}")"
    printf '# Plan: crlf\r\n\r\n## Approach\r\n\r\n```bash\r\necho CRLF CODE\r\n```\r\n\r\nCRLF PROSE\r\n\r\n## Files to Change\r\n\r\nTAIL SECTION\r\n' \
        > "${MOCK_REPO}/${PLAN_REL}"

    local exit_code
    exit_code=$(run_gemini_sync)
    assert_exit_code "review completes" "0" "$exit_code"

    local block
    block=$(plan_context_block "${MOCK_REPO}/${PROMPT_REL}")
    assert_contains "CRLF prose is kept" "CRLF PROSE" "$block"
    assert_not_contains "the next section is still excluded" "TAIL SECTION" "$block"
    assert_eq "the outer closer carries no CR" "1" \
        "$(grep -c $'^````$' <<< "$block" || true)"
    assert_plan_context_well_formed "${MOCK_REPO}/${PROMPT_REL}" "CRLF"

    teardown
}

test_plan_context_outer_fence_longer_than_inner_run() {
    echo "TEST: a 5-backtick run inside the excerpt gets a 6-backtick outer fence (#320)"
    require_plan_parser || return 0
    setup

    write_plan_fixture "${MOCK_REPO}/${PLAN_REL}" '## Approach

`````text
five-backtick fence
`````

Inline run ````` mid-line too.

## Files to Change

TAIL SECTION'

    local exit_code
    exit_code=$(run_gemini_sync)
    assert_exit_code "review completes" "0" "$exit_code"
    local block
    block=$(plan_context_block "${MOCK_REPO}/${PROMPT_REL}")
    assert_contains "outer opener is 6 backticks" '^``````markdown$' "$block"
    assert_contains "outer closer is 6 backticks" '^``````$' "$block"
    assert_plan_context_well_formed "${MOCK_REPO}/${PROMPT_REL}" "5-backtick run"

    teardown
}

test_plan_context_extractor_ignores_boundaries_in_fences() {
    echo "TEST: a # comment or --- inside a plan fence does not end the Approach (#320)"
    require_plan_parser || return 0
    setup

    write_plan_fixture "${MOCK_REPO}/${PLAN_REL}" '## Approach

```bash
# a shell comment, not a heading
echo AFTER COMMENT
```

~~~yaml
---
key: AFTER RULE
~~~

```js
## not a heading either
```

PROSE AFTER FENCES

## Files to Change

TAIL SECTION'

    local exit_code
    exit_code=$(run_gemini_sync)
    assert_exit_code "review completes" "0" "$exit_code"
    local block
    block=$(plan_context_block "${MOCK_REPO}/${PROMPT_REL}")
    assert_contains "content after a # line in a fence is kept" "AFTER COMMENT" "$block"
    assert_contains "content after a --- line in a fence is kept" "AFTER RULE" "$block"
    assert_contains "prose after the fences is kept" "PROSE AFTER FENCES" "$block"
    assert_not_contains "the next real section still ends it" "TAIL SECTION" "$block"
    assert_plan_context_well_formed "${MOCK_REPO}/${PROMPT_REL}" "boundaries in fences"

    teardown
}

test_plan_context_extractor_long_rule() {
    echo "TEST: a thematic break of four or more dashes ends the Approach (#320)"
    require_plan_parser || return 0
    setup

    write_plan_fixture "${MOCK_REPO}/${PLAN_REL}" '## Approach

APPROACH BEFORE LONG RULE

-----

LONG RULE SECTION BODY'

    local exit_code
    exit_code=$(run_gemini_sync)
    assert_exit_code "review completes" "0" "$exit_code"
    local block
    block=$(plan_context_block "${MOCK_REPO}/${PROMPT_REL}")
    assert_contains "Approach body is included" "APPROACH BEFORE LONG RULE" "$block"
    assert_not_contains "content after a ----- rule does not leak in" \
        "LONG RULE SECTION BODY" "$block"

    teardown
}

test_plan_context_extractor_star_underscore_rules() {
    echo "TEST: ***, * * *, ___ and indented rules end the Approach (#320)"
    require_plan_parser || return 0
    setup

    local rule exit_code block
    for rule in '***' '* * *' '___' '_ _ _' '   ***' '  - - -' '*****'; do
        write_plan_fixture "${MOCK_REPO}/${PLAN_REL}" "## Approach

**APPROACH BOLD LINE** is not a rule
__ALSO NOT A RULE__

${rule}

AFTER RULE BODY"

        exit_code=$(run_gemini_sync)
        assert_exit_code "review completes with rule '${rule}'" "0" "$exit_code"
        block=$(plan_context_block "${MOCK_REPO}/${PROMPT_REL}")
        assert_contains "bold text before '${rule}' is kept" "APPROACH BOLD LINE" "$block"
        assert_contains "underscore text before '${rule}' is kept" "ALSO NOT A RULE" "$block"
        assert_not_contains "content after '${rule}' does not leak in" \
            "AFTER RULE BODY" "$block"
    done

    # Four leading spaces is indented code, not a rule: the Approach goes on.
    write_plan_fixture "${MOCK_REPO}/${PLAN_REL}" "## Approach

APPROACH START

    ***

STILL APPROACH

## Files to Change

TAIL SECTION"
    exit_code=$(run_gemini_sync)
    assert_exit_code "review completes with an indented-code ***" "0" "$exit_code"
    block=$(plan_context_block "${MOCK_REPO}/${PROMPT_REL}")
    assert_contains "a 4-space-indented *** does not end the Approach" \
        "STILL APPROACH" "$block"
    assert_not_contains "the next heading still ends it" "TAIL SECTION" "$block"

    teardown
}

test_plan_context_extractor_unclosed_fence() {
    echo "TEST: an unclosed plan fence cuts at the first boundary seen inside it (#320)"
    require_plan_parser || return 0
    setup

    # The fence opened in the Approach never closes, so every later line is
    # "inside" it. The Approach must still stop at the first boundary-shaped
    # line rather than running to EOF and pulling the later sections in.
    write_plan_fixture "${MOCK_REPO}/${PLAN_REL}" '## Approach

APPROACH BEFORE FENCE

```bash
echo INSIDE UNCLOSED FENCE

## Files to Change

LATER SECTION BODY

## Verification

LAST SECTION BODY'

    local exit_code
    exit_code=$(run_gemini_sync)
    assert_exit_code "review completes" "0" "$exit_code"
    local block
    block=$(plan_context_block "${MOCK_REPO}/${PROMPT_REL}")
    assert_contains "Approach body before the fence is kept" "APPROACH BEFORE FENCE" "$block"
    assert_contains "fence body before the boundary is kept" "INSIDE UNCLOSED FENCE" "$block"
    assert_not_contains "the later section does not leak in" "LATER SECTION BODY" "$block"
    assert_not_contains "the last section does not leak in" "LAST SECTION BODY" "$block"
    assert_not_contains "the boundary heading itself is not included" \
        "^## Files to Change$" "$block"
    assert_plan_context_well_formed "${MOCK_REPO}/${PROMPT_REL}" "unclosed fence"

    teardown
}

test_plan_context_stops_at_h1_or_rule() {
    echo "TEST: the Approach extractor stops at an H1 or a thematic break (#320)"
    require_plan_parser || return 0
    setup

    # (a) H1 after Approach.
    write_plan_fixture "${MOCK_REPO}/${PLAN_REL}" '## Approach

APPROACH BODY ONE

# Appendix

H1 SECTION BODY'

    local exit_code
    exit_code=$(run_gemini_sync)
    assert_exit_code "review completes with an H1 after Approach" "0" "$exit_code"
    local block
    block=$(plan_context_block "${MOCK_REPO}/${PROMPT_REL}")
    assert_contains "Approach body is included" "APPROACH BODY ONE" "$block"
    assert_not_contains "content after an H1 does not leak in" "H1 SECTION BODY" "$block"

    # (b) Thematic break after Approach.
    write_plan_fixture "${MOCK_REPO}/${PLAN_REL}" '## Approach

APPROACH BODY TWO

---

RULE SECTION BODY'

    exit_code=$(run_gemini_sync)
    assert_exit_code "review completes with a rule after Approach" "0" "$exit_code"
    block=$(plan_context_block "${MOCK_REPO}/${PROMPT_REL}")
    assert_contains "Approach body is included" "APPROACH BODY TWO" "$block"
    assert_not_contains "content after a thematic break does not leak in" \
        "RULE SECTION BODY" "$block"

    teardown
}

test_plan_context_extractor_fence_after_closed_fence() {
    echo "TEST: an unclosed fence after a closed one keeps the text between them (#320 round 6)"
    require_plan_parser || return 0
    setup

    # The awk extractor remembered the `# comment` inside the CLOSED fence
    # as the cut point and never reset it when that fence closed, so the
    # later unclosed fence truncated the Approach back to STEP A.
    write_plan_fixture "${MOCK_REPO}/${PLAN_REL}" '## Approach

STEP A

```sh
# comment
```

STEP B

```bash
echo UNCLOSED

## Files to Change

LATER SECTION BODY'

    local exit_code
    exit_code=$(run_gemini_sync)
    assert_exit_code "review completes" "0" "$exit_code"
    local block
    block=$(plan_context_block "${MOCK_REPO}/${PROMPT_REL}")
    assert_contains "text before the closed fence is kept" "STEP A" "$block"
    assert_contains "text between the closed and the unclosed fence is kept" "STEP B" "$block"
    assert_contains "the unclosed fence body before the boundary is kept" "echo UNCLOSED" "$block"
    assert_not_contains "the later section does not leak in" "LATER SECTION BODY" "$block"
    assert_plan_context_well_formed "${MOCK_REPO}/${PROMPT_REL}" "fence after closed fence"

    teardown
}

test_plan_context_extractor_approach_in_earlier_fence() {
    echo "TEST: a ## Approach line inside an earlier fenced example is not the section (#320 round 6)"
    require_plan_parser || return 0
    setup

    write_plan_fixture "${MOCK_REPO}/${PLAN_REL}" '# Plan: example

## Context

```md
## Approach
EXAMPLE ONLY
```

## Approach

REAL APPROACH BODY

## Files to Change

TAIL SECTION'

    local exit_code
    exit_code=$(run_gemini_sync)
    assert_exit_code "review completes" "0" "$exit_code"
    local block
    block=$(plan_context_block "${MOCK_REPO}/${PROMPT_REL}")
    assert_contains "the real Approach is extracted" "REAL APPROACH BODY" "$block"
    assert_not_contains "the fenced example is not taken as the Approach" "EXAMPLE ONLY" "$block"
    assert_not_contains "the next section is still excluded" "TAIL SECTION" "$block"

    teardown
}

test_plan_context_extractor_unclosed_fence_before_approach() {
    echo "TEST: an unclosed fence in an earlier section does not hide the ## Approach heading (#320 round 7)"
    require_plan_parser || return 0
    setup

    # Per CommonMark the unclosed fence in Context swallows the rest of the
    # plan, heading included; the extractor would then report "no section"
    # and the Plan Context block would vanish without a warning.
    write_plan_fixture "${MOCK_REPO}/${PLAN_REL}" '## Context

```sh
open

## Approach

REAL

## Files to Change

TAIL SECTION'

    local exit_code
    exit_code=$(run_gemini_sync)
    assert_exit_code "review completes" "0" "$exit_code"
    local block
    block=$(plan_context_block "${MOCK_REPO}/${PROMPT_REL}")
    assert_contains "the Approach after the unclosed fence is extracted" "^REAL$" "$block"
    assert_not_contains "the unclosed fence body is not taken as the Approach" "^open$" "$block"
    assert_not_contains "the next section is still excluded" "TAIL SECTION" "$block"
    assert_plan_context_well_formed "${MOCK_REPO}/${PROMPT_REL}" "unclosed fence before approach"

    teardown
}

test_plan_context_extractor_indented_heading() {
    echo "TEST: an ATX heading indented 1-3 spaces ends the Approach (#320 round 6)"
    require_plan_parser || return 0
    setup

    local indent exit_code block
    for indent in ' ' '  ' '   '; do
        write_plan_fixture "${MOCK_REPO}/${PLAN_REL}" "## Approach

APPROACH BODY

${indent}## Files to Change

INDENTED NEXT SECTION"

        exit_code=$(run_gemini_sync)
        assert_exit_code "review completes (${#indent}-space heading)" "0" "$exit_code"
        block=$(plan_context_block "${MOCK_REPO}/${PROMPT_REL}")
        assert_contains "Approach body is kept (${#indent}-space heading)" "APPROACH BODY" "$block"
        assert_not_contains "the section after a ${#indent}-space-indented heading does not leak in" \
            "INDENTED NEXT SECTION" "$block"
    done

    teardown
}

test_plan_context_extractor_setext_headings() {
    echo "TEST: setext H1 (===) and H2 (---) headings end the Approach, title line included (#320 round 6)"
    require_plan_parser || return 0
    setup

    # (a) `===` underline: an H1, which the awk extractor did not see at all.
    write_plan_fixture "${MOCK_REPO}/${PLAN_REL}" '## Approach

APPROACH BODY ONE

SETEXT TITLE ONE
===

AFTER SETEXT ONE'

    local exit_code block
    exit_code=$(run_gemini_sync)
    assert_exit_code "review completes with a === heading" "0" "$exit_code"
    block=$(plan_context_block "${MOCK_REPO}/${PROMPT_REL}")
    assert_contains "Approach body is kept before a === heading" "APPROACH BODY ONE" "$block"
    assert_not_contains "the === heading title does not leak in" "SETEXT TITLE ONE" "$block"
    assert_not_contains "the section after a === heading does not leak in" \
        "AFTER SETEXT ONE" "$block"

    # (b) `---` underline: an H2. The awk extractor cut at the underline
    # (as a thematic break) but had already kept the title line above it.
    write_plan_fixture "${MOCK_REPO}/${PLAN_REL}" '## Approach

APPROACH BODY TWO

SETEXT TITLE TWO
---

AFTER SETEXT TWO'

    exit_code=$(run_gemini_sync)
    assert_exit_code "review completes with a --- heading" "0" "$exit_code"
    block=$(plan_context_block "${MOCK_REPO}/${PROMPT_REL}")
    assert_contains "Approach body is kept before a --- heading" "APPROACH BODY TWO" "$block"
    assert_not_contains "the --- heading title does not leak in" "SETEXT TITLE TWO" "$block"
    assert_not_contains "the section after a --- heading does not leak in" \
        "AFTER SETEXT TWO" "$block"

    teardown
}

test_plan_context_parser_unavailable() {
    echo "TEST: without markdown-it-py (or on an extractor error) the review runs with one warning and no plan context (#320)"
    setup

    write_plan_fixture "${MOCK_REPO}/${PLAN_REL}" '## Approach

HIDDEN APPROACH BODY

## Files to Change

TAIL SECTION'

    # (a) Library missing. A package of the same name that fails to import,
    # first on PYTHONPATH, shadows the real one for every interpreter the
    # script tries (the .venv python3 and python3 on PATH alike).
    local shadow="${TMPDIR_BASE}/shadow-markdown-it"
    mkdir -p "${shadow}/markdown_it"
    printf 'raise ImportError("hidden by test_plan_context_parser_unavailable")\n' \
        > "${shadow}/markdown_it/__init__.py"

    local err_file="${TMPDIR_BASE}/parser-unavailable.err" exit_code=0
    cd "${MOCK_REPO}"
    PYTHONPATH="$shadow" PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 bash "${SCRIPT_UNDER_TEST}" \
        --pr 99 < /dev/null > /dev/null 2> "$err_file" || exit_code=$?

    assert_exit_code "review still completes without the library" "0" "$exit_code"
    local stderr prompt
    stderr=$(cat "$err_file")
    assert_contains "a warning names the missing library" \
        "WARNING: plan context omitted: markdown-it-py is not importable" "$stderr"
    assert_eq "exactly one plan-context warning" "1" \
        "$(grep -c 'plan context omitted' "$err_file" || true)"
    prompt=$(cat "${MOCK_REPO}/${PROMPT_REL}")
    assert_not_contains "no plan context heading without the library" "^## Plan Context$" "$prompt"
    assert_not_contains "the plan body never reaches the prompt" "HIDDEN APPROACH BODY" "$prompt"
    assert_contains "the output-format footer is still there" "^## Output Format$" "$prompt"
    # The mock agy answers with the prompt it was sent, so a findings file
    # carrying the footer is a review that ran end to end.
    assert_contains "the review itself was still produced" "^## Output Format$" \
        "$(cat "${MOCK_REPO}/${FINDINGS_REL}")"

    # (b) Any other extractor failure: an unreadable plan. Same outcome,
    # with the extractor's own reason in the warning.
    if [[ "$(id -u)" -ne 0 ]] && require_plan_parser; then
        chmod 000 "${MOCK_REPO}/${PLAN_REL}"
        exit_code=0
        PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 bash "${SCRIPT_UNDER_TEST}" \
            --pr 99 < /dev/null > /dev/null 2> "$err_file" || exit_code=$?
        chmod 644 "${MOCK_REPO}/${PLAN_REL}"
        assert_exit_code "review still completes on an extractor error" "0" "$exit_code"
        stderr=$(cat "$err_file")
        assert_contains "a warning carries the extractor's reason" \
            "WARNING: plan context omitted: _plan_approach.py failed \(exit 3\): .*PermissionError" "$stderr"
        prompt=$(cat "${MOCK_REPO}/${PROMPT_REL}")
        assert_not_contains "no plan context heading on an extractor error" \
            "^## Plan Context$" "$prompt"
    fi
    rm -f "$err_file"

    teardown
}

test_require_plan_parser_fails_under_ci() {
    echo "TEST: with no markdown-it-py, the extractor-test guard fails under CI and skips locally (#320 round 7)"
    # Subshells, so neither the emptied PLAN_PARSER_PYTHON nor the guard's
    # own FAIL increment leaks into this run's totals.
    local out rc
    rc=0
    out=$(PLAN_PARSER_PYTHON="" CI=true FAIL=0
        require_plan_parser || rc=$?
        echo "rc=${rc} fail=${FAIL}")
    assert_contains "CI=true: the guard reports a failure" "^  FAIL: markdown-it-py" "$out"
    assert_contains "CI=true: the guard returns 1 and counts one failure" "^rc=1 fail=1$" "$out"

    rc=0
    out=$(PLAN_PARSER_PYTHON="" FAIL=0
        unset CI
        require_plan_parser || rc=$?
        echo "rc=${rc} fail=${FAIL}")
    assert_contains "CI unset: the guard skips with a reason" "^  SKIP: markdown-it-py" "$out"
    assert_contains "CI unset: the guard returns 1 and counts no failure" "^rc=1 fail=0$" "$out"

    rc=0
    out=$(PLAN_PARSER_PYTHON="" CI=false FAIL=0
        require_plan_parser || rc=$?
        echo "rc=${rc} fail=${FAIL}")
    assert_contains "CI=false: the guard skips" "^rc=1 fail=0$" "$out"
}

test_plan_context_missing_extractor_is_an_error() {
    echo "TEST: a missing _plan_approach.py is reported as an extractor error, not a missing library (#320 round 7)"
    setup

    write_plan_fixture "${MOCK_REPO}/${PLAN_REL}" '## Approach

HIDDEN APPROACH BODY'

    # A copy of the scripts directory without the extractor. python exits 2
    # when it cannot open a script, which must not read as the extractor's
    # own "markdown-it-py missing" code and send the loop to the next
    # interpreter.
    local scripts_copy="${TMPDIR_BASE}/scripts-copy"
    mkdir -p "$scripts_copy"
    cp -p "${SCRIPT_DIR}/.."/*.sh "$scripts_copy/"
    [[ ! -e "${scripts_copy}/_plan_approach.py" ]] || rm "${scripts_copy}/_plan_approach.py"

    local err_file="${TMPDIR_BASE}/missing-extractor.err" exit_code=0
    cd "${MOCK_REPO}"
    PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 bash "${scripts_copy}/cross_model_review.sh" \
        --pr 99 < /dev/null > /dev/null 2> "$err_file" || exit_code=$?

    assert_exit_code "review still completes without the extractor" "0" "$exit_code"
    local stderr prompt
    stderr=$(cat "$err_file")
    assert_contains "the warning reports an extractor failure" \
        "WARNING: plan context omitted: _plan_approach.py failed \(exit 2\): .*_plan_approach.py" "$stderr"
    assert_not_contains "the warning does not blame the library" "markdown-it-py is not importable" "$stderr"
    assert_eq "exactly one plan-context warning" "1" \
        "$(grep -c 'plan context omitted' "$err_file" || true)"
    prompt=$(cat "${MOCK_REPO}/${PROMPT_REL}")
    assert_not_contains "no plan context heading without the extractor" "^## Plan Context$" "$prompt"
    rm -f "$err_file"

    teardown
}

# ---- Parallel dispatch tests (#206, ADR-0015) ----
#
# tmux is gone: every agent runs in its own background job, all in
# parallel, bounded by AGENT_TIMEOUT (non-gemini).
#
# Since #313 the codex/claude/copilot arms no longer exec their CLI
# directly: they exec _cli_review.sh, which owns the findings file and
# validates the result. The mocks below therefore reproduce each CLI's
# real output shape rather than "write the review to stdout" — a generic
# mock would pass a gate that the real CLIs would not:
#   * codex writes its transcript (banner, echoed prompt, chatter) to
#     stdout and only the final message to the `-o` file;
#   * claude prints one JSON result object;
#   * copilot reads the prompt from stdin (`-p ""` + `-s`) and prints
#     only the response.
#
# Shared knobs (env, per agent, NAME upper-cased):
#   MOCK_<NAME>_SLEEP=<s>    sleep before answering
#   MOCK_<NAME>_EXIT=<n>     exit status (default 0)
#   MOCK_<NAME>_EMPTY=1      produce an empty result at exit 0 (the #288
#                            class: denial / aborted turn)
#   MOCK_<NAME>_ERRMARK=<m>  return <m> as the result at exit 0 (a quota /
#                            rate-limit / auth error printed as output)
#   MOCK_<NAME>_STDERR=<m>   also write <m> to the CLI's stderr
#   MOCK_CODEX_ECHO_FULL=1   codex echoes the WHOLE prompt into its
#                            stdout transcript, as the real one does
#   MOCK_CODEX_IGNORE_TERM=1 codex ignores SIGTERM (only SIGKILL ends it)
#   MOCK_CLAUDE_RAW=<s>      claude prints <s> verbatim instead of JSON
#   MOCK_CLAUDE_SUBTYPE=<s>  claude result subtype (default success)
#   MOCK_CLAUDE_IS_ERROR=<b> claude is_error (default false)
#   MOCK_TIMES_DIR=<dir>     where <name>.start / .end / .pid are written
#   MOCK_ARGV_DIR=<dir>      where <name>.argv is written (one arg per
#                            line) so each CLI's invocation contract can
#                            be asserted — including that the prompt is
#                            never on argv (#212, #274)
#   MOCK_STDIN_DIR=<dir>     where <name>.stdin (everything the CLI read
#                            from stdin) is written

# Preamble shared by every mock: record argv, times, pid and stdin.
MOCK_PREAMBLE='
name=$(basename "$0")
# Sleep in 0.1s slices so a SIGKILLed mock leaves no long-lived orphan
# sleep behind, and so a TERM is handled promptly. Whole seconds only.
mock_sleep() { local n="$1" i; for ((i = 0; i < n * 10; i++)); do sleep 0.1; done; }
[[ -n "${MOCK_ARGV_DIR:-}" ]] && printf "%s\n" "$@" > "${MOCK_ARGV_DIR}/${name}.argv"
[[ -n "${MOCK_TIMES_DIR:-}" ]] && date +%s.%N > "${MOCK_TIMES_DIR}/${name}.start"
[[ -n "${MOCK_TIMES_DIR:-}" ]] && echo $$ > "${MOCK_TIMES_DIR}/${name}.pid"
prompt=$(cat)
[[ -n "${MOCK_STDIN_DIR:-}" ]] && printf "%s" "$prompt" > "${MOCK_STDIN_DIR}/${name}.stdin"
'

make_mock_agent() {
    local name="$1"
    case "$name" in
        codex)
            {
                echo '#!/usr/bin/env bash'
                echo "$MOCK_PREAMBLE"
                cat << 'CODEX_EOF'
# codex exec [-o FILE]: the final message goes to FILE, everything else
# (banner, echoed prompt, tool chatter) to the transcript on stdout.
out=""
args=("$@"); i=0
while [[ $i -lt ${#args[@]} ]]; do
    [[ "${args[$i]}" == "-o" || "${args[$i]}" == "--output-last-message" ]] && out="${args[$((i + 1))]}"
    i=$((i + 1))
done
# MOCK_CODEX_IGNORE_TERM=1: a CLI that ignores SIGTERM, so only an
# escalation to SIGKILL can end it. Armed before the sleep.
[[ -n "${MOCK_CODEX_IGNORE_TERM:-}" ]] && trap '' TERM HUP
# MOCK_CODEX_ORPHAN_PIDFILE=<f>: start a child that ignores SIGTERM and
# outlives this mock unless its whole process group is killed; its pid
# goes to <f> (#660). Its fds are detached so it holds no caller pipe.
if [[ -n "${MOCK_CODEX_ORPHAN_PIDFILE:-}" ]]; then
    ( trap '' TERM HUP; echo "$BASHPID" > "${MOCK_CODEX_ORPHAN_PIDFILE}"
      for ((j = 0; j < 300; j++)); do sleep 0.1; done ) </dev/null >/dev/null 2>&1 &
    for ((j = 0; j < 50; j++)); do [[ -s "${MOCK_CODEX_ORPHAN_PIDFILE}" ]] && break; sleep 0.05; done
fi
[[ -n "${MOCK_CODEX_ENV_DUMP:-}" ]] && env > "${MOCK_CODEX_ENV_DUMP}"
[[ -n "${MOCK_CODEX_SLEEP:-}" ]] && mock_sleep "${MOCK_CODEX_SLEEP}"
echo "codex-cli 0.155.1 (mock banner)"
echo "MOCK TRANSCRIPT: prompt was ${#prompt} bytes"
# Real codex echoes the whole prompt into its transcript. The default
# two-line excerpt keeps the other tests small; MOCK_CODEX_ECHO_FULL=1
# reproduces the full echo, which is what made the error-marker scan over
# this channel a live false positive (#313 round 1).
if [[ -n "${MOCK_CODEX_ECHO_FULL:-}" ]]; then
    printf '%s\n' "$prompt"
else
    printf '%s\n' "$prompt" | head -n 2
fi
[[ -n "${MOCK_CODEX_STDERR:-}" ]] && echo "${MOCK_CODEX_STDERR}" >&2
if [[ -z "$out" ]]; then
    echo "mock codex: no -o file given; the helper must ask for the final message" >&2
    exit 9
fi
if [[ -n "${MOCK_CODEX_EMPTY:-}" ]]; then
    : > "$out"
elif [[ -n "${MOCK_CODEX_ERRMARK:-}" ]]; then
    printf '%s\n' "${MOCK_CODEX_ERRMARK}" > "$out"
else
    printf '### Findings\nreviewed by codex\n' > "$out"
fi
[[ -n "${MOCK_TIMES_DIR:-}" ]] && date +%s.%N > "${MOCK_TIMES_DIR}/codex.end"
exit "${MOCK_CODEX_EXIT:-0}"
CODEX_EOF
            } > "${MOCK_BIN}/codex"
            ;;
        claude)
            {
                echo '#!/usr/bin/env bash'
                echo "$MOCK_PREAMBLE"
                cat << 'CLAUDE_EOF'
# claude -p --output-format json: exactly one result object on stdout.
[[ -n "${MOCK_CLAUDE_SLEEP:-}" ]] && mock_sleep "${MOCK_CLAUDE_SLEEP}"
[[ -n "${MOCK_CLAUDE_STDERR:-}" ]] && echo "${MOCK_CLAUDE_STDERR}" >&2
if [[ -n "${MOCK_CLAUDE_RAW:-}" ]]; then
    printf '%s\n' "${MOCK_CLAUDE_RAW}"
else
    result=$'### Findings\nreviewed by claude'
    [[ -n "${MOCK_CLAUDE_EMPTY:-}" ]] && result=""
    [[ -n "${MOCK_CLAUDE_ERRMARK:-}" ]] && result="${MOCK_CLAUDE_ERRMARK}"
    jq -cn --arg r "$result" --arg s "${MOCK_CLAUDE_SUBTYPE:-success}" \
        --argjson e "${MOCK_CLAUDE_IS_ERROR:-false}" \
        '{type:"result",subtype:$s,is_error:$e,result:$r}'
fi
[[ -n "${MOCK_TIMES_DIR:-}" ]] && date +%s.%N > "${MOCK_TIMES_DIR}/claude.end"
exit "${MOCK_CLAUDE_EXIT:-0}"
CLAUDE_EOF
            } > "${MOCK_BIN}/claude"
            ;;
        copilot)
            {
                echo '#!/usr/bin/env bash'
                echo "$MOCK_PREAMBLE"
                cat << 'COPILOT_EOF'
# copilot -p "" -s --available-tools='' ...: prompt on stdin (#212), response
# only (no stats footer) on stdout. A prompt smuggled onto argv would
# show up in <name>.argv and fail the stdin-contract test.
[[ -n "${MOCK_COPILOT_SLEEP:-}" ]] && mock_sleep "${MOCK_COPILOT_SLEEP}"
[[ -n "${MOCK_COPILOT_STDERR:-}" ]] && echo "${MOCK_COPILOT_STDERR}" >&2
if [[ -n "${MOCK_COPILOT_EMPTY:-}" ]]; then
    :
elif [[ -n "${MOCK_COPILOT_ERRMARK:-}" ]]; then
    printf '%s\n' "${MOCK_COPILOT_ERRMARK}"
else
    printf '### Findings\nreviewed by copilot\n'
fi
[[ -n "${MOCK_TIMES_DIR:-}" ]] && date +%s.%N > "${MOCK_TIMES_DIR}/copilot.end"
exit "${MOCK_COPILOT_EXIT:-0}"
COPILOT_EOF
            } > "${MOCK_BIN}/copilot"
            ;;
        *)
            echo "make_mock_agent: unknown agent '${name}'" >&2
            return 1
            ;;
    esac
    chmod +x "${MOCK_BIN}/${name}"
}

# Run --agents <list> for PR 99 (issue 42); stdout captured to $1, exit
# code echoed. Extra args after the list are passed through.
# RUN_AGENTS_PATH (env) replaces the PATH prefix so a test can hide the
# real CLIs installed on the developer's machine (~/.local/bin etc.).
run_agents() {
    local out_file="$1" agents="$2"; shift 2
    cd "${MOCK_REPO}"
    local exit_code=0
    PATH="${RUN_AGENTS_PATH:-${MOCK_BIN}:${PATH}}" WORKTREE_ISSUE=42 bash "${SCRIPT_UNDER_TEST}" \
        --pr 99 --agents "$agents" "$@" < /dev/null > "$out_file" 2>/dev/null || exit_code=$?
    echo "$exit_code"
}

# A PATH that has the mocks and the system tools but none of the real
# agent CLIs; paired with HOME pointed at an empty dir so the script's
# ~/.local/bin-style fallbacks find nothing either.
HIDDEN_CLI_PATH() { echo "${MOCK_BIN}:/usr/bin:/bin"; }

findings_of() { cat "${MOCK_REPO}/.agent/work-plans/issue-42/review-$1-findings.md"; }

test_sync_flag_rejected() {
    echo "TEST: --sync is rejected as removed (#206)"
    setup
    cd "${MOCK_REPO}"
    local exit_code=0 stderr
    stderr=$(PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 bash "${SCRIPT_UNDER_TEST}" \
        --pr 99 --sync 2>&1 >/dev/null) || exit_code=$?
    assert_exit_code "--sync exits 2" "2" "$exit_code"
    assert_contains "message names --sync and the removal" "\-\-sync was removed" "$stderr"
    teardown
}

test_agents_all_succeed() {
    echo "TEST: --agents runs every agent, prints one triplet each, exits 0 (#206)"
    setup
    make_mock_agent codex; make_mock_agent copilot
    local argv="${TMPDIR_BASE}/argv"; mkdir -p "$argv"
    local out="${TMPDIR_BASE}/out.txt" exit_code
    exit_code=$(MOCK_ARGV_DIR="$argv" run_agents "$out" "gemini,codex,copilot")
    assert_exit_code "all-succeed exits 0" "0" "$exit_code"
    # Per-agent invocation contract, now owned by _cli_review.sh (#313):
    # codex takes a pinned `-s read-only -a never` sandbox/approval policy
    # (rolker/ros2_agent_workspace #660 — this workspace's local deviation
    # from upstream, which relies on codex-cli's current defaults) ahead
    # of `exec` plus a final-message file; copilot takes an empty
    # -p with -s and the least-privilege tool flags. Both read the
    # prompt from stdin, so
    # neither may carry the prompt in argv (#212, #274).
    local codex_argv copilot_argv
    codex_argv=$(cat "$argv/codex.argv")
    copilot_argv=$(cat "$argv/copilot.argv")
    assert_eq "codex invoked with pinned sandbox/approval flags before exec" \
        "$(printf -- '-s\nread-only\n-a\nnever\nexec')" \
        "$(head -n 5 "$argv/codex.argv")"
    assert_contains "codex asked for a final-message file" "^-o$" "$codex_argv"
    assert_eq "copilot argv is the least-privilege print form" \
        "$(printf -- '-p\n\n-s\n--available-tools=\n--disable-builtin-mcps\n--no-ask-user')" \
        "$copilot_argv"
    assert_not_contains "copilot is never given --allow-all-tools" "allow-all-tools" "$copilot_argv"
    assert_not_contains "codex prompt is not on argv" "Adversarial Code Review" "$codex_argv"
    assert_not_contains "copilot prompt is not on argv" "Adversarial Code Review" "$copilot_argv"
    local stdout; stdout=$(cat "$out")
    assert_contains "MODE=parallel-sync printed once" "^MODE=parallel-sync$" "$stdout"
    assert_eq "exactly one MODE line" "1" "$(grep -c '^MODE=' "$out")"
    assert_eq "three AGENT= lines" "3" "$(grep -c '^AGENT=' "$out")"
    assert_eq "three EXIT=0 lines" "3" "$(grep -c '^EXIT=0$' "$out")"
    assert_not_contains "no TMUX_SESSION line" "TMUX_SESSION" "$stdout"
    # Findings paths are announced before the agents run (tail -f contract):
    # the informational line for codex must precede its AGENT= triplet.
    local info_line triplet_line
    info_line=$(grep -n "^  codex: .*review-codex-findings.md" "$out" | head -n1 | cut -d: -f1)
    triplet_line=$(grep -n "^AGENT=codex$" "$out" | head -n1 | cut -d: -f1)
    if [[ -n "$info_line" && -n "$triplet_line" && "$info_line" -lt "$triplet_line" ]]; then
        echo "  PASS: findings path announced before the triplet"; PASS=$((PASS + 1))
    else
        echo "  FAIL: findings path not announced before the triplet (info=${info_line:-none} triplet=${triplet_line:-none})"; FAIL=$((FAIL + 1))
    fi
    # Triplet order follows the --agents order; EXIT immediately follows FINDINGS_FILE.
    local block; block=$(grep -E '^(AGENT|FINDINGS_FILE|EXIT)=' "$out" | tr '\n' ' ')
    assert_contains "triplets in selection order" \
        "AGENT=gemini FINDINGS_FILE=[^ ]*review-gemini-findings.md EXIT=0 AGENT=codex FINDINGS_FILE=[^ ]*review-codex-findings.md EXIT=0 AGENT=copilot" "$block"
    assert_contains "gemini findings complete" "Review complete" "$(findings_of gemini)"
    assert_contains "codex findings hold its output" "reviewed by codex" "$(findings_of codex)"
    assert_contains "codex findings complete" "Review complete" "$(findings_of codex)"
    assert_contains "copilot findings complete" "Review complete" "$(findings_of copilot)"
    assert_contains "each agent got its own prompt" "Adversarial Code Review" \
        "$(cat "${MOCK_REPO}/.agent/work-plans/issue-42/review-codex-prompt.md")"
    teardown
}

test_agents_partial_failure() {
    echo "TEST: one failing agent does not disturb the others; exit 3 with per-agent EXIT= (#206)"
    setup
    make_mock_agent codex; make_mock_agent copilot
    local out="${TMPDIR_BASE}/out.txt" exit_code
    exit_code=$(MOCK_CODEX_EXIT=7 run_agents "$out" "gemini,codex,copilot")
    assert_exit_code "partial failure exits 3" "3" "$exit_code"
    # EXIT= is the job's status. Since #313 that is _cli_review.sh's own
    # exit 1 ("no usable result"); the CLI's status is in the findings
    # file's reason, the same shape gemini has had since #288.
    assert_contains "codex EXIT=1" "^EXIT=1$" "$(cat "$out")"
    assert_contains "codex findings name the CLI's own status" "codex exited 7" "$(findings_of codex)"
    assert_eq "two EXIT=0 lines" "2" "$(grep -c '^EXIT=0$' "$out")"
    assert_contains "codex findings marked failed" "Review failed" "$(findings_of codex)"
    assert_not_contains "codex findings not marked complete" "Review complete" "$(findings_of codex)"
    assert_contains "gemini findings still complete" "Review complete" "$(findings_of gemini)"
    assert_contains "copilot findings still complete" "Review complete" "$(findings_of copilot)"
    teardown
}

test_agents_timeout() {
    echo "TEST: a hung agent is cut off by AGENT_TIMEOUT; the fast agent's marker is not delayed (#206)"
    setup
    make_mock_agent codex; make_mock_agent copilot
    local times="${TMPDIR_BASE}/times"; mkdir -p "$times"
    local out="${TMPDIR_BASE}/out.txt" exit_code
    exit_code=$(AGENT_TIMEOUT=1 MOCK_CODEX_SLEEP=6 MOCK_TIMES_DIR="$times" run_agents "$out" "codex,copilot")
    assert_exit_code "timeout run exits 3" "3" "$exit_code"
    assert_contains "codex EXIT=124 (timeout)" "^EXIT=124$" "$(cat "$out")"
    assert_contains "codex findings name the timeout" "timed out \(AGENT_TIMEOUT=1\)" "$(findings_of codex)"
    assert_contains "codex findings marked failed" "Review failed" "$(findings_of codex)"
    assert_contains "copilot findings complete" "Review complete" "$(findings_of copilot)"
    # codex must have been killed: its mock only writes .end when it ran
    # to completion, which AGENT_TIMEOUT=1 forbids.
    if [[ -f "$times/codex.end" ]]; then
        echo "  FAIL: codex ran to completion despite AGENT_TIMEOUT=1"; FAIL=$((FAIL + 1))
    else
        echo "  PASS: codex was killed before it could finish"; PASS=$((PASS + 1))
    fi
    teardown
}

test_agents_concurrency() {
    echo "TEST: agents run concurrently, not one after another (#206)"
    setup
    make_mock_agent codex; make_mock_agent copilot; make_mock_agent claude
    local times="${TMPDIR_BASE}/times"; mkdir -p "$times"
    local out="${TMPDIR_BASE}/out.txt" exit_code
    exit_code=$(MOCK_CODEX_SLEEP=2 MOCK_COPILOT_SLEEP=2 MOCK_CLAUDE_SLEEP=2 MOCK_TIMES_DIR="$times" \
        run_agents "$out" "codex,copilot,claude")
    assert_exit_code "concurrent run exits 0" "0" "$exit_code"
    # Interval overlap is the whole assertion: every agent started before
    # the first one ended, which sequential dispatch cannot produce. A
    # wall-clock bound was tried and dropped — it was the suite's one
    # load-sensitive check and could fail on a busy machine with nothing
    # actually regressed.
    local first_end; first_end=$(sort -n "$times"/*.end | head -n 1)
    local overlap=true a
    for a in codex copilot claude; do
        if ! awk -v s="$(cat "$times/$a.start")" -v e="$first_end" 'BEGIN{exit !(s < e)}'; then overlap=false; fi
    done
    if [[ "$overlap" == true ]]; then
        echo "  PASS: all three agents started before the first finished (intervals overlap)"; PASS=$((PASS + 1))
    else
        echo "  FAIL: agent intervals did not overlap"; FAIL=$((FAIL + 1))
    fi
    teardown
}

test_gemini_backstop_cuts_off_wedged_agy() {
    echo "TEST: a wedged agy is cut off by the outer gemini backstop (#206)"
    setup
    local times="${TMPDIR_BASE}/times"; mkdir -p "$times"
    local out="${TMPDIR_BASE}/out.txt" exit_code
    # agy consumes the prompt and never answers, so _agy_review.sh's own
    # --print-timeout handling never runs. Backstop = print-timeout (1s) +
    # margin (2s) = 3s; only that bound can end the run.
    local scratch="${TMPDIR_BASE}/scratch"; mkdir -p "$scratch"
    # REVIEW_KILL_ESCALATION must stay under AGENT_KILL_AFTER (#313
    # round 2), so a 1s outer grace needs a 0s helper escalation.
    exit_code=$(TMPDIR="$scratch" AGY_PRINT_TIMEOUT=1s GEMINI_BACKSTOP_MARGIN=2 AGENT_KILL_AFTER=1 \
        REVIEW_KILL_ESCALATION=0 \
        MOCK_AGY_STALL=1 MOCK_TIMES_DIR="$times" run_agents "$out" "gemini")
    assert_exit_code "backstopped run exits 3" "3" "$exit_code"
    assert_contains "gemini EXIT=124 (timeout)" "^EXIT=124$" "$(cat "$out")"
    assert_contains "findings name the backstop and the print-timeout" \
        "GEMINI_BACKSTOP=3s, above AGY_PRINT_TIMEOUT=1s" "$(findings_of gemini)"
    assert_contains "gemini findings marked failed" "Review failed" "$(findings_of gemini)"
    # The wedged agy must be dead, not merely abandoned.
    sleep 0.3
    local agy_pid; agy_pid=$(cat "$times/agy.pid" 2>/dev/null || echo "")
    if [[ -n "$agy_pid" ]] && kill -0 "$agy_pid" 2>/dev/null; then
        kill "$agy_pid" 2>/dev/null || true
        echo "  FAIL: the wedged agy process survived the backstop"; FAIL=$((FAIL + 1))
    else
        echo "  PASS: the wedged agy process was killed"; PASS=$((PASS + 1))
    fi
    # Nothing is left in the scratch root: the parent owns the helper's
    # TMPDIR precisely because a SIGKILLed helper skips its own EXIT trap.
    assert_eq "no temp files survive the backstopped run" "0" "$(ls -A "$scratch" | wc -l)"
    teardown
}

test_gemini_not_bound_by_agent_timeout() {
    echo "TEST: gemini is not wrapped by the plain AGENT_TIMEOUT path (ADR-0015 §3) (#206)"
    setup
    make_mock_agent codex
    local out="${TMPDIR_BASE}/out.txt" exit_code
    # AGENT_TIMEOUT=1 kills codex (3s) but must not touch gemini, whose
    # bound is AGY_PRINT_TIMEOUT plus the backstop derived above it. If a
    # future change routed gemini through AGENT_TIMEOUT, its 3s turn would
    # be cut off and this test would fail.
    exit_code=$(AGENT_TIMEOUT=1 MOCK_AGY_SLEEP=3 MOCK_CODEX_SLEEP=3 run_agents "$out" "gemini,codex")
    assert_exit_code "mixed run exits 3 (codex timed out)" "3" "$exit_code"
    assert_contains "codex EXIT=124 under AGENT_TIMEOUT=1" "^EXIT=124$" "$(cat "$out")"
    assert_contains "codex findings name AGENT_TIMEOUT" "timed out \(AGENT_TIMEOUT=1\)" "$(findings_of codex)"
    assert_contains "gemini completed anyway" "Review complete" "$(findings_of gemini)"
    assert_not_contains "gemini not marked failed" "Review failed" "$(findings_of gemini)"
    assert_not_contains "gemini findings carry no AGENT_TIMEOUT note" "AGENT_TIMEOUT" "$(findings_of gemini)"
    teardown
}

# Run the script with one knob overridden; echoes "<exit>|<stderr>".
run_with_knob() {
    local assignment="$1" ec=0 stderr
    stderr=$(env "$assignment" PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 \
        bash "${SCRIPT_UNDER_TEST}" --pr 99 --agents codex 2>&1 >/dev/null) || ec=$?
    printf '%s|%s' "$ec" "$stderr"
}

test_duration_knobs_validated() {
    echo "TEST: bad duration knobs and a bad --pr exit 2 with a clear message (#206)"
    setup
    cd "${MOCK_REPO}"
    local ec stderr knob result
    # Shape: a non-duration is rejected for every knob.
    for knob in AGENT_TIMEOUT AGENT_KILL_AFTER GEMINI_BACKSTOP_MARGIN; do
        result=$(run_with_knob "${knob}=abc")
        assert_exit_code "bad ${knob} exits 2" "2" "${result%%|*}"
        assert_contains "message names ${knob} and the shape" "${knob} value 'abc' is not a valid duration" "${result#*|}"
    done

    # Range: 0 is a shape-valid value that silently removes a bound, so
    # the knobs whose whole purpose is a bound must refuse it.
    for knob in AGENT_TIMEOUT GEMINI_BACKSTOP_MARGIN; do
        result=$(run_with_knob "${knob}=0")
        assert_exit_code "${knob}=0 exits 2" "2" "${result%%|*}"
        assert_contains "${knob}=0 message demands a positive value" \
            "${knob} value '0' must be greater than zero" "${result#*|}"
    done
    result=$(run_with_knob "AGY_PRINT_TIMEOUT=0s")
    assert_exit_code "AGY_PRINT_TIMEOUT=0s exits 2" "2" "${result%%|*}"
    assert_contains "AGY_PRINT_TIMEOUT=0s message demands a positive value" \
        "AGY_PRINT_TIMEOUT value '0s' must be greater than zero" "${result#*|}"
    # AGENT_KILL_AFTER=0 is refused too: `timeout -k 0` DISABLES the
    # SIGKILL escalation rather than sending it at once (#660), so zero
    # would remove the backstop for a CLI that ignores SIGTERM.
    result=$(run_with_knob "AGENT_KILL_AFTER=0")
    assert_exit_code "AGENT_KILL_AFTER=0 exits 2" "2" "${result%%|*}"
    assert_contains "AGENT_KILL_AFTER=0 names the disabled escalation" \
        "DISABLES the SIGKILL escalation" "${result#*|}"
    local out="${TMPDIR_BASE}/out.txt"
    make_mock_agent codex
    # A grace the helper cannot fit its escalation inside is refused by
    # the helper (exit 2) rather than silently orphaning the CLI.
    ec=$(AGENT_KILL_AFTER=1 REVIEW_KILL_ESCALATION=5 run_agents "$out" "codex")
    assert_exit_code "AGENT_KILL_AFTER below the escalation fails the agent" "3" "$ec"
    assert_contains "reason names both knobs" \
        "AGENT_KILL_AFTER \(1\) must be greater than REVIEW_KILL_ESCALATION \(5\)" "$(findings_of codex)"
    assert_contains "the refused run is marked failed" "Review failed" "$(findings_of codex)"

    # Go-duration subset: AGY_PRINT_TIMEOUT reaches agy's --print-timeout,
    # which needs an explicit s/m/h unit and has no `d`. Both shapes below
    # are valid for coreutils `timeout` and would otherwise pass.
    for knob in "AGY_PRINT_TIMEOUT=90" "AGY_PRINT_TIMEOUT=1d"; do
        result=$(run_with_knob "$knob")
        assert_exit_code "${knob} exits 2" "2" "${result%%|*}"
        assert_contains "${knob} message names the Go-duration requirement" \
            "is not a valid Go duration" "${result#*|}"
    done
    # The same unit-less value stays valid for the coreutils-only knob.
    ec=$(AGENT_TIMEOUT=90 run_agents "$out" "codex")
    assert_exit_code "AGENT_TIMEOUT=90 (unit-less) is accepted" "0" "$ec"

    ec=0; stderr=$(PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 \
        bash "${SCRIPT_UNDER_TEST}" --pr 9x --agents codex 2>&1 >/dev/null) || ec=$?
    assert_exit_code "non-integer --pr exits 2" "2" "$ec"
    assert_contains "--pr message names the value" "\-\-pr value '9x' is not a positive integer" "$stderr"
    teardown
}

test_agents_missing_binary() {
    echo "TEST: a missing CLI fails only that agent (#206)"
    setup
    make_mock_agent codex
    # copilot deliberately not mocked and hidden from PATH fallbacks.
    local out="${TMPDIR_BASE}/out.txt" exit_code
    exit_code=$(HOME="${TMPDIR_BASE}/nohome" RUN_AGENTS_PATH="$(HIDDEN_CLI_PATH)" run_agents "$out" "codex,copilot")
    assert_exit_code "missing-binary run exits 3" "3" "$exit_code"
    assert_contains "codex EXIT=0" "^EXIT=0$" "$(cat "$out")"
    assert_contains "copilot EXIT=1" "^EXIT=1$" "$(cat "$out")"
    assert_contains "copilot findings name the missing CLI" "copilot CLI not found" "$(findings_of copilot)"
    assert_contains "copilot findings marked failed" "Review failed" "$(findings_of copilot)"
    assert_contains "codex findings complete" "Review complete" "$(findings_of codex)"
    teardown
}

test_agents_none_usable() {
    echo "TEST: no usable CLI at all is a dependency error, exit 1 (#206)"
    setup
    local out="${TMPDIR_BASE}/out.txt" exit_code
    exit_code=$(HOME="${TMPDIR_BASE}/nohome" RUN_AGENTS_PATH="$(HIDDEN_CLI_PATH)" run_agents "$out" "codex,copilot")
    assert_exit_code "none usable exits 1" "1" "$exit_code"
    assert_not_contains "no triplets printed" "^AGENT=" "$(cat "$out")"
    teardown
}

test_agents_argument_hygiene() {
    echo "TEST: --agents hygiene — exclusion, unknown, empty entry, dedupe, case/space (#206)"
    setup
    make_mock_agent codex
    cd "${MOCK_REPO}"
    local ec stderr
    ec=0; stderr=$(PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 bash "${SCRIPT_UNDER_TEST}" \
        --pr 99 --agent codex --agents gemini 2>&1 >/dev/null) || ec=$?
    assert_exit_code "--agent with --agents exits 2" "2" "$ec"
    assert_contains "mutual-exclusion message" "mutually exclusive" "$stderr"

    ec=0; stderr=$(PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 bash "${SCRIPT_UNDER_TEST}" \
        --pr 99 --agents gemini,grok 2>&1 >/dev/null) || ec=$?
    assert_exit_code "unknown agent exits 2" "2" "$ec"
    assert_contains "unknown agent named" "Unknown agent 'grok'" "$stderr"

    ec=0; stderr=$(PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 bash "${SCRIPT_UNDER_TEST}" \
        --pr 99 --agents "gemini,,codex" 2>&1 >/dev/null) || ec=$?
    assert_exit_code "empty entry exits 2" "2" "$ec"
    assert_contains "empty-entry message" "empty entry" "$stderr"

    ec=0; stderr=$(PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 bash "${SCRIPT_UNDER_TEST}" \
        --pr 99 --agents "codex," 2>&1 >/dev/null) || ec=$?
    assert_exit_code "trailing comma exits 2" "2" "$ec"

    local out="${TMPDIR_BASE}/out.txt"
    ec=$(run_agents "$out" " Codex , codex ")
    assert_exit_code "dedupe + trim + case run exits 0" "0" "$ec"
    assert_eq "duplicate collapses to one triplet" "1" "$(grep -c '^AGENT=' "$out")"
    assert_contains "normalised to lowercase codex" "^AGENT=codex$" "$(cat "$out")"
    teardown
}

test_agents_shared_diff_failure() {
    echo "TEST: a failed shared diff marks every selected findings file, no triplets, exit 3 (#206)"
    setup
    make_mock_agent codex
    cat > "${MOCK_BIN}/gh" << 'GH_EOF'
#!/usr/bin/env bash
if [[ "$1" == "pr" && "$2" == "view" ]]; then
    shift 3; [[ "${1:-}" == "-R" ]] && shift 2
    case "$2" in body) echo "Closes #42" ;; title) echo "Test PR" ;; url) echo "https://github.com/test/repo/pull/99" ;; esac
    exit 0
elif [[ "$1" == "pr" && "$2" == "diff" ]]; then
    echo "gh: connection reset" >&2; exit 1
fi
exit 0
GH_EOF
    chmod +x "${MOCK_BIN}/gh"
    local out="${TMPDIR_BASE}/out.txt" exit_code
    exit_code=$(run_agents "$out" "gemini,codex")
    assert_exit_code "shared diff failure exits 3" "3" "$exit_code"
    assert_not_contains "no AGENT= triplets" "^AGENT=" "$(cat "$out")"
    assert_contains "gemini findings carry the error marker" "Review error: failed to retrieve diff" "$(findings_of gemini)"
    assert_contains "codex findings carry the error marker" "Review error: failed to retrieve diff" "$(findings_of codex)"
    teardown
}

test_agents_interrupt_kills_jobs() {
    echo "TEST: terminating the script stops the running agents promptly (#206)"
    setup
    make_mock_agent codex; make_mock_agent copilot
    local times="${TMPDIR_BASE}/times"; mkdir -p "$times"
    cd "${MOCK_REPO}"
    # SIGTERM, not SIGINT: bash ignores INT in background children of a
    # non-interactive shell, so a test-sent INT would never arrive. TERM
    # is what timeouts and callers send.
    local t0; t0=$(date +%s)
    MOCK_CODEX_SLEEP=30 MOCK_COPILOT_SLEEP=30 MOCK_TIMES_DIR="$times" \
        PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 bash "${SCRIPT_UNDER_TEST}" \
        --pr 99 --agents codex,copilot < /dev/null > /dev/null 2>&1 &
    local script_pid=$!
    local i
    for ((i = 0; i < 50; i++)); do
        [[ -f "$times/codex.pid" && -f "$times/copilot.pid" ]] && break
        sleep 0.1
    done
    kill -TERM "$script_pid" 2>/dev/null || true
    local ec=0; wait "$script_pid" || ec=$?
    local elapsed=$(( $(date +%s) - t0 ))
    assert_exit_code "terminated script exits 143" "143" "$ec"
    if [[ "$elapsed" -lt 10 ]]; then
        echo "  PASS: script returned in ${elapsed}s, not after the agents' 30s sleep"; PASS=$((PASS + 1))
    else
        echo "  FAIL: script took ${elapsed}s to return after TERM"; FAIL=$((FAIL + 1))
    fi
    sleep 0.3
    local alive=0 p
    for p in codex copilot; do
        if [[ -f "$times/$p.pid" ]] && kill -0 "$(cat "$times/$p.pid")" 2>/dev/null; then
            alive=$((alive + 1))
            kill "$(cat "$times/$p.pid")" 2>/dev/null || true
        fi
    done
    assert_eq "no agent process survives the termination" "0" "$alive"
    teardown
}

test_single_agent_output_unchanged() {
    echo "TEST: --agent keeps the single-agent stdout contract (no EXIT=, no triplets) (#206)"
    setup
    make_mock_agent codex
    cd "${MOCK_REPO}"
    local out="${TMPDIR_BASE}/out.txt" ec=0
    PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 bash "${SCRIPT_UNDER_TEST}" \
        --pr 99 --agent codex < /dev/null > "$out" 2>/dev/null || ec=$?
    assert_exit_code "single agent exits 0" "0" "$ec"
    local stdout; stdout=$(cat "$out")
    assert_contains "MODE=sync" "^MODE=sync$" "$stdout"
    assert_contains "AGENT=codex" "^AGENT=codex$" "$stdout"
    assert_contains "FINDINGS_FILE line" "^FINDINGS_FILE=.*review-codex-findings.md$" "$stdout"
    assert_not_contains "no EXIT= line in single-agent mode" "^EXIT=" "$stdout"
    assert_contains "completion line" "^Review complete. Results:" "$stdout"
    assert_contains "codex findings complete" "Review complete" "$(findings_of codex)"
    teardown
}

# ---- _cli_review.sh result validation (#313, folding in #212) ----
#
# The codex/claude/copilot arms used to be gated on the CLI's exit code
# alone, so an empty response, a quota error printed as output, or (for
# codex) the raw stdout transcript all landed in the findings file as if
# they were a review. Each case below is one of those failure modes.

CLI_HELPER_UNDER_TEST="${SCRIPT_DIR}/../_cli_review.sh"
CLI_AGENTS=(codex claude copilot)

# Invoke _cli_review.sh directly (no cross_model_review.sh around it).
# Echoes the exit code; the findings file is the caller's to inspect.
run_cli_helper() {
    local agent="$1" prompt="$2" findings="$3" ec=0
    shift 3
    TMPDIR="${TMPDIR_BASE}/helper-tmp" PATH="${MOCK_BIN}:${PATH}" \
        bash "$CLI_HELPER_UNDER_TEST" "$agent" "${MOCK_BIN}/${agent}" \
        "$prompt" "$findings" "$@" >/dev/null 2>&1 || ec=$?
    echo "$ec"
}

test_cli_codex_transcript_not_in_findings() {
    echo "TEST: codex's stdout transcript never becomes the review (#313)"
    setup
    make_mock_agent codex
    local out="${TMPDIR_BASE}/out.txt" exit_code
    exit_code=$(run_agents "$out" "codex")
    assert_exit_code "codex run exits 0" "0" "$exit_code"
    local content; content=$(findings_of codex)
    assert_contains "findings hold the final message" "reviewed by codex" "$content"
    assert_not_contains "banner is not in the findings" "mock banner" "$content"
    assert_not_contains "echoed prompt is not in the findings" "MOCK TRANSCRIPT" "$content"
    assert_contains "findings complete" "Review complete" "$content"
    teardown
}

test_cli_prompt_reaches_every_cli_on_stdin() {
    echo "TEST: the prompt reaches every CLI on stdin, not /dev/null (#313)"
    setup
    local stdin_dir="${TMPDIR_BASE}/stdin"; mkdir -p "$stdin_dir"
    local agent out exit_code
    for agent in "${CLI_AGENTS[@]}"; do
        make_mock_agent "$agent"
    done
    out="${TMPDIR_BASE}/out.txt"
    exit_code=$(MOCK_STDIN_DIR="$stdin_dir" run_agents "$out" "codex,claude,copilot")
    assert_exit_code "all three exit 0" "0" "$exit_code"
    for agent in "${CLI_AGENTS[@]}"; do
        assert_contains "${agent} read the whole prompt from stdin" "Adversarial Code Review" \
            "$(cat "${stdin_dir}/${agent}.stdin" 2>/dev/null || echo "")"
    done
    teardown
}

test_cli_empty_response_is_failure() {
    echo "TEST: an empty response at exit 0 is a failed review for every CLI (#313, #288 class)"
    setup
    local agent out exit_code content
    for agent in "${CLI_AGENTS[@]}"; do
        make_mock_agent "$agent"
        out="${TMPDIR_BASE}/out-${agent}.txt"
        # run_agents is a shell function, so the knob is exported rather
        # than prefixed through `env`.
        export "MOCK_${agent^^}_EMPTY=1"
        exit_code=$(run_agents "$out" "$agent")
        unset "MOCK_${agent^^}_EMPTY"
        assert_exit_code "${agent} empty response exits 3" "3" "$exit_code"
        content=$(findings_of "$agent")
        assert_contains "${agent} findings name the empty response" "empty response" "$content"
        assert_contains "${agent} findings marked failed" "Review failed" "$content"
        assert_not_contains "${agent} findings not marked complete" "Review complete" "$content"
    done
    teardown
}

test_cli_nonzero_exit_is_failure() {
    echo "TEST: a non-zero CLI exit is reported with its status for every CLI (#313)"
    setup
    local agent out exit_code content
    for agent in "${CLI_AGENTS[@]}"; do
        make_mock_agent "$agent"
        out="${TMPDIR_BASE}/out-${agent}.txt"
        export "MOCK_${agent^^}_EXIT=7" "MOCK_${agent^^}_STDERR=boom"
        exit_code=$(run_agents "$out" "$agent")
        unset "MOCK_${agent^^}_EXIT" "MOCK_${agent^^}_STDERR"
        assert_exit_code "${agent} crash exits 3" "3" "$exit_code"
        content=$(findings_of "$agent")
        assert_contains "${agent} findings name the exit status" "${agent} exited 7" "$content"
        assert_contains "${agent} findings carry the CLI's output" "boom" "$content"
        assert_contains "${agent} findings marked failed" "Review failed" "$content"
    done
    teardown
}

test_cli_error_text_explains_never_causes() {
    echo "TEST: error TEXT never fails a run; it explains a failure the exit status already caused (#313 round 2)"
    setup
    local agent out exit_code content
    for agent in "${CLI_AGENTS[@]}"; do
        make_mock_agent "$agent"
        out="${TMPDIR_BASE}/out-${agent}.txt"

        # Exit 0 with a quota-looking answer: PASSES. Every attempt to
        # catch this by text also discarded real reviews (rounds 1 and
        # 2), so the text is handed through as the review.
        export "MOCK_${agent^^}_ERRMARK=Error: you have exceeded your usage limit."
        exit_code=$(run_agents "$out" "$agent")
        unset "MOCK_${agent^^}_ERRMARK"
        assert_exit_code "${agent}: error text at exit 0 does not fail the run" "0" "$exit_code"
        assert_contains "${agent}: the text is kept as the review" "usage limit" "$(findings_of "$agent")"

        # Same marker, this time on stderr with a non-zero exit: FAILS,
        # and the marker picks the reason line.
        export "MOCK_${agent^^}_EXIT=4" "MOCK_${agent^^}_STDERR=fatal: you have exceeded your usage limit"
        exit_code=$(run_agents "$out" "$agent")
        unset "MOCK_${agent^^}_EXIT" "MOCK_${agent^^}_STDERR"
        assert_exit_code "${agent}: non-zero exit fails the run" "3" "$exit_code"
        content=$(findings_of "$agent")
        assert_contains "${agent}: reason names the exit status" "${agent} exited 4" "$content"
        assert_contains "${agent}: marker explains the failure" \
            "looks like a quota / rate-limit / authentication problem" "$content"
        assert_contains "${agent} findings marked failed" "Review failed" "$content"

        # A transient stderr warning with exit 0 is NOT a failure
        # (gemini must-fix 3: "[WARN] overloaded, retrying in 1s").
        export "MOCK_${agent^^}_STDERR=[WARN] Server overloaded, retrying in 1s..."
        exit_code=$(run_agents "$out" "$agent")
        unset "MOCK_${agent^^}_STDERR"
        assert_exit_code "${agent}: a transient stderr warning does not fail the run" "0" "$exit_code"
        assert_contains "${agent}: the review survives the warning" "reviewed by ${agent}" "$(findings_of "$agent")"
    done
    teardown
}

test_cli_codex_transcript_marker_does_not_fail_the_review() {
    echo "TEST: a marker word inside the reviewed diff does not fail codex (#313 round 1, live false positive)"
    setup
    make_mock_agent codex
    # A diff whose body contains the very words the marker scan looks
    # for. Real codex echoes the whole prompt into its stdout transcript,
    # so scanning that channel failed this branch's own review.
    local diff="${TMPDIR_BASE}/marker.diff"
    {
        echo "diff --git a/auth.py b/auth.py"
        echo "--- a/auth.py"
        echo "+++ b/auth.py"
        echo "@@ -1,3 +1,4 @@"
        echo "+# retry when the server is overloaded or the request is unauthorized"
        echo "+RATE_LIMIT_MESSAGE = 'you have exceeded your usage limit'"
    } > "$diff"
    local out="${TMPDIR_BASE}/out.txt" exit_code
    exit_code=$(MOCK_GH_DIFF_FILE="$diff" MOCK_CODEX_ECHO_FULL=1 run_agents "$out" "codex")
    assert_exit_code "codex review of a marker-bearing diff exits 0" "0" "$exit_code"
    local content; content=$(findings_of codex)
    assert_contains "the review is kept" "reviewed by codex" "$content"
    assert_contains "findings complete" "Review complete" "$content"
    assert_not_contains "not failed as a quota error" "Review failed" "$content"
    # codex prints its transcript on BOTH channels — prompt, tool calls
    # and the tools' own output — so stderr is no cleaner than stdout
    # (#313 round 2: a `jq: error` line from a command codex ran failed
    # the run). Neither channel may fail a codex review.
    exit_code=$(MOCK_GH_DIFF_FILE="$diff" MOCK_CODEX_ECHO_FULL=1 \
        MOCK_CODEX_STDERR="jq: error (at <stdin>:1): Cannot index string with string; you have exceeded your usage limit" \
        run_agents "$out" "codex")
    assert_exit_code "codex error text on stderr does not fail the review" "0" "$exit_code"
    assert_contains "the review is still kept" "reviewed by codex" "$(findings_of codex)"
    assert_not_contains "no failure marker" "Review failed" "$(findings_of codex)"
    teardown
}

test_cli_review_text_is_never_reclassified() {
    echo "TEST: no review is discarded for how its text reads (#313 round 2, gemini must-fix 1-2)"
    setup
    make_mock_agent copilot
    local out="${TMPDIR_BASE}/out.txt" ec

    # A review that OPENS like an error: an adversarial review of auth
    # code legitimately starts "# Error Handling in auth.py" and then
    # discusses rate limits. The round-1 opener rule failed exactly this.
    local error_headed_review="# Error Handling in auth.py

The retry path ignores the rate limit header, so an unauthorized
response is retried forever; an overloaded backend then sees the same
request repeatedly. Recommend honouring Retry-After."
    ec=$(MOCK_COPILOT_ERRMARK="$error_headed_review" run_agents "$out" "copilot")
    assert_exit_code "a review headed 'Error Handling' is accepted" "0" "$ec"
    assert_contains "the review body is kept" "ignores the rate limit header" "$(findings_of copilot)"

    # Concise, list-formatted findings whose every line names a marker.
    local concise_review="- The retry path ignores the rate limit header.
- Unauthorized responses are retried forever."
    ec=$(MOCK_COPILOT_ERRMARK="$concise_review" run_agents "$out" "copilot")
    assert_exit_code "concise list findings are accepted" "0" "$ec"
    assert_contains "the findings are kept verbatim" "retry path ignores the rate limit" "$(findings_of copilot)"

    # A one-line review that happens to name a marker.
    ec=$(MOCK_COPILOT_ERRMARK="No blocking issues; the rate limit handling is correct." \
        run_agents "$out" "copilot")
    assert_exit_code "a one-line clean review is accepted" "0" "$ec"

    # An EMPTY result still fails — that is machine state, not text.
    ec=$(MOCK_COPILOT_EMPTY=1 run_agents "$out" "copilot")
    assert_exit_code "an empty result still fails" "3" "$ec"
    assert_contains "reason names the empty response" "empty response" "$(findings_of copilot)"
    teardown
}

test_cli_claude_error_payload_in_reason() {
    echo "TEST: claude's .error payload reaches the failure reason (#313)"
    setup
    make_mock_agent claude
    local out="${TMPDIR_BASE}/out.txt" ec

    # Synthetic fallback coverage for the `.error` branch — not observed
    # CLI output. The live captures in test_cli_claude_live_failure_shapes
    # show claude 2.1.281 uses `.errors` (an array), not `.error`, for
    # max-turns; these two keep the `.error` reader covered for shapes
    # that were not captured live (e.g. error_during_execution).
    # is_error with an EMPTY .result: the payload is the only cause there is.
    ec=$(MOCK_CLAUDE_RAW='{"type":"result","subtype":"error_during_execution","is_error":true,"result":"","error":{"message":"upstream connect error: 503"}}' \
        run_agents "$out" "claude")
    assert_exit_code "is_error with an object payload exits 3" "3" "$ec"
    assert_contains "reason carries .error.message" "upstream connect error: 503" "$(findings_of claude)"

    # A string-valued .error on a bad subtype (synthetic, as above).
    ec=$(MOCK_CLAUDE_RAW='{"type":"result","subtype":"error_max_turns","is_error":false,"result":"","error":"max turns exceeded"}' \
        run_agents "$out" "claude")
    assert_exit_code "bad subtype with a string payload exits 3" "3" "$ec"
    assert_contains "reason names the subtype" "error_max_turns" "$(findings_of claude)"
    assert_contains "reason carries the string .error" "max turns exceeded" "$(findings_of claude)"
    teardown
}

# Full result objects captured live from claude 2.1.281 (#336) with
# `--output-format json --permission-prompts none`, from a scratch
# directory. Scrubbed: session_id / uuid, and the max-turns shape's
# permission_denials tool_input values and tool_use_id (fields kept,
# values replaced). Every failure shape exited 1 — the reason is in the
# JSON only, which is why the helper must read it before the exit code.
CLAUDE_LIVE_BAD_MODEL=$(cat << 'JSON_EOF'
{"api_error_status":404,"duration_api_ms":0,"duration_ms":611,"fast_mode_disabled_reason":"sdk_opt_in_required","fast_mode_state":"off","is_error":true,"modelUsage":{},"num_turns":1,"permission_denials":[],"queued_turn_count":0,"result":"There's an issue with the selected model (bogus-model-xyz). It may not exist or you may not have access to it. Run --model to pick a different model.","result_index":0,"session_id":"<scrubbed>","stop_reason":"stop_sequence","subagent_stats":{"by_type":{},"completed":0,"failed":0,"killed":{"parent":0,"system":0,"user":0},"max_depth":0,"refused":{"budget":0,"concurrency_limit":0,"depth_limit":0},"requested":{"background":0,"foreground":0,"unset":0},"spawned":0,"spawned_by_subagents":0,"started_in_background":0},"subtype":"success","terminal_reason":"api_error","total_cost_usd":0,"type":"result","usage":{"cache_creation":{"ephemeral_1h_input_tokens":0,"ephemeral_5m_input_tokens":0},"cache_creation_input_tokens":0,"cache_read_input_tokens":0,"inference_geo":"","input_tokens":0,"iterations":[],"output_tokens":0,"output_tokens_details":{"thinking_tokens":0},"server_tool_use":{"web_fetch_requests":0,"web_search_requests":0},"service_tier":"standard","speed":"standard"},"uuid":"<scrubbed>"}
JSON_EOF
)
CLAUDE_LIVE_MAX_TURNS=$(cat << 'JSON_EOF'
{"duration_api_ms":2561,"duration_ms":2707,"errors":["Reached maximum number of turns (1)"],"fast_mode_disabled_reason":"sdk_opt_in_required","fast_mode_state":"off","is_error":true,"modelUsage":{"claude-opus-5-5":{"cacheCreationInputTokens":0,"cacheReadInputTokens":19247,"canonicalModel":"claude-opus-5-5","contextWindow":1000000,"costBasis":"list","costUSD":0.0071774000000000004,"inputTokens":2,"maxOutputTokens":128000,"outputTokens":166,"provider":"firstParty","thinkingTokens":50,"webSearchRequests":0}},"num_turns":2,"permission_denials":[{"tool_input":{"command":"<scrubbed>","description":"<scrubbed>"},"tool_name":"Bash","tool_use_id":"<scrubbed>"}],"queued_turn_count":0,"result_index":0,"session_id":"<scrubbed>","stop_reason":"tool_use","subagent_stats":{"by_type":{},"completed":0,"failed":0,"killed":{"parent":0,"system":0,"user":0},"max_depth":0,"refused":{"budget":0,"concurrency_limit":0,"depth_limit":0},"requested":{"background":0,"foreground":0,"unset":0},"spawned":0,"spawned_by_subagents":0,"started_in_background":0},"subtype":"error_max_turns","terminal_reason":"max_turns","total_cost_usd":0.0071774000000000004,"type":"result","usage":{"cache_creation":{"ephemeral_1h_input_tokens":0,"ephemeral_5m_input_tokens":0},"cache_creation_input_tokens":0,"cache_read_input_tokens":19247,"inference_geo":"not_available","input_tokens":2,"iterations":[{"cache_creation":{"ephemeral_1h_input_tokens":0,"ephemeral_5m_input_tokens":0},"cache_creation_input_tokens":0,"cache_read_input_tokens":19247,"input_tokens":2,"output_tokens":166,"type":"message"}],"output_tokens":166,"output_tokens_details":{"thinking_tokens":50},"server_tool_use":{"web_fetch_requests":0,"web_search_requests":0},"service_tier":"standard","speed":"standard"},"uuid":"<scrubbed>"}
JSON_EOF
)
CLAUDE_LIVE_MAX_BUDGET=$(cat << 'JSON_EOF'
{"duration_api_ms":0,"duration_ms":3429,"errors":["Reached maximum budget ($0.0001)"],"fast_mode_disabled_reason":"sdk_opt_in_required","fast_mode_state":"off","is_error":true,"modelUsage":{"claude-opus-5-5":{"cacheCreationInputTokens":0,"cacheReadInputTokens":19261,"canonicalModel":"claude-opus-5-5","contextWindow":1000000,"costBasis":"list","costUSD":0.0080202,"inputTokens":2,"maxOutputTokens":128000,"outputTokens":208,"provider":"firstParty","thinkingTokens":170,"webSearchRequests":0}},"num_turns":1,"permission_denials":[],"queued_turn_count":0,"result_index":0,"session_id":"<scrubbed>","stop_reason":"end_turn","subagent_stats":{"by_type":{},"completed":0,"failed":0,"killed":{"parent":0,"system":0,"user":0},"max_depth":0,"refused":{"budget":0,"concurrency_limit":0,"depth_limit":0},"requested":{"background":0,"foreground":0,"unset":0},"spawned":0,"spawned_by_subagents":0,"started_in_background":0},"subtype":"error_max_budget_usd","terminal_reason":"budget_exhausted","total_cost_usd":0.0080202,"type":"result","usage":{"cache_creation":{"ephemeral_1h_input_tokens":0,"ephemeral_5m_input_tokens":0},"cache_creation_input_tokens":0,"cache_read_input_tokens":0,"inference_geo":"","input_tokens":0,"iterations":[],"output_tokens":0,"output_tokens_details":{"thinking_tokens":0},"server_tool_use":{"web_fetch_requests":0,"web_search_requests":0},"service_tier":"standard","speed":"standard"},"uuid":"<scrubbed>"}
JSON_EOF
)
CLAUDE_LIVE_SUCCESS=$(cat << 'JSON_EOF'
{"api_error_status":null,"duration_api_ms":2254,"duration_ms":2369,"fast_mode_disabled_reason":"sdk_opt_in_required","fast_mode_state":"off","first_content_frame_ms":834,"is_error":false,"modelUsage":{"claude-opus-5-5":{"cacheCreationInputTokens":7291,"cacheReadInputTokens":11945,"canonicalModel":"claude-opus-5-5","contextWindow":1000000,"costBasis":"list","costUSD":0.062805,"inputTokens":2,"maxOutputTokens":128000,"outputTokens":104,"provider":"firstParty","thinkingTokens":100,"webSearchRequests":0}},"num_turns":1,"permission_denials":[],"queued_turn_count":0,"result":"OK","result_index":0,"session_id":"<scrubbed>","stop_reason":"end_turn","subagent_stats":{"by_type":{},"completed":0,"failed":0,"killed":{"parent":0,"system":0,"user":0},"max_depth":0,"refused":{"budget":0,"concurrency_limit":0,"depth_limit":0},"requested":{"background":0,"foreground":0,"unset":0},"spawned":0,"spawned_by_subagents":0,"started_in_background":0},"subtype":"success","terminal_reason":"completed","time_to_request_ms":115,"total_cost_usd":0.062805,"ttft_ms":1835,"ttft_stream_ms":834,"type":"result","usage":{"cache_creation":{"ephemeral_1h_input_tokens":7291,"ephemeral_5m_input_tokens":0},"cache_creation_input_tokens":7291,"cache_read_input_tokens":11945,"inference_geo":"not_available","input_tokens":2,"iterations":[{"cache_creation":{"ephemeral_1h_input_tokens":7291,"ephemeral_5m_input_tokens":0},"cache_creation_input_tokens":7291,"cache_read_input_tokens":11945,"input_tokens":2,"output_tokens":104,"type":"message"}],"output_tokens":104,"output_tokens_details":{"thinking_tokens":100},"server_tool_use":{"web_fetch_requests":0,"web_search_requests":0},"service_tier":"standard","speed":"standard"},"uuid":"<scrubbed>"}
JSON_EOF
)

test_cli_claude_live_failure_shapes() {
    echo "TEST: claude 2.1.281's live failure shapes (exit 1) report the JSON reason, not just the exit code (#336)"
    setup
    make_mock_agent claude
    local out="${TMPDIR_BASE}/out.txt" ec content

    # Unknown --model: no `.errors`, reason in `.result`, subtype success.
    ec=$(MOCK_CLAUDE_RAW="$CLAUDE_LIVE_BAD_MODEL" MOCK_CLAUDE_EXIT=1 run_agents "$out" "claude")
    assert_exit_code "bad model exits 3" "3" "$ec"
    content=$(findings_of claude)
    assert_contains "bad model: reason carries .result" "issue with the selected model \(bogus-model-xyz\)" "$content"
    assert_contains "bad model: reason carries api_error_status 404" "api_error_status: 404" "$content"
    assert_contains "bad model: reason carries terminal_reason" "terminal_reason: api_error" "$content"
    assert_contains "bad model: exit code kept" "claude also exited 1" "$content"
    assert_not_contains "bad model: not the bare exit-code reason" "^claude exited 1" "$content"

    # --max-turns: `.errors` array, `.result` null.
    ec=$(MOCK_CLAUDE_RAW="$CLAUDE_LIVE_MAX_TURNS" MOCK_CLAUDE_EXIT=1 run_agents "$out" "claude")
    assert_exit_code "max turns exits 3" "3" "$ec"
    content=$(findings_of claude)
    assert_contains "max turns: reason carries .errors" "Reached maximum number of turns \(1\)" "$content"
    assert_contains "max turns: reason carries terminal_reason" "terminal_reason: max_turns" "$content"
    assert_contains "max turns: exit code kept" "claude also exited 1" "$content"

    # --max-budget-usd: `.errors` array, `.result` null.
    ec=$(MOCK_CLAUDE_RAW="$CLAUDE_LIVE_MAX_BUDGET" MOCK_CLAUDE_EXIT=1 run_agents "$out" "claude")
    assert_exit_code "max budget exits 3" "3" "$ec"
    content=$(findings_of claude)
    assert_contains "max budget: reason carries .errors" "Reached maximum budget \(\\\$0.0001\)" "$content"
    assert_contains "max budget: reason carries terminal_reason" "terminal_reason: budget_exhausted" "$content"

    # Control: the live success shape at exit 0 is still accepted.
    ec=$(MOCK_CLAUDE_RAW="$CLAUDE_LIVE_SUCCESS" MOCK_CLAUDE_EXIT=0 run_agents "$out" "claude")
    assert_exit_code "live success shape exits 0" "0" "$ec"
    content=$(findings_of claude)
    assert_contains "live success: the result is the review" "^OK$" "$content"
    assert_contains "live success: completion marker" "Review complete" "$content"
    teardown
}

test_cli_claude_exit_code_never_lost() {
    echo "TEST: reading claude's JSON first never loses a non-zero exit (#336)"
    setup
    make_mock_agent claude
    local out="${TMPDIR_BASE}/out.txt" ec content

    # No JSON at all + a kill-style exit: the exit code is the reason.
    ec=$(MOCK_CLAUDE_RAW='not json' MOCK_CLAUDE_EXIT=137 run_agents "$out" "claude")
    assert_exit_code "no JSON + exit 137 exits 3" "3" "$ec"
    content=$(findings_of claude)
    assert_contains "reason is the exit code" "claude exited 137" "$content"
    assert_not_contains "not the no-JSON reason" "did not emit a JSON result object" "$content"

    # A success-looking result with a non-zero exit is not trusted.
    ec=$(MOCK_CLAUDE_RAW="$CLAUDE_LIVE_SUCCESS" MOCK_CLAUDE_EXIT=4 run_agents "$out" "claude")
    assert_exit_code "success JSON + exit 4 exits 3" "3" "$ec"
    content=$(findings_of claude)
    assert_contains "reason names the exit despite the JSON" \
        "claude exited 4 despite a successful-looking JSON result" "$content"
    assert_not_contains "the result is not kept as a review" "^OK$" "$content"
    teardown
}

test_cli_claude_errors_field_type_safe() {
    echo "TEST: a non-string .errors element is reported, not a jq crash (#336)"
    setup
    make_mock_agent claude
    local out="${TMPDIR_BASE}/out.txt" ec content

    ec=$(MOCK_CLAUDE_RAW='{"type":"result","subtype":"error_during_execution","is_error":true,"result":"","errors":[{"code":1,"message":"boom"},"plain"]}' \
        run_agents "$out" "claude")
    assert_exit_code "object-element .errors exits 3" "3" "$ec"
    content=$(findings_of claude)
    assert_not_contains "the read did not fail" "could not be read" "$content"
    assert_contains "object element is kept as JSON" '\{"code":1,"message":"boom"\}; plain' "$content"

    # A non-array .errors falls back to tostring.
    ec=$(MOCK_CLAUDE_RAW='{"type":"result","subtype":"error_during_execution","is_error":true,"result":"","errors":"just a string"}' \
        run_agents "$out" "claude")
    assert_exit_code "string .errors exits 3" "3" "$ec"
    content=$(findings_of claude)
    assert_not_contains "string .errors: the read did not fail" "could not be read" "$content"
    assert_contains "string .errors is kept" "just a string" "$content"
    teardown
}

test_cli_codex_empty_response_excerpts_transcript() {
    echo "TEST: a codex empty result reports its transcript, not the unwritten stderr file (#313)"
    setup
    make_mock_agent codex
    local out="${TMPDIR_BASE}/out.txt" ec
    ec=$(MOCK_CODEX_EMPTY=1 run_agents "$out" "codex")
    assert_exit_code "empty codex result exits 3" "3" "$ec"
    local content; content=$(findings_of codex)
    assert_contains "reason names the empty response" "empty response" "$content"
    assert_contains "reason carries the transcript" "codex transcript \(last 20 lines\)" "$content"
    assert_contains "the transcript excerpt has content" "mock banner" "$content"
    teardown
}

# A CLI that ignores SIGTERM must still be ended by the helper, which
# escalates to SIGKILL rather than exiting and leaving it running — that
# would both defeat the caller's `timeout -k` backstop and pull TMP_DIR
# out from under a live CLI.
# Args: <label> <pid-file the mock writes> <command to run the helper...>
assert_term_escalation() {
    local label="$1" times="$2"
    shift 2
    local ec=0 i
    "$@" >/dev/null 2>&1 &
    local helper_pid=$!
    for ((i = 0; i < 60; i++)); do
        [[ -f "$times" ]] && break
        sleep 0.1
    done
    kill -TERM "$helper_pid" 2>/dev/null || true
    wait "$helper_pid" || ec=$?
    assert_exit_code "${label}: helper exits 143 after escalating" "143" "$ec"
    sleep 0.3
    local pid; pid=$(cat "$times" 2>/dev/null || echo "")
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
        kill -9 "$pid" 2>/dev/null || true
        echo "  FAIL: ${label}: the TERM-ignoring CLI survived the helper"; FAIL=$((FAIL + 1))
    else
        echo "  PASS: ${label}: the TERM-ignoring CLI was killed before the helper exited"; PASS=$((PASS + 1))
    fi
}

test_cli_claude_non_object_json_is_a_reported_failure() {
    echo "TEST: truthy non-object JSON from claude fails with a reason, not a silent jq crash (#313 round 2)"
    setup
    make_mock_agent claude
    local out="${TMPDIR_BASE}/out.txt" ec raw content
    # Each of these passes `jq -e .` but cannot be indexed: before the
    # type check, `.is_error` aborted jq and the helper died under set -e
    # leaving an EMPTY findings file under the caller's failure marker.
    for raw in '"oops"' '[]' '1' 'true' '"Error: quota exceeded"'; do
        ec=$(MOCK_CLAUDE_RAW="$raw" run_agents "$out" "claude")
        assert_exit_code "claude JSON ${raw} exits 3" "3" "$ec"
        content=$(findings_of claude)
        assert_contains "claude JSON ${raw}: reason recorded" "did not emit a JSON result object" "$content"
        assert_contains "claude JSON ${raw}: marked failed" "Review failed" "$content"
    done
    teardown
}

# The escalation watchdog is a subshell forked while the helper's
# terminate handler has INT/TERM/HUP ignored; an ignored disposition is
# inherited, so without a reset the `kill "$watchdog"` that cancels it
# after a clean CLI exit is a no-op and the watchdog outlives the helper
# by the whole escalation window, then `kill -9`s whatever process holds
# the dead CLI's PID by then (rolker/ros2_agent_workspace #660). The
# forked subshell keeps the helper's argv, so the per-test findings path
# identifies it.
assert_watchdog_cancelled() {
    local label="$1" findings="$2" i
    for ((i = 0; i < 10; i++)); do
        pgrep -f -- "$findings" >/dev/null || break
        sleep 0.1
    done
    if pgrep -f -- "$findings" >/dev/null; then
        echo "  FAIL: ${label}: escalation watchdog still running after a clean CLI exit"; FAIL=$((FAIL + 1))
        pkill -f -- "$findings" 2>/dev/null || true
    else
        echo "  PASS: ${label}: escalation watchdog cancelled with the helper"; PASS=$((PASS + 1))
    fi
}

test_cli_helper_returns_promptly_on_term() {
    echo "TEST: TERM to the helper returns at once when the CLI exits cleanly (#313 round 2, gemini 4)"
    setup
    make_mock_agent codex
    local times="${TMPDIR_BASE}/times"; mkdir -p "$times" "${TMPDIR_BASE}/helper-tmp"
    local prompt="${TMPDIR_BASE}/prompt.md" findings="${TMPDIR_BASE}/findings.md"
    echo "review this" > "$prompt"
    # A fast-exiting mock (no TERM trap): the escalation window is 5s, so
    # anything close to that means the handler waited on its watchdog.
    MOCK_CODEX_SLEEP=30 MOCK_TIMES_DIR="$times" REVIEW_KILL_ESCALATION=5s \
        TMPDIR="${TMPDIR_BASE}/helper-tmp" PATH="${MOCK_BIN}:${PATH}" \
        bash "$CLI_HELPER_UNDER_TEST" codex "${MOCK_BIN}/codex" "$prompt" "$findings" 1800 \
        >/dev/null 2>&1 &
    local helper_pid=$! i
    for ((i = 0; i < 60; i++)); do
        [[ -f "$times/codex.pid" ]] && break
        sleep 0.1
    done
    local t0 t1 ec=0
    t0=$(date +%s.%N)
    kill -TERM "$helper_pid" 2>/dev/null || true
    wait "$helper_pid" || ec=$?
    t1=$(date +%s.%N)
    assert_exit_code "helper exits 143" "143" "$ec"
    if awk -v a="$t0" -v b="$t1" 'BEGIN{exit !((b - a) < 3)}'; then
        echo "  PASS: helper returned well inside the 5s escalation window"; PASS=$((PASS + 1))
    else
        echo "  FAIL: helper waited out the escalation window after a clean CLI exit"; FAIL=$((FAIL + 1))
    fi
    assert_watchdog_cancelled "cli helper" "$findings"
    if pgrep -fx 'sleep 5s' >/dev/null; then
        echo "  FAIL: cli helper: the watchdog's sleep was left running"; FAIL=$((FAIL + 1))
    else
        echo "  PASS: cli helper: no orphan watchdog sleep"; PASS=$((PASS + 1))
    fi
    teardown
}

test_claude_unavailable_without_jq() {
    echo "TEST: claude is marked unavailable when jq is missing (#313 round 2, gemini 8)"
    setup
    make_mock_agent claude; make_mock_agent codex
    # A PATH with the mocks and the system tools but no jq: a wrapper dir
    # shadows jq with a non-executable stub so `command -v` misses it.
    local nojq="${TMPDIR_BASE}/nojq"; mkdir -p "$nojq"
    local realbin; realbin=$(mktemp -d -p "$TMPDIR_BASE")
    local tool
    for tool in bash env timeout mktemp cat tail head grep sed awk tr wc date sleep kill git dirname basename cut rm mkdir ls cp printf; do
        [[ -x "/usr/bin/${tool}" ]] && ln -sf "/usr/bin/${tool}" "${realbin}/${tool}"
        [[ -x "/bin/${tool}" && ! -e "${realbin}/${tool}" ]] && ln -sf "/bin/${tool}" "${realbin}/${tool}"
    done
    cd "${MOCK_REPO}"
    local out="${TMPDIR_BASE}/out.txt" ec=0
    PATH="${MOCK_BIN}:${realbin}" WORKTREE_ISSUE=42 bash "${SCRIPT_UNDER_TEST}" \
        --pr 99 --agents claude,codex < /dev/null > "$out" 2>/dev/null || ec=$?
    assert_exit_code "run without jq exits 3 (claude unavailable, codex fine)" "3" "$ec"
    assert_contains "claude findings name the missing jq" "jq is required" "$(findings_of claude)"
    assert_contains "claude marked failed" "Review failed" "$(findings_of claude)"
    assert_contains "codex still completed" "Review complete" "$(findings_of codex)"
    teardown
}

test_cleanup_reaps_jobs_before_dropping_tmp_root() {
    echo "TEST: an interrupted run reaps its jobs before removing the shared temp root (#313 round 2, codex 2)"
    setup
    make_mock_agent codex
    local times="${TMPDIR_BASE}/times"; mkdir -p "$times"
    local scratch="${TMPDIR_BASE}/scratch"; mkdir -p "$scratch"
    cd "${MOCK_REPO}"
    # A CLI that ignores TERM: its helper legitimately spends its
    # escalation window before exiting, and the parent must not drop
    # AGENT_TMP_ROOT (where that CLI's temp dir lives) until it has.
    MOCK_CODEX_IGNORE_TERM=1 MOCK_CODEX_SLEEP=30 MOCK_TIMES_DIR="$times" \
        REVIEW_KILL_ESCALATION=1 TMPDIR="$scratch" PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 \
        bash "${SCRIPT_UNDER_TEST}" --pr 99 --agents codex < /dev/null > /dev/null 2>&1 &
    local script_pid=$! i
    for ((i = 0; i < 60; i++)); do
        [[ -f "$times/codex.pid" ]] && break
        sleep 0.1
    done
    kill -TERM "$script_pid" 2>/dev/null || true
    local ec=0; wait "$script_pid" || ec=$?
    assert_exit_code "interrupted run exits 143" "143" "$ec"
    # The CLI is gone and nothing is left in the scratch root.
    sleep 0.3
    local pid; pid=$(cat "$times/codex.pid" 2>/dev/null || echo "")
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
        kill -9 "$pid" 2>/dev/null || true
        echo "  FAIL: the TERM-ignoring CLI outlived the interrupted run"; FAIL=$((FAIL + 1))
    else
        echo "  PASS: the TERM-ignoring CLI was killed before the parent exited"; PASS=$((PASS + 1))
    fi
    assert_eq "no temp files survive the interrupted run" "0" "$(ls -A "$scratch" | wc -l)"
    teardown
}

test_cleanup_survives_a_wedged_job() {
    echo "TEST: a wedged job cannot hang the exit path; the temp root is still removed (#313 round 3)"
    setup
    make_mock_agent codex
    local times="${TMPDIR_BASE}/times"; mkdir -p "$times"
    local scratch="${TMPDIR_BASE}/scratch"; mkdir -p "$scratch"
    # A WEDGED HELPER: a copy of the scripts dir whose _cli_review.sh
    # ignores every signal. Its job shell therefore never returns, which
    # is the only way a correctly-configured budget can expire — and the
    # case where cleanup used to `wait` unconditionally and hang forever.
    local fake_dir="${TMPDIR_BASE}/wedged"; mkdir -p "$fake_dir"
    cp "${SCRIPT_DIR}/.."/*.sh "$fake_dir/"
    cat > "${fake_dir}/_cli_review.sh" << 'WEDGED_EOF'
#!/usr/bin/env bash
# Stub helper that cannot be signalled. Self-limits so the suite never
# leaves it behind.
trap '' INT TERM HUP
echo "$$" > "${WEDGE_PID_FILE}"
: > "$4"
for ((i = 0; i < 200; i++)); do sleep 0.1; done
WEDGED_EOF
    chmod +x "${fake_dir}/_cli_review.sh"
    cd "${MOCK_REPO}"
    local t0; t0=$(date +%s)
    WEDGE_PID_FILE="${times}/wedge.pid" \
        AGENT_KILL_AFTER=90 REVIEW_KILL_ESCALATION=1 CLEANUP_REAP_TIMEOUT=2 \
        TMPDIR="$scratch" PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 \
        bash "${fake_dir}/cross_model_review.sh" --pr 99 --agents codex < /dev/null > /dev/null 2>&1 &
    local script_pid=$! i
    for ((i = 0; i < 80; i++)); do
        [[ -f "${times}/wedge.pid" ]] && break
        sleep 0.1
    done
    kill -TERM "$script_pid" 2>/dev/null || true
    local ec=0; wait "$script_pid" || ec=$?
    local elapsed=$(( $(date +%s) - t0 ))
    assert_exit_code "interrupted run still exits 143" "143" "$ec"
    if [[ "$elapsed" -lt 20 ]]; then
        echo "  PASS: exit path returned in ${elapsed}s, not blocked on the wedged job"; PASS=$((PASS + 1))
    else
        echo "  FAIL: exit path took ${elapsed}s — the wedged job blocked it"; FAIL=$((FAIL + 1))
    fi
    assert_eq "the shared temp root was removed anyway" "0" "$(ls -A "$scratch" | wc -l)"
    # The stub ignores TERM; cleanup's kill_tree (#660) now SIGKILLs it
    # with its job, so it should already be gone.
    local pid; pid=$(cat "${times}/wedge.pid" 2>/dev/null || echo "")
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
        echo "  FAIL: the wedged job's CLI outlived cleanup"; FAIL=$((FAIL + 1))
        kill -9 "$pid" 2>/dev/null || true
    else
        echo "  PASS: the wedged job's CLI was killed with its job"; PASS=$((PASS + 1))
    fi
    teardown
}

test_cleanup_budget_follows_the_escalation() {
    echo "TEST: the reap budget is derived from REVIEW_KILL_ESCALATION, not hard-coded (#313 round 3)"
    setup
    make_mock_agent codex
    local times="${TMPDIR_BASE}/times"; mkdir -p "$times"
    local scratch="${TMPDIR_BASE}/scratch"; mkdir -p "$scratch"
    cd "${MOCK_REPO}"
    # An escalation of 10s is past the old hard-coded 8s budget: with
    # that bug the parent gave up and dropped the temp root while the
    # helper was still escalating. Here the CLI ignores TERM for 4s of
    # real work, so the helper's SIGKILL (at 10s) is never reached —
    # what matters is that the parent waits for the helper rather than
    # timing out at 8s.
    local t0; t0=$(date +%s)
    MOCK_CODEX_SLEEP=4 MOCK_TIMES_DIR="$times" \
        AGENT_KILL_AFTER=20 REVIEW_KILL_ESCALATION=10 \
        TMPDIR="$scratch" PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 \
        bash "${SCRIPT_UNDER_TEST}" --pr 99 --agents codex < /dev/null > /dev/null 2>&1 &
    local script_pid=$! i
    for ((i = 0; i < 80; i++)); do
        [[ -f "$times/codex.pid" ]] && break
        sleep 0.1
    done
    kill -TERM "$script_pid" 2>/dev/null || true
    local ec=0; wait "$script_pid" || ec=$?
    local elapsed=$(( $(date +%s) - t0 ))
    assert_exit_code "interrupted run exits 143" "143" "$ec"
    assert_eq "temp root removed after the helper finished" "0" "$(ls -A "$scratch" | wc -l)"
    if [[ "$elapsed" -lt 20 ]]; then
        echo "  PASS: cleanup completed in ${elapsed}s with a 10s escalation"; PASS=$((PASS + 1))
    else
        echo "  FAIL: cleanup took ${elapsed}s"; FAIL=$((FAIL + 1))
    fi
    # A budget that does not clear the escalation is refused up front.
    local stderr ec2=0
    stderr=$(CLEANUP_REAP_TIMEOUT=3 REVIEW_KILL_ESCALATION=10 PATH="${MOCK_BIN}:${PATH}" \
        WORKTREE_ISSUE=42 bash "${SCRIPT_UNDER_TEST}" --pr 99 --agents codex 2>&1 >/dev/null) || ec2=$?
    assert_exit_code "a budget below the escalation exits 2" "2" "$ec2"
    assert_contains "message names both knobs" \
        "CLEANUP_REAP_TIMEOUT \(3\) must exceed REVIEW_KILL_ESCALATION \(10\)" "$stderr"
    teardown
}

test_job_finished_without_proc() {
    echo "TEST: job liveness does not depend on /proc (#313 round 3)"
    setup
    # job_finished is read straight out of the script and exercised with
    # /proc reads forced to fail, the way it behaves on macOS/BSD or in a
    # container without /proc. Bash's own job table must carry it: a
    # finished-but-unreaped child (still answering `kill -0`) has to be
    # reported as finished, or every cleanup burns the whole budget.
    local probe="${TMPDIR_BASE}/probe.sh"
    {
        echo '#!/usr/bin/env bash'
        echo 'set -uo pipefail'
        # Force the /proc branch to be unavailable.
        echo 'proc_state() { return 1; }'
        sed -n '/^job_finished() {$/,/^}$/p' "${SCRIPT_DIR}/../cross_model_review.sh"
        cat << 'PROBE_EOF'
sleep 30 &
live=$!
job_finished "$live" && echo "RUNNING-REPORTED-FINISHED" || echo "running: alive"
sleep 0.2 &
dead=$!
sleep 1   # the child has exited but has NOT been waited on yet
if job_finished "$dead"; then echo "dead: finished"; else echo "DEAD-REPORTED-ALIVE"; fi
kill "$live" 2>/dev/null; wait 2>/dev/null
PROBE_EOF
    } > "$probe"
    local output; output=$(bash "$probe" 2>&1)
    assert_contains "a running job is reported as running" "running: alive" "$output"
    assert_contains "an exited-but-unreaped job is reported as finished" "dead: finished" "$output"
    assert_not_contains "no misreport of a running job" "RUNNING-REPORTED-FINISHED" "$output"
    assert_not_contains "no misreport of a dead job" "DEAD-REPORTED-ALIVE" "$output"
    teardown
}

test_job_finished_proc_comm_with_space() {
    echo "TEST: job liveness via /proc survives a comm containing spaces (#313 round 4)"
    if [[ ! -r /proc/self/stat ]]; then
        echo "  SKIP: no /proc on this host"
        return 0
    fi
    setup
    # With bash's job table forced empty, job_finished falls back to
    # /proc/<pid>/stat. The comm field is parenthesised and may contain
    # spaces, so the state must be read after the last `)` — a fixed
    # field misreads both processes below: "a Z b" (running) would read
    # as Z, and "x y" (a zombie) would read as "y)".
    local bindir="${TMPDIR_BASE}/comm-bin"; mkdir -p "$bindir"
    cp "$(command -v sleep)" "${bindir}/a Z b"
    cp "$(command -v sleep)" "${bindir}/x y"
    local probe="${TMPDIR_BASE}/probe-proc.sh"
    {
        echo '#!/usr/bin/env bash'
        echo 'set -uo pipefail'
        echo 'jobs() { :; }'
        sed -n '/^proc_state() {$/,/^}$/p' "${SCRIPT_DIR}/../cross_model_review.sh"
        sed -n '/^job_finished() {$/,/^}$/p' "${SCRIPT_DIR}/../cross_model_review.sh"
        echo "bindir='${bindir}'"
        echo "pidfile='${TMPDIR_BASE}/zombie.pid'"
        cat << 'PROBE_EOF'
"${bindir}/a Z b" 30 &
live=$!
# The zombie must belong to a parent that never reaps it. A child of
# this bash would be reaped by bash's SIGCHLD handler during the sleep
# below, and job_finished would then return through `kill -0` without
# ever reading /proc. `exec sleep` replaces the sh, and sleep does not
# wait, so "x y" stays a zombie until its parent is killed.
sh -c '"$1" 0.2 & echo $! > "$2"; exec sleep 10' _ "${bindir}/x y" "$pidfile" &
reaper=$!
for _ in $(seq 50); do [[ -s "$pidfile" ]] && break; sleep 0.1; done
dead=$(< "$pidfile")
sleep 1   # "x y" has exited; its non-reaping parent keeps it a zombie
# Precondition: the zombie still answers kill -0, so the verdict below
# can only come from the /proc state read.
kill -0 "$dead" 2>/dev/null && echo "zombie: present" || echo "ZOMBIE-ALREADY-REAPED"
job_finished "$live" && echo "RUNNING-REPORTED-FINISHED" || echo "running: alive"
if job_finished "$dead"; then echo "dead: finished"; else echo "DEAD-REPORTED-ALIVE"; fi
# Killing the parent reparents the zombie to init, which reaps it.
kill "$live" "$reaper" 2>/dev/null; wait 2>/dev/null
PROBE_EOF
    } > "$probe"
    local output; output=$(bash "$probe" 2>&1)
    assert_contains "the 'x y' zombie is still unreaped when probed" "zombie: present" "$output"
    assert_contains "a running job named 'a Z b' is reported as running" "running: alive" "$output"
    assert_contains "a zombie named 'x y' is reported as finished" "dead: finished" "$output"
    assert_not_contains "no misreport of a running job" "RUNNING-REPORTED-FINISHED" "$output"
    assert_not_contains "no misreport of a dead job" "DEAD-REPORTED-ALIVE" "$output"
    teardown
}

test_cli_helper_escalates_to_sigkill() {
    echo "TEST: _cli_review.sh waits for a TERM-ignoring CLI and escalates to SIGKILL (#313)"
    setup
    make_mock_agent codex
    local times="${TMPDIR_BASE}/times"; mkdir -p "$times" "${TMPDIR_BASE}/helper-tmp"
    local prompt="${TMPDIR_BASE}/prompt.md" findings="${TMPDIR_BASE}/findings.md"
    echo "review this" > "$prompt"
    assert_term_escalation "_cli_review.sh" "$times/codex.pid" \
        env MOCK_CODEX_IGNORE_TERM=1 MOCK_CODEX_SLEEP=30 MOCK_TIMES_DIR="$times" \
        REVIEW_KILL_ESCALATION=1 TMPDIR="${TMPDIR_BASE}/helper-tmp" PATH="${MOCK_BIN}:${PATH}" \
        bash "$CLI_HELPER_UNDER_TEST" codex "${MOCK_BIN}/codex" "$prompt" "$findings" 1800
    assert_eq "no helper temp dir survives the escalation" "0" \
        "$(ls -A "${TMPDIR_BASE}/helper-tmp" | wc -l)"
    teardown
}

test_agy_helper_escalates_to_sigkill() {
    echo "TEST: _agy_review.sh waits for a TERM-ignoring agy and escalates to SIGKILL (#313)"
    setup
    local times="${TMPDIR_BASE}/times"; mkdir -p "$times" "${TMPDIR_BASE}/helper-tmp"
    local prompt="${TMPDIR_BASE}/prompt.md" findings="${TMPDIR_BASE}/findings.md"
    echo "review this" > "$prompt"
    assert_term_escalation "_agy_review.sh" "$times/agy.pid" \
        env MOCK_AGY_IGNORE_TERM=1 MOCK_AGY_STALL=1 MOCK_TIMES_DIR="$times" \
        REVIEW_KILL_ESCALATION=1 TMPDIR="${TMPDIR_BASE}/helper-tmp" PATH="${MOCK_BIN}:${PATH}" \
        bash "${SCRIPT_DIR}/../_agy_review.sh" "${MOCK_BIN}/agy" "$prompt" "$findings" 30m
    assert_eq "no agy helper temp dir survives the escalation" "0" \
        "$(ls -A "${TMPDIR_BASE}/helper-tmp" | wc -l)"
    teardown
}

test_cli_timeout_kills_the_cli() {
    echo "TEST: AGENT_TIMEOUT cuts off each CLI through the helper (#313)"
    setup
    local agent out exit_code times
    for agent in "${CLI_AGENTS[@]}"; do
        make_mock_agent "$agent"
        times="${TMPDIR_BASE}/times-${agent}"; mkdir -p "$times"
        out="${TMPDIR_BASE}/out-${agent}.txt"
        export "MOCK_${agent^^}_SLEEP=6"
        exit_code=$(AGENT_TIMEOUT=1 AGENT_KILL_AFTER=1 REVIEW_KILL_ESCALATION=0 MOCK_TIMES_DIR="$times" \
            run_agents "$out" "$agent")
        unset "MOCK_${agent^^}_SLEEP"
        assert_exit_code "${agent} timeout exits 3" "3" "$exit_code"
        assert_contains "${agent} EXIT=124" "^EXIT=124$" "$(cat "$out")"
        assert_contains "${agent} findings name the timeout" "timed out \(AGENT_TIMEOUT=1\)" "$(findings_of "$agent")"
        if [[ -f "$times/${agent}.end" ]]; then
            echo "  FAIL: ${agent} ran to completion despite AGENT_TIMEOUT=1"; FAIL=$((FAIL + 1))
        else
            echo "  PASS: ${agent} was killed before it could finish"; PASS=$((PASS + 1))
        fi
        # The CLI process itself must be gone, not merely abandoned: the
        # helper forwards the TERM to its child.
        sleep 0.3
        local pid; pid=$(cat "$times/${agent}.pid" 2>/dev/null || echo "")
        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
            echo "  FAIL: the ${agent} process survived the timeout"; FAIL=$((FAIL + 1))
        else
            echo "  PASS: the ${agent} process was killed"; PASS=$((PASS + 1))
        fi
    done
    teardown
}

test_cli_claude_json_contract() {
    echo "TEST: claude's JSON result is validated, not trusted (#313)"
    setup
    make_mock_agent claude
    local out="${TMPDIR_BASE}/out.txt" ec

    ec=$(MOCK_CLAUDE_RAW="not json at all" run_agents "$out" "claude")
    assert_exit_code "non-JSON output exits 3" "3" "$ec"
    assert_contains "reason names the missing JSON result" "did not emit a JSON result object" "$(findings_of claude)"
    assert_not_contains "the raw output is not kept as a review" "not json at all" \
        "$(head -n 1 "${MOCK_REPO}/.agent/work-plans/issue-42/review-claude-findings.md")"

    ec=$(MOCK_CLAUDE_IS_ERROR=true MOCK_CLAUDE_ERRMARK="something broke" run_agents "$out" "claude")
    assert_exit_code "is_error=true exits 3" "3" "$ec"
    assert_contains "reason names is_error" "is_error=true" "$(findings_of claude)"

    ec=$(MOCK_CLAUDE_SUBTYPE=error_during_execution run_agents "$out" "claude")
    assert_exit_code "non-success subtype exits 3" "3" "$ec"
    assert_contains "reason names the subtype" "subtype is 'error_during_execution'" "$(findings_of claude)"

    ec=$(run_agents "$out" "claude")
    assert_exit_code "a valid result exits 0" "0" "$ec"
    assert_contains "findings hold .result only" "reviewed by claude" "$(findings_of claude)"
    assert_not_contains "the JSON envelope is not in the findings" "subtype" "$(findings_of claude)"
    teardown
}

test_copilot_stdin_contract() {
    echo "TEST: copilot's prompt goes over stdin, never argv (#212, #274)"
    setup
    make_mock_agent copilot
    local argv="${TMPDIR_BASE}/argv" stdin="${TMPDIR_BASE}/stdin"
    mkdir -p "$argv" "$stdin"
    local out="${TMPDIR_BASE}/out.txt" ec
    ec=$(MOCK_ARGV_DIR="$argv" MOCK_STDIN_DIR="$stdin" run_agents "$out" "copilot")
    assert_exit_code "copilot run exits 0" "0" "$ec"
    local argv_text stdin_text
    argv_text=$(cat "$argv/copilot.argv")
    stdin_text=$(cat "$stdin/copilot.stdin")
    assert_not_contains "prompt content is NOT on argv" "Adversarial Code Review" "$argv_text"
    assert_contains "argv is the empty-prompt print form" "^-p$" "$argv_text"
    assert_contains "argv carries -s (response only)" "^-s$" "$argv_text"
    assert_contains "argv carries the empty tool set" "^--available-tools=$" "$argv_text"
    assert_contains "argv disables the built-in MCPs" "^--disable-builtin-mcps$" "$argv_text"
    assert_contains "argv disables the ask_user tool" "^--no-ask-user$" "$argv_text"
    assert_not_contains "argv never carries --allow-all-tools" "allow-all-tools" "$argv_text"
    assert_contains "the prompt arrived on stdin" "Adversarial Code Review" "$stdin_text"
    teardown
}

test_cli_no_prompt_size_ceiling() {
    echo "TEST: a >128 KiB prompt is reviewed, not rejected — stdin has no argv limit (#313, #212)"
    setup
    make_mock_agent copilot; make_mock_agent codex; make_mock_agent claude
    mkdir -p "${TMPDIR_BASE}/helper-tmp"
    # 128 KiB + 1: past MAX_ARG_STRLEN, where an argv form would
    # exec-fail. On the stdin path it must simply work — a size guard
    # here could only reject large reviews that would otherwise succeed.
    local big="${TMPDIR_BASE}/big-prompt.md" findings="${TMPDIR_BASE}/big-findings.md"
    head -c 131073 /dev/zero | tr '\0' 'x' > "$big"
    local agent ec stdin_dir="${TMPDIR_BASE}/bigstdin"
    mkdir -p "$stdin_dir"
    for agent in "${CLI_AGENTS[@]}"; do
        ec=$(MOCK_STDIN_DIR="$stdin_dir" run_cli_helper "$agent" "$big" "$findings" 1800)
        assert_exit_code "${agent} accepts a 128 KiB + 1 prompt" "0" "$ec"
        assert_eq "${agent} received every byte on stdin" "131073" \
            "$(wc -c < "${stdin_dir}/${agent}.stdin" | tr -d ' ')"
    done
    teardown
}

test_cli_helper_forwards_term() {
    echo "TEST: TERM to the helper kills its CLI child, not just the helper (#313)"
    setup
    make_mock_agent codex
    local times="${TMPDIR_BASE}/times"; mkdir -p "$times" "${TMPDIR_BASE}/helper-tmp"
    local prompt="${TMPDIR_BASE}/prompt.md" findings="${TMPDIR_BASE}/findings.md"
    echo "review this" > "$prompt"
    MOCK_CODEX_SLEEP=30 MOCK_TIMES_DIR="$times" TMPDIR="${TMPDIR_BASE}/helper-tmp" \
        PATH="${MOCK_BIN}:${PATH}" bash "$CLI_HELPER_UNDER_TEST" codex "${MOCK_BIN}/codex" \
        "$prompt" "$findings" 1800 >/dev/null 2>&1 &
    local helper_pid=$! i
    for ((i = 0; i < 50; i++)); do
        [[ -f "$times/codex.pid" ]] && break
        sleep 0.1
    done
    kill -TERM "$helper_pid" 2>/dev/null || true
    local ec=0; wait "$helper_pid" || ec=$?
    assert_exit_code "helper exits 143 on TERM" "143" "$ec"
    sleep 0.3
    local pid; pid=$(cat "$times/codex.pid" 2>/dev/null || echo "")
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
        kill "$pid" 2>/dev/null || true
        echo "  FAIL: the codex mock survived the helper's TERM"; FAIL=$((FAIL + 1))
    else
        echo "  PASS: the codex mock was killed with the helper"; PASS=$((PASS + 1))
    fi
    assert_eq "no helper temp dir survives the TERM" "0" "$(ls -A "${TMPDIR_BASE}/helper-tmp" | wc -l)"
    teardown
}

test_cli_findings_truncated_and_usage_errors() {
    echo "TEST: the helper truncates stale findings first and refuses bad usage (#313)"
    setup
    make_mock_agent codex
    mkdir -p "${TMPDIR_BASE}/helper-tmp"
    local prompt="${TMPDIR_BASE}/prompt.md" findings="${TMPDIR_BASE}/findings.md" ec
    echo "review this" > "$prompt"

    # Stale findings from a previous run must not survive a failure.
    echo "STALE FINDINGS FROM LAST RUN" > "$findings"
    ec=$(MOCK_CODEX_EXIT=7 run_cli_helper codex "$prompt" "$findings" 1800)
    assert_exit_code "crashed CLI exits 1" "1" "$ec"
    assert_not_contains "stale findings are gone" "STALE FINDINGS" "$(cat "$findings")"

    # An unknown agent is a usage error (exit 2) with the reason recorded.
    echo "STALE FINDINGS FROM LAST RUN" > "$findings"
    ec=0
    TMPDIR="${TMPDIR_BASE}/helper-tmp" bash "$CLI_HELPER_UNDER_TEST" grok "${MOCK_BIN}/codex" \
        "$prompt" "$findings" >/dev/null 2>&1 || ec=$?
    assert_exit_code "unknown agent exits 2" "2" "$ec"
    assert_not_contains "stale findings gone on a usage error too" "STALE FINDINGS" "$(cat "$findings")"
    assert_contains "reason names the unsupported agent" "unsupported agent 'grok'" "$(cat "$findings")"

    # A missing prompt file fails with a reason rather than running the CLI.
    ec=$(run_cli_helper codex "${TMPDIR_BASE}/nope.md" "$findings" 1800)
    assert_exit_code "missing prompt exits 1" "1" "$ec"
    assert_contains "reason names the prompt file" "prompt file not readable" "$(cat "$findings")"

    assert_eq "no helper temp files left behind" "0" "$(ls -A "${TMPDIR_BASE}/helper-tmp" | wc -l)"
    teardown
}

test_cli_no_temp_leak() {
    echo "TEST: _cli_review.sh leaves no temp files on success or failure (#313)"
    setup
    make_mock_agent codex; make_mock_agent claude; make_mock_agent copilot
    local leak_dir="${TMPDIR_BASE}/leakcheck"; mkdir -p "$leak_dir"
    local out="${TMPDIR_BASE}/out.txt"
    cd "${MOCK_REPO}"
    TMPDIR="$leak_dir" PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 \
        bash "${SCRIPT_UNDER_TEST}" --pr 99 --agents codex,claude,copilot \
        < /dev/null > "$out" 2>/dev/null || true
    MOCK_CODEX_EMPTY=1 MOCK_CLAUDE_EXIT=4 MOCK_COPILOT_ERRMARK="Error: rate limit reached" \
        TMPDIR="$leak_dir" PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 \
        bash "${SCRIPT_UNDER_TEST}" --pr 99 --agents codex,claude,copilot \
        < /dev/null > "$out" 2>/dev/null || true
    assert_eq "no temp files left after success + three failure modes" "" "$(ls -A "$leak_dir")"
    teardown
}

test_cli_helper_missing_is_unavailable() {
    echo "TEST: a missing _cli_review.sh makes only codex/claude/copilot unavailable (#313)"
    setup
    make_mock_agent codex
    # A copy of the scripts dir with _cli_review.sh removed: the precheck
    # must fail codex and leave gemini (whose own helper is still there)
    # untouched. The whole dir is copied because the script sources
    # siblings (_resolve_work_plans_dir.sh and friends) from beside itself.
    local fake_dir="${TMPDIR_BASE}/fakescripts"
    mkdir -p "$fake_dir"
    cp "${SCRIPT_DIR}/.."/*.sh "$fake_dir/"
    rm -f "${fake_dir}/_cli_review.sh"
    cd "${MOCK_REPO}"
    local out="${TMPDIR_BASE}/out.txt" ec=0
    PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 bash "${fake_dir}/cross_model_review.sh" \
        --pr 99 --agents gemini,codex < /dev/null > "$out" 2>/dev/null || ec=$?
    assert_exit_code "run with a missing helper exits 3" "3" "$ec"
    assert_contains "codex findings name the missing helper" "_cli_review.sh is missing or not executable" \
        "$(findings_of codex)"
    assert_contains "gemini still completed" "Review complete" "$(findings_of gemini)"
    teardown
}

# ---- Run all tests ----
echo "=== cross_model_review.sh tests ==="
echo ""

test_missing_pr_flag
test_unknown_argument
test_invalid_repo_slug
test_issue_extraction
test_repo_flag_accepted
test_work_dir_flag
test_empty_diff_guard
test_resolver_refuses_without_worktree_issue
test_flag_as_value_rejected
test_gh_repo_view_resolves_alias
test_gh_repo_view_failure_falls_back
test_issue_flag_overrides_extraction
test_missing_keyword_aborts
test_issue_flag_validates_integer
test_gh_pr_view_failure_distinct_error
test_agy_stdin_invocation
test_agy_large_prompt
test_agy_denial_is_failure
test_agy_timeout_is_failure
test_agy_partial_denial_is_noted
test_diff_fetch_failure_is_marked
test_agy_api_error_message_kept
test_agy_viewfile_denial_is_failure
test_agy_output_token_cutoff_is_named
test_agy_live_cutoff_fixture
test_agy_cutoff_phrase_in_response_not_misread
test_agy_cutoff_halves_on_separate_lines_not_misread
test_agy_findings_truncated
test_agy_no_temp_leak
test_shared_temp_no_leak_on_early_abort
test_prompt_tool_use_guidance
test_work_plans_excluded_from_diff
test_branch_mode_filter_survives_noprefix
test_diff_fence_context_line_pr_mode
test_diff_fence_four_backtick_run_pr_mode
test_diff_fence_branch_mode
test_plan_context_present
test_plan_context_absent_no_plan
test_plan_context_absent_no_approach_section
test_plan_context_truncated
test_plan_context_no_progress
test_plan_context_large_approach
test_plan_context_outer_fence_basic
test_plan_context_outer_fence_no_backticks
test_plan_context_outer_fence_info_string_inside
test_plan_context_outer_fence_four_backticks
test_plan_context_outer_fence_list_item_cut
test_plan_context_outer_fence_indented_code
test_plan_context_outer_fence_crlf
test_plan_context_outer_fence_longer_than_inner_run
test_plan_context_extractor_ignores_boundaries_in_fences
test_plan_context_extractor_long_rule
test_plan_context_extractor_unclosed_fence
test_plan_context_extractor_star_underscore_rules
test_plan_context_stops_at_h1_or_rule
test_plan_context_extractor_fence_after_closed_fence
test_plan_context_extractor_approach_in_earlier_fence
test_plan_context_extractor_unclosed_fence_before_approach
test_plan_context_extractor_indented_heading
test_plan_context_extractor_setext_headings
test_plan_context_parser_unavailable
test_plan_context_missing_extractor_is_an_error
test_require_plan_parser_fails_under_ci
test_sync_flag_rejected
test_agents_all_succeed
test_agents_partial_failure
test_agents_timeout
test_agents_concurrency
test_gemini_backstop_cuts_off_wedged_agy
test_gemini_not_bound_by_agent_timeout
test_duration_knobs_validated
test_agents_missing_binary
test_agents_none_usable
test_agents_argument_hygiene
test_agents_shared_diff_failure
test_agents_interrupt_kills_jobs
test_single_agent_output_unchanged
test_cli_codex_transcript_not_in_findings
test_cli_prompt_reaches_every_cli_on_stdin
test_cli_empty_response_is_failure
test_cli_nonzero_exit_is_failure
test_cli_error_text_explains_never_causes
test_cli_codex_transcript_marker_does_not_fail_the_review
test_cli_review_text_is_never_reclassified
test_cli_claude_error_payload_in_reason
test_cli_claude_live_failure_shapes
test_cli_claude_exit_code_never_lost
test_cli_claude_errors_field_type_safe
test_cli_claude_non_object_json_is_a_reported_failure
test_cli_helper_returns_promptly_on_term
test_claude_unavailable_without_jq
test_cleanup_reaps_jobs_before_dropping_tmp_root
test_cleanup_survives_a_wedged_job
test_cleanup_budget_follows_the_escalation
test_job_finished_without_proc
test_job_finished_proc_comm_with_space
test_cli_codex_empty_response_excerpts_transcript
test_cli_helper_escalates_to_sigkill
test_agy_helper_escalates_to_sigkill
test_cli_timeout_kills_the_cli
test_cli_claude_json_contract
test_copilot_stdin_contract
test_cli_no_prompt_size_ceiling
test_cli_helper_forwards_term
test_cli_findings_truncated_and_usage_errors
test_cli_no_temp_leak
test_cli_helper_missing_is_unavailable

# --- Local fixes to the port (rolker/ros2_agent_workspace #660) ---

extract_fn() { sed -n "/^$1()/,/^}/p" "$2"; }

# A child the CLI started, ignoring SIGTERM, must not outlive the helper:
# the helpers signal the CLI's whole process group, not just its PID
# (Codex cross-model review of PR #662). Needs setsid.
assert_orphan_gone() {
    local label="$1" pidfile="$2" i pid
    pid=$(cat "$pidfile" 2>/dev/null || echo "")
    if [[ -z "$pid" ]]; then
        echo "  FAIL: ${label}: the mock never started its child"; FAIL=$((FAIL + 1)); return
    fi
    for ((i = 0; i < 20; i++)); do kill -0 "$pid" 2>/dev/null || break; sleep 0.1; done
    if kill -0 "$pid" 2>/dev/null; then
        kill -9 "$pid" 2>/dev/null || true
        echo "  FAIL: ${label}: the CLI's TERM-ignoring child survived the helper"; FAIL=$((FAIL + 1))
    else
        echo "  PASS: ${label}: the CLI's TERM-ignoring child is gone"; PASS=$((PASS + 1))
    fi
}

test_local_helpers_kill_the_cli_process_group() {
    echo "TEST: the helpers kill a CLI's TERM-ignoring child, on TERM and on a normal exit (#660)"
    if ! command -v setsid >/dev/null 2>&1; then
        echo "  SKIP: no setsid on this host"; return
    fi
    setup
    make_mock_agent codex
    local times="${TMPDIR_BASE}/times"; mkdir -p "$times" "${TMPDIR_BASE}/helper-tmp"
    local prompt="${TMPDIR_BASE}/prompt.md" findings="${TMPDIR_BASE}/findings.md" i ec
    echo "review this" > "$prompt"

    # TERM path, codex: the CLI dies on TERM, its child does not.
    local orphan="${TMPDIR_BASE}/codex-orphan.pid"
    MOCK_CODEX_ORPHAN_PIDFILE="$orphan" MOCK_CODEX_SLEEP=30 MOCK_TIMES_DIR="$times" \
        REVIEW_KILL_ESCALATION=1 TMPDIR="${TMPDIR_BASE}/helper-tmp" PATH="${MOCK_BIN}:${PATH}" \
        bash "$CLI_HELPER_UNDER_TEST" codex "${MOCK_BIN}/codex" "$prompt" "$findings" 1800 \
        >/dev/null 2>&1 &
    local helper_pid=$!
    for ((i = 0; i < 50; i++)); do [[ -s "$orphan" ]] && break; sleep 0.1; done
    kill -TERM "$helper_pid" 2>/dev/null || true
    ec=0; wait "$helper_pid" || ec=$?
    assert_exit_code "codex helper exits 143 on TERM" "143" "$ec"
    assert_orphan_gone "codex, TERM" "$orphan"

    # Normal exit, codex: the review finished, its leftovers must not run on.
    orphan="${TMPDIR_BASE}/codex-orphan2.pid"
    ec=$(MOCK_CODEX_ORPHAN_PIDFILE="$orphan" run_cli_helper codex "$prompt" "$findings" 1800)
    assert_exit_code "codex review with a leftover child still succeeds" "0" "$ec"
    assert_orphan_gone "codex, normal exit" "$orphan"

    # TERM path, agy.
    orphan="${TMPDIR_BASE}/agy-orphan.pid"
    MOCK_AGY_ORPHAN_PIDFILE="$orphan" MOCK_AGY_STALL=1 MOCK_TIMES_DIR="$times" \
        REVIEW_KILL_ESCALATION=1 TMPDIR="${TMPDIR_BASE}/helper-tmp" PATH="${MOCK_BIN}:${PATH}" \
        bash "${SCRIPT_DIR}/../_agy_review.sh" "${MOCK_BIN}/agy" "$prompt" "$findings" 30m \
        >/dev/null 2>&1 &
    helper_pid=$!
    for ((i = 0; i < 50; i++)); do [[ -s "$orphan" ]] && break; sleep 0.1; done
    kill -TERM "$helper_pid" 2>/dev/null || true
    ec=0; wait "$helper_pid" || ec=$?
    assert_exit_code "agy helper exits 143 on TERM" "143" "$ec"
    assert_orphan_gone "agy, TERM" "$orphan"
    teardown
}

test_local_concurrent_runs_refused() {
    echo "TEST: a second run into the same artifact dir is refused before it writes (#660)"
    if ! command -v flock >/dev/null 2>&1; then
        echo "  SKIP: no flock on this host"; return
    fi
    setup
    make_mock_agent codex
    local dir="${MOCK_REPO}/.agent/work-plans/issue-42" ec=0 err
    mkdir -p "$dir"
    echo "first run's findings" > "${dir}/review-codex-findings.md"
    # Stand in for a first run holding the lock for 5s.
    flock "${dir}/.cross-model-review.lock" sleep 5 &
    local holder=$! i
    for ((i = 0; i < 20; i++)); do
        flock -n "${dir}/.cross-model-review.lock" true 2>/dev/null || break
        sleep 0.1
    done
    cd "${MOCK_REPO}"
    err=$(PATH="${MOCK_BIN}:${PATH}" WORKTREE_ISSUE=42 bash "${SCRIPT_UNDER_TEST}" \
        --pr 99 --agents codex 2>&1 >/dev/null) || ec=$?
    kill "$holder" 2>/dev/null || true
    wait "$holder" 2>/dev/null || true
    assert_exit_code "second run exits 5" "5" "$ec"
    assert_contains "names the other run" "already reviewing into" "$err"
    assert_eq "first run's findings untouched" "first run's findings" \
        "$(cat "${dir}/review-codex-findings.md")"
    teardown
}

test_local_branch_short_flag_not_swallowed() {
    echo "TEST: --branch does not take a following short flag as its base ref (#660)"
    setup
    cd "${MOCK_REPO}"
    local ec=0 err
    err=$(PATH="${MOCK_BIN}:${PATH}" bash "${SCRIPT_UNDER_TEST}" --branch -R owner/repo --agents nosuch 2>&1 >/dev/null) || ec=$?
    assert_exit_code "exits 2 on the unknown agent" "2" "$ec"
    assert_not_contains "-R owner/repo parsed as --repo" "Unknown argument" "$err"
    assert_contains "reaches agent validation" "Unknown agent 'nosuch'" "$err"
    teardown
}

test_local_home_unset() {
    echo "TEST: an unset HOME does not abort CLI discovery (#660)"
    setup
    cd "${MOCK_REPO}"
    local ec=0 err
    err=$(env -u HOME PATH="$(HIDDEN_CLI_PATH)" WORKTREE_ISSUE=42 bash "${SCRIPT_UNDER_TEST}" \
        --pr 99 --agents gemini 2>&1 >/dev/null) || ec=$?
    [[ "$ec" == 0 || "$ec" == 1 ]] \
        && { echo "  PASS: ran to a normal outcome (exit ${ec})"; PASS=$((PASS + 1)); } \
        || { echo "  FAIL: exit ${ec} with HOME unset: ${err}"; FAIL=$((FAIL + 1)); }
    assert_not_contains "no unbound-variable abort" "unbound variable" "$err"
    teardown
}

test_local_work_dir_conflicts_with_no_progress() {
    echo "TEST: --work-dir with --no-progress is rejected (#660)"
    setup
    cd "${MOCK_REPO}"
    local ec=0 err
    # No codex CLI anywhere on the search path, as on a hosted runner: the
    # usage error must win over CLI discovery's exit 1.
    err=$(HOME="${TMPDIR_BASE}/nohome" PATH="$(HIDDEN_CLI_PATH)" bash "${SCRIPT_UNDER_TEST}" \
        --pr 99 --no-progress --work-dir "${MOCK_REPO}" --agents codex 2>&1 >/dev/null) || ec=$?
    assert_exit_code "exits 2" "2" "$ec"
    assert_contains "names the conflict" "conflict" "$err"
    [[ ! -e "${MOCK_REPO}/.agent/work-plans/issue-noprogress" ]] \
        && { echo "  PASS: no issue-noprogress dir written"; PASS=$((PASS + 1)); } \
        || { echo "  FAIL: issue-noprogress dir written"; FAIL=$((FAIL + 1)); }
    teardown
}

test_local_job_finished_without_proc() {
    echo "TEST: job_finished reads an exited job as finished without /proc (#660)"
    local rc=0
    bash -c "$(extract_fn job_finished "${SCRIPT_UNDER_TEST}")"'
        proc_state() { return 1; }
        true & p=$!
        sleep 0.3
        job_finished "$p"' || rc=$?
    assert_exit_code "exited, unreaped job is finished" "0" "$rc"
    rc=0
    bash -c "$(extract_fn job_finished "${SCRIPT_UNDER_TEST}")"'
        proc_state() { return 1; }
        sleep 5 & p=$!
        job_finished "$p"; r=$?; kill "$p"; exit "$r"' || rc=$?
    assert_exit_code "running job is not finished" "1" "$rc"
}

test_local_kill_tree() {
    echo "TEST: kill_tree kills a job's descendants, not just the job (#660)"
    setup
    local marker="${TMPDIR_BASE}/kt-marker"
    bash -c "$(extract_fn kill_tree "${SCRIPT_UNDER_TEST}")"'
        bash -c "bash -c \"trap \\\"\\\" TERM; exec -a '"${marker}"' sleep 30\" & wait" &
        job=$!
        sleep 0.5
        kill_tree "$job"' >/dev/null 2>&1
    sleep 0.3
    if pgrep -f -- "$marker" >/dev/null; then
        echo "  FAIL: a descendant survived kill_tree"; FAIL=$((FAIL + 1))
        pkill -9 -f -- "$marker" || true
    else
        echo "  PASS: no descendant survived kill_tree"; PASS=$((PASS + 1))
    fi
    teardown
}

test_local_no_path_format() {
    echo "TEST: the plan-context lookup works on git without --path-format (#660)"
    setup
    make_mock_agent codex
    local shim="${TMPDIR_BASE}/gitshim" log="${TMPDIR_BASE}/pathformat.log" realgit
    realgit=$(command -v git)
    mkdir -p "$shim"
    cat > "${shim}/git" <<SHIM
#!/usr/bin/env bash
for a in "\$@"; do [[ "\$a" == --path-format* ]] && { echo "\$*" >> "${log}"; echo "unknown option" >&2; exit 129; }; done
exec "${realgit}" "\$@"
SHIM
    chmod +x "${shim}/git"
    mkdir -p "${MOCK_REPO}/.agent/work-plans/issue-42"
    printf '# Plan\n\n## Approach\n\nDo the thing.\n' > "${MOCK_REPO}/.agent/work-plans/issue-42/plan.md"
    local out="${TMPDIR_BASE}/out.txt"
    RUN_AGENTS_PATH="${shim}:${MOCK_BIN}:${PATH}" run_agents "$out" codex >/dev/null
    [[ ! -s "$log" ]] \
        && { echo "  PASS: git --path-format never called"; PASS=$((PASS + 1)); } \
        || { echo "  FAIL: git --path-format still called: $(cat "$log")"; FAIL=$((FAIL + 1)); }
    teardown
}

test_local_slugless_worktree_name() {
    echo "TEST: a worktree named issue-<N> (no slug) resolves its work-plans dir (#660)"
    setup
    local wt="${TMPDIR_BASE}/issue-42" out rc=0
    mkdir -p "$wt"; git -C "$wt" init -q -b scratch
    out=$(cd "$wt" && unset WORKTREE_ISSUE && source "${SCRIPT_DIR}/../_resolve_work_plans_dir.sh" \
        && resolve_work_plans_dir 42 2>/dev/null) || rc=$?
    assert_exit_code "resolves" "0" "$rc"
    assert_contains "points into the worktree" "issue-42/.agent/work-plans/issue-42" "$out"
    teardown
}

test_local_default_branch_prefers_fresher_origin() {
    echo "TEST: the default-branch resolver skips a local branch strictly behind origin (#660)"
    setup
    local bare="${TMPDIR_BASE}/remote.git" wc="${TMPDIR_BASE}/wc" up="${TMPDIR_BASE}/up"
    cd "$TMPDIR_BASE"
    g() { git -c user.name=t -c user.email=t@t "$@"; }
    git init -q --bare -b main "$bare"
    g clone -q "$bare" "$up" 2>/dev/null; g -C "$up" commit -q --allow-empty -m a; g -C "$up" push -q origin HEAD:main
    g clone -q "$bare" "$wc"; git -C "$wc" remote set-head origin main
    resolve() { (source "${SCRIPT_DIR}/../_resolve_default_branch.sh" && resolve_default_branch "$wc"); }
    assert_eq "equal: local" "main" "$(resolve)"
    g -C "$up" commit -q --allow-empty -m b; g -C "$up" push -q origin HEAD:main; git -C "$wc" fetch -q
    assert_eq "behind: origin" "origin/main" "$(resolve)"
    g -C "$wc" commit -q --allow-empty -m local
    assert_eq "diverged: local" "main" "$(resolve)"
    teardown
}

test_local_claude_preamble() {
    echo "TEST: a text line before claude's JSON result does not fail the review (#660)"
    setup
    make_mock_agent claude; make_mock_agent codex
    local out="${TMPDIR_BASE}/out.txt" ec
    ec=$(MOCK_CLAUDE_RAW=$'Update available: 9.9.9\n{"type":"result","subtype":"success","is_error":false,"result":"### Findings\\nok"}' \
        run_agents "$out" claude)
    assert_exit_code "claude with a preamble line succeeds" "0" "$ec"
    assert_contains "result kept" "Findings" "$(findings_of claude)"
    teardown
}

test_local_codex_env_allowlist() {
    echo "TEST: codex runs with an environment allowlist (#660)"
    setup
    make_mock_agent codex
    local out="${TMPDIR_BASE}/out.txt" dump="${TMPDIR_BASE}/codex.env" ec
    ec=$(GH_TOKEN=secret-gh AWS_SECRET_ACCESS_KEY=secret-aws OPENAI_API_KEY=k MOCK_CODEX_ENV_DUMP="$dump" \
        run_agents "$out" codex)
    assert_exit_code "codex succeeds" "0" "$ec"
    local envs; envs=$(cat "$dump" 2>/dev/null)
    assert_not_contains "GH_TOKEN stripped" "secret-gh" "$envs"
    assert_not_contains "AWS secret stripped" "secret-aws" "$envs"
    assert_contains "OPENAI_API_KEY kept" "OPENAI_API_KEY=k" "$envs"
    assert_contains "HOME kept" "HOME=" "$envs"
    teardown
}

test_local_branch_short_flag_not_swallowed
test_local_home_unset
test_local_work_dir_conflicts_with_no_progress
test_local_job_finished_without_proc
test_local_kill_tree
test_local_no_path_format
test_local_slugless_worktree_name
test_local_default_branch_prefers_fresher_origin
test_local_claude_preamble
test_local_codex_env_allowlist
test_local_helpers_kill_the_cli_process_group
test_local_concurrent_runs_refused

echo ""
echo "=== Results: ${PASS} passed, ${FAIL} failed ==="

if [[ "$FAIL" -gt 0 ]]; then
    exit 1
fi
