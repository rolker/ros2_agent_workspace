#!/bin/bash
# .agent/scripts/tests/test_redact.sh
# Dedicated tests for .agent/scripts/redact.sh — the sole redaction test
# surface named as missing in rolker/ros2_agent_workspace#626. Prior coverage
# lived only inline inside test_resolve_repo_checkout.sh (a handful of simple
# cases); this file is the discoverable, sourced-function test suite for
# redact_url/redact_text themselves, hermetic and with no network or fixture
# workspace required.
#
# Shape matches the other test_*.sh files in this directory: pass/fail
# counters, a summary line, and a non-zero exit if anything failed.

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REAL_SCRIPTS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

TEST_PASS=0
TEST_FAIL=0
pass() { echo "✅ PASS: $1"; TEST_PASS=$((TEST_PASS + 1)); }
fail() { echo "❌ FAIL: $1"; TEST_FAIL=$((TEST_FAIL + 1)); }

# shellcheck source=../redact.sh
source "$REAL_SCRIPTS_DIR/redact.sh"

# --- redact_url: simple cases -------------------------------------------------
if [ "$(redact_url 'https://bob:hunter2@github.com/o/r.git')" = 'https://<redacted>@github.com/o/r.git' ]; then
    pass "redact_url: a single-@ userinfo is replaced"
else
    fail "redact_url: single-@ userinfo not replaced: $(redact_url 'https://bob:hunter2@github.com/o/r.git')"
fi

if [ "$(redact_url 'ssh://bob:hunter2@example.com/o/r.git')" = 'ssh://<redacted>@example.com/o/r.git' ]; then
    pass "redact_url: ssh:// scheme is handled the same as https://"
else
    fail "redact_url: ssh scheme not redacted: $(redact_url 'ssh://bob:hunter2@example.com/o/r.git')"
fi

# scp-style urls have no password field; left readable.
if [ "$(redact_url 'git@github.com:o/r.git')" = 'git@github.com:o/r.git' ]; then
    pass "redact_url: scp-form url is left untouched (no password field)"
else
    fail "redact_url: scp-form url was altered: $(redact_url 'git@github.com:o/r.git')"
fi

if [ "$(redact_url 'https://github.com/o/r.git')" = 'https://github.com/o/r.git' ]; then
    pass "redact_url: a url with no userinfo at all is left untouched"
else
    fail "redact_url: no-userinfo url was altered: $(redact_url 'https://github.com/o/r.git')"
fi

# --- redact_url / redact_text: an @ INSIDE the password ----------------------
# The userinfo character class must reach the LAST @ before the host, not the
# first — a password containing a literal @ previously left its own tail
# ("ss@host") unredacted.
if [ "$(redact_url 'https://bob:p@ss@github.com/o/r.git')" = 'https://<redacted>@github.com/o/r.git' ]; then
    pass "redact_url: an @ inside the password is redacted through to the host"
else
    fail "redact_url: password @ leaked through: $(redact_url 'https://bob:p@ss@github.com/o/r.git')"
fi

got=$(redact_text 'fatal: could not read from https://bob:p@ss@github.com/o/r.git')
case "$got" in
    *'ss@github.com'*) fail "redact_text: password @ leaked through: $got" ;;
    *'<redacted>@github.com'*) pass "redact_text: an @ inside the password is redacted through to the host" ;;
    *) fail "redact_text: unexpected output for password-@ case: $got" ;;
esac

# --- redact_text: two distinct urls on one line ------------------------------
# The widened match must not merge two different urls' credentials into one:
# every scheme:// prefix contains a literal /, which halts the greedy run for
# the first url's userinfo before it can reach into the second url.
got=$(redact_text 'fatal: could not read from https://bob:hunter2@github.com/o/r.git or from ssh://alice:secret@example.com/o/r2.git')
if case "$got" in *hunter2*) true ;; *) false ;; esac; then
    fail "redact_text: first url's credential leaked: $got"
elif case "$got" in *secret*) true ;; *) false ;; esac; then
    fail "redact_text: second url's credential leaked: $got"
elif case "$got" in *'<redacted>@github.com'*'<redacted>@example.com'*) true ;; *) false ;; esac; then
    pass "redact_text: two urls with credentials on one line are both redacted, independently"
else
    fail "redact_text: two-url case did not redact both independently: $got"
fi

# --- redact_text: REDACT_PATH_PREFIXES ---------------------------------------
# shellcheck disable=SC2034  # read by redact_text
REDACT_PATH_PREFIXES=("/home/someone/ws=<workspace>" "/home/someone=~")
if [ "$(redact_text 'failed at /home/someone/ws/.agent/scratchpad/x')" = 'failed at <workspace>/.agent/scratchpad/x' ]; then
    pass "redact_text: the most specific path prefix wins"
else
    fail "redact_text: workspace prefix not applied: $(redact_text 'failed at /home/someone/ws/.agent/scratchpad/x')"
fi
if [ "$(redact_text 'failed at /home/someone/elsewhere')" = 'failed at ~/elsewhere' ]; then
    pass "redact_text: the HOME prefix applies outside the workspace prefix"
else
    fail "redact_text: HOME prefix not applied: $(redact_text 'failed at /home/someone/elsewhere')"
fi

# A prefix whose VALUE contains '=' must still be recognised as one spec, with
# the replacement taken from after the LAST '=' — splitting on the first '='
# (the pre-fix behavior) corrupted the prefix and left the path unmatched.
# shellcheck disable=SC2034
REDACT_PATH_PREFIXES=("/home/build=1.0/workspace=<workspace>")
if [ "$(redact_text 'failed at /home/build=1.0/workspace/.agent/scratchpad/x')" = 'failed at <workspace>/.agent/scratchpad/x' ]; then
    pass "redact_text: a prefix containing '=' is split on the LAST '=', not the first"
else
    fail "redact_text: '='-in-prefix case not handled: $(redact_text 'failed at /home/build=1.0/workspace/.agent/scratchpad/x')"
fi

# No prefixes configured, and no url in the string: returned unchanged.
unset REDACT_PATH_PREFIXES
if [ "$(redact_text 'plain reason, nothing to strip')" = 'plain reason, nothing to strip' ]; then
    pass "redact_text: a plain string with nothing to redact is returned unchanged"
else
    fail "redact_text: plain string was altered: $(redact_text 'plain reason, nothing to strip')"
fi

# A bare '/' prefix would rewrite every path separator in the string — refused.
# shellcheck disable=SC2034
REDACT_PATH_PREFIXES=("/=<root>")
if [ "$(redact_text 'failed at /home/someone/x')" = 'failed at /home/someone/x' ]; then
    pass "redact_text: a bare '/' prefix is refused, not applied"
else
    fail "redact_text: bare '/' prefix was applied: $(redact_text 'failed at /home/someone/x')"
fi
unset REDACT_PATH_PREFIXES

# A LABEL containing `=`: the old last-`=` split turned this spec into
# prefix `/home/someone=<workspace` + label `main>`, the prefix never matched,
# and the full path leaked with no error.
# shellcheck disable=SC2034
REDACT_PATH_PREFIXES=("/home/someone=<workspace=main>")
if [ "$(redact_text 'failed at /home/someone/ws/src/x')" = 'failed at <workspace=main>/ws/src/x' ]; then
    pass "redact_text: a label containing '=' still redacts the path"
else
    fail "redact_text: label containing '=' leaked: $(redact_text 'failed at /home/someone/ws/src/x')"
fi
# Both sides containing `=` at once: parsed by the label's shape, not a fixed `=`.
# shellcheck disable=SC2034
REDACT_PATH_PREFIXES=("/tmp/a=b=<x=y>")
if [ "$(redact_text 'see /tmp/a=b/file')" = 'see <x=y>/file' ]; then
    pass "redact_text: '=' in both the path and the label is parsed by the label's shape"
else
    fail "redact_text: '=' in path and label: $(redact_text 'see /tmp/a=b/file')"
fi
# The `~` label form, as the skills use for $HOME.
# shellcheck disable=SC2034
REDACT_PATH_PREFIXES=("/home/some=one=~")
if [ "$(redact_text 'in /home/some=one/x')" = 'in ~/x' ]; then
    pass "redact_text: '~' label with '=' in the path"
else
    fail "redact_text: '~' label with '=' in the path: $(redact_text 'in /home/some=one/x')"
fi
unset REDACT_PATH_PREFIXES

# --- redact.sh executed rather than sourced -----------------------------------
out=$(bash "$REAL_SCRIPTS_DIR/redact.sh" 2>&1); rc=$?
if [ "$rc" -eq 2 ] && [ -n "$out" ]; then
    pass "redact.sh executed directly → exit 2 with a reason, never a silent 0"
else
    fail "redact.sh executed directly: rc=$rc out='$out' (expected 2 / non-empty)"
fi

echo ""
echo "Passed: $TEST_PASS  Failed: $TEST_FAIL"
[ "$TEST_FAIL" -eq 0 ]
