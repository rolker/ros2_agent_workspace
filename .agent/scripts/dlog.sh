#!/bin/bash
# .agent/scripts/dlog.sh
# Prompt-free, accurate-timestamp appender for deployment live-ops logging.
#
# Bakes the timestamp from `date` at WRITE time (never typed, never
# approval-delayed) and appends one entry to a deployment log file. Allowlist
# this ONE script -- Bash(<workspace_root>/.agent/scripts/dlog.sh:*) -- and
# every log entry is then prompt-free AND carries a measured timestamp.
#
# Why a script (not the Edit tool): the Edit tool avoids per-entry permission
# prompts (#516) but cannot run `date`, so the agent must TYPE the timestamp --
# reintroducing the inaccurate / "durable lie" times that #515 exists to kill.
# Only a shell can stamp the real write time. This helper does both: one
# standing allowlist entry (prompt-free) + a measured `date` stamp (accurate).
# Generic + committed, so it is NOT re-created per deployment (the abandoned
# per-deployment dlog_<N>.sh approach #515 rightly flags as clunky).
#
# Usage:
#   .agent/scripts/dlog.sh <logfile> <message...>
#
# Examples:
#   .agent/scripts/dlog.sh docs/logs/2026/2026-06-20_dev_logs.md "in water; controls good"
#   .agent/scripts/dlog.sh "$LOGFILE" "boat at dock (operator-reported ~12:35)"
#
# For an event you did NOT time yourself, let this stamp the real write time and
# put the operator's reported time in the message text, e.g.
# "... (operator-reported ~12:35)" -- never back-date the measured stamp.

set -euo pipefail

# Executed, not sourced.
if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
    echo "dlog.sh must be executed, not sourced" >&2
    return 1 2>/dev/null || exit 1
fi

if [[ $# -lt 2 ]]; then
    echo "usage: dlog.sh <logfile> <message...>" >&2
    exit 2
fi

LOGFILE="$1"
shift
MSG="$*"

# Reject an empty / whitespace-only message -- writing a contentless entry is a
# silent live-ops logging failure (the contract is "fail loud on bad usage").
if [[ ! "$MSG" =~ [^[:space:]] ]]; then
    echo "dlog.sh: empty log message" >&2
    exit 2
fi

if [[ ! -d "$(dirname "$LOGFILE")" ]]; then
    echo "dlog.sh: log directory does not exist: $(dirname "$LOGFILE")" >&2
    exit 1
fi

# %:z (colon timezone offset, e.g. -04:00) is a GNU coreutils `date` extension --
# fine on the Linux dev/field hosts; would emit a literal "%:z" on BSD/macOS.
TS="$(date '+%Y-%m-%d %H:%M %:z')"
printf '\n**%s** — %s\n' "$TS" "$MSG" >> "$LOGFILE"
printf 'logged: %s — %s\n' "$TS" "$MSG"
