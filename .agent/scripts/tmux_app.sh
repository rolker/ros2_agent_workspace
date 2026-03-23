#!/usr/bin/env bash
# tmux_app.sh — Launch, list, and stop named app sessions for agent worktrees.
#
# Sessions are named issue-<N>-<label> so the dashboard can discover and display them.
#
# Usage:
#   tmux_app.sh --issue 46 --label ros2launch -- ros2 launch my_pkg my_launch.py
#   tmux_app.sh --list --issue 46
#   tmux_app.sh --stop --issue 46 --label ros2launch
#
# Guard: must be executed, not sourced.
if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
    echo "ERROR: tmux_app.sh must be executed, not sourced." >&2
    return 1 2>/dev/null || exit 1
fi

set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  tmux_app.sh --issue <N> --label <label> -- <command...>   Launch an app session
  tmux_app.sh --list --issue <N>                            List app sessions for an issue
  tmux_app.sh --stop --issue <N> --label <label>            Stop an app session

Options:
  --issue <N>       Issue number (required)
  --label <label>   Short label for the session (required for launch/stop)
  --list            List mode: show app sessions for the given issue
  --stop            Stop mode: terminate an app session
  --help            Show this help
EOF
}

MODE="launch"
ISSUE=""
LABEL=""
CMD_ARGS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --issue)  ISSUE="$2"; shift 2 ;;
        --label)  LABEL="$2"; shift 2 ;;
        --list)   MODE="list"; shift ;;
        --stop)   MODE="stop"; shift ;;
        --help)   usage; exit 0 ;;
        --)       shift; CMD_ARGS=("$@"); break ;;
        *)        echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
    esac
done

if [[ -z "$ISSUE" ]]; then
    echo "ERROR: --issue is required." >&2
    exit 1
fi

if ! [[ "$ISSUE" =~ ^[0-9]+$ ]]; then
    echo "ERROR: --issue must be numeric, got: $ISSUE" >&2
    exit 1
fi

# Validate label format (alphanumeric + hyphens only)
validate_label() {
    if [[ ! "$1" =~ ^[a-zA-Z0-9]([a-zA-Z0-9-]*[a-zA-Z0-9])?$ ]]; then
        echo "ERROR: Label must be alphanumeric (hyphens allowed, not at start/end): $1" >&2
        exit 1
    fi
}

if ! command -v tmux &>/dev/null; then
    echo "ERROR: tmux is required but not found." >&2
    exit 1
fi

SESSION_PREFIX="issue-${ISSUE}-"

case "$MODE" in
    list)
        # List all tmux sessions matching the issue prefix
        tmux list-sessions -F "#{session_name}" 2>/dev/null \
            | grep "^${SESSION_PREFIX}" \
            || echo "(no app sessions for issue $ISSUE)"
        ;;

    stop)
        if [[ -z "$LABEL" ]]; then
            echo "ERROR: --label is required for --stop." >&2
            exit 1
        fi
        validate_label "$LABEL"
        SESSION_NAME="${SESSION_PREFIX}${LABEL}"

        if ! tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
            echo "Session '$SESSION_NAME' does not exist." >&2
            exit 1
        fi

        # Send SIGTERM to the pane process, then kill the session
        PANE_PID=$(tmux display-message -t "${SESSION_NAME}:0.0" -p "#{pane_pid}" 2>/dev/null || true)
        if [[ -n "$PANE_PID" ]] && kill -0 "$PANE_PID" 2>/dev/null; then
            kill -TERM "$PANE_PID" 2>/dev/null || true
            # Brief wait for graceful shutdown
            for _ in 1 2 3 4 5; do
                kill -0 "$PANE_PID" 2>/dev/null || break
                sleep 0.5
            done
        fi
        tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true
        echo "Stopped session: $SESSION_NAME"
        ;;

    launch)
        if [[ -z "$LABEL" ]]; then
            echo "ERROR: --label is required for launch." >&2
            exit 1
        fi
        validate_label "$LABEL"

        if [[ ${#CMD_ARGS[@]} -eq 0 ]]; then
            echo "ERROR: No command specified. Use -- before the command." >&2
            exit 1
        fi

        SESSION_NAME="${SESSION_PREFIX}${LABEL}"

        if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
            echo "ERROR: Session '$SESSION_NAME' already exists. Use --stop first." >&2
            exit 1
        fi

        tmux new-session -d -s "$SESSION_NAME" "${CMD_ARGS[@]}"
        echo "Started session: $SESSION_NAME"
        echo "  Attach: tmux attach -t $SESSION_NAME"
        ;;
esac
