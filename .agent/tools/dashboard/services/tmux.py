"""tmux integration: capture pane output, detect status, send keys."""

import re
import subprocess


# Pattern to strip ANSI escape sequences
_ANSI_RE = re.compile(r"\x1b\[[0-9;]*[a-zA-Z]|\x1b\].*?\x07|\x1b\[.*?[@-~]")


def list_panes():
    """List all tmux panes with their working directories.

    Returns list of dicts: [{"pane_id": "sess:0.1", "cwd": "/path", "pid": "123"}, ...]
    """
    try:
        result = subprocess.run(
            [
                "tmux",
                "list-panes",
                "-a",
                "-F",
                "#{session_name}:#{window_index}.#{pane_index}\t#{pane_current_path}\t#{pane_pid}",
            ],
            capture_output=True,
            text=True,
            timeout=5,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return []

    if result.returncode != 0:
        return []

    panes = []
    for line in result.stdout.strip().split("\n"):
        if not line:
            continue
        parts = line.split("\t")
        if len(parts) >= 3:
            panes.append(
                {
                    "pane_id": parts[0],
                    "cwd": parts[1],
                    "pid": parts[2],
                }
            )
    return panes


def capture_pane(pane_id, lines=80):
    """Capture terminal output from a tmux pane, stripping ANSI codes."""
    try:
        result = subprocess.run(
            ["tmux", "capture-pane", "-p", "-t", pane_id, "-S", f"-{lines}"],
            capture_output=True,
            text=True,
            timeout=5,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return ""

    if result.returncode != 0:
        return ""

    return _ANSI_RE.sub("", result.stdout)


def detect_status(pane_id):
    """Detect agent status by reading the last few lines of pane output.

    Returns: 'working', 'waiting', or 'idle'
    """
    output = capture_pane(pane_id, lines=10)
    if not output.strip():
        return "idle"

    last_lines = output.strip().split("\n")[-5:]
    joined = " ".join(last_lines).lower()

    # Agent is asking a question or requesting permission
    if any(
        kw in joined
        for kw in [
            "approve",
            "permission",
            "deny",
            "(y/n)",
            "do you want",
            "should i",
            "would you like",
        ]
    ):
        return "waiting"

    # Check for Claude Code's prompt indicator ("> " at start of last line)
    last_line = last_lines[-1].strip() if last_lines else ""
    if last_line == ">" or last_line.endswith("> "):
        return "idle"

    return "working"


def list_sessions():
    """List all tmux session names.

    Returns list of session name strings.
    """
    try:
        result = subprocess.run(
            ["tmux", "list-sessions", "-F", "#{session_name}"],
            capture_output=True,
            text=True,
            timeout=5,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return []

    if result.returncode != 0:
        return []

    return [s for s in result.stdout.strip().split("\n") if s]


def is_session_alive(session_name):
    """Check if a tmux session's pane process is still running.

    Returns True if the process is alive, False if it has exited.
    """
    try:
        result = subprocess.run(
            [
                "tmux",
                "display-message",
                "-t",
                f"{session_name}:0.0",
                "-p",
                "#{pane_dead}",
            ],
            capture_output=True,
            text=True,
            timeout=5,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False

    if result.returncode != 0:
        return False

    # pane_dead is "1" when the process has exited, "0" when alive
    return result.stdout.strip() != "1"


def send_keys(pane_id, text):
    """Send text to a tmux pane followed by Enter.

    Uses -l (literal) so tmux key names in the text are not interpreted.
    Enter is sent as a separate command since -l suppresses key lookup.
    """
    try:
        r1 = subprocess.run(
            ["tmux", "send-keys", "-t", pane_id, "-l", text], capture_output=True, timeout=5
        )
        if r1.returncode != 0:
            return False
        r2 = subprocess.run(
            ["tmux", "send-keys", "-t", pane_id, "Enter"], capture_output=True, timeout=5
        )
        return r2.returncode == 0
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False
