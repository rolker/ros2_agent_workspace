#!/usr/bin/env python3
"""
Sync Workspace Repositories

This script safely synchronizes workspace repositories by pulling updates on
default branches (main/jazzy) and fetching on feature branches. It respects
dirty working directories and detached HEAD states.

Usage:
    python3 sync_repos.py [--dry-run] [--throttle SECONDS]

Options:
    --dry-run    Simulate actions without executing
    --throttle   Pause between per-repo network operations. Off by default;
                 if a dropped connection is detected mid-run (an upstream
                 firewall rate-limiting rapid successive SSH connections),
                 the remaining repos are automatically paced at 2.0s.
                 Pass a value to force a fixed pace from the start
                 (0 disables pacing entirely, including the adaptive one).
"""

import sys
import enum
import collections
import math
import time
import shutil
import subprocess
import argparse
from pathlib import Path

# Add script directory to path to import list_overlay_repos
SCRIPT_DIR = Path(__file__).parent.resolve()
sys.path.append(str(SCRIPT_DIR))
sys.path.insert(0, str(SCRIPT_DIR / "lib"))

try:
    import list_overlay_repos
except ImportError:
    print(f"Error: Could not import list_overlay_repos from {SCRIPT_DIR}", file=sys.stderr)
    sys.exit(1)

from remote_utils import (  # noqa: E402
    reset_transient_error_seen,
    retry_transient,
    transient_error_seen,
)

# Pace applied to the rest of the run once the remote starts dropping
# connections (only when --throttle was not given explicitly).
ADAPTIVE_THROTTLE = 2.0


def make_throttler(explicit_throttle, dry_run):
    """Return a pause() callable implementing the pacing policy.

    explicit_throttle is the --throttle value or None if not given:
      None -> adaptive: no pause until a dropped connection has been seen,
              then ADAPTIVE_THROTTLE for the rest of the run.
      N    -> fixed pace of N seconds (0 disables pacing entirely).
    Dry runs never pause.
    """
    noticed = [False]

    def pause():
        if dry_run:
            return
        if explicit_throttle is not None:
            if explicit_throttle > 0:
                time.sleep(explicit_throttle)
            return
        if transient_error_seen():
            if not noticed[0]:
                noticed[0] = True
                print(
                    f"ℹ️  Remote dropped a connection — pacing remaining repos "
                    f"by {ADAPTIVE_THROTTLE}s."
                )
            time.sleep(ADAPTIVE_THROTTLE)

    return pause


def run_git_cmd(repo_path, cmd_args, dry_run=False):
    """Run a git command in the given repo path."""
    full_cmd = ["git"] + cmd_args
    if dry_run:
        print(f"[DRY-RUN] {repo_path.name}: {' '.join(full_cmd)}")
        return True, ""

    try:
        result = subprocess.run(
            full_cmd, cwd=str(repo_path), capture_output=True, text=True, check=True
        )
        return True, result.stdout.strip()
    except subprocess.CalledProcessError as e:
        return False, e.stderr.strip()
    except OSError as e:
        # The repo directory itself is unusable (unreadable, vanished mid-run,
        # a dangling symlink). Without this the exception escapes and kills the
        # whole run with no summary — loud, but it strands every later repo and
        # contradicts SyncOutcome.FAILED's "a state we cannot even read" (#609).
        return False, f"cannot run git in {repo_path}: {e}"


def run_network_cmd(repo_path, cmd_args, dry_run=False):
    """Run a git network command, retrying once after a backoff if the
    connection was dropped (rate-limit style failures — signatures and
    backoff live in lib/remote_utils.py, shared with push/pull_remote)."""
    return retry_transient(run_git_cmd, repo_path, cmd_args, dry_run)


def is_dirty(repo_path, dry_run=False):
    """Check if repo has uncommitted changes.

    Returns True (dirty), False (clean), or None when `git status` itself
    failed. The three cases must stay distinct: a working tree we could not
    read is NOT a clean one, and folding it into False let a repo with a
    corrupt index and real uncommitted changes classify as SYNCED — the exact
    false green #609 exists to remove.
    """
    # Dirty check is a read-only operation, always execute regardless of dry_run
    # but we need to call run_git_cmd without dry_run flag to actually execute
    success, output = run_git_cmd(repo_path, ["status", "--porcelain"], dry_run=False)
    if not success:
        return None
    return bool(output)


def get_current_branch(repo_path, dry_run=False):
    """Get the current checked out branch.

    Returns the branch name, or "" on a genuine detached HEAD (the command
    succeeded and printed nothing), or None when the git command itself failed
    (not a repo, corrupt .git). Callers MUST distinguish those two: `""` is a
    benign skip, `None` is a broken repo (#609).
    """
    # Getting branch is a read-only operation, always execute regardless of dry_run
    success, output = run_git_cmd(repo_path, ["branch", "--show-current"], dry_run=False)
    if success:
        return output
    return None


class SyncOutcome(enum.Enum):
    """What happened to one repository during a sync (#609).

    Three states, not two, because the reason a repo was left alone decides
    whether the run failed:

    - SYNCED — the pull/fetch ran and succeeded.
    - SKIPPED — deliberately left alone for a benign, operator-visible reason
      (uncommitted changes, detached HEAD). Normal, and NOT a failure: making
      an intentionally dirty repo turn the whole run red would produce a signal
      everyone learns to ignore, which is worse than the false green this fix
      removes.
    - FAILED — the repo was supposed to sync and did not: the network command
      failed, or the repo is in a state we cannot even read.

    Beware: every Enum member is truthy — and so is the SyncResult tuple that
    carries one — so a call site left as `if sync_repo(...)` would silently
    treat FAILED as success. Compare `.outcome` against members explicitly.
    """

    SYNCED = "synced"
    SKIPPED = "skipped"
    FAILED = "failed"


SyncResult = collections.namedtuple("SyncResult", ["outcome", "reason"], defaults=("",))
SyncResult.__doc__ = """One repo's outcome plus why (#609).

The reason is what the failure summary prints, so it must name the actual
cause — "sync failed" for every repo alike tells the operator nothing about
whether to check the network, the layer setup, or the repo itself.
"""

# Signatures of "this host has no route to the remote at all". These are NOT
# the dropped-connection signatures in lib/remote_utils.py (TRANSIENT_ERRORS,
# which are retried): an offline host fails every repo instantly, and saying so
# once beats 35 identical "pull failed" lines.
OFFLINE_SIGNATURES = (
    "Could not resolve hostname",
    "Temporary failure in name resolution",
    "Network is unreachable",
    "No route to host",
    "Name or service not known",
)


def describe_network_failure(action, output):
    """Reason string for a failed pull/fetch, reusing git's own error text."""
    lines = [line.strip() for line in (output or "").splitlines() if line.strip()]
    detail = lines[0][:200] if lines else "no error output"
    if any(sig in (output or "") for sig in OFFLINE_SIGNATURES):
        return f"remote unreachable ({action}): {detail}"
    return f"{action} failed: {detail}"


def preflight_repo(repo_path, dry_run=False):
    """Decide whether a repo can be synced at all.

    Returns (result, branch): a SyncResult when the repo should not be synced,
    or (None, branch) when it is ready. Split out from sync_repo() to keep each
    function's branching legible now that the outcome is tri-state.
    """
    if not repo_path.exists():
        print(f"  ❌ Path does not exist: {repo_path}")
        return SyncResult(SyncOutcome.FAILED, "path does not exist"), None

    dirty = is_dirty(repo_path, dry_run)
    if dirty is None:
        print("  ❌ Cannot read working tree (corrupt .git, or not a repo).")
        return SyncResult(SyncOutcome.FAILED, "cannot read working tree"), None

    if dirty:
        if dry_run:
            print("  ⚠️  (Dry run) Would skip: Uncommitted changes detected.")
        else:
            print("  ⚠️  Skipping: Uncommitted changes detected.")
        return SyncResult(SyncOutcome.SKIPPED, "uncommitted changes"), None

    branch = get_current_branch(repo_path, dry_run)
    if branch is None:
        # The git command itself failed — not a repo, or a corrupt .git. This
        # is NOT the same as a detached HEAD, and collapsing the two is how a
        # genuinely broken repo stays silently stale under a green exit (#609).
        print("  ❌ Cannot read git state (not a repo, or corrupt .git).")
        return SyncResult(SyncOutcome.FAILED, "cannot read git state"), None
    if not branch:
        print("  ⚠️  Skipping: Detached HEAD.")
        return SyncResult(SyncOutcome.SKIPPED, "detached HEAD"), None

    return None, branch


def sync_repo(repo_path, repo_name, dry_run=False):
    """Synchronize a single repository. Returns a SyncResult (#609)."""
    print(f"Checking {repo_name}...")

    result, branch = preflight_repo(repo_path, dry_run)
    if result is not None:
        return result

    # 2. Sync Logic
    if branch in ["main", "jazzy", "rolling"]:
        print(f"  On default branch '{branch}'. Pulling updates...")
        success, output = run_network_cmd(repo_path, ["pull", "--rebase"], dry_run)
        if success:
            if dry_run:
                print("     (Dry run successful)")
            elif "Already up to date." in output:
                print("     ✅ Already up to date.")
            else:
                print(f"     ✅ Updated:\n{output}")
        else:
            print(f"     ❌ Update failed: {output}")
            return SyncResult(SyncOutcome.FAILED, describe_network_failure("pull", output))

    else:
        print(f"  On feature branch '{branch}'. Fetching only...")
        success, output = run_network_cmd(repo_path, ["fetch"], dry_run)

        if success:
            if dry_run:
                print("     (Dry run successful)")
            else:
                # Check status relative to upstream
                # Assuming upstream is 'origin'
                s_success, s_msg = run_git_cmd(repo_path, ["status", "-sb"], dry_run)
                if not s_success:
                    # The fetch itself succeeded, so this is not a failed sync —
                    # but a bare "✅ Fetched." would imply we checked whether the
                    # branch is behind when we could not (#609).
                    print("     ✅ Fetched (could not read ahead/behind status).")
                elif "behind" in s_msg:
                    print(
                        "     ⚠️  Branch is behind remote."
                        " Run 'git merge' or 'git rebase' manually."
                    )
                else:
                    print("     ✅ Fetched.")
        else:
            print(f"     ❌ Fetch failed: {output}")
            return SyncResult(SyncOutcome.FAILED, describe_network_failure("fetch", output))

    return SyncResult(SyncOutcome.SYNCED)


def sync_gitbug(repo_path, dry_run=False):
    """Sync git-bug issues for a repo if git-bug is installed and a bridge is configured."""
    if not shutil.which("git-bug"):
        return

    # Check if a bridge is configured in this repo
    try:
        result = subprocess.run(
            ["git", "bug", "bridge", "list"],
            cwd=str(repo_path),
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode != 0 or not result.stdout.strip():
            return
    except OSError:
        return

    repo_name = repo_path.name
    if dry_run:
        print(f"  [DRY-RUN] {repo_name}: git bug pull")
        print(f"  [DRY-RUN] {repo_name}: git bug push")
        return

    print(f"  Syncing git-bug issues for {repo_name}...")
    # git-bug pull/push are network operations against the same remote —
    # route them through the retry wrapper so a rate-limited SSH connection
    # gets the same second chance as the plain git pulls.
    for cmd_args in (["bug", "pull"], ["bug", "push"]):
        success, output = run_network_cmd(repo_path, cmd_args)
        if not success:
            print(f"     ⚠️  git {' '.join(cmd_args)} failed: {output}")
            return
    print("     ✅ git-bug synced.")


def main():
    parser = argparse.ArgumentParser(description="Safely sync workspace repositories.")
    parser.add_argument(
        "--dry-run", action="store_true", help="Simulate actions without executing."
    )
    parser.add_argument(
        "--throttle",
        type=float,
        default=None,
        help=f"Seconds to pause between per-repo network operations. "
        f"Default: none, auto-enabling a {ADAPTIVE_THROTTLE}s pace for the rest "
        f"of the run if the remote drops a connection. "
        f"Pass 0 to disable pacing entirely.",
    )
    args = parser.parse_args()
    if args.throttle is not None and not (math.isfinite(args.throttle) and args.throttle >= 0):
        parser.error("--throttle must be a finite value >= 0")

    # Each run adapts from a clean slate — matters only for in-process
    # callers, but makes the fresh-process assumption explicit.
    reset_transient_error_seen()

    root_dir = SCRIPT_DIR.parent.parent

    # Get repos list using the existing tool
    repos = list_overlay_repos.get_overlay_repos(include_underlay=False)

    throttle = make_throttler(args.throttle, args.dry_run)

    # Per-repo outcomes drive the summary and the exit status (#609).
    synced = 0
    skipped = 0
    failures = []

    def record(repo_path, repo_name, result):
        """Tally one repo's outcome, running git-bug only on a real sync."""
        nonlocal synced, skipped
        if result.outcome is SyncOutcome.SYNCED:
            synced += 1
            sync_gitbug(repo_path, args.dry_run)
        elif result.outcome is SyncOutcome.SKIPPED:
            skipped += 1
        else:
            # The reason names the actual cause; the fallback only guards a
            # future FAILED path that forgets to supply one.
            failures.append((repo_name, result.reason or "sync failed"))

    # Also include the root repo itself. Compare against the member explicitly:
    # every Enum member is truthy, so `if sync_repo(...)` would treat a FAILED
    # sync as success (#609).
    record(
        root_dir, "ros2_agent_workspace", sync_repo(root_dir, "ros2_agent_workspace", args.dry_run)
    )

    for repo in repos:
        throttle()
        # Determine workspace directory from source file (e.g. core.repos -> core_ws)
        ws_name = repo["source_file"].replace(".repos", "_ws")
        candidate_path = root_dir / "layers" / "main" / ws_name / "src" / repo["name"]

        repo_path = None
        tried_paths = [str(candidate_path)]

        # First, try the conventional workspace layout.
        if candidate_path.exists():
            repo_path = candidate_path
        else:
            # Fall back to an explicit path if provided by list_overlay_repos.
            explicit_path = repo.get("path")
            if explicit_path:
                explicit_path = Path(explicit_path)
                if not explicit_path.is_absolute():
                    explicit_path = root_dir / explicit_path
                tried_paths.append(str(explicit_path))
                if explicit_path.exists():
                    repo_path = explicit_path

        if repo_path is None:
            paths_str = ", ".join(tried_paths)
            print(f"  ❌ {repo['name']}: could not resolve repository path (tried {paths_str}).")
            # A configured repo we cannot even locate is a failure, not a benign
            # skip — it is precisely the "left stale, nobody noticed" case #609
            # exists to kill. A layer simply not imported on this host lands
            # here too, so the reason names it rather than saying "sync failed".
            failures.append((repo["name"], "path not resolved"))
            continue

        record(repo_path, repo["name"], sync_repo(repo_path, repo["name"], args.dry_run))

    # Summary + exit status (#609). This previously printed an unconditional
    # success line and always exited 0, so a run that left repos stale still
    # reported green — and `make sync` runs at the tail of merge_pr.sh, where
    # nobody is reading the output.
    print()
    if failures:
        print(f"❌ Sync finished with {len(failures)} failure(s):")
        for name, reason in failures:
            print(f"   - {name}: {reason}")
        print(f"   ({synced} synced, {skipped} skipped)")
        sys.exit(1)

    print(f"✅ Sync complete — {synced} synced, {skipped} skipped, 0 failures.")


if __name__ == "__main__":
    main()
