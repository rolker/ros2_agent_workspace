#!/usr/bin/env python3
"""
Fetch/pull from a named remote for all workspace repositories.

Default mode: fetch and report which repos have new commits, listing them.
With --pull: merge remote changes into the current local branch.
With --branch: pull remote changes into a named local branch.

Usage:
    python3 pull_remote.py --remote gitcloud                    # fetch + report
    python3 pull_remote.py --remote gitcloud --pull             # fetch + merge
    python3 pull_remote.py --remote gitcloud --branch sync/gc   # fetch into branch
    python3 pull_remote.py --remote gitcloud --json             # fetch + JSON output

Prerequisites:
    Remotes must already exist in each repo. Use add_remote.py for one-time setup.

Exit status:
    0  every repo was fetched/merged, or deliberately skipped (dirty tree,
       detached HEAD, off the default branch, remote not configured, or absent
       from a layer that is optional on this host, or a default branch that
       exists only as a remote-tracking ref — see STATE_NO_LOCAL_BRANCH, which
       --json reports as its own entry rather than as an error or a silence).
    1  at least one repo errored: a failed fetch/merge, a working tree or git
       state that could not be read at all, a configured repo with no checkout
       in a *required* layer, or an enumeration that produced no repos at all
       (configs/manifest missing, or a manifest that would not parse). A probe
       that failed is not a benign answer — see is_dirty()/get_current_branch()
       (#609). --json exits 1 on the same conditions, after printing each to
       stderr and the (possibly partial) report to stdout.
    2  argparse usage error.

    `make pull-remote` reports GNU make's own 2 for any non-zero recipe status,
    so branch on these codes only when calling the script directly.
"""

import argparse
import json as json_mod
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent.resolve()
sys.path.insert(0, str(SCRIPT_DIR / "lib"))

from remote_utils import (  # noqa: E402
    NO_REPOS_ENUMERATED,
    UNREADABLE_STATE,
    UNREADABLE_TREE,
    RemoteState,
    add_common_args,
    classify_missing_repo,
    classify_remote_state,
    get_default_branch,
    get_repos,
    remote_probe,
    resolve_repo_path,
    run_git,
    run_git_network,
    run_script,
)
from workspace import WorkspaceConfigError, get_optional_layers  # noqa: E402

# Named states for a comparison that produced no counts. Both are normal,
# supported workspace states, so neither may turn a run red — but both must be
# *said*, never inferred from an absence, which is the whole of #609.
#
# NO_LOCAL_BRANCH is what `vcs import` leaves behind for a SHA- or tag-pinned
# manifest entry: the default branch exists only as a remote-tracking ref.
# validate_workspace.py protects the identical state for the identical reason
# ("a repo pinned to a SHA is legitimately detached, and flagging it would be a
# false red"). It used to be an error in --json and a benign skip in default
# mode — the same repo, the same state, two verdicts. The modes now agree: a
# visible non-fatal report in both, and its own entry in the JSON so
# /import-field-changes sees the state rather than reading "nothing to import"
# out of a silence.
STATE_AHEAD = "ahead"
STATE_NO_LOCAL_BRANCH = "no-local-branch"
STATE_NO_REMOTE_BRANCH = "no-remote-branch"


def no_local_branch_detail(branch, remote_ref):
    """The one wording both modes report for STATE_NO_LOCAL_BRANCH."""
    return (
        f"cannot compare: no local '{branch}' to compare against {remote_ref} — "
        "the default branch exists only as a remote-tracking ref (a SHA- or "
        f"tag-pinned manifest entry). Check out '{branch}' to compare."
    )


def is_dirty(repo_path):
    """Check if the repo has uncommitted changes.

    Returns True (dirty), False (clean), or None when `git status` itself
    failed. The three cases must stay distinct: a working tree we could not
    read is NOT a clean one. The old `success and bool(output)` reported
    exactly that, and this script is where it costs the most — `--pull`
    *merges* on this answer, so an unreadable working tree with real
    uncommitted changes would have been merged into (#609).
    """
    success, output, _ = run_git(repo_path, ["status", "--porcelain"])
    if not success:
        return None
    return bool(output)


def get_current_branch(repo_path):
    """Get the current branch name.

    Returns the branch name, or "" on a genuine detached HEAD (the command
    succeeded and printed nothing), or None when the git command itself failed
    (not a repo, corrupt .git). Callers MUST distinguish those two: "" is a
    benign skip, None is a repo we cannot read at all (#609).
    """
    success, output, _ = run_git(repo_path, ["branch", "--show-current"])
    if not success:
        return None
    return output


def _compare_branches(repo_path, branch, remote_ref):
    """Compare local and remote branches. Returns (status, message)."""
    # Verify both refs exist. Probed in their own ref namespaces rather than
    # by loose name: `rev-parse --verify jazzy` also matches refs/tags/jazzy,
    # and the rev-list below would then compare the tag (get_default_branch()
    # verifies refs/heads/ for the same reason).
    success, _, _ = run_git(repo_path, ["rev-parse", "--verify", f"refs/remotes/{remote_ref}"])
    if not success:
        return "skip", f"fetched (no {remote_ref} on remote)"

    success, _, _ = run_git(repo_path, ["rev-parse", "--verify", f"refs/heads/{branch}"])
    if not success:
        # The same state --json reports as STATE_NO_LOCAL_BRANCH. Benign in
        # both modes, named in both, and 0 in both (#609).
        return "skip", f"fetched ({no_local_branch_detail(branch, remote_ref)})"

    # Count commits ahead/behind
    success, output, err = run_git(
        repo_path,
        ["rev-list", "--left-right", "--count", f"{branch}...{remote_ref}"],
    )
    if not success:
        # Both refs verified above, so this failing means we could not read the
        # repo — and a bare "fetched" would claim we checked whether the remote
        # is ahead when we could not. sync_repos.py fixed the byte-identical
        # line for `git status -sb` on this branch (#609).
        return "error", f"cannot compare {branch}...{remote_ref}: {err or 'git rev-list failed'}"

    parts = output.split()
    if len(parts) != 2 or not all(part.isdigit() for part in parts):
        return "error", f"unreadable rev-list output for {branch}...{remote_ref}: {output!r}"

    ahead, behind = int(parts[0]), int(parts[1])
    if ahead == 0 and behind == 0:
        return "ok", "up to date"

    lines = []
    if behind > 0:
        lines.append(f"local is {behind} commit(s) behind {remote_ref}")
    if ahead > 0:
        lines.append(f"local is {ahead} commit(s) ahead of {remote_ref}")

    # List new commits from the remote
    if behind > 0:
        success, log_output, _ = run_git(
            repo_path,
            ["log", "--oneline", f"{branch}..{remote_ref}", "--max-count=20"],
        )
        if success and log_output:
            lines.append("  new remote commits:")
            for line in log_output.splitlines():
                lines.append(f"    {line}")
            if behind > 20:
                lines.append(f"    ... and {behind - 20} more")

    return "changes", "\n  ".join(lines)


def _fetch_and_report(repo_path, remote_name, version, dry_run):
    """Fetch from remote and report new commits. Returns (status, message)."""
    success, _, err = run_git_network(repo_path, ["fetch", remote_name], dry_run)
    if not success:
        return "error", f"fetch failed: {err}"
    if dry_run:
        return "ok", "fetched (dry run)"

    branch = get_default_branch(repo_path, version)
    remote_ref = f"{remote_name}/{branch}"
    return _compare_branches(repo_path, branch, remote_ref)


def _check_pull_preconditions(repo_path, version):
    """Check if repo is ready for a merge. Returns (branch, problem).

    `problem` is None when the repo is ready to merge, otherwise a
    (status, message) pair ready to return from process_repo(). "skip" is for
    a benign, operator-visible reason that leaves the repo deliberately alone
    (dirty tree, detached HEAD, off the default branch); "error" is for a git
    state we could not read, which must not be reported as a benign skip and
    must make the run exit non-zero (#609).
    """
    dirty = is_dirty(repo_path)
    if dirty is None:
        return None, ("error", UNREADABLE_TREE)
    if dirty:
        return None, ("skip", "uncommitted changes — skipping merge")

    branch = get_default_branch(repo_path, version)
    current = get_current_branch(repo_path)
    if current is None:
        # The git command itself failed. NOT the same as a detached HEAD:
        # collapsing the two reported a repo we cannot read as a benign skip,
        # and the run still exited 0 (#609).
        return None, ("error", UNREADABLE_STATE)
    if not current:
        return None, ("skip", "detached HEAD — skipping merge")
    if current != branch:
        return None, ("skip", f"not on default branch (on '{current}', expected '{branch}')")
    return branch, None


def _fetch_and_pull(repo_path, remote_name, version, dry_run):
    """Fetch and merge from remote. Returns (status, message)."""
    branch, problem = _check_pull_preconditions(repo_path, version)
    if problem:
        return problem

    # Fetch
    success, _, err = run_git_network(repo_path, ["fetch", remote_name], dry_run)
    if not success:
        return "error", f"fetch failed: {err}"

    # Merge
    remote_ref = f"{remote_name}/{branch}"
    success, out, err = run_git(repo_path, ["merge", remote_ref, "--ff-only"], dry_run)
    if not success:
        return "error", f"merge failed (non-fast-forward?): {err}"
    if dry_run:
        return "ok", "merged (dry run)"
    if "Already up to date" in out:
        return "ok", "already up to date"
    return "ok", f"merged from {remote_ref}"


def _fetch_into_branch(repo_path, remote_name, version, target_branch, dry_run):
    """Fetch and update a local branch from the remote. Returns (status, message)."""
    # git branch -f fails if the target branch is currently checked out
    current = get_current_branch(repo_path)
    if current is None:
        # We cannot tell whether target_branch is checked out, so we cannot
        # know that `git branch -f` is safe here (#609).
        return "error", UNREADABLE_STATE
    if current == target_branch:
        return "skip", (
            f"branch '{target_branch}' is currently checked out — "
            "cannot force-update; use --pull instead"
        )

    branch = get_default_branch(repo_path, version)

    # Fetch
    success, _, err = run_git_network(repo_path, ["fetch", remote_name], dry_run)
    if not success:
        return "error", f"fetch failed: {err}"

    remote_ref = f"{remote_name}/{branch}"

    # Check if remote ref exists
    if not dry_run:
        success, _, _ = run_git(repo_path, ["rev-parse", "--verify", remote_ref])
        if not success:
            return "ok", f"fetched (no {remote_ref} on remote)"

    # Create or update the target branch to point at the remote ref
    success, _, err = run_git(repo_path, ["branch", "-f", target_branch, remote_ref], dry_run)
    if not success:
        return "error", f"branch update failed: {err}"

    return "ok", f"branch '{target_branch}' updated to {remote_ref}"


def process_repo(repo_path, repo_name, version, args):
    """Dispatch to the appropriate pull mode. Returns (status, message)."""
    problem = classify_remote_state(remote_probe(repo_path, args.remote), args.remote)
    if problem:
        return problem

    if args.pull:
        return _fetch_and_pull(repo_path, args.remote, version, args.dry_run)
    if args.branch:
        return _fetch_into_branch(repo_path, args.remote, version, args.branch, args.dry_run)
    return _fetch_and_report(repo_path, args.remote, version, args.dry_run)


def _get_ahead_commits(repo_path, branch, remote_ref, max_count=50):
    """List the commits on the remote but not local. Returns (commits, error).

    `error` is a string when `git log` itself failed. The caller only asks once
    it already knows the remote is ahead, so an empty list from a *failed* log
    would describe a repo with pending field commits as having none (#609).
    """
    success, output, err = run_git(
        repo_path, ["log", "--oneline", f"--max-count={max_count}", f"{branch}..{remote_ref}"]
    )
    if not success:
        return [], f"cannot list {branch}..{remote_ref}: {err or 'git log failed'}"
    commits = []
    for line in output.splitlines():
        parts = line.split(" ", 1)
        commits.append({"sha": parts[0], "subject": parts[1] if len(parts) > 1 else ""})
    return commits, None


def _ahead_behind(repo_path, branch, remote_ref):
    """Count commits each side of branch...remote_ref. Returns (counts, state, error).

    Exactly one of the three is ever set, and keeping them apart is the point
    of #609:

    - `counts` — an (ahead, behind) pair; the comparison was made.
    - `state`  — a named, benign non-answer (STATE_NO_REMOTE_BRANCH or
      STATE_NO_LOCAL_BRANCH). Real answers, not failures: neither turns a run
      red, and neither is left to be inferred from a silence.
    - `error`  — a probe *failed*, so we never learned whether the remote is
      ahead. Never reportable as one of the benign answers above.

    Both ref probes name their ref namespace rather than looking the name up
    loosely: `rev-parse --verify jazzy` also matches refs/tags/jazzy, and the
    rev-list would then compare the tag.
    """
    # No such branch on the remote: nothing to import, and not an error.
    # Checked before the local branch so a repo the remote simply does not
    # carry stays silent rather than complaining about a local ref.
    success, _, _ = run_git(repo_path, ["rev-parse", "--verify", f"refs/remotes/{remote_ref}"])
    if not success:
        return None, STATE_NO_REMOTE_BRANCH, None

    # The remote branch exists but the local one does not — the state
    # `vcs import` leaves for a SHA- or tag-pinned entry, and the trigger for
    # the rc-128 rev-list below. Default mode calls this a skip and exits 0;
    # this mode used to call the identical repo an error. Reported, not failed.
    success, _, _ = run_git(repo_path, ["rev-parse", "--verify", f"refs/heads/{branch}"])
    if not success:
        return None, STATE_NO_LOCAL_BRANCH, None

    success, output, err = run_git(
        repo_path, ["rev-list", "--left-right", "--count", f"{branch}...{remote_ref}"]
    )
    if not success:
        return None, None, f"cannot compare {branch}...{remote_ref}: {err or 'git rev-list failed'}"

    parts = output.split()
    if len(parts) != 2 or not all(part.isdigit() for part in parts):
        return None, None, f"unreadable rev-list output for {branch}...{remote_ref}: {output!r}"

    return (int(parts[0]), int(parts[1])), None, None


def json_report(repo_path, repo_name, version, remote_name):
    """Build the JSON entry for one repo. Returns (entry, error).

    Public rather than module-private: this pair *is* the --json contract that
    `/import-field-changes` consumes, and it is asserted directly in
    test_pull_remote.py rather than only through main().

    At most one of the two is ever set, and the distinction is the point (#609):

    - `(entry, None)` — something for the consumer to act on. Every entry
      carries an explicit `state`, so no consumer has to infer one:
        * STATE_AHEAD — the remote is ahead; these are the commits.
        * STATE_NO_LOCAL_BRANCH — the default branch exists only as a
          remote-tracking ref, so no comparison was possible. Not an error
          (default mode skips the identical repo and exits 0), but reported
          rather than silent: an absence here would read as "nothing to
          import" for a repo that may carry every unreconciled field commit.
    - `(None, None)` — nothing to act on and nothing to say: the remote has no
      such branch, or it has one and it is not ahead. Mirrors what non-JSON
      mode reports as a skip / "up to date".
    - `(None, error)` — a probe *failed*, so we never learned whether the remote
      is ahead. This used to collapse into the benign `None` above, and the
      only consumer of --json is `/import-field-changes`, which stops when the
      report is empty. A field repo whose commits were never reconciled to
      GitHub therefore reported green — the exact false green this issue is
      about, in its most costly form.
    """
    branch = get_default_branch(repo_path, version)
    remote_ref = f"{remote_name}/{branch}"
    identity = {
        "repo": repo_name,
        "path": str(repo_path),
        "default_branch": branch,
        "remote_ref": remote_ref,
    }

    counts, state, err = _ahead_behind(repo_path, branch, remote_ref)
    if err:
        return None, err
    if state == STATE_NO_LOCAL_BRANCH:
        return {
            **identity,
            "state": STATE_NO_LOCAL_BRANCH,
            "detail": no_local_branch_detail(branch, remote_ref),
        }, None
    if counts is None:
        return None, None  # STATE_NO_REMOTE_BRANCH: the remote carries no such branch

    ahead, behind = counts
    if behind == 0:
        return None, None  # remote carries nothing we do not already have

    commits, err = _get_ahead_commits(repo_path, branch, remote_ref)
    if err:
        return None, err
    return {
        **identity,
        "state": STATE_AHEAD,
        "ahead": ahead,
        "behind": behind,
        "diverged": ahead > 0 and behind > 0,
        "commits": commits,
        # Non-JSON mode prints "... and N more"; without this a consumer
        # diffing len(commits) against `behind` sees an unexplained shortfall.
        "commits_truncated": behind > len(commits),
    }, None


def main():
    parser = argparse.ArgumentParser(
        description="Fetch/pull from a named remote for all workspace repositories."
    )
    add_common_args(parser)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--pull",
        action="store_true",
        help="Merge remote changes into current branch (fast-forward only)",
    )
    mode.add_argument(
        "--branch",
        help="Create/update a local branch with remote changes (e.g., sync/gitcloud)",
    )
    mode.add_argument(
        "--json",
        action="store_true",
        help="Output JSON report of repos with remote-ahead commits (fetch-only mode)",
    )
    args = parser.parse_args()

    if args.json:
        root_dir = SCRIPT_DIR.parent.parent
        results = []
        errors = []
        try:
            repos = get_repos(args)
        except WorkspaceConfigError as exc:
            # A .repos file we could not parse: every repo it declares goes
            # unenumerated, and an empty report reads as "nothing to import".
            repos = []
            errors.append(str(exc))
        else:
            if not repos:
                errors.append(NO_REPOS_ENUMERATED)

        def collect(repo_path, repo_name, version):
            """Probe, fetch and report one repo into results/errors."""
            state = remote_probe(repo_path, args.remote)
            if state is RemoteState.UNREADABLE:
                errors.append(f"{repo_name}: {UNREADABLE_STATE}")
                return
            if state is RemoteState.ABSENT:
                # Most repos legitimately have no secondary remote. Probing the
                # workspace root too keeps --json consistent with non-JSON mode,
                # which skips it rather than reporting a failed fetch.
                return
            success, _, err = run_git_network(repo_path, ["fetch", args.remote], args.dry_run)
            if not success:
                errors.append(f"{repo_name}: fetch failed: {err}")
                return
            entry, report_err = json_report(repo_path, repo_name, version, args.remote)
            if report_err:
                errors.append(f"{repo_name}: {report_err}")
            elif entry:
                results.append(entry)

        collect(root_dir, "ros2_agent_workspace", get_default_branch(root_dir, None))

        # Read the optional-layer list once, not per repo.
        optional_layers = get_optional_layers(root_dir)

        for repo in repos:
            repo_path = resolve_repo_path(root_dir, repo)
            if repo_path is None:
                # Not found on disk. Optional layers are allowed to be absent;
                # anything else is a repo this report silently omitted (#609).
                skip, reason = classify_missing_repo(root_dir, repo, optional_layers)
                if not skip:
                    errors.append(f"{repo['name']}: {reason}")
                continue
            collect(repo_path, repo["name"], repo["version"])

        if errors:
            for e in errors:
                print(f"ERROR: {e}", file=sys.stderr)

        print(json_mod.dumps(results, indent=2))
        sys.exit(1 if errors else 0)

    if args.pull or args.branch:
        labels = [
            ("ok", "updated"),
            ("skip", "skipped"),
            ("error", "errors"),
            ("missing", "missing"),
        ]
    else:
        labels = [
            ("ok", "up to date"),
            ("changes", "with changes"),
            ("skip", "skipped"),
            ("error", "errors"),
            ("missing", "missing"),
        ]

    run_script(
        SCRIPT_DIR,
        args,
        process_repo,
        {"ok": 0, "changes": 0, "skip": 0, "error": 0, "missing": 0},
        labels,
    )


if __name__ == "__main__":
    main()
