#!/usr/bin/env python3
import sys
import subprocess
import argparse
from pathlib import Path

# Add script directory to path to import list_overlay_repos
SCRIPT_DIR = Path(__file__).parent.resolve()
sys.path.append(str(SCRIPT_DIR))

try:
    import list_overlay_repos
except ImportError:
    print(f"Error: Could not import list_overlay_repos from {SCRIPT_DIR}", file=sys.stderr)
    sys.exit(1)

def run_git_cmd(repo_path, cmd_args, dry_run=False):
    """Run a git command in the given repo path."""
    full_cmd = ["git"] + cmd_args
    if dry_run:
        print(f"[DRY-RUN] {repo_path.name}: {' '.join(full_cmd)}")
        return True, ""
    
    try:
        result = subprocess.run(
            full_cmd,
            cwd=str(repo_path),
            capture_output=True,
            text=True,
            check=True
        )
        return True, result.stdout.strip()
    except subprocess.CalledProcessError as e:
        return False, e.stderr.strip()

def is_dirty(repo_path):
    """Check if repo has uncommitted changes."""
    success, output = run_git_cmd(repo_path, ["status", "--porcelain"])
    return success and bool(output)

def get_current_branch(repo_path):
    """Get the current checked out branch."""
    success, output = run_git_cmd(repo_path, ["branch", "--show-current"])
    if success:
        return output
    return None

def sync_repo(repo_path, repo_name, dry_run=False):
    """Synchronize a single repository."""
    print(f"Checking {repo_name}...")
    
    if not repo_path.exists():
        print(f"  ❌ Path does not exist: {repo_path}")
        return

    # 1. Check for local modifications
    if is_dirty(repo_path):
        if dry_run:
            print(f"  ⚠️  (Dry run) Would skip: Uncommitted changes detected.")
        else:
            print(f"  ⚠️  Skipping: Uncommitted changes detected.")
        return

    branch = get_current_branch(repo_path)
    if not branch:
        print(f"  ❌ Skipping: Detached HEAD or invalid git state.")
        return

    # 2. Sync Logic
    if branch in ['main', 'jazzy', 'rolling']:
        print(f"  🚀 On default branch '{branch}'. Pulling updates...")
        success, output = run_git_cmd(repo_path, ["pull", "--rebase"], dry_run)
        if success:
            if dry_run:
                print("     (Dry run successful)")
            elif "Already up to date." in output:
                print("     ✅ Already up to date.")
            else:
                print(f"     ✅ Updated:\n{output}")
        else:
            print(f"     ❌ Update failed: {output}")
            
    else:
        print(f"  🌿 On feature branch '{branch}'. Fetching only...")
        success, output = run_git_cmd(repo_path, ["fetch"], dry_run)
        
        if success:
            if dry_run:
                print("     (Dry run successful)")
            else:
                # Check status relative to upstream
                # Assuming upstream is 'origin'
                s_success, s_msg = run_git_cmd(repo_path, ["status", "-sb"], dry_run)
                if s_success and "behind" in s_msg:
                    print(f"     ⚠️  Branch is behind remote. Run 'git merge' or 'git rebase' manually.")
                else:
                    print("     ✅ Fetched.")
        else:
            print(f"     ❌ Fetch failed: {output}")

def main():
    parser = argparse.ArgumentParser(description="Safely sync workspace repositories.")
    parser.add_argument("--dry-run", action="store_true", help=" Simulate actions without executing.")
    args = parser.parse_args()

    root_dir = SCRIPT_DIR.parent.parent
    
    # Get repos list using the existing tool
    repos = list_overlay_repos.get_overlay_repos(include_underlay=False)
    
    # Also include the root repo itself
    sync_repo(root_dir, "ros2_agent_workspace", args.dry_run)
    
    for repo in repos:
        # Determine workspace directory from source file (e.g. core.repos -> core_ws)
        ws_name = repo['source_file'].replace('.repos', '_ws')
        repo_path = root_dir / "workspaces" / ws_name / "src" / repo['name']
        sync_repo(repo_path, repo['name'], args.dry_run)
        
    print("\n✅ Sync complete.")

if __name__ == "__main__":
    main()
