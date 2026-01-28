#!/usr/bin/env python3
"""
Validate workspace matches .repos configuration.

This script verifies that the actual workspace repositories match what is
configured in the .repos files. It checks:
1. All repos in configs are cloned
2. All cloned repos are in configs (detect orphans)
3. Repos are on expected branches/versions

Usage:
    python3 validate_workspace.py [--fix] [--verbose]
"""

import os
import sys
import argparse
import subprocess
import yaml
from pathlib import Path

# Add lib directory to path
SCRIPT_DIR = Path(__file__).parent.resolve()
sys.path.insert(0, str(SCRIPT_DIR / 'lib'))

from workspace import get_workspace_root


def get_all_repos_config():
    """
    Get all repositories defined in .repos files.
    
    Returns:
        dict: Dictionary mapping repo name to config info:
            {
                'repo_name': {
                    'url': str,
                    'version': str,
                    'source_file': str,
                    'workspace': str (e.g., 'core_ws')
                }
            }
    """
    workspace_root = Path(get_workspace_root())
    repos_config = {}
    
    # Check both configs/ directory and migrated Key Repo location
    config_dirs = [
        workspace_root / "configs",
        workspace_root / "workspaces/core_ws/src/unh_marine_autonomy/config/repos"
    ]
    
    for config_dir in config_dirs:
        if not config_dir.exists():
            continue
            
        for repos_file in config_dir.glob("*.repos"):
            # Skip underlay.repos - those are system dependencies
            if repos_file.name == "underlay.repos":
                continue
                
            # Derive workspace name from filename (e.g., core.repos -> core_ws)
            workspace_name = repos_file.stem + "_ws"
            
            try:
                with open(repos_file, 'r', encoding='utf-8') as f:
                    data = yaml.safe_load(f)
                    
                if not data or 'repositories' not in data:
                    continue
                    
                for repo_name, repo_info in data['repositories'].items():
                    if repo_name in repos_config:
                        print(f"Warning: Duplicate repo '{repo_name}' in {repos_file.name}", 
                              file=sys.stderr)
                        continue
                        
                    repos_config[repo_name] = {
                        'url': repo_info.get('url', ''),
                        'version': repo_info.get('version', ''),
                        'source_file': repos_file.name,
                        'workspace': workspace_name
                    }
                    
            except yaml.YAMLError as e:
                print(f"Error parsing {repos_file}: {e}", file=sys.stderr)
                
    return repos_config


def get_actual_repos():
    """
    Get all repositories actually present in workspace src directories.
    
    Returns:
        dict: Dictionary mapping repo name to actual location:
            {
                'repo_name': {
                    'path': Path,
                    'workspace': str,
                    'branch': str or None
                }
            }
    """
    workspace_root = Path(get_workspace_root())
    actual_repos = {}
    
    workspaces_dir = workspace_root / "workspaces"
    if not workspaces_dir.exists():
        return actual_repos
        
    # Find all *_ws directories
    for ws_dir in workspaces_dir.glob("*_ws"):
        src_dir = ws_dir / "src"
        if not src_dir.exists():
            continue
            
        workspace_name = ws_dir.name
        
        # Find all directories in src (each is potentially a repo)
        for repo_dir in src_dir.iterdir():
            if not repo_dir.is_dir() or repo_dir.name.startswith('.'):
                continue
                
            # Check if it's a git repo
            if (repo_dir / ".git").exists():
                # Get current branch
                branch = get_git_branch(repo_dir)
                
                actual_repos[repo_dir.name] = {
                    'path': repo_dir,
                    'workspace': workspace_name,
                    'branch': branch
                }
                
    return actual_repos


def get_git_branch(repo_path):
    """
    Get the current branch of a git repository.
    
    Args:
        repo_path: Path to the repository
        
    Returns:
        str: Branch name or None if detached HEAD or error
    """
    try:
        result = subprocess.run(
            ["git", "branch", "--show-current"],
            cwd=str(repo_path),
            capture_output=True,
            text=True,
            check=True
        )
        branch = result.stdout.strip()
        return branch if branch else None
    except subprocess.CalledProcessError:
        return None


def validate_workspace(verbose=False):
    """
    Validate workspace matches configuration.
    
    Args:
        verbose: If True, print detailed information
        
    Returns:
        tuple: (is_valid, missing_repos, extra_repos, version_mismatches)
    """
    print("Validating workspace against .repos configuration...")
    print()
    
    # Get configured and actual repos
    config_repos = get_all_repos_config()
    actual_repos = get_actual_repos()
    
    if verbose:
        print(f"Found {len(config_repos)} configured repositories")
        print(f"Found {len(actual_repos)} actual repositories in workspace")
        print()
    
    # Find missing repos (in config but not in workspace)
    missing_repos = []
    for repo_name, config in config_repos.items():
        if repo_name not in actual_repos:
            missing_repos.append({
                'name': repo_name,
                'config': config
            })
    
    # Find extra repos (in workspace but not in config)
    extra_repos = []
    for repo_name, actual in actual_repos.items():
        if repo_name not in config_repos:
            extra_repos.append({
                'name': repo_name,
                'actual': actual
            })
    
    # Find version mismatches (wrong branch/version)
    version_mismatches = []
    for repo_name in set(config_repos.keys()) & set(actual_repos.keys()):
        config = config_repos[repo_name]
        actual = actual_repos[repo_name]
        
        expected_version = config['version']
        actual_branch = actual['branch']
        
        # Check if workspace location matches
        expected_ws = config['workspace']
        actual_ws = actual['workspace']
        
        if expected_ws != actual_ws:
            version_mismatches.append({
                'name': repo_name,
                'type': 'workspace_mismatch',
                'expected': expected_ws,
                'actual': actual_ws
            })
        
        # Check if branch matches expected version
        # Note: version can be a branch, tag, or commit SHA
        if actual_branch and actual_branch != expected_version:
            version_mismatches.append({
                'name': repo_name,
                'type': 'version_mismatch',
                'expected': expected_version,
                'actual': actual_branch,
                'path': actual['path']
            })
    
    # Print results
    is_valid = len(missing_repos) == 0 and len(extra_repos) == 0 and len(version_mismatches) == 0
    
    print("=" * 60)
    print("Workspace Validation Results")
    print("=" * 60)
    print(f"Configured repos: {len(config_repos)}")
    print(f"Actual repos: {len(actual_repos)}")
    print()
    
    if missing_repos:
        print(f"❌ Missing repos (in config but NOT in workspace): {len(missing_repos)}")
        for repo in sorted(missing_repos, key=lambda x: x['name']):
            config = repo['config']
            print(f"   - {repo['name']}")
            if verbose:
                print(f"     Source: {config['source_file']}")
                print(f"     Workspace: {config['workspace']}")
                print(f"     URL: {config['url']}")
                print(f"     Version: {config['version']}")
        print()
    else:
        print("✅ All configured repos are present")
        print()
    
    if extra_repos:
        print(f"⚠️  Extra repos (in workspace but NOT in config): {len(extra_repos)}")
        for repo in sorted(extra_repos, key=lambda x: x['name']):
            actual = repo['actual']
            print(f"   - {repo['name']} (in {actual['workspace']})")
            if verbose:
                print(f"     Path: {actual['path']}")
                print(f"     Branch: {actual['branch']}")
        print()
    else:
        print("✅ No orphaned repos found")
        print()
    
    if version_mismatches:
        print(f"⚠️  Version/location mismatches: {len(version_mismatches)}")
        for mismatch in sorted(version_mismatches, key=lambda x: x['name']):
            if mismatch['type'] == 'workspace_mismatch':
                print(f"   - {mismatch['name']}: wrong workspace")
                print(f"     Expected: {mismatch['expected']}")
                print(f"     Actual: {mismatch['actual']}")
            else:  # version_mismatch
                print(f"   - {mismatch['name']}: wrong branch/version")
                print(f"     Expected: {mismatch['expected']}")
                print(f"     Actual: {mismatch['actual']}")
        print()
    else:
        print("✅ All repos on expected branches")
        print()
    
    print("=" * 60)
    if is_valid:
        print("✅ Workspace validation PASSED!")
    else:
        print("❌ Workspace validation FAILED")
        print()
        print("Run with --fix to automatically import missing repositories")
    print("=" * 60)
    
    return is_valid, missing_repos, extra_repos, version_mismatches


def fix_workspace(missing_repos, verbose=False):
    """
    Fix workspace by importing missing repositories.
    
    Args:
        missing_repos: List of missing repository information
        verbose: If True, print detailed information
        
    Returns:
        bool: True if all fixes succeeded
    """
    if not missing_repos:
        print("No missing repos to fix!")
        return True
    
    workspace_root = Path(get_workspace_root())
    print(f"Attempting to import {len(missing_repos)} missing repositories...")
    print()
    
    # Group repos by workspace
    repos_by_workspace = {}
    for repo in missing_repos:
        ws = repo['config']['workspace']
        if ws not in repos_by_workspace:
            repos_by_workspace[ws] = []
        repos_by_workspace[ws].append(repo)
    
    all_success = True
    
    for workspace, repos in repos_by_workspace.items():
        print(f"Importing {len(repos)} repos into {workspace}...")
        
        # Find the .repos file
        source_file = repos[0]['config']['source_file']
        repos_file = None
        
        # Check both possible locations
        for config_dir in [workspace_root / "configs", 
                          workspace_root / "workspaces/core_ws/src/unh_marine_autonomy/config/repos"]:
            candidate = config_dir / source_file
            if candidate.exists():
                repos_file = candidate
                break
        
        if not repos_file:
            print(f"  ❌ Could not find .repos file: {source_file}")
            all_success = False
            continue
        
        # Create workspace src directory if needed
        ws_src = workspace_root / "workspaces" / workspace / "src"
        ws_src.mkdir(parents=True, exist_ok=True)
        
        # Use vcs import to clone the missing repos
        try:
            cmd = ["vcs", "import", str(ws_src)]
            
            if verbose:
                print(f"  Running: {' '.join(cmd)} < {repos_file}")
            
            with open(repos_file, 'r') as f:
                result = subprocess.run(
                    cmd,
                    stdin=f,
                    cwd=str(workspace_root),
                    capture_output=True,
                    text=True
                )
                
            if result.returncode == 0:
                print(f"  ✅ Successfully imported repos")
                if verbose and result.stdout:
                    print(f"     {result.stdout}")
            else:
                print(f"  ❌ Failed to import repos")
                print(f"     {result.stderr}")
                all_success = False
                
        except Exception as e:
            print(f"  ❌ Error running vcs import: {e}")
            all_success = False
    
    print()
    return all_success


def main():
    parser = argparse.ArgumentParser(
        description="Validate workspace matches .repos configuration"
    )
    parser.add_argument(
        "--fix",
        action="store_true",
        help="Automatically import missing repositories using vcs"
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Print detailed information"
    )
    
    args = parser.parse_args()
    
    # Validate workspace
    is_valid, missing_repos, extra_repos, version_mismatches = validate_workspace(args.verbose)
    
    # If fix requested and there are missing repos, fix them
    if args.fix and missing_repos:
        print()
        if fix_workspace(missing_repos, args.verbose):
            print("✅ Workspace fixed successfully!")
            print()
            print("Re-validating workspace...")
            print()
            is_valid, _, _, _ = validate_workspace(args.verbose)
        else:
            print("❌ Some fixes failed. Please check the errors above.")
            sys.exit(1)
    
    # Exit with appropriate code
    sys.exit(0 if is_valid else 1)


if __name__ == "__main__":
    main()
