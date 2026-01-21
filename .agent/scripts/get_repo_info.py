#!/usr/bin/env python3
"""
Get Repository Info

This script searches for a repository in the workspace .repos files and returns
specific information (default: version/branch).

Usage:
    python3 get_repo_info.py <repo_name>

Output:
    The version string (e.g. "jazzy", "main", "feature/foo")
    Returns "unknown" if not found.
"""

import os
import sys
import yaml
import glob
import argparse

WORKSPACE_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def find_repo_version(target_repo):
    # Find all .repos files in configs/ and migrated Key Repo paths
    # Note: We include underlay.repos here because we want to validate ALL repos
    config_dirs = [
        os.path.join(WORKSPACE_ROOT, "configs"),
        os.path.join(WORKSPACE_ROOT, "workspaces/core_ws/src/unh_marine_autonomy/config/repos")
    ]
    
    repo_files = []
    for d in config_dirs:
        if os.path.isdir(d):
            repo_files.extend(glob.glob(os.path.join(d, "*.repos")))
    
    for repo_file in repo_files:
        with open(repo_file, 'r') as f:
            try:
                data = yaml.safe_load(f)
                if not data or 'repositories' not in data:
                    continue
                
                if target_repo in data['repositories']:
                    repo_info = data['repositories'][target_repo]
                    return repo_info.get("version", "unknown")
                    
            except yaml.YAMLError:
                continue
                
    return "unknown"

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Get repository info from .repos files")
    parser.add_argument("repo_name", help="Name of the repository to look up")
    args = parser.parse_args()
    
    version = find_repo_version(args.repo_name)
    print(version)
