#!/usr/bin/env python3
"""
List Overlay Repositories

This script scans the configs/ directory for .repos files and outputs a JSON
list of all repositories defined across all layers (excluding underlay.repos).

Usage:
    python3 list_overlay_repos.py

Output:
    JSON array containing:
    - Repository name
    - URL
    - Version/branch
    - Source .repos file
"""

import os
import sys
import yaml
import json
import glob
import argparse

WORKSPACE_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CONFIGS_DIR = os.path.join(WORKSPACE_ROOT, "configs")
IGNORED_FILES = ["underlay.repos"]


def get_overlay_repos(include_underlay=False):
    repos_list = []
    
    # Find all .repos files in configs/ and migrated Key Repo paths
    config_dirs = [
        os.path.join(WORKSPACE_ROOT, "configs"),
        os.path.join(WORKSPACE_ROOT, "workspaces/core_ws/src/unh_marine_autonomy/config/repos")
    ]
    
    repo_files = []
    for d in config_dirs:
        if os.path.isdir(d):
            repo_files.extend(glob.glob(os.path.join(d, "*.repos")))
    
    for repo_file in repo_files:
        filename = os.path.basename(repo_file)
        if filename in IGNORED_FILES and not include_underlay:
            continue
            
        with open(repo_file, 'r') as f:
            try:
                data = yaml.safe_load(f)
                if not data or 'repositories' not in data:
                    continue
                
                for name, info in data['repositories'].items():
                    entry = {
                        "name": name,
                        "url": info.get("url", ""),
                        "version": info.get("version", ""),
                        "source_file": filename
                    }
                    repos_list.append(entry)
            except yaml.YAMLError as e:
                print(f"Error parsing {filename}: {e}", file=sys.stderr)
                
    return repos_list

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="List repositories defined in .repos files")
    parser.add_argument("--include-underlay", action="store_true", help="Include repositories from underlay.repos")
    parser.add_argument("--format", choices=["json", "names"], default="json", help="Output format")
    
    args = parser.parse_args()
    
    repos = get_overlay_repos(include_underlay=args.include_underlay)
    
    if args.format == "names":
        for repo in repos:
            print(repo["name"])
    else:
        print(json.dumps(repos, indent=2))
