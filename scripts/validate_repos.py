#!/usr/bin/env python3
"""
Validate .repos files for correct syntax and structure.

This script checks that all .repos files in the configs/ directory:
- Are valid YAML
- Have a 'repositories' key
- Each repository has required fields (type, url, version)
- Repository names are unique across all files
"""

import argparse
import sys
from pathlib import Path
import yaml


def validate_repos_file(file_path):
    """
    Validate a single .repos file.

    Args:
        file_path: Path to the .repos file

    Returns:
        tuple: (is_valid, errors_list)
    """
    errors = []

    try:
        with open(file_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
    except yaml.YAMLError as e:
        errors.append(f"YAML parsing error: {e}")
        return False, errors
    except Exception as e:
        errors.append(f"Error reading file: {e}")
        return False, errors

    # Check for repositories key
    if not data or "repositories" not in data:
        errors.append("Missing 'repositories' key")
        return False, errors

    repos = data["repositories"]
    if not isinstance(repos, dict):
        errors.append("'repositories' should be a dictionary")
        return False, errors

    # Validate each repository
    for repo_name, repo_data in repos.items():
        if not isinstance(repo_data, dict):
            errors.append(f"Repository '{repo_name}': should be a dictionary")
            continue

        # Check required fields
        required_fields = ["type", "url", "version"]
        for field in required_fields:
            if field not in repo_data:
                errors.append(f"Repository '{repo_name}': missing required field '{field}'")

        # Validate type
        if "type" in repo_data:
            valid_types = ["git", "hg", "svn", "bzr"]
            if repo_data["type"] not in valid_types:
                errors.append(
                    f"Repository '{repo_name}': invalid type '{repo_data['type']}', "
                    f"must be one of {valid_types}"
                )

    return len(errors) == 0, errors


def check_duplicate_repos(configs_dir):
    """
    Check for duplicate repository names across all .repos files.

    Args:
        configs_dir: Path to configs directory

    Returns:
        list: List of duplicate repository names
    """
    repo_names = {}
    duplicates = []

    for repos_file in Path(configs_dir).glob("*.repos"):
        try:
            with open(repos_file, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
                if data and "repositories" in data:
                    for repo_name in data["repositories"].keys():
                        if repo_name in repo_names:
                            duplicates.append(
                                f"Repository '{repo_name}' appears in both "
                                f"{repo_names[repo_name]} and {repos_file.name}"
                            )
                        else:
                            repo_names[repo_name] = repos_file.name
        except Exception:
            # Skip files that can't be read (will be caught by validate_repos_file)
            pass

    return duplicates


def main():
    parser = argparse.ArgumentParser(
        description="Validate .repos files in the configs directory"
    )
    parser.add_argument(
        "--configs-dir",
        default="configs",
        help="Directory containing .repos files (default: configs)",
    )
    parser.add_argument(
        "--strict", action="store_true", help="Exit with error on any warning"
    )

    args = parser.parse_args()
    configs_dir = Path(args.configs_dir)

    if not configs_dir.exists():
        print(f"Error: Directory '{configs_dir}' does not exist", file=sys.stderr)
        sys.exit(1)

    # Find all .repos files
    repos_files = list(configs_dir.glob("*.repos"))
    if not repos_files:
        print(f"Warning: No .repos files found in '{configs_dir}'", file=sys.stderr)
        sys.exit(0 if not args.strict else 1)

    print(f"Validating {len(repos_files)} .repos files in {configs_dir}...")
    print()

    all_valid = True

    # Validate each file
    for repos_file in sorted(repos_files):
        print(f"Checking {repos_file.name}...", end=" ")
        is_valid, errors = validate_repos_file(repos_file)

        if is_valid:
            print("✅ OK")
        else:
            print("❌ FAILED")
            all_valid = False
            for error in errors:
                print(f"  - {error}")

    print()

    # Check for duplicates
    print("Checking for duplicate repository names...", end=" ")
    duplicates = check_duplicate_repos(configs_dir)
    if duplicates:
        print("❌ DUPLICATES FOUND")
        all_valid = False
        for dup in duplicates:
            print(f"  - {dup}")
    else:
        print("✅ OK")

    print()

    if all_valid:
        print("✅ All validation checks passed!")
        sys.exit(0)
    else:
        print("❌ Validation failed. Please fix the errors above.")
        sys.exit(1)


if __name__ == "__main__":
    main()
