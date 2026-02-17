#!/usr/bin/env python3
"""
Build Report Generator for ROS2 Agent Workspace

This script parses colcon's events.log file and generates a markdown table row
summarizing the build results for a single workspace layer.

Usage:
    python3 build_report_generator.py --log-dir <path> --layer-name <name>

Output:
    Prints a markdown table row with:
    - Layer name
    - Package counts (total/successful)
    - Failed/warning packages
    - Status emoji
"""

import argparse
import ast
import os
import re


def parse_args():
    parser = argparse.ArgumentParser(description="Generate build report from colcon events.log")
    parser.add_argument(
        "--log-dir", required=True, help="Path to the directory containing events.log"
    )
    parser.add_argument("--layer-name", required=True, help="Name of the workspace layer")
    return parser.parse_args()


_LOG_LINE_RE = re.compile(r"^\[[\d.]+\] \(([^)]+)\) (\w+): (.*)$")


def parse_log_line(line):
    """
    Parse a colcon log line into (pkg_name, event_type, data_str).

    Returns the raw data string rather than parsing it, because some event
    types (e.g. JobQueued, Command) contain non-literal Python objects like
    OrderedDict() that ast.literal_eval cannot handle.  Callers parse the
    data string only for event types that need it.
    """
    m = _LOG_LINE_RE.match(line)
    if not m:
        return None
    return m.group(1), m.group(2), m.group(3)


def main():
    args = parse_args()
    log_file = os.path.join(args.log_dir, "events.log")

    if not os.path.exists(log_file):
        print(f"| {args.layer_name} | 0 | - | ⚠️ Log Not Found |")
        return

    packages = set()
    failed_packages = set()
    stderr_packages = set()

    # Track completion to ensure we don't count skipped/aborted as success blindly
    # Dictionary of pkg -> return_code
    results = {}

    with open(log_file, "r") as f:
        for line in f:
            parsed = parse_log_line(line.strip())
            if not parsed:
                continue

            pkg_name, event_type, data_str = parsed

            if event_type == "JobQueued":
                packages.add(pkg_name)

            elif event_type == "JobEnded":
                try:
                    data = ast.literal_eval(data_str)
                    rc = data.get("rc", 0)
                except (ValueError, SyntaxError):
                    # Default to failure when we can't parse the return code;
                    # silently assuming success would hide real build failures.
                    rc = 1
                results[pkg_name] = rc
                if rc != 0:
                    failed_packages.add(pkg_name)

            elif event_type == "StderrLine":
                stderr_packages.add(pkg_name)

    # Compile Summary
    total = len(packages)
    failed_count = len(failed_packages)
    success_count = total - failed_count

    # Determine status icon
    if failed_count > 0:
        status = "❌ Failed"
    elif total == 0:
        status = "⚠️ No Pkgs"
    else:
        status = "✅ Success"

    # Formatting lists
    failed_list = ", ".join(sorted(list(failed_packages))) if failed_packages else "-"
    stderr_list = ", ".join(sorted(list(stderr_packages))) if stderr_packages else "-"

    if failed_packages:
        # If failed, we highlight the failed ones in the output column preferentially
        output_col = f"**Failed**: {failed_list}"
        if stderr_packages:
            # Add other warnings if needed, but failures are priority
            # Remove failed from stderr to just see "warnings"
            warnings = stderr_packages - failed_packages
            if warnings:
                output_col += f"<br/>**Warnings**: {', '.join(sorted(list(warnings)))}"
    else:
        output_col = stderr_list if stderr_packages else "-"

    # Output Markdown Row
    # | Layer | Packages (Total/Success) | Output (Warnings/Errors) | Status |
    print(f"| {args.layer_name} | {total} (OK: {success_count}) | {output_col} | {status} |")


if __name__ == "__main__":
    main()
