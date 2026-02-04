#!/usr/bin/env python3
"""
Parse Feature Track issue status and return machine-readable JSON.

Usage:
    read_feature_status.py --issue <number>
    
Output JSON format:
    {
        "phase": 2,
        "current_task": "Write implementation code",
        "status": "in_progress",
        "percent_complete": 65
    }
"""

import argparse
import json
import re
import subprocess
import sys
from typing import Dict, List, Optional, Tuple


def get_issue_body(issue_number: int) -> str:
    """Fetch issue body from GitHub using gh CLI."""
    try:
        result = subprocess.run(
            ['gh', 'issue', 'view', str(issue_number), '--json', 'body', '-q', '.body'],
            capture_output=True,
            text=True,
            check=True
        )
        return result.stdout
    except subprocess.CalledProcessError as e:
        print(f"Error fetching issue #{issue_number}: {e.stderr}", file=sys.stderr)
        sys.exit(1)


def parse_checkboxes(body: str) -> List[Tuple[bool, str, int]]:
    """
    Parse markdown checkboxes from issue body.
    
    Returns:
        List of (checked, task_text, phase) tuples
    """
    tasks = []
    current_phase = 0
    
    # Match markdown checkboxes: - [ ] or - [x] or - [X]
    checkbox_pattern = re.compile(r'^[\s-]*\[([xX\s])\]\s+(.+)$', re.MULTILINE)
    
    # Match phase headers (e.g., "### Phase 1:", "## Phase 2:")
    phase_pattern = re.compile(r'^#+\s*Phase\s+(\d+)', re.IGNORECASE | re.MULTILINE)
    
    lines = body.split('\n')
    
    for line in lines:
        # Check if this line defines a new phase
        phase_match = phase_pattern.match(line)
        if phase_match:
            current_phase = int(phase_match.group(1))
            continue
        
        # Check if this line is a checkbox
        checkbox_match = checkbox_pattern.match(line)
        if checkbox_match:
            checked = checkbox_match.group(1).strip().lower() == 'x'
            task_text = checkbox_match.group(2).strip()
            tasks.append((checked, task_text, current_phase))
    
    return tasks


def calculate_status(tasks: List[Tuple[bool, str, int]]) -> Dict[str, any]:
    """
    Calculate feature status from parsed tasks.
    
    Returns:
        Dict with phase, current_task, status, percent_complete
    """
    if not tasks:
        return {
            "phase": 0,
            "current_task": None,
            "status": "done",
            "percent_complete": 100
        }
    
    total_tasks = len(tasks)
    completed_tasks = sum(1 for checked, _, _ in tasks if checked)
    
    # Find first unchecked task
    current_task = None
    current_phase = 0
    
    for checked, task_text, phase in tasks:
        if not checked:
            current_task = task_text
            current_phase = phase
            break
    
    # Determine status
    if current_task is None:
        # All tasks completed
        status = "done"
        phase = tasks[-1][2] if tasks else 0  # Last phase
    else:
        # Check if there's any indication of being blocked
        # (Could be enhanced to detect specific "BLOCKED" markers)
        status = "in_progress"
        phase = current_phase
    
    percent_complete = int((completed_tasks / total_tasks) * 100) if total_tasks > 0 else 100
    
    return {
        "phase": phase,
        "current_task": current_task,
        "status": status,
        "percent_complete": percent_complete
    }


def main():
    parser = argparse.ArgumentParser(
        description='Parse Feature Track issue status and return JSON'
    )
    parser.add_argument(
        '--issue',
        type=int,
        required=True,
        help='GitHub issue number'
    )
    parser.add_argument(
        '--pretty',
        action='store_true',
        help='Pretty-print JSON output'
    )
    
    args = parser.parse_args()
    
    # Fetch issue body
    body = get_issue_body(args.issue)
    
    # Parse checkboxes
    tasks = parse_checkboxes(body)
    
    # Calculate status
    status = calculate_status(tasks)
    
    # Output JSON
    if args.pretty:
        print(json.dumps(status, indent=2))
    else:
        print(json.dumps(status))


if __name__ == '__main__':
    main()
