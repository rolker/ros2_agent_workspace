#!/usr/bin/env python3
"""Extract entries from a work-plan ``progress.md`` as JSON.

``progress.md`` files (``.agent/work-plans/issue-<N>/progress.md``) are the
per-issue lifecycle timeline. Each entry is a ``## <Entry Type>`` section
following the ADR-0013 schema. This tool parses those entries into structured
JSON so consumers — notably the ``triage-reviews`` integrator (#470 phase B) and
the sub-agent orchestrator (#481 phase C) — can filter by entry type and
correlation key without re-parsing markdown.

Usage::

    progress_read.py <progress.md> [--type "Entry Type"] ...

Output (stdout) is a JSON object::

    {"file": <path>, "issue": <N or null>, "entries": [ <entry>, ... ]}

Each entry::

    {
      "type": "Plan Review",
      "recognized": true,            # canonical ADR-0013 type?
      "predecessor_of": null,        # "Integrated Review" for External Review
      "status": "complete",
      "when": "2026-05-25 15:27 -04:00",
      "when_has_offset": true,       # ADR-0013 requires an explicit offset
      "by": "...",
      "correlation": {...} or null,
      "findings": [ {"section","checked","source_hint","text"}, ... ]
    }

Correlation by entry type (ADR-0013 "Consume by entry-type filter"):

* ``Issue Review``                         -> ``{"kind":"issue","issue":N}``
* ``Plan Authored`` / ``Plan Review``      -> ``{"kind":"plan","path":..,"sha":..}``
* ``Local Review`` / ``Local Review (Pre-Push)`` / ``Integrated Review`` /
  ``External Review`` / ``Implementation`` -> ``{"kind":"pr","pr":N,"sha":..}``
  or ``{"kind":"branch","branch":..,"sha":..}``

``--type`` filters the emitted entries to the given type(s). Filtering on
``Integrated Review`` also returns ``External Review`` entries, since
ADR-0013 recognizes the latter as the predecessor of the former.
"""

import argparse
import json
import re
import sys
from pathlib import Path

# Canonical entry types (ADR-0013 Decision table).
CANONICAL_TYPES = {
    "Issue Review",
    "Plan Authored",
    "Plan Review",
    "Local Review",
    "Local Review (Pre-Push)",
    "Integrated Review",
    "External Review",
    "Implementation",
}

# ADR-0013 "Predecessor recognition": new writes use the value, but historical
# entries under the key must still be consumed as that value's history.
PREDECESSOR_OF = {"External Review": "Integrated Review"}

# Types whose correlation key is a PR/branch head SHA.
_PR_BRANCH_TYPES = {
    "Local Review",
    "Local Review (Pre-Push)",
    "Integrated Review",
    "External Review",
    "Implementation",
}
_PLAN_TYPES = {"Plan Authored", "Plan Review"}

# A level-2 heading that is not a level-3 (``### ``) heading.
_ENTRY_HEADING = re.compile(r"^## ([^#].*)$")
_SUBSECTION = re.compile(r"^### (.+)$")
_CHECKBOX = re.compile(r"^- \[([ xX])\]\s+(.*)$")
_LEADING_PAREN = re.compile(r"^\(([^)]*)\)")
_OFFSET = re.compile(r"(?:[+-]\d{2}:\d{2}|Z)$")


def _field(header_lines, name):
    """Return the value of ``**name**:`` found anywhere in the header lines.

    ADR-0013 fixes the field by name, not by line offset, so skill-specific
    fields may precede the canonical correlation field. Returns ``None`` if
    absent.
    """
    pattern = re.compile(r"^\*\*" + re.escape(name) + r"\*\*:\s*(.*)$")
    for line in header_lines:
        match = pattern.match(line.strip())
        if match:
            return match.group(1).strip()
    return None


def _strip_ticks(value):
    return value.strip().strip("`").strip()


def _corr_issue(header_lines):
    raw = _field(header_lines, "Issue")
    match = re.search(r"#?(\d+)", raw) if raw else None
    return {"kind": "issue", "issue": int(match.group(1))} if match else None


def _corr_plan(header_lines):
    raw = _field(header_lines, "Plan")
    # `<path>` at `<sha>`
    match = re.match(r"`?(.+?)`?\s+at\s+`?([0-9a-fA-F]+)`?$", raw) if raw else None
    if not match:
        return None
    return {
        "kind": "plan",
        "path": _strip_ticks(match.group(1)),
        "sha": match.group(2),
    }


def _corr_pr_or_branch(header_lines):
    """``**PR**: #N at `sha``` or, as the alternative, ``**Branch**: <name> at
    `sha```."""
    raw = _field(header_lines, "PR")
    if raw:
        match = re.search(r"#(\d+)\s+at\s+`?([0-9a-fA-F]+)`?", raw)
        if match:
            return {"kind": "pr", "pr": int(match.group(1)), "sha": match.group(2)}
    raw = _field(header_lines, "Branch")
    if raw:
        match = re.match(r"(\S+)\s+at\s+`?([0-9a-fA-F]+)`?", raw)
        if match:
            return {
                "kind": "branch",
                "branch": match.group(1),
                "sha": match.group(2),
            }
    return None


def _parse_correlation(entry_type, header_lines):
    """Extract the correlation key appropriate to ``entry_type`` (ADR-0013)."""
    if entry_type == "Issue Review":
        return _corr_issue(header_lines)
    if entry_type in _PLAN_TYPES:
        return _corr_plan(header_lines)
    if entry_type in _PR_BRANCH_TYPES:
        return _corr_pr_or_branch(header_lines)
    return None


def _parse_findings(body_lines):
    """Collect checkbox items across all ``### `` sub-sections of an entry."""
    findings = []
    section = None
    for line in body_lines:
        sub = _SUBSECTION.match(line)
        if sub:
            section = sub.group(1).strip()
            continue
        box = _CHECKBOX.match(line)
        if box:
            text = box.group(2).strip()
            hint = _LEADING_PAREN.match(text)
            findings.append(
                {
                    "section": section,
                    "checked": box.group(1).lower() == "x",
                    "source_hint": hint.group(1) if hint else None,
                    "text": text,
                }
            )
    return findings


def _split_frontmatter(text):
    """Return (issue_number_or_None, body_without_frontmatter)."""
    if text.startswith("---"):
        end = text.find("\n---", 3)
        if end != -1:
            front = text[3:end]
            body = text[end + 4 :]
            match = re.search(r"^\s*issue:\s*(\d+)\s*$", front, re.MULTILINE)
            issue = int(match.group(1)) if match else None
            return issue, body.lstrip("\n")
    return None, text


def _split_entries(body):
    """Yield (heading, [lines]) for each ``## `` entry block."""
    entries = []
    current_heading = None
    current_lines = []
    for line in body.splitlines():
        heading = _ENTRY_HEADING.match(line)
        if heading:
            if current_heading is not None:
                entries.append((current_heading, current_lines))
            current_heading = heading.group(1).strip()
            current_lines = []
        elif current_heading is not None:
            current_lines.append(line)
    if current_heading is not None:
        entries.append((current_heading, current_lines))
    return entries


def parse_progress(text, path=None):
    """Parse ``progress.md`` text into the JSON-able dict described in the
    module docstring."""
    issue, body = _split_frontmatter(text)
    result = {"file": path, "issue": issue, "entries": []}

    for heading, lines in _split_entries(body):
        # Header lines run until the first ``### `` sub-section.
        header_lines = []
        for line in lines:
            if _SUBSECTION.match(line):
                break
            header_lines.append(line)

        when = _field(header_lines, "When")
        entry = {
            "type": heading,
            "recognized": heading in CANONICAL_TYPES,
            "predecessor_of": PREDECESSOR_OF.get(heading),
            "status": _field(header_lines, "Status"),
            "when": when,
            "when_has_offset": bool(when and _OFFSET.search(when.strip())),
            "by": _field(header_lines, "By"),
            "correlation": _parse_correlation(heading, header_lines),
            "findings": _parse_findings(lines),
        }
        result["entries"].append(entry)

    return result


def _matches_type(entry, wanted):
    """True if ``entry`` should be emitted for the ``--type`` filter.

    Exact type match, plus predecessor recognition: requesting
    ``Integrated Review`` also matches ``External Review`` entries.
    """
    if entry["type"] in wanted:
        return True
    return entry["predecessor_of"] in wanted


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("path", help="path to a progress.md file")
    parser.add_argument(
        "--type",
        action="append",
        default=[],
        dest="types",
        metavar="ENTRY_TYPE",
        help="filter to this entry type (repeatable); Integrated Review also "
        "matches External Review predecessors",
    )
    args = parser.parse_args(argv)

    path = Path(args.path)
    if not path.is_file():
        print(f"error: not a file: {path}", file=sys.stderr)
        return 1

    result = parse_progress(path.read_text(encoding="utf-8"), path=str(path))
    if args.types:
        wanted = set(args.types)
        result["entries"] = [e for e in result["entries"] if _matches_type(e, wanted)]

    json.dump(result, sys.stdout, indent=2)
    sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
