#!/usr/bin/env python3
"""Extract entries from a work-plan ``progress.md`` as JSON.

``progress.md`` files (``.agent/work-plans/issue-<N>/progress.md``) are the
per-issue lifecycle timeline. Each entry is a ``## <Entry Type>`` section
following the ADR-0013 schema. This tool parses those entries into structured
JSON so consumers — notably the ``triage-reviews`` integrator (#470 phase B) and
the sub-agent orchestrator (#481 phase C) — can filter by entry type and
correlation key without re-parsing markdown.

Usage::

    python3 .agent/scripts/progress_read.py <progress.md> [--type "Entry Type"] ...

Output (stdout) is a JSON object::

    {"file": <path>, "issue": <N or null>, "entries": [ <entry>, ... ]}

Each entry::

    {
      "type": "Plan Review",         # full heading text (may carry a suffix)
      "base_type": "Plan Review",    # canonical type (suffix stripped if non-canonical)
      "recognized": true,            # is base_type a canonical ADR-0013 type?
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

``--type`` filters the emitted entries to the given type(s), matching on the
full heading, the canonical ``base_type`` (so legacy suffixed headings like
``External Review (Round 5-6)`` match ``External Review``), and predecessor
recognition (filtering ``Integrated Review`` also returns ``External Review``
entries, since ADR-0013 recognizes the latter as the predecessor of the former).
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
# Fenced code block delimiter (``` or ~~~, optionally indented / with info string).
_FENCE = re.compile(r"^\s*(?:```|~~~)")


def _canonical_base(heading):
    """Map a heading to its canonical ADR-0013 entry type.

    Returns the heading unchanged when it is already canonical (including
    ``Local Review (Pre-Push)``, whose parenthetical is part of the canonical
    name). Otherwise strips a trailing ``(...)`` suffix and returns the base if
    that base is canonical — so legacy non-conformant headings like
    ``External Review (Round 5-6)`` are still recognized as ``External Review``
    (and thus consumed as ``Integrated Review`` predecessors) rather than
    silently dropped by type filters. Falls back to the heading unchanged.
    """
    if heading in CANONICAL_TYPES:
        return heading
    match = re.match(r"^(.*?)\s*\([^)]*\)\s*$", heading)
    if match and match.group(1) in CANONICAL_TYPES:
        return match.group(1)
    return heading


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
    """Collect checkbox items appearing under a ``### `` sub-section of an entry.

    Checkboxes before the first sub-section header are ignored: ADR-0013 places
    findings/actions checkbox lists under `### Findings` / `### Actions` /
    `### Open questions`, so a stray checkbox in the header area is not a finding.
    """
    findings = []
    section = None
    for line in body_lines:
        sub = _SUBSECTION.match(line)
        if sub:
            section = sub.group(1).strip()
            continue
        box = _CHECKBOX.match(line)
        if box and section is not None:
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
            # Only treat the block as YAML frontmatter if it actually contains a
            # `key:` line; otherwise a leading `---` horizontal rule with a later
            # `---` would swallow real body content.
            if re.search(r"^\s*[\w-]+:\s", front, re.MULTILINE):
                body = text[end + 4 :]
                match = re.search(r"^\s*issue:\s*(\d+)\s*$", front, re.MULTILINE)
                issue = int(match.group(1)) if match else None
                return issue, body.lstrip("\n")
    return None, text


def _split_entries(body):
    """Yield (heading, [lines]) for each ``## `` entry block.

    Fence-aware: lines inside fenced code blocks are skipped entirely, so a
    ``## <Type>`` heading or checkbox quoted inside a ``` ```markdown ``` block
    (e.g. an embedded template or a quoted prior entry) does not produce a
    phantom entry. Downstream parsing (header fields, findings) therefore never
    sees fenced content.
    """
    entries = []
    current_heading = None
    current_lines = []
    in_fence = False
    for line in body.splitlines():
        if _FENCE.match(line):
            in_fence = not in_fence
            continue
        if in_fence:
            continue
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
        base = _canonical_base(heading)
        entry = {
            "type": heading,
            "base_type": base,
            "recognized": base in CANONICAL_TYPES,
            "predecessor_of": PREDECESSOR_OF.get(base),
            "status": _field(header_lines, "Status"),
            "when": when,
            "when_has_offset": bool(when and _OFFSET.search(when.strip())),
            "by": _field(header_lines, "By"),
            "correlation": _parse_correlation(base, header_lines),
            "findings": _parse_findings(lines),
        }
        result["entries"].append(entry)

    return result


def _matches_type(entry, wanted):
    """True if ``entry`` should be emitted for the ``--type`` filter.

    Matches on the full heading, on the canonical base type (so legacy
    suffixed headings like ``External Review (Round 5-6)`` match
    ``External Review``), and on predecessor recognition (requesting
    ``Integrated Review`` also matches ``External Review`` entries).
    """
    if entry["type"] in wanted or entry.get("base_type") in wanted:
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
