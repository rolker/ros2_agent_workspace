#!/usr/bin/env python3
"""Print the ``## Approach`` section of a work plan, verbatim.

Usage: _plan_approach.py <plan.md>

Helper for ``cross_model_review.sh`` (issue #320), which re-admits a
plan's Approach to the review prompt as labelled context. The section is
located with a real CommonMark parser (markdown-it-py) rather than a line
scanner, so fenced code, indented headings, setext headings and thematic
breaks are all classified exactly as a Markdown renderer classifies them:

* The section starts after the first top-level level-2 heading whose
  inline text is exactly ``Approach`` (ATX ``## Approach`` or setext
  ``Approach`` over a ``---`` underline). A ``## Approach`` line inside a
  fenced block is code, not a heading, so it never matches.
* It ends at the next top-level heading of level 1 or 2 (ATX, indented
  ATX, or setext) or thematic break, whichever comes first, or at EOF.
* An unclosed fence would, per CommonMark, run to the end of the document
  and pull every later line in. Instead the lines after its opener are
  re-parsed as if the opener were absent. Inside the Approach, the section
  then ends at the first boundary found there: shorter, never longer, than
  a fence-aware cut of a well-formed plan. Before the Approach, the same
  re-parse keeps an unclosed fence in an earlier section from hiding the
  ``## Approach`` heading (which would otherwise read as "no section").

The section's source lines (the lines between those two points) are
printed unchanged, except that line endings are normalised to ``\\n``.

Exit status:
    0  an Approach section was found and printed (it may be blank; the
       caller decides what an all-whitespace section means)
    1  the plan has no Approach section
    3  any other error (unreadable file, bad usage, parser failure); the
       reason is printed to stderr
    4  markdown-it-py is not importable by this interpreter

Codes 1 and 2 are also what python itself exits with (1 for an uncaught
exception, 2 for "can't open file" or a usage error), so the "try the
next interpreter" code is 4, which python never emits on its own: a
missing or unreadable copy of this script reads as an error (exit 2 falls
into the caller's error branch), never as a missing library.
"""

import re
import sys

EXIT_FOUND = 0
EXIT_NO_SECTION = 1
EXIT_ERROR = 3
EXIT_NO_LIBRARY = 4

# markdown-it's own line normalisation (rules_core/normalize): CRLF and a
# lone CR are both line breaks. Splitting the source the same way keeps
# token.map line numbers aligned with the lines printed here.
_LINE_BREAK = re.compile(r"\r\n|\r|\n")


def _split_lines(text):
    lines = _LINE_BREAK.split(text)
    # A trailing line break terminates the last line; it does not start
    # an empty one.
    if lines and lines[-1] == "":
        lines.pop()
    return lines


def _is_boundary(tok):
    """A top-level H1/H2 heading or thematic break ends the section."""
    if tok.level != 0:
        return False
    return tok.type == "hr" or (tok.type == "heading_open" and tok.tag in ("h1", "h2"))


def _fence_closed(tok, lines, offset):
    """Did this top-level fence token end at a closing fence line?

    markdown-it gives an unclosed fence a map running to the end of its
    container (for a top-level fence, the end of the document) and no
    closer. A closed fence's last mapped line is its closer: at most three
    spaces of indent, then the opener's character repeated at least as many
    times as the opener, then only spaces or tabs.
    """
    start, end = tok.map
    if end - start < 2:
        return False
    char = re.escape(tok.markup[0])
    closer = re.compile(r"^ {0,3}" + char + "{" + str(len(tok.markup)) + r",}[ \t]*$")
    return bool(closer.match(lines[offset + end - 1]))


def _top_level(md, lines, start=0):
    """Yield (offset, tokens, i) for each block token of lines[start:].

    `offset` is the line index that token.map is relative to. An unclosed
    top-level fence would, per CommonMark, swallow everything to EOF; the
    lines after its opener are instead re-parsed as if the opener were
    absent, and the walk continues there. Both the heading search and the
    section-end search walk this one stream, so an unclosed fence before
    the Approach can no more hide its heading than one inside the Approach
    can hide its end.

    Every restart point sits at a top-level block boundary (just after a
    heading, or just after an unclosed fence's opener), so parsing the
    remaining lines on their own classifies them as they are classified in
    the whole document.
    """
    while True:
        restart = None
        tokens = md.parse("\n".join(lines[start:]))
        for i, tok in enumerate(tokens):
            if tok.type == "fence" and tok.level == 0 and not _fence_closed(tok, lines, start):
                # `start` strictly increases, so the walk terminates.
                restart = start + tok.map[0] + 1
                break
            yield start, tokens, i
        if restart is None:
            return
        start = restart


def _section_end(md, lines, start):
    """Line index (exclusive) where the section beginning at `start` ends."""
    for offset, tokens, i in _top_level(md, lines, start):
        if _is_boundary(tokens[i]):
            return offset + tokens[i].map[0]
    return len(lines)


def extract(md, text):
    """Return the Approach section's lines, or None if there is none."""
    lines = _split_lines(text)
    for offset, tokens, i in _top_level(md, lines):
        tok = tokens[i]
        if tok.type != "heading_open" or tok.tag != "h2" or tok.level != 0:
            continue
        inline = tokens[i + 1] if i + 1 < len(tokens) else None
        if inline is None or inline.type != "inline" or inline.content.strip() != "Approach":
            continue
        start = offset + tok.map[1]
        return lines[start : _section_end(md, lines, start)]
    return None


def main(argv):
    if len(argv) != 2:
        print("usage: _plan_approach.py <plan.md>", file=sys.stderr)
        return EXIT_ERROR
    try:
        from markdown_it import MarkdownIt  # pylint: disable=import-outside-toplevel
    except ImportError:
        return EXIT_NO_LIBRARY
    try:
        # surrogateescape round-trips any byte that is not valid UTF-8, so a
        # stray Latin-1 byte in a plan is carried through, not a crash.
        with open(argv[1], encoding="utf-8", errors="surrogateescape", newline="") as fh:
            text = fh.read()
        section = extract(MarkdownIt("commonmark"), text)
        if section is None:
            return EXIT_NO_SECTION
        out = "".join(line + "\n" for line in section)
        sys.stdout.buffer.write(out.encode("utf-8", errors="surrogateescape"))
        sys.stdout.flush()
    # Every failure must map to EXIT_ERROR: an uncaught exception would exit
    # 1, which the caller reads as "no Approach section".
    except Exception as exc:  # pylint: disable=broad-except
        print(f"_plan_approach.py: {argv[1]}: {type(exc).__name__}: {exc}", file=sys.stderr)
        return EXIT_ERROR
    return EXIT_FOUND


if __name__ == "__main__":
    sys.exit(main(sys.argv))
