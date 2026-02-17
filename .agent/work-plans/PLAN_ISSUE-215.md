# Work Plan: Issue #215 - build_report_generator.py shows 0 packages due to ast.literal_eval failure on OrderedDict

**Issue**: #215
**Title**: build_report_generator.py shows 0 packages due to ast.literal_eval failure on OrderedDict
**Assignee**: Claude Code Agent
**Started**: 2026-02-17
**Status**: ✅ Complete

---

## Problem Analysis

The `build_report_generator.py` script was reporting "0 packages" for every layer even
when builds completed successfully. The root cause: colcon's `events.log` uses Python
`repr()` for event data, which includes `OrderedDict(...)` objects. The script called
`ast.literal_eval()` on every log line — but `ast.literal_eval` only supports Python
literals (strings, numbers, dicts, lists, etc.), not arbitrary constructors like
`OrderedDict`. This caused `ValueError` on non-literal lines (e.g., `JobQueued`,
`Command`), preventing the `JobQueued` events from ever being recorded — resulting in
an empty `packages` set.

## Proposed Approach

Restructure parsing so that `ast.literal_eval` is only called on event types that
actually need parsed data (`JobEnded`, to extract the return code). Other event types
(`JobQueued`, `StderrLine`, `Command`) are handled via regex or string presence alone.

This avoids the `OrderedDict` issue entirely since `JobEnded` events use plain dicts.

## Implementation Tasks

### Phase 1: Fix parsing logic
- [x] Extract event type and package name via regex before attempting `ast.literal_eval`
- [x] Only call `ast.literal_eval` inside the `JobEnded` branch
- [x] Handle `JobQueued` by simply adding the package name (no data parsing needed)
- [x] Handle `StderrLine` by simply adding the package name (no data parsing needed)

### Phase 2: Harden error handling
- [x] Default `rc` to `1` (failure) when `ast.literal_eval` fails on a `JobEnded` event,
      so unknown log formats surface as failures rather than being silently marked green

## Design Decisions

### ✅ Regex-first parsing instead of fixing ast.literal_eval
**Rationale**: Replacing `ast.literal_eval` with a more permissive parser (e.g., regex
for `rc`) would also work, but restructuring the control flow to only parse when needed
is simpler, more robust, and avoids parsing data we don't use.

### ✅ Conservative failure default (rc=1) for unparseable JobEnded events
**Rationale**: If we can't determine the return code, it's safer to report failure than
to silently assume success. A false positive (flagged failure for an actually-successful
build) is easier to diagnose than a false negative (hidden failure marked green).

## Success Criteria

- [x] `build_report_generator.py` correctly counts packages from `events.log`
- [x] Build report shows accurate pass/fail counts for each layer
- [x] Unparseable `JobEnded` events default to failure (rc=1), not success
- [x] No regression: valid `JobEnded` events still parsed correctly

## Notes

The fix was verified by inspecting actual `events.log` files from workspace builds.
The `JobEnded` events consistently use plain dict syntax (`{'rc': 0}`), so
`ast.literal_eval` works for them. The problem was only with `JobQueued` and `Command`
events that include `OrderedDict(...)`.

---

**Authored-By**: `Claude Code Agent`
**Model**: `Claude Opus 4.6`
