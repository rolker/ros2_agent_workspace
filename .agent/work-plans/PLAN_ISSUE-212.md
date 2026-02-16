# Work Plan: Issue #212 - CLAUDE.md tells agents to use `pre-commit run` but pre-commit is not on PATH

**Issue**: #212
**Title**: CLAUDE.md tells agents to use `pre-commit run` but pre-commit is not on PATH
**Assignee**: AI Agent
**Started**: 2026-02-16
**Status**: ✅ Completed

---

## Problem Analysis

Agent instruction files (`CLAUDE.md`, `.github/copilot-instructions.md`,
`.agent/instructions/gemini-cli.instructions.md`) told agents to run
`pre-commit run --all-files` for linting. However, `pre-commit` is installed
inside a Python virtual environment and is not on the agent's default `$PATH`.
The `make lint` target already wraps `pre-commit` correctly using the venv.

## Proposed Approach

Replace `pre-commit run --all-files` with `make lint` in all agent instruction
files. Leave developer-facing documentation (`.agent/hooks/README.md`) unchanged
since those references describe the pre-commit hooks system itself and are aimed
at human developers who have `pre-commit` available.

## Implementation Tasks

### Phase 1: Update agent instruction files
- [x] Update `CLAUDE.md` — change `pre-commit run --all-files` to `make lint`
- [x] Update `.github/copilot-instructions.md` — same change
- [x] Update `.agent/instructions/gemini-cli.instructions.md` — same change

### Phase 2: Verification
- [x] Grep all three files to confirm zero remaining `pre-commit run` references
- [x] Run `make lint` to confirm hooks pass

## Design Decisions

### ✅ Use `make lint` instead of adding pre-commit to PATH
**Rationale**: `make lint` already exists and correctly activates the venv.
Adding `pre-commit` to `$PATH` globally would be more fragile and require
changes to `env.sh`.

### ✅ Leave `.agent/hooks/README.md` unchanged
**Rationale**: Those references document the pre-commit hooks system for human
developers, not agent-executable instructions. They correctly describe how to
run individual hooks with `pre-commit run`.

## Success Criteria

- [x] No agent instruction file references `pre-commit run --all-files`
- [x] `make lint` passes cleanly
- [x] PR addresses Copilot review feedback

---

**Authored-By**: `Claude Code Agent`
**Model**: `Claude Opus 4.6`
