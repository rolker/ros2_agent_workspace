# Plan: progress_append.sh: eliminate finish-phase permission prompts

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/594

## Context

When a dispatched phase (container or in-process sub-agent) completes, the host
session re-invokes to read results and write a `progress.md` entry. Three
patterns in that path hit permission prompts:

1. **Ad-hoc interpreter heredocs** (`python3 - <<'EOF' …`) — used for one-off
   JSON/progress parsing (~250 occurrences across sessions). Cannot be
   allowlisted (arbitrary code execution).
2. **Progress-entry appends** (`cat >> .agent/work-plans/issue-<N>/progress.md
   <<'EOF'`) — a write, so it prompts (~120 occurrences).
3. **The `progress:` commit** (`git -c … commit`) — a write, so it prompts.

The `dlog.sh` precedent (#515/#516) shows the pattern: a narrow, committed
script does the write, allowlisted once, prompt-free thereafter. This issue
applies that pattern to progress appends.

The issue review (#594) flagged two consequences not in the issue body:
- New script requires an entry in AGENTS.md's script reference table (ADR-0006).
- A smoke test reduces regression risk on a script that commits to git.

## Approach

1. **Write `.agent/scripts/progress_append.sh`** — reads entry text from stdin,
   appends to `.agent/work-plans/issue-<N>/progress.md` (creates file +
   frontmatter if absent, per review-code step-8 pattern), `git add`s only that
   file, and commits with agent identity (`-c` overrides) and a message of the
   form `progress: <entry-type> for #<N>`. The entry-type slot is extracted from
   the first `## Heading` line in the stdin text. Scope discipline: no arbitrary
   paths, no arbitrary commit messages beyond the entry-type slot, nothing else
   staged.

2. **Add allowlist entry to `.claude/settings.json`** — `Bash(.agent/scripts/progress_append.sh:*)`,
   consistent with existing entries (`dispatch_subagent.sh`, `dlog.sh`,
   `dashboard.sh`, etc.). This is what makes every future progress append
   prompt-free.

3. **Add AGENTS.md script reference table entry** — one row for
   `progress_append.sh` with its purpose, following the existing table format.
   Required by ADR-0006 and flagged by the issue review.

4. **Add a smoke test** — `scripts/test_progress_append.sh` exercises the script
   against a temp git repo fixture: verify it creates progress.md with correct
   frontmatter when absent, appends correctly when present, extracts the
   entry-type and forms the commit message correctly, and fails loud on bad input
   (no issue dir parent, empty stdin). Run from `make test-scripts`.

5. **Update skill guidance** — replace `cat >>` / `python3 -` heredoc patterns
   with the prompt-free equivalents in:
   - `.claude/skills/review-code/SKILL.md` step 8 — write progress entry via
     `progress_append.sh`, not `cat >>`/`git commit` inline.
   - `.claude/skills/triage-reviews/SKILL.md` — same pattern for the
     `## Integrated Review` append.
   - `.claude/skills/run-issue/SKILL.md` — guidance to use `jq` (auto-allowed)
     and `progress_read.py` (allowlisted) for parsing, and `progress_append.sh`
     for progress writes — never ad-hoc heredocs.

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/progress_append.sh` | New script — stdin→progress.md append + git add + commit |
| `.claude/settings.json` | Add `Bash(.agent/scripts/progress_append.sh:*)` allowlist entry |
| `AGENTS.md` | Add `progress_append.sh` row to script reference table |
| `.agent/scripts/test_progress_append.sh` | New smoke test against temp git repo fixture |
| `.claude/skills/review-code/SKILL.md` | Step 8: replace inline `cat >>` + `git commit` with `progress_append.sh` |
| `.claude/skills/triage-reviews/SKILL.md` | Replace inline progress append with `progress_append.sh` |
| `.claude/skills/run-issue/SKILL.md` | Guidance: `jq`/`progress_read.py` for parsing; `progress_append.sh` for writes |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Push, PR creation, and merge are explicitly out of scope and stay human-gated. The allowlist entry makes the new automation visible and bounded. |
| Enforcement over documentation | Script + allowlist entry = mechanical enforcement, not just docs. Guidance in skills is the instruction layer on top. |
| A change includes its consequences | AGENTS.md table entry and smoke test are included. |
| Only what's needed | Script is minimal by design; accepts only issue number + stdin; commits only one file with a fixed message pattern. |
| Improve incrementally | Small, scoped change following an established precedent (dlog.sh). |
| Test what breaks | Smoke test included; exercises create, append, entry-type extraction, and failure modes. |
| Never document from assumptions | Skill guidance updates are based on current skill text, not assumed content. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0006 — Shared AGENTS.md | Yes | Script reference table entry added in same PR. |
| ADR-0013 — progress.md vocabulary | Yes | Script extracts entry type from the `## Heading` line in stdin text; commit message is `progress: <entry-type> for #<N>` — the canonical form. Creates frontmatter on first write consistent with the vocabulary spec. |
| ADR-0004 — Enforcement hierarchy | Partial | Instruction-layer updates to skills + mechanical script+allowlist entry. No pre-commit hook or CI check for regression to `cat >>` patterns — acceptable given the behavioral nature of the change and precedent in dlog.sh. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Add new script to `.agent/scripts/` | AGENTS.md script reference table | Yes — step 3 |
| Add allowlist entry | Verify format matches existing entries | Yes — step 2 |
| Change skill guidance on progress writes | Skill instruction files for run-issue, triage-reviews, review-code | Yes — step 5 |
| Add smoke test | Integrate into `make test-scripts` Makefile target | Yes — step 4 (test-scripts already exists per AGENTS.md) |

## Open Questions

- [ ] No open questions — plan is review-plan-ready.

## Estimated Scope

Single PR — all changes are workspace infrastructure, tightly coupled, and small enough to review together.
