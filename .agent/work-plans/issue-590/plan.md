# Plan: Flip local review specialist 5f from default-on to opt-in

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/590

## Context

The Local Model Adversarial Specialist (5f) in `/review-code` was made default-on in
#570 (PR #571) on the theory that local Ollama inference is quota-free. In practice,
on 8GB-VRAM hardware a ~500-line diff can exceed 600 seconds (`LOCAL_REVIEW_TIMEOUT`
notes in `local_review.sh`), making the default-on pass the wall-clock long pole on
every review. The fix is to flip the default: specialist 5f becomes opt-in (`--local`)
rather than opt-out (`--no-local`). The `local_review.sh` script is unchanged — it
remains standalone-capable and is the offline/field-mode review path.

## Approach

1. **Update `.claude/skills/review-code/SKILL.md`** — flip all `--no-local`/default-on
   references to `--local`/default-off. Specific locations:
   - Usage lines (lines 11, 13): replace `[--no-local]` with `[--local]`
   - Flag docs (lines 45–55): rewrite flag entry to describe `--local` as opt-in;
     carry the hardware-speed rationale so the why survives issue closure
   - Overview specialist list (line 99–100): flip "default-on" / `--no-local`
   - Specialists overview bullet (lines 119–120): flip to "default-off" / `--local`
   - Step 1 argument-parsing block (lines 145–146): change `--no-local` token and
     `NO_LOCAL=1` description; also flip the "on by default" description to
     "off by default"; add `--no-local` as a deprecated no-op (symmetric with
     `--no-copilot`)
   - Usage examples (line 168): change `--no-local` example comment
   - Tier dispatch lists (lines 344–345, 357–358): change `unless NO_LOCAL=1
     (--no-local)` to `only if LOCAL=1 (--local)`
   - Deep tier paragraph (line 366–368): flip default-on language
   - Section 5f header + opening paragraph (lines 724–726): flip to default-off
   - Section 5f closing rationale paragraph (line 799, "default-on despite the
     noise"): flip to opt-in wording, keeping the keep-it-exercised-for-field
     rationale (review-plan must-fix — bare `default-on`, no flag token)
   - Report templates: replace `off (--no-local)` with `off (default)` in the
     standard/deep template (line 875, 933); in the Light template (line 959) flip
     the comment about omitting when opted-out and the "default-on" note (line 978)

2. **Update `.agent/knowledge/review_depth_classification.md`** — 5 references
   (review-plan must-fix: the Deep-tier bullet was missing from the original
   enumeration):
   - Lines 73–74: flip Light tier specialist bullet
   - Lines 80: flip Light "unless `--no-local`" report-format note
   - Lines 98–99: flip Standard tier specialist bullet
   - Line 114: flip Deep tier "Local Model Adversarial default-on" bullet
   - Lines 132–134: flip the note's default-on/`--no-local` description

3. **Update `.agent/knowledge/skill_workflows.md`** — 2 references (lines 40–41):
   flip "default-on at every tier, opted out via `--no-local`" to "off by default,
   opt-in via `--local`"

4. **Update `.agent/knowledge/inspiration_agent_workspace_digest.md`** — 1 reference
   (line 82): the Copilot section mentions "Originally default-on … opt-out via
   `--no-copilot`" as a historical note. The parallel Local Adversarial history
   (also originally default-on) lives in this file implicitly; add a note parallel
   to the Copilot entry documenting the flip for #590.

5. **Grep for any remaining stale references in `.claude/` and `.agent/`** to
   catch stragglers not covered above — match `no-local`, `NO_LOCAL`, and the
   bare wordings `default-on` / `default on` (review-plan suggestion: the
   flag-only grep would miss prose references like the 5f rationale paragraph).

## Files to Change

| File | Change |
|------|--------|
| `.claude/skills/review-code/SKILL.md` | Flip ~12 occurrences from `--no-local`/default-on to `--local`/default-off; add `--no-local` deprecated no-op; carry hardware-speed rationale in flag description |
| `.agent/knowledge/review_depth_classification.md` | 5 occurrences flipped |
| `.agent/knowledge/skill_workflows.md` | 2 occurrences flipped |
| `.agent/knowledge/inspiration_agent_workspace_digest.md` | 1 historical note updated |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Opt-in is more explicit — the local pass must be actively chosen; its wall-clock cost is no longer invisible to users who never read the flag docs |
| Capture decisions, not just implementations | Hardware-speed rationale (#590 / 8GB-VRAM constraint) must appear in the `--local` flag description so the why survives issue closure; #585 (container Ollama) may revisit the default |
| A change includes its consequences | All four knowledge-file references updated in the same PR — no stale docs |
| Only what's needed | Minimal targeted change; `local_review.sh` untouched |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0006 — Shared AGENTS.md | No | No changes to instruction adapter files; `--no-local` confirmed absent from adapter files in issue review |
| 0013 — progress.md vocabulary | Yes | progress.md entries follow the vocabulary throughout |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `--no-local` flag semantics in SKILL.md | All knowledge files that document the flag | Yes — steps 2–4 |
| Local Adversarial default | Report templates (show `off (default)` not `off (--no-local)`) | Yes — step 1 |
| `--no-local` retired | Add as deprecated no-op (symmetric with `--no-copilot`) | Yes — step 1 |

## Open Questions

- [ ] No open questions — plan is review-plan-ready.

## Estimated Scope

Single PR. ~15 line-level edits across 4 files; no logic changes.
