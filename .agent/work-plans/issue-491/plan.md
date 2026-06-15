# Plan: 'address-findings' skill (consume ## Integrated Review)

## Issue
https://github.com/rolker/ros2_agent_workspace/issues/491

## Context
Phase C of #481 needs a thin workflow skill that closes the review loop: after
`triage-reviews` writes a `## Integrated Review` entry (a fix plan of `- [ ]`
actions), `address-findings` works through those actions, commits the fixes
atomically, checks the boxes, and writes a closing entry. It's the phase the
`/run-issue` orchestrator (#492) dispatches between triage and re-review — the
reason we build C before D.

The integration points already exist:
- `progress_read.py --type "Integrated Review"` returns each entry's `findings[]`
  array (`{section, checked, source_hint, text}`) — the `- [ ]` actions, parsed.
  No markdown re-parsing needed.
- `## Integrated Review` entry type exists (#485 merged — the issue's stated
  blocker is cleared).

## Approach
1. **New skill** `.claude/skills/address-findings/SKILL.md` (thin), steps:
   1. Resolve the issue's `progress.md`; run `progress_read.py --type "Integrated Review"`; take the **last** entry's unchecked `findings[]` (`checked == false`). If none, report "nothing to address" and exit.
   2. For each finding: make the code change, run pre-commit, commit atomically (`git -c` identity per AGENTS.md), then check its box (`- [ ]`→`- [x]`) in the `## Integrated Review` entry — same-commit so the timeline tracks resolution.
   3. Skip/annotate findings that are dismissals (plain bullets) or already `- [x]`.
   4. Write one closing `## Implementation` entry (see ADR gate) summarizing what was addressed + what was deferred, with a `### Actions` checklist for anything left.
   5. Emit a "Next step" handoff to **review-code** (re-review the fixes) — no auto-chaining (Scope E).
2. **Adapter skill lists** — add `address-findings` to `.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md`.
3. **De-dup note (scope decision)** — the issue says this "replaces ad-hoc external-review handling in `triage-reviews`+`review-code`." Propose: this PR only *adds* the skill; the removal of the old handling is a **follow-up** (keeps the PR focused and the removal independently reviewable). Open question 2.

## Files to Change
| File | Change |
|------|--------|
| `.claude/skills/address-findings/SKILL.md` | **New** — the thin skill |
| `.github/copilot-instructions.md` | Add skill to list |
| `.agent/instructions/gemini-cli.instructions.md` | Add skill to list |
| `.agent/AGENT_ONBOARDING.md` | Add skill to list |

## Principles Self-Check
| Principle | Consideration |
|---|---|
| Only what's needed | Reuse `progress_read.py` + an existing entry type; no new parser, no new ADR (per recommendation) |
| Capture decisions | Entry-type choice recorded; ADR-0013 honored |
| A change includes its consequences | Skill-list adapter updates in the same PR |
| Improve incrementally | De-dup of old handling deferred to a focused follow-up |

## ADR Compliance
| ADR | Triggered | How addressed |
|---|---|---|
| 0013 — progress.md vocabulary | **Yes** | Skill **writes `## Implementation`** (recommended) — an existing canonical type, no superseding ADR. Minting `## Findings Addressed` is rejected unless #492 needs to distinguish it (it doesn't: orchestrator reads the *last* entry; "Implementation → re-review" is the correct transition either way). |
| 0004/0005 — enforcement layering | Yes (satisfied) | Convention-only skill, consistent with peers |
| 0006 — shared AGENTS.md | Yes | Skill listed in all framework adapters |

## Consequences
| If we change... | Also update... | In plan? |
|---|---|---|
| Add a workflow skill | Skill list in non-Claude adapters | Yes |
| Add a skill that writes progress.md | Use an ADR-0013 type | Yes (`## Implementation`) |

## Open Questions
- [ ] **Entry type (ADR-0013 gate)** — recommend reusing `## Implementation` (no new ADR). Confirm, or mint `## Findings Addressed` via a superseding ADR.
- [ ] **De-dup scope** — remove the ad-hoc external-review handling from `triage-reviews`/`review-code` in *this* PR, or a focused follow-up? Recommend follow-up.

## Estimated Scope
Single PR (skill + adapter lists). De-dup follow-up tracked separately.
