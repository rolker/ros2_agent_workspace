## Review

### Scope Assessment

**Well-scoped?** Yes — creates one new file (`.claude/skills/wrap-up-deployment/SKILL.md`), one AGENTS.md pointer update, and a `/review-code` clean pass. The seven-step workflow is already validated (4 manual end-to-end runs). Acceptance criteria is explicit and bounded to a single PR.

**Right repo?** Yes — workspace repo; deployment-mode lifecycle tooling belongs in workspace `.claude/skills/` per ADR-0014 and ADR-0003.

**Dependencies**: `/import-field-changes` already exists (`.claude/skills/import-field-changes/`). `/debrief-deployment` (#435) is a *downstream consumer* of this skill ("feeds"), not a prerequisite — this skill can ship independently. Related issues: #495 (umbrella), #496 (recovery checklist), #497 (import-field-changes).

---

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Skill documents each step; sterile-cockpit relaxation is intentional per ADR-0014 and well-framed. |
| Enforcement over documentation | Watch | Skills are documentation, not mechanically enforced. Consistent with `/start-deployment` precedent; ADR-0014 accepts this trade-off for behavioral modes. |
| Capture decisions, not just implementations | OK | ADR-0014 already records the wrap-up phase design; SKILL.md is the implementation. No new ADR needed unless the skill introduces design choices not covered by ADR-0014. |
| A change includes its consequences | Action needed | AGENTS.md pointer update is in scope (listed in acceptance criteria). Also: the skill list in non-Claude adapters (`.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md`) will also need updating per the consequences map. `make generate-skills` must be run to register `/wrap-up-deployment` as a slash command. |
| Only what's needed | OK | Codifying a 4-times-validated manual workflow; no speculative features. |
| Improve incrementally | OK | Adds phase [3] tooling without touching phases [0]–[2]. |
| Test what breaks | Watch | Workflow skills have no automated tests; the acceptance criteria includes `/review-code` clean as a quality gate. The 4-run manual validation is appropriate evidence for a SKILL.md-only change. |
| Workspace vs. project separation | Watch | SKILL.md must keep project-specific details (gitcloud reconciliation paths, bag data offload steps) in `.agents/deployment.yaml` config, not embedded in the skill. The ADR-0014 three-tier split is the key constraint for the implementation. |
| Workspace improvements cascade | OK | Skill is portable across deployment projects via `.agents/deployment.yaml`. |
| Primary framework first, portability where free | OK | Claude Code skill format; AGENTS.md update is framework-agnostic. |

---

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| 0001 — Adopt ADRs | No | ADR-0014 already records the wrap-up phase design. A new ADR is only needed if the implementation introduces a significant design choice not covered there. |
| 0002 — Worktree isolation | Yes | Workspace issue; worktree is already created. OK. |
| 0003 — Project-agnostic workspace | Yes | SKILL.md must not embed project-specific content (gitcloud paths, platform inventory, bag paths). All project-specific config must come from `.agents/deployment.yaml` at runtime. Verify the three-tier split during implementation. |
| 0006 — Shared AGENTS.md | Yes | AGENTS.md update is in scope. Also update the skill list in non-Claude adapters (`.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md`) — these list available workflow skills and will be stale without this update. |
| 0013 — `progress.md` vocabulary | Watch | If `/wrap-up-deployment` writes a `progress.md` entry to the deployment issue's timeline (likely, as it closes the deployment issue), clarify which ADR-0013 entry type it uses. Existing types: `## Issue Review`, `## Plan Authored`, `## Plan Review`, `## Local Review`, `## Local Review (Pre-Push)`, `## Integrated Review`, `## Implementation`. A "deployment wrap-up completion" could fit `## Implementation`. If none fit, a new type requires an ADR addendum (ADR-0012/0013 rules). This must be decided in the SKILL.md, not left implicit. |
| 0014 — Deployment mode | Yes (key) | Skill is explicitly phase [3] per ADR-0014's lifecycle. Ensure the urgency-contract language in SKILL.md ("relaxes in wrap-up") is consistent with ADR-0014's wording. ADR-0014 Status is currently "Proposed" — once the full lifecycle tooling ships, updating to "Accepted" is worth a note or a follow-up issue. |

---

### Consequences

- New skill added → non-Claude adapter skill lists must be updated (`.github/copilot-instructions.md` skill list, `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md` `## Workflow Skills` section).
- `make generate-skills` must be run after adding the new skill directory to register `/wrap-up-deployment` as a slash command.
- AGENTS.md "Deployment mode" section currently says "The wrap-up half (`/wrap-up-deployment`) and recovery checklist are tracked under umbrella #495" — this placeholder text should be updated to reference the skill's location and usage once it lands.
- ADR-0014 Status is "Proposed" — consider filing a follow-up issue to update it to "Accepted" once `/wrap-up-deployment` ships (this is the last major lifecycle piece).

---

### Recommendations

1. In the SKILL.md, explicitly document which fields are read from `.agents/deployment.yaml` and which steps are generic vs. project-driven — this is the primary ADR-0003 compliance risk and helps future adopters.
2. Early in the SKILL.md, specify which ADR-0013 `progress.md` entry type the skill writes (or note that it doesn't write one). If a new type is needed, flag it as a pre-implementation decision.
3. After the PR merges, update ADR-0014 Status from "Proposed" to "Accepted" (or open a follow-up issue to do so) — the lifecycle tooling is now substantially complete.

---
**Authored-By**: `Claude Code Agent`
**Model**: `Claude Opus 4.6`
