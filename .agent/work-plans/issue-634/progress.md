---
issue: 634
---

# Issue #634 — Conventional-path discovery of planning documents in janitor-sweep and audit-project (sub-issue 2 of #628)

## Issue Review
**Status**: complete
**When**: 2026-09-18 10:04 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)

**Issue**: #634
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: needs-more-detail

### Scope Assessment

**Well-scoped?** Mostly, but one bullet needs resolving before plan-task: see
Action 1 below. Bounded to workspace-repo skill logic (`janitor-sweep`,
`audit-project`), consistent with how sub-issue 1 (PR #638) was scoped and
merged as a single PR.

**Right repo?** Yes — workspace repo; the probed skills live here and the
change is generic ROS 2 workspace infra (ADR-0003), not project-specific.

**Dependencies**: Depends on the merged design draft
(`docs/design/planning_document_vocabulary.md`, PR #638) for the
expected-location table it must probe exactly. No other open sub-issue of
#628 blocks it. The draft's own "Promotion path to an ADR" section
(line ~612) names this issue's reader as one of three gates for promoting the
draft to ADR-0637 — so #637 (ADR promotion) is downstream of #634, not the
reverse.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Workspace vs. project separation | OK | Change is confined to workspace-repo skills; nothing project-specific proposed. |
| Only what's needed | Watch | The externally-hosted-document fallback (org GitHub Project board via `gh api orgs/<org>/projects`, `homepage` field, README link matching) is explicitly "discovery only... no requirement, no check, no finding" per the design draft. Bundling it into the same PR as the core path-probe adds a network/auth dependency (org Projects API scope, rate limits) for a feature that changes no outcome. Worth asking whether it should be a follow-up issue rather than in-scope here, since the issue's own "Out of scope" line ("Any check that treats absence... as a finding") already fences it off from ever mattering to pass/fail. |
| Test what breaks | Watch | The issue requires "an explicit test case" for graceful absence, but `janitor-sweep` and `audit-project` are markdown-procedure skills today (`.claude/skills/janitor-sweep/SKILL.md`, `.claude/skills/audit-project/SKILL.md`) with no backing script and no test harness in the repo. The conditional bullet ("If the probe is worth factoring into a shared script...") is doing load-bearing work here: without factoring the probe into an actual script, there is nothing to attach an automated test to. Recommend treating that factoring as effectively required, not optional, given the test-case requirement two bullets above it. |
| Capture decisions, not just implementations | OK | The design draft already captures the "no per-repo file, no schema" decision (2026-09-17) with rationale; this issue implements it rather than re-deciding it. |
| Enforcement over documentation | OK | Probing becomes code inside existing skills rather than a new markdown-only convention. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0003 — Project-agnostic workspace | Yes | Already explicitly cited in the issue's own "graceful absence" bullet; the change stays generic. |
| ADR-0013 — progress.md entry-type vocabulary | No (exception applies) | `janitor-sweep` and `audit-project` are periodic, non-issue-scoped skills — the review guide's consequences map carves them out of the "must persist a typed progress.md entry" row. No new entry type needed here. |
| ADR-0017 — Extend AGENTS.md to project repos | Watch | Not directly triggered, but the design draft's declared-file alternative was rejected partly citing ADR-0017's own Negative ("one more per-repo file") — worth a passing check during plan-task that the probe doesn't reintroduce a per-repo file through the back door (e.g. a cache/marker file). |

### Consequences

- If the probe is factored into a shared script (conditional bullet), `AGENTS.md`'s Script Reference table needs a new row — the issue already names this.
- If a `kind:` marker convention is actually introduced (see Action 1), the design draft's expected-location table and its "no per-repo declaration file and no schema" statement would need to be reconciled or amended, since a marker is a form of per-document schema the draft currently forecloses.
- `docs/design/planning_document_vocabulary.md` line ~612 names "the conventional-path reader has landed" as gate 1 of 3 for promoting the draft to an ADR (#637) — closing #634 should update that gate's status where #637 tracks it.

### Recommendations

- Split the externally-hosted-document discovery fallbacks (GitHub Project board API, `homepage` field, README link matching) into a follow-up issue, or clearly flag them as optional/stretch within this PR — they are explicitly non-blocking per the design draft and add an auth/network surface unrelated to the core, testable, path-probe requirement.
- Factor the probe into a shared script (not left as skill-markdown prose only) so the "explicit test case" for graceful absence has something to run against.

### Actions
- [ ] **Action needed**: the scope bullet "Ships the `kind:` marker convention and the `.agent/templates/` kind marker" references a mechanism that does not exist anywhere in the merged design draft it is supposed to implement "exactly." The draft (`docs/design/planning_document_vocabulary.md`) explicitly settled on path-only discovery and rejected a per-repo declaration file (Options considered, "Options considered and not taken"); no `kind:` marker syntax is defined in the draft, its templates (`.agent/templates/roadmap.md`, `adr_template.md`, `project_governance.md`), or anywhere else in the tracked tree (verified via `grep -rn "^kind:"`). This "kind:" marker appears to be a holdover from the *pre-draft* proposal in the parent issue (#628's original body, item 3: "a one-line `kind:` marker at the top of each document"), which the settled draft superseded. Before plan-task proceeds, this bullet needs either (a) removal from #634's scope as stale, or (b) an operator decision to add the marker convention to the design draft first (which would also require reconciling it with the "no schema" decision). Recommend surfacing this to the operator rather than plan-task silently inventing a marker syntax or silently dropping the bullet.
- [ ] Consider splitting the externally-hosted-document discovery fallbacks (GitHub org Project board API, `homepage` field, README link matching) into a follow-up issue — they are non-blocking discovery-only per the design draft and add auth/network surface to an otherwise self-contained path-probe change.
- [ ] Confirm during plan-task whether the probe needs factoring into a shared script (vs. skill-markdown prose only) to satisfy the issue's "explicit test case" requirement for graceful absence — the workspace currently has no test harness for markdown-only skill procedures.
