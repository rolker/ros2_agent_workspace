# Review Depth Classification

**Status: experimental.** Ported from `rolker/agent_workspace` in PR #453.
The signal-to-tier table below has not yet been validated against real PR
data in this workspace; thresholds and triggers are expected to churn.
Promotion to ADR is reserved for after we accumulate enough real-PR
evidence to commit to specific cut-points.

## Context

PR #448 surfaced a systemic failure mode: 13 review rounds with ~45
Copilot findings, many of which were consequences of prior fixes that the
existing specialists missed because their context carried forward across
rounds. Two design responses, both ported from the fork:

1. A **fresh-context Adversarial Specialist** that re-reads the diff cold.
2. **Depth tiers** that scale specialist count and review effort to the
   risk of the change, so small mechanical PRs aren't held up by a full
   review battery and large/governance-touching PRs reliably get the
   full set.

This doc covers (2). The Adversarial Specialist itself lives in
`review-code/SKILL.md` (sub-section 5d).

## Current thinking

### Risk Signals

`review-code` collects these from `gh pr view <N>` output:

| Signal | Source | How to measure |
|--------|--------|----------------|
| Lines changed | `additions + deletions` | Total lines added and removed |
| File count | `files` array length | Number of files in the diff |
| File types | File paths | Categorize each file (see below) |
| Override triggers | File paths | Check against override-trigger lists below |
| Tests included | File paths | Whether the diff includes files in `test/`, `tests/`, or named `test_*`, `*_test.*` — absence of tests for code changes is noted in the report but does not, on its own, change the tier |

#### File type categories

| Category | Examples |
|----------|---------|
| Code | `.py`, `.cpp`, `.hpp`, `.sh`, `.js`, `.ts` |
| Config | `.yaml`, `.yml`, `.json`, `.toml`, `.xml` |
| Documentation | `.md`, `.rst`, `.txt` |
| Enforcement | See override-trigger list below |
| Governance | See override-trigger list below |
| Test | Files in `test/`, `tests/`, or named `test_*`, `*_test.*` |

### Depth Tiers

#### Light

**Criteria**: All of the following:
- < 50 changed lines (additions + deletions)
- ≤ 3 files
- No override-trigger files
- No Deep promotion triggers

**Specialists dispatched**:
- Static Analysis only

**Report format**: Condensed — static analysis findings plus a one-line
governance note ("No governance concerns for a change of this scope").

#### Standard

**Criteria**: Any of the following (and no Deep triggers):
- 50–199 changed lines
- 4–9 files
- Any override-trigger file present

**Specialists dispatched**:
- Static Analysis
- Governance
- Plan Drift
- Adversarial (Claude, fresh context)

**Report format**: Full report with all sections.

#### Deep

**Criteria**: Any of the following:
- 200+ changed lines
- 10+ files
- Cross-layer changes (files in both workspace infrastructure and project
  code in the same PR)
- Any Deep promotion trigger

**Specialists dispatched**:
- Same as Standard. The Adversarial Specialist is already running at
  Standard; Deep adds extra emphasis in the prompt (longer file horizon,
  explicit security/concurrency/lifecycle checklist) but uses the same
  fresh-context dispatch mechanism.

**Report format**: Full report with all sections.

> **Note on cross-model adversarial**: The fork runs a Cross-Model
> Adversarial Specialist at Deep tier (`cross_model_review.sh` —
> Gemini/Codex/Copilot via tmux). That dispatch is not adopted here (see
> the "Not adopted" entry in `inspiration_agent_workspace_digest.md`).
> When we want a second model's read on a Deep PR, run that agent's
> review-code skill manually.

## Override-Trigger Files

These files have outsized impact relative to their size. Their presence in
a diff bumps the review to **at least Standard**, regardless of line/file
count. Lists are split by repo type because a layered workspace has both
workspace-level and project-level governance and enforcement files.

### Workspace-repo triggers

**Enforcement** (mechanical rules — bypass causes silent drift):

- `.github/workflows/*.yml` (CI)
- `.pre-commit-config.yaml`
- `.claude/settings.json`, `.claude/hooks/*`
- `.repos` (workspace repo manifest)
- `.agent/scripts/setup_layers.sh` (layer bootstrap)
- `Makefile` (build/test orchestration)
- `framework_config.sh` (agent identity defaults)

**Governance** (rules people read and follow):

- `AGENTS.md`, `CLAUDE.md`
- `.github/copilot-instructions.md`
- `.agent/instructions/*.md`
- `docs/PRINCIPLES.md`
- `docs/decisions/*.md` (ADRs)
- `.claude/skills/*/SKILL.md` (skill definitions)
- `.agent/knowledge/*.md` (knowledge docs, including this file)
- `.agent/templates/*.md` (issue / package / plan templates)

### Project-repo triggers

For PRs in `layers/main/<layer>_ws/src/<project_repo>/`:

- `.agents/README.md` (project orientation)
- `.agents/review-context.yaml` (compact relevance map)
- Project-level `PRINCIPLES.md`
- Project-level `docs/decisions/*.md`
- Project-level `.agent/work-plans/issue-*/plan.md` (rare — implementation
  diff usually doesn't include the plan, but a plan-only change is
  governance-adjacent)

## Deep Promotion Triggers

These signals always bump the review to **Deep**, regardless of other
signals:

- **Security-relevant changes**: authentication, authorization, permissions,
  secrets handling, credential management, token storage, command
  injection surface (any change to a script that takes user input and
  shells out)
- **Cross-layer changes**: files modified in both workspace infrastructure
  (`.agent/`, `.claude/`, `docs/`, root configs) and project code
  (`layers/main/*/src/`) in the same PR. Cross-layer is harder to review
  because consequence analysis spans repos.
- **ADR additions or substantive ADR rewrites**: a new file in
  `docs/decisions/` or a non-status-bump edit to an existing ADR. Title
  and "Last reviewed" tweaks don't trigger.

## Tier Promotion Logic

1. Start at Light.
2. Walk the signals — if any signal meets Standard criteria, promote to
   Standard.
3. Walk the signals — if any signal meets Deep criteria (including the
   Deep promotion triggers above), promote to Deep.
4. The highest tier triggered wins. There is no mechanism to downgrade.

## User Override

Override automatic classification by appending a depth keyword to the
`/review-code` invocation:

- `/review-code 42 light` — force Light review
- `/review-code 42 standard` — force Standard review
- `/review-code 42 deep` — force Deep review

User overrides take precedence over automatic classification. Useful for
forcing thorough review on a small change that the author finds risky, or
a quick review on a large but mechanical change (bulk renames, licence
header additions).

## Consequences

- **Quality bar follows risk, not size alone.** A 10-line edit to
  `AGENTS.md` triggers Standard; a 1500-line bulk-rename triggers Deep.
  Authors should expect adversarial findings on governance edits, not just
  on big diffs.
- **`review-code` requires a one-line classification block in its report
  header.** The tier and the primary signal that determined it are always
  shown so authors can sanity-check (and override on the next run).
- **The Adversarial Specialist activates at Standard** — meaning every
  governance-touching PR gets a fresh-context second pass even if it's
  small. This is the intended behaviour: governance edits have
  out-of-proportion blast radius and benefit from a re-read cold.
- **Trigger lists are workspace-specific.** Adding a new file pattern
  (e.g., a new `.claude/` directory, a new ADR-equivalent doc) means
  updating the trigger lists here so the depth classifier sees it.
- **This doc is experimental.** When the thresholds prove themselves on
  real PR data, promote the structural decision (depth tiers exist;
  signals fire as listed) into an ADR. The numeric thresholds can stay in
  this doc and tune as we learn.
