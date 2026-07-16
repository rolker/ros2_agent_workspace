# Skill Workflows

## Per-Issue Lifecycle

The governance skills follow a sequence from idea exploration through
post-PR review triage:

```
brainstorm → review-issue → plan-task → review-plan → implement
          → review-code → push / open PR → triage-reviews
          → address-findings → review-code (re-review) → …
```

**`/run-issue` orchestrates this whole sequence** (#492): it dispatches each
phase as a fresh-context sub-agent, reads each phase's `progress.md` entry to
choose the next action, and pauses at `AskUserQuestion` checkpoints. It is the
*driver*, not a dispatched phase — running the lifecycle by hand (one
`/review-issue`, `/plan-task`, … at a time) is the manual equivalent. The
pipeline is **local-first**: `plan-task` no longer opens a PR by default (commit
to branch only; `--draft-pr` to publish early), and `/run-issue` creates the PR
**at the end**, after a clean local `review-code`, gated by a user checkpoint.

Most steps are optional — simple issues can skip straight to
`plan-task` or implementation. **`review-code` is the exception**:
AGENTS.md Post-Task Verification step 5 makes a pre-push `review-code`
pass an expected check before opening a PR (the framework adapters in
`.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`,
and `.agent/AGENT_ONBOARDING.md` mirror this for non-Claude runtimes;
the Claude Adversarial Specialist requires Claude Code's `Agent` tool,
and the opt-in Copilot Adversarial Specialist runs from any runtime
that has the `copilot` CLI available when invoked with `--copilot`).
Pre-push mode (no arguments) catches issues while still cheap to fix
locally; the specialists that actually run depend on the
auto-classified depth tier — Light runs Static Analysis + one Claude
Adversarial pass, while Standard and Deep add Governance, Plan Drift,
and a second disjoint-lens Claude Adversarial pass. Copilot Adversarial
is opt-in at every tier via `--copilot` (off by default to conserve the
Premium quota). Local Model Adversarial (a quota-free Ollama read via
`.agent/scripts/local_review.sh`, runtime-agnostic — needs only
bash/curl/jq and a local Ollama server) is default-on at every tier,
opted out via `--no-local`. `review-code` also
accepts a PR number / URL for post-PR review of someone else's work.
`triage-reviews` runs after a PR has accumulated review comments
(human + bot) and classifies each against the current code.

## Skill Index

### Lifecycle skills (per-issue sequence)

| Skill | Position | Purpose |
|-------|----------|---------|
| `run-issue` | **Driver** (orchestrates the whole sequence) | Host orchestrator: dispatches each phase, reads its `progress.md` entry, pauses at checkpoints; local-first PR-at-end. Not itself a dispatched phase (#492) |
| `brainstorm` | Before review-issue | Explore possibilities using research digests and project knowledge |
| `review-issue` | Before plan-task | Evaluate issue scope, principle alignment, and ADR applicability |
| `plan-task` | Before implementation | Generate a principles-aware work plan, commit to branch |
| `review-plan` | After plan-task, before implementation | Independent evaluation of a committed work plan (accepts PR / file path / `--issue <N>`) |
| `review-code` | Between implement and push / open PR (also post-PR for someone else's diff) | Lead reviewer orchestrating specialist sub-reviews (static analysis, governance, plan drift, adversarial). Pre-push mode is the default; depth tiers (Light / Standard / Deep) scale specialist count to change risk |
| `triage-reviews` | After PR review comments arrive | Classifies each comment (human + bot) against current code, persists triage to `progress.md` |
| `address-findings` | After triage-reviews, before a re-review | Works the open `- [ ]` actions from the latest `## Integrated Review`, commits each fix atomically, writes a `## Implementation` entry |

### Utility skills (on-demand / periodic)

| Skill | When to use | Purpose |
|-------|-------------|---------|
| `research` | Any time | Survey external sources, maintain living research digests |
| `gather-project-knowledge` | After repo changes | Scan repos and generate project knowledge summaries |
| `audit-workspace` | Periodically | Check workspace governance health: enforcement, drift, staleness |
| `audit-project` | Before or after repo work | Check a project repo against workspace and project conventions |
| `onboard-project` | When onboarding a repo | Interactive audit + fix: add CI, pre-commit, agent guide, GitHub settings |
| `skill-importer` | When importing external skills | Evaluate, adapt, and import skills from external sources |
| `inspiration-tracker` | Periodically | Track external projects for portable enhancements and interesting patterns |
| `document-package` | After audit-project flags doc gaps | Generate or update ROS 2 package README and API docs from source |
| `issue-triage` | Periodically | Cross-repo issue scanning, categorization, and stale issue detection |
| `test-engineering` | After audit-project flags test gaps | Test scaffolding, debugging, and coverage analysis for ROS 2 packages |

### Makefile skills

`make_*` skills are auto-generated wrappers around Makefile targets (e.g.,
`/make_build`, `/make_test`). Run `make generate-skills` after adding or
removing `.PHONY` targets to keep them current.

## Lifecycle Handoff Convention (#490)

Each lifecycle skill's final step includes a `### Next step` block that
describes how to hand off to the subsequent phase. The full lifecycle map:

| Skill | Entry type written | Next skill | Next entry type |
|-------|--------------------|------------|-----------------|
| `review-issue` | `## Issue Review` | `plan-task` | `## Plan Authored` |
| `plan-task` | `## Plan Authored` | `review-plan` | `## Plan Review` |
| `review-plan` | `## Plan Review` | implement (no skill yet) → `review-code` | `## Local Review` |
| `review-code` | `## Local Review` | `triage-reviews` | `## Integrated Review` |
| `triage-reviews` | `## Integrated Review` | `address-findings` (if open findings) / done | `## Implementation` |
| `address-findings` | `## Implementation` | `review-code` (re-review) | `## Local Review` |

### How to hand off

Use `dispatch_subagent.sh` to build a kickoff prompt that embeds the identity
contract and `progress.md` exit contract, then pass it to a fresh sub-agent:

```bash
# In-process (fast; no filesystem isolation):
.agent/scripts/dispatch_subagent.sh --mode in-process --issue <N> --skill <next-skill>
# → emits a handoff block; paste into a fresh Agent tool call

# Container (isolation; use for implementation-heavy phases):
.agent/scripts/dispatch_subagent.sh --mode container --issue <N> --prompt-file <task.md>
```

The sub-agent reads the last `## <prev-entry-type>` entry in
`.agent/work-plans/issue-<N>/progress.md` for context, and appends its own
typed entry when done. The host reads the last entry (filtered by expected
entry type) to determine the outcome and whether a new entry was written at all
(a missing new entry = the sub-agent died before reporting).

### Scope-E no-auto-chaining rule

**Skills never dispatch the next phase themselves.** The host orchestrator
(`/run-issue`, [#492](https://github.com/rolker/ros2_agent_workspace/issues/492))
drives the lifecycle, invoking each phase in sequence and pausing at user
checkpoints between them. A skill's `### Next step` block is a prompt for
the operator or orchestrator to act on — not an instruction for the skill to
execute autonomously. This keeps each step's entry independently attributed and
prevents a single runaway sub-agent from racing through the whole lifecycle.
