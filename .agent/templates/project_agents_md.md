# AGENTS.md — {REPO_NAME}

Instructions for AI agents working in this repository — including **GitHub
Copilot code review**, which reads this file when reviewing PRs. Coding
agents: the deep guide (packages, layout, pitfalls) is
[`.agents/README.md`](.agents/README.md); read it before making changes.

## Workspace Rules

This repo is developed inside a [ROS 2 Agent Workspace]({WORKSPACE_REPO_URL}).
The workspace root `AGENTS.md` carries the full shared rules (worktree
isolation, issue-first policy, commit conventions, AI signatures). This file
**references** those rules and adds repo-specific context only — it must
never restate or fork them.

## Quality Standard

This is software for autonomous robot boats operating on open water.
Robustness is not optional.

- Fix bugs completely: add the test, handle the edge case, check the
  lifecycle transition.
- Concerns about error handling, silent failures, stale data, or missing
  validation are not nits — flag them unless the failure mode genuinely
  cannot occur. "Config is under our control" and "pathological input" are
  not blanket dismissals; field configs change under pressure.
- A change includes its consequences: tests, documentation, and dependent
  references update in the same PR.

## Reviewing PRs

- If the PR carries a work plan (`.agent/work-plans/issue-<N>/plan.md` or a
  plan in the PR body), the plan is kept **in sync with the implementation
  as it evolves** — an implementation that matches the current plan text is
  not "plan drift", even if the plan changed after the PR opened.
- Verify claims against source: parameters, topics, services, and message
  types in docs must match the code.

## Review Context — {REPO_NAME}

<!-- Repo-specific context for reviewers: what this code controls, known
     risk areas, conventions that look wrong but are deliberate.
     Examples: "safety-critical: publishes cmd_vel on the live boat",
     "TF frames use the <ns>/ prefix convention", "hardware I/O in pkg X
     cannot be unit-tested; sim coverage lives in Y".
     Replace this comment in each repo. -->

## Instructions for Use

<!-- DELETE this section when instantiating the template in a repo. -->

1. Copy this file to the project repo root as `AGENTS.md`.
2. Replace `{REPO_NAME}` and `{WORKSPACE_REPO_URL}` (use `gh repo view
   --json url` on the workspace repo — never guess URLs).
3. Fill in **Review Context** with repo-specific guidance; delete the
   placeholder comment.
4. Keep it short (≤60 lines). Detail belongs in `.agents/README.md`; shared
   rules belong in the workspace `AGENTS.md`. Reference, never fork.
5. Keep the `## Quality Standard` heading verbatim — `audit-project` uses it
   as the currency marker.
