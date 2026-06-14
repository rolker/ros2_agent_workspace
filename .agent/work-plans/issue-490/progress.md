---
issue: 490
---

# Issue #490 — dispatch_subagent.sh + per-skill handoff boilerplate

## Plan Authored
**Status**: complete
**When**: 2026-06-13 17:14 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-490/plan.md` at `24896d3`
**PR**: https://github.com/rolker/ros2_agent_workspace/pull/519 (`[PLAN]` prefix)
**Phases**: single (Scope A + B + E in one PR; #493 deferred, gated on #492)

### Open questions
- [ ] #493 sequencing: #490 adds the progress.md exit contract but leaves push_gateway in place; deletion is #493, gated on #492 — confirm the split.
- [ ] Kickoff output format: stream-json vs json vs text for the host to consume (leaning stream-json).
- [ ] PR split: dispatch+headless (A) and skill handoff blocks (B) as one PR or two (leaning one).

## Plan Review
**Status**: complete
**When**: 2026-06-13 17:49 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context)) — fresh-context sub-agent dispatch (independent of plan-author context; same agent identity)

**Plan**: `.agent/work-plans/issue-490/plan.md` at `4290908` (findings addressed)
**PR**: https://github.com/rolker/ros2_agent_workspace/pull/519
**Verdict**: changes-requested (2 must-fix + 3 suggestions; all folded into plan at `4290908`)

### Findings
- [x] (must-fix) Exit-contract under-specified: crash-before-write reads stale entry; progress_read.py has no tail selector; could match wrong skill's entry → gate on exit status + require newer entry + key on expected type — plan Approach step 3
- [x] (must-fix) 3 framework adapters not in Files-to-Change (handoff is Claude-Agent-specific) — now listed
- [x] (suggestion) Container-mode must forward AGENT_NAME/EMAIL for entrypoint --detect identity — plan Approach step 2
- [x] (suggestion) Layer-worktree container check is manual, not CI — clarified in plan Approach step 5
- [x] (suggestion) State no Makefile target for dispatch_subagent.sh — added to Consequences

## Implementation
**Status**: complete
**When**: 2026-06-14 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet 4.6)

**Scope**: Scope B — per-skill handoff blocks and knowledge doc
**Commits**: `e1d0efb` (5 skill SKILL.md files), `703e7ff` (skill_workflows.md)

### What was added

Five lifecycle skill files each received a `### Next step` block as
the last section of their procedure (before `## Guidelines`):

- `review-issue/SKILL.md` → hands off to `plan-task` (in-process); entry pair: `## Issue Review` → `## Plan Authored`
- `plan-task/SKILL.md` → hands off to `review-plan` (in-process); entry pair: `## Plan Authored` → `## Plan Review`
- `review-plan/SKILL.md` → describes two-phase hand-off: implement (container, no skill yet) then `review-code` (in-process); entry pair: `## Plan Review` → `## Local Review`
- `review-code/SKILL.md` → hands off to `triage-reviews` (in-process, after push); entry pair: `## Local Review` → `## Integrated Review`
- `triage-reviews/SKILL.md` → end-of-loop description (address findings / merge); no dispatch line needed

`.agent/knowledge/skill_workflows.md` received a new
`## Lifecycle Handoff Convention (#490)` section: entry-type table,
dispatcher usage examples, and the Scope-E no-auto-chaining rule.

### Ask-first files intentionally omitted

`.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`,
`.agent/AGENT_ONBOARDING.md`, and `AGENTS.md` are held for human approval per
the plan's Files-to-Change "ask-first" annotation and the task prompt's Do-NOT-touch list.

### Actions
- [ ] Host: push branch and open/update PR for review
- [ ] Ask-first: add one-line "handoff is Claude-Agent-specific" note to the 3 framework adapters + AGENTS.md Script Reference row (human approval needed)

## Local Review
**Status**: complete
**When**: 2026-06-14 07:22 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))
**Verdict**: changes-requested

**PR**: #519 at `0139dda`
**Mode**: post-PR
**Depth**: Deep (reason: +1032/-39, 16 files, container-isolation/auth/identity scripts)
**Must-fix**: 4 | **Suggestions**: 6 | **Ask-first follow-ups**: 3

### Findings
- [ ] (must-fix) exit-contract fails OPEN: non-numeric `entry_count` makes `[ -le ]` error → success branch → false "OK" — `dispatch_subagent.sh` (PRE/POST_COUNT). Fix: validate numeric, fail closed.
- [ ] (must-fix) empty `--prompt-file` → `DISPATCH_MODE` stays false → launches interactive `-it` container — `docker_run_agent.sh`. Fix: reject empty file (`[ -s ]`).
- [ ] (must-fix) stale comment describes the now-deleted entrypoint `configure_git_identity.sh --detect` step — `docker_run_agent.sh` (forward-identity block). Fix: rewrite to the pure-`-c` + env-from-setup.bash reality.
- [ ] (must-fix) exit-contract prints `Result: OK` even when the entry's `**Status**` is `failed`/`partial` — `dispatch_subagent.sh` OK branch. Fix: read `entries[-1].status`, headline FAILED/PARTIAL, exit non-zero. (High value: #492 greps this.)
- [ ] (suggestion) env-fallback identity diverges from the `-c` literals: `setup.bash`→`set_git_identity_env.sh --detect` re-derives `claude-code` instead of honoring forwarded `AGENT_NAME/EMAIL` (only differs for a custom agent name; benign for human-misattribution). Fix: entrypoint export `GIT_AUTHOR_*/GIT_COMMITTER_*` from forwarded vars before sourcing setup.bash.
- [ ] (suggestion) OK-branch last-entry printer is unfiltered; show the `--type`-filtered last entry to match the gate — `dispatch_subagent.sh`.
- [ ] (suggestion) `--prompt`/`--prompt-file` "mutually exclusive" claimed in help but not enforced — `docker_run_agent.sh`. Enforce or soften.
- [ ] (suggestion) add "never write credentials into files/commits/output" to the handoff contract — `dispatch_subagent.sh` (defense-in-depth).
- [ ] (suggestion) warn when the layer-worktree glob matches >1 dir (silently picks first) — `dispatch_subagent.sh`/`docker_run_agent.sh`.
- [ ] (suggestion) validate `--output-format` against stream-json|json|text up front — `dispatch_subagent.sh`.
- [ ] (ask-first) add `docker_run_agent.sh` row to AGENTS.md Script Reference (sibling of the dispatch row; still missing).
- [ ] (ask-first) sync `configure_git_identity.sh` containerized-identity docs (`.agent/scripts/README.md`, `.agent/AI_IDENTITY_STRATEGY.md`) — now stale after the entrypoint step was removed.
- [ ] (follow-up) `framework_config.sh` model entry stale (`Claude Opus 4.6`) — pre-existing, cosmetic; separate from this PR.

## Implementation
**Status**: complete
**When**: 2026-06-14 08:10 -04:00
**By**: Claude Code Agent (Claude Opus 4.8) — fresh-context sub-agent dispatch

**Scope**: address the Local Review findings (4 must-fix + 6 in-scope
suggestions); ask-first files left untouched.

**Commits**:
- `e6770db` — `dispatch_subagent.sh` + `test_dispatch_subagent.sh`
- `a2d898d` — `docker_run_agent.sh`
- `cc26ead` — `agent-entrypoint.sh`

### Findings fixed
- **#1 (must-fix)** exit-contract fail-closed: `PRE_COUNT`/`POST_COUNT`
  coerced (`-1`/`0`) when non-numeric so `[ -le ]` can't error into the
  success branch — `dispatch_subagent.sh`.
- **#2 (must-fix)** empty `--prompt-file` rejected via `[ -s ]` —
  `docker_run_agent.sh`.
- **#3 (must-fix)** stale `configure_git_identity.sh --detect` comment
  rewritten to the `git -c` literals + entrypoint env-export reality —
  `docker_run_agent.sh`.
- **#4 (must-fix)** status-aware exit contract: reads the gated entry's
  `**Status**`, emits `Result: FAILED`/`PARTIAL` and exits non-zero for
  failed/partial; `complete` → OK, unknown/missing → OK-but-says-so —
  `dispatch_subagent.sh`.
- **#5 (suggestion)** entrypoint pre-exports `GIT_AUTHOR_*`/`GIT_COMMITTER_*`
  from forwarded `AGENT_NAME`/`AGENT_EMAIL` before sourcing `setup.bash`,
  so the env-fallback matches the handoff literals — `agent-entrypoint.sh`.
- **#6 (suggestion)** last-entry printer now `--type`-filtered to match the
  gate (folded into #4) — `dispatch_subagent.sh`.
- **#7 (suggestion)** `--prompt`/`--prompt-file` mutual exclusion enforced
  via a `PROMPT_SET` tracker — `docker_run_agent.sh`.
- **#8 (suggestion)** credential-handling line added to the handoff
  contract — `dispatch_subagent.sh`.
- **#9 (suggestion)** ambiguous worktree match warns on stderr (both
  scripts).
- **#10 (suggestion)** `--output-format` validated against
  `stream-json|json|text` up front in both scripts.

### Out of scope (left for human / follow-up)
The 3 ask-first follow-ups (AGENTS.md Script Reference row,
`configure_git_identity.sh` doc sync in `README.md`/`AI_IDENTITY_STRATEGY.md`)
and the cosmetic `framework_config.sh` model-entry follow-up were **not**
touched, per the Do-NOT-touch list.

### Tests
`bash -n` clean on all 4 edited scripts. `test_dispatch_subagent.sh`:
**29/29 pass** (added: `--output-format` validation, the no-credentials
handoff rule, and the ambiguous-worktree warning). Container-mode behavior
(fixes #1/#4/#6 runtime path) remains covered by the manual layer-worktree
verification noted in the plan — not unit-testable without docker.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-14 12:25 +00:00
**By**: Claude Code Agent (Claude Opus 4.8)
**Verdict**: changes-requested

**Branch**: feature/issue-490 at `65d3408`
**Mode**: pre-push
**Depth**: Deep (reason: security-relevant scripts — container dispatch, --dangerously-skip-permissions, OAuth-token handling, in-container git identity)
**Must-fix**: 1 | **Suggestions**: 5

Re-review of the current tree after the prior post-PR fixes (`e6770db`,
`a2d898d`, `cc26ead`). All earlier must-fix items verify as applied;
`test_dispatch_subagent.sh` 29/29; both adversarial lenses confirmed the
exit-contract + privilege-drop logic correct. Static analysis: `bash -n`
clean (shellcheck unavailable). Copilot: off.

### Findings
- [ ] (must-fix) `--prompt ""` sets `PROMPT_SET=true` but `DISPATCH_MODE` stays false (gated on `[ -n "$PROMPT" ]`) → silently launches interactive `-it` container; same class as prior must-fix #2 for `--prompt-file`. Gate on `PROMPT_SET` + reject empty `--prompt` — `docker_run_agent.sh:156`
- [ ] (suggestion) GH read-only token forwarded inline `-e "GH_TOKEN=$AGENT_GH_TOKEN"` → visible in host `ps`/cmdline; PR's new adjacent OAuth forwarding (line 369) uses the safe bare-`-e` form. Pre-existing line, 1-line fix while here — `docker_run_agent.sh:371`
- [ ] (suggestion) "died before reporting" headline is a false-negative when the sub-agent writes a real entry of a different type than the gated `--entry-type`; add a caveat to the message — `dispatch_subagent.sh:265`
- [ ] (suggestion) secrets read via `read -r VAR < file`; use `IFS= read -r` to avoid whitespace stripping — `docker_run_agent.sh:181,345`
- [ ] (suggestion/ask-first) AGENTS.md Script Reference adds `dispatch_subagent.sh` row but omits `docker_run_agent.sh` (extended into the dispatch backend) — `AGENTS.md:466`
- [ ] (suggestion/ask-first) `configure_git_identity.sh` still documented as "ideal for containerized agents" (README.md 216-255, AI_IDENTITY_STRATEGY.md) but the entrypoint no longer runs it (new model: `git -c` literals + env-fallback) — stale/contradictory
- [ ] (governance) Confirm human approval for the 4 ask-first instruction-file edits now in the diff (AGENTS.md, copilot-instructions.md, gemini-cli.instructions.md, AGENT_ONBOARDING.md) before pushing

## Implementation
**Status**: complete
**When**: 2026-06-14 12:59 +00:00
**By**: Claude Code Agent (Claude Opus 4.8) — fresh-context sub-agent dispatch

**Scope**: address the round-2 `Local Review (Pre-Push)` findings — 1 must-fix
+ 1 must-fix (exit-contract) + 2 in-scope suggestions + 2 human-approved
ask-first doc edits. Do-NOT-touch list honored (`framework_config.sh`,
`set_git_identity_env.sh`, and the 4 earlier framework-adapter/AGENTS handoff
edits left untouched).

**Branch**: feature/issue-490

**Commits**:
- `3d98375` — `docker_run_agent.sh` (#1, #3, #4)
- `da0b52b` — `dispatch_subagent.sh` (#2)
- `f702f4b` — `AGENTS.md` Script Reference row (#5)
- `05a553c` — `README.md` + `AI_IDENTITY_STRATEGY.md` doc sync (#6)

### Findings fixed
- **#1 (must-fix)** `--prompt ""` dispatch gate — `--prompt` arm now rejects
  an empty value at parse time, and `DISPATCH_MODE` is gated on `PROMPT_SET`
  (intent) instead of `[ -n "$PROMPT" ]` (value), so an empty prompt can no
  longer fall through to an interactive `-it` container —
  `docker_run_agent.sh`.
- **#2 (must-fix)** exit-contract missed review-code's pre-push variant.
  **Note on the task's `base_type` premise:** verified via `progress_read.py`
  JSON that `Local Review (Pre-Push)` is *itself* a canonical ADR-0013 type,
  so its `base_type` is the full `Local Review (Pre-Push)` — it does **not**
  strip to `Local Review`. The documented fallback was therefore required:
  `entry_count` and the OK-branch last-entry display now match over the
  unfiltered `progress_read.py` JSON with the predicate
  `type == ENTRY_TYPE OR base_type == ENTRY_TYPE OR type.startswith(ENTRY_TYPE)`
  (the `startswith` arm catches the parenthetical variant). `progress_read.py`
  left unchanged (changing `_canonical_base` would broadly re-scope a distinct
  canonical type and risk other consumers). Failure message softened to
  acknowledge the unexpected-type case — `dispatch_subagent.sh`.
- **#3 (suggestion)** `GH_TOKEN` now forwarded via bare `-e GH_TOKEN`
  (value-in-env, exported when set) instead of `-e "GH_TOKEN=$AGENT_GH_TOKEN"`
  (value-in-argv, visible in host `ps`) — matches the OAuth pattern —
  `docker_run_agent.sh`.
- **#4 (suggestion)** both secret-file reads use `IFS= read -r` to avoid
  whitespace stripping — `docker_run_agent.sh`.
- **#5 (ask-first, approved)** `docker_run_agent.sh` row added to the AGENTS.md
  Script Reference (verbatim per the task) — `AGENTS.md`.
- **#6 (ask-first, approved)** `configure_git_identity.sh` containerized-identity
  docs synced to the current model (handoff `git -c` literals + entrypoint
  `GIT_AUTHOR_*`/`GIT_COMMITTER_*` env); the script is reframed as the
  dedicated-checkout method, not the container method — `README.md`,
  `AI_IDENTITY_STRATEGY.md`. Claims cross-checked against
  `agent-entrypoint.sh` (lines 67–80).

### Tests
- `bash -n` clean on both edited scripts (`docker_run_agent.sh`,
  `dispatch_subagent.sh`).
- `test_dispatch_subagent.sh`: **29/29 pass**.
- shellcheck (via pre-commit) passed on both scripts.
- The #2 matching predicate runs only in the container-mode exit-contract path
  (not reachable without docker, consistent with the existing test file's
  documented scope), so no new unit test was added; it was verified manually
  with the live `progress.md` — `ENTRY_TYPE="Local Review"` now matches both
  `Local Review` and `Local Review (Pre-Push)` (count 1 → 2).

### Out of scope (untouched)
`framework_config.sh` stale-model follow-up, `set_git_identity_env.sh`, and the
4 already-committed framework-adapter/AGENTS handoff edits — per the
Do-NOT-touch list. No `git push` (host performs pushes).
