# Plan: dispatch_subagent.sh + per-skill handoff boilerplate (#490)

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/490 — part of #481
(phase C of #470). Planned jointly with #493 (delete push_gateway) per
request; see Open Questions for why #493 is sequenced *after* #492, not here.

## Context

The sandboxed agent container already runs tasks today — `docker_run_agent.sh
--issue <N>` builds the image, mounts the worktree, forwards identity, and runs
an **interactive** `claude --dangerously-skip-permissions --strict-mcp-config`
with zero permission prompts (hardened by #489). What's missing is a
*programmatic dispatch layer* so a host orchestrator (#492 `/run-issue`) can
launch any workflow skill into a fresh-context sub-agent — in-process (`Agent`
tool) or container (docker) — with a prepared kickoff prompt, identity literals,
and a defined exit contract. #490 is that keystone; #491/#492 consume it.

Two concrete gaps the dispatch must close:

1. **`docker_run_agent.sh` is interactive-only** (`-it`, interactive claude).
   Container dispatch needs a **headless kickoff** path. Confirmed the image's
   `claude` supports `-p/--print` with `--output-format`.
2. **The exit contract.** Today the container signals work via `push_gateway.sh`
   + `.agent/scratchpad/push-requests/*.json`. #481's model replaces that: the
   sub-agent writes a final `progress.md` entry (ADR-0013 vocabulary) and the
   host reads the last entry for outcome + next action. #490 introduces this
   contract **for the dispatch path only** — it does *not* delete the gateway
   (that's #493, which depends on #492 — see Open Questions).

## Approach

1. **Headless kickoff in `docker_run_agent.sh`** — add `--prompt <text>` /
   `--prompt-file <path>`. When given, launch non-interactively (drop `-it`):
   `claude -p "<kickoff>" --output-format stream-json
   --dangerously-skip-permissions --strict-mcp-config`, tee to a per-run log,
   exit with claude's status. No prompt → existing interactive behavior
   unchanged. **Guard** the post-exit `push_gateway` block so it is skipped in
   dispatch mode (host reads `progress.md` instead) — guard, don't delete.
2. **`dispatch_subagent.sh` (NEW)** — `--mode in-process|container`,
   `--issue <N>`, and `--skill <name>` or `--prompt-file`. Builds the handoff
   prompt: identity **literals** via `git -c` (AGENTS.md § Agent Commit
   Identity), the "read the last `## <EntryType>` entry in
   `progress.md`" vessel instruction, and a mode hint.
   - **in-process**: emit the prompt to stdout for the host to paste into a
     fresh `Agent` call (same context root; fast; no isolation).
   - **container**: emit the prompt + launch `docker_run_agent.sh` headless
     with it as kickoff; on exit, read the worktree-issue's last `progress.md`
     entry and print outcome + next action.
3. **Exit-contract read** — reuse `.agent/scripts/progress_read.py` to extract
   the last typed entry. Document that the contract is **convention-only, no
   enforcement** (per #490 / ADR-0004 honesty about enforcement tiers).
4. **Per-skill handoff boilerplate (Scope B)** — add a "Next step" block to the
   final step of `review-issue`, `plan-task`, `review-plan`, `review-code`,
   `triage-reviews` (+ address-findings when #491 lands): identity literals,
   the `progress.md` handoff vessel, and a mode hint (in-process for short
   reviews; container for isolation-worthy phases). Document **Scope E**: no
   skill-level auto-chaining — the host orchestrator (#492) drives next-phase
   dispatch.
5. **Tests** — `test_dispatch_subagent.sh` (mode selection, prompt emission,
   exit-contract read), following the `test_check_commit_identity.sh` pattern.
   **Layer-worktree container check**: verify container mode works for a
   project-repo (layer) worktree — the 2026-05-20 experiment only exercised a
   *workspace* worktree, and #492 will rely on layer dispatch.

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/dispatch_subagent.sh` | NEW — dual-mode dispatch + handoff-prompt builder + exit-contract read |
| `.agent/scripts/docker_run_agent.sh` | Headless `--prompt`/`--prompt-file` kickoff; guard the push_gateway post-exit block off in dispatch mode |
| `.agent/scripts/test_dispatch_subagent.sh` | NEW — unit tests |
| `.claude/skills/review-issue/SKILL.md` | "Next step" handoff block |
| `.claude/skills/plan-task/SKILL.md` | "Next step" handoff block |
| `.claude/skills/review-plan/SKILL.md` | "Next step" handoff block |
| `.claude/skills/review-code/SKILL.md` | "Next step" handoff block |
| `.claude/skills/triage-reviews/SKILL.md` | "Next step" handoff block |
| `.agent/knowledge/skill_workflows.md` | Document handoff convention + Scope-E no-auto-chain |
| `AGENTS.md` | Script Reference row for `dispatch_subagent.sh` (**ask-first** — instruction file) |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Worktree isolation (ADR-0002) | Dispatch operates on existing per-issue worktrees; container mounts them — no branch switching |
| Project-agnostic infra (ADR-0003) | Dispatch + handoff are workspace-generic; no project-specific assumptions |
| Honest enforcement (ADR-0004/0005) | Exit contract is convention-only and documented as such — no false enforcement signal |
| Quality / robustness | Headless path must surface a non-zero claude exit; layer-worktree path tested, not assumed |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0013 (progress.md vocab) | Yes | Exit contract reads/writes existing typed entries; no new entry type added here (address-findings type is #491's call) |
| ADR-0002 (worktree isolation) | Yes | Dispatch uses existing worktrees, not branch checkouts |
| New orchestration ADR | Maybe | A dispatch/orchestration ADR may be warranted once #492 lands; flag — do not author in #490 |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Add `dispatch_subagent.sh` | AGENTS.md Script Reference table | Yes (ask-first) |
| Edit 6 skills' final step | Skill list / framework adapters (consequences map) | Yes — handoff is Claude-`Agent`-specific; note non-Claude limitation in adapters |
| `docker_run_agent.sh` exit flow | `push_gateway` lifecycle | Partial — guarded now; full removal is #493 (gated on #492) |

## Decisions (resolved with user, 2026-06-13)

- **#493 sequencing — leave the gateway intact.** #490 introduces the
  `progress.md` exit contract for the *dispatch path only*; the legacy
  interactive `docker_run_agent.sh --issue N` keeps `push_gateway.sh`
  unchanged. #492 provides the host-side replacement; #493 (gated on #492)
  deletes the gateway. No deprecation warning added in #490 — the gateway
  stays as-is until its replacement exists. Plan #493's deletions separately
  after #492.
- **Kickoff output format — `stream-json`** (paired with `--verbose`, which
  Claude Code requires for streaming `--print`). Gives #492 structured,
  parseable live progress + a final result object. The authoritative outcome
  still comes from the `progress.md` exit contract; `--output-format` remains a
  per-call override.
- **One PR, closes #490** — Scope A (dispatch + headless docker + tests) and B
  (6 skill handoff blocks + docs) land together. A is independently testable
  via `--prompt-file`, but bundling keeps the keystone cohesive in one review.

## Open Questions

- [ ] None remaining — all three resolved above; plan is review-plan-ready.

## Estimated Scope

Single PR for #490 (Scope A + B + E), closing the issue. #493 is a separate,
later PR gated on #492 — recommend a dedicated `plan-task 493` after #492 lands
rather than folding its deletions in here.
