# Plan: Port the external cross-model reviewer (Gemini via agy + Codex) from agent_workspace into review-code

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/660

## Context

`review-code`'s adversarial coverage today is two in-house Claude lenses
(5d, always on) plus an opt-in Copilot pass (5e, `--copilot`) and an
opt-in local-Ollama pass (5f, `--local`). Copilot Premium quota is
exhausted for September 2026 (`reference_copilot_quota_exhausted_sept_2026.md`),
so 5e is effectively unavailable right now. `rolker/agent_workspace`
(the successor workspace, "the direction of travel") already has a
quota-free second-vendor read — Gemini via the `agy` CLI and Codex via
`codex exec` — dispatched in parallel by `cross_model_review.sh`. Issue
#658's PR was reviewed with that script, unmodified, from its own
worktree; both agents independently caught a real bug (`$PROJECT_HEALTH_BODY`
never assigned) that two in-house Claude rounds had missed.

This workspace never ported the Gemini/Codex arm — only the Copilot
slice was adopted (#467, digest "Partially adopted"). This plan ports
it, gated behind a new step **5g** so 5e/Copilot's working, quota-gated
contract is untouched.

**Owner decisions (issue #660, comment 2026-09-23 14:37 -04:00)** — this
plan implements these as given, not as open questions:

1. Activation: **Standard + Deep, default on** for `gemini,codex`, with
   `--no-cross-model` to opt out. Light stays static-only.
2. Slot: **new step 5g**; 5e (Copilot) is unchanged — its opt-in flag,
   binary probe, and untrusted-PR gate stay exactly as they are. 5g runs
   `--agents gemini,codex` only; Copilot is never folded into the 5g call.
3. ADR: **reference, no new ADR.** Cite `agent_workspace`'s ADR-0015
   (parallel-sync-only dispatch) from the script header and from 5g's
   SKILL.md text; this workspace's own `docs/decisions/0015-*.md` is a
   different, unrelated ADR (dispatch handoff context contract) and is
   left alone.
4. AGENTS.md Script Reference rows: **approved** (Ask-First item
   pre-cleared) — add rows for the five ported/adapted scripts.
5. Trust weighting: **full weight**, like Copilot (5e) — not 5f's
   low-trust discount. Both agents produced zero false positives on #658.

**Source re-verified at plan time**: `~/agent_workspace` `main` @
`48b0d82` ("Merge pull request #341 from rolker/feature/issue-320"),
same commit the issue review cited — no drift since.

## What the port table missed

Reading `cross_model_review.sh` in full (not just the issue's port
table) surfaced one dependency the issue didn't name:

- **`.agent/scripts/_plan_approach.py`** — the `## Plan Context`
  extractor (issue #320's feature: re-admits a plan's `## Approach`
  section into the review prompt as labelled context, capped at 200
  lines, via a real CommonMark parser rather than a line scanner). It
  needs `markdown-it-py`, which is **not** in this workspace's
  `requirements.txt` (verified: only `pre-commit` and `playwright` are
  listed). Without it, the script still runs — the extractor is
  attempted from two Python interpreters and a missing library degrades
  to one `WARNING:` line and an omitted `## Plan Context` section, never
  a hard failure — but the feature it exists for (flagging diff/plan
  divergence, which is exactly what Plan Drift's 5c specialist and this
  workspace's plan-first workflow care about) would silently never fire.
  **This plan ports `_plan_approach.py` and adds `markdown-it-py>=3.0,<5`
  to `requirements.txt`** so the feature actually works here, matching
  the source's current behavior rather than a degraded copy of it.

## Approach

1. **Port the five scripts into `.agent/scripts/`**, with per-file
   verbatim/adapted treatment below (see "Port table").
2. **Port `_plan_approach.py`** into `.agent/scripts/` (verbatim — it is
   pure Python, no workspace-layout coupling) and add
   `markdown-it-py>=3.0,<5` to `requirements.txt` so `make lint`'s
   `.venv` setup installs it.
3. **Port the test suite** — `tests/test_cross_model_review.sh` — into
   `.agent/scripts/tests/`. It self-discovers via
   `run_script_tests.sh`'s `test_*.sh` glob; no Makefile change needed.
   Its markdown-it-py-dependent assertions self-skip when the library
   isn't importable, and self-fail loudly when `$CI` is set (matching
   this workspace's `NONINTERACTIVE=1`/`CI=1` convention) — after step 2,
   `make lint` makes them run for real rather than skip.
4. **Add the `.gitignore` rows** for the regenerated, non-committed
   prompt/findings files.
5. **Add step 5g to `review-code`'s `SKILL.md`**: new specialist section
   mirroring 5e/5f's structure (activation, invocation, report
   contribution, trust weighting) but scoped to `gemini,codex` only, with
   `--no-cross-model` as its opt-out flag. Update the Usage/Overview
   flag list, the Standard/Deep tier dispatch lists, and remove the
   stale "remains unadopted" callout after 5d that currently points at
   `inspiration_agent_workspace_digest.md`.
6. **Add `AGENTS.md` Script Reference rows** for the five scripts.
7. **Update `inspiration_agent_workspace_digest.md`**'s "Partially
   adopted" section: move the Gemini/Codex line out of "What remains
   unadopted" into the adopted description, dated and issue-linked.
8. **Live verification** (see "Live verification" below) — run the
   ported `cross_model_review.sh` against a real diff on this host
   before calling the port done.

## Port table

| Source file | Treatment | Why |
|---|---|---|
| `.agent/scripts/cross_model_review.sh` | **Adapted** (small, targeted edits over an otherwise-verbatim port) | Two changes: (a) header comment cites agent_workspace's ADR-0015 by name+repo instead of a bare `ADR-0015` reference, since this workspace's own `0015-*.md` is unrelated (owner decision 3). (b) the ~15-line "NOT user-tier promoted (#317)" comment block (lines ~474–489) references `.agent/user_tier_scripts.txt` and an ADR-0016 "user-tier promotion" rule — **neither exists in this workspace**: there is no `user_tier_scripts.txt` file and this workspace's own ADR-0016 is "Runtime Layer Chaining," an unrelated topic. The comment is dropped rather than ported — it documents a promotion mechanism this workspace doesn't have, and porting it verbatim would misdocument the script under a maintained-looking ADR citation that doesn't back it. No behavior change; the script was never going to be user-tier-promoted here either way, for the same underlying reason (it takes `--repo`/`--work-dir` to target arbitrary checkouts, which no cwd guard can admit). |
| `.agent/scripts/_agy_review.sh` | **Verbatim** | Self-contained; the only workspace-specific assumption is `jq`'s availability, already a soft dependency elsewhere in this workspace. Its `--print-timeout`, stream-json contract, and result validation are agy-version facts, not workspace facts. Comment "(see bootstrap.sh)" for the jq dependency is a documentation pointer — left as-is; `bootstrap.sh` exists here too, even though it doesn't currently mention jq by name. |
| `.agent/scripts/_cli_review.sh` | **Verbatim** | Same reasoning — codex/claude/copilot invocation contracts are CLI-version facts, not workspace-layout facts. Verified against codex-cli 0.155.1/claude 2.x/copilot 1.0.61 upstream; re-verify during live verification (below) against the versions actually installed here (agy 1.2.9, codex-cli 0.156.1 — both one patch/minor ahead of what the source's header claims to have verified). |
| `.agent/scripts/_resolve_work_plans_dir.sh` | **Net-new shared helper, adapted** | Per the Issue Review: this workspace has no equivalent today (`check_branch_updates.sh` resolves things inline). Rules 1 and 2 (explicit override; `$WORKTREE_ISSUE` match + `git rev-parse --show-toplevel`) work unmodified for both this workspace's `.workspace-worktrees/issue-<repo>-<N>` and `layers/worktrees/issue-<repo>-<N>` layouts, because they never inspect the directory name — they trust `$WORKTREE_ISSUE`, which `worktree_enter.sh` exports correctly in both cases. Rule 2b's basename glob (`issue-*-"$issue"`) also happens to match this workspace's `issue-<repo-slug>-<N>` worktree basenames as a coincidence of the glob shape — **but only for workspace-type worktrees**; a *project*-repo (layer) worktree's actual working directory is `layers/worktrees/issue-<repo>-<N>/<layer>_ws/src/<project_repo>`, a *separate* git worktree whose toplevel basename is the **project repo/package name**, not `issue-*-<N>`. Rule 2b's fallback therefore does not (and structurally cannot, without a repo-identity lookup this helper doesn't have) cover a layer-worktree session that reached the right directory without sourcing `worktree_enter.sh` — it falls through to Rule 3's abort, which is a safe, explicit failure (not a silent misroute), so this is accepted as a known, narrower fallback rather than patched. **Required edit**: Rule 3's abort remediation text names `worktree_enter.sh --issue <N> --type workspace` / `--type project` — **this workspace's `worktree_enter.sh` takes no `--type` flag at all** (`Usage: source worktree_enter.sh (--issue <number> \| --skill <name>) [--repo-slug <slug>]`). The remediation block is rewritten to the actual usage (`source .agent/scripts/worktree_enter.sh --issue <N> [--repo-slug <slug>]`) so a real failure doesn't hand the operator a command that doesn't parse. |
| `.agent/scripts/_resolve_default_branch.sh` | **Net-new shared helper, verbatim** | No adaptation needed: pure git-symbolic-ref resolution with a `main` fallback, no path assumptions. Confirmed net-new per Issue Review — do not fold `check_branch_updates.sh`'s inline resolution into this helper as part of this PR (scope creep the Issue Review flagged explicitly); that refactor is a candidate follow-up issue, not bundled here. |
| `.agent/scripts/_plan_approach.py` | **Verbatim (new to the port table — see above)** | Pure Python, no workspace-layout coupling; the CommonMark-based extraction logic is markdown-shape facts, not workspace facts. |
| `.agent/scripts/tests/test_cross_model_review.sh` | **Verbatim** | Fully self-contained: builds its own `mktemp -d` mock git repo and mock CLI binaries per test, and drives the script via `WORKTREE_ISSUE=<N>` env-var injection rather than real worktree paths — none of its 198 assertions depend on this workspace's directory layout. Picked up automatically by `run_script_tests.sh`'s `test_*.sh` glob; no Makefile edit. |
| `.gitignore` rows (`review-*-prompt.md`, `review-*-findings.md`) | **Verbatim** | Confirmed (Issue Review) this workspace currently has no rows in this space — net-new addition, no merge conflict. |
| `review-code` SKILL.md cross-model section | **Adapted, new 5g (not a copy of 5e)** | Source's 5e is a unified `--agents gemini,codex,copilot` call folding all three into one specialist; owner decision 2 keeps this workspace's 5e (Copilot) untouched and narrows 5g to `gemini,codex` only. So 5g is written fresh, borrowing 5e's *structure* (activation line, invocation snippet, reading-the-result contract, report contribution) but with Copilot-specific machinery (binary probe, untrusted-PR gate, `--allow-all-tools` security note) omitted — those don't apply to gemini/codex, which run with no tool access by design (`_agy_review.sh`'s "Do NOT run shell commands" tool-use footer; `_cli_review.sh`'s codex/claude/copilot arms also carry no `--allow-all-tools`-equivalent). |
| ADR-0015 (agent_workspace) | **Referenced, not ported** (owner decision 3) | `docs/decisions/0015-*.md` here is taken by an unrelated ADR. Script header and SKILL.md text cite `rolker/agent_workspace`'s ADR-0015 by name and link; no file is added under `docs/decisions/` in this workspace. |

## Files to Change

| File | Change |
|---|---|
| `.agent/scripts/cross_model_review.sh` | New file, ported + adapted (see port table) |
| `.agent/scripts/_agy_review.sh` | New file, verbatim |
| `.agent/scripts/_cli_review.sh` | New file, verbatim |
| `.agent/scripts/_resolve_work_plans_dir.sh` | New file, adapted (Rule 3 remediation text) |
| `.agent/scripts/_resolve_default_branch.sh` | New file, verbatim |
| `.agent/scripts/_plan_approach.py` | New file, verbatim |
| `.agent/scripts/tests/test_cross_model_review.sh` | New file, verbatim |
| `requirements.txt` | Add `markdown-it-py>=3.0,<5` |
| `.gitignore` | Add `.agent/work-plans/*/review-*-prompt.md`, `.agent/work-plans/*/review-*-findings.md` |
| `.claude/skills/review-code/SKILL.md` | Add step 5g section; update Usage flags, Overview specialist list, Standard/Deep dispatch lists, `--no-cross-model` flag; remove the stale "remains unadopted" callout under 5d |
| `AGENTS.md` | Add 5 Script Reference rows |
| `.agent/knowledge/inspiration_agent_workspace_digest.md` | Update "Partially adopted" section |

## SKILL.md 5g — content outline

- **Activation**: Standard + Deep, **default on**. `--no-cross-model`
  opts out for this invocation (both pre-push and post-PR modes).
  Light never dispatches it (matches 5d/5b/5c's Light exclusion).
- **Agent selection**: `gemini,codex` always — never `copilot` (owner
  decision 2 narrows the source's caller-framework-aware "all
  non-caller agents" logic to a fixed pair). If the calling agent
  *is* Gemini CLI or Codex CLI (framework detection via
  `$AGENT_FRAMEWORK` / `detect_cli_env.sh`, same mechanism 5e already
  documents), drop that one agent from the list rather than having it
  review itself — mirrors 5e's non-caller framework aliasing, narrowed
  to the two-agent set.
- **Invocation**: one
  `.agent/scripts/cross_model_review.sh --pr <N> --agents gemini,codex --repo owner/repo`
  (post-PR) or `--branch [<base>] --agents gemini,codex [--no-progress]`
  (pre-push) call, per ADR-0015 (agent_workspace) parallel-sync dispatch.
  No availability probe block is needed in the skill text the way 5e
  needs one for Copilot — `cross_model_review.sh` already resolves each
  agent's binary itself and marks an unavailable one as a per-agent
  failure without aborting the run.
- **No untrusted-PR gate**: unlike Copilot's `--allow-all-tools`, neither
  `_agy_review.sh` nor `_cli_review.sh`'s codex/claude/copilot arms grant
  tool access for the gemini/codex path — Gemini's headless mode denies
  shell commands outright and the prompt is the only input either CLI
  gets. No gate needed or ported.
- **Reading results**: `MODE=parallel-sync`, one
  `AGENT=`/`FINDINGS_FILE=`/`EXIT=` triplet per agent; a failed agent
  is noted in the report, doesn't fail the review, doesn't block the
  others (same contract 5e already documents for the source's unified
  call — reuse that prose, narrowed to two agents).
- **Trust weighting**: full weight (owner decision 5) — same standing
  as 5d/5e findings in the silence filter (step 6), not 5f's low-trust
  discount. A finding corroborated across gemini/codex/Claude/Copilot is
  flagged as cross-model confirmed, same mechanism already in place for
  5e overlap.
- **Report section**: `#### Gemini Adversarial` / `#### Codex
  Adversarial` sub-sections under "Cross-Model Reviews" (mirrors 5e's
  existing report contribution pattern), each carrying its findings or
  a skip/failure reason.

## Live verification

Per the task's requirement: after landing the port, run the ported
script against a real diff **on this host**, where `agy` (1.2.9) and
`codex` (codex-cli 0.156.1) are installed and authenticated:

```bash
cd .workspace-worktrees/issue-workspace-660   # this worktree, once the port is committed
.agent/scripts/cross_model_review.sh --branch --agents gemini,codex --no-progress
```

- `--branch --no-progress` avoids needing an open PR and avoids writing
  into `.agent/work-plans/issue-660/` (keeps generated review artifacts
  out of the plan directory).
- Codex must run at its default model — no `-m gpt-5` (rejected on a
  ChatGPT-auth account per the issue's Prerequisites note); the ported
  `_cli_review.sh` never passes `-m`, so this is satisfied by not
  adding one.
- Confirms: both CLIs resolve and run against this workspace's actual
  diff shape (not the source's own repo), `_plan_approach.py`'s
  markdown-it-py path works or degrades cleanly depending on whether
  `make lint` has run yet, and the two CLI versions installed here
  (one patch/minor ahead of what the source verified against) still
  satisfy the helpers' contracts.
- Record the outcome (pass/fail, findings summary, any version-specific
  surprises) in this plan's Implementation Notes during implementation.

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | 5g's opt-out (`--no-cross-model`) and per-agent EXIT= markers keep the same explicit, inspectable contract as 5e/5f. |
| Enforcement over documentation | Ported script carries its 198-assertion test suite, wired into `make test-scripts` via auto-discovery — not just documented behavior. |
| Capture decisions, not just implementations | ADR-0015 is cited, not silently dropped; the reason (0015 collision) is recorded in this plan and in the script header. |
| A change includes its consequences | AGENTS.md rows, digest update, `.gitignore` rows, and `requirements.txt` addition are all in this PR, not deferred. |
| Only what's needed | 5e (Copilot) and its safety gate are untouched; no repo-wide refactor of `check_branch_updates.sh` into the new helpers. |
| Improve incrementally | Slots into the existing 5-specialist pattern (5g) rather than restructuring step 5 or the tier-dispatch tables beyond adding one line each. |
| Test what breaks | Full ported test suite plus a live-CLI verification run before calling the port done. |
| Workspace vs. project separation | Pure workspace-repo infra; no project-repo coupling. |
| Workspace improvements cascade to projects | N/A directly, but this is itself a reverse-direction cascade (agent_workspace → this workspace) per recent precedent (ws#654, ws#652). |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0001 — Adopt ADRs | Yes | Owner decision: reference agent_workspace's ADR-0015, no new local ADR (0015 is taken locally by an unrelated decision). |
| 0003 — Project-agnostic workspace | Yes | Satisfied — ported scripts are generic dispatch tooling, no project coupling. |
| 0006 — Shared AGENTS.md | Yes | New Script Reference rows added (approved, Ask-First pre-cleared). |
| 0009 — Python package management policy | Yes | `markdown-it-py` added to `requirements.txt`, installed into `.venv` by `make lint` — not a bare `pip install`. |
| 0013 — progress.md entry-type vocabulary | No | No new progress.md entry type; 5g's findings land in the existing `## Local Review (Pre-Push)` / `## Integrated Review` entry types under a new report sub-section, same as 5e today. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `review-code` SKILL.md (new 5g) | Usage flag list, Overview specialist list, tier dispatch tables | Yes — step 5 of Approach |
| Script Reference table (AGENTS.md) | — | Yes — approved by owner |
| `inspiration_agent_workspace_digest.md` "Partially adopted" note | — | Yes — step 7 of Approach |
| `requirements.txt` (new dependency) | `.venv` gets it via `make lint`; no separate install step | Yes |
| `.gitignore` (new rows) | — | Yes |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): `.agent/knowledge/inspiration_agent_workspace_digest.md`'s
  "Partially adopted" section currently states the Gemini/Codex dispatch
  "remains unadopted" — this PR makes that false, so the section must be
  updated in the same PR. `.claude/skills/review-code/SKILL.md`'s own
  "remains unadopted... see inspiration_agent_workspace_digest.md"
  callout (after 5d) is likewise stale once 5g exists and is removed.
- **Agent-instruction candidates** (proposals only): None beyond what's
  already captured — the `_resolve_work_plans_dir.sh` Rule 2b gap for
  layer-worktree sessions entered without sourcing `worktree_enter.sh`
  (see port table) is a real but narrow edge case with a safe abort;
  worth a one-line mention in `.agent/WORKTREE_GUIDE.md` if it's ever
  hit in practice, but not proposed here since it hasn't caused an
  actual failure yet — the operator decides whether it's worth
  documenting pre-emptively.

## Open Questions

- None blocking. The owner's four checkpoint decisions resolved every
  design point the Issue Review flagged as open. The one item this plan
  surfaces on its own — the `_resolve_work_plans_dir.sh` Rule 2b gap on
  layer-worktree paths — is not a blocking question: it degrades to a
  safe, clear abort (Rule 3) rather than a silent misroute, so it does
  not need a decision before implementation, only the port-table note
  above.

## Estimated Scope

Single PR. Seven new files (six scripts/tests + `_plan_approach.py`),
four edited files (`requirements.txt`, `.gitignore`, `SKILL.md`,
`AGENTS.md`), one edited knowledge doc, plus the live-verification run
recorded in Implementation Notes.
