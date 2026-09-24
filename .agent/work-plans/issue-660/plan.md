# Plan: Port the external cross-model reviewer (Gemini via agy + Codex) from agent_workspace into review-code

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/660

## Context

`review-code`'s adversarial coverage today is two in-house Claude lenses
(5d, always on) plus an opt-in Copilot pass (5e, `--copilot`) and an
opt-in local-Ollama pass (5f, `--local`). Copilot Premium quota is
exhausted for September 2026 (`reference_copilot_quota_exhausted_sept_2026.md`).
`rolker/agent_workspace` (the successor workspace, "the direction of
travel") already has a quota-free second-vendor read — Gemini via the
`agy` CLI and Codex via `codex exec` — dispatched in parallel by
`cross_model_review.sh`. Issue #658's PR was reviewed with that script,
unmodified, from its own worktree; both agents independently caught a
real bug (`$PROJECT_HEALTH_BODY` never assigned) that two in-house
Claude rounds had missed.

This workspace never ported the Gemini/Codex arm — only the Copilot
slice was adopted (#467, digest "Partially adopted"). This plan ports
it and **replaces** the Copilot Adversarial specialist (5e) with it —
see "Scope change" below.

**Owner decisions (issue #660, comment 2026-09-23 14:37 -04:00)**:

1. Activation: **Standard + Deep, default on** for `gemini,codex`, with
   `--no-cross-model` to opt out. Light stays static-only.
2. ~~Slot: new step 5g; 5e (Copilot) unchanged~~ — **superseded**, see
   Scope change below: the new step now takes the 5e slot and Copilot
   is removed outright.
3. ADR: **reference, no new ADR.** Cite `agent_workspace`'s ADR-0015
   (parallel-sync-only dispatch) from the script header and from 5e's
   (new) SKILL.md text; this workspace's own `docs/decisions/0015-*.md`
   is a different, unrelated ADR (dispatch handoff context contract)
   and is left alone.
4. AGENTS.md Script Reference rows: **approved** (Ask-First item
   pre-cleared) — add rows for the five ported/adapted scripts.
5. Trust weighting: **full weight** — same standing 5e (Copilot) carried,
   not 5f's low-trust discount. Both agents produced zero false
   positives on #658.

**Scope change (owner, issue #660 comment 2026-09-23 14:45 -04:00)** —
supersedes decision 2 above: **drop the Copilot CLI reviewer entirely.**
The Copilot CLI runs Claude or GPT models, so next to the in-house
Claude lenses and Codex it adds no new vendor, and it costs Premium
quota.

- Delete review-code step **5e (Copilot Adversarial Specialist)**, the
  `--copilot` and `--allow-untrusted-copilot` flags, and every reference
  to them (run-issue's publish-checkpoint text, AGENTS.md, knowledge
  docs, tests). Deleted outright — no deprecation shim, no `--no-copilot`
  no-op kept around for this flag pair (unlike the existing
  `--no-copilot`/`--no-local` no-ops, which predate this change and stay
  as they are for their own flags).
- The new Gemini+Codex step **takes the 5e slot** — it is `review-code`'s
  step 5e now, not a new 5g.
- **Untouched:** the GitHub-side `copilot-pull-request-reviewer` PR
  review that `run-issue` waits for and `triage-reviews` reads — that is
  a GitHub Actions/App integration, not a CLI invocation, and is out of
  scope for this port.
- The ported `cross_model_review.sh` keeps its `copilot`/`claude` agent
  arms verbatim (unused here, since 5e only ever calls
  `--agents gemini,codex`), so the script stays reconcilable with
  upstream at cutover.

**Plan checkpoint decisions (owner, issue #660 comment 2026-09-23
14:48 -04:00)**, folded into this revision:

1. **Pin Codex flags + gate untrusted PRs.** Invoke Codex with explicit
   `-s read-only -a never` rather than relying on codex-cli's current
   defaults (Plan Review r1 must-fix 1). On a post-PR review of a fork
   or non-collaborator PR, 5e skips unless `--allow-untrusted-cross-model`
   is passed — reusing the untrusted-PR detection the deleted Copilot
   step carried, renamed. File an upstream `agent_workspace` issue for
   the Codex flag pin, since `_cli_review.sh` is no longer verbatim
   (see "Upstream issue draft" below).
2. **CI change approved (Ask-First):** `.github/workflows/validate.yml`'s
   `script-tests` job installs `markdown-it-py` (Plan Review r1
   must-fix 2).
3. Revise the plan (this revision), then implement without a second
   plan review. Folds in: the two must-fixes above; an explicit
   Bash-tool-safe timeout under 600s in 5e's invocation (Plan Review r1
   should-fix); `make lint` before the live verification (Plan Review r1
   suggestion); the real source test count (**461**, verified via
   `grep -c assert_`, not the "~370" #313 comment estimate); and the
   scope change above.

**Source re-verified at plan time**: `~/agent_workspace` `main` @
`48b0d82` ("Merge pull request #341 from rolker/feature/issue-320"),
same commit the issue review and this revision both checked — no drift.

### Codex flag placement (verified on this host, codex-cli 0.156.1)

`codex exec --help` shows `-s/--sandbox` as an `exec`-subcommand flag
but **not** `-a/--ask-for-approval` — that flag exists only at the
top-level `codex` command, before the subcommand. Confirmed by testing
both orders on this host:

```
codex exec -a never --help   # ERROR: unexpected argument '-a'
codex -a never exec -s read-only --help   # OK
```

So the pinned invocation is `"$CLI_BIN_RESOLVED" -s read-only -a never exec -o "$CODEX_OUT_FILE"`
— both flags placed **before** `exec`, not after it as `-s read-only -a never`
appended to the existing `exec -o ...` call would parse. This is the
exact divergence from upstream `_cli_review.sh`'s bare `exec -o <file>`
call that must be drafted as an upstream issue (below).

### Upstream issue draft (for the host to file against `rolker/agent_workspace`)

**Title**: Pin codex's sandbox/approval flags in `_cli_review.sh`
instead of relying on CLI defaults

**Body**:
> `_cli_review.sh`'s codex arm invokes `codex exec -o <file>` with no
> `-s`/`-a` flags, relying on codex-cli's current defaults (`sandbox:
> read-only`, `approval: never` at the time of writing). That is an
> upstream CLI default, not a guarantee this script enforces — a future
> codex-cli release (or a host-level `~/.codex/config.toml` override)
> could silently grant write or full-access execution to a headless
> review turn reading an untrusted diff, with nothing in this script to
> catch it.
>
> Recommend pinning explicitly: `codex -s read-only -a never exec -o
> <file>` — note the flag placement: `-a/--ask-for-approval` is a
> **top-level** `codex` flag, not an `exec`-subcommand flag (confirmed
> via `codex exec --help` vs `codex --help` on codex-cli 0.156.1); it
> must go before `exec`, not after it alongside `-o`.
>
> Filed downstream while porting this script into
> `rolker/ros2_agent_workspace` (issue #660), where the same fix was
> applied locally.

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
5. **Delete Copilot Adversarial (old 5e) and replace it with the new
   cross-model step, in the same slot**: remove the Copilot specialist
   section, the `--copilot`/`--allow-untrusted-copilot` flags, the
   Copilot binary probe, and the Copilot untrusted-PR gate from
   `review-code`'s `SKILL.md`; write the new **5e — Cross-Model
   Adversarial (Gemini + Codex)** section in its place (structure below).
   Update the Usage/Overview flag list, the Standard/Deep tier dispatch
   lists, the report templates (5 places: full report header, Light
   condensed header, Light body comment block, no-findings header,
   progress.md persistence), and remove the stale "remains unadopted"
   callout after 5d that currently points at
   `inspiration_agent_workspace_digest.md`.
6. **Pin Codex's sandbox/approval flags** in the ported `_cli_review.sh`:
   change the codex arm's invocation from `"$CLI_BIN_RESOLVED" exec -o
   "$CODEX_OUT_FILE"` to `"$CLI_BIN_RESOLVED" -s read-only -a never exec
   -o "$CODEX_OUT_FILE"` (flag placement verified above — this makes the
   file no longer byte-identical to upstream, so it moves from
   "Verbatim" to "Adapted" in the port table).
7. **Remove every other `--copilot`/`--allow-untrusted-copilot`
   reference** outside `review-code/SKILL.md`: `run-issue/SKILL.md`
   (publish-checkpoint text), `AGENTS.md` (none currently — verify),
   `.agent/knowledge/skill_workflows.md`,
   `.agent/knowledge/review_depth_classification.md`,
   `.agent/knowledge/inspiration_agent_workspace_digest.md`,
   `.agent/AGENT_ONBOARDING.md`, and the flag-specific lines in
   `.github/copilot-instructions.md` /
   `.agent/instructions/gemini-cli.instructions.md` (instruction files —
   owner approved editing only the lines naming these flags, not a
   rewrite of the files). `pr_status.sh`'s Copilot references are the
   GitHub-side `gh pr review --copilot` reviewer — out of scope, left
   alone. `detect_cli_env.sh` / `test_detect_cli_env.sh`'s
   `COPILOT_API_URL`/`COPILOT_AGENT_CALLBACK_URL` env-var detection is
   framework identification (is the *calling* agent Copilot CLI?),
   unrelated to the deleted specialist — left alone.
8. **Add `AGENTS.md` Script Reference rows** for the five scripts.
9. **Update `inspiration_agent_workspace_digest.md`**'s "Partially
   adopted" section: move the Gemini/Codex line out of "What remains
   unadopted" into the adopted description, and update its Copilot
   description to reflect the specialist's removal — dated and
   issue-linked.
10. **`.github/workflows/validate.yml`**: add `markdown-it-py` to the
    `script-tests` job's pip-install step (owner-approved CI change).
11. **Live verification** (see "Live verification" below) — `make lint`
    first, then run the ported `cross_model_review.sh` against a real
    diff on this host before calling the port done.

## Port table

| Source file | Treatment | Why |
|---|---|---|
| `.agent/scripts/cross_model_review.sh` | **Adapted** (small, targeted edits over an otherwise-verbatim port) | Two changes: (a) header comment cites agent_workspace's ADR-0015 by name+repo instead of a bare `ADR-0015` reference, since this workspace's own `0015-*.md` is unrelated (owner decision 3). (b) the ~15-line "NOT user-tier promoted (#317)" comment block (lines ~474–489) references `.agent/user_tier_scripts.txt` and an ADR-0016 "user-tier promotion" rule — **neither exists in this workspace**: there is no `user_tier_scripts.txt` file and this workspace's own ADR-0016 is "Runtime Layer Chaining," an unrelated topic. The comment is dropped rather than ported — it documents a promotion mechanism this workspace doesn't have, and porting it verbatim would misdocument the script under a maintained-looking ADR citation that doesn't back it. No behavior change; the script was never going to be user-tier-promoted here either way, for the same underlying reason (it takes `--repo`/`--work-dir` to target arbitrary checkouts, which no cwd guard can admit). |
| `.agent/scripts/_agy_review.sh` | **Verbatim** | Self-contained; the only workspace-specific assumption is `jq`'s availability, already a soft dependency elsewhere in this workspace. Its `--print-timeout`, stream-json contract, and result validation are agy-version facts, not workspace facts. Comment "(see bootstrap.sh)" for the jq dependency is a documentation pointer — left as-is; `bootstrap.sh` exists here too, even though it doesn't currently mention jq by name. |
| `.agent/scripts/_cli_review.sh` | **Adapted** (one targeted edit) | Codex/claude/copilot invocation contracts are otherwise CLI-version facts, not workspace-layout facts — verified against codex-cli 0.155.1/claude 2.x/copilot 1.0.61 upstream, re-verified during implementation against codex-cli 0.156.1 and agy 1.2.9 actually installed here. **Edit**: the codex arm's `run_cli ... "$CLI_BIN_RESOLVED" exec -o "$CODEX_OUT_FILE"` becomes `run_cli ... "$CLI_BIN_RESOLVED" -s read-only -a never exec -o "$CODEX_OUT_FILE"` (Plan checkpoint decision 1) — pins the sandbox/approval policy explicitly rather than trusting codex-cli's current defaults, closing the residual-tool-access gap Plan Review r1 flagged. The `copilot`/`claude` arms are untouched (kept verbatim per the scope-change decision, for upstream reconcilability, even though 5e never calls them). |
| `.agent/scripts/_resolve_work_plans_dir.sh` | **Net-new shared helper, adapted** | Per the Issue Review: this workspace has no equivalent today (`check_branch_updates.sh` resolves things inline). Rules 1 and 2 (explicit override; `$WORKTREE_ISSUE` match + `git rev-parse --show-toplevel`) work unmodified for both this workspace's `.workspace-worktrees/issue-<repo>-<N>` and `layers/worktrees/issue-<repo>-<N>` layouts, because they never inspect the directory name — they trust `$WORKTREE_ISSUE`, which `worktree_enter.sh` exports correctly in both cases. Rule 2b's basename glob (`issue-*-"$issue"`) also happens to match this workspace's `issue-<repo-slug>-<N>` worktree basenames as a coincidence of the glob shape — **but only for workspace-type worktrees**; a *project*-repo (layer) worktree's actual working directory is `layers/worktrees/issue-<repo>-<N>/<layer>_ws/src/<project_repo>`, a *separate* git worktree whose toplevel basename is the **project repo/package name**, not `issue-*-<N>`. Rule 2b's fallback therefore does not (and structurally cannot, without a repo-identity lookup this helper doesn't have) cover a layer-worktree session that reached the right directory without sourcing `worktree_enter.sh` — it falls through to Rule 3's abort, which is a safe, explicit failure (not a silent misroute), so this is accepted as a known, narrower fallback rather than patched. **Required edit**: Rule 3's abort remediation text names `worktree_enter.sh --issue <N> --type workspace` / `--type project` — **this workspace's `worktree_enter.sh` takes no `--type` flag at all** (`Usage: source worktree_enter.sh (--issue <number> \| --skill <name>) [--repo-slug <slug>]`). The remediation block is rewritten to the actual usage (`source .agent/scripts/worktree_enter.sh --issue <N> [--repo-slug <slug>]`) so a real failure doesn't hand the operator a command that doesn't parse. |
| `.agent/scripts/_resolve_default_branch.sh` | **Net-new shared helper, verbatim** | No adaptation needed: pure git-symbolic-ref resolution with a `main` fallback, no path assumptions. Confirmed net-new per Issue Review — do not fold `check_branch_updates.sh`'s inline resolution into this helper as part of this PR (scope creep the Issue Review flagged explicitly); that refactor is a candidate follow-up issue, not bundled here. |
| `.agent/scripts/_plan_approach.py` | **Verbatim (new to the port table — see above)** | Pure Python, no workspace-layout coupling; the CommonMark-based extraction logic is markdown-shape facts, not workspace facts. |
| `.agent/scripts/tests/test_cross_model_review.sh` | **Verbatim** | Fully self-contained: builds its own `mktemp -d` mock git repo and mock CLI binaries per test, and drives the script via `WORKTREE_ISSUE=<N>` env-var injection rather than real worktree paths — none of its **461** assertions (verified `grep -c assert_`, not the #313 comment's "~370" estimate) depend on this workspace's directory layout. Its codex-mock assertions exercise the mock's own `-o`-flag handling regardless of what real flags precede `exec` on the caller side, so the `_cli_review.sh` flag-pin edit above does not require a test-suite edit. Picked up automatically by `run_script_tests.sh`'s `test_*.sh` glob; no Makefile edit. |
| `.gitignore` rows (`review-*-prompt.md`, `review-*-findings.md`) | **Verbatim** | Confirmed (Issue Review) this workspace currently has no rows in this space — net-new addition, no merge conflict. |
| `review-code` SKILL.md — Copilot Adversarial (old 5e) | **Deleted** (scope change) | The whole specialist section, its Usage-flag documentation, its availability probe, its untrusted-PR gate, and every report-template mention are removed. Replaced in the same slot by the new cross-model specialist below — see "SKILL.md 5e — content outline". |
| `review-code` SKILL.md cross-model section | **New, takes the 5e slot** (scope change; supersedes the original "new 5g" plan) | Source's own 5e is a unified `--agents gemini,codex,copilot` call folding all three into one specialist; this workspace's 5e never calls `copilot` (deleted) — `--agents gemini,codex` only. Written fresh, borrowing the deleted Copilot 5e's *structure* (activation line, invocation snippet, reading-the-result contract, report contribution, untrusted-PR gate — reused and renamed rather than dropped, per Plan checkpoint decision 1) but with Copilot-specific machinery (binary probe, `--allow-all-tools` security note) omitted — those don't apply to gemini/codex, which run with no tool access by design (`_agy_review.sh`'s "Do NOT run shell commands" tool-use footer; the pinned `-s read-only -a never` codex arm; `claude`'s `--permission-prompts none`). |
| ADR-0015 (agent_workspace) | **Referenced, not ported** (owner decision 3) | `docs/decisions/0015-*.md` here is taken by an unrelated ADR. Script header and SKILL.md text cite `rolker/agent_workspace`'s ADR-0015 by name and link; no file is added under `docs/decisions/` in this workspace. |

## Files to Change

| File | Change |
|---|---|
| `.agent/scripts/cross_model_review.sh` | New file, ported + adapted (see port table) |
| `.agent/scripts/_agy_review.sh` | New file, verbatim |
| `.agent/scripts/_cli_review.sh` | New file, adapted (codex flag pin) |
| `.agent/scripts/_resolve_work_plans_dir.sh` | New file, adapted (Rule 3 remediation text) |
| `.agent/scripts/_resolve_default_branch.sh` | New file, verbatim |
| `.agent/scripts/_plan_approach.py` | New file, verbatim |
| `.agent/scripts/tests/test_cross_model_review.sh` | New file, verbatim |
| `requirements.txt` | Add `markdown-it-py>=3.0,<5` |
| `.gitignore` | Add `.agent/work-plans/*/review-*-prompt.md`, `.agent/work-plans/*/review-*-findings.md` |
| `.github/workflows/validate.yml` | `script-tests` job installs `markdown-it-py` |
| `.claude/skills/review-code/SKILL.md` | Delete Copilot Adversarial (old 5e) entirely; add the new Cross-Model Adversarial section in the same 5e slot; update Usage flags, Overview specialist list, Standard/Deep dispatch lists, all report templates, `--no-cross-model` / `--allow-untrusted-cross-model` flags; remove the stale "remains unadopted" callout under 5d |
| `.claude/skills/run-issue/SKILL.md` | Remove `--copilot` publish-checkpoint text |
| `.agent/knowledge/skill_workflows.md` | Remove Copilot Adversarial / `--copilot` references |
| `.agent/knowledge/review_depth_classification.md` | Remove Copilot Adversarial / `--copilot` references; update the "Note on cross-model adversarial" callout |
| `.agent/AGENT_ONBOARDING.md` | Remove Copilot Adversarial / `--copilot` reference |
| `.github/copilot-instructions.md` | Remove only the lines naming `--copilot`/Copilot Adversarial (instruction file — owner approved this scope only) |
| `.agent/instructions/gemini-cli.instructions.md` | Remove only the lines naming `--copilot`/Copilot Adversarial (instruction file — owner approved this scope only) |
| `AGENTS.md` | Add 5 Script Reference rows |
| `.agent/knowledge/inspiration_agent_workspace_digest.md` | Update "Partially adopted" section (Gemini/Codex now adopted; Copilot specialist removed) |

## SKILL.md 5e (new) — content outline

- **Activation**: Standard + Deep, **default on**. `--no-cross-model`
  opts out for this invocation (both pre-push and post-PR modes).
  Light never dispatches it (matches 5d/5b/5c's Light exclusion).
- **Agent selection**: `gemini,codex` always — `copilot` is never an
  option (deleted). If the calling agent *is* Gemini CLI or Codex CLI
  (framework detection via `$AGENT_FRAMEWORK` / `detect_cli_env.sh`,
  the same mechanism the deleted Copilot 5e used for its own
  non-caller aliasing), drop that one agent from the list rather than
  having it review itself.
- **Invocation**: one
  `.agent/scripts/cross_model_review.sh --pr <N> --agents gemini,codex --repo owner/repo`
  (post-PR) or `--branch [<base>] --agents gemini,codex [--no-progress]`
  (pre-push) call, per ADR-0015 (agent_workspace) parallel-sync dispatch.
  `cross_model_review.sh` resolves each agent's binary itself and marks
  an unavailable one as a per-agent failure without aborting the run —
  no separate availability-probe block is needed in the skill text (the
  Copilot specialist needed one because it was a hand-rolled bash probe;
  this one is the ported script's own job).
  **Explicit Bash-tool-safe timeout** (Plan checkpoint decision 3): the
  invocation sets `AGENT_TIMEOUT` and `AGY_PRINT_TIMEOUT` below the Bash
  tool's 600s hard cap (`AGENT_TIMEOUT=480 AGY_PRINT_TIMEOUT=420s
  GEMINI_BACKSTOP_MARGIN=60` — see Implementation Notes for why the
  margin and the unit are both required)
  so the script's own internal per-agent timeout fires and emits its
  `EXIT=` marker *before* the harness kills the whole invocation —
  otherwise a wedged CLI produces a hard-killed call with no
  `EXIT=`/`FINDINGS_FILE=` output at all, a different failure shape than
  "a failed agent doesn't fail the review." Typical runs are 140–200s
  (per issue #660's evidence and this port's own live-verification run,
  recorded in Implementation Notes), so 480s leaves comfortable margin
  under the 600s cap while still well above typical run time.
- **Untrusted-PR gate, reused from the deleted Copilot step** (Plan
  checkpoint decision 1): post-PR mode only. Even though neither
  `_agy_review.sh` nor claude/gemini's arms of `_cli_review.sh` grant
  tool access, the pinned codex arm (`-s read-only -a never`) still has
  read access to the local filesystem — it can read and quote local
  files (env, other repos, credentials) into review text under a
  prompt-injected diff. Reuse the deleted Copilot 5e's fork/non-collaborator
  detection (`gh pr view --json authorAssociation,headRepository,baseRepository`),
  renamed: `ALLOW_UNTRUSTED_CROSS_MODEL` / `--allow-untrusted-cross-model`
  in place of `ALLOW_UNTRUSTED_COPILOT` / `--allow-untrusted-copilot`.
  Gated: skip with a one-line notice
  (`Cross-Model Adversarial skipped: external PR (head=<repo>, author=<assoc>); pass --allow-untrusted-cross-model after reviewing the diff to bypass`)
  unless the flag is passed.
- **Reading results**: `MODE=parallel-sync`, one
  `AGENT=`/`FINDINGS_FILE=`/`EXIT=` triplet per agent; a failed agent
  is noted in the report, doesn't fail the review, doesn't block the
  others.
- **Trust weighting**: full weight (owner decision 5) — same standing
  the deleted Copilot 5e carried, in the silence filter (step 6), not
  5f's low-trust discount. A finding corroborated across gemini/codex/
  Claude is flagged as cross-model confirmed.
- **Report section**: `#### Gemini Adversarial` / `#### Codex
  Adversarial` sub-sections under the 5e report contribution (replaces
  the deleted `#### Copilot Adversarial` single section — same
  placement, now two sub-sections since two agents run), each carrying
  its findings or a skip/failure reason.

## Live verification

Per the task's requirement: **run `make lint` first** (Plan checkpoint
decision 3 / Plan Review r1 suggestion — populates `.venv` with
`markdown-it-py` so the live run exercises the real `## Plan Context`
path rather than the degraded WARNING path), then run the ported script
against a real diff **on this host**, where `agy` (1.2.9) and `codex`
(codex-cli 0.156.1) are installed and authenticated:

```bash
make lint
cd .workspace-worktrees/issue-workspace-660   # this worktree, once the port is committed
AGENT_TIMEOUT=480 AGY_PRINT_TIMEOUT=420s GEMINI_BACKSTOP_MARGIN=60 \
  .agent/scripts/cross_model_review.sh --branch origin/main --agents gemini,codex --no-progress
```

- `--branch --no-progress` avoids needing an open PR and avoids writing
  into `.agent/work-plans/issue-660/` (keeps generated review artifacts
  out of the plan directory).
- The explicit `AGENT_TIMEOUT`/`AGY_PRINT_TIMEOUT` below 600s matches
  the Bash-tool-safe bound the SKILL.md invocation now documents (Plan
  checkpoint decision 3) — the live-verification run exercises the same
  bound the shipped skill text specifies, not the script's much larger
  upstream defaults.
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

## Implementation Notes

**Live verification (2026-09-24)**, codex-cli 0.156.1, agy 1.2.10, run as
`--branch --issue 660` (artifacts in the gitignored work-plans dir, not
`--no-progress`), in the background rather than under the 600s bound:

- **Run 1, port at upstream `48b0d82`**: codex completed in about a
  minute with three real findings (below). Gemini **failed**: agy tried
  a tool call, headless mode denied it, and the response was empty. The
  script reported that as a failed review (EXIT=1), not as a clean one.
- **Upstream had fixed that failure after the snapshot**
  (rolker/agent_workspace#336, "Gemini reviews fail on real branches").
  The port was re-synced to upstream `main` @ `97a87fa`: the
  `48b0d82..97a87fa` delta for `cross_model_review.sh`, `_agy_review.sh`,
  `_cli_review.sh` and the test suite applied cleanly over the local
  adaptations (codex pin, header, Rule 3 text), and the suite went from
  594 to 662 assertions, all passing. `_plan_approach.py` and the two
  resolvers had no upstream changes.
- **Run 2, gemini only, after the re-sync**: completed in about 5
  minutes on a ~350 KB prompt, with a full review.

**Defects the reviews found and this PR fixes:**

1. The skill's `AGY_PRINT_TIMEOUT=480` was rejected by the script (it is
   a Go duration and needs a unit), so every 5e run would have exited 2
   before any agent started. With the unit added, gemini's outer bound
   is still `AGY_PRINT_TIMEOUT + GEMINI_BACKSTOP_MARGIN` (default 300s),
   above the 600s cap, so the margin is set explicitly too.
2. The untrusted-PR gate asked `gh pr view --json` for
   `authorAssociation` and `baseRepository`, fields gh does not expose,
   so the gate errored on every PR. It now reads the REST pull object
   and fails closed when the lookup fails.
3. The helpers' escalation watchdog is forked while the terminate
   handler has INT/TERM/HUP ignored and inherited that, so cancelling it
   was a no-op: it outlived every clean shutdown by the escalation window
   and then `kill -9`ed whatever process held the dead CLI's PID. It now
   resets the dispositions first, and a new test assertion covers it.
   Upstream has the same bug.
4. 5e's pre-push call passes `--branch "origin/$BASE"`: without a base
   the script prefers the local default branch, which in this workspace
   moves only on `make sync`.

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | 5e's opt-out (`--no-cross-model`), untrusted-PR gate (`--allow-untrusted-cross-model`), and per-agent EXIT= markers keep the same explicit, inspectable contract the deleted Copilot 5e had. |
| Enforcement over documentation | Ported script carries its 461-assertion test suite, wired into `make test-scripts` via auto-discovery — not just documented behavior. |
| Capture decisions, not just implementations | ADR-0015 is cited, not silently dropped; the reason (0015 collision) is recorded in this plan and in the script header. The Copilot-removal rationale (vendor overlap, quota cost) is recorded in this plan's Scope change section, not just implied by the diff. |
| A change includes its consequences | AGENTS.md rows, digest update, `.gitignore` rows, `requirements.txt` addition, and the CI job's `markdown-it-py` install are all in this PR, not deferred. |
| Only what's needed | The Copilot deletion is scoped to the specialist and its flags — the GitHub-side `copilot-pull-request-reviewer` integration is untouched; no repo-wide refactor of `check_branch_updates.sh` into the new helpers. |
| Improve incrementally | Replaces one specialist (5e) in place rather than restructuring step 5 or the tier-dispatch tables beyond that one section and its report-template mentions. |
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
| 0013 — progress.md entry-type vocabulary | No | No new progress.md entry type; 5e's findings land in the existing `## Local Review (Pre-Push)` / `## Integrated Review` entry types under its report sub-section, replacing the deleted Copilot section's slot. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `review-code` SKILL.md (5e replaced) | Usage flag list, Overview specialist list, tier dispatch tables, all report templates, `run-issue`/knowledge-doc/instruction-file `--copilot` mentions | Yes — Approach steps 5–7 |
| Script Reference table (AGENTS.md) | — | Yes — approved by owner |
| `inspiration_agent_workspace_digest.md` "Partially adopted" note | — | Yes — Approach step 9 |
| `requirements.txt` (new dependency) | `.venv` gets it via `make lint`; no separate install step | Yes |
| `.github/workflows/validate.yml` `script-tests` job | Install `markdown-it-py` so the ported test suite's CI-mode assertions don't hard-fail | Yes — owner-approved CI change |
| `.gitignore` (new rows) | — | Yes |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): `.agent/knowledge/inspiration_agent_workspace_digest.md`'s
  "Partially adopted" section currently states the Gemini/Codex dispatch
  "remains unadopted" — this PR makes that false, so the section must be
  updated in the same PR. `.claude/skills/review-code/SKILL.md`'s own
  "remains unadopted... see inspiration_agent_workspace_digest.md"
  callout (after 5d) is likewise stale once the new 5e exists and is
  removed.
- **Agent-instruction candidates** (proposals only): None beyond what's
  already captured — the `_resolve_work_plans_dir.sh` Rule 2b gap for
  layer-worktree sessions entered without sourcing `worktree_enter.sh`
  (see port table) is a real but narrow edge case with a safe abort;
  worth a one-line mention in `.agent/WORKTREE_GUIDE.md` if it's ever
  hit in practice, but not proposed here since it hasn't caused an
  actual failure yet — the operator decides whether it's worth
  documenting pre-emptively.

## Open Questions

- None blocking. The owner's checkpoint decisions (issue comments) and
  the Plan Review r1 must-fixes/should-fix/suggestions (folded in above)
  resolved every design point left open. The one item this plan surfaces
  on its own — the `_resolve_work_plans_dir.sh` Rule 2b gap on
  layer-worktree paths — is not a blocking question: it degrades to a
  safe, clear abort (Rule 3) rather than a silent misroute, so it does
  not need a decision before implementation, only the port-table note
  above. Per owner decision (Plan checkpoint decision 3), this revision
  is implemented **without a second plan review**.

## Estimated Scope

Single PR. Seven new files (six scripts/tests + `_plan_approach.py`),
one deleted specialist section replaced with a new one in
`.claude/skills/review-code/SKILL.md`, seven other edited files
(`requirements.txt`, `.gitignore`, `.github/workflows/validate.yml`,
`AGENTS.md`, `run-issue/SKILL.md`, two instruction files), three edited
knowledge docs, plus the live-verification run recorded in
Implementation Notes.
