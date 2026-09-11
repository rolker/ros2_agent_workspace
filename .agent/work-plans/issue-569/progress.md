---
issue: 569
---

# Issue #569 — Scheduled janitor for staleness/drift detectors

## Issue Review
**Status**: complete
**When**: 2026-09-11 12:40 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #569
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped (as narrowed by the 2026-09-11 scope-decision comment)

### Actions
- [ ] Settle in plan-task: `audit-project`'s repo-location step (`find layers/main/*/src/<repo-name>`) assumes a local layer checkout, which conflicts with the issue's "must not assume `layers/` exists; it clones what it needs" constraint. Decide whether `audit-project` itself gains a clone-fallback (benefits all callers) or the janitor shallow-clones into a scratch dir before invoking it. `issue-triage` needs no such fallback — it already enumerates repos from tracked `.repos` files via `list_overlay_repos.py`, not the checked-out tree.
- [ ] Include in the same PR: add the new skill to the cross-framework skill lists the Consequences Map requires — `.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md` — plus `.agent/knowledge/skill_workflows.md`, which also enumerates `audit-workspace`/`audit-project`/`issue-triage`/`research` and would go stale otherwise.
- [ ] Settle in plan-task: the durable-output design. The three existing periodic skills it chains (`audit-workspace`, `audit-project`, `issue-triage`) report to the conversation only — fine for manual trigger, since a human reads the same session. The janitor is explicitly meant to run unattended, so "one rolling GitHub issue, updated in place" needs to be the actual persisted record, not just a conversational summary that then gets posted; confirm the plan's write step treats the GitHub issue update as the durable output (mirroring `review-issue`'s canonical-local-write-then-best-effort-post pattern) rather than assuming GitHub is always reachable — the plan should say explicitly what happens when the API call to update the issue fails, per the false-green lesson from #609 the issue itself already cites for skipped/failed checks.
- [ ] Forward-looking, non-blocking: this slice explicitly defers the trigger mechanism, but note for that follow-up that updating the rolling issue is itself a GitHub *write*. If the eventual trigger is a container dispatch, ADR-0015/ADR-0019 already establish the pattern to reuse (host publishes; container produces + best-effort posts) — worth citing when that decision is made rather than re-deriving it.

## Notes (not blocking, recorded for the record)

**Scope / right repo**: Yes to both. The proposed skill (chain existing detectors, write one rolling report, no PRs opened) is workspace-generic tooling consistent with ADR-0003 and fits the existing `.claude/skills/{audit-workspace,audit-project,issue-triage,research}` pattern of "Utility/periodic, not tied to the per-issue lifecycle." The 2026-09-11 scope comment's split (sweep now, trigger later) is a good incremental slice — it removes the prior #564 sequencing dependency (correctly noted as lifted: the sweep now *produces* #564's enforcement inventory rather than depending on it) and keeps this PR reviewable on its own.

**Dependencies**: None blocking. All four detectors it chains already exist and are independently invocable. The research-digest half of the motivation is already satisfied — `.agent/knowledge/research_digest.md` was refreshed in PR #624 (merged), which added the "Scheduled & Background Maintenance Agents" entry that itself cites #569 and independently confirms two of the issue's design choices: single rolling report over per-finding issues, and (for the deferred trigger question) that Routines don't reach the `layers/` tree or gitcloud-origin field repos, so a cron+container path remains the fit for anything touching layers.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Report-only, no PRs opened by the sweep; findings surfaced for the operator to triage into issues manually — matches the operator's explicit no-issue-spam preference. |
| Enforcement over documentation | Watch | The mechanism stays manual-trigger this slice by design (trigger deferred); acceptable as an intentional, recorded sequencing choice, not a gap in this issue. |
| Capture decisions, not just implementations | OK | The scope split is recorded as an issue comment (2026-09-11); no ADR needed yet since no lasting architecture decision is made this slice (the trigger/architecture choice is explicitly deferred). |
| A change includes its consequences | Action needed | New-skill cross-adapter list updates (see Actions above) must land in the same PR — this is a standing Consequences Map item ("Workflow skill list (add/remove a skill)") this issue doesn't currently mention. |
| Only what's needed | OK | Chains existing detectors rather than rebuilding any of them; no new infra proposed beyond the report step. |
| Improve incrementally | OK | Explicitly sliced from the trigger/scheduling decision; single reviewable PR. |
| Test what breaks | Watch | The "state what was skipped/failed rather than report clean" constraint (carried over from #609) needs an actual degraded-path test (GitHub API unreachable, a repo not onboarded, `layers/` absent), not just the happy path. |
| Workspace vs. project separation | OK | Generic skill; no project-specific content baked in. Repo rotation should stay data-driven (reads onboarded-repo list), not hardcoded. |
| Workspace improvements cascade to projects | OK | Applies uniformly across onboarded repos via the existing `audit-project` rotation concept. |
| Primary framework first, portability where free | OK | Authored as a Claude Code skill, consistent with the other three detectors it chains. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0003 (project-agnostic workspace) | Yes | Satisfied — sweep and report logic are generic; per-repo rotation is config/data, not hardcoded project content. |
| ADR-0013 (progress.md entry vocabulary) | Watch, not blocking | This skill is Utility/periodic like its three siblings, none of which write `progress.md` — consistent, not a violation. But flag explicitly in the plan that the *rolling GitHub issue* is the durable record for this one (unlike its siblings, meant to run unattended), so a failed post isn't silently indistinguishable from "nothing to report." |
| ADR-0015 / ADR-0019 (dispatch handoff contract / container model) | Not triggered this slice | The issue explicitly defers the trigger mechanism (cloud Routine vs. cron container vs. anacron), which is where these ADRs would apply (GitHub write auth to update the rolling issue, container-vs-in-process rationale). Recorded as a forward pointer only. |

### Consequences

- Skill list additions owed in the same PR: `.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md`, `.agent/knowledge/skill_workflows.md` (all four currently enumerate `audit-workspace`/`audit-project`/`issue-triage`/`research`; confirmed via grep).
- `audit-project`'s local-checkout assumption (`find layers/main/*/src/<repo-name>`) is a real integration gap against the issue's own "clones what it needs" constraint — needs a design decision in plan-task, not just wiring the skill call.

### Recommendations

- Reuse `review-issue`'s two-step output pattern (canonical local artifact first, best-effort GitHub post second) for the rolling-issue update, so a GitHub outage degrades to "report written, not yet posted" rather than silently reporting nothing.
- When the trigger-mechanism follow-up issue is filed, point it at ADR-0015/ADR-0019 up front rather than re-deriving the container-vs-write-auth reasoning from scratch.

## Plan Authored
**Status**: complete
**When**: 2026-09-11 12:46 -04:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-569/plan.md` at `8afd8b5`
**Branch**: feature/issue-569 at `8afd8b5`
**Phases**: single

Four items the `## Issue Review` entry left for this phase are decided in the plan
as **[D1]**–**[D4]**: (D1) the layer-checkout assumption is fixed at its source — a new
`.agent/scripts/resolve_repo_checkout.sh` (layer checkout, else shallow clone into
`.agent/scratchpad/`) that `audit-project` step 1 calls, with layer-dependent checks
reporting SKIPPED in clone mode, rather than a janitor-local workaround; (D2) the four
skill-list sites land in the same PR; (D3) durable output is a local canonical write
first, then the rolling issue (body updated in place + sweep posted as a comment), with
a named POST FAILED path and a next-run backlog post so an unattended failed publish
cannot vanish, plus a per-check OK/FINDINGS/SKIPPED/FAILED contract so a report can only
read clean when all four checks completed; (D4) the deferred trigger decision is pointed
at ADR-0015/ADR-0019 in the skill's deferred-trigger section.

### Open questions
- [ ] The PR edits four instruction files (`AGENTS.md` + three adapters) — Ask First; confirm blanket approval or review at PR time.
- [ ] Should the consequences-map row on durable-findings skills gain a clause for non-issue-scoped skills? Out of this PR unless wanted.
- [ ] May the sweep create the rolling issue on first run (title `Janitor sweep report (rolling)`, no label), or will the operator open it by hand?

## Plan Review
**Status**: complete
**When**: 2026-09-11 12:48 -04:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-569/plan.md` at `8afd8b5`
**PR**: PR-less (`--issue` mode, local-first)
**Verdict**: changes-requested

Reviewed against the 2026-09-11 scope-decision comment (sweep only; trigger deferred;
#564 gate lifted), not the issue body's "weekly scheduled routine" framing. The plan is
structurally sound — all four `## Issue Review` action items are decided ([D1]–[D4]),
the four skill-enumeration sites it names are exactly the four that exist (verified by
`git grep -l issue-triage`), and fixing the repo-location gap in `audit-project` rather
than in the janitor is the right call. Two must-fix findings remain, both of the
false-green / fix-every-site class this workspace already paid for.

### Findings
- [ ] (must-fix) The rotation's real precondition is `configs/manifest`, not `layers/`: it is gitignored and absent in every worktree and fresh clone, and `list_overlay_repos.py` then prints an empty list at **exit 0** (verified in this worktree). A zero-length candidate set would render as "no repos to audit" rather than `FAILED(no repo manifest)` — the exact #609 false-green the issue forbids. The empty-manifest case must be a named FAILED/SKIPPED status, distinct from "repo not listed in any manifest" in `resolve_repo_checkout.sh` too — `plan.md` Approach 3 and 9
- [ ] (must-fix) `audit-project` hardcodes `layers/main/...` at **five** sites (SKILL.md lines 34, 40, 62, 106, 130); the plan rewrites step 1 and marks steps 5/7 SKIPPED, leaving line 62 — the root-`AGENTS.md` currency check, which is precisely what the janitor's onboarding probe needs — and line 130's `**Location**` report header still layer-only — `plan.md` Approach 1, Files to Change
- [ ] (should-fix) Step 5 is mis-characterised as layer-dependent: `audit-project`'s Guidelines say "Don't run tests by default", so step 5 is a test-file-existence check that works fine in clone mode. Only the optional `colcon test` invocation is layer-dependent; marking all of step 5 SKIPPED would under-report a check that did run — `plan.md` Approach 1
- [ ] (should-fix) The failed-post recovery ("the next run detects any unpublished report and posts it before its own") rests on `.agent/scratchpad/janitor/`, which is gitignored and **per-worktree, per-host**. A next run from a different worktree, a fresh clone, or a container cannot see the orphaned report, so the guarantee silently does not hold. Anchor the path at the workspace root and state the limitation in the skill — `plan.md` Approach 5(d)
- [ ] (should-fix) `run_script_tests.sh` is contracted hermetic ("temp sandboxes, stubbed `gh`, no network"). The "clones when `layers/` is absent" case must use a local `file://` origin or a stubbed `git`, never a real remote clone — `plan.md` Approach 9
- [ ] (suggestion) `run_script_tests.sh` needs no edit — it globs `"$TESTS_DIR"/test_*.sh`. Drop the "registered in `run_script_tests.sh`" claim and that Files-to-Change row — `plan.md` Approach 9, Files to Change
- [ ] (suggestion) Creating the rolling issue via `gh_create_issue.sh` from inside a worktree auto-injects `Part of #<WORKTREE_ISSUE>` (gh_create_issue.sh:82-95), permanently mis-parenting the rolling issue to whatever issue the operator was on. Use plain `gh issue create` (no label is needed; validation is skipped when none is given) and include the AGENTS.md AI signature — `plan.md` Open Questions
- [ ] (suggestion) ISO-week-mod rotation carries no coverage guarantee while the trigger is deferred: repeated hand-runs in one week re-audit the same chunk and unrun weeks are never made up, so "cycles all repos in `ceil(N/3)` weeks" holds only under a real weekly trigger. Say so, and have the report name the chunk index and week — `plan.md` Approach 3
- [ ] (suggestion) Consequences-map row "Add a workflow skill that produces durable findings → persist a typed `progress.md` entry" (principles_review_guide.md:49) reads unconditionally; the plan argues inapplicability only in its own ADR table and Open Question 2. Either land the one-line clarifying clause in this PR or get explicit operator sign-off — otherwise every future `audit-workspace` / `review-code` pass re-flags it — `plan.md` ADR Compliance, Open Questions

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-11 13:09 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-569 at `602f3dc`
**Mode**: pre-push
**Depth**: Deep (reason: 1034 lines / 12 files, instruction-file + skill override triggers)
**Must-fix**: 14 | **Suggestions**: 18
**Round**: 1 | **Ship**: continue — must-fix count is high, five are the #609 false-green class the issue forbids (two reproduced directly), and one (public-repo publication) needs an operator decision rather than a mechanical fix

Specialists: Static Analysis (shellcheck clean, pre-commit all-pass, new test 7/7), Governance,
Plan Drift, Claude Adversarial Lens A + Lens B. Copilot and local model not run (not opted in).
Design is sound — fixing the layer assumption at its source in `audit-project`, honest
SKIPPED/FAILED vocabulary, local-write-then-publish, real hermetic tests. All nine prior
plan-review findings verified resolved; all consequence sites landed, none missed.

### Findings
- [ ] (must-fix) Step 3 builds the rotation with a worktree-relative `list_overlay_repos.py` while step 1 and the resolver anchor at the main root; returns `[]` at exit 0 in a worktree, so the sweep aborts FAILED on a fully set-up host — unrunnable in its primary environment (4-way cross-confirmed) — `.claude/skills/janitor-sweep/SKILL.md:88`
- [ ] (must-fix) `printf "$(cd "$candidate" && pwd)"` discards the subshell status: an unreadable layer dir prints an empty path at rc 0 — an empty success, the class the header claims closed (reproduced) — `.agent/scripts/resolve_repo_checkout.sh:78`
- [ ] (must-fix) Layer probe is `[[ -d ]]` only, so a partially-imported empty `src/<repo>` resolves as mode `layer` at rc 0 and nothing downstream validates it (reproduced) — `.agent/scripts/resolve_repo_checkout.sh:76-81`
- [ ] (must-fix) Manifest `version:` is parsed then discarded; the clone takes the remote default branch. Measured: 9 of 45 repos differ from the pinned `jazzy`, four of them `noetic` — clone mode would grade a different branch than layer mode (2-way cross-confirmed) — `.agent/scripts/resolve_repo_checkout.sh:108-115,141`
- [ ] (must-fix) Cache keyed on repo name only; refresh never verifies the cached clone's `origin` still matches the manifest URL, and a name present in two of the three manifests silently takes the first (2-way cross-confirmed) — `.agent/scripts/resolve_repo_checkout.sh:126,133-138`
- [ ] (must-fix) `REPO_NAME` validated only for non-empty / no leading `-`, then interpolated into `TARGET` and fed to `rm -rf`; vcstool keys are paths and `--repos` is operator input — constrain to one path segment — `.agent/scripts/resolve_repo_checkout.sh:60-64,126,140`
- [ ] (must-fix) The onboarding probe's 404 means both "no AGENTS.md" and "repo not visible to this token"; the guidance publishes the latter as the factual exclusion "not onboarded" — a not-run check rendered as a finding — `.claude/skills/janitor-sweep/SKILL.md:103-107`
- [ ] (must-fix) Only checks 2 and 4 have FAILED criteria; checks 1 and 3 are sub-skills with no defined failure evidence, and `issue-triage` has no empty-list guard of its own, so it reports OK over zero repos scanned (2-way cross-confirmed) — `.claude/skills/janitor-sweep/SKILL.md:118-126`
- [ ] (must-fix) `rolker/ros2_agent_workspace` verified PUBLIC, yet the report publishes `**Host**: <hostname>`, the names of excluded field/gitcloud repos, and an absolute local path on POST FAILED — scrub them or require a private rolling issue (operator decision) — `.claude/skills/janitor-sweep/SKILL.md:153,178-185,247`
- [ ] (must-fix) Report filename is date-only, so a second run the same day overwrites an earlier *unpublished* report — defeating the step-2 backlog that exists so a failed post cannot vanish — `.claude/skills/janitor-sweep/SKILL.md:141`
- [ ] (must-fix) The rolling issue is found via the lagging `gh issue list --search` index; two close runs can both create one, wedging every later run at FAILED(ambiguous) and contradicting the skill's own "never a second" invariant (2-way cross-confirmed) — `.claude/skills/janitor-sweep/SKILL.md:197-208`
- [ ] (must-fix) Internal contradiction: Usage says `--dry-run` publishes nothing, but step 2 publishes the backlog unconditionally before step 5, where `--dry-run` stops — `.claude/skills/janitor-sweep/SKILL.md:16` vs `:75-81,254`
- [ ] (must-fix) The added Exception claims all four named periodic skills persist to "its own durable output"; false for three of them, which have no durable output at all — a future audit will flag them against a rule this PR just wrote — `.agent/knowledge/principles_review_guide.md:49`
- [ ] (must-fix) Hardcodes `rolker/ros2_agent_workspace` in four executed `gh` commands, failing ADR-0003's fork-reusability test; sibling skills parameterise it — `.claude/skills/janitor-sweep/SKILL.md:193,198,220,230-231`
- [ ] (suggestion) Shared clone cache has no lock; concurrent runs race on `rm -rf`/`clone`/`reset --hard` and the failure path can delete a concurrent run's fresh tree — close before any trigger lands (2-way cross-confirmed) — `.agent/scripts/resolve_repo_checkout.sh:125-146`
- [ ] (suggestion) `git clone "$repo_url"` has no `--` separator or scheme check, and no `GIT_TERMINAL_PROMPT=0`/timeout for unattended use — `.agent/scripts/resolve_repo_checkout.sh:134,141`
- [ ] (suggestion) A listed repo with no `url:` key misreports as exit 4 "not listed"; the `repo_url` substitution's status is unchecked — `.agent/scripts/resolve_repo_checkout.sh:108-120`
- [ ] (suggestion) `2>&1` folds stderr into the JSON payload, so benign stderr noise on a successful run turns a readable manifest into exit 6 — `.agent/scripts/resolve_repo_checkout.sh:90`
- [ ] (suggestion) Test gaps: exit 6 untested; the refresh case cannot distinguish refresh from re-clone or a stale tree; no failure case asserts stdout is empty; usage covers only the missing-argument arm — `.agent/scripts/tests/test_resolve_repo_checkout.sh:68-142`
- [ ] (suggestion) The no-repo-name branch never sets `$REPO_PATH`, so line 87 reads `/AGENTS.md`; and the resolver is called by a relative path that does not exist in a layer worktree — `.claude/skills/audit-project/SKILL.md:37-39,49,87`
- [ ] (suggestion) `--repos` is item 5 of an ordered list, leaving it ambiguous whether the no-manifest FAILED guard and the exclusion filters still apply on a hand-run — `.claude/skills/janitor-sweep/SKILL.md:111`
- [ ] (suggestion) The AI signature is mandated on the created issue and every comment, but the `--body-file` written is the unsigned canonical report — say where the signature is appended — `.claude/skills/janitor-sweep/SKILL.md:219-241`
- [ ] (suggestion) The non-GitHub exclusion hand-rolls the host allowlist; AGENTS.md names `field_mode.sh` authoritative, and it admits `ssh.github.com` — `.claude/skills/janitor-sweep/SKILL.md:99`
- [ ] (suggestion) The onboarding probe tests AGENTS.md *presence* while ADR-0017's currency signal is the `## Quality Standard` marker; say presence, not currency, is the rotation gate — `.claude/skills/janitor-sweep/SKILL.md:103`
- [ ] (suggestion) Both describe exit 3 as "the state of every worktree and fresh clone", but the script anchors at the main root so a worktree on a set-up host never hits it — `AGENTS.md:571`, `.agent/scripts/resolve_repo_checkout.sh:28-30`
- [ ] (suggestion) Plan Files-to-Change still says "six hermetic cases"; `[R5]` and the test say seven — residual plan drift — `.agent/work-plans/issue-569/plan.md:192`
- [ ] (suggestion) The Ask-First approval for the instruction-file edits exists only as agent-authored plan prose with no durable trace; finding 13 also landed more than the "one-line clause" described — re-confirm at PR time — `.agent/work-plans/issue-569/plan.md:26-33,238-247`
