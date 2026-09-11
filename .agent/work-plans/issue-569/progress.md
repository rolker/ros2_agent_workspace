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

### Notes (not blocking, recorded for the record)

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
- [x] (must-fix) Step 3 builds the rotation with a worktree-relative `list_overlay_repos.py` while step 1 and the resolver anchor at the main root; returns `[]` at exit 0 in a worktree, so the sweep aborts FAILED on a fully set-up host — unrunnable in its primary environment (4-way cross-confirmed) — `.claude/skills/janitor-sweep/SKILL.md:88`
- [x] (must-fix) `printf "$(cd "$candidate" && pwd)"` discards the subshell status: an unreadable layer dir prints an empty path at rc 0 — an empty success, the class the header claims closed (reproduced) — `.agent/scripts/resolve_repo_checkout.sh:78`
- [x] (must-fix) Layer probe is `[[ -d ]]` only, so a partially-imported empty `src/<repo>` resolves as mode `layer` at rc 0 and nothing downstream validates it (reproduced) — `.agent/scripts/resolve_repo_checkout.sh:76-81`
- [x] (must-fix) Manifest `version:` is parsed then discarded; the clone takes the remote default branch. Measured: 9 of 45 repos differ from the pinned `jazzy`, four of them `noetic` — clone mode would grade a different branch than layer mode (2-way cross-confirmed) — `.agent/scripts/resolve_repo_checkout.sh:108-115,141`
- [x] (must-fix) Cache keyed on repo name only; refresh never verifies the cached clone's `origin` still matches the manifest URL, and a name present in two of the three manifests silently takes the first (2-way cross-confirmed) — `.agent/scripts/resolve_repo_checkout.sh:126,133-138`
- [x] (must-fix) `REPO_NAME` validated only for non-empty / no leading `-`, then interpolated into `TARGET` and fed to `rm -rf`; vcstool keys are paths and `--repos` is operator input — constrain to one path segment — `.agent/scripts/resolve_repo_checkout.sh:60-64,126,140`
- [x] (must-fix) The onboarding probe's 404 means both "no AGENTS.md" and "repo not visible to this token"; the guidance publishes the latter as the factual exclusion "not onboarded" — a not-run check rendered as a finding — `.claude/skills/janitor-sweep/SKILL.md:103-107`
- [x] (must-fix) Only checks 2 and 4 have FAILED criteria; checks 1 and 3 are sub-skills with no defined failure evidence, and `issue-triage` has no empty-list guard of its own, so it reports OK over zero repos scanned (2-way cross-confirmed) — `.claude/skills/janitor-sweep/SKILL.md:118-126`
- [x] (must-fix) `rolker/ros2_agent_workspace` verified PUBLIC, yet the report publishes `**Host**: <hostname>`, the names of excluded field/gitcloud repos, and an absolute local path on POST FAILED — scrub them or require a private rolling issue (operator decision) — `.claude/skills/janitor-sweep/SKILL.md:153,178-185,247`
- [x] (must-fix) Report filename is date-only, so a second run the same day overwrites an earlier *unpublished* report — defeating the step-2 backlog that exists so a failed post cannot vanish — `.claude/skills/janitor-sweep/SKILL.md:141`
- [x] (must-fix) The rolling issue is found via the lagging `gh issue list --search` index; two close runs can both create one, wedging every later run at FAILED(ambiguous) and contradicting the skill's own "never a second" invariant (2-way cross-confirmed) — `.claude/skills/janitor-sweep/SKILL.md:197-208`
- [x] (must-fix) Internal contradiction: Usage says `--dry-run` publishes nothing, but step 2 publishes the backlog unconditionally before step 5, where `--dry-run` stops — `.claude/skills/janitor-sweep/SKILL.md:16` vs `:75-81,254`
- [x] (must-fix) The added Exception claims all four named periodic skills persist to "its own durable output"; false for three of them, which have no durable output at all — a future audit will flag them against a rule this PR just wrote — `.agent/knowledge/principles_review_guide.md:49`
- [x] (must-fix) Hardcodes `rolker/ros2_agent_workspace` in four executed `gh` commands, failing ADR-0003's fork-reusability test; sibling skills parameterise it — `.claude/skills/janitor-sweep/SKILL.md:193,198,220,230-231`
- [x] (suggestion) Shared clone cache has no lock; concurrent runs race on `rm -rf`/`clone`/`reset --hard` and the failure path can delete a concurrent run's fresh tree — close before any trigger lands (2-way cross-confirmed) — `.agent/scripts/resolve_repo_checkout.sh:125-146`
- [x] (suggestion) `git clone "$repo_url"` has no `--` separator or scheme check, and no `GIT_TERMINAL_PROMPT=0`/timeout for unattended use — `.agent/scripts/resolve_repo_checkout.sh:134,141`
- [x] (suggestion) A listed repo with no `url:` key misreports as exit 4 "not listed"; the `repo_url` substitution's status is unchecked — `.agent/scripts/resolve_repo_checkout.sh:108-120`
- [x] (suggestion) `2>&1` folds stderr into the JSON payload, so benign stderr noise on a successful run turns a readable manifest into exit 6 — `.agent/scripts/resolve_repo_checkout.sh:90`
- [x] (suggestion) Test gaps: exit 6 untested; the refresh case cannot distinguish refresh from re-clone or a stale tree; no failure case asserts stdout is empty; usage covers only the missing-argument arm — `.agent/scripts/tests/test_resolve_repo_checkout.sh:68-142`
- [x] (suggestion) The no-repo-name branch never sets `$REPO_PATH`, so line 87 reads `/AGENTS.md`; and the resolver is called by a relative path that does not exist in a layer worktree — `.claude/skills/audit-project/SKILL.md:37-39,49,87`
- [x] (suggestion) `--repos` is item 5 of an ordered list, leaving it ambiguous whether the no-manifest FAILED guard and the exclusion filters still apply on a hand-run — `.claude/skills/janitor-sweep/SKILL.md:111`
- [x] (suggestion) The AI signature is mandated on the created issue and every comment, but the `--body-file` written is the unsigned canonical report — say where the signature is appended — `.claude/skills/janitor-sweep/SKILL.md:219-241`
- [x] (suggestion) The non-GitHub exclusion hand-rolls the host allowlist; AGENTS.md names `field_mode.sh` authoritative, and it admits `ssh.github.com` — `.claude/skills/janitor-sweep/SKILL.md:99`
- [x] (suggestion) The onboarding probe tests AGENTS.md *presence* while ADR-0017's currency signal is the `## Quality Standard` marker; say presence, not currency, is the rotation gate — `.claude/skills/janitor-sweep/SKILL.md:103`
- [x] (suggestion) Both describe exit 3 as "the state of every worktree and fresh clone", but the script anchors at the main root so a worktree on a set-up host never hits it — `AGENTS.md:571`, `.agent/scripts/resolve_repo_checkout.sh:28-30`
- [x] (suggestion) Plan Files-to-Change still says "six hermetic cases"; `[R5]` and the test say seven — residual plan drift — `.agent/work-plans/issue-569/plan.md:192`
- [x] (suggestion) The Ask-First approval for the instruction-file edits exists only as agent-authored plan prose with no durable trace; finding 13 also landed more than the "one-line clause" described — re-confirm at PR time — `.agent/work-plans/issue-569/plan.md:26-33,238-247`

## Implementation
**Status**: complete
**When**: 2026-09-11 13:35 -04:00
**By**: Claude Opus

**Branch**: feature/issue-569 at `c0ff8fc`
**Addressed**: `## Local Review (Pre-Push)` (2026-09-11 13:09 -04:00, branch at `602f3dc`), verdict changes-requested, round 1
**Commits**: `c24fc16`, `0de04d8`, `af824dd`, `ff0d236`, `c6ab389`, `3f889c3`, `c179c2e`, `07dbf3d`, `c0ff8fc`

### Scope change made at this checkpoint

The operator chose **local report only for now**: the sweep writes its local
report file and nothing else. The rolling GitHub issue, the exact-title lookup,
the create-once path, the `POST FAILED` headline, the `.unpublished` backlog
and `--dry-run` are removed from this slice. Publishing is deferred **together
with** the trigger, since where a sweep's findings live durably depends on what
runs the sweep; any future GitHub publish must cite ADR-0015/ADR-0019
(container produces, host publishes). `plan.md` marks those deliverables
`[DEFERRED-PUB]` rather than deleting their design.

Three must-fix findings are therefore **resolved by removal**, not on their
merits — called out individually below so they are not read as fixed.

### Actions — must-fix

- [x] 1. Step 3 built the rotation with a worktree-relative `list_overlay_repos.py` — **fixed**. Every path the sweep touches is addressed through `$ROOT` (the main workspace root), and step 1 now carries the three environments this must work in with how each is verified: set-up host from the main checkout, set-up host from a worktree, and no `layers/` at all. The worktree half is covered mechanically by the resolver's new worktree test case — `.claude/skills/janitor-sweep/SKILL.md` steps 1-2, `.agent/scripts/tests/test_resolve_repo_checkout.sh` case 9
- [x] 2. `printf "$(cd ...)"` discarded the subshell status — **fixed**. The `cd` status reaches the exit code; an unreadable candidate is exit 5 with a reason, and nothing is written to stdout on any failure path — `.agent/scripts/resolve_repo_checkout.sh`
- [x] 3. Layer probe was `[[ -d ]]` only — **fixed**. An empty `src/<repo>` (partial `vcs import`) falls through to the clone path with a note. Unreadable is tested *before* empty, since `ls -A` cannot tell them apart and a permissions fault must not be downgraded to a silent fallback — `.agent/scripts/resolve_repo_checkout.sh`
- [x] 4. Manifest `version:` parsed then discarded — **fixed**. The clone takes the pin (branch, tag, or a SHA via a ref fetch) and refreshes against that same ref. Re-measured on this host's manifest rather than quoting the review's number: 35 overlay repos, of which `rqt_marine_radar` pins `jazzy` while the remote still defaults to `noetic` — `.agent/scripts/resolve_repo_checkout.sh`
- [x] 5. Cache never verified against the manifest URL, and a duplicated name took the first — **fixed**. A cached clone whose `origin` differs from the manifest url is re-cloned; one name with conflicting urls across manifests is the new exit 7, never a silent first-match — `.agent/scripts/resolve_repo_checkout.sh`
- [x] 6. `REPO_NAME` unvalidated before `rm -rf` — **fixed**. It must now be a single path segment; `../..`, `foo/bar`, `-rf`, `.`, `..`, empty and embedded-space names are all exit 2, asserted in the test — `.agent/scripts/resolve_repo_checkout.sh`
- [x] 7. The onboarding probe published "not visible to this token" as "not onboarded" — **fixed**. The repo's visibility is probed first; only a *visible* repo's missing `AGENTS.md` is an exclusion, and any other probe failure is FAILED for that repo. The step also states it gates on presence, not currency — `.claude/skills/janitor-sweep/SKILL.md` step 2
- [x] 8. Checks 1 and 3 had no FAILED criteria, and `issue-triage` had no empty-list guard — **fixed in both places**. The janitor names each check's failure evidence; the empty-manifest / unreadable-manifest / failed-per-repo-list guards landed in `issue-triage`'s own step 1, not the janitor's wrapper, because every other caller shares the hole — `.claude/skills/janitor-sweep/SKILL.md` step 3, `.claude/skills/issue-triage/SKILL.md` step 1
- [x] 9. The report published the hostname, excluded field-repo names and an absolute path to a public repo — **resolved by removal** (nothing is published). The concern still shaped the local format: it names files relative to the workspace root and records no hostname, so the deferred publish decision inherits a format needing no scrubbing. Excluded repo names stay — they are already in the tracked manifests
- [x] 10. Date-only report filename overwrote an earlier report — **fixed**, and it still applies with publishing gone. The name is timestamped, generated with `date` and never hand-typed, so successive runs accumulate — `.claude/skills/janitor-sweep/SKILL.md` step 4
- [x] 11. Two close runs could each create a rolling issue through the lagging search index — **resolved by removal**. The settled design is carried forward in `plan.md` as `[DEFERRED-PUB]`, with the race recorded as something the deferred decision must solve before a create path ships
- [x] 12. `--dry-run` published the backlog before the step where it stopped — **resolved by removal**. `--dry-run` is gone with the publish step it existed to suppress; an opt-out publish path is still a publish path to review
- [x] 13. The consequences-map clause claimed durable output for all four periodic skills — **fixed**. It now says only what is true: the row does not apply to a non-issue-scoped skill; three of the four persist nothing at all (a gap, not a pattern to copy); only `janitor-sweep` has a durable output, and after the scope change that is a local file — `.agent/knowledge/principles_review_guide.md:49`
- [x] 14. Hardcoded `rolker/ros2_agent_workspace` in four `gh` commands — **fixed by removal plus re-check**. All four were in the publish step. Re-grepped after the removal: no repo slug remains anywhere in the skill, and the only `gh` calls left derive `<owner>/<repo>` from the manifest url (ADR-0003's fork test). The deferred publish design records `gh repo view --json nameWithOwner` as the derivation to use

### Actions — suggestions

- [x] 15. Unlocked shared clone cache — **fixed** (taken because the cache was being reworked anyway). A per-repo `flock`, held across the whole clone/refresh, serialises runs; where `flock` is absent the script proceeds and says so — `.agent/scripts/resolve_repo_checkout.sh`
- [x] 16. `git clone` without `--`, scheme check, `GIT_TERMINAL_PROMPT=0` or timeout — **fixed**. All four, plus `GIT_ASKPASS`; an unrecognised url is exit 6 and never reaches git
- [x] 17. A listed repo with no `url:` misreported as exit 4 — **fixed**: it is exit 6, a malformed entry, and the lookup's status is checked
- [x] 18. `2>&1` folded stderr into the JSON payload — **fixed**: stdout and stderr are kept apart, so benign noise on a successful run no longer reads as an unreadable manifest
- [x] 19. Test gaps — **fixed**: the suite went from 7 cases to 18, adding exit 6 (three arms), a refresh distinguishable from a re-clone via an untracked sentinel plus a real upstream advance, an empty-stdout assertion on every failure case, and all three usage arms
- [x] 20. `audit-project`'s unset `REPO_PATH` and relative resolver path — **fixed**: both branches set `REPO_PATH`/`REPO_MODE`, and scripts are addressed through the main root with the layer-worktree caveat stated — `.claude/skills/audit-project/SKILL.md` step 1
- [x] 21. `--repos` ambiguity as item 5 of an ordered list — **fixed**: Usage says it replaces the chunk selection only, every other guard still applies, and names the direct `/audit-project <repo>` route for auditing an excluded repo deliberately
- [x] 22. The AI signature was mandated but the `--body-file` written was unsigned — **resolved by removal** (no issue or comment is created)
- [x] 23. Hand-rolled GitHub host allowlist — **fixed**: defers to `field_mode.sh`, which AGENTS.md names authoritative and which admits `ssh.github.com`
- [x] 24. Probe tested presence while ADR-0017's signal is the `## Quality Standard` marker — **fixed**: the rotation gate says presence, and the marker stays `audit-project`'s check
- [x] 25. "the state of every worktree and fresh clone" described exit 3 wrongly — **fixed in both places**: the script header and the `AGENTS.md` row now say an un-bootstrapped clone or a container, since the script resolves against the main root
- [x] 26. Plan said "six hermetic cases" while `[R5]` said seven — **fixed**: both now describe the eighteen actually written
- [x] 27. The Ask-First approval had only agent-authored plan prose as its trace — **recorded, and flagged for re-confirmation**. The plan now says where the approval was given and that it must be re-confirmed at PR time before merge; a durable trace is the operator's to leave

### Notes

- The review entry's header says **18 suggestions**, but only **13** suggestion
  checkboxes are present in its Findings list. All 13 are actioned above; the
  five-item gap is in the source entry's count, not an unactioned backlog.
- Two commits are coarser than one-finding-per-commit: the `janitor-sweep`
  rewrite (`af824dd`) and the resolver rewrite (`c24fc16`). Both files were
  rewritten wholesale — the janitor because the scope change rewrote the steps
  the other findings live in — so the commit messages enumerate each finding
  instead.
- Verification: `pre-commit run --all-files` all-pass (18 hooks; run via the
  workspace venv, since `make lint` is broken on this machine by an unrelated
  `core.hooksPath` setting), and `.agent/scripts/tests/run_script_tests.sh`
  green — 17 shell suites including `test_resolve_repo_checkout.sh` at 18/18,
  plus 215 pytest cases.
- Not pushed, per the dispatch contract.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-11 13:52 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-569 at `c974e26`
**Mode**: pre-push
**Depth**: Deep (reason: 1653 lines / 13 files, instruction-file + new-script + skill triggers)
**Must-fix**: 9 | **Suggestions**: 20
**Round**: 2 | **Ship**: continue — must-fix is down from 14 but not low, and three are genuine correctness concerns rather than mechanical edits: a reproduced command-execution-plus-false-green path through the manifest `version:`, a sweep rule that cites a script it cannot call and whose failure mode silently *includes* the repos it exists to exclude, and a chained skill that reports FAILED on every worktree run

Specialists: Static Analysis (shellcheck clean at `--severity=warning`; pre-commit 18 hooks all-pass on the changed files; `test_resolve_repo_checkout.sh` 18/18), Governance, Plan Drift, Claude Adversarial Lens A + Lens B. Copilot and local model not run (not opted in).

All 27 round-1 findings independently verified closed in the code — 24 fixed on their
merits, 3 genuinely and completely removed with the publish step (no residue in the skill;
the design is carried forward in `plan.md` as `[DEFERRED-PUB]`). The design continues to
hold up: the resolver is the right shape, the tests are hermetic and non-vacuous, and the
SKIPPED/FAILED vocabulary is real. This round's findings are new ground reached by looking
past the round-1 list — chiefly the three environments the sweep claims to run in, and the
one input the resolver still does not validate.

### Findings
- [x] (must-fix) A manifest `version:` reaches `git fetch --depth 1 origin "$REPO_VERSION"` unvalidated and with no `--`, so `--upload-pack=<script>` executes arbitrary code — and the run still exits 0 reporting mode `clone` with the pin silently unhonoured (reproduced end-to-end); `REPO_URL` is validated for exactly this, `version:` is not — `.agent/scripts/resolve_repo_checkout.sh:283,308`
- [x] (must-fix) Step 2 rule 2 names `field_mode.sh` the authoritative allowlist, but `is_field_mode` needs a checkout on disk and the rotation is built from manifest urls before anything is resolved; with no checkout it returns 1 = "dev mode", so every gitcloud repo is silently *included* — the false green the rule exists to prevent — `.claude/skills/janitor-sweep/SKILL.md:107-112`
- [x] (must-fix) `issue-triage` step 1 still enumerates with a worktree-relative `list_overlay_repos.py` (0 repos from a worktree vs 35 from the main root, reproduced), so the guard this PR adds fires on every sweep run from a worktree — the sweep's stated primary environment — as `FAILED: no repo manifest configured — run 'make setup-all'`, a false FAILED with wrong remediation — `.claude/skills/issue-triage/SKILL.md:32`
- [x] (must-fix) If `gh` is unauthenticated or rate-limited every candidate lands in rule 3's FAILED bucket, the chunk is empty, and check 2's rollup defines FAILED only for "failed to resolve / failed to audit" — so the report can read `Project governance | OK | 0 repos audited` — `.claude/skills/janitor-sweep/SKILL.md:127-139,168-171`
- [x] (must-fix) Step 1's environment table claims a no-`layers/` host still enumerates repos, but `configs/manifest` is a symlink *into* `layers/main/core_ws/src/unh_marine_autonomy/config` here, so the stated verification can never pass and the sweep is FAILED there — contradicting the resolver header and the AGENTS.md row, which both say a container hits exit 3 — `.claude/skills/janitor-sweep/SKILL.md:87,39`, `.claude/skills/audit-project/SKILL.md:18,67`
- [x] (must-fix) The consequences-map clause this PR adds requires a durable-output skill to "name the state in which that write failed"; `janitor-sweep` is the only skill it applies to and step 4 names none — it asserts the write always succeeds, and the four-state contract has no row for a read-only or full filesystem — `.agent/knowledge/principles_review_guide.md:49` vs `.claude/skills/janitor-sweep/SKILL.md:201-202`
- [x] (must-fix) Principles Self-Check still lists "exactly one rolling issue" as a delivered property — the one place in the plan that still promises the publish step this slice removed — `.agent/work-plans/issue-569/plan.md:267`
- [x] (must-fix) "(operator decision, 2026-09-11 on issue #569)" is not true of the issue: #569's only scope comment still promises "writing one rolling report". Post the later decision as a comment or cite the timeline instead — `.claude/skills/janitor-sweep/SKILL.md:287-288`
- [x] (must-fix) `GIT_TERMINAL_PROMPT=0` + `GIT_ASKPASS=/bin/true` do not cover ssh, which reads host-key and passphrase prompts from `/dev/tty`; the manifests carry a `git@github.com:` url, so the header's "never prompt / meant to run unattended" guarantee is false there, and unbounded on a host without `timeout` — `.agent/scripts/resolve_repo_checkout.sh:92-97`
- [x] (suggestion) `flock 9` has no `-w` timeout, so one stuck run blocks every concurrent run on that repo indefinitely — `.agent/scripts/resolve_repo_checkout.sh:258`
- [x] (suggestion) The lock is released when the resolver exits, so the caller audits `$TARGET` unlocked and a concurrent `clone_fresh`'s `rm -rf` can still delete a tree being read; the header overstates what the lock buys — `.agent/scripts/resolve_repo_checkout.sh:49-51,273`
- [x] (suggestion) No retention policy for either output: `janitor-repos/` accumulates a shallow clone per audited repo and `janitor/` a report per run, both gitignored so nothing will flag the growth — `.claude/skills/janitor-sweep/SKILL.md:193,243`
- [x] (suggestion) The rotation ignores `configs/manifest/optional_layers.txt`, so a repo from an optional layer produces `FAILED(repo probe)` on every sweep forever — a permanent false red — `.claude/skills/janitor-sweep/SKILL.md:122-131`
- [x] (suggestion) The script header's exit-5 vocabulary ("clone or refresh failed") is narrower than the code, which also returns 5 for `mktemp`, cache `mkdir`, `flock` and an unreadable layer checkout; the AGENTS.md row is more complete than the script's own header — `.agent/scripts/resolve_repo_checkout.sh:39`
- [x] (suggestion) Test gaps on the most intricate branches: the SHA-pin fallback, the `FETCH_REF=HEAD` refresh, and the `flock` path are unexercised, and the worktree case covers only the layer branch, not the clone branch — `.agent/scripts/tests/test_resolve_repo_checkout.sh`
- [x] (suggestion) Failure messages interpolate `$REPO_URL` verbatim into stderr, which the caller funnels into the report step 4 says must stay scrubbed — latent credential leak if a manifest ever carries userinfo — `.agent/scripts/resolve_repo_checkout.sh:287,294,306`
- [x] (suggestion) `underlay.repos` is excluded from the search, but exit 4 says "not listed in any of the N configured repos" — name the exclusion so the operator does not hunt a manifest bug — `.agent/scripts/resolve_repo_checkout.sh:209`
- [x] (suggestion) `ISO-week mod chunk-count` is left to be re-derived while every other computation is given as a snippet; `date +%V` is zero-padded, so `$(( 08 % n ))` is an invalid-octal error — give the `10#` form — `.claude/skills/janitor-sweep/SKILL.md:140-141`
- [x] (suggestion) The no-repo-name branch asserts `REPO_MODE=layer` without observing it (so step 7's "correct layer" can report Yes where it could not run), step 5's snippet is still fully relative against the main-root rule added above it, and the "has at least one `package.xml`" verification has no check — `.claude/skills/audit-project/SKILL.md:57-61,166`
- [x] (suggestion) Step 1's `ROOT=` snippet is byte-identical to `audit-project`'s but omits its layer-worktree caveat — `.claude/skills/janitor-sweep/SKILL.md:70-71`
- [x] (suggestion) Step 2's example runs `list_overlay_repos.py` with no `--format` while the table uses `names` and the resolver uses `json` — say which "the list" is — `.claude/skills/janitor-sweep/SKILL.md:95`
- [x] (suggestion) Nothing tracks the deferred publishing-and-trigger decision: the skill, the plan and `skill_workflows.md` all cross-reference it with no issue number — file it at PR time (recorded as an open item in `plan.md` § Open items for PR time — the operator files the issue, not the agent)
- [x] (suggestion) "The resolver is tested, including three distinct failure paths" is stale — exits 2 through 7 are each tested across 18 cases — `.agent/work-plans/issue-569/plan.md:270`
- [x] (suggestion) "six resolver defects" reads as a complete enumeration of the merit-fixed round-1 findings but omits 7, 10 and 14 (and five, not six, were against the resolver) — `.agent/work-plans/issue-569/plan.md:59`
- [x] (suggestion) [R9] still describes the landed clause as "persists its record to its own rolling report" — stale wording, and contradicted by the Files-to-Change row 47 lines later — `.agent/work-plans/issue-569/plan.md:203`
- [x] (suggestion) `## Notes (not blocking, recorded for the record)` is a non-canonical H2 among ADR-0013 entry types, so `progress_read.py` cannot see its content — demote to H3 — `.agent/work-plans/issue-569/progress.md:22`
- [x] (suggestion) The AGENTS.md row says "a per-repo `flock` serialises concurrent runs" without the script's own documented degradation: absent `flock`, it warns and proceeds unlocked — `AGENTS.md:571`
- [x] (suggestion) Run from a worktree the script is the worktree's copy but executes the MAIN branch's `list_overlay_repos.py`; deliberate, but a worktree that changes both tests only half its change — say so in the header — `.agent/scripts/resolve_repo_checkout.sh:139-151`
- [x] (suggestion) The Ask-First re-confirmation the plan itself defers to PR time is still owed, and the `principles_review_guide` clause landed as a four-sentence paragraph rather than the "one-line clause" the approval was framed around — `.agent/work-plans/issue-569/plan.md:305-309` (recorded as an open item in `plan.md`; the clause's growth past "one line" is named there for the re-confirmation)
- [x] (suggestion) The issue title is "scheduled ... sweep" and the trigger is deferred, so the PR body should say "Part of #569" and leave it open rather than closing it (recorded as an open item in `plan.md` § Open items for PR time — the PR body is written at push time)

## Implementation
**Status**: complete
**When**: 2026-09-11 14:08 -04:00
**By**: Claude Opus (Claude Opus 5, 1M context)

**Branch**: feature/issue-569 at `0074164`
**Addressed**: the `## Local Review (Pre-Push)` of 2026-09-11 13:52 -04:00 (round 2, branch at `c974e26`) — 9 must-fix, 20 suggestions
**Commits**: 04f07c3, ebd81bc, 57d9538, d882440, 6076662, 60fa85e, ee37859, f00af9a, 8bd3786, 0074164

Three of the nine were design-level and changed what the slice contains, so
they are recorded here rather than only in the boxes:

- **The manifest is the first thing "it clones what it needs" must cover.**
  `configs/manifest` is a symlink *into* `layers/`, so the no-`layers/`
  environment had no manifests at all and the skill's claim about it could not
  be true. Rather than soften the claim, it is made true: the tracked
  `configs/project_bootstrap.url` names the manifest repo and branch, so
  `manifest_fallback.sh` derives, shallow-clones and hands back a config dir,
  and `list_overlay_repos.py --config-dir` **adds** it to the search path.
  Resolver exit 3 now means "no manifest AND no bootstrap pointer"; a manifest
  clone that failed is exit 5 — `make setup-all` cannot fix an unreachable
  remote, so it must not be the advice. Hermetic throughout: the derived clone
  host is overridable (`WORKSPACE_MANIFEST_GIT_BASE`), and the tests point it at
  a local `file://` fixture.
- **One allowlist, reachable without a checkout.** `is_field_url` is factored
  out of `is_field_mode` in `field_mode.sh` — the same file, so ADR-0011 keeps
  one authority — and the sweep classifies manifest urls with it.
- **`issue-triage` anchors its enumeration at `$ROOT`**, so the empty-list guard
  this PR added fires only when nothing is genuinely configured (0 repos from a
  worktree vs 35 from the main root, re-measured on this host).

Verification: `make lint` — all 18 pre-commit hooks Passed;
`.agent/scripts/tests/run_script_tests.sh` — 26 shell test files ✅, 217 pytest
cases passed, `✅ All script tests passed.` The resolver's own suite is 29
hermetic cases (was 18).

### Actions

Must-fix:
- [x] 1 — `version:` validated as a full SHA or a ref-safe name before git sees it, `--` on every user-derived refspec, SHA pins verified against HEAD after the detach, url userinfo redacted in stderr — `.agent/scripts/resolve_repo_checkout.sh` (ebd81bc). Negative tests include a real `--upload-pack=` helper asserting it was never executed.
- [x] 2 — `is_field_url` in `.agent/scripts/field_mode.sh`; the sweep's step 2 rule 2 calls it and says why `is_field_mode` is wrong there (04f07c3, ee37859).
- [x] 3 — `issue-triage` step 1 enumerates through `$ROOT`, with the manifest fallback before the empty-list guard — `.claude/skills/issue-triage/SKILL.md` (60fa85e).
- [x] 4 — check 2's rollup now covers the rotation: any failed candidate probe is FAILED, an empty chunk is never OK, its three causes are distinguished, and `gh auth status` is probed once so a shared cause is named once (f00af9a).
- [x] 5 — the environment table's third row states what the code now does (manifest clone → enumerate → mode `clone`), and the verification named is one that passes; the resolver header and the `AGENTS.md` row say the same (d882440, f00af9a).
- [x] 6 — `FAILED(report write: <reason>)` added as the fifth state in the status contract and named in step 4 and step 5, with the findings printed inline so a run is not lost with its file (f00af9a).
- [x] 7 — the Principles Self-Check line no longer promises a rolling issue (0074164).
- [x] 8 — cited to the issue comment that actually records the decision (issue #569 comment 5637608783's successor, the 2026-09-11 "Scope update"), which the host posted; nothing re-posted from here (f00af9a).
- [x] 9 — `GIT_SSH_COMMAND` with `BatchMode=yes` / `StrictHostKeyChecking=accept-new` / `ConnectTimeout`, `timeout` where available and a loud note where not; the header states precisely what each piece buys (57d9538).

Suggestions — all taken except three that belong to PR time:
- [x] `flock -w` (300s, `RESOLVE_LOCK_TIMEOUT`), and an honest account of what the lock does not buy: it is released at exit, so the caller reads the tree unlocked (6076662).
- [x] Retention policy documented for both scratchpad outputs — keep the last 20 reports; the clone caches are disposable (f00af9a).
- [x] `optional_layers.txt` honoured: an inaccessible optional-layer repo is an exclusion, not a permanent false red (ee37859).
- [x] `date +%V` given as `10#$WEEK`, with the chunk arithmetic as a snippet (ee37859).
- [x] Exit-vocabulary consistency (header ⇄ `AGENTS.md`), the `underlay.repos` exclusion named in exit 4, the worktree/MAIN-script split recorded (6076662).
- [x] Test gaps closed: SHA pin honoured and SHA pin absent, `FETCH_REF=HEAD` refresh, both `flock` branches, and the worktree case's **clone** branch (ebd81bc, 6076662).
- [x] `audit-project`: the mode is observed rather than asserted, the `package.xml` verification has a check, step 5's snippet is `$ROOT`-addressed (8bd3786).
- [x] Sweep step 1 carries `audit-project`'s layer-worktree caveat; step 2 says `--format json` and why (ee37859, f00af9a).
- [x] Plan staleness: "three distinct failure paths", "six resolver defects", [R9]'s rolling-report wording (0074164).
- [x] `## Notes` demoted to `### Notes` so `progress_read.py` can see its content.
- [x] `AGENTS.md` row records the `flock` degradation (proceeds unlocked, and says so) (6076662).
- [x] The deferred publishing-and-trigger decision has no tracking issue — recorded as an open item in `plan.md` § Open items for PR time. **Not filed from here**: the operator files it (deferred by instruction).
- [x] The Ask-First re-confirmation, and the clause that landed as a paragraph rather than one line — recorded in `plan.md`'s Open Questions with what grew, for the re-confirmation at PR time (deferred: it is the operator's confirmation to give).
- [x] PR body must say "Part of #569" — recorded as an open item in `plan.md`; the body is written at push time (deferred: nothing is pushed from here).

### Not done
- Nothing is pushed; no PR is opened or updated from this pass.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-11 14:20 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-569 at `9880b8e`
**Mode**: pre-push
**Depth**: Deep (reason: 2691 added / 19 files, instruction-file + new-script + skill triggers)
**Must-fix**: 4 | **Suggestions**: 18
**Round**: 3 | **Ship**: continue — must-fix is down 14 → 9 → 4 and every remaining item is a precise, named fix, but one of them is a *reproduced* false green in a shipped script (a cached manifest clone is reused without checking its origin against the bootstrap pointer, so the whole repo rotation can enumerate from a manifest the workspace no longer points at), and three more mean the no-`layers/` environment this round was built for does not work end to end as written. One short fix pass, not another design round.

Specialists: Static Analysis (pre-commit, 18 hooks all Passed on `main...HEAD`; `run_script_tests.sh` — 26 shell test files ✅, 217 pytest cases; resolver suite 29/29, field-mode 38/38, workspace-lib 20/20), Governance, Plan Drift, Claude Adversarial Lens A + Lens B. Copilot and local model not run (not opted in).

All 9 round-2 must-fix findings independently verified closed in the code, and 18 of 20
suggestions; the two partials (optional-layer exclusion, exit-vocabulary consistency) are
re-raised below. The `version:` validation and the ssh/no-prompt guarantees were checked
against the code and hold as documented — the `--upload-pack=` negative test proves the
helper is never executed, and no network git call escapes `GIT_NET` /
`_manifest_fallback_git`. Environment 3 was traced end to end on this host: the tracked
pointer parses, the derived url and branch match `bootstrap.yaml` exactly, and the manifest
repo is public. This round's findings are the seam *between* the new manifest fallback and
the callers that were taught about it — three of the four must-fix are cross-confirmed by
two or more independent passes.

### Findings
- [x] (must-fix) A cached manifest clone is reused without verifying its `origin` against the url derived from the current bootstrap pointer — repointing the pointer at a different manifest repo returns the OLD clone's config dir at rc 0, so the entire rotation enumerates from a manifest the workspace no longer points at; `resolve_repo_checkout.sh:446` performs exactly this guard one level down (reproduced) — `.agent/scripts/manifest_fallback.sh:110-125`
- [x] (must-fix) Step 2's rotation command drops the `--config-dir` step 1 just computed, so on a no-`layers/` host it enumerates zero repos and rule 1 fires `FAILED(no repo manifest configured — run 'make setup-all')` — the wrong remedy lines 117-120 exist to forbid, and a direct contradiction of rule 1's own "step 1's fallback has already added the cloned manifest's config dir" (Lens A + Lens B) — `.claude/skills/janitor-sweep/SKILL.md:151-155` vs `:166-169`
- [x] (must-fix) The optional-layer exclusion reads `$ROOT/configs/manifest/optional_layers.txt`, the path behind the missing symlink: `get_optional_layers` gained no `extra_config_dirs`, so in the no-`layers/` environment it returns an empty set and every inaccessible optional-layer repo is `FAILED(repo probe)` on every sweep — the permanent false red this rule was written to prevent (Governance + Lens B + Plan Drift) — `.claude/skills/janitor-sweep/SKILL.md:207-220`, `.agent/scripts/lib/workspace.py:124-137`
- [x] (must-fix) `issue-triage`'s fallback snippet short-circuits on rc 5 as well as rc 3, so an unreachable manifest repo never reaches an enumeration and lands on `FAILED: no repo manifest configured — run 'make setup-all'`; the guard list has no rc-5 arm, so the consequence `AGENTS.md`'s own `manifest_fallback` row states ("never reported as 'no manifest configured'") is unreachable from this skill (Lens A + Lens B) — `.claude/skills/issue-triage/SKILL.md:40-78`
- [x] (suggestion) `field_mode.sh` gained a second public entry point, `is_field_url`, and neither its `AGENTS.md` script row nor § Field Mode's sourced-usage example mentions it — while the sweep sends readers to that section as the authority for the very `is_field_url`-not-`is_field_mode` distinction — `AGENTS.md:175-195,556`
- [x] (suggestion) Exit-vocabulary drift, now inverted from round 2: `AGENTS.md:571` exit 6 omits the malformed-`version:` case and exit 5 omits the mktemp / cache-mkdir / lock causes the script header lists; `manifest_fallback`'s row calls exit 5 "could not be cloned" when it also covers cache-mkdir, lock timeout and a pointer/manifest disagreement — `AGENTS.md:571-572`, `.claude/skills/audit-project/SKILL.md:88`
- [x] (suggestion) `WORKSPACE_MANIFEST_GIT_BASE` is read from the ambient environment and interpolated into `git clone` with no validation, while `resolve_repo_checkout.sh:321-326` validates exactly this shape for repo urls — and this is the root of the trust chain (manifest repo → the repo list → every repo cloned) — `.agent/scripts/manifest_fallback.sh:88,120`
- [x] (suggestion) The manifest git url and branch are derived from the raw.githubusercontent path rather than read from `bootstrap.yaml`'s authoritative `git_url:`/`branch:` (what `setup_layers.sh` uses); they agree here, but a fork whose bootstrap names a different host would enumerate from a repo the workspace never uses, silently — `.agent/scripts/manifest_fallback.sh:67-88` (deferred: out of scope for this fix pass — the derived url and branch were verified to match bootstrap.yaml on this host; reading the YAML authoritatively is a design change, recorded for the next round)
- [x] (suggestion) `manifest_config_dir` returns success on a config dir that merely exists, never checking it holds any `.repos`; an empty-but-successful manifest clone then reaches the `EMPTY` branch whose message asserts "and none could be derived from configs/project_bootstrap.url" — false on that path — `.agent/scripts/manifest_fallback.sh:127`, `.agent/scripts/resolve_repo_checkout.sh:293` (deferred: recorded — the EMPTY-branch wording overlaps the manifest-unreadable vocabulary; a wording/behaviour change beyond this pass)
- [x] (suggestion) The manifest fd-8 lock is always taken inside a command substitution, so it drops the instant the function returns and the caller reads the clone unlocked; the resolver documents exactly this caveat for its own lock, the manifest helper and its `AGENTS.md` row advertise the lock without it — `.agent/scripts/manifest_fallback.sh:100-107` (deferred: recorded at the operator's direction — widening the lock's scope changes the caller's contract, not a one-line fix)
- [x] (suggestion) `_manifest_fallback_git` silently drops the wall-clock bound when `timeout(1)` is absent, where the resolver prints a loud note in the identical situation — and the manifest clone is the first network operation a bare host performs — `.agent/scripts/manifest_fallback.sh:139-147` (deferred: recorded — the loud-note parity with the resolver is cosmetic next to this round's false-green fixes)
- [x] (suggestion) No sourced-vs-executed guard: run directly, `manifest_fallback.sh` defines two functions and exits 0 having done nothing — a silent success; the resolver has the mirror-image guard and the `AGENTS.md` row marks this one `(source)` with nothing enforcing it — `.agent/scripts/manifest_fallback.sh`
- [x] (suggestion) An `exec 9>` failure (unwritable cache dir) is reported as "could not lock … within 300s — another run may be wedged", sending the operator after a phantom process instead of a permissions fault; separate it from the `flock -w` timeout — `.agent/scripts/resolve_repo_checkout.sh:382-387` (deferred: recorded — separating the exec 9> failure from the flock timeout is a resolver message change, not part of the no-layers/ path this pass targets)
- [x] (suggestion) The layer-checkout glob takes a silent first match when a repo name exists under two layers, and `AMBIGUOUS` keys only on differing urls so two manifests with the same url and different `version:` pins silently use `matches[0]` — both are the asymmetry exit 7 exists to refuse (Lens A + Lens B) — `.agent/scripts/resolve_repo_checkout.sh:176-198,266-276` (deferred: recorded — the version-pin asymmetry in AMBIGUOUS is a resolver design question, not a short fix)
- [x] (suggestion) Resolver stderr embeds absolute paths, which `audit-project` funnels into the FAILED reason and the sweep records in a report step 4 requires to be free of host identity and absolute paths — the script's own comment anticipates the funnel but guards only credentials — `.agent/scripts/resolve_repo_checkout.sh:182,186,193,385,448,455` (deferred: recorded — redacting absolute paths from resolver stderr touches the report contract; next round)
- [x] (suggestion) The onboarding probe `gh api repos/$SLUG/contents/AGENTS.md` reads the repo's DEFAULT branch while the audit grades the manifest-PINNED version; one of the 35 overlay repos diverges today (`rqt_marine_radar`, noetic vs jazzy) — add `?ref=<version>` so the gate and the audit read the same tree — `.claude/skills/janitor-sweep/SKILL.md:186-203` (deferred: recorded at the operator's direction — ?ref=<version> changes what the gate reads)
- [x] (suggestion) `LIST_ARGS` is assigned only on the success branch while the rc 3 / rc 5 arms are bare `:`, so the following `"${LIST_ARGS[@]}"` is unbound under `set -u`, and on rc 5 the snippet enumerates anyway although the prose says that state is FAILED — `.claude/skills/janitor-sweep/SKILL.md:106-120`
- [x] (suggestion) `CHUNK_COUNT=$(( (N + 2) / 3 ))` is 0 when N=0, making the modulo a division by zero; rule 4 is meant to stop first but the snippet carries no guard beside the base-10 fix it does document — `.claude/skills/janitor-sweep/SKILL.md:240-249`
- [x] (suggestion) The second `gh api` probe lacks `>/dev/null` (unlike the first), so a successful onboarding probe dumps base64 file JSON into the transcript; and rule 2 does not cover an empty `url:`, which `is_field_url` classifies as dev mode by design — `field_mode.sh`'s header explicitly assigns that non-empty check to this caller — `.claude/skills/janitor-sweep/SKILL.md:174,193`
- [x] (suggestion) `audit-project`'s mode observation compares `$REPO_PATH` against `"$ROOT"/layers/*/src/*`, but when cwd IS a layer checkout `$ROOT` is the project repo's own root, so a genuine layer checkout is observed as mode `clone`; conservative, but wrong, and the caveat text names only layer worktrees — `.claude/skills/audit-project/SKILL.md:56-73` (deferred: recorded — audit-project's mode observation is outside the no-layers/ seam this pass targets)
- [x] (suggestion) Plan staleness: the exit-3 row and its "verified in this worktree" parenthetical are now wrong about shipped behaviour (this worktree's pointer IS derivable, and the resolver resolves against the MAIN root) and contradict the plan's own round-2 section; "Eighteen cases" contradicts the verified 29 stated 44 lines later; exits 5/6 and the resolver's Files-to-Change row omit the round-2 additions — `.agent/work-plans/issue-569/plan.md:129,131-132,275,296`
- [x] (suggestion) Unstated trust model: the sweep now clones remote repos into the workspace and an agent with full host tool access reads their `AGENTS.md`, docs and configs as audit input; neither skill nor script says cloned content is untrusted data, never instructions. Also `.agent/scratchpad/README.md`'s cleanup advice (`find … -mtime +1 -delete`) would gut the janitor clone caches and contradicts the keep-last-20 retention — `.claude/skills/janitor-sweep/SKILL.md` step 3, `.agent/scratchpad/README.md` (deferred: recorded — the trust-model statement and the scratchpad README retention conflict are their own change)

## Implementation
**Status**: complete
**When**: 2026-09-11 14:33 -04:00
**By**: Claude Opus (Claude Code Agent)

**Branch**: feature/issue-569 at `96485fe`
**Addressed**: `## Local Review (Pre-Push)` (round 3, 2026-09-11 14:20 -04:00, branch at `9880b8e`) — 4 must-fix, 18 suggestions
**Commits**: `2624717`, `9e7e93a`, `aaa371c`, `7653e7a`, `da8c528`, `69b155e`, `9a40d91`, `07103bd`, `390d08a`, `e7ff6bb`

Scope per the operator: one short fix pass — the four must-fix (all in the
no-`layers/` path) plus the suggestions adjacent to them. The remaining ten
suggestions are checked-and-deferred with a reason on each, and stay on the
record for the next round.

**Verification**: `make lint` — all 18 hooks Passed. `.agent/scripts/tests/run_script_tests.sh` — "✅ All script tests passed" (26 shell test files, `test_resolve_repo_checkout.sh` now 32/32; 220 pytest cases, `test_workspace_lib.py` 23).

### Actions
- [x] (must-fix) Cached manifest clone reused without an origin check — `.agent/scripts/manifest_fallback.sh:109-123` now reads `git remote get-url origin` and re-clones on any mismatch with the url the current bootstrap pointer derives, mirroring `resolve_repo_checkout.sh:446`. Hermetic test 6h2 repoints the pointer at a second local manifest fixture and asserts both the cached clone's new origin and that the repo resolved is the one the NEW manifest declares.
- [x] (must-fix) Step 2's rotation command dropped `--config-dir` — `.claude/skills/janitor-sweep/SKILL.md` step 2 now passes `"${LIST_ARGS[@]}"`, with a comment naming the consequence of dropping it; step 1 sets `EXTRA_CONFIG`/`LIST_ARGS` before the `if`, enumerates only on the success path, and names the FAILED status in each failure arm.
- [x] (must-fix) Optional-layer exclusion read the path behind the missing symlink — `get_optional_layers()` takes `extra_config_dirs` and searches each dir and its parent (the file sits beside `repos/` in the manifest repo); the workspace's own file still wins. Three tests in `test_workspace_lib.py`. The skill passes step 1's `$EXTRA_CONFIG`.
- [x] (must-fix) `issue-triage` had no rc-5 arm — `.claude/skills/issue-triage/SKILL.md:40-90` snippet branches rc 3 / rc 5 / unexpected, both arms terminal, and the guard list carries the rc-5 case with an accurate remedy (`make setup-all`'s own first step is the clone that just failed); rc 3 keeps `make setup-all`.
- [x] (suggestion) `is_field_url` undocumented — `AGENTS.md` § Field Mode gains a sourced example and the empty-url note; the `field_mode.sh` script row names both entry points and why a url caller must not use `is_field_mode`.
- [x] (suggestion) Exit-vocabulary drift — `AGENTS.md`'s resolver row (exit 5 scaffolding causes, exit 6 `version:`), its `manifest_fallback` row (exit 2/3/5, origin re-clone), and `audit-project`'s caller snippet now match the script headers, which are the vocabulary of record.
- [x] (suggestion) `WORKSPACE_MANIFEST_GIT_BASE` unvalidated — required to be `<scheme>://<host>/<path>` before it reaches `git clone`, refused at exit 5; test 6h3 proves `--upload-pack=evil` never reaches git.
- [x] (suggestion) No sourced-vs-executed guard — `manifest_fallback.sh` exits 2 with a reason when executed, the mirror of the resolver's guard; test 6h4.
- [x] (suggestion) `LIST_ARGS` unbound under `set -u` / rc-5 enumerating anyway — fixed with the step-1 restructure above.
- [x] (suggestion) `CHUNK_COUNT` zero ⇒ division by zero — guarded beside the base-10 fix, pointing at rule 4.
- [x] (suggestion) Second `gh api` probe lacked `>/dev/null`; rule 2 did not cover an empty `url:` — both fixed in step 2/3.
- [x] (suggestion) Plan staleness — `plan.md` exit-3/5/6 rows, "Eighteen cases" → 32 (matching the count stated later, also corrected), and the Files-to-Change rows for the resolver, `manifest_fallback.sh` and `workspace.py` now describe what shipped.

### Deferred (checked — consciously handled, not changed)
- [x] (suggestion) Manifest url/branch derived from the raw path rather than `bootstrap.yaml`'s `git_url:`/`branch:` (deferred: reading the YAML authoritatively is a design change; the derived values were verified to match on this host)
- [x] (suggestion) `manifest_config_dir` succeeds on a config dir holding no `.repos` (deferred: a wording/behaviour change to the EMPTY vocabulary, beyond this pass)
- [x] (suggestion) The fd-8 manifest lock drops at the command substitution (deferred: at the operator's direction — widening the lock's scope changes the caller's contract)
- [x] (suggestion) `timeout(1)` absent silently drops the wall-clock bound (deferred: parity note with the resolver, cosmetic next to this round's false-green fixes)
- [x] (suggestion) `exec 9>` failure reported as a lock timeout (deferred: resolver message change, outside the no-`layers/` seam this pass targets)
- [x] (suggestion) Layer-checkout glob first-match / `AMBIGUOUS` keyed only on url (deferred: resolver design question, not a short fix)
- [x] (suggestion) Resolver stderr embeds absolute paths that reach the report (deferred: touches the report's host-identity contract; next round)
- [x] (suggestion) Onboarding probe reads the default branch, not the pinned `version:` (deferred: at the operator's direction — `?ref=` changes what the gate reads)
- [x] (suggestion) `audit-project`'s mode observation misreads a layer checkout as `clone` (deferred: outside the no-`layers/` seam this pass targets)
- [x] (suggestion) Unstated trust model for cloned content; `.agent/scratchpad/README.md` cleanup advice would gut the caches (deferred: its own change — a trust-model statement plus a retention correction)

### Next
`review-code` (re-review) on the fixes. Not pushed — the host performs pushes.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-11 14:48 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-569 at `4f06a80`
**Mode**: pre-push
**Depth**: Deep (reason: 2691 added / 19 files, instruction-file + new-script + skill triggers)
**Must-fix**: 3 | **Suggestions**: 18
**Round**: 4 | **Ship**: recommended — must-fix is down 14 → 9 → 4 → 3 and the character has changed completely: round 3's four were "the environment this was built for does not work end to end", and Lens A has now verified all three environments working end to end for the first time (including a full environment-3 simulation from a bare root). The three remaining are one internal contradiction about which tree check 1 grades, one reproduced false RED on a non-primary manifest layout, and one missing report state — each a precise one-line fix with an obvious correction, none a design question. Address them in a short fix pass and ship; do not spend another full round.

Specialists: Static Analysis (`make lint` 18/18 hooks Passed; `run_script_tests.sh` green — 25 shell suites, 220 pytest; resolver 32/32, field-mode 38/38, workspace-lib 23/23), Governance, Plan Drift, Claude Adversarial Lens A + Lens B. Copilot and local model not run (not opted in).

All four round-3 must-fix independently verified closed in the code: the cached-manifest origin check (`manifest_fallback.sh:143-147`), step 2's restored `--config-dir` (`janitor-sweep/SKILL.md:168`), `get_optional_layers(..., extra_config_dirs)` searching dir and parent, and `issue-triage`'s rc-5 arm. The central round-3 doubt is discharged: Lens A exercised environment 3 for real (bare root, real bootstrap pointer, manifest served from a local bare clone) and got 35 repos, `{site}` from the cloned `optional_layers.txt`, and mode `clone`. This round's must-fix are the seam between the sweep and the sub-skills it delegates to, plus one manifest layout the fallback's early return does not recognise. Must-fix 3 is cross-confirmed by Governance and Lens A independently; must-fix 2 was reproduced in a sandbox.

### Findings
- [x] (must-fix) Check 1 is the only check not anchored at `$ROOT`: step 1 mandates `$ROOT` for "**every** script and file the sweep touches" and `:304-306` lists check 1's inputs as `$ROOT/...` paths, but it delegates to `/audit-workspace`, which addresses everything by bare relative path (`audit-workspace/SKILL.md:31,55,64,73,80`) and audits **cwd** — so a sweep run from a worktree (the skill's own stated normal case) grades that branch's governance docs for check 1 and the main root for checks 2-4, and reports the mixture as one workspace state — `.claude/skills/janitor-sweep/SKILL.md:295,304-310`
- [x] (must-fix) The "does this workspace have its own manifest?" early return is narrower than `list_overlay_repos.py`'s own search path — it returns only on `configs/manifest/repos`, while `get_overlay_repos` also reads `configs/*.repos` (`lib/workspace.py:89-92`) — so on that supported layout the helper attempts an unneeded network clone and `resolve_repo_checkout.sh:225` turns its rc 5 into `exit 5` BEFORE the manifest on disk is ever read; reproduced in a sandbox (same tree resolves fine with the bootstrap pointer removed, hard-fails at exit 5 with it present) — a false RED over a workable state — `.agent/scripts/manifest_fallback.sh:64`
- [x] (must-fix) A manifest that could not be REFRESHED is used silently as far as the report is concerned: the helper warns on stderr and returns 0 with the cached clone, both callers capture stdout only, and the status contract has no state for it — so a sweep can report `4 of 4 completed` over a repo list enumerated from a manifest it could not refresh, the report-level false green the contract exists to prevent; the origin re-clone guard also keys on url only, not branch (Governance + Lens A) — `.claude/skills/janitor-sweep/SKILL.md:105-118`, `.claude/skills/issue-triage/SKILL.md:53-62`, `.agent/scripts/manifest_fallback.sh:150-157`
- [x] (suggestion) The `$ROOT` defect deferred in round 3 has a second site with a false rationale attached: the comment says "in a layer worktree neither `.agent/scripts/` nor `layers/` sits beside you", but step 1's own caveat (`:48-50`) says `$ROOT` resolves to the PROJECT repo's root there, so the `$ROOT` paths are exactly as absent as the relative ones they replaced — the deferral should cover both sites or neither (fix-every-site rule) — `.claude/skills/audit-project/SKILL.md:180-183`
- [x] (suggestion) The bare enumeration command still precedes the fallback snippet, so on a no-`layers/` host it prints `[]` at exit 0 and the guard maps that to the wrong remedy; `janitor-sweep` was restructured so enumeration can only happen on the fallback's success path, `issue-triage` was not — `.claude/skills/issue-triage/SKILL.md:41` vs `:74-78`
- [x] (suggestion) A legal nested vcstool manifest key (`tools/foo`) is reported as a USAGE error blaming the caller, although `janitor-sweep` feeds names straight out of the manifest; name it as a manifest-shape limitation instead. Latent — no manifest here uses one today — `.agent/scripts/resolve_repo_checkout.sh:109` (deferred: latent, no manifest uses a nested key today — recorded for the re-review, out of scope for this post-publish must-fix pass)
- [x] (suggestion) `--repos` has no rule for a name that is in no manifest, so `/janitor-sweep --repos typo_name` lands on rule 4 and reports `SKIPPED(no eligible repos)` for a repo the operator explicitly asked for; `issue-triage`, edited in the same PR, states the analogous rule — `.claude/skills/janitor-sweep/SKILL.md:14-20` (deferred: recorded for the re-review — a --repos rule change is behaviour, not one of the three approved fixes)
- [x] (suggestion) `RESOLVE_LOCK_TIMEOUT` reaches `flock -w` unvalidated: a non-numeric value makes flock exit 64 immediately and the script prints "could not lock ... within abcs — another run may be wedged", sending the operator after a phantom process — one fix with the deferred `exec 9>` misdirection — `.agent/scripts/resolve_repo_checkout.sh:381-386` (deferred: recorded — bundled with the deferred `exec 9>` misdirection item, as the finding itself notes)
- [x] (suggestion) The manifest helper's three url messages are unredacted while the resolver has `redact_url` for exactly this funnel-into-the-report reason, and the accepted `WORKSPACE_MANIFEST_GIT_BASE` form admits `https://user:pw@host` — `.agent/scripts/manifest_fallback.sh:145,161,168` (deferred: recorded for the re-review — redaction is its own pass over all three messages)
- [x] (suggestion) Two snippets use `continue` / `return` outside any loop or function and are bash errors if run verbatim; both are illustrative, but every other snippet in the file is copy-pasteable — `.claude/skills/janitor-sweep/SKILL.md:188,282` (deferred: recorded — both snippets are illustrative, and the file states that)
- [x] (suggestion) The `manifest_fallback` row says exit 5 covers "the clone **or refresh** failed"; the code returns 0 on a refresh failure with a stale-cache warning, and the script's own header gets this right — AGENTS.md is the single outlier, and it is wrong in the false-green direction (the doc side of must-fix 3) — `AGENTS.md:585`
- [x] (suggestion) The inline exit-5 gloss omits the fourth cause, a layer checkout that exists and cannot be read (`resolve_repo_checkout.sh:181-184,192-195`) — the one case where "fall back to a clone" would be wrong; AGENTS.md:584 has it — `.claude/skills/audit-project/SKILL.md:88-89` (deferred: recorded for the re-review)
- [x] (suggestion) The usage block and function list still show the pre-change signatures (`get_overlay_repos(include_underlay=False)`, no `--config-dir`); weak bar, since that README is already non-exhaustive — `.agent/scripts/README.md:562-570,611-619` (deferred: recorded for the re-review — the reviewer graded the bar weak and the README non-exhaustive)
- [x] (suggestion) The sweep's read-only workhorses (`resolve_repo_checkout.sh`, `list_overlay_repos.py`) are not allowlisted while peers are, so an unattended run prompts per repo — correctly out of scope while the trigger is deferred, flagged as an input to that decision — `.claude/settings.json` (deferred: recorded — an input to the deferred trigger decision, as the finding says, not a change to make now)
- [x] (suggestion) The resolver's "the lock is released at exit, so the caller reads `$TARGET` unlocked — concurrent audits of the SAME repo are not safe" caveat is carried to neither calling skill; the visible result of losing that race is a PHANTOM finding (`MISSING: no root AGENTS.md`) indistinguishable from a real one, and several agents run at once on this host — `.claude/skills/janitor-sweep/SKILL.md:145-154`, `.claude/skills/audit-project/SKILL.md:56-95` (deferred: recorded — lock scope, left recorded by operator direction)
- [x] (suggestion) Retention caps the 20 small reports and never prunes the clone caches, which are the output that actually consumes disk (one shallow working tree per repo ever resolved, including repos later dropped from the manifest) — on a host whose dev disk has already hit 100% — `.claude/skills/janitor-sweep/SKILL.md:145-154`
- [x] (suggestion) Re-grade the deferred trust-model item before merge: ADR-0019 §3 says a phase that fetches its own text "must carry the fence in its own SKILL.md", this sweep fetches its own text by design, and clones land INSIDE the agent's own tree (`$MAIN_ROOT/.agent/scratchpad/janitor-repos/<repo>`) where instruction files auto-load — a stronger exposure than recorded — `.claude/skills/janitor-sweep/SKILL.md`, `.agent/scripts/resolve_repo_checkout.sh:358` (deferred: recorded — trust model, left recorded by operator direction for a merge-time re-grade)
- [x] (suggestion) The plan under-states the AGENTS.md Ask-First edits: the row names only the resolver, while the diff makes FOUR changes (resolver row, new `manifest_fallback.sh` row, rewritten `field_mode.sh` row, and a 17-line § Field Mode prose block added in round 3 carrying a normative empty-url sentence). Open Questions tells the operator to re-confirm "two details" — the § Field Mode prose is a third, later addition and appears nowhere in the plan, so the re-confirmation checklist is incomplete — `.agent/work-plans/issue-569/plan.md:299,305,355-366` (deferred: recorded for the re-review — the operator re-confirmation list is a checkpoint item, not an agent edit)
- [x] (suggestion) No round-3 plan-revision narrative, unlike rounds 1 and 2: round 3's changes exist only as "Round 3 —" clauses inside three Files-to-Change rows, so a future agent reading top-to-bottom sees two review rounds where the timeline records three — `.agent/work-plans/issue-569/plan.md:37,67` (deferred: a round-4 narrative was written instead; round 3 stays recorded in the Files-to-Change rows)
- [x] (suggestion) `## Documentation & Instruction Impact` omits `.claude/skills/issue-triage/SKILL.md`, which the diff rewrites substantially (+60/−2, a chained skill's failure vocabulary) — it is in Files-to-Change, but Doc-Impact is the section a reviewer reads for "which instruction/procedure files does this PR touch" — `.agent/work-plans/issue-569/plan.md:344-347` (deferred: recorded for the re-review)
- [x] (suggestion) Still open from round 3 and shipping with this PR: `.agent/scratchpad/README.md`'s `find .agent/scratchpad -type f -mtime +1 -delete` advice would gut the janitor caches and contradicts the keep-last-20 retention — the caches self-heal (both origin checks re-clone on a broken tree), the reports do not — `.agent/scratchpad/README.md:82-83`

### Operator items (not agent-fixable)
- The **Ask-First re-confirmation** for `AGENTS.md`, the three adapters and `principles_review_guide.md` is still outstanding, and is now known to cover four AGENTS.md changes rather than the one the plan names.
- The **deferred publishing-and-trigger decision** still has no issue number filed; the skill, the plan and `skill_workflows.md` all cross-reference it by prose only.

### Next
`address-findings` on the three must-fix (short fix pass), then push. Not pushed — the host performs pushes.

## Implementation
**Status**: complete
**When**: 2026-09-11 14:59 -04:00
**By**: Claude Opus (Claude Opus 5, 1M context)

**Branch**: feature/issue-569 at `09e358a`
**PR**: #625
**Addressed**: `## Local Review (Pre-Push)` of 2026-09-11 14:48 -04:00 (round 4, `Ship: recommended`, branch at `4f06a80`)
**Commits**: `8e1f779`, `1cb2113`, `0f9345c`, `0e501ff`, `aa9f72f`, `09e358a`

The PR is already open and the operator approved publishing with the three
must-fix landed afterwards. This pass is those three plus the adjacent
suggestions; the remaining suggestions are deferred **checked with a reason**,
recorded for the re-review rather than dropped.

### Actions

**Must-fix**

- [x] Check 1 graded a different tree from checks 2-4 — `.claude/skills/janitor-sweep/SKILL.md:304,312-318` (`0f9345c`).
  `/audit-workspace` takes no root and addresses everything by bare relative
  path, so it audits **cwd**: a sweep from a worktree graded that branch's
  governance docs for check 1 and the main root for the rest. The step now says
  `cd "$ROOT"` for that check and stay there, and says why.
- [x] A manifest on disk was bypassed for a clone — `.agent/scripts/manifest_fallback.sh:63-82` (`8e1f779`).
  The early return recognised only `configs/manifest/repos`; `get_overlay_repos`
  also reads `configs/*.repos`, so on that layout the helper attempted a network
  clone whose failure `resolve_repo_checkout.sh` turned into exit 5 before the
  manifest on disk was ever opened. Both layouts now return early
  (`underlay.repos` excluded — that search ignores it, so a workspace holding
  only it does still need the fallback). Hermetic case
  `test_resolve_repo_checkout.sh` 6h5, verified to FAIL against the pre-change
  helper (rc 5) and pass after.
- [x] A failed refresh was rc 0 with a stderr warning — `.agent/scripts/manifest_fallback.sh:157-176`, `AGENTS.md:585`, both calling skills (`1cb2113`).
  Chose the distinct non-zero exit over a documented stale state: the four-state
  contract grades a check, and "ran over an unverifiable manifest" is not a
  fourth state of that check — it is `FAILED`. rc **6**, mapped to
  `FAILED(manifest refresh: <reason>)` by `janitor-sweep` and `issue-triage` and
  to exit 5 by the resolver ("a clone or refresh failed"). `AGENTS.md`'s exit-5
  row described behaviour the code did not have; it now says what the code does,
  exit 6 included. On the finding's parenthetical (the origin guard keys on url,
  not branch): no code change — the refresh hard-resets to the current pointer's
  `$branch` on every run, so a clone cached at another branch is corrected, and
  that is now stated at the guard instead of left to be re-derived. Hermetic case
  6h4b: manifest cached, origin removed → exit 5 with the reason, never a silent
  stale read.

**Suggestions taken**

- [x] Second `$ROOT` site with a false rationale — `.claude/skills/audit-project/SKILL.md:172-177` (`0e501ff`). Fix-every-site: the comment claimed `$ROOT` solved the layer-worktree case that step 1's own caveat says it does not.
- [x] `issue-triage`'s bare enumeration ahead of the fallback — `.claude/skills/issue-triage/SKILL.md:37-53` (`0e501ff`). One enumeration site, on the fallback's success path, as `janitor-sweep` already had.
- [x] `AGENTS.md` exit-5 row wording — `AGENTS.md:585` (`1cb2113`, with must-fix 3; it is the doc side of the same defect).
- [x] Retention says nothing about the clone caches — `.claude/skills/janitor-sweep/SKILL.md:161-174,450-455` (`0e501ff`). They, not the 20 small reports, are what fills a disk; the footer now names both directories with their sizes and says why they are left uncapped.
- [x] `.agent/scratchpad/README.md`'s age sweep would gut those caches — `.agent/scratchpad/README.md:81-97` (`09e358a`). Taken as adjacent to the retention item, and it ships with this PR: scoped to `-maxdepth 1`, with the managed subdirectories and their real retention named.

**Deferred — recorded, not actioned** (each checked with its reason in the round-4 entry): the nested-manifest-key usage message, the `--repos` unknown-name rule, `RESOLVE_LOCK_TIMEOUT` validation, url redaction in the helper's messages, the two illustrative snippets, the `audit-project` exit-5 gloss, `.agent/scripts/README.md` signatures, the `.claude/settings.json` allowlist, the **lock-scope** caveat and the **trust-model** re-grade (both left recorded by operator direction), the plan's Ask-First re-confirmation list, the missing round-3 narrative (a round-4 one was written instead), and the Doc-Impact omission.

**Plan sync**: `plan.md` carries a round-4 revision section and Round-4 clauses on the three affected Files-to-Change rows (`aa9f72f`).

**Checks**: `make lint` — all hooks Passed. `.agent/scripts/tests/run_script_tests.sh` — `✅ All script tests passed.` (34 cases in `test_resolve_repo_checkout.sh`, 220 pytest).

**Not done**: nothing pushed — the host pushes.
