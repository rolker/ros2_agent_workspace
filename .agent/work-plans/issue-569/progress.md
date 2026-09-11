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
