---
issue: 530
---

# Issue #530 — v1 `/wrap-up-deployment` skill — phase [3] orchestrator (deployment-mode gap)

## Issue Review
**Status**: partial
**When**: 2026-06-20 00:00 +00:00
**By**: Claude Code Agent (Claude Opus 4.6)

**Issue**: #530
**Comment**: not posted — `gh` CLI lacks auth in this container environment (no GH_TOKEN, no `~/.config/gh/`). Review content saved to `.agent/work-plans/issue-530/review-comment.md` for the host to post.
**Scope verdict**: well-scoped

### Review Summary

Issue proposes creating `.claude/skills/wrap-up-deployment/SKILL.md` + AGENTS.md pointer update, codifying a 4-times-validated manual deployment wrap-up workflow. Single PR, clear acceptance criteria, no blocking dependencies.

**Principle findings** (Action needed):
- A change includes its consequences: Non-Claude adapter skill lists (`.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md`) and `make generate-skills` are not called out in acceptance criteria but are required by the consequences map.

**ADR findings** (Action needed):
- ADR-0006: Non-Claude adapter skill lists need updating alongside AGENTS.md.
- ADR-0013: SKILL.md must specify which `progress.md` entry type the skill writes (or confirm it does not write one). If a new type is needed, an ADR addendum is required before implementation.

**Watch items**:
- ADR-0003: SKILL.md must not embed project-specific content; all project details must come from `.agents/deployment.yaml`.
- ADR-0014: ADR Status is "Proposed" — updating to "Accepted" after this skill ships is a follow-up.

### Actions
- [ ] Update non-Claude adapter skill lists when adding the skill (`.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md`)
- [ ] Run `make generate-skills` after adding `.claude/skills/wrap-up-deployment/`
- [x] **RESOLVED** (operator, 2026-06-20): skill writes **NO `progress.md` entry**. `/wrap-up-deployment` is a deployment-lifecycle skill (writes the deployment dev log + GitHub issue), mirroring `/start-deployment`; `progress.md`/ADR-0013 govern the per-issue *development* pipeline only. No ADR-0013 addendum needed.
- [ ] Keep project-specific config in `.agents/deployment.yaml`, not in SKILL.md (ADR-0003 three-tier split)
- [x] Posted by host (gh auth available on host) — #530 issue comment.
- [ ] After merge: consider updating ADR-0014 Status from "Proposed" to "Accepted"

## Plan Authored
**Status**: complete
**When**: 2026-06-20 10:30 +00:00
**By**: Claude Code Agent (Claude Sonnet 4.6)

**Plan**: `.agent/work-plans/issue-530/plan.md` at `7cc1133`
**Branch**: feature/issue-530 at `7cc1133`
**Phases**: single

### Open questions
- [ ] Should `.agents/deployment.yaml` gain a `wrap_up:` section for project-specific steps (bag extraction commands, analysis scripts)? V1 defers bag extraction to operator/debrief (#435); out of scope unless a project has concrete needs.

## Plan Review
**Status**: complete
**When**: 2026-06-20 13:34 +00:00
**By**: Claude Code Agent (Claude Opus 4.6) (in-context — author self-review)

**Plan**: `.agent/work-plans/issue-530/plan.md` at `7cc1133`
**PR**: PR-less (--issue mode; dispatched sub-agent review)
**Verdict**: approve-with-suggestions

### Findings
- [ ] (suggestion) Make `issue_sync`/`dev_push` absence graceful in SKILL.md — dev-side optional per `deployment_config.yaml:79-81`; mirror `start-deployment` — `plan.md:40`
- [ ] (suggestion) Open the ADR-0014 Status-flip ("Proposed"→"Accepted") follow-up issue rather than leaving it implicit — `plan.md:102`
- [ ] (suggestion) State bag-extraction/analysis as operator-driven / out-of-scope for v1 in SKILL.md so adopters see the deferral — `plan.md:108-112`

Plan is well-scoped, addresses all review-issue findings, and correctly resolves the ADR-0003 (project-agnostic config) and ADR-0013 (no per-issue progress entry) risks. Config keys referenced (`log_dir`, `issue_sync.dev_push`) verified against the template. Ready for implementation; the three items are refinements, not blockers.

## Implementation
**Status**: complete
**When**: 2026-06-20 (sub-agent)
**By**: Claude Code Agent (Claude Sonnet 4.6)

Created `.claude/skills/wrap-up-deployment/SKILL.md` — a 9-step wrap-up orchestrator mirroring `start-deployment/SKILL.md` in structure and tone. The skill adds an operator-correction interview (step 5, one question at a time before consolidating logs) and separates field-code reconciliation into two distinct mechanisms: SHA-preserving `git merge` for the primary deployment repo (step 7a) and `/import-field-changes` for other repos (step 7b). Bag extraction is explicitly deferred to the project debrief checklist (#435). `issue_sync`/`dev_push` absence is graceful (dev-side optional, mirroring start-deployment).

Also updated AGENTS.md "Deployment mode" section (replaced #495 placeholder with skill pointer + #496 recovery note), all three non-Claude adapter skill lists, and plan.md to reflect the 9-step implemented flow. `make generate-skills` ran successfully (generated `.claude/skills/make_*/` files which are gitignored).

**Host follow-up**: ADR-0014 Status "Proposed"→"Accepted" flip requires `gh` auth to post — post once the PR merges to main.

**Files changed**:
- `.claude/skills/wrap-up-deployment/SKILL.md` — created (main deliverable)
- `AGENTS.md` — deployment mode section updated
- `.github/copilot-instructions.md` — `wrap-up-deployment` added to skill list
- `.agent/instructions/gemini-cli.instructions.md` — `wrap-up-deployment` added to skill list
- `.agent/AGENT_ONBOARDING.md` — `wrap-up-deployment` added to skill list
- `.agent/work-plans/issue-530/plan.md` — synced to reflect 9-step flow

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-20 14:14 +00:00
**By**: Claude Code Agent (Claude Opus 4.8)
**Verdict**: changes-requested

**Branch**: feature/issue-530 at `6b09e11`
**Mode**: pre-push
**Depth**: Deep (reason: 455-line new SKILL.md / 743 lines >200 + governance-trigger files)
**Must-fix**: 6 | **Suggestions**: 5

### Findings
- [ ] (must-fix) Step-8/7a git ops don't specify the project-repo worktree (gitcloud/origin remotes live there, not workspace root); add `git -C` — `.claude/skills/wrap-up-deployment/SKILL.md:292,355`
- [ ] (must-fix) gitcloud push assumes unconditional fast-forward; no `--ff-only`/re-fetch guard for a gitcloud that advances during the long wrap-up window — `.claude/skills/wrap-up-deployment/SKILL.md:356`
- [ ] (must-fix) Draft PR cannot be merged: `--draft` create then immediate `gh pr merge --merge`; need `gh pr ready` or drop `--draft` — `.claude/skills/wrap-up-deployment/SKILL.md:330,345`
- [ ] (must-fix) Operator corrections + dev-log sections are staged/written but step 8 has no `git commit` before `gh pr create` → corrections never enter PR, lost on worktree removal — `.claude/skills/wrap-up-deployment/SKILL.md:225,320`
- [ ] (must-fix) No idempotency/interruption recovery across irreversible step 8 (PR-merge closes issue; re-run is stranded, worktree orphaned) — `.claude/skills/wrap-up-deployment/SKILL.md:320`
- [ ] (must-fix) Consequence miss: knowledge doc still calls `/wrap-up-deployment` a future follow-up in 3 places — `.agent/knowledge/deployment_mode.md:25,239,303`
- [ ] (suggestion) Treat `issue_sync.dev_push == "TODO"` as absent so step 8 doesn't run `TODO` as a shell command — `.claude/skills/wrap-up-deployment/SKILL.md:362`
- [ ] (suggestion) Re-fetch gitcloud before 7a merge, handle fetch failure, document `git merge --abort` safe-out — `.claude/skills/wrap-up-deployment/SKILL.md:136,292`
- [ ] (suggestion) Verify gitcloud push succeeded before removing the worktree (removal destroys retry context) — `.claude/skills/wrap-up-deployment/SKILL.md:355`
- [ ] (suggestion) `rca` label may not exist; create-or-skip when filing RCA issues — `.claude/skills/wrap-up-deployment/SKILL.md:404`
- [ ] (suggestion) `ls` log glob errors on no-match; add "if none, note and continue" guard — `.claude/skills/wrap-up-deployment/SKILL.md:145`

## Implementation
**Status**: complete
**When**: 2026-06-20
**By**: Claude Code Agent (Claude Sonnet 4.6)

**Commit**: `7230154`
**Summary**: Addressed review-code round 1 — all 6 must-fix findings and all 5 suggestions applied to `.claude/skills/wrap-up-deployment/SKILL.md` and `.agent/knowledge/deployment_mode.md`.

**Changes by finding:**
- **#1 (git -C project-repo worktree)**: Added `git -C "<project-repo-worktree-path>"` to all git ops in steps 4, 6a, 6d, 7a, and 8. Path derivation note added in step 4 and step 7a.
- **#2 (gitcloud push ff-guard)**: Step 8 now re-fetches gitcloud + origin after PR merge, runs `merge-base --is-ancestor` to verify ff before pushing, stops on divergence with actionable message.
- **#3 (draft PR → mergeable)**: Removed `--draft` flag from `gh pr create` in step 8.
- **#4 (commit before PR create)**: Step 6a no longer says "do NOT commit yet"; new step 6d commits the dev log and all corrected field logs before step 8.
- **#5 (idempotency/recovery)**: Step 8 opens with a "if re-running after interruption" block that checks PR state and issue state before repeating irreversible actions.
- **#6 (deployment_mode.md)**: Three stale "future follow-up" references updated — lifecycle tooling description (line 25), wrap-up checklist note (line 239), and "What's NOT in v1" section (removed `/wrap-up-deployment` line from list).
- **#7 (TODO guard)**: `issue_sync.dev_push` check now guards `== "TODO"` as absent.
- **#8 (re-fetch + merge --abort)**: Step 7a now fetches gitcloud before merge, stops on fetch failure, documents `merge --abort` safe-out.
- **#9 (verify push before remove)**: Step 8 notes "verify the push succeeded before proceeding to worktree removal."
- **#10 (rca label create-or-skip)**: Step 9 adds `gh label create rca … 2>/dev/null || true` before filing.
- **#11 (glob no-match guard)**: Step 4 adds "if the glob matches no files, note and continue — do not abort."

**Files changed**:
- `.claude/skills/wrap-up-deployment/SKILL.md` — steps 4, 6a, 6d (new), 7a, 8, 9 updated
- `.agent/knowledge/deployment_mode.md` — 3 stale future-references updated

**Next step**: re-review (`/review-code`) before opening the PR.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-20 14:40 +00:00
**By**: Claude Code Agent (Claude Opus 4.8)
**Verdict**: changes-requested

**Branch**: feature/issue-530 at `7230154`
**Mode**: pre-push
**Depth**: Deep (reason: 562-line new SKILL.md + governance-trigger files AGENTS.md/deployment_mode.md/ADR-0006 adapters)
**Must-fix**: 3 | **Suggestions**: 6

Round 2. All 11 round-1 findings verified applied and correct (not re-reported).
3 new must-fix items, all from SKILL.md assumptions about how `/start-deployment`'s
worktree is actually created, plus a `git add` rename-staging bug. Both Lens B
must-fix claims verified against `worktree_create.sh` (760/763/942) and
`start-deployment/SKILL.md` (--type layer, no --plan-file).

### Findings
- [ ] (must-fix) Project-repo worktree path template wrong in all 3 segments — should be `layers/worktrees/issue-<REPO_SLUG>-<N>/<layer>_ws/src/<packages[0]>` (not `.workspace-worktrees/issue-<N>/layers/main/...`) — `.claude/skills/wrap-up-deployment/SKILL.md:136,325`
- [ ] (must-fix) Deployment branch never pushed before `gh pr create`; start-deployment creates worktree without --plan-file (local-only branch), repo has two remotes so gh can't auto-pick — add explicit `git push -u origin <branch>` — `.claude/skills/wrap-up-deployment/SKILL.md:395`
- [ ] (must-fix) Renamed mislabeled log's old-path deletion unstaged: shell-glob `git add` only sees existing files — use `git mv` or `git add -A <log_dir>/<YYYY>/` — `.claude/skills/wrap-up-deployment/SKILL.md:234,299`
- [ ] (suggestion) Editing field logs in place (6a) before merging gitcloud (7a) guarantees a merge conflict on every corrected log; merge first or document expected-conflict resolution — `.claude/skills/wrap-up-deployment/SKILL.md:216,343`
- [ ] (suggestion) Worktree removed (step 8) before RCA filing (step 9) breaks `gh` origin auto-detection; file RCA before removal or pass `-R <owner/repo>` — `.claude/skills/wrap-up-deployment/SKILL.md:471,477`
- [ ] (suggestion) gitcloud reconcile doesn't confirm PR actually merged before re-fetch/ancestor-check; poll `gh pr view --json state` for MERGED first — `.claude/skills/wrap-up-deployment/SKILL.md:423`
- [ ] (suggestion) Broken reference link: `import-field-changes` is a skill, not `.agent/scripts/`; point to `.claude/skills/import-field-changes/SKILL.md` — `.claude/skills/wrap-up-deployment/SKILL.md:558`
- [ ] (suggestion) Surface `/import-field-changes`'s own config prereq (`.agent/project_config.yaml:field_remote`, different file than deployment.yaml) — `.claude/skills/wrap-up-deployment/SKILL.md:363`
- [ ] (suggestion) deployment_mode.md phase [3] lists bag-DB extraction/analyses as wrap-up work, contradicting SKILL's explicit out-of-scope (#435); add a parenthetical — `.agent/knowledge/deployment_mode.md:~141`

## Implementation
**Status**: complete
**When**: 2026-06-20
**By**: Claude Code Agent (Claude Sonnet 4.6)

Addressed review-code round 2: 3 must-fix + 6 suggestions. Commit: `0bb34a6`.

**Must-fix**:
1. Fixed wrong project-repo worktree path template in both occurrences — changed `.workspace-worktrees/issue-<N>/layers/main/...` to `layers/worktrees/issue-<REPO_SLUG>-<N>/...` (the correct layer-worktree path from `worktree_create.sh`).
2. Added explicit `git push -u origin "<branch-name>"` before `gh pr create` in step 9.
3. Changed `git add <glob>` in step 7a to `git mv` for renames + `git add -A <log_dir>/<YYYY>/` to correctly stage old-path deletions.

**Suggestions (all applied)**:
4. Reordered: SHA-preserving gitcloud merge (step 5) now precedes operator interview (step 6) and log corrections (step 7) — avoids merge conflicts on every corrected log.
5. Moved RCA issue filing (step 10) before worktree removal (which is now the last action inside step 10) — `gh` origin auto-detection intact during RCA filing.
6. Added `gh pr view <PR_number> --json state --jq '.state'` confirm-MERGED poll before gitcloud reconcile in step 9.
7. Fixed broken reference link: `.agent/scripts/import-field-changes` → `.claude/skills/import-field-changes/SKILL.md`.
8. Added `/import-field-changes` config prereq note (reads `field_remote` from `.agent/project_config.yaml`) at step 8 invocation.
9. Added parenthetical to `deployment_mode.md` phase [3] noting bag extraction/analyses are the debrief skill's job (#435), not v1 wrap-up.

**Files changed**: `.claude/skills/wrap-up-deployment/SKILL.md`, `.agent/knowledge/deployment_mode.md`

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-20 15:05 +00:00
**By**: Claude Code Agent (Claude Opus 4.8)
**Verdict**: changes-requested

**Branch**: feature/issue-530 at `0bb34a6`
**Mode**: pre-push
**Depth**: Deep (reason: 598-line new SKILL.md orchestrating irreversible git/gh ops + governance-trigger files AGENTS.md/deployment_mode.md/ADR-0006 adapters)
**Must-fix**: 4 | **Suggestions**: 3

Round 3. All 11 round-1 + 9 round-2 findings verified applied (not re-reported).
4 new must-fix from reading the final flow end-to-end (2 Lens B, 2 Lens A); both
Lens B merge/CI claims verified against git/pre-commit stage semantics and the
`gh pr merge --auto` queue behavior. Static analysis: `.md` only, no linter profile.

### Findings
- [ ] (must-fix) Step-5 `git merge gitcloud/<default_branch>` omits `-c user.name/-c user.email`; merge commit lands under wrong/absent author silently (identity hook is `pre-commit` stage, doesn't fire on merge commits) — `.claude/skills/wrap-up-deployment/SKILL.md:206`
- [ ] (must-fix) `gh pr merge --merge` (no `--auto`) + "wait and re-poll" hangs forever on required CI checks — the command fails immediately, nothing drives the merge; use `--auto` or re-invoke after checks — `.claude/skills/wrap-up-deployment/SKILL.md:437,452`
- [ ] (must-fix) `<branch-name>` is an undefined placeholder (used only at the push), never derived like every other identifier — define as `feature/issue-<N>` or read worktree HEAD — `.claude/skills/wrap-up-deployment/SKILL.md:413`
- [ ] (must-fix) Dev-log/field-log paths bare-relative: `dlog.sh`/`ls` resolve against CWD, `git -C` staging against worktree root; run from workspace root → dev log written but never committed, lost on worktree removal — anchor to `<project-repo-worktree-path>` — `.claude/skills/wrap-up-deployment/SKILL.md:152,169-170`
- [ ] (suggestion) `.agent/project_config.yaml` write uses CWD-dependent bare path; anchor to `<workspace_root>/.agent/...` or note run-from-root — `.claude/skills/wrap-up-deployment/SKILL.md:382`
- [ ] (suggestion) Step-7d hardcodes `Claude Code Agent` identity; skill is advertised to non-Claude runtimes (gemini/copilot adapter lists) → wrong author lands silently; use `$AGENT_NAME`/`$AGENT_EMAIL` or placeholder — `.claude/skills/wrap-up-deployment/SKILL.md:363-366`
- [ ] (suggestion) "squash → force-push" rationale imprecise: a squash would fail the ancestor check and block the push, not force it; reword — `.claude/skills/wrap-up-deployment/SKILL.md:568`
