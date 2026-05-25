---
issue: 488
---

# Issue #488 — Add make merge-pr: merge + clean up worktree/branches + make sync (adapt from agent_workspace)

## Plan Authored
**Status**: complete
**When**: 2026-05-25 18:57 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Plan**: `.agent/work-plans/issue-488/plan.md` at `4634f0b`
**PR**: https://github.com/rolker/ros2_agent_workspace/pull/494 (`[PLAN]` prefix)
**Phases**: single

### Open questions
- [ ] Project-repo PR resolution UX: default workspace + worktree-narrowed scan of layer repos with `--repo-slug`/`REPO=` override (proposed), vs. requiring `REPO=` explicitly for any non-workspace PR (simpler, less magic)?
- [ ] Should `merge-pr` write a terminal `progress.md` entry, or stay mechanical (lifecycle entries owned by #481-C / the `## Implementation` marker)? Proposed: stay mechanical — confirm the boundary.

## Plan Review
**Status**: complete
**When**: 2026-05-25 19:14 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context)) — fresh-context sub-agent (dispatched by plan author; independent context)

**Plan**: `.agent/work-plans/issue-488/plan.md` at `4634f0b`
**PR**: https://github.com/rolker/ros2_agent_workspace/pull/494
**Verdict**: approve-with-suggestions

### Findings
- [x] (must-fix) Worktree-narrowing is a chicken-and-egg bug: it greps `worktree_list.sh` for `issue-<slug>-<N>` using N=**PR** number, but worktree dirs are named by **issue** number, and the issue number isn't known until after the repo is resolved + `gh pr view`. The narrowing can't work for layer PRs. — `plan.md:26-28`
- [x] (must-fix) Resolution also breaks if the issue's worktree was already removed. Both dissolve by **requiring `REPO=`/`--repo-slug` for non-workspace PRs** (resolves Open Question 1), or remote-scanning candidates. — `plan.md:26-30`
- [x] (must-fix) `/make_merge-pr` would run `make merge-pr` with no `PR=`: `generate_make_skills.sh` only parameterizes `revert-feature|agent-run|agent-shell`; others get a bare `make <target>`. Add `merge-pr` to the parameterized case (`PR=$ARGUMENTS`) + update `test_generate_make_skills.sh`. The plan treated generate-skills as a no-op. — `generate_make_skills.sh`
- [x] (suggestion) Move the field-mode guard **before** the first `gh pr view` (resolve repo locally by origin host first), else a field repo fails with a confusing "PR not found" before the clear guard message. — `plan.md:31-33`
- [x] (suggestion) `cd "$ROOT_DIR"` in the script's own shell before `worktree_remove` AND before `make sync` (worktree_remove refuses if cwd is inside the target; sync must run from root). — `plan.md:37`
- [x] (suggestion) `worktree_remove.sh` does NOT delete layer branches (only prunes refs + prints a note), so `merge_pr.sh` owns local+remote branch deletion in the **project** repo. Step 7 is load-bearing — implement it pointed at `layers/main/.../src/<slug>`. — `plan.md:40-41`
- [x] (suggestion) Note upstream deps intentionally NOT carried over: `_issue_helpers.sh` and `update_roadmap.sh` don't exist here — don't copy the `source`/call lines. — `plan.md`
- [x] (suggestion) Human-control note: `/make_merge-pr` exposes merge+branch-delete as one-shot, but the generator's default template sets `disable-model-invocation: true` (human-only). Record this posture + preserve it in the parameterized case. — `plan.md:72`

### Notes
- Reviewer's recommended resolution: answer Open Question 1 with "require `REPO=`/`--repo-slug` for non-workspace PRs" — dissolves the two must-fix resolution findings with less magic. Flow shape (resolve → field-guard → CI-wait → merge → remove worktree → delete branches → make sync) and the adapt-don't-port divergences are otherwise sound.
- **Resolution (2026-05-25, with user)**: went further than "require `REPO=`" — reframed to **worktree/issue-keyed** resolution (cwd zero-arg → `ISSUE=`/`REPO=` → `PR=` escape hatch), since most PRs in a mature workspace are project PRs and the PR number is the ambiguous thing; PR# is now derived from (repo, branch). All 8 findings addressed in the plan amendment this round (field-guard reordered before `gh`; `generate_make_skills.sh` + test added with `ISSUE=$ARGUMENTS`/`disable-model-invocation` preserved; cd-to-root, layer-branch-deletion ownership, dropped upstream deps, human-control posture). OQ2: merge-pr stays mechanical.
