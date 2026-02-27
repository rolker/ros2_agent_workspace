# Plan: Allow skills to create worktrees without requiring a GitHub issue

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/304

## Context

Skills that commit changes (research, gather-project-knowledge) currently cannot
create worktrees because `worktree_create.sh` requires `--issue <N>`. Since `main`
is protected, these skills can't commit directly. Creating a full GitHub issue for
each recurring skill operation (updating a digest, refreshing knowledge) is overhead
without value.

The owner has confirmed (issue comment): an ADR is warranted, and the list of skills
that can use this shortcut should be restricted (not open to any string).

## Approach

1. **Add ADR-0007** — Document the decision to allow skill-mode worktrees, including
   which skills are allowed, what constraints apply (PRs still required), and why the
   issue-first policy is relaxed for this category.

2. **Modify `worktree_create.sh`** — Accept `--skill <name>` as an alternative to
   `--issue`. Validate against an allowlist of skill names (`research`,
   `gather-project-knowledge`). Generate a timestamp-based ID
   (e.g., `research-20260227-143022`) for branch naming (`skill/<id>`) and worktree
   path (`skill-<name>-<timestamp>`). Skip issue validation. Omit `Closes #N` from
   draft PR body. Error if both `--issue` and `--skill` are provided. Error if
   neither is provided.

3. **Modify `worktree_enter.sh`** — Accept `--skill <name>`, find skill worktrees by
   globbing `skill-<name>-*` directories. Set `WORKTREE_SKILL` env var instead of
   `WORKTREE_ISSUE`. Handle ambiguity (multiple worktrees for same skill) by picking
   the most recent or listing options.

4. **Modify `worktree_remove.sh`** — Accept `--skill <name>`, find and remove skill
   worktrees using the same glob pattern as enter.

5. **Update `worktree_list.sh`** — Display skill worktrees with a distinct icon/label
   (e.g., `🔧 Skill: research`). The existing `extract_issue_repo` regex won't match
   `skill-*` directories, so add a parallel pattern.

6. **Update `_worktree_helpers.sh`** — If any shared logic needs skill-mode awareness
   (currently unlikely — helpers operate on directory structure, not naming).

7. **Update `AGENTS.md`** — Amend the issue-first policy to exempt skills that
   maintain living documents, listing the specific allowed skills. Add `--skill`
   to the worktree workflow section and script reference.

8. **Update `.agent/WORKTREE_GUIDE.md`** — Add a "Skill Worktrees" section with
   examples, directory layout, and the allowlist.

9. **Update research skill (`SKILL.md`)** — Replace the commit workflow to use
   `--skill research` for worktree creation and push+PR instead of direct commits.

10. **Check framework adapters** — Review `.github/copilot-instructions.md` and
    other adapters for any issue-first references that need updating.

## Files to Change

| File | Change |
|------|--------|
| `docs/decisions/0007-skill-worktrees-without-issues.md` | New ADR documenting the design decision |
| `.agent/scripts/worktree_create.sh` | Add `--skill` flag, allowlist validation, timestamp ID generation, modified PR body |
| `.agent/scripts/worktree_enter.sh` | Add `--skill` flag, glob-based skill worktree discovery, `WORKTREE_SKILL` env var |
| `.agent/scripts/worktree_remove.sh` | Add `--skill` flag, glob-based skill worktree discovery |
| `.agent/scripts/worktree_list.sh` | Display skill worktrees with distinct label |
| `AGENTS.md` | Amend issue-first policy, add `--skill` to worktree section |
| `.agent/WORKTREE_GUIDE.md` | Add skill worktrees section |
| `.claude/skills/research/SKILL.md` | Update workflow to use `--skill research` + PR |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Enforcement over documentation | The allowlist is enforced in `worktree_create.sh` (script rejects unknown skill names). PRs are still required — skill-mode only changes worktree creation, not the review gate. |
| Capture decisions, not just implementations | ADR-0007 records why skills get an exemption, which skills qualify, and what constraints remain. |
| A change includes its consequences | Consequences map requires: worktree script changes → update WORKTREE_GUIDE.md and AGENTS.md; framework skill changes → update adapter. All included in scope. |
| Only what's needed | Solves a concrete pain point without over-generalizing — restricted to named skills, not an open bypass. |
| Human control and transparency | PRs still required; skill worktrees are visible in `worktree_list.sh`; ADR documents the rationale. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0001 — Adopt ADRs | Yes | New ADR-0007 documents this design decision |
| 0002 — Worktree isolation | Yes | Skill-mode still uses worktrees — consistent with ADR-0002's core requirement |
| 0004 — Enforcement hierarchy | Yes | Allowlist enforced in script (environment guardrail layer); PR requirement unchanged (CI/branch protection layer) |
| 0006 — Shared AGENTS.md | Yes | AGENTS.md updated; framework adapters checked for impact |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Worktree scripts | `.agent/WORKTREE_GUIDE.md` | Yes (step 8) |
| Worktree scripts | `AGENTS.md` worktree section | Yes (step 7) |
| `AGENTS.md` | Framework adapters | Yes (step 10) |
| Research skill | Adapter file | Yes (step 10) |

## Open Questions

- Should the allowlist live in the script itself or in a config file (e.g.,
  `.agent/scripts/skill_allowlist.txt`)? Script-inline is simpler; config file
  is more visible and editable. **Recommendation**: inline in the script for now
  (only 2 entries), extract to config if the list grows.
- Should `gather-project-knowledge` use `--type layer` skill worktrees, or only
  `--type workspace`? The issue only describes workspace worktrees, but
  gather-project-knowledge works on project repos. **Recommendation**: support
  both types (the `--type` flag already exists), but only document workspace
  usage initially.

## Estimated Scope

Single PR — 8 files changed, all in the workspace repo.
