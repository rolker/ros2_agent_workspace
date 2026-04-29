# Plan: worktree_create.sh — gate git-bug lookup on workspace-repo target

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/457

## Context

`worktree_create.sh` resolves the issue title (and state) in two stages —
git-bug first, `gh` as fallback. The git-bug call at line 602 is hardcoded
to query `$ROOT_DIR` (the workspace) regardless of `WORKTREE_TYPE`:

```bash
_BUG_TITLE=$(gitbug_lookup "$ROOT_DIR" "$ISSUE_NUM" title 2>/dev/null || echo "")
```

For layer-type worktrees, the issue lives in a project repo, not the
workspace — so this returns the workspace repo's issue with the same
number when one exists, populating both `ISSUE_TITLE` (line 605) and
`ISSUE_STATE` (lines 606–608) from the wrong source. The `gh` fallback at
line 632 only engages when both fields are empty, so once git-bug returns
*anything* the wrong-repo data sticks.

The actual git work (branch, push, PR target) is unaffected — those paths
correctly use `GH_REPO_SLUG`. The defect is in the human-facing metadata
display only.

## Approach

1. **Capture the workspace's GitHub slug once.** Add `WORKSPACE_GH_SLUG`
   resolution near the existing `GH_REPO_SLUG` logic (around lines 547–567),
   pulling from `$ROOT_DIR`'s origin via `extract_gh_slug`. This becomes
   the canonical "where git-bug data lives" identifier.

2. **Gate the git-bug lookup.** At line 601, only call `gitbug_lookup`
   when `GH_REPO_SLUG` is empty (best-effort fallback for unresolved
   targets) **or** equals `WORKSPACE_GH_SLUG`. Mirrors the existing
   cross-repo precedent at line 1192–1193.

3. **No change to the `gh` fallback.** Lines 632–642 already correctly use
   `--repo "$GH_REPO_SLUG"`. With git-bug gated, the fallback engages
   automatically for project-repo issues and returns the right data.

4. **Regression test.** Add a focused test to
   `.agent/scripts/tests/test_worktree_create.sh` that simulates the
   collision: stub `gitbug_lookup` to return a fake title, set
   `GH_REPO_SLUG` to a non-workspace value, and assert the gh-fallback
   path is taken (not the git-bug result).

5. **ADR-0010 cross-reference addendum.** Add a References section to
   ADR-0010 noting that `gitbug_lookup` is only valid against the bridged
   (workspace) repo, with a link to #457 / this PR. Permitted under
   ADR-0012 (cross-reference addendums).

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/worktree_create.sh` | Add `WORKSPACE_GH_SLUG` resolution; gate `gitbug_lookup` call on `GH_REPO_SLUG = WORKSPACE_GH_SLUG` (or empty). |
| `.agent/scripts/tests/test_worktree_create.sh` | Add collision-scenario regression test. |
| `docs/decisions/0010-adopt-git-bug-for-local-issue-tracking.md` | References-section addendum (ADR-0012-style) noting the workspace-only validity. |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Banner stops misleading agents about which task they're on — direct improvement to a transparency surface. |
| Enforcement over documentation | The doc-only "verify issue matches before first commit" rule survives this fix; the supporting tooling no longer lies. Mechanical enforcement of the verification rule itself is tracked separately (out of scope). |
| Only what's needed | One gating condition + one slug capture; no new abstractions or helper functions beyond what already exists. |
| Test what breaks | Regression test added for the exact collision scenario — the kind of "hard to find in the field" defect that justifies a permanent guard. |
| A change includes its consequences | ADR-0010 addendum captures the workspace-only constraint for future scripts. AGENTS.md script-reference table needs no update (no signature change). |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0002 — Worktree isolation | Yes | Decision unchanged; only the metadata-display path of an existing enforcement script. |
| 0010 — git-bug for local issue tracking | Yes | Lookup priority preserved (git-bug → gh), but constrained to bridged repo. Addendum makes the constraint explicit for future maintainers. |
| 0012 — Cross-reference addendums | Yes | Used as the mechanism for the ADR-0010 References-section update. |
| 0004 — Enforcement hierarchy | Watch (deferred) | The doc-only "verify issue matches" rule is a separate enforcement-layer gap. Out of scope here; see Open Questions for follow-up. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `.agent/scripts/worktree_create.sh` | AGENTS.md script reference table; Makefile target | Not needed — invocation signature and Makefile integration unchanged. |
| ADR in `docs/decisions/` | Review guide ADR table | Not needed — addendum only adds References, does not change ADR scope or decision. |
| Worktree scripts | `.agent/WORKTREE_GUIDE.md`; AGENTS.md worktree section | Not needed — no behavior change visible to script users; only the displayed title becomes correct. |

## Open Questions

- **Land the ADR-0010 addendum in this PR, or split?** Recommend **same PR**:
  it's a one-paragraph cross-reference change, ADR-0012-permitted, and
  avoids a follow-up doc-only PR.
- **File a separate issue for the ADR-0004 enforcement-hierarchy gap?**
  The "verify issue matches before first commit" rule is currently
  doc-only. Worth a follow-up issue proposing a mechanical check (e.g.,
  a one-shot helper invoked at first-commit time). Should I open that
  issue when this PR lands, or now?

## Estimated Scope

Single PR. Changes are localized to one script, one test file, and a
References-section addendum on one ADR. No coordinated rollout, no
migration. Estimate: ~30 lines of script change, ~40 lines of test,
~10 lines of ADR addendum.
