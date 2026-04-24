# ADR-0011: Field Mode — Permit Direct Default-Branch Edits in Non-GitHub-Origin Repos

## Status

Accepted

## Context

[ADR-0002](0002-worktree-isolation-over-branch-switching.md) established that
all feature work uses git worktrees, and "never edit files in the main tree"
became a top-of-`AGENTS.md` rule (further hardened by issue #247). The rule
is correct on dev machines where origin is GitHub: the worktree + PR workflow
is the quality gate.

The rule breaks down on field machines (e.g., BizzyBoat, mercat) where:

- The project-repo origin is `gitcloud` (or another non-GitHub remote) —
  there is no GitHub on the field side
- There is no PR review, no CI, no protected branch
- A hotfix has to land on the default branch and be pushed before the next run
- Worktree-creating scripts implicitly assume PR review on the other end

Field deployments started using the workspace in early 2026 (BizzyBoat
sessions Apr 2-6, etc.). Initially, agents either (a) violated the rule
under time pressure or (b) went through worktree+PR motions that nobody
read because the field machine was the only reader.

The reconciliation half — pulling field commits back to GitHub for review on
a connected dev machine — was already shipped in PR #440 (the
`/import-field-changes` skill, which did not require its own ADR). What was
deferred (per issue #432) was the field-side edit permission itself, which
this ADR now records.

## Decision

**A repo is in "field mode" when its `origin` host is not on the
GitHub allowlist.** The allowlist lives in
[`.agent/scripts/field_mode.sh`](../../.agent/scripts/field_mode.sh)
and currently contains `github.com` and `ssh.github.com` (GitHub's
SSH-over-443 fallback). Hosts are added to the allowlist only when
GitHub itself documents them for git clone. Field-mode repos may:

- Edit tracked files directly in the main/default tree
- Commit directly to the default branch (e.g. `main`, `jazzy`)
- Push to `origin` without opening a PR

Field mode does NOT relax: pre-commit hooks, AI signature, atomic commits,
secret-handling rules, or destructive-op confirmation.

Detection is implemented in
[`.agent/scripts/field_mode.sh`](../../.agent/scripts/field_mode.sh)
(sourceable helper or executable), inspecting the host portion of the
origin URL with a tight host-anchor pattern so substrings like
`mygithub.com` are not misclassified. Reconciliation back to GitHub uses
the existing `/import-field-changes` skill on a connected dev machine.

This is a scoped exception to ADR-0002. Worktree isolation remains the
default for GitHub-origin repos. The exception applies only when the
origin URL tells us the workspace's GitHub-based PR workflow isn't
available — other forges (Forgejo, GitLab) may have their own PR/CI
mechanisms, but those don't plug into this workspace's review pipeline.

### Decisions Recorded on Issue #445

1. **Mode detection: per-repo origin URL check** — rejected machine-wide
   config files and `.agent/project_config.yaml`-based detection because
   the origin URL is the ground truth and field machines bootstrap with
   non-GitHub origin already.
2. **No hard-coded workspace-repo carve-out** — `is_field_mode()` will
   return true on a field-cloned workspace repo. Hotfixing workspace
   infrastructure from the field is discouraged in the walkthrough but
   not mechanically blocked. If workspace commits ever appear on `main`
   from a field machine, they'll be handled ad hoc.
3. **`setup.bash` checkout guardrail stays strict** — field hotfixes are
   edit-and-commit on the current branch, not branch switching, so the
   guardrail doesn't need a mode-aware bypass.
4. **Project-repo `.agents/README.md` template gets a one-line pointer** —
   `AGENTS.md` owns the rule. Per-project rule duplication is avoided to
   prevent drift; project-repo guides defer to the workspace canonical
   docs.

## Consequences

**Positive:**
- Field deployments can ship hotfixes without ceremony nobody reads
- The relaxation is mechanical and documented, not informal — agents
  can determine mode from the repo state, not from memory or convention
- Reconciliation (PR #440 / `/import-field-changes`) ensures field
  commits still pass through the dev-mode review gate, just delayed
- The carve-out is narrow: GitHub-origin repos still enforce ADR-0002

**Negative:**
- Two parallel rule paths increase doc complexity (mitigated by `AGENTS.md`
  + the field-mode walkthrough being the only canonical surfaces)
- Field commits skip the immediate review gate; the dev-side import
  depends on someone running the skill in a timely way
- A non-GitHub mirror clone of the workspace would technically be in
  field mode without intent — accepted because the workspace repo is
  unlikely to be edited from the field, and the failure mode is
  recoverable by ad-hoc cleanup

## References

- [ADR-0002](0002-worktree-isolation-over-branch-switching.md) — Worktree
  Isolation Over Branch Switching (this ADR is a scoped exception)
- Issue [#445](https://github.com/rolker/ros2_agent_workspace/issues/445) —
  Field mode design and decisions
- Issue [#247](https://github.com/rolker/ros2_agent_workspace/issues/247) —
  Dev-mode worktree hardening (the rule field mode carves out from)
- Issue [#432](https://github.com/rolker/ros2_agent_workspace/issues/432) —
  Asked the question this ADR answers (closed when PR #440 shipped the
  reconciliation half without addressing the edit-side question)
- PR [#440](https://github.com/rolker/ros2_agent_workspace/pull/440) —
  Shipped `/import-field-changes` for the reconciliation half
- [`.agent/knowledge/field_mode_hotfix.md`](../../.agent/knowledge/field_mode_hotfix.md) —
  Concrete walkthrough
- [`.agent/scripts/field_mode.sh`](../../.agent/scripts/field_mode.sh) —
  Detection helper
