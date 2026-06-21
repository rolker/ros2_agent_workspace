# ADR-0015: Dispatch Handoff Context Contract (host-injects read, container-produces / host-publishes write)

## Status

Accepted.

## Context

`dispatch_subagent.sh` runs a workflow phase (`review-issue`, `plan-task`,
`review-code`, …) in a fresh-context sub-agent — either in-process (the host's
`Agent` tool) or in a headless container
([#490](https://github.com/rolker/ros2_agent_workspace/issues/490),
[#481](https://github.com/rolker/ros2_agent_workspace/issues/481) phase C). The
container path is deliberately **headless and local-first**: it runs with a
long-lived Claude subscription token but **no GitHub write auth**, so that
nothing publishes from inside the sandbox.

[#532](https://github.com/rolker/ros2_agent_workspace/issues/532) framed the
**write** side of that boundary: the container *produces* artifacts (review
findings, a `progress.md` entry, commits) and the **host publishes** them
(pushes, opens the PR, posts review comments). The container's best-effort
GitHub *posts* skip silently when no write token is present — the canonical
record is the committed `progress.md` entry, not the GitHub comment.

Two gaps surfaced driving [#466](https://github.com/rolker/ros2_agent_workspace/issues/466)
through `/run-issue` with container dispatch, both of which are really about the
**read** side of the same boundary — the side #532 did not yet name:

1. **Read-side auth dependency.** A phase whose first step is `gh issue view <N>`
   (e.g. `review-issue`) fails outright in a headless container: it has no GitHub
   read auth, so it cannot fetch the issue body the review is built from. The
   #466 workaround embedded the body in a `--prompt-file`, which abandoned
   `--skill` (losing the auto entry-type and auto model).

2. **Exit-contract read.** The host learns a phase's outcome by reading the
   **last** `progress.md` entry it wrote — a *read* of container-produced state.
   The gate that decides whether a fresh entry exists must track reality, or the
   host routes on a false `FAILED`.

The unifying question: **who supplies the inputs a dispatched phase reads, and
who consumes the outputs it produces, when the container has no GitHub auth in
either direction?**

## Decision

Adopt a symmetric **dispatch handoff context contract**, the read-side
complement to #532's write-side framing:

### Read side — the host injects context; the container never fetches

When a dispatched phase needs GitHub-hosted context (an issue or PR body), the
**host** fetches it (`gh issue view` / `gh pr view`, with the host's own auth)
and passes it to the dispatcher via **`dispatch_subagent.sh --context-file
<path>`**. The dispatcher splices the file into the handoff prompt under a
clearly delimited `## Injected GitHub context (issue/PR body, fetched
host-side)` heading and instructs the sub-agent to read it **instead of**
`gh issue view`.

- The **fetch stays in the caller** (`run-issue` / the operator). The dispatcher
  never gains a GitHub client; it only splices a file the host already fetched.
  This keeps the container's no-GitHub-auth boundary intact and makes the
  mechanism unit-testable.
- `--context-file` is **composable with `--skill`** (unlike `--prompt-file`): the
  phase keeps its auto entry-type and auto model *and* receives the injected
  body.
- **Transparency** (human-control principle): the dispatcher logs to stderr what
  was injected — byte size and first non-empty line — never the raw contents.
- Phases that need no GitHub read (pre-push `review-code`, `review-plan` — they
  work from the diff / local `plan.md`) take no `--context-file`.

### Write side — the container produces; the host publishes (restated, #532)

Unchanged from #532, recorded here for symmetry: a container phase *produces*
artifacts and commits them; the **host publishes** (push, PR, review comments).
GitHub posts from inside the container are best-effort and skip silently with no
write token. The canonical record is the committed `progress.md` entry.

### Exit-contract read — gate on entry freshness, not raw count delta

The host reads the phase's outcome from the **last** matching `progress.md`
entry. The dispatcher's gate decides an entry is **fresh** iff the PRE→POST
entry count **grew** (an entry was appended) **or** the count stayed flat but the
last matching entry's `when|status` **signature changed** (a re-dispatch
*replaced* the typed entry in place — e.g. a prior `failed` → a `complete`).
Only a flat count *and* an unchanged signature reads as `FAILED` ("died before
reporting"). A same-minute/same-status re-dispatch collapses to an identical
signature and so reads as `FAILED` — **fail-closed**, the accepted safe
direction.

## Consequences

**Positive:**
- A container phase that needs an issue/PR body works with **zero GitHub auth
  inside the sandbox** — the host injects what the phase reads, preserving the
  local-first boundary in both directions.
- `--context-file` replaces the brittle `--prompt-file` workaround without
  sacrificing `--skill`'s auto entry-type / model.
- The freshness gate makes the host's lifecycle routing track reality: a
  re-dispatch that legitimately *replaces* a typed entry no longer reports a
  false `FAILED`.
- The read/write symmetry gives future phase authors a single contract to reason
  about: **host supplies GitHub inputs, host consumes GitHub outputs; the
  container touches neither.**

**Negative:**
- The host must remember to fetch + pass `--context-file` for context-needing
  phases; a phase dispatched without it (and without read auth) still fails on
  `gh issue view`. Mitigation: `run-issue` documents which phases need it.
- The fail-closed freshness edge (same-minute/same-status re-dispatch → `FAILED`)
  can mislabel a genuine same-minute success. Accepted: a stuck re-dispatch is
  the far more likely cause, and `FAILED` surfaces to the operator rather than
  silently passing.
- Adds one flag to the dispatcher's surface and one decision document.

## References

- [ADR-0001](0001-adopt-architecture-decision-records.md) — Adopt Architecture
  Decision Records (parent; this ADR captures a workspace decision per
  ADR-0001's rule).
- [ADR-0013](0013-progress-md-entry-type-vocabulary.md) — the `progress.md`
  entry-type vocabulary the exit-contract read consumes (`**Status**` / `**When**`).
- Issue [#532](https://github.com/rolker/ros2_agent_workspace/issues/532) — the
  write-side "container-produces, host-publishes" framing this ADR completes on
  the read side.
- Issue [#552](https://github.com/rolker/ros2_agent_workspace/issues/552) — this
  issue: `--context-file` (read-side injection) + the freshness exit-gate.
- Issues [#466](https://github.com/rolker/ros2_agent_workspace/issues/466) /
  [#550](https://github.com/rolker/ros2_agent_workspace/issues/550) — sibling
  dispatch-contract work; #466's live run surfaced both gaps fixed here.
- Issue [#490](https://github.com/rolker/ros2_agent_workspace/issues/490) /
  [#481](https://github.com/rolker/ros2_agent_workspace/issues/481) — the
  dispatch plumbing + `run-issue` orchestrator this contract governs.
