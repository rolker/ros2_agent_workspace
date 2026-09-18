# Plan: Conventional-path discovery of planning documents in janitor-sweep and audit-project (sub-issue 2 of #628)

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/634

## Context

The merged design draft (`docs/design/planning_document_vocabulary.md`, PR
[#638](https://github.com/rolker/ros2_agent_workspace/issues/638)) publishes an
**expected-location table** for four planning-document kinds:

| Kind | Expected location |
|---|---|
| vision | a `## Vision` section in `README.md` at the repo root |
| roadmap | `ROADMAP.md` at the repo root |
| decision | `docs/decisions/` |
| health | `docs/health.md` |

The draft is explicit that this is a *published expectation, not a
requirement*: a repo lacking any of these paths is not in violation, and
absence must never surface as a finding. Today `janitor-sweep` and
`audit-project` (`.claude/skills/janitor-sweep/SKILL.md`,
`.claude/skills/audit-project/SKILL.md`) know nothing about this table — they
have no way to report what planning documents a repo publishes.

Operator decisions recorded on the issue (2026-09-18):
1. The `kind:` marker bullet is **dropped** — the merged draft defines no
   marker; it was a holdover from #628's pre-draft proposal.
2. External-hosting discovery fallbacks (org Project board API, `homepage`
   field, README link matching) are **split to
   [#643](https://github.com/rolker/ros2_agent_workspace/issues/643)** — out
   of scope here.
3. Carried from the issue-review checkpoint: the probe must be a real script
   (not skill-markdown prose only), so the graceful-absence requirement has
   something to attach an automated test to, and it must not introduce a
   per-repo cache or marker file (ADR-0017's rejection of a declaration file
   extends to this reader).

This PR ships exactly: the probe script, its wiring into `audit-project` (and,
through that, `janitor-sweep`), the graceful-absence test, and the `AGENTS.md`
Script Reference row.

## Approach

1. **Add `.agent/scripts/planning_doc_probe.sh`** — a small, sourceable +
   CLI-executable script, in the same shape as `field_mode.sh` /
   `workspace_root.sh`. It takes one repo path and probes *exactly* the four
   rows of the expected-location table above — no other paths, no
   externally-hosted fallback (that is #643's job).

   - `probe_vision <repo_path>`: present iff `README.md` exists at the repo
     root and contains a line matching `^## Vision` (case-sensitive, matching
     the draft's own heading and this workspace's `README.md`).
   - `probe_roadmap <repo_path>`: present iff `ROADMAP.md` exists at the repo
     root (plain file existence — the draft names no required heading for
     this kind).
   - `probe_decision <repo_path>`: present iff `docs/decisions/` exists and
     contains at least one entry (an empty placeholder directory reads as
     absent, not present — the kind is "a decision was recorded here", not
     "the directory was created").
   - `probe_health <repo_path>`: present iff `docs/health.md` exists.
   - `probe_all <repo_path>`: runs all four and prints one TSV line per kind
     to stdout: `<kind>\t<present|absent>\t<relative-path-or-empty>`. This is
     the machine-readable contract `audit-project` and (transitively)
     `janitor-sweep` read.
   - CLI form (`planning_doc_probe.sh <repo_path>`) calls `probe_all` and
     exits 0 whenever `repo_path` is a readable directory — **presence or
     absence of any or all four kinds is never a non-zero exit**, per the
     issue's "Out of scope" line. Exit 2 = usage (missing/extra args); exit 3
     = `repo_path` does not exist or is not a directory (a real error, not an
     absence finding — the probe couldn't run at all).
   - The script reads the filesystem only; it writes nothing, caches
     nothing, and creates no marker file anywhere — satisfying the ADR-0017
     constraint carried from issue-review.

2. **Wire `audit-project` to call it** — new step reading
   `"$ROOT/.agent/scripts/planning_doc_probe.sh" "$REPO_PATH"` (addressed
   through `$ROOT` exactly like `resolve_repo_checkout.sh` in step 1, since a
   layer worktree has no `.agent/scripts/` beside it) and rendering the four
   TSV lines as a new **"Planning Documents"** report section: a `Kind |
   Status | Location` table, `Present` / `Not found` — deliberately not
   "Missing", which the existing Governance Coverage table uses for
   items that *are* mandatory-ish and do drive a Recommended Actions bullet.
   Planning-document absence drives **no** Recommended Actions entry — the
   section is descriptive only, matching the draft's "absence is never a
   finding" rule.

3. **`janitor-sweep` inherits it without a second call site.**
   `janitor-sweep`'s check 2 ("Project governance") already runs
   `/audit-project <repo>` per repo in the rotation chunk and embeds that
   report verbatim under `#### Project governance — <repo> (mode:
   layer/clone)` (SKILL.md step 3 / report format). Once `audit-project`
   carries the Planning Documents section, `janitor-sweep`'s sweep report
   carries it for free through that existing embed — there is deliberately
   **no separate script invocation from `janitor-sweep`**, so there is one
   call site for the probe, not two lists that can drift apart (the same
   principle the issue invokes for why this reader must probe the draft's
   table directly rather than duplicating it). `janitor-sweep`'s four-check
   status contract (`OK`/`FINDINGS`/`SKIPPED`/`FAILED`) is untouched — this
   is not a fifth check, since a graded check could not honor "absence is
   never a finding" without inventing a fifth state that means "ran clean by
   definition," which the status contract doesn't have.

4. **Add `.agent/scripts/tests/test_planning_doc_probe.sh`**, matching the
   shape of `test_field_mode.sh` (source the script, build throwaway repos
   under a `mktemp -d`, assert against `probe_vision` / `probe_roadmap` /
   `probe_decision` / `probe_health` / `probe_all` output). Cases:
   - Each kind present alone, each kind absent alone.
   - `## Vision` heading variants: present, absent (file exists but no
     matching heading), heading at a different level (`### Vision` must not
     match — the draft's row is specifically a `##` heading).
   - `docs/decisions/` present-but-empty → absent (per rule 1c above).
   - **The graceful-absence case, named explicitly**: a repo with none of
     the four paths — `probe_all` returns four `absent` lines, the CLI form
     exits 0, and stderr is empty. This is the automated instantiation of
     the issue's headline requirement.
   - CLI usage/error cases: missing arg (exit 2), nonexistent path (exit 3).
   - Register the new test in whatever aggregator runs
     `.agent/scripts/tests/*.sh` (`run_script_tests.sh` — confirm during
     implementation whether it auto-discovers `test_*.sh` or needs an
     explicit add).

5. **Add the `AGENTS.md` Script Reference row** for
   `.agent/scripts/planning_doc_probe.sh`, describing what it probes, its TSV
   output contract, and its exit codes — following the existing row style
   (see `field_mode.sh`'s row for the closest analog: CLI + sourceable,
   documents both entry points). **This is an instruction-file edit and
   AGENTS.md's own Boundaries table requires Ask-First approval before it
   lands** — flagged here rather than applied silently; the operator should
   confirm the row's wording (or the fact that a row is added at all) before
   this PR is merged, consistent with `AGENTS.md § Boundaries → Ask First →
   "Modifying instruction files"`.

## Files to Change

| File | Change |
|---|---|
| `.agent/scripts/planning_doc_probe.sh` | New. Probe script (CLI + sourceable functions), no writes/caches/markers. |
| `.agent/scripts/tests/test_planning_doc_probe.sh` | New. Unit tests incl. the explicit graceful-absence case. |
| `.claude/skills/audit-project/SKILL.md` | New step calling the probe; new "Planning Documents" report section (descriptive, no Recommended Actions entries for absence). |
| `.claude/skills/janitor-sweep/SKILL.md` | Note/example update only — the existing per-repo embed of `audit-project`'s report already carries the new section; update the report-format example under step 4 to show it so the template stays accurate. |
| `AGENTS.md` | New Script Reference row for `planning_doc_probe.sh` — **Ask-First edit, needs operator confirmation before merge.** |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Workspace vs. project separation | Change is entirely workspace-repo skill/script logic; nothing project-specific. |
| Only what's needed | External-hosting fallbacks explicitly excluded (split to #643); no fifth janitor-sweep check invented; no new per-repo file. |
| Test what breaks | The probe is a real script with a dedicated test file, not left as untestable skill prose — resolves the "Watch" flagged at issue-review. |
| Capture decisions, not just implementations | The graceful-absence rule and the "no marker" constraint are both design-draft decisions this plan implements rather than re-litigates. |
| Enforcement over documentation | Probing becomes code (a script + a test), not a second markdown description of the table that could drift from the draft's own. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0003 (project-agnostic workspace) | Yes | The probe checks generic paths only; no project-specific assumption. A repo with none of the four paths is unaffected — the graceful-absence test is this ADR expressed as a test, per the design draft's own framing. |
| ADR-0013 (progress.md entry-type vocabulary) | No (carve-out) | `janitor-sweep`/`audit-project` are periodic, non-issue-scoped skills, exempted per the review guide's consequences map. |
| ADR-0017 (extend AGENTS.md to project repos / reject per-repo declaration files) | Yes | The probe reads only paths a project repo would already have reason to hold on its own account (README, ROADMAP, docs/decisions, docs/health.md) and writes nothing — no cache, no marker file, no schema. This is the constraint carried from the issue-review checkpoint. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `audit-project`'s report format | `janitor-sweep`'s report-format example (step 4), since it embeds `audit-project`'s output verbatim | Yes — item 3 above |
| `AGENTS.md` Script Reference table | Nothing else references this table's row format | Yes, but gated on operator Ask-First confirmation |
| The design draft's "Promotion path to an ADR" gate 1 (conventional-path reader has landed) | `docs/design/planning_document_vocabulary.md`, tracked on #637 | No — issue-review already noted this belongs to #637, not this PR |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): `.claude/skills/audit-project/SKILL.md`
  and `.claude/skills/janitor-sweep/SKILL.md` — both describe report formats
  that become inaccurate once the probe exists; both are edited in this PR
  (items 2–3 above).
- **Agent-instruction candidates** (proposals only — operator decides): the
  new `AGENTS.md` Script Reference row (item 5) is itself the candidate this
  PR proposes — it is called out above as needing explicit Ask-First
  confirmation rather than being treated as a routine doc update.

## Open Questions

- [ ] Confirm the `AGENTS.md` Script Reference row wording before merge
  (Ask-First: instruction file edit).
- [ ] Confirm `run_script_tests.sh` auto-discovers `test_*.sh` under
  `.agent/scripts/tests/`, or needs the new test file added to an explicit
  list — resolve during implementation, not a design decision.
- [ ] Confirm `## Vision` heading matching should be exact-level (`^## Vision`)
  and not also match trailing text on the same line (e.g. `## Vision and
  Goals`) — this plan assumes a prefix match on the heading text is fine
  (the workspace's own `README.md` uses a bare `## Vision`), but a project
  repo could plausibly use a longer heading. Flagging rather than guessing;
  low-stakes since it only affects a descriptive Present/Not-found row.

## Estimated Scope

Single PR.
