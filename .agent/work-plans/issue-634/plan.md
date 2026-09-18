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
     exits 0 whenever `repo_path` is a readable, searchable directory — **presence or
     absence of any or all four kinds is never a non-zero exit**, per the
     issue's "Out of scope" line. Exit 2 = usage (missing/extra args); exit 3
     = `repo_path` does not exist, is not a directory, or is not readable and
     searchable (a real error, not an
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
  (Ask-First: instruction file edit). The operator approved this instruction-file
  edit at the plan-review checkpoint, 2026-09-18; the added row is at
  `AGENTS.md` § Script Reference, immediately after the `field_mode.sh` row.
- [x] Resolved (plan-review Finding 1, confirmed during implementation):
  `run_script_tests.sh` already auto-discovers `test_*.sh` via `nullglob`
  over both `"$SCRIPTS_DIR"/test_*.sh` and `"$TESTS_DIR"/test_*.sh` — no
  explicit registration list exists. `test_planning_doc_probe.sh` needed no
  extra wiring beyond being placed in `.agent/scripts/tests/`; confirmed by
  running `run_script_tests.sh`, which picked it up and ran it without any
  change to the runner.
- [x] Resolved during implementation: `## Vision` heading matching is a
  **prefix match at the `##` level** — `^## Vision` — which matches a bare
  `## Vision` (this workspace's own `README.md`) and also a longer heading
  that starts with the word "Vision", such as `## Vision and Goals` (a
  plausible project-repo phrasing the draft names no exact wording for). It
  does **not** match `### Vision` (wrong heading level) or `## Our Vision`
  (does not start with "Vision"). The choice and its rationale are documented
  in `planning_doc_probe.sh`'s header comment, and all four cases (bare
  heading, trailing-text heading, wrong level, non-matching prefix) are
  covered in `test_planning_doc_probe.sh`'s "Vision heading variants" block.

## Estimated Scope

Single PR.

## Implementation notes (as built)

- `probe_decision` excludes dotfiles when counting `docs/decisions/` entries
  (plan-review Finding 2): a directory holding only a `.gitkeep` reads as
  `absent`, matching the stated intent that an empty placeholder directory is
  not "a decision was recorded here." Covered by a dedicated
  `.gitkeep`-only test case in `test_planning_doc_probe.sh`, alongside the
  already-planned present/empty/absent cases.
- `audit-project`'s new step became step 7 (renumbering the former step 7
  "Cross-reference with workspace" to step 8); the two internal
  cross-references to "step 7's ... correct layer check" were updated to
  "step 8's" to match.
- `janitor-sweep`'s report-format example was extended in place (the
  `#### Project governance — <repo>` block already embeds `audit-project`'s
  report verbatim) to show the Planning Documents table appearing inside
  that embed — no new call site, no new check, no change to the four-check
  status contract, exactly as the plan specifies.
- Full test suite (`run_script_tests.sh`) and `shellcheck --severity=warning`
  (via the pre-commit hook) both pass on the two new scripts; see the PR /
  progress.md for the run output.

### Address-findings pass (Local Review Pre-Push, round 1)

- **Must-fix**: `audit-project` step 7 now guards on `$ROOT` being empty
  (reports `SKIPPED (no workspace root — planning_doc_probe.sh unreachable)`)
  and on the probe script existing at all (`SKIPPED (planning_doc_probe.sh
  not found under $ROOT/.agent/scripts/)`), matching step 5's identical
  precondition. The Planning Documents report section renders that single
  `SKIPPED` note instead of the four-row table when either guard fires.
- `probe_vision`'s heading match gained a word-boundary check
  (`^## Vision([[:space:]]|$)`) so `## Visionary Roadmap` no longer
  false-positives; the header comment and a new test case
  (`'## Visionary Roadmap' ... -> absent`) document and cover it.
- Permission-denied paths (unreadable `README.md`, unlistable
  `docs/decisions/`) now emit a distinct stderr diagnostic while still
  reporting `absent` on the TSV contract (unchanged) — covered by new
  chmod-based tests in `test_planning_doc_probe.sh` (skipped when running as
  root, since permission bits aren't enforced there).
- `probe_decision`'s dotfile exclusion now uses `find ... ! -name '.*'`
  instead of relying on the caller's (unset-by-default) `dotglob` shell
  option — covered by a new "`.gitkeep` alongside a real entry -> present"
  test. The broken-symlink-reads-as-absent behavior (`[ -e ]` following a
  symlink) is now documented explicitly in the function's header comment and
  covered by a new dedicated test case.
- `janitor-sweep`'s Check 2 rollup rule now names the Planning Documents
  table explicitly as non-scoring, tying back to `audit-project`'s own § 7
  rule instead of relying solely on the report-format example's HTML
  comment.
- Both report-format templates (`audit-project` § Report Format and
  `janitor-sweep`'s embedded example) now show Location as empty (`—`) on
  "Not found" rows, matching what `planning_doc_probe.sh` actually emits
  (an empty TSV field for absent kinds), instead of showing the kind's
  expected path as if it had been found.
- **Skipped deliberately**: markdown-context-aware matching (skipping a
  `## Vision` line inside a fenced code block or blockquote) — the header
  comment now records the reason: a stateful line scanner or markdown parser
  is a lot of complexity for a self-inflicted edge case, and since the
  Planning Documents section is descriptive-only, the worst outcome is a
  harmless false "present".

### Address-findings pass (Integrated Review, post-PR)

- **Must-fix**: the CLI entry guard now also checks `[ -x ]` (search
  permission) on `repo_path`, not just `-d`/`-r`. Without it, a
  readable-but-unsearchable `repo_path` let every probe silently report
  `absent` for all four kinds instead of hitting the documented exit-3
  "could not run" contract, since nothing inside an unsearchable directory
  can be stat'd. Covered by a new CLI test (`chmod 600`, skipped when
  running as root).
- **Must-fix**: `audit-project` step 7's guard now checks `[ -x ]` on
  `planning_doc_probe.sh` (subsuming the prior existence-only `[ -f ]`
  check, since `-x` is also false on a nonexistent path) — a
  present-but-non-executable script previously reached the direct
  invocation and failed with a raw "Permission denied" instead of the
  intended `SKIPPED`.
- **Suggestion**: the PR description's "unreadable paths get a stderr
  diagnostic, distinct from absence" claim previously only held for
  `probe_vision`/`probe_decision`. Rather than narrow the description,
  extended `probe_roadmap` and `probe_health` to detect the permission
  problem they're actually capable of detecting — an unlistable parent
  directory (`repo_path` for `ROADMAP.md`, `docs/` for `docs/health.md`) —
  and emit the same distinct stderr diagnostic there, covered by new
  chmod-based tests. Both probes still correctly report "present" for a
  target file that exists but is itself unreadable, since neither reads
  file contents. `probe_decision`'s `find` now uses `-L` so listing
  descends through `docs/decisions` when it is itself a symlink to a
  populated directory (previously misreported `absent`, since plain `find`
  does not descend into a symlinked start path) — this also closes one of
  the three test-only symlink gaps Local Review round 2 left open.
- While already in the test file for the suggestion above, also added the
  other two round-2 symlink test gaps: a broken-symlink `ROADMAP.md` and a
  broken-symlink `docs/health.md` (both already correctly read `absent` via
  `[ -f ]`; test-completeness gaps only, no behavior change).
- Full test suite (`run_script_tests.sh`, 27/27 shell incl. 44/44
  `test_planning_doc_probe.sh` assertions, 220/220 pytest) and
  `shellcheck --severity=warning` (via the pre-commit hook) both pass on
  the modified scripts.

### Round-3 fix (host-inline, 2026-09-18)

The address-findings pass had made `probe_roadmap` / `probe_health` require
both `r` and `x` on the parent directory; a stat by known name needs only `x`,
so a search-only parent misreported a present file as absent. Guards now check
`x` alone, with `chmod 100` tests for both probes. The `find -L` symlink
following in `probe_decision` is documented as unbounded to `repo_path` (one
level, present/absent only).

### Symlink bound (host-inline, 2026-09-18)

Copilot's post-push review and the round-3 Lens B pass both flagged that a
symlinked `docs/decisions` could resolve outside the repo. The probe now
follows such a symlink only when its target resolves inside `repo_path`
(`realpath` prefix check); otherwise absent with a diagnostic. Tested.

### Copilot round on the symlink-bound head (host-inline, 2026-09-18)

Four points, all applied: the inside-repo rule now applies per entry of
`docs/decisions` as well as to the directory symlink itself; the audit-project
guard uses `[ -f ]` + `bash` like step 1 (exec bits are lost on noexec mounts,
archives and CIFS shares); the plan's CLI contract and the AGENTS.md row now
say "readable and searchable", matching the script.

### Copilot round on the bash-invocation head (host-inline, 2026-09-18)

Two points, both applied: `probe_decision` now distinguishes an unsearchable
`docs/` parent from a missing `docs/decisions` (diagnostic, tested); the
audit-project block captures the probe's exit status and renders `SKIPPED`
on a non-zero exit instead of silently proceeding with no TSV. A `-r || -x`
guard Copilot flagged at the same time is `probe_decision`'s own, which
enumerates a directory and does need read; roadmap/health are `-x` only.
