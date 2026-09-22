# Plan: janitor-sweep: local-first — --publish opt-in, local diff source, optional per-project health when the project root has a ROADMAP.md

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/652

## Context

`janitor-sweep`'s workspace scope currently *always* commits `docs/health.md`
and opens a PR (§ 7, "Publish the workspace scope") as its terminal step, and
always diffs the workspace scope against that committed file (§ 5). The
operator's 2026-09-21 direction (quoted in the issue) backburners the cloud
trigger (#636) and asks for a local-first default: every run writes its local
report; the commit-and-PR flow becomes opt-in behind `--publish`; and an
optional second local report is added for the project this workspace clone is
configured for, when that project keeps a root `ROADMAP.md`.

The `## Issue Review` entry in `.agent/work-plans/issue-652/progress.md` left
four open actions unresolved by the issue text. This plan resolves all four,
plus the "Also in this PR" items (ROADMAP.md rows, two knowledge-doc
sentences, SKILL.md Overview/Deferred rewrite).

**Baseline**: main is post-#651 (PR #655, merged) — § 5's coverage-gated
`Not re-examined` diff state is already in the skill text this plan edits
against. Nothing in #651 conflicts with this issue's three decisions (verified
in the Issue Review).

## Approach

### 1. `manifest_fallback.sh` — extract the URL→identity parsing into a reusable function

Today `manifest_config_dir()` inlines the regex that derives
`<owner>/<repo>/<branch>/<config_path>` from `configs/project_bootstrap.url`
(or `$BOOTSTRAP_URL`) — there is no separate function that returns just the
identity. Action 1 requires reusing this parsing, not duplicating it, so:

- Extract the parsing + validation block (the `raw.githubusercontent.com`
  regex match, the owner/repo/branch safe-character checks, the `repo="${repo%.git}"`
  normalization) out of `manifest_config_dir()` into a new function,
  `manifest_bootstrap_identity <workspace_root>`, that prints
  `<owner>\t<repo>\t<branch>\t<config_path>` on success and returns the same
  exit codes `manifest_config_dir` already documents at this point in its own
  flow (3 = no pointer / unsupported url form). It does not clone anything —
  purely a string-parse of the tracked pointer file (or `$BOOTSTRAP_URL`).
- `manifest_config_dir()` calls `manifest_bootstrap_identity` internally and
  continues exactly as it does today (clone, refresh, bootstrap.yaml
  cross-check) using the returned fields — behavior-preserving refactor, no
  change to any of `manifest_config_dir`'s documented exit codes or output.
- Add a short doc comment above the new function, and update the file's
  top-of-file usage comment to list it alongside `manifest_config_dir`.
- This keeps the "only one place parses the pointer" property the workspace
  already relies on for `manifest_config_dir`, and gives `janitor-sweep` (and
  any future caller) the owner/repo identity without a clone.

### 2. `janitor-sweep` SKILL.md — resolve the project root (new step, after step 1)

Add a new step, **"1a. Resolve the configured project root"**, run once per
sweep, right after the workspace-root/report-directory setup (current step 1)
and before the repo-rotation build (current step 2 — renumbered to 3, see
below only if needed; prefer inserting as `1a` to avoid renumbering the many
cross-references to step 5/6/7/8 elsewhere in the skill and in this plan).

```bash
# shellcheck source=/dev/null
source "$ROOT/.agent/scripts/manifest_fallback.sh" || {
    PROJECT_ROOT_STATUS="SKIPPED(no project configured on this checkout)"
}
if [ -z "${PROJECT_ROOT_STATUS:-}" ]; then
    if ! identity=$(manifest_bootstrap_identity "$ROOT"); then
        # exit 3 from manifest_bootstrap_identity: no pointer file, or a url
        # in a form it cannot parse (e.g. a cloud checkout with no project
        # configured on this clone — decision 4). Not an error; the run
        # continues with no per-project health report.
        PROJECT_ROOT_STATUS="SKIPPED(no project configured on this checkout)"
    else
        PROJECT_REPO_NAME=$(cut -f2 <<< "$identity")
        if ! resolved=$("$ROOT/.agent/scripts/resolve_repo_checkout.sh" "$PROJECT_REPO_NAME"); then
            PROJECT_ROOT_STATUS="FAILED(project root resolve: could not resolve $PROJECT_REPO_NAME — see stderr)"
        else
            PROJECT_ROOT_PATH=${resolved%%$'\t'*}
            PROJECT_ROOT_MODE=${resolved##*$'\t'}
            if ! ROADMAP_ROW=$(bash "$ROOT/.agent/scripts/planning_doc_probe.sh" "$PROJECT_ROOT_PATH" 2>/dev/null | awk -F'\t' '$1=="roadmap"'); then
                PROJECT_ROOT_STATUS="FAILED(project root resolve: planning_doc_probe.sh could not run on $PROJECT_REPO_NAME)"
            elif [ "$(cut -f2 <<< "$ROADMAP_ROW")" = "present" ]; then
                PROJECT_ROOT_STATUS="configured: $PROJECT_REPO_NAME (mode: $PROJECT_ROOT_MODE)"
            else
                PROJECT_ROOT_STATUS="no-roadmap: $PROJECT_REPO_NAME"
            fi
        fi
    fi
fi
```

- **Never hardcodes a project name** — `$PROJECT_REPO_NAME` comes from the
  tracked pointer, exactly as `manifest_config_dir` already derives it for
  the manifest clone. This is the same repo in both cases: "the project root"
  *is* the manifest repo checkout, per the issue's own parenthetical and the
  two-root rule.
- Reuses `resolve_repo_checkout.sh` (works without `layers/`) and
  `planning_doc_probe.sh`'s `probe_all`/CLI TSV output (`probe_roadmap`'s
  contract, read off the `roadmap` row) rather than a hand-rolled `[ -f
  ROADMAP.md ]` test — consistent with decision 3 ("no new declaration file,
  no schema") and with how `audit-project` already probes the same four
  kinds.
- Four terminal states, all recorded (none silently swallowed):
  `SKIPPED(no project configured on this checkout)` (no pointer, or a form
  `manifest_bootstrap_identity` cannot parse — covers both "a cloud checkout
  configured for no project" per decision 4 and a dev checkout with no
  pointer at all, since the underlying condition — no project identity
  derivable — is the same in both places),
  `FAILED(project root resolve: <reason>)` (pointer parsed but the checkout
  or the probe could not be produced),
  `no-roadmap: <repo>` (resolved fine, `ROADMAP.md` absent — decision 3's "a
  project without a root roadmap gets what it gets today": no per-project
  report this run, no error),
  `configured: <repo> (mode: <layer|clone>)` (per-project report proceeds).
- This step's outcome feeds the existing `## Projects` section's per-repo
  findings for `$PROJECT_REPO_NAME` (already computed by the rotation, when
  that repo is in this run's chunk) and the new per-project report (§ 3
  below) — it does not run its own separate `audit-project`; it reuses the
  Projects section's data for that one repo.
- **Rotation interaction, stated explicitly**: the project root repo is
  audited by the existing rotation exactly like any other repo — only when
  it falls in this run's ISO-week chunk (current step 2/"Build the repo
  rotation"). When it is not in this run's chunk, `PROJECT_ROOT_STATUS` may
  still be `configured: ...`, but there is no per-repo finding data to
  re-scope this run — the per-project report step (§ 3) renders
  `not audited this run (not in this week's chunk)` instead of a findings
  section, and is not diffed against the prior report in that case (nothing
  new to compare; the prior report's own findings simply carry forward
  unstated — the reader is pointed at the prior report's own timestamp
  rather than a synthesized diff against absent data).

### 3. `janitor-sweep` SKILL.md — the per-project local health report (step 6, alongside the existing two artifacts)

Step 6 currently states "one render pass that produces two artifacts"
(`$HEALTH_BODY`, `$REPORT_BODY`). Add a third, conditional on
`PROJECT_ROOT_STATUS` starting with `configured:` **and** the project repo
having been audited this run (§ 2's rotation-interaction note):

- `$PROJECT_HEALTH_BODY` — re-scoped, not newly computed. It is built from
  the *same* per-repo findings already rendered into `$PROJECTS_SECTION` for
  `$PROJECT_REPO_NAME` (the tier-by-tier `- **<repo>** [tag]: ...` lines and
  the repo's own Planning Documents sub-table), filtered to that one repo and
  re-headed as its own document:

  ```markdown
  # Project health — <project-repo-name> — <ts>

  Generated by the `janitor-sweep` skill's optional per-project report
  (opt-in: the project root has a root `ROADMAP.md`). Re-scopes this run's
  findings for <project-repo-name> from the workspace run's `## Projects`
  section — not independently computed. Full record (all repos, all tiers):
  the workspace run's own local report, `.agent/scratchpad/janitor/<sweep-report-file>`.

  <the same tier sections as the ## Projects section, filtered to this repo,
  same [New/Resolved/Unchanged/Not re-examined ...] inline tags, same
  Planning Documents sub-table, same "no findings this run" / tier-omission
  rules>
  ```

  This is stated explicitly per action 2: the per-project report is a
  **re-scoping** of the Projects section's own findings for one repo, never a
  second, independently-computed data source — so it inherits the #651
  coverage gate for free, exactly as the Projects section's inline tags do,
  because it is built from those same tagged lines.

- **Diff source (action 2)**: best-effort against the most recent prior local
  `.agent/scratchpad/janitor/<ts>-<project-repo-name>-health.md`, same rule
  and same "no prior report" wording project scope already uses today (§ 5's
  Project-scope bullet: `no prior report for this repo` /
  `prior report covered a different chunk`) — this report *is* project scope,
  narrowed to one repo, so it takes the identical diff rule rather than a new
  one. Concretely: since the body's findings are re-scoped from
  `$PROJECTS_SECTION`, which is already diffed per-finding in § 5, this
  report's diff state is **inherited from that computation**, not
  recomputed — the retention-pruned local-report lookup in § 5's Project
  scope bullet already reads `.agent/scratchpad/janitor/` for the repo's own
  prior local reports; the per-project `<ts>-<project>-health.md` files
  become part of that same lookup set for `$PROJECT_REPO_NAME` (they are
  local reports "for the same repo" per § 5's existing wording, so no new
  lookup logic is needed — just confirming in the skill text that this
  filename pattern participates in that glob/lookup).

- **Write** (mirrors `$REPORT_BODY`'s own write in step 6, same style):

  ```bash
  if [ "${PROJECT_ROOT_STATUS#configured:}" != "$PROJECT_ROOT_STATUS" ] && [ -n "${PROJECT_REPO_AUDITED_THIS_RUN:-}" ]; then
      PROJECT_REPORT="$REPORT_DIR/$(date '+%Y%m%dT%H%M%S')-$$-${PROJECT_REPO_NAME}-health.md"
      if ! printf '%s\n' "$PROJECT_HEALTH_BODY" > "$PROJECT_REPORT"; then
          PROJECT_HEALTH_STATUS="FAILED(project health write: could not write $(redact_text "$PROJECT_REPORT"))"
      else
          PROJECT_HEALTH_STATUS="written: $(redact_text "$PROJECT_REPORT")"
      fi
  fi
  ```

  **Write-failure naming (action 4)**: `FAILED(project health write: <reason>)`
  — the same "the report write is itself a state" convention § The status
  contract already applies to the primary local report
  (`FAILED(report write: <reason>)`) and to the workspace-scope commit
  (`FAILED(workspace publish: health write: ...)`), extended to this third
  durable local output with its own name so it is never confused with either.
  Unlike the primary report's write failure, this one is **not terminal for
  the run** — by the time it is attempted, the primary `$REPORT` has already
  been written successfully (this step runs after it), so a project-health
  write failure is reported alongside the run's other outcomes (step 8),
  never in place of them. This mirrors the existing rule that the
  workspace-scope publish failure (the sixth state) never invalidates the
  fifth state (the primary report write) — this is now a seventh, named the
  same way, with the same non-terminal relationship to the states before it.

  `$PROJECT_HEALTH_STATUS` is appended to `$REPORT` (the primary local
  report) alongside the existing `## Publish outcome` section (§ 4 below),
  so a reader of the primary report always sees whether a per-project report
  was attempted, written, or skipped, without opening a second file.

- Redact `$PROJECT_HEALTH_BODY` through `redact_text` before writing it, same
  as `$HEALTH_BODY`/`$REPORT_BODY` in step 6 today — it is built from the
  same unredacted-until-now findings text.

### 4. `janitor-sweep` SKILL.md — `--publish` flag, gating step 7 and `$HEALTH_BODY`

- **Usage section**: add `[--publish]` — "Commit `docs/health.md` and open a
  PR for the workspace scope (§ 7). Off by default (local-first) — every run
  writes the local report regardless; `--publish` only gates the
  commit-and-PR flow and the workspace-scope diff source (below)."
- **Step 6** (action 3): gate `$HEALTH_BODY`'s computation itself on
  `--publish`, not just its use — "`$HEALTH_BODY` is rendered only under
  `--publish`; on a default run it is never computed, since nothing consumes
  it (step 7 is not run — below)." The `| Variable | ... |` table's
  `$HEALTH_BODY` row gets a `(only under --publish)` qualifier.
- **Step 5** (action 3/decision 2), the Workspace-scope diff-source bullet
  gets a leading branch:

  > **When `--publish` was passed**: unchanged — diff against the last
  > **committed** `docs/health.md` (`git show HEAD:docs/health.md`), exactly
  > as today.
  > **When `--publish` was not passed (the default)**: diff against the most
  > recent prior **local** workspace-scope report
  > (`.agent/scratchpad/janitor/<ts>-sweep.md`'s own `## Workspace` section),
  > using the identical best-effort rule and identical "no prior report"
  > wording project scope already uses (§ 5's Project-scope bullet) — read
  > from the retention-pruned set (last 20 runs, step 1), and say so plainly
  > (`no prior local report` / `prior report predates retention`) rather than
  > rendering an empty diff as "no changes." First-run handling (empty
  > `PREV_HEALTH`, "first committed run" note) applies the same way in both
  > branches, with the note's wording adjusted to "first local run — no prior
  > report" when not publishing.

  This is decision 2 verbatim; the coverage-gate mechanism itself (which
  finding resolves vs. carries forward) is unaffected — orthogonal to where
  the previous-run text comes from, as the Issue Review already confirmed.

- **Report rendering** (action 3): the `## Workspace` section gains one
  stated line, directly under the check-status table, naming which source
  this run diffed against:

  ```markdown
  **Diffed against**: committed `docs/health.md` @ `<short-sha>` <!-- --publish -->
  **Diffed against**: local report `<file>` <!-- no --publish, prior report found -->
  **Diffed against**: none — first local run, no prior report <!-- no --publish, none found -->
  ```

- **Step 7** ("Publish the workspace scope") gets a one-line gate at its top:
  "**Runs only when `--publish` was passed.**" and the `## Publish outcome`
  section changes shape on the default path (action 3):

  ```bash
  if [ "$PUBLISH" = "1" ]; then
      # 7a–7h exactly as today; $PUBLISH_LINE assigned as today
      printf '\n## Publish outcome\n\n%s\n' "$(redact_text "$PUBLISH_LINE")" >> "$REPORT"
  else
      printf '\n## Publish outcome: not requested (--publish off)\n' >> "$REPORT"
  fi
  ```

  The heading itself carries the reason (per action 3's exact wording), so
  no body line is needed under it on the default path — a reader scanning
  headings sees the state without opening the section. `$PUBLISH_LINE` is
  never assigned in this branch (nothing to report — 7a–7h did not run).

- **Step 8** ("Report to the operator"): state the publish outcome using
  whichever heading was written (the PR URL / `FAILED(workspace publish:
  ...)` / `not requested (--publish off)`), and — new — state the
  project-health outcome (`$PROJECT_ROOT_STATUS` / `$PROJECT_HEALTH_STATUS`)
  on its own line: `no project configured on this checkout` /
  `<repo> has no root ROADMAP.md — no per-project report` /
  `<repo>: <written path>` / `<repo>: FAILED(project health write: ...)`.

### 5. `janitor-sweep` SKILL.md — Overview / Known limitations / Deferred rewrite

- **Overview**: replace "Workspace scope publishes: it commits a
  `docs/health.md` at the repo root and opens a PR" with the local-first
  framing — every run's durable output is the local report(s) under
  `.agent/scratchpad/janitor/`; the workspace-scope commit-and-PR is opt-in
  via `--publish`; add one sentence introducing the optional per-project
  local report (decision 3, one line, cross-referencing § 3 above).
- **Known limitations**: the first bullet ("the workspace half no longer
  is [ephemeral]") gets a qualifier — durable only when `--publish` was
  used; on a default run the workspace-scope portion is exactly as ephemeral
  as project scope always was. Add a bullet for the per-project report: same
  host-local/ephemeral limitation as project scope, since it is also never
  committed.
- **Deferred**: unchanged in substance (rollup shape, the weekly trigger)
  but reworded where it currently states "Workspace-scope publish is live...
  as of this change" to instead say publish is **opt-in** as of this change,
  and cross-reference #636 as the consumer of `--publish` for the unparked
  cloud trigger (issue decision 1's stated reason for keeping the flag
  rather than removing 7a–7h).

### 6. `ROADMAP.md`

- Row for #636: change `Status` from `planned` to `deferred`; `Notes`:
  "Backburnered 2026-09-21 (operator direction, #652) in favor of a local
  health report; `--publish` (#652) keeps the commit-and-PR path this issue
  will use once unparked."
- New row: this issue (#652) — "Local-first sweep: `--publish` opt-in,
  local diff source, optional per-project health report" — `Status`: the row
  is added once the PR lands, so `in progress` while this plan is being
  implemented (matches how #635's row read while it was open).
- New row: #653 (the what-next port) — `Status: planned`, `Notes`: "Health-
  report refresh trigger + per-area ranking for parallel agents; design draft
  first (per #653's own title)."
- (#651's own row: already landed per the Issue Review's Consequences check;
  confirm at implementation time whether a row for it is still missing and
  add one only if so — the issue text asks for "rows for this issue, #651
  and the what-next port," so do not skip it if absent.)

### 7. `.agent/knowledge/skill_workflows.md` and `principles_review_guide.md`

Both currently describe the durable output as always-committed
(`skill_workflows.md`: "workspace scope publishes a committed `docs/health.md`
via PR, project scope stays report-only"; `principles_review_guide.md`:
"the workspace-scope portion durably publishes as a committed, PR-reviewed
`docs/health.md`"). Per the issue's "Also in this PR" and per action 3,
rewrite both back to: **local report file by default; committed
`docs/health.md` only with `--publish`** — and add the third durable output
(the optional per-project local report) to `principles_review_guide.md`'s
Consequences Map row, since that row is what names write-failure states for
this skill's outputs (action 4 lands here too, in prose form: "a third
optional local output, the per-project health report, gated on the project
root having a `ROADMAP.md`; its write failure is named
`FAILED(project health write: <reason>)`, same convention, non-terminal for
the run").

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/manifest_fallback.sh` | Extract `manifest_bootstrap_identity()` from `manifest_config_dir()`'s inline parsing; `manifest_config_dir()` calls it internally (behavior-preserving refactor) |
| `.claude/skills/janitor-sweep/SKILL.md` | Usage: `--publish` flag. New step 1a: resolve project root. Step 5: `--publish`-gated workspace diff source. Step 6: gate `$HEALTH_BODY` on `--publish`; add `$PROJECT_HEALTH_BODY` third artifact + its write + `FAILED(project health write: ...)`; add "Diffed against" report line. Step 7: gated on `--publish`; `## Publish outcome: not requested (--publish off)` heading on the default path. Step 8: report project-health outcome. Overview / Known limitations / Deferred: reworded per § 5 above |
| `ROADMAP.md` | #636 row → `deferred` with reason; new rows for #652 and #653; confirm/add #651's row |
| `.agent/knowledge/skill_workflows.md` | Durable-output sentence → local-by-default, `--publish`-gated commit |
| `.agent/knowledge/principles_review_guide.md` | Consequences Map row: same wording fix + name the third (per-project) durable output and its write-failure naming |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Workspace vs. project separation | The project-root probe reads only the tracked pointer + existing resolver scripts; no project-specific content enters the workspace repo, and the mechanism stays generic across any project a clone is configured for. |
| Only what's needed | No new declaration file or schema (decision 3) — reuses `planning_doc_probe.sh`'s existing `ROADMAP.md`-presence probe and `resolve_repo_checkout.sh`'s existing resolution, and re-scopes already-computed Projects-section findings rather than computing new ones for the per-project report. |
| Human control and transparency | `--publish` off by default is itself the safety property (no surprise commits/PRs); the `## Publish outcome` heading and the "Diffed against" line make the non-publish path self-describing rather than silent about what didn't happen. |
| A change includes its consequences | The two knowledge docs and ROADMAP.md are updated in this same PR (§ 6–7); the new report's diff source, coverage-gate inheritance, and write-failure naming are all specified (§ 3), closing all four open actions from the Issue Review. |
| Capture decisions, not just implementations | This plan records the re-scoping rationale for the per-project report explicitly (action 2's "state this explicitly" instruction) rather than leaving it implicit in code. |
| Enforcement over documentation | No enforcement gap introduced — `--publish` absence is enforced by the flag simply not being passed by default; nothing relies on an agent remembering not to publish. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0003 — Project-agnostic workspace | Yes | `manifest_bootstrap_identity()` derives the project repo name from the tracked pointer at runtime; no project name is hardcoded anywhere in `manifest_fallback.sh` or `SKILL.md`. |
| 0013 — progress.md entry-type vocabulary | No | `janitor-sweep` remains the documented periodic-skill exception; this plan does not add a progress.md-writing obligation to it. |
| 0017 — Extend AGENTS.md to project repos | No | Not implicated — no project-repo agent guide is touched. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `manifest_fallback.sh`'s internal structure | Nothing external calls the old inline block directly (only `manifest_config_dir()` did, and it still does, via the new function) | Yes — behavior-preserving, no external caller to update |
| `janitor-sweep`'s durable-output shape | `skill_workflows.md`, `principles_review_guide.md` Consequences Map | Yes (§ 7) |
| `docs/health.md`'s publish cadence (no longer every run) | `ROADMAP.md`'s Health document line ("replaces it wholesale on every workspace-scope run") — this line becomes inaccurate once publish is opt-in | Yes — added as a follow-up item below; not silently left stale |
| ROADMAP.md rows for #636/#652/#653 | Nothing else references these rows by number | Yes (§ 6) |

**Follow-up caught during planning, folded into scope rather than deferred**:
`ROADMAP.md` line 19–21 ("first committed 2026-09-21 by the `janitor-sweep`
skill, which replaces it wholesale on every workspace-scope run") describes
the pre-`--publish` behavior. Update it during implementation to "...on every
`--publish` run" so it stays accurate — this is exactly the kind of
change-includes-its-consequences gap the Quality Standard asks not to leave
for review to catch.

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): `ROADMAP.md`'s Health-document
  provenance line (line 19–21, see Consequences above);
  `.agent/knowledge/skill_workflows.md`'s `janitor-sweep` row;
  `.agent/knowledge/principles_review_guide.md`'s Consequences Map row for
  `janitor-sweep`; `.claude/skills/janitor-sweep/SKILL.md` itself (Overview,
  Known limitations, Deferred, Usage, steps 1/5/6/7/8).
- **Agent-instruction candidates** (proposals only — operator decides): None
  — this issue is itself the instruction change; nothing further surfaced
  that would belong in a separate `.agent/knowledge/` doc.

## Open Questions

- [ ] Step-numbering choice: this plan inserts the project-root resolution
  as **step 1a** (not renumbered into the main sequence) specifically to
  avoid touching the many existing cross-references to "step 5", "step 6",
  "§ 7" elsewhere in the 1500-line skill file. Confirm at review-plan time
  that `1a` (rather than a full renumber) is acceptable style for this
  document — the skill's existing sub-lettering precedent (`7a`–`7h`) supports
  it, but this would be the first *top-level* lettered insertion.
- [ ] Whether `manifest_bootstrap_identity()`'s new exit-code table should be
  added to `manifest_fallback.sh`'s existing top-of-file usage comment as a
  second documented entry point (alongside `manifest_config_dir`), or folded
  into the same table since it shares exit code 3. Leaning toward a second,
  clearly-separated block — implementation should follow whichever reads
  less ambiguously once the diff is in hand.
- [ ] Whether the per-project report's filename should use the *repo* name
  (`<project-repo-name>`, e.g. `unh_marine_autonomy`) or a shorter "project"
  label if one is ever distinguished from "repo" by future work (#628's
  vocabulary draft doesn't currently draw that distinction) — this plan uses
  the repo name throughout, since decision 3's own wording ("the project
  root ... has a ROADMAP.md") and the two-root rule treat them as the same
  thing today.

## Estimated Scope

Single PR. Five files: one script refactor (`manifest_fallback.sh`), one
large skill-doc edit (`janitor-sweep/SKILL.md` — additive in most places,
touching Usage + steps 1/5/6/7/8 + Overview/Known limitations/Deferred), and
three doc updates (`ROADMAP.md`, two knowledge docs). No new scripts, no new
tests (the skill has no executable test harness of its own — verification is
by hand-running the sweep, per this skill's existing practice for prior
changes such as #648's first live run).
