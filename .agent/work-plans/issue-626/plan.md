# Plan: janitor-sweep redaction-class fixes (gate for #635)

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/626

## Context

Issue #626 collects 23 deferred review suggestions from PR #625 rounds 5-6.
The operator's 2026-09-18 comment (and the `## Issue Review` entry already on
this issue's `progress.md`) fixes scope for **this PR** to the **redaction
class only**: five confirmed bugs where an absolute host path or a credential
tail reaches stderr and is transcribed verbatim into the janitor-sweep report
that `#635` will publish as `docs/health.md` into graded repos. Everything
else on the issue (false-RED case sensitivity, path-traversal via symlinked
`config_path`, doc-accuracy nits, two named test-fixture gaps) stays deferred
on #626.

All five items were re-verified against current `HEAD` source during the
Issue Review pass; two (the `@`-in-password leak and the `REDACT_PATH_PREFIXES`
`=`-split corruption) were reproduced live in a sourced shell. None were
already fixed.

## Approach

1. **Stop the two pre-`redact.sh` refusals in `resolve_repo_checkout.sh` from
   naming an absolute path.** Match the wording `manifest_fallback.sh:107`
   already uses ("cannot load redact.sh beside it") instead of interpolating
   `$SCRIPT_DIR/redact.sh`.
2. **Stop a failed lock-file `exec` from leaking bash's own error text**
   (which bypasses the `say`/`_manifest_fallback_say` redaction funnel
   entirely) in both `resolve_repo_checkout.sh` and `manifest_fallback.sh`.
   Save/restore stderr around the `exec` so bash's built-in error message
   never reaches the real stderr, and report failure through the existing
   redacted `say()` call instead.
3. **Fix the `@`-in-password leak** in both `redact_url` and `redact_text` by
   widening the userinfo character class so the match is greedy up to the
   *last* `@` before the host, not the first.
4. **Fix the `REDACT_PATH_PREFIXES` first-`=`-split corruption** by splitting
   each `<prefix>=<replacement>` spec on the *last* `=` instead of the first,
   so a workspace path containing `=` is preserved intact as the prefix.
5. **Stop the janitor-sweep report-directory `mkdir` failure from printing an
   absolute path, and make it terminal.** Move `redact.sh` sourcing and
   `REDACT_PATH_PREFIXES` setup earlier in `janitor-sweep/SKILL.md` step 1
   (right after `$ROOT` is resolved), route the `mkdir` failure message
   through `redact_text`, and `exit 1` instead of falling through.

Ride-alongs (required, per operator scope):

6. **Close the lock fd for child `git` processes** so an inherited fd 9/8
   cannot hold the advisory lock open past the parent's own exit (a stalled
   or orphaned `git` child currently keeps the lock alive). Append `9>&-`
   (`8>&-`) to every `git` invocation that runs after the lock fd is opened,
   in both scripts.
7. **New `.agent/scripts/tests/test_redact.sh`**, in the shape of the
   existing `tests/test_*.sh` files (sourced-function tests, `pass`/`fail`
   counters, non-zero exit on any failure), covering: the `@`-in-password
   case for both `redact_url`/`redact_text`, a `REDACT_PATH_PREFIXES` spec
   whose value contains `=`, and the pre-existing simple-case behavior
   (so the file also serves as the dedicated, discoverable redact.sh test
   surface the issue named as missing).
8. **Fix the stale `AGENTS.md` `redact.sh` Script Reference row** — Ask-First
   already confirmed by the operator via #626's own framing (this row
   directly documents the refusal contract touched by item 1/2 above).

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/resolve_repo_checkout.sh:130-131,141` | Drop `$SCRIPT_DIR/redact.sh` from both bare-`echo` refusal messages; match `manifest_fallback.sh`'s "cannot load redact.sh beside it" wording. |
| `.agent/scripts/resolve_repo_checkout.sh:465-471` | Save/restore stderr around `exec 9>"$CACHE_DIR/.$REPO_NAME.lock"`; on failure, route a redacted message through `say()` and `exit 5` instead of letting bash's own error print the raw path. |
| `.agent/scripts/resolve_repo_checkout.sh:485,493,494,495,503,517,530,534,535` | Append `9>&-` to each `git`/`"${GIT_NET[@]}" git` invocation after the lock fd is opened, so children don't inherit fd 9. |
| `.agent/scripts/manifest_fallback.sh:205-208` | Same save/restore pattern around `exec 8>"$cache/.$repo.lock"` (use fd 6 for the saved copy — distinct from `resolve_repo_checkout.sh`'s fd 7, in case the two are ever nested). |
| `.agent/scripts/manifest_fallback.sh` (git calls in the manifest-clone path, e.g. `manifest_config_dir`'s clone/fetch/reset lines near :190-230) | Append `8>&-` to each, mirroring item 6. |
| `.agent/scripts/redact.sh:41` | `redact_url`: change `[^/@]*@` → `[^/]*@` so the greedy match reaches the *last* `@` before any `/`. |
| `.agent/scripts/redact.sh:57` | `redact_text`: change `[^/[:space:]@]+@` → `[^/[:space:]]+@` in the `sed -E` pattern, same fix. |
| `.agent/scripts/redact.sh:61-62` | `REDACT_PATH_PREFIXES` parsing: `prefix=${spec%%=*}` → `prefix=${spec%=*}`; `replacement=${spec#*=}` → `replacement=${spec##*=}` (split on the *last* `=`). |
| `.claude/skills/janitor-sweep/SKILL.md` (step 1, ~lines 92-112) | Move `REDACT_PATH_PREFIXES=("$ROOT=<workspace>")` (+ `$HOME` line) and a direct `source "$ROOT/.agent/scripts/redact.sh"` to immediately after `$ROOT` is resolved, before the `mkdir -p "$REPORT_DIR"` line. Change that line to `if ! mkdir -p "$REPORT_DIR"; then echo "FAILED(report directory: cannot create $(redact_text "$REPORT_DIR"))"; exit 1; fi`. Remove the now-redundant `REDACT_PATH_PREFIXES` assignment further down in step 2 (~line 136), since it is already set (idempotent duplicate, drop it to avoid drift). |
| `AGENTS.md` (Script Reference, `redact.sh` row) | "Both callers **refuse to run** when it is missing rather than printing around it" → "...when it is missing **or will not load** (exit 5) rather than printing around it". |
| `.agent/scripts/tests/test_redact.sh` (new) | New hermetic test file per item 7 above. |
| `.agent/scripts/tests/test_resolve_repo_checkout.sh` | Extend the existing "bad_redact"/missing-redact fixture (~lines 485-503) to assert stderr does **not** contain the absolute `$SCRIPT_DIR` path for either refusal. Add a new case near the existing lock section (~line 700, "6k") exercising an unwritable/unopenable lock-file path: assert exit 5, and assert stderr contains the `say()`-emitted, redacted message but never a raw bash `exec`/permission-denied line naming the absolute lock path. |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | The sweep report is read directly by operators and (via #635) published into other repos' `docs/health.md`; every fix in this plan removes a channel by which a host's real filesystem layout or a credential fragment could reach that surface unintentionally. |
| A change includes its consequences | Item 5's `mkdir` fix reorders `REDACT_PATH_PREFIXES` setup in `janitor-sweep/SKILL.md`; the plan explicitly removes the now-duplicate later assignment rather than leaving two copies to drift. Item 8 updates the one doc row that describes the exact refusal contract items 1-2 touch. |
| Test what breaks | Every one of the 5 redaction-class bugs plus the fd-inheritance ride-along gets a new or extended test (see Files to Change); none of the five items had prior test coverage per the Issue Review. |
| Only what's needed | The 17 deferred items (false-RED case sensitivity, path-traversal, lock-lifecycle-via-`manifest_config_dir`-return doc gap, doc nits, two named test-fixture gaps) are explicitly left off this PR per operator scope; this plan does not touch them. |
| Improve incrementally | This is scoped as its own gate PR ahead of #635, not folded into a larger rewrite. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0013 (progress.md entry-type vocabulary) | Yes | This plan's own `## Plan Authored` entry and all subsequent phase entries follow the vocabulary; no new entry types needed. |
| Others (0001-0012, 0014-0019) | No | This work is a bug-fix pass on existing shell scripts and skill docs; it does not touch worktrees, deployment mode, dispatch contracts, or layer chaining. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `redact.sh`'s `redact_url`/`redact_text` regexes | `test_resolve_repo_checkout.sh`'s inline simple-case assertions (~lines 887-907) must still pass unchanged | Yes — the fix only widens the greedy match; the existing single-`@`, no-`@`, and scp-form cases are unaffected (verified by re-deriving the regex behavior by hand; will re-run `test_resolve_repo_checkout.sh` after the edit). |
| `REDACT_PATH_PREFIXES` split direction | Every caller that composes a spec (`resolve_repo_checkout.sh` ~lines 220-223, `janitor-sweep/SKILL.md` ~line 136, `issue-triage/SKILL.md` if present) | Checked: all existing callers use replacement tokens (`<workspace>`, `<worktree>`, `~`) that never contain `=`, so splitting on the last `=` instead of the first is a pure bugfix with no behavior change for any existing caller. |
| `janitor-sweep/SKILL.md` step 1/2 ordering | `issue-triage/SKILL.md` and `audit-project/SKILL.md`, which the doc says share "the same snippet" for the workspace-root walk | Checked: neither has the `mkdir -p "$REPORT_DIR"` line (grep confirmed no hits in either file) — the bug is unique to `janitor-sweep/SKILL.md`. No follow-up needed; noted here so a future reader doesn't have to re-check. |
| Appending `9>&-`/`8>&-` to git calls | Any *other* fd the git calls might need to inherit | Checked: neither script opens any other high-numbered fd before these git calls; `9>&-`/`8>&-` only closes the lock fd, nothing else. |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): `AGENTS.md`'s `redact.sh` Script
  Reference row (item 8) — it currently says callers refuse to run "when it
  is missing," which is stale now that `resolve_repo_checkout.sh:141` and
  `manifest_fallback.sh`'s guard also refuse on a `redact.sh` that fails to
  *load* (source error / functions not defined). Fixing the row's wording
  and adding the exit code lands in this PR since the code it describes is
  being touched.
- **Agent-instruction candidates** (proposals only): None — this is a
  targeted bugfix pass; it does not surface a new pattern that belongs in
  `.agent/knowledge/` beyond what item 8 already covers.

## Open Questions

- Should the `9>&-`/`8>&-` fd-closing ride-along also cover the `rm -rf`,
  `mkdir -p`, and other non-`git` subprocess calls inside the locked regions
  (e.g., `find`, if any are added later)? This plan only closes the fd for
  `git` invocations, since those are the only long-running/network-facing
  children today and the ones the deferred note names explicitly.
- The deferred item "Manifest-cache lock releases when `manifest_config_dir`
  returns... read unlocked" (a `$(...)`-subshell scoping issue, not a
  redaction bug) is adjacent to item 6's `exec`/`flock` code but explicitly
  out of scope per the operator's framing — confirming it stays deferred and
  is not accidentally "fixed" as a side effect of the fd-inheritance change
  (the fd-inheritance fix does not touch where the lock is acquired, only
  what inherits it, so no overlap is expected).

## Estimated Scope

Single PR. Five small, independently testable script/doc edits plus one new
test file; no cross-repo or cross-layer impact.
