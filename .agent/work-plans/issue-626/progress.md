---
issue: 626
---

# Issue #626 — janitor-sweep: 23 deferred review suggestions from PR #625 rounds 5-6

## Issue Review
**Status**: complete
**When**: 2026-09-18 13:36 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)

**Issue**: #626
**Comment**: not posted (no GitHub read/write auth in this dispatch — host-injected context was used per the review-issue skill's zero-auth path; progress.md is the canonical record)
**Scope verdict**: needs-splitting — the operator has already split it: this review's job was to say which of the 23 items are the split.

### Operator framing (2026-09-18)

The upcoming sweep rewrite (#635) will publish `docs/health.md` into graded
repos, gated on closing the **REDACTION CLASS**: items where an absolute host
path or a credential reaches stderr that the sweep report transcribes. The
operator wants that class fixed first, as its own PR. Every item below was
re-verified against the current worktree source (`redact.sh`,
`resolve_repo_checkout.sh`, `manifest_fallback.sh`, the janitor-sweep and
issue-triage `SKILL.md`), not taken on the issue text alone — none had already
been fixed.

### In scope for the redaction-class gate PR

- **Resolver's two bare-`echo` refusals still print the absolute `redact.sh` path** — `resolve_repo_checkout.sh:130-131,141` — confirmed unchanged; these run *before* `redact.sh` is sourced, so they bypass `say()` entirely and print `$SCRIPT_DIR/redact.sh` verbatim. `manifest_fallback.sh:107`'s equivalent refusal deliberately names no path — the asymmetry is real.
- **A failed `exec 9>`/`exec 8>` prints the absolute lock path via bash's own error message, bypassing the redaction funnel** — `resolve_repo_checkout.sh:466` (`exec 9>"$CACHE_DIR/.$REPO_NAME.lock"`), `manifest_fallback.sh:205` (`exec 8>"$cache/.$repo.lock"`). Confirmed: neither exec's stderr is captured or routed through `say`/`_manifest_fallback_say` before the `&&`.
- **A password containing a literal `@` leaks its tail** — `redact.sh:41` (`redact_url`) and `:57` (`redact_text`, same regex shape). Reproduced live:
  `redact_url "https://user:pa@ssword@github.com/org/repo.git"` → `https://<redacted>@ssword@github.com/org/repo.git` — the tail `ssword@github.com` is untouched because `[^/@]*@` stops at the *first* `@`. No test exists for this at all (`.agent/scripts/tests/` has no `test_redact.sh`; only `test_resolve_repo_checkout.sh` exercises redact indirectly).
- **`REDACT_PATH_PREFIXES` splits on the first `=`, corrupting redaction when a workspace path itself contains `=`** — `redact.sh:59-65`. Reproduced live: a prefix spec `"/home/roland/proj=ects=<workspace>"` (i.e., a real path containing `=`) parses as prefix `/home/roland/proj`, replacement `ects=<workspace>`, so the real host path is never matched and the substitution instead corrupts the string (`ects=<workspace>=ects=<workspace>/foo/bar`). This is the mechanism the other two scripts rely on to strip `$MAIN_ROOT`/`$HOME` from every diagnostic — a bug here can leak the real host path into the report on affected hosts.
- **`mkdir -p "$REPORT_DIR" || echo "FAILED(...)"` prints the absolute `$REPORT_DIR` path and is not terminal** — `.claude/skills/janitor-sweep/SKILL.md:111-112`. Confirmed unchanged; this runs before `REDACT_PATH_PREFIXES` is even set up in that step, so the path reaches the report raw. (Its non-exit behavior is a second, non-redaction bug — see deferred item on arm consistency below; fixing the path leak does not require fixing the control flow, though the same line is being touched either way.)

### Deferred — stays on #626 for later (not redaction-class, or documentation/test-gap on unrelated code)

- `[x]` (already withdrawn 2026-09-14 by the reporting agent — the `${extra:+...}` word-splitting claim was untrue; no action)
- `declared_config_path` checked only for leading `/`/`..` — a committed symlink can still escape the clone dir (`manifest_fallback.sh:281` area) — real path-traversal concern, but not a stderr/report redaction issue; separate fix.
- Plan §Documentation & Instruction Impact omits the Field Mode addition from the AGENTS.md script-table note (`.agent/work-plans/issue-569/plan.md:428-436`) — stale plan doc, not code.
- fd 8/9 inherited by git children, holding the advisory lock past script exit (`flock -o` or explicit `{fd}>` close) — both scripts, still present. Adjacent to the exec-path-leak fix above (same lines) but a distinct bug class (lock lifecycle, not disclosure) — does not have to ride along; flagged for whoever does the exec-line rewrite as a cheap add-on if convenient, not required.
- `WORKTREE_GUIDE.md` never mentions the main-checkout hop / `$WORKSPACE_ROOT` — confirmed absent by grep; doc-only.
- `redact.sh` doesn't document that `git clone` writes a credential-bearing url verbatim into a scratch clone's `.git/config` (`redact.sh:25-27`, janitor-sweep `SKILL.md:176-185`) — this is a *disk-state* exposure (the cache directory), not the stderr-into-report path the gate PR is scoped to; it's a documentation gap about a different vector. Deferred.
- Branch-vs-clone-branch conflation in `manifest_fallback.sh`'s `declared_branch` check (~line 304) can false-fail a supported config — correctness bug, not redaction.
- Exit-code asymmetry between the file-absent and source-failure siblings for a missing `manifest_fallback.sh` (`resolve_repo_checkout.sh:306-308`, now the `else` at line ~307) — that branch's message already goes through `say()` after `REDACT_PATH_PREFIXES` is set, so it is *not* a redaction gap, just a behavioral inconsistency.
- `grep|cut|awk` bootstrap.yaml parsing inherits `setup_layers.sh`'s brittleness but turns a parse quirk into a hard failure (`manifest_fallback.sh:246-248` area) — correctness, not redaction.
- Manifest-cache lock releases when `manifest_config_dir` returns (every call site is a command substitution), so the returned repos dir is read unlocked — documented for the janitor-repos cache, not the manifest-repo cache (`manifest_fallback.sh:178,181` area) — concurrency/doc gap, not redaction.
- Test gap: the "redact.sh will not load" fixture in `test_resolve_repo_checkout.sh:487-495` doesn't isolate the rc check it was added for (a pre-existing `declare -F` guard alone answers exit 5). Named by the issue's own notes as a test-only gap, separate from the redaction class.
- Test gap: the `*)` arm closing the rc-127 fallthrough (`resolve_repo_checkout.sh:302`) has no dedicated test. Same note as above.
- Retention sentence in `janitor-sweep/SKILL.md` cites "step 3" for something that's step 1, and leans on name-sortability where the actual mechanism is `ls -1t` (`SKILL.md:399-400` area) — doc-accuracy nit.
- `AGENTS.md`'s `redact.sh` row still says callers refuse "when it is missing," not the current "missing or will not load," and is the only row in that table describing a refusal contract with no exit code — confirmed still stale by reading the current row. Directly describes the mechanism the gate PR touches; **recommend bundling as a 1-line fix** since it documents exactly the code being changed, but not required by the redaction-class scope itself.
- `AGENTS.md`'s `workspace_root.sh` row doesn't mention the main-checkout hop being gated on the destination actually being a workspace root, nor that it redirects even an explicit `$WORKSPACE_ROOT` — doc-only, unrelated to redaction.
- `test_workspace_root.sh`'s `walk_up_from` fixture isn't the skills' snippet verbatim, and its "layer worktree" case doesn't match what `worktree_create.sh` actually produces — test-fixture accuracy, not redaction.
- Failure-arm inconsistency within one copy-pasted block: the `if ! source manifest_fallback.sh` arm `exit`s, but the four `case "$rc"` arms below it only `echo` — confirmed present in both `.claude/skills/janitor-sweep/SKILL.md` (~lines 140-159) and `.claude/skills/issue-triage/SKILL.md` (~lines 100-112). Same general pattern as the `mkdir -p` non-terminal arm above but different lines; not required to ride along, though a future "make every FAILED arm terminal" pass would naturally cover both.
- URL-key case sensitivity: `_manifest_fallback_url_key` lowercases the host (`_manifest_fallback_url_host`) but not `<owner>/<repo>` (`_manifest_fallback_repo_path`), so a bootstrap differing only in owner/repo case still hard-fails at exit 5 (`manifest_fallback.sh:338-363`, confirmed by reading both helper functions). This is the same **false-RED** class PR #625's round-5 must-fix 2 closed — explicitly a different class from redaction per the issue's own "Notes for whoever picks this up." Deferred to a false-RED-focused pass.

### Verification method

Read `redact.sh`, `resolve_repo_checkout.sh`, `manifest_fallback.sh`, both
`SKILL.md` files, and the relevant `AGENTS.md` rows at current `HEAD`
(`05f0793`). Reproduced the two most load-bearing class bugs live (password-
with-`@` leak in both `redact_url`/`redact_text`; `REDACT_PATH_PREFIXES`
`=`-splitting corruption) with a sourced shell rather than trusting the
issue's line numbers. No item was already fixed.

### Actions
- [ ] Fix the redaction-class items (5 above) in one PR, gating the #635 sweep-report publish: bare-echo `redact.sh`-missing refusals naming a path (`resolve_repo_checkout.sh:130-131,141`), unredacted `exec 9>`/`exec 8>` failure paths (`resolve_repo_checkout.sh:466`, `manifest_fallback.sh:205`), the `@`-in-password leak in both `redact_url`/`redact_text` (`redact.sh:41,57`), the `REDACT_PATH_PREFIXES` first-`=`-split corruption (`redact.sh:59-65`), and the `mkdir -p "$REPORT_DIR"` absolute-path leak (`janitor-sweep/SKILL.md:111-112`).
- [ ] Add `test_redact.sh` (none exists) covering at minimum: `@` in password, a path-prefix spec whose value contains `=`, and the existing simple-case behavior — closing the "tests exercise only the simple form" gap named in the issue.
- [ ] Keep the remaining ~17 items open on #626 for later passes (false-RED url-key case-sensitivity, path-traversal via symlinked `config_path`, lock-lifecycle/fd-inheritance, doc-accuracy nits, and the two named test gaps for the round-5 `*)`/will-not-load arms).
- [ ] Optional, low-cost bundle candidate for the gate PR (not required): fix the stale `redact.sh` row in `AGENTS.md` ("when it is missing" → "missing or will not load", plus the exit code) since it describes the exact code the gate PR touches.

## Plan Authored
**Status**: complete
**When**: 2026-09-18 13:47 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)

**Plan**: `.agent/work-plans/issue-626/plan.md` at `858744a`
**Branch**: feature/issue-626 at `858744a`
**Phases**: single

### Open questions
- [ ] Should the fd-close ride-along (`9>&-`/`8>&-`) also cover non-`git` subprocess calls in the locked regions, or only `git` (as scoped)?
- [ ] Confirm the fd-inheritance fix does not overlap with the separately-deferred "manifest_config_dir lock releases on subshell return" item — plan states no overlap expected.

## Plan Review
**Status**: complete
**When**: 2026-09-18 13:56 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-626/plan.md` at `858744a`
**PR**: PR-less
**Verdict**: changes-requested

### Evaluation

| Dimension | Verdict | Notes |
|---|---|---|
| Scope | Good | Tightly scoped to the operator's 2026-09-18 named redaction-class items + 3 ride-alongs; correctly excludes the other ~17 deferred items. |
| Issue alignment | Good | Matches the `## Issue Review` entry's "in scope"/"deferred" split exactly; every in-scope item is addressed, no deferred item is silently touched. |
| File targeting | Needs work | Core fix targets verified correct against current `HEAD` source (see Findings 3-4 for regex/split verification). Two test-targeting issues found (Findings 1-2). |
| Consequences | Good | The three consequence rows were independently re-verified: no other `REDACT_PATH_PREFIXES` caller uses a `=`-bearing replacement token; `issue-triage`/`audit-project` don't share the `mkdir -p "$REPORT_DIR"` line; no other fd is opened before the git calls that get `9>&-`/`8>&-`. |
| Documentation & instruction impact | Good | Non-silent; AGENTS.md redact.sh row update correctly scoped as a required same-PR edit. |
| Principle alignment | Good | "Test what breaks" is the one weakened by Finding 1 (see below) — the test plan for item 1 doesn't actually exercise the code being changed. |
| ADR compliance | N/A | No ADRs triggered; plan's own table is accurate. |
| ROS conventions | N/A | Workspace-repo shell/doc fix, not a ROS package change. |

### Findings

1. **[File targeting / Test what breaks]** — The Files-to-Change row "Extend the existing 'bad_redact'/missing-redact fixture (~lines 485-503)" targets the wrong fixture. `.agent/scripts/tests/test_resolve_repo_checkout.sh:489-503` (`no_redact`/`bad_redact`) copies and sources `manifest_fallback.sh`, testing *that* script's own redact.sh guard (`manifest_fallback.sh:107`, whose wording is not being changed by this plan). There is **no existing fixture at all** for `resolve_repo_checkout.sh`'s own two bare-echo refusals at lines 131/141 — the actual code item 1 changes. Grepped the whole test file for any copy-and-execute of `resolve_repo_checkout.sh` itself with `redact.sh` removed: none exists. Implementation must **add new** cases (executing, not sourcing, a copy of `resolve_repo_checkout.sh` with `redact.sh` missing/broken) rather than "extend" 489-503, or item 1's fix ships with zero test coverage. — `plan.md:81`, `.agent/scripts/tests/test_resolve_repo_checkout.sh:489-503`

2. **[File targeting]** — The new "direct `source \"$ROOT/.agent/scripts/redact.sh\"`" the plan adds to `janitor-sweep/SKILL.md` step 1 (Files-to-Change row 4) has no exit-code/`declare -F` guard, unlike the pattern items 1-2 establish in `resolve_repo_checkout.sh`/`manifest_fallback.sh`. A missing or broken `redact.sh` at that point would fail the bare `source` and print bash's own unredacted message naming the absolute `$ROOT/.agent/scripts/redact.sh` path — reintroducing, in new code, the exact bug class (`$SCRIPT_DIR`/path leak from a bare `source`/`echo`) that items 1-2 are fixing elsewhere in this same PR. Should check the source's exit status (and that `redact_url`/`redact_text` are defined) and fail with a redaction-safe message, or route the failure the same way `resolve_repo_checkout.sh:131,141` now will. — `plan.md:78`, `.claude/skills/janitor-sweep/SKILL.md` step 1

3. **[Documentation nit]** — The Files-to-Change row for removing the "now-redundant `REDACT_PATH_PREFIXES` assignment" labels its location as "further down in step 2 (~line 136)". Line 136 is confirmed correct, but it is inside **step 1** ("### 1. Resolve the workspace root and the report directory", which runs through line 239); step 2 doesn't begin until line 240. No impact on implementation (the line number is right), but the section label should be corrected so a future reader isn't misdirected. — `plan.md:78`

4. **[File targeting nit]** — The `manifest_fallback.sh` fd-close ride-along row cites "git calls in the manifest-clone path... near :190-230", but the actual git invocations after the lock (`exec 8>` at line 205) are at lines 223 (`remote get-url origin`), 244 (`fetch`), 245 (`reset --hard`), and 251 (`clone`) — outside the cited range, and the `remote get-url` call at 223 isn't named at all. Not a correctness problem (the plan says "e.g." and the AGENTS.md consequences check is otherwise sound), but worth calling out so the implementer greps for every git call after the lock rather than trusting the cited line range. — `plan.md:74`, `.agent/scripts/manifest_fallback.sh:223,244,245,251`

### Verification notes (no action needed — recorded so it isn't re-derived)

- Manually traced the widened userinfo regexes (`redact_url`: `[^/@]*@`→`[^/]*@`; `redact_text`: `[^/[:space:]@]+@`→`[^/[:space:]]+@`) against several two-URL-on-one-line inputs. They cannot merge two distinct URLs' credentials into one match: every `scheme://` prefix contains a literal `/`, which the widened character class still excludes, so the greedy run for one URL's userinfo always halts at the next URL's `://` (or at any `/` in the first URL's own path). Confirmed the existing simple-case assertions at `test_resolve_repo_checkout.sh:884-907` (single-`@`, no-`@`, scp-form) are unaffected by the widening, matching the plan's Consequences claim.
- Verified the `REDACT_PATH_PREFIXES` last-`=`-split fix is a correctly paired change: `prefix=${spec%=*}` (single `%`, shortest-suffix removal) now splits at the *last* `=`, and `replacement=${spec##*=}` (double `##`, longest-prefix removal) takes only the text after that same last `=` — consistent with "replacement tokens are `=`-free but paths may contain `=`". No existing caller (`resolve_repo_checkout.sh:220-223`, `janitor-sweep`/`issue-triage` `SKILL.md`) uses a `=`-bearing replacement, confirmed by grep across the whole tracked tree.
- Both Open Questions in the plan are settled by the code as the plan already asserts: (1) only `git` calls are long-running/network-facing inside the locked regions of both scripts (confirmed by reading every call between each `exec 9>`/`8>` and its matching unlock — `rm -rf`/`mkdir -p` are short-lived and not orphan-risks), so scoping the fd-close to `git` only is correct; (2) the fd-inheritance fix and the deferred "`manifest_config_dir` lock releases on subshell return" item are unrelated mechanisms — the former closes an explicit fd for a *child process*, the latter is the *parent subshell itself* exiting (via `$(...)`) and closing all its fds, including the lock fd, on return. No overlap.

### Summary

The plan's diagnosis and five core redaction fixes are accurate against current `HEAD` — every claimed bug was independently re-derived from source, and the regex/split fixes are correct. Two must-fix gaps: the test-extension instruction for item 1 targets the wrong existing fixture (no test currently covers the code being changed), and the new direct `source redact.sh` line the plan adds to `janitor-sweep/SKILL.md` lacks the failure guard the rest of this same PR is establishing as the pattern. Both are fixable in minutes during implementation; recommend the implementer read this entry alongside the plan rather than requiring a plan rewrite.

### Recommended Actions

- [ ] Add new (not "extended") test cases in `test_resolve_repo_checkout.sh` for `resolve_repo_checkout.sh`'s own missing/broken-`redact.sh` refusals (lines 131/141), executing a copy of the script itself rather than sourcing `manifest_fallback.sh`.
- [ ] Add an exit-status/`declare -F` guard around the new direct `source ".../redact.sh"` in `janitor-sweep/SKILL.md` step 1, matching the guard pattern in `resolve_repo_checkout.sh`/`manifest_fallback.sh`.
- [ ] Fix the plan's "step 2 (~line 136)" label to "step 1 (~line 136)" for the redundant `REDACT_PATH_PREFIXES` assignment being removed.
- [ ] When doing the `manifest_fallback.sh` fd-close ride-along, grep for every git call after `exec 8>` (lines 223, 244, 245, 251) rather than relying on the cited "~190-230" range.
