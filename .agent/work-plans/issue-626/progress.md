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
