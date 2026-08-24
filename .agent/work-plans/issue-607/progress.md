---
issue: 607
---

# Issue #607 — Flip dispatch-mode default from container to in-process (auto mode retires #545's premise)

## Issue Review
**Status**: complete
**When**: 2026-08-24 09:26 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #607
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Fact-check against source

Verified the issue's quotations and claims against the actual files rather
than trusting them:

- **Quoted passages TRUE, verbatim.** `.claude/skills/run-issue/SKILL.md`
  (~L98-100) reads "**Lean toward `container`**… whenever keeping the
  operator **out of the approval loop** matters" and (~L92) "That prompt
  elimination is the biggest practical reason to use it." `.agent/knowledge/skill_workflows.md`
  (~L131-133) reads "**Fan-out goes to containers, not in-process agents.**
  Review/exploration fan-out via in-process Agent-tool sub-agents floods the
  operator with permission prompts; container dispatch runs sandboxed and
  prompt-free."
- **Section 4 masking `layers/main/*_ws/install`: TRUE.** `docker_run_agent.sh:369`,
  "4. Anonymous volumes for build/install/log in each layer workspace."
- **`--context-file` covers `review-issue` not `triage-reviews`: TRUE.**
  `triage-reviews/SKILL.md` calls `gh pr view` / `fetch_pr_reviews.sh` /
  `gh issue view` directly with no context-file path — matches run-issue's own
  documented limitation.
- **No host Agent tool in container: TRUE**, consistent with run-issue/SKILL.md's
  own text.
- **#581 / #558 / #585: UNVERIFIABLE locally** — no local trace (git log,
  `.agent/knowledge/`) of these issue numbers; this dispatch has no GitHub
  read auth to check them directly. Not a reason to doubt the issue, just a
  gap this review can't close.
- **Root-owned volumes fix: TRUE but mis-cited.** Only **#604** appears in
  git log for the worktree-volume-chown fix (`8789f1e fix: chown the
  dispatched worktree's anonymous volumes`, `7596d74 Shield dispatched
  worktree build artifacts from host (#602)`); **`#606` appears nowhere** in
  history. The root-owned symptom itself traces to `#566` in code comments,
  not `#604`/`#606`. Minor citation error, doesn't affect the substance of
  the claim.
- **Auto mode's existence/effect: corroborated by this very session** — the
  dispatching orchestrator's own session carries a system reminder
  "Auto Mode Active" that biases toward not stopping for approvals, matching
  the issue's premise.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Issue is explicitly framed as a governance proposal "raised for the operator rather than made unilaterally"; non-goals section keeps scope honest. |
| Enforcement over documentation | Watch | Purely advisory prose in both target files — nothing mechanically enforces the mode choice, matching how the original #545 preference was also advisory-only. Consistent, not a regression. |
| Capture decisions, not just implementations | Watch | #545 wasn't ADR'd, so no precedent requires one here either; the proposed change already plans to record "auto mode" as the reason the advice moved, which satisfies the spirit at doc level. |
| A change includes its consequences | Action needed | See below — the `run-issue/SKILL.md` heading currently reads "**Choosing a mode (#545).**"; the rewrite should update that inline citation to the new decision (#607) so a future reader doesn't find a dangling reference to the superseded issue. Also verified: no other adapter file (`.github/copilot-instructions.md`, `.agent/instructions/`) echoes this advice, so no additional cascade needed. |
| Only what's needed | OK | Non-goals section explicitly excludes touching `dispatch_subagent.sh` and doesn't propose deprecating containers. |
| Improve incrementally | OK | Single PR, two files, scoped edits. |
| Test what breaks | OK | N/A — advisory prose, no enforced logic to test. |
| Workspace vs. project separation | OK | Both target files are workspace infra (skills/knowledge), correct repo. |
| Primary framework first, portability where free | Action needed | The issue's premise (auto mode eliminates prompts) is Claude-Code-specific and further conditional on the **operator's session actually running in auto mode** — not all Claude Code sessions do. The proposed replacement text ("in-process becomes the default") should be conditioned on auto mode being active, not stated as an unconditional default; otherwise an operator in normal (non-auto) permission mode gets misled back into the exact prompt-flood problem #545 was written to solve. This is the one substantive content gap for plan-task to close. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| 0001 — Adopt ADRs | Watch, not required | This reverses operational guidance future agents rely on, which is ADR territory in spirit — but the original #545 decision was never captured as an ADR either, so requiring one now would be an inconsistent bar. Not blocking. |
| 0003 — Project-agnostic workspace | OK | Both files are generic workspace dispatch tooling, not project-specific. |
| 0015 — Dispatch handoff context contract | Not triggered | Issue doesn't touch the `--context-file` read/write mechanism itself, only the mode-selection advice around it. |

### Consequences

- Update the `(#545)` citation in `run-issue/SKILL.md`'s "Choosing a mode" heading to reference this issue once the text is replaced.
- No other files need updates — confirmed no adapter file duplicates the advice being changed.

### Actions
- [ ] Condition the new "in-process is the default" guidance on the operator's session actually running in Claude Code auto mode — state the fallback (container, or "expect prompts") for non-auto-mode sessions and non-Claude host runtimes, so the rewrite doesn't silently reintroduce the prompt-flood problem #545 solved.
- [ ] Update the inline `(#545)` issue citation in `run-issue/SKILL.md`'s "Choosing a mode" heading to point at this issue.
- [ ] Correct the citation in the issue body itself (or in the resulting doc text) from "#604 / #606" to "#604" for the root-owned-volume fix — `#606` does not appear in git history.

**Correction (from plan-task, 2026-08-24)**: the #606 action item above is
superseded — #606 is an open PR (fixing #604) created by the host after this
review ran; it is not a citation error. The plan leaves that citation as-is.

## Plan Authored
**Status**: complete
**When**: 2026-08-24 09:31 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-607/plan.md` at `8dd6e01`
**Branch**: feature/issue-607 at `8dd6e01`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.

## Plan Review
**Status**: complete
**When**: 2026-08-24 09:33 -04:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-607/plan.md` at `8dd6e01`
**PR**: PR-less (`--issue` mode; no plan PR pushed)
**Verdict**: changes-requested

Note on independence: the `## Plan Authored` entry's `**By**` agent-name portion
("Claude Code Agent") matches this reviewer's `$AGENT_NAME`, which would trigger
the skill's self-review annotation — but that heuristic collides in this
workspace because every dispatched agent shares one `$AGENT_NAME`. This is a
separate fresh-context sub-agent on a different model (plan: Sonnet; review:
Opus), so the annotation is deliberately omitted as a false positive.

### Evaluation

| Dimension | Verdict | Notes |
|---|---|---|
| Scope | Good | Two files, prose-only, matches the issue's non-goals; no tooling change. |
| Issue alignment | Needs work | Covers proposal items 1 and 2; item 3 ("record auto mode as the reason") lands only in `run-issue/SKILL.md`, not in `skill_workflows.md`. |
| File targeting | Needs work | Misses a third stale surface inside `skill_workflows.md` itself (~L102-107). |
| Consequences | Needs work | The consequences grep keyed on "prompt-free"/"permission prompt" and so could not have found the stale line, which uses neither phrase. |
| Documentation & instruction impact | Good | Section present and non-silent; candidates correctly framed as "None — this change *is* the instruction update". |
| Principle alignment | Needs work | "A change includes its consequences" — the plan's own do-not-touch fence preserves text the change makes false. "Never document from assumptions" — the load-bearing inheritance claim is asserted, not evidenced. |
| ADR compliance | Good | Agree ADR not required (parity with #545's prose-only precedent); 0003 and 0015 correctly assessed. |
| ROS conventions | N/A | Workspace instruction files only. |

### Findings

1. **[File targeting / Consequences]** — `.agent/knowledge/skill_workflows.md`
   ~L102-107 (§ Sub-agent dispatch, the two-line code-block comment) reads
   `# In-process (fast; no filesystem isolation):` and
   `# Container (isolation; use for implementation-heavy phases):`. The second
   comment is the retired advice in miniature: implementation-heavy phases are
   exactly the class (`implement` / `address-findings`) the new default
   reassigns to in-process. Left as-is, the file contradicts itself ~25 lines
   apart. Add it to step 2's edit list. Note *why* the plan's consequences
   grep missed it: it searched "prompt-free" / "permission prompt", and this
   line contains neither — worth widening the check to the phrase
   "implementation-heavy" and to bare `--mode container` occurrences before
   declaring the surface list closed.

2. **[Principle alignment — a change includes its consequences]** — Step 1's
   "do not touch the in-process/container bullet definitions (~L76-91)" fence
   is drawn in the wrong place: it protects three statements the change
   falsifies.
   - L77 heads the in-process bullet `**in-process** for **quick / cheap
     phases**` — a scoping label that directly contradicts "in-process is the
     default".
   - L83-86's `**Caveat:** an in-process Agent phase … can prompt the operator
     — phase after phase, edit after edit` is stated unconditionally; under
     the new text it is true only outside auto mode and must carry the same
     condition.
   - L87 heads the container bullet `**container** for **isolation *and*
     prompt-free dispatch**`, and the "biggest practical reason" claim the
     plan does carve out actually spans L90-92 — partly outside the L76-91
     range the fence names. Redraw the fence: the whole L77-96 bullet pair is
     in scope; the genuinely orthogonal material is the `--context-file`
     paragraphs above and the background/freshness paragraphs below.

3. **[Principle alignment — safety reasoning]** — Step 1 instructs that the
   sandbox-boundary paragraph (L104-110) be kept "content unchanged" while the
   surrounding recommendation is inverted. As written today that paragraph is a
   *caution against* reaching for container ("keep it in mind before dispatching
   anything that processes untrusted input") — because its job was to temper
   "Lean toward `container`". Bolted onto text that now says *choose* container
   for untrusted input, the two read as contradictory advice about the same
   case. The reasoning survives the flip, but the hinge does not: rewrite it to
   the affirmative form — when you dispatch untrusted input to a container, the
   sandbox boundary is the *whole* of your containment, because the dispatched
   agent runs with broad tool access under a long-lived token and no per-call
   human check sits behind it. Preserve every clause of the reasoning; change
   only what it is warning you about.

4. **[Principle alignment — safety reasoning, mirror case]** — The plan attaches
   the safety note only to the container path, which under the new default is
   the *rare* path. Nothing in the plan says what contains an in-process phase
   running under auto mode — which is likewise a broad-tool-access agent
   executing without per-call approval, contained by neither a sandbox nor the
   operator's eye, but by the host permission policy/allowlist, the worktree,
   and the surviving `run-issue` checkpoints. Without a sentence to that effect
   the rewritten section reads as "auto mode removed the prompts", from which a
   future agent may infer "…so the risk went away too". The issue asserts that
   auto mode removes the *prompt* premise; it does not claim the containment
   reasoning transfers. Add the one-sentence statement of what does the
   containing on the new default path.

5. **[Issue alignment / documentation accuracy]** — The conditioning is
   unactionable as specified: the plan writes the advice as conditional on auto
   mode while explicitly declining to say how a reader determines which case it
   is in ("no new detection mechanism"). The reader here *is* the agent making
   the choice. There is a cheap observable tell — the Claude Code host session
   carries an auto-mode indicator (the Issue Review corroborated exactly this
   from the orchestrator's own session) — and the text should name it. Two
   wording requirements: (a) state the tell; (b) make the fallback fail-safe —
   "if you cannot confirm auto mode is active, the container-leaning guidance is
   in force." As drafted the headline is the in-process default, so an uncertain
   reader lands on the prompt-flooding branch, which is the asymmetry #545
   existed to prevent.

6. **[Documentation accuracy]** — "in-process `Agent`-tool sub-agents inherit
   the session's permission mode" is the load-bearing premise of the whole
   change and is stated as a mechanism claim about Claude Code internals that
   this workspace cannot verify from source (the Issue Review verified that auto
   mode *exists*, not that it is inherited). Real evidence does exist and is
   better: the issue's own #604 lifecycle — seven phases driven in-process under
   auto mode, zero prompts for the dispatched work. Frame the new text on that
   observed behaviour rather than on an asserted inheritance rule, per AGENTS.md
   "never document from assumptions".

7. **[Issue alignment — suggestion]** — The issue's proposal item 3 asks that
   auto mode be recorded as the reason the advice moved. The plan does this only
   in `run-issue/SKILL.md` (via the `(#545)` → `(#607)` citation swap).
   `skill_workflows.md` would carry the new rule with no trace of why it
   changed. Add a short parenthetical citation there too — a reader arriving at
   the knowledge file first should not have to find the skill file to learn the
   rule was reversed deliberately.

8. **[Scope — suggestion, non-blocking]** — Two surfaces were checked and are
   correctly *not* in scope; record them so review-code does not re-litigate:
   `.claude/skills/review-plan/SKILL.md:435` already says "`--mode container`
   for **isolation-worthy** implementation work" (already keyed on isolation,
   consistent with the new framing), and `.devcontainer/agent/README.md:21`
   describes the container's own no-prompt property (a true statement about the
   container, not mode-choice advice). `presentations/raising-agents/slides.md:308`
   is a delivered-talk record, not live guidance — leave it.

**Confirmed accurate**: the plan's handling of the `#606` citation is correct —
#606 is an OPEN PR ("Chown the dispatched worktree's anonymous volumes, and
guard the mount-to-chown pairing") fixing OPEN issue #604. Verified with `gh`
during this review. Leave the citation untouched, as the plan says.

### Summary

Structurally the plan is right — correct files, correct scope, correct non-goals,
and it already absorbed the Issue Review's central content fix. But the
deliverable here *is* wording, and three of the four wording judgements are
underspecified in ways that would ship a self-contradicting document: a stale
line left standing inside one of the two target files, a do-not-touch fence that
preserves text the change falsifies, a safety paragraph told to stay verbatim
while its subject inverts, and a conditional with no way for the reader to
resolve which branch applies to them. All are cheap to fix at plan level.

### Recommended Actions

- [ ] Add `skill_workflows.md` ~L102-107's code-block comments to step 2's edit list (finding 1).
- [ ] Redraw step 1's do-not-touch fence to include the full L77-96 bullet pair (finding 2).
- [ ] Rewrite the sandbox-boundary paragraph's hinge to the affirmative, preserving every clause of its reasoning (finding 3).
- [ ] Add a sentence stating what contains an in-process phase under auto mode (finding 4).
- [ ] Name the auto-mode tell and make the fallback fail-safe ("cannot confirm → container guidance is in force") (finding 5).
- [ ] Ground the prompt-free claim in the observed #604 lifecycle rather than an asserted inheritance mechanism (finding 6).
- [ ] Carry a `(#607)` reason-citation into `skill_workflows.md` as well (finding 7).

### Findings
- [ ] (must-fix) Third stale surface: `skill_workflows.md` ~L102-107 "Container (isolation; use for implementation-heavy phases)" left contradicting the new rule — `plan.md:66-75`
- [ ] (must-fix) Do-not-touch fence preserves falsified text (L77 "quick/cheap phases", L83-86 unconditional Caveat, L87 container heading) — `plan.md:59-63`
- [ ] (must-fix) Safety paragraph kept "content unchanged" while its subject inverts; hinge must move to the affirmative — `plan.md:50-55`
- [ ] (must-fix) New default path has no containment statement — safety note attached only to the now-rare container path — `plan.md:50-55`
- [ ] (must-fix) Auto-mode conditioning gives the reader no tell and no fail-safe fallback direction — `plan.md:37-49`
- [ ] (suggestion) Permission-mode inheritance asserted as mechanism; ground it in the observed #604 lifecycle instead — `plan.md:12-15`
- [ ] (suggestion) Record the auto-mode reason in `skill_workflows.md` too, not only in `run-issue/SKILL.md` — `plan.md:66-75`
- [ ] (suggestion) Record review-plan/SKILL.md:435 and .devcontainer README as checked-and-out-of-scope so review-code need not re-litigate — `plan.md:76-80`

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-08-24 09:55 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-607 at `d8d50d7`
**Mode**: pre-push
**Depth**: Standard (reason: governance-touching instruction files that other agents read as authority; ~79 net prose lines across two skill files + one knowledge file)
**Must-fix**: 6 | **Suggestions**: 6
**Round**: 1 | **Ship**: continue — the change is inert as written (the auto-mode tell it names is operator-facing, not agent-observable), its safety paragraph claims containment that does not exist, and its fail-safe branch routes `review-code` to a mode the next bullet says cannot host it

Specialists: static analysis (no linter profile for `.md`; pre-commit clean on all
four changed files), governance, plan drift, Claude Adversarial x2 (Lens A + Lens B).
Local Adversarial skipped: model `qwen3.5:35b` not pulled. Copilot off (default).

### Findings
- [x] (must-fix) Auto-mode tell is not agent-observable: the text points at the session's permission-mode indicator, a terminal UI element rendered for the operator, so a careful reader must answer "cannot confirm" every time and fall through to container — the new default never fires. The signal the agent can actually read is the injected `While auto mode is active:` system reminder. Re-opens Plan Review must-fix 5 (nominally closed) — `.claude/skills/run-issue/SKILL.md:97-99`, `.agent/knowledge/skill_workflows.md:131-132`
- [x] (must-fix) In-process containment sentence overstates on all three clauses: auto mode is precisely what stands the permission policy down; `.claude/settings.json` carries an `allow` array only, no `deny` anywhere (verified), so the allowlist refuses nothing; the worktree confines nothing — `dispatch_subagent.sh` writes the scoping as prose and its own header calls the contract "convention-only (no enforcement, per ADR-0004/0005)", and this PR's sibling file says "no filesystem isolation"; checkpoints gate publication after the phase has run. The paragraph also omits the real giveaway — in-process hands the phase the host's credentials, the property `.devcontainer/agent/README.md` makes the sandbox's headline. Re-opens Plan Review must-fix 4 — `.claude/skills/run-issue/SKILL.md:126-135`
- [x] (must-fix) The fail-safe branch routes the `review-code` fan-out to container, while the next bullet says further `Agent`-tool fan-out and the host Ollama endpoint cannot run there at all — a direct contradiction between adjacent bullets, mirrored in the knowledge file, landing in the branch written for the uncertain reader — `.claude/skills/run-issue/SKILL.md:113-120`, `.agent/knowledge/skill_workflows.md:139,146-148`
- [x] (must-fix) The load-bearing evidence claim miscounts itself: "all seven phases" enumerates eight, and the real #604 timeline holds nine typed entries — the enumeration drops the `triage-reviews` / Integrated Review phase, the single datapoint that best supports the host-auth bullet. Cited as "Observed, not assumed" — `.claude/skills/run-issue/SKILL.md:103-107`, `.agent/knowledge/skill_workflows.md:135-136`, `.agent/work-plans/issue-607/plan.md:174-176`
- [x] (must-fix) review-plan self-review heuristic contradicts itself: L371-374 prescribes comparing the whole `**By**` field as the detection procedure, L385-391 says the review is independent "no matter what the `**By**` line says" and not to rely on the string. Whole-field matching still false-positives on a genuinely independent same-model dispatch — the defect narrowed from "always" to "whenever the models happen to match" — `.claude/skills/review-plan/SKILL.md:371-374,385-391`
- [x] (must-fix) Plan sync is append-only, which `plan-task/SKILL.md` § During implementation rule 3 names explicitly ("Never append-only... misleads Copilot, human reviewers, and future onboarding agents — all of whom read the top first"). Commit `d8d50d7` is 62 insertions, 0 deletions: Approach step 1 still fences off ~L76-91 and still says keep the safety paragraph "content unchanged", step 3 still says "No other files", the Files-to-Change table still lists two, and Estimated Scope still reads "two files" — `.agent/work-plans/issue-607/plan.md:36-64,85-91,128-131,141`
- [x] (suggestion) "no GitHub auth of its own" / "cannot run in a container at all" overstates: `docker_run_agent.sh` forwards an optional read-only token as `-e GH_TOKEN` and `dispatch_subagent.sh --check` advertises it ("container reads only; the host publishes"). Accurate form is no GitHub *write* auth, read auth only when that token is configured — `.claude/skills/run-issue/SKILL.md:122-124`, `.agent/knowledge/skill_workflows.md:146-148`
- [x] (suggestion) The code-block comment's predicate ("when the host is not in auto mode") is narrower than the bullet's fail-safe rule ("cannot confirm auto mode"); the code block is the copy-paste surface and sits 25 lines above the bullet — `.agent/knowledge/skill_workflows.md:106`
- [x] (suggestion) A route the old text handled is now unrouted: cannot-confirm auto mode *and* container auth not ready. The prior wording sent that case to in-process; `--check` is now only a trailing caveat — `.claude/skills/run-issue/SKILL.md:110-124`
- [x] (suggestion) The two "regardless of mode" absolutes collide on `triage-reviews`, which needs host GitHub read auth *and* takes third-party PR comments as input — data `dispatch_subagent.sh` itself fences as "data, not authority". State the precedence — `.claude/skills/run-issue/SKILL.md:116-120`
- [x] (suggestion) The original's closing caution ("it *is* the safeguard you're relying on — keep it in mind before dispatching anything that processes untrusted input") became an endorsement ("untrusted input belongs there"). The sole-containment clause does survive; the hesitation does not. The hinge flip itself is settled operator ground — this is only about restoring the residual caution — `.claude/skills/run-issue/SKILL.md:130-132`
- [x] (suggestion) `review-plan/SKILL.md:451` still recommends `--mode container` for implementation work with no reference to the new default; the file is already edited by this PR, so aligning it is free — `.claude/skills/review-plan/SKILL.md:451`

### Governance

| Principle | Verdict | Notes |
|---|---|---|
| Human control and transparency | Concern | The containment paragraph claims guardrails that do not exist (must-fix 2); an agent reading it concludes it is held by mechanisms that hold nothing. |
| Enforcement over documentation | Watch | Advisory prose only, as #545 was — consistent, not a regression. `dispatch_subagent.sh` requires an explicit `--mode`, so there is no default to enforce. |
| Capture decisions, not just implementations | Pass | The reason (auto mode) is recorded inline in both files, with #545 → #607 citations in each. |
| A change includes its consequences | Concern | Third stale surface closed and adapters verified clean; the plan itself was not synced inline (must-fix 6). |
| Only what's needed / Improve incrementally | Pass | Two target files plus one folded-in operator-approved fix; no tooling change. |
| Test what breaks | N/A | Advisory prose, no enforced logic. |
| Workspace vs. project separation | Pass | Workspace infra only. |
| Primary framework first, portability where free | Pass | Claude-Code-specific default with an explicit non-Claude fallback branch. |

| ADR | Triggered | Compliant | Notes |
|---|---|---|---|
| 0001 — Adopt ADRs | Watch, not required | Yes | Parity with #545's prose-only precedent; requiring one now would be an inconsistent bar. Consider a "supersedes the #545 default because X" line so the next reader tells reversal from drift. |
| 0003 — Project-agnostic workspace | Yes | Yes | Generic dispatch tooling. |
| 0004 / 0005 — Enforcement hierarchy | Watch | Yes | No new compliance rule; but must-fix 2 is this pair's point exactly — the doc claims enforcement the layers do not provide. |
| 0013 — progress.md vocabulary | Yes | Yes | Entry types correct throughout the timeline. |
| 0015 — Dispatch handoff context contract | No | N/A | The `--context-file` mechanism is untouched. |

| Changed | Required update | Status |
|---|---|---|
| `.claude/skills/*` (framework skill) | Framework adapter files | Done — verified `.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md`, `CLAUDE.md` carry no mode-choice rationale |
| Retired dispatch-mode advice | Every surface repeating it | Done — independent widened search over "implementation-heavy", bare `--mode container`, "lean toward", "prefer container", "prompt-free", "permission prompt", "approval loop" found no further live surface beyond the one fixed (`review-plan:451` raised as a suggestion) |
| Implementation diverged from plan | Plan synced inline per `plan-task` § During implementation | Missing — append-only (must-fix 6) |

### Notes
- The #606 citation is correct and was left alone, as instructed: #606 is the open PR fixing #604. Not re-litigated.
- Two operator-settled decisions were treated as ground truth, not re-opened: the safety reasoning covering both dispatch paths, and folding the review-plan heuristic fix into this PR.
- Prior Plan Review verdict was changes-requested (5 must-fix, 3 suggestions). Findings 1, 2, 3, 6, 7 are genuinely closed. Findings 4 and 5 are closed in form but not in substance — the text now *has* a containment sentence and *has* a named tell, but the containment sentence is inaccurate and the tell is unreadable by its intended reader. Those are must-fix 2 and must-fix 1 here.

## Implementation
**Status**: complete
**When**: 2026-08-24 10:03 -04:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-607 at `fa34c85`
**Addressed**: `## Local Review (Pre-Push)` (2026-08-24 09:55 -04:00, branch at `d8d50d7`) — Round 1, 6 must-fix + 6 suggestions, all actioned, none deferred
**Commits**: 5a8a95b, 677e5cf, 047427f, ddcd0c3, 3f87811, 8394d9e, fa34c85

Every claim rewritten in this pass was verified against source before it was
written — the failure mode must-fix 2 named. Verification notes are inline
below.

### Actions
- [x] (must-fix) Auto-mode tell is now agent-observable. Both files name the injected `While auto mode is active:` system reminder as the signal, with "present ⇒ active, absent ⇒ unconfirmed", and explicitly demote the operator's permission-mode indicator as terminal UI the agent cannot see. **Verified by direct observation**: that system reminder is present in this dispatched sub-agent's own context, so the condition is one an agent can actually evaluate — `.claude/skills/run-issue/SKILL.md`, `.agent/knowledge/skill_workflows.md` (`5a8a95b`)
- [x] (must-fix) Containment paragraph rewritten from verified fact, and it says *less* contains an in-process phase, not more. Each clause checked: auto mode is what stands the host permission policy down (the change's own premise); `.claude/settings.json` has `permissions.allow` (31 entries) and **no `deny` key in any settings file** — `.claude/settings.json`, `.claude/settings.local.json`, or `~/.claude/settings.json` — so the allowlist pre-approves and refuses nothing; the worktree confines nothing (`dispatch_subagent.sh:7-8` "convention-only (no enforcement, per ADR-0004/0005)", and `skill_workflows.md`'s own in-process line reads "no filesystem isolation"). The omitted real distinction is added: in-process hands the phase the host's credentials, which `.devcontainer/agent/README.md:3-5` makes the sandbox's headline ("No GitHub credentials enter the container"). Checkpoints are kept but correctly scoped — they gate publication after the phase ran, catching a bad result, not a bad act. The gap is stated plainly as *strengthening* the case for containers on untrusted input — `.claude/skills/run-issue/SKILL.md` (`677e5cf`)
- [x] (must-fix) Fail-safe contradiction resolved. `review-code` is removed from the container-fallback list and called out as the exception: its fan-out needs the `Agent` tool and the host Ollama endpoint, so it cannot be containerized in any mode and runs in-process paying the prompts. Mirrored in both files — `.claude/skills/run-issue/SKILL.md`, `.agent/knowledge/skill_workflows.md` (`047427f`)
- [x] (must-fix) Evidence claim corrected against the real timeline. Read `.workspace-worktrees/issue-workspace-604/.agent/work-plans/issue-604/progress.md` and the `feature/issue-604` git log: **nine** typed entries — `review-issue`, `plan-task`, `review-plan`, two `review-code` rounds, `triage-reviews`, three `address-findings` passes — plus the implementation pass, which committed `8789f1e..96ad78b` between the plan review and the first `review-code` without writing an entry of its own. `triage-reviews` is now named and used as the datapoint that most supports the host-auth bullet. Fixed in all three surfaces including the plan — `.claude/skills/run-issue/SKILL.md`, `.agent/knowledge/skill_workflows.md`, `.agent/work-plans/issue-607/plan.md` (`ddcd0c3`)
- [x] (must-fix) review-plan self-review heuristic no longer contradicts itself and no longer false-positives. The detection procedure is now a question about the reviewer's own invocation ("did *you* write the Plan Authored entry in **this** context?"), and the `**By**` comparison is demoted to one-directional corroboration: differing model strings are positive evidence of independence, matching strings are evidence of nothing. The same-model independent-dispatch false positive is stated explicitly as the reason no comparison can serve as the test — `.claude/skills/review-plan/SKILL.md` (`3f87811`)
- [x] (must-fix) Plan synced **inline**, per `plan-task` § During implementation rule 3 — the commit is 71 insertions / 41 deletions, not append-only. Corrected at the top where readers start: Approach step 1's do-not-touch fence redrawn and its "content unchanged" instruction marked superseded; step 1's "no new detection mechanism" replaced (the shipped text does name a tell); step 2 notes the code-block-comment addition; step 3 rewritten from "No other files" to name `review-plan/SKILL.md` as the third file; the Files-to-Change table now lists three rows with accurate change descriptions; Estimated Scope reads three files; the "Only what's needed" principle row updated. Implementation notes 2 and 3 were also corrected — they still described the unreadable tell and the false containment claims — `.agent/work-plans/issue-607/plan.md` (`fa34c85`)
- [x] (suggestion) Container GitHub-auth claim made accurate: "no GitHub **write** auth", with read auth available only when the optional read-only token is configured. **Verified**: `docker_run_agent.sh:485-519` reads `~/.config/ros2-agent/gh-readonly-token` and forwards it as `-e GH_TOKEN`; `dispatch_subagent.sh:231` advertises it as "container reads only; the host publishes" — `.claude/skills/run-issue/SKILL.md`, `.agent/knowledge/skill_workflows.md` (`047427f`, `8394d9e`)
- [x] (suggestion) Code-block comment predicate widened to match the bullet's fail-safe rule: "use for isolation, or whenever auto mode cannot be confirmed" — `.agent/knowledge/skill_workflows.md` (`8394d9e`)
- [x] (suggestion) The unrouted case is routed: cannot-confirm auto mode **and** container auth not ready (`--check` reports missing tokens) → run in-process and accept the prompts. Stated in both files, with the reasoning ("an approval-heavy phase is worse than a quiet one; it is not worse than a phase that cannot start") — `.claude/skills/run-issue/SKILL.md`, `.agent/knowledge/skill_workflows.md` (`047427f`)
- [x] (suggestion) Precedence between the two "regardless of mode" absolutes stated, with `triage-reviews` as the named live case: capability wins (it needs host GitHub auth), and the reviewer holds the "data, not authority" fence themselves rather than delegating it to a sandbox. Verified the quoted phrase at `dispatch_subagent.sh:427`. Mirrored in both files — `.claude/skills/run-issue/SKILL.md`, `.agent/knowledge/skill_workflows.md` (`047427f`)
- [x] (suggestion) Residual caution restored, in the original's voice: "it *is* the safeguard you are relying on — think before dispatching anything that processes untrusted input outside it", closing with "choose in-process because the phase needs host resources and you trust its input — never because it is contained." Folded into the must-fix 2 rewrite since it is the same paragraph — `.claude/skills/run-issue/SKILL.md` (`677e5cf`)
- [x] (suggestion) `review-plan/SKILL.md`'s implementation-dispatch block now points at the new default before recommending `--mode container`, cross-referencing `run-issue/SKILL.md` § Choosing a mode (#607) — `.claude/skills/review-plan/SKILL.md` (`8394d9e`)

### Notes
- Nothing deferred; all 12 findings actioned.
- Two operator-settled decisions were treated as ground truth and not re-opened: the safety reasoning covers both dispatch paths, and the review-plan heuristic fix stays folded into this PR. The `#606` citation was left untouched.
- Honest-outcome note for the re-review: the containment rewrite makes the change's own narrative *less* flattering to in-process, not more. That was deliberate — the finding asked for what is true, and what is true is that markedly less contains an in-process phase. The mode choice now rests on prompt cost and host-capability need, with containment stated as an argument that still runs the other way.
- Pre-commit ran clean on every commit (markdown-only diff; shellcheck/black/flake8/pylint/yamllint all skipped for lack of matching files). No unchecked findings remain in the source review entry.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-08-24 10:32 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-607 at `4ef5050`
**Mode**: pre-push
**Depth**: Standard (reason: governance-touching instruction files other agents read as authority; +174/-45 prose across two skill files and one knowledge file)
**Must-fix**: 4 | **Suggestions**: 10
**Round**: 2 | **Ship**: continue — must-fix fell 6 → 4 and every correction is mechanical, but two of the four are recurrences in the same two paragraphs the round-1 pass reported as verified (the containment paragraph is wrong a second time, now on its container half; the fail-safe branch still emits an impossible instruction for one of the three cases it enumerates), and the tell the whole change hinges on does not hold as written

Specialists: static analysis (no linter profile for `.md`; worktree clean), governance,
plan drift, Claude Adversarial x2 (Lens A + Lens B). Local Adversarial off by default
(`--local` not passed); it could not have run regardless — the Ollama server is up but
`ollama list` is empty, so `qwen3.5:35b` is not pulled on this host. Copilot off (default).

### Findings
- [ ] (must-fix) The containment paragraph is wrong a second time — this round on its **container** half, the half the fix pass did not re-verify. Three false clauses: "a separate filesystem" (`docker_run_agent.sh:509` bind-mounts the **entire workspace root read-write at the same absolute path**, plus both worktree trees at 596-597; only `.agent/` is re-mounted `:ro`); "no route to the host's own caches or credentials" / "no host credentials inside" (§6 at 599-616 mounts `~/.claude/.credentials.json`, `~/.claude.json` and `~/.claude/settings.json`, and the long-lived `CLAUDE_CODE_OAUTH_TOKEN` is forwarded at 690); and "cannot reach anything the host is logged into", which contradicts this same section's own line 148 (`-e GH_TOKEN` read auth). The cited README says only "No **GitHub** credentials enter the container" — narrower than the claim made of it. Accurate: the container isolates the OS, dependency state and build artifacts, and withholds GitHub write auth; it does **not** isolate the host workspace tree. Cross-pass confirmed (Lens A + Lens B + lead) — `.claude/skills/run-issue/SKILL.md:90,154-159`
- [ ] (must-fix) The auto-mode tell does not hold as written, in two ways. (a) "into every turn" is false by direct observation: in this dispatched sub-agent the `While auto mode is active:` reminder was injected **once**, attached to the first tool result, and has not recurred across ~30 subsequent tool calls. A reader applying the rule to the current turn answers "absent ⇒ unconfirmed" and falls through to container — the new default never fires in the sessions it was written for. The fail-safe direction means this is inert, not dangerous, but the PR's central mechanism does not work as specified. Correct form: present **at least once in this context** — check the whole context, not the current turn. (b) Both files now contain the literal sentinel, so an agent that checks by string match will match the document it just read; say the tell counts only as an injected `system-reminder`, never as a quotation of this doc — `.claude/skills/run-issue/SKILL.md:99-104`, `.agent/knowledge/skill_workflows.md:134-136`
- [ ] (must-fix) The fail-safe branch still issues an impossible instruction for one of the three cases it enumerates. It lists "a non-Claude runtime with no `Agent` tool" among cannot-confirm cases, then the `review-code` sub-bullet says it "cannot be containerized in *any* mode. Run it in-process" — and in-process is **defined** twenty lines above as "spawns the phase via Claude Code's `Agent` tool", which that runtime does not have. Line 83 already has the right answer ("drive the phases manually"). Same defect class as round-1 must-fix 3, relocated rather than closed. Mirrored in the knowledge file — `.claude/skills/run-issue/SKILL.md:121-129` (vs `79-84`), `.agent/knowledge/skill_workflows.md:144-149`
- [ ] (must-fix) Misquoted citation on the worktree claim. `dispatch_subagent.sh:7-9`'s "convention-only (no enforcement, per ADR-0004/0005)" describes the **sub-agent's `progress.md` exit contract**, not worktree scoping — the sentence's subject is "The sub-agent's exit contract". The conclusion (in-process has no filesystem confinement) is correct and independently verified; the evidence for it is the handoff prose the script emits at line 469 ("within this issue's worktree; do not touch other issues"). As written, an agent that follows the citation finds a mismatch, and ADR-0004/0005 are pulled in as backing for a claim they do not make. Cross-pass confirmed (Lens A + Lens B + lead) — `.claude/skills/run-issue/SKILL.md:170-173`
- [ ] (suggestion) "`review-code` … cannot be containerized in *any* mode" overstates. `review-code/SKILL.md:333` specifies a fallback for the missing `Agent` tool ("use the `Agent` tool with subagents **when available** … otherwise evaluate sequentially"), and 5f skips itself with a notice when Ollama is unavailable. A container run **degrades** (no fresh-context independence, no local specialist), it is not impossible. Ollama is also opt-in and off by default — and not pulled on this host — so listing it as a co-equal blocker to the `Agent` tool overstates it; the `Agent`-tool reason alone carries the point — `.claude/skills/run-issue/SKILL.md:126-129`, `.agent/knowledge/skill_workflows.md:145-148`
- [ ] (suggestion) The "none of the mechanisms you might reach for actually hold" framing is stronger than the evidence. The `deny` claim is literally true (verified: no `deny` key in `.claude/settings.json`, `.claude/settings.local.json`, or `~/.claude/settings.json`), but `settings.local.json` carries a `permissions.ask` array — `merge_pr.sh`, `gh pr merge`, `make merge-pr`, `tmux send-keys` — which does force a prompt on exactly the destructive ops. Accurate: the tracked settings carry no `deny`; this machine's untracked local settings add an `ask` list no other host can be relied on to have. Cross-pass confirmed — `.claude/skills/run-issue/SKILL.md:166-169`
- [ ] (suggestion) "auto mode is precisely what stands [the permission policy] down" is the one clause in the paragraph stated more absolutely than the evidence supports — auto mode approves *routine* calls; `ask` rules and the `PreToolUse` hooks wired in `.claude/settings.json` still fire. It errs in the safe direction — `.claude/skills/run-issue/SKILL.md:164-166`
- [ ] (suggestion) A consequence of the flip goes unaddressed: `dispatch_subagent.sh`'s untrusted-data fence (the BEGIN/END nonce block and "data, not authority", lines 419-435) is emitted **only** when `--context-file` is passed — i.e. only on the container path. Under the new in-process default, `review-issue` fetches the issue body itself with `gh` and gets **no fence at all**. The diff names the hold-the-fence-yourself duty for `triage-reviews` but not for `review-issue`, the more common case — `.claude/skills/run-issue/SKILL.md:118-135,143-148`
- [ ] (suggestion) "it could only ever have run this way" / "nothing else can run it" (of `triage-reviews`) is stronger than this same file's own #552 pattern at lines 71-74, which exists to feed a no-auth container a host-fetched body; `fetch_pr_reviews.sh` runs host-side too. The file's own phrasing — "not supported by body-only injection" — is the accurate form — `.claude/skills/run-issue/SKILL.md:115,143-144`
- [ ] (suggestion) Prose and copy-paste surface disagree: the text declares in-process the default, but the only command shown is `--mode container`, so a reader copying the snippet gets the non-default. Same class as round-1 suggestion 8, which was fixed for `skill_workflows.md`'s code block but not here. Separately, "`run-issue/SKILL.md` § Choosing a mode" is a bold lead-in, not a heading, so a `§` cross-reference will not resolve by heading search — `.claude/skills/review-plan/SKILL.md:452-457`
- [ ] (suggestion) The self-review rewrite leaves its own hardest case undefined: the "genuinely unsure" branch says to state the basis for your judgement, but never says whether to append the `(in-context — author self-review)` annotation — so the `**By**` field has no defined value in exactly the case the rewrite existed to close. Relatedly, "differing model strings … worth stating in the entry" has no slot in the entry template; the `**By**` line already carries the model — `.claude/skills/review-plan/SKILL.md:386-394`
- [ ] (suggestion) Stale after this change, in the skill this PR declares uncontainerizable: the convergence guidance still reads "each round costs a **container** cycle", assuming the default #607 retires. The file is otherwise untouched by this PR, so this is a cheap consequence to close here — `.claude/skills/review-code/SKILL.md:866`
- [ ] (suggestion) "a phase needing … host-built layer installs … cannot run in a container **at all**" overstates — a container can build the layers itself, at cost. The genuine blockers in that list are host GitHub auth, the Ollama endpoint, and `Agent`-tool fan-out — `.agent/knowledge/skill_workflows.md:155-158`
- [ ] (suggestion) The appended `## Implementation notes (plan sync)` section is changelog-shaped ("What actually shipped differs from the plan above in four ways"), where `plan-task` § During implementation rule 2 scopes an appended section to *rationale only, not a changelog*. Style only — the inline sync itself is genuine and rules 1 and 3 are satisfied — `.agent/work-plans/issue-607/plan.md:161-163`

### Governance

| Principle | Verdict | Notes |
|---|---|---|
| Human control and transparency | Concern | Second consecutive round in which the paragraph whose stated purpose is "be accurate about what does the containing" contains false containment claims — this time favouring the sandbox. An agent deciding where to put untrusted-input work is misled about how much the container actually isolates. |
| Enforcement over documentation | Watch | Advisory prose only, as #545 was — consistent, not a regression. `dispatch_subagent.sh` requires an explicit `--mode`, so there is no default to enforce. |
| Capture decisions, not just implementations | Pass | The reason (auto mode) is recorded inline in both files, each carrying its own #545 → #607 citation. |
| A change includes its consequences | Concern | Adapter files verified clean and the retired advice found nowhere else; but two consequences are open — the untrusted-input fence that only the container path emits (suggestion 8) and `review-code/SKILL.md:866` (suggestion 12). |
| Only what's needed / Improve incrementally | Pass | Three files, prose-only, no tooling change; the third is operator-approved. |
| Test what breaks | N/A | Advisory prose, no enforced logic. |
| Workspace vs. project separation | Pass | Workspace infra only. |
| Primary framework first, portability where free | Concern | The non-Claude-runtime branch is named but routed to an instruction that runtime cannot execute (must-fix 3). |

| ADR | Triggered | Compliant | Notes |
|---|---|---|---|
| 0001 — Adopt ADRs | Watch, not required | Yes | Parity with #545's prose-only precedent; both files now carry the #545 → #607 citation, so a future reader can tell reversal from drift. |
| 0003 — Project-agnostic workspace | Yes | Yes | Generic dispatch tooling. |
| 0004 / 0005 — Enforcement hierarchy | Yes | Concern | Cited by name in the diff as backing for the worktree claim, which they do not make (must-fix 4). The substantive point — documentation is the weakest layer — is exactly this pair's thesis and is correctly applied. |
| 0013 — progress.md vocabulary | Yes | Yes | Entry types correct throughout. |
| 0015 — Dispatch handoff context contract | Watch | Yes | `--context-file` untouched, but suggestion 8 is a consequence of the flip for the fence ADR-0015's mechanism carries. |

| Changed | Required update | Status |
|---|---|---|
| `.claude/skills/*` (framework skill) | Framework adapter files | Done — re-verified: no mode-choice rationale in `.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md`, `CLAUDE.md` |
| Retired dispatch-mode advice | Every surface repeating it | Done for the dispatch-mode rule itself — independent widened search found no live surface beyond the three files changed. One second-order surface remains (`review-code/SKILL.md:866`, suggestion 12) |
| Implementation diverged from plan | Plan synced inline per `plan-task` § During implementation | Done — `fa34c85` is 71 insertions / 41 deletions, editing the Approach steps, Files-to-Change table, Principles Self-Check and Estimated Scope in place. Round-1 must-fix 6 genuinely closed |

### Round-1 closure check
- Genuinely closed: must-fix 4 (#604 evidence — verified independently against the merged `.agent/work-plans/issue-604/progress.md` on `main`: exactly nine `##` entries, matching the claimed mapping, with the implementation pass unentried between `## Plan Review` and the first `## Local Review (Pre-Push)`); must-fix 5 (review-plan heuristic now keys on invocation, a test the reader can actually apply — I applied it to myself); must-fix 6 (plan synced inline); suggestions 7, 8, 9, 10, 11, 12.
- Closed in form, not in substance: must-fix 1 (tell named but mis-specified — must-fix 2 here); must-fix 2 (in-process half now correct and verified, container half now wrong and one citation misattributed — must-fix 1 and 4 here); must-fix 3 (contradiction removed from the review-code bullet pair, but the same impossible-instruction shape survives for the non-Claude-runtime case — must-fix 3 here).

### Notes
- Two operator-settled decisions treated as ground truth, not re-opened: the safety reasoning covering both dispatch paths, and folding the review-plan heuristic fix into this PR. The `#606` citation is correct (PR #606, now merged as `a00193c`) and was left alone as instructed.
- The verification-of-the-verification asked for by the host: the auto-mode tell **is** agent-observable (confirmed in this sub-agent's own context) but is mis-specified as per-turn; the containment paragraph's in-process half **is** now verified-accurate, its container half is not; the #604 count **is** correct. The recurring pattern across this round's findings is citation precision — three claims are true but attributed to sources that say something adjacent (`dispatch_subagent.sh`'s exit-contract header, the README's GitHub-only credential claim, the fence that only the `--context-file` path emits).
