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
- [x] (must-fix) The containment paragraph is wrong a second time — this round on its **container** half, the half the fix pass did not re-verify. Three false clauses: "a separate filesystem" (`docker_run_agent.sh:509` bind-mounts the **entire workspace root read-write at the same absolute path**, plus both worktree trees at 596-597; only `.agent/` is re-mounted `:ro`); "no route to the host's own caches or credentials" / "no host credentials inside" (§6 at 599-616 mounts `~/.claude/.credentials.json`, `~/.claude.json` and `~/.claude/settings.json`, and the long-lived `CLAUDE_CODE_OAUTH_TOKEN` is forwarded at 690); and "cannot reach anything the host is logged into", which contradicts this same section's own line 148 (`-e GH_TOKEN` read auth). The cited README says only "No **GitHub** credentials enter the container" — narrower than the claim made of it. Accurate: the container isolates the OS, dependency state and build artifacts, and withholds GitHub write auth; it does **not** isolate the host workspace tree. Cross-pass confirmed (Lens A + Lens B + lead) — `.claude/skills/run-issue/SKILL.md:90,154-159`
- [x] (must-fix) The auto-mode tell does not hold as written, in two ways. (a) "into every turn" is false by direct observation: in this dispatched sub-agent the `While auto mode is active:` reminder was injected **once**, attached to the first tool result, and has not recurred across ~30 subsequent tool calls. A reader applying the rule to the current turn answers "absent ⇒ unconfirmed" and falls through to container — the new default never fires in the sessions it was written for. The fail-safe direction means this is inert, not dangerous, but the PR's central mechanism does not work as specified. Correct form: present **at least once in this context** — check the whole context, not the current turn. (b) Both files now contain the literal sentinel, so an agent that checks by string match will match the document it just read; say the tell counts only as an injected `system-reminder`, never as a quotation of this doc — `.claude/skills/run-issue/SKILL.md:99-104`, `.agent/knowledge/skill_workflows.md:134-136`
- [x] (must-fix) The fail-safe branch still issues an impossible instruction for one of the three cases it enumerates. It lists "a non-Claude runtime with no `Agent` tool" among cannot-confirm cases, then the `review-code` sub-bullet says it "cannot be containerized in *any* mode. Run it in-process" — and in-process is **defined** twenty lines above as "spawns the phase via Claude Code's `Agent` tool", which that runtime does not have. Line 83 already has the right answer ("drive the phases manually"). Same defect class as round-1 must-fix 3, relocated rather than closed. Mirrored in the knowledge file — `.claude/skills/run-issue/SKILL.md:121-129` (vs `79-84`), `.agent/knowledge/skill_workflows.md:144-149`
- [x] (must-fix) Misquoted citation on the worktree claim. `dispatch_subagent.sh:7-9`'s "convention-only (no enforcement, per ADR-0004/0005)" describes the **sub-agent's `progress.md` exit contract**, not worktree scoping — the sentence's subject is "The sub-agent's exit contract". The conclusion (in-process has no filesystem confinement) is correct and independently verified; the evidence for it is the handoff prose the script emits at line 469 ("within this issue's worktree; do not touch other issues"). As written, an agent that follows the citation finds a mismatch, and ADR-0004/0005 are pulled in as backing for a claim they do not make. Cross-pass confirmed (Lens A + Lens B + lead) — `.claude/skills/run-issue/SKILL.md:170-173`
- [x] (suggestion) "`review-code` … cannot be containerized in *any* mode" overstates. `review-code/SKILL.md:333` specifies a fallback for the missing `Agent` tool ("use the `Agent` tool with subagents **when available** … otherwise evaluate sequentially"), and 5f skips itself with a notice when Ollama is unavailable. A container run **degrades** (no fresh-context independence, no local specialist), it is not impossible. Ollama is also opt-in and off by default — and not pulled on this host — so listing it as a co-equal blocker to the `Agent` tool overstates it; the `Agent`-tool reason alone carries the point — `.claude/skills/run-issue/SKILL.md:126-129`, `.agent/knowledge/skill_workflows.md:145-148`
- [x] (suggestion) The "none of the mechanisms you might reach for actually hold" framing is stronger than the evidence. The `deny` claim is literally true (verified: no `deny` key in `.claude/settings.json`, `.claude/settings.local.json`, or `~/.claude/settings.json`), but `settings.local.json` carries a `permissions.ask` array — `merge_pr.sh`, `gh pr merge`, `make merge-pr`, `tmux send-keys` — which does force a prompt on exactly the destructive ops. Accurate: the tracked settings carry no `deny`; this machine's untracked local settings add an `ask` list no other host can be relied on to have. Cross-pass confirmed — `.claude/skills/run-issue/SKILL.md:166-169`
- [x] (suggestion) "auto mode is precisely what stands [the permission policy] down" is the one clause in the paragraph stated more absolutely than the evidence supports — auto mode approves *routine* calls; `ask` rules and the `PreToolUse` hooks wired in `.claude/settings.json` still fire. It errs in the safe direction — `.claude/skills/run-issue/SKILL.md:164-166`
- [x] (suggestion) A consequence of the flip goes unaddressed: `dispatch_subagent.sh`'s untrusted-data fence (the BEGIN/END nonce block and "data, not authority", lines 419-435) is emitted **only** when `--context-file` is passed — i.e. only on the container path. Under the new in-process default, `review-issue` fetches the issue body itself with `gh` and gets **no fence at all**. The diff names the hold-the-fence-yourself duty for `triage-reviews` but not for `review-issue`, the more common case — `.claude/skills/run-issue/SKILL.md:118-135,143-148`
- [x] (suggestion) "it could only ever have run this way" / "nothing else can run it" (of `triage-reviews`) is stronger than this same file's own #552 pattern at lines 71-74, which exists to feed a no-auth container a host-fetched body; `fetch_pr_reviews.sh` runs host-side too. The file's own phrasing — "not supported by body-only injection" — is the accurate form — `.claude/skills/run-issue/SKILL.md:115,143-144`
- [x] (suggestion) Prose and copy-paste surface disagree: the text declares in-process the default, but the only command shown is `--mode container`, so a reader copying the snippet gets the non-default. Same class as round-1 suggestion 8, which was fixed for `skill_workflows.md`'s code block but not here. Separately, "`run-issue/SKILL.md` § Choosing a mode" is a bold lead-in, not a heading, so a `§` cross-reference will not resolve by heading search — `.claude/skills/review-plan/SKILL.md:452-457`
- [x] (suggestion) The self-review rewrite leaves its own hardest case undefined: the "genuinely unsure" branch says to state the basis for your judgement, but never says whether to append the `(in-context — author self-review)` annotation — so the `**By**` field has no defined value in exactly the case the rewrite existed to close. Relatedly, "differing model strings … worth stating in the entry" has no slot in the entry template; the `**By**` line already carries the model — `.claude/skills/review-plan/SKILL.md:386-394`
- [x] (suggestion) Stale after this change, in the skill this PR declares uncontainerizable: the convergence guidance still reads "each round costs a **container** cycle", assuming the default #607 retires. The file is otherwise untouched by this PR, so this is a cheap consequence to close here — `.claude/skills/review-code/SKILL.md:866`
- [x] (suggestion) "a phase needing … host-built layer installs … cannot run in a container **at all**" overstates — a container can build the layers itself, at cost. The genuine blockers in that list are host GitHub auth, the Ollama endpoint, and `Agent`-tool fan-out — `.agent/knowledge/skill_workflows.md:155-158`
- [x] (suggestion) The appended `## Implementation notes (plan sync)` section is changelog-shaped ("What actually shipped differs from the plan above in four ways"), where `plan-task` § During implementation rule 2 scopes an appended section to *rationale only, not a changelog*. Style only — the inline sync itself is genuine and rules 1 and 3 are satisfied — `.agent/work-plans/issue-607/plan.md:161-163`

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

## Implementation
**Status**: complete
**When**: 2026-08-24 10:37 -04:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-607 at `f31e5e4`
**Addressed**: `## Local Review (Pre-Push)` (Round 2), 2026-08-24 10:32 -04:00, branch at `4ef5050` — 4 must-fix + 10 suggestions, all actioned, none deferred
**Commits**: `93aae26`, `eb52753`, `0c373bf`, `052cc0b`, `dfeb917`, `9d24614`, `5c88066`, `d314c20`, `e0737a3`, `3c5343a`, `430a18c`, `1af2dee`, `f31e5e4`

### Actions
- [x] (must-fix) Container half of the containment paragraph rewritten against source — the container isolates OS/dependency state and build artifacts and admits no GitHub credentials, but `docker_run_agent.sh:509` bind-mounts the whole workspace root read-write at the same absolute path (worktrees again at 596-597, only `.agent/` `:ro`) and §6 (599-616) mounts the host's Claude credentials with `CLAUDE_CODE_OAUTH_TOKEN` forwarded at 688. The lead-in no longer claims the modes are far apart; it names the real axis (GitHub write auth + machine state) — `eb52753`, `.claude/skills/run-issue/SKILL.md`
- [x] (must-fix) Auto-mode tell re-specified: injected **once**, attached to an early tool result, so the check is "present anywhere in the whole context", never per-turn; and it counts only as an injected `system-reminder`, never as a quotation of these docs, which now both contain the sentinel — `93aae26`, `.claude/skills/run-issue/SKILL.md`, `.agent/knowledge/skill_workflows.md`
- [x] (must-fix) The non-Claude-runtime case split out of the cannot-confirm list into its own bullet — there is no in-process option there, because the `Agent` tool *is* in-process dispatch; drive the phases manually or containerize. `review-code` keeps only the cases in-process actually applies to — `0c373bf`, both files
- [x] (must-fix) Worktree claim now cites `dispatch_subagent.sh:469` (the handoff sentence itself). The header's "convention-only (no enforcement, per ADR-0004/0005)" is explicitly marked as being about the *exit contract*, with ADR-0004/0005 kept only as the enforcement-hierarchy lens — `052cc0b`, `.claude/skills/run-issue/SKILL.md`
- [x] (suggestion) "Cannot be containerized in *any* mode" softened: `review-code/SKILL.md:333-335` evaluates specialists sequentially without the `Agent` tool, so a container run **degrades** rather than being impossible — `0c373bf`, `430a18c`, both files
- [x] (suggestion) "None of the mechanisms hold" replaced with a per-mechanism read; the no-`deny` claim scoped to the tracked baseline, with `.claude/settings.local.json`'s untracked `permissions.ask` list named as the per-machine exception — `dfeb917`, `.claude/skills/run-issue/SKILL.md`
- [x] (suggestion) "Auto mode is precisely what stands it down" narrowed — auto mode approves the *routine* calls; `ask` rules and `PreToolUse` hooks still fire — `dfeb917`, `.claude/skills/run-issue/SKILL.md`
- [x] (suggestion) `review-issue`'s hold-the-fence-yourself duty stated in both files: the untrusted-data fence is emitted only on the `--context-file` path, so an in-process `review-issue` fetching the body with `gh` gets none — `9d24614`, `f31e5e4`
- [x] (suggestion) "Could only ever have run this way" / "nothing else can run it" replaced with the file's own accurate form — body-only injection does not cover PR review comments and CI status — `9d24614`, `.claude/skills/run-issue/SKILL.md`
- [x] (suggestion) `review-plan`'s dispatch snippet now shows `--mode in-process` (the declared default) with `--mode container` named as the swap, and the cross-reference points at `§ How phases are dispatched` rather than a bold lead-in — `5c88066`, `.claude/skills/review-plan/SKILL.md`
- [x] (suggestion) The self-review "genuinely unsure" branch now has a defined answer — apply the `(in-context — author self-review)` annotation and say what left you unsure — and the model-string note points at the `**By**` line that already carries it — `5c88066`, `.claude/skills/review-plan/SKILL.md`
- [x] (suggestion) `review-code/SKILL.md:866` no longer costs a review round as "a container cycle" — `d314c20`
- [x] (suggestion) "Cannot run in a container at all" split: host GitHub auth / Ollama / `Agent`-tool fan-out are genuine blockers; host-built layer installs are merely expensive — `d314c20`, `.agent/knowledge/skill_workflows.md`
- [x] (suggestion) The appended plan section is now `## Implementation Notes` carrying rationale only, per `plan-task` § During implementation rule 2; the changelog framing and the "Scope additions" list are gone, their substance already synced inline. Six cross-references the rewrite orphaned were repointed — `e0737a3`, `3c5343a`, `.agent/work-plans/issue-607/plan.md`

### Beyond the listed findings
Re-verified every neighbouring claim in each paragraph touched, per the
recurrence pattern flagged in rounds 1 and 2. Three further corrections, none
of which any review raised:

- **The container mode-definition bullet repeated the false claim.** Twenty
  lines above the containment paragraph, it also said "no route to the host's
  own caches or credentials" — the identical clause must-fix 1 falsifies. It
  now points at the containment paragraph instead of restating it, so the two
  cannot drift apart again (`1af2dee`).
- **Specialist 5f is on by default, not opt-in.** The review's suggestion-5
  parenthetical ("Ollama is also opt-in and off by default") is wrong against
  source: `review-code/SKILL.md:46` and `:740` both state it activates by
  default at every tier and is opted out with `--no-local` (the flip to opt-in
  is a separate open issue). The first fix pass repeated the review's wording;
  `430a18c` corrects it in both files. Corollary: a container run loses the
  cross-model pass silently, which is a slightly *stronger* reason to keep
  `review-code` in-process than the review credited.
- **The plan's Context still asserted the unverifiable mechanism.** It read
  "in-process `Agent`-tool sub-agents inherit the session's permission mode" —
  the exact claim the shipped text deliberately backed away from in favour of
  the observed #604 lifecycle. Corrected inline (`e0737a3`).

Every `file:line` citation in the diff was re-checked against source, not just
the one must-fix 4 named: `docker_run_agent.sh:509`, `596-597`, `599-616`,
`688`, `690`; `dispatch_subagent.sh:469`; `review-code/SKILL.md:333-335`,
`:46`, `:740`, `:866`; `.devcontainer/agent/README.md:4`;
`skill_workflows.md:102`. All verified. The two operator-settled decisions
(safety reasoning covering both dispatch paths; the `review-plan` heuristic
staying in this PR) were treated as ground truth, and the `#606` citation was
left alone.

### Checks
Pre-commit hooks ran clean on all thirteen commits (`--no-verify` never used).
Prose-only diff — no package tests apply. Worktree clean; nothing pushed.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-08-24 10:50 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-607 at `b33e250`
**Mode**: pre-push
**Depth**: Standard (reason: governance-touching instruction files other agents read as authority; +201/-49 prose across two skill files, one knowledge file and the plan)
**Must-fix**: 4 | **Suggestions**: 7
**Round**: 3 | **Ship**: continue — must-fix is flat at 4, not falling, and one of them is a *new* factual inversion the fix pass introduced while asserting the round-2 review was wrong; but the cheaper convergence path is a merge + targeted re-check, not a fourth full round (see Convergence)

Specialists: static analysis (no linter profile for `.md`; no trailing whitespace or over-long added lines; worktree clean), governance, plan drift, Claude Adversarial x2 (Lens A + Lens B, fresh-context sub-agents). Copilot off (default). **Local Adversarial (5f) did not run, and could not have**: on current `main` it is opt-in via `--local` (#590) and was not passed — and the model is absent anyway, the Ollama server answers but `/api/tags` returns `{"models":[]}` and `ollama list` is empty, so `qwen3.5:35b` is not pulled on this host.

### Findings
- [x] (must-fix) The 5f default is inverted in both files, and the round-2 review it overruled was right: on current `main` `review-code/SKILL.md:45-47,104-106,152-158,750-752` make 5f **opt-in via `--local`, off by default** (#590, merged into `main` as `0d0aebf`), with `--no-local` a deprecated no-op. `430a18c` verified against the branch's stale copy of that file and shipped "on by default; `--no-local` opts out" into both. Cross-pass confirmed (Lens A + Lens B + lead). Fix: "specialist 5f (opt-in via `--local`) cannot run there at all — no host Ollama endpoint"; the `Agent`-tool reason already carries the exception — `.claude/skills/run-issue/SKILL.md:140-141`, `.agent/knowledge/skill_workflows.md:154-155`
- [x] (must-fix) Citations are verified against two different baselines and one ships wrong. `review-code/SKILL.md:333-335` resolves only in the branch's stale copy; on `main` those lines are the static-analysis linter table and the sequential-fallback sentence is at **343-346**. The `docker_run_agent.sh` citations are the mirror image — correct against `main`, ~160 lines off against the branch's own copy. Root cause: the branch does not contain `main` (`git merge-base --is-ancestor main HEAD` fails; `main` has moved through #590/#594/#602/#604 since the base), which is also what produced the finding above. Fix: merge `main` first (required before push regardless), then re-resolve every citation against the merged tree. Cross-pass confirmed (Lens A + Lens B + lead) — `.claude/skills/run-issue/SKILL.md:139`
- [x] (must-fix) The in-process bullet whose job is to deflate residual containment re-inflates it: "`ask` rules and the `PreToolUse` hooks wired in `.claude/settings.json` still fire". That file defines **no** `ask` rules (they are in the untracked `.claude/settings.local.json`, as the very next bullet says) and exactly **one** `PreToolUse` hook — matcher `AskUserQuestion`, invoked `… || true`, warn-only by design (`AGENTS.md:585`: "never blocks") and unable to match an Edit/Bash/commit call at all. Cross-pass confirmed (Lens B + lead) — `.claude/skills/run-issue/SKILL.md:209-211`
- [x] (must-fix) "**no GitHub credentials enter**" is contradicted nine lines later by its own citation (`-e GH_TOKEN` forwarded at `docker_run_agent.sh:690`), by the container bullet at `:175-177`, and by the knowledge file's mirror, which correctly says *write* auth. Same absolute again at `:230-231` ("GitHub auth included … exactly the capability the sandbox withholds") and `skill_workflows.md:166-167` ("simply does not have host GitHub auth"). Fix: say **write** auth consistently in both files; and re-attribute the README claim, which leads with "container filesystem isolation as the security boundary" (`.devcontainer/agent/README.md:3`) — the framing this same paragraph refutes — not with the credential property. Cross-pass confirmed (Lens A + Lens B + lead) — `.claude/skills/run-issue/SKILL.md:189-191,230-231`, `.agent/knowledge/skill_workflows.md:166-167`
- [x] (suggestion) "skips itself with a one-line notice … and **silently** dropping the cross-model pass" contradicts itself inside one sentence: `review-code/SKILL.md:797` and the report/progress templates (`:891,949,968-975,994`) all emit `Local Adversarial skipped: <reason>`. Drop "silently" — `.claude/skills/run-issue/SKILL.md:143-144`
- [x] (suggestion) "only when `--context-file` is passed, **i.e. only on the container path**" — the gloss is false: `--context-file` is orthogonal to `--mode` (`dispatch_subagent.sh:312-317`), and `CONTEXT_SECTION` is spliced into the handoff at `:461-463`, before the mode branch, so an in-process dispatch that passes it gets the fence too. The knowledge file's sibling sentence stops at "only on the `--context-file` path" and is correct; delete the gloss — `.claude/skills/run-issue/SKILL.md:167-168`
- [x] (suggestion) The fence-loss enumeration reads as exhaustive but names two phases; `plan-task/SKILL.md:36` and `review-plan/SKILL.md:112` both fetch `comments`, and post-PR `review-code/SKILL.md:256` fetches `comments,reviews`. Generalize the heading (the bullet's closing sentence already does) rather than listing phases — `.claude/skills/run-issue/SKILL.md:165-172`
- [x] (suggestion) "only `.agent/` is re-mounted `:ro`" omits that `.agent/scratchpad` is punched back read-write over it (`docker_run_agent.sh:515` on `main`) — `.claude/skills/run-issue/SKILL.md:196`
- [x] (suggestion) "no GitHub *write* auth **ever**" is stronger than the script: `docker_run_agent.sh:651-665` takes `AGENT_GH_TOKEN` as an explicit override ahead of the `gh-readonly-token` file and validates no scopes — read-only is a filename convention, not an enforced property. Drop "ever" — `.agent/knowledge/skill_workflows.md:170-172`
- [x] (suggestion) Off-by-one: the quoted handoff sentence begins at `dispatch_subagent.sh:468` ("…dispatched for **issue #$ISSUE**. Work only"); cite `:468-469` — `.claude/skills/run-issue/SKILL.md:221`
- [x] (suggestion) Two unchanged neighbours in the same file now disagree with the new text: `:48` still says "A container dispatch **has no GitHub read auth**" against the new `:175-177`; and `:344` cites `review-code/SKILL.md:864-866, 872` for the post-PR `## Local Review` heading, which lives at `:1029-1036` — `864-872` is the ship-verdict block this PR edits. Both cheap to close in the same pass — `.claude/skills/run-issue/SKILL.md:48,344`

### Verified, not flagged
- **The auto-mode tell now works as specified, confirmed firsthand.** In this dispatched sub-agent the `While auto mode is active:` reminder was injected **once**, attached to the first tool result, and did not recur across ~25 subsequent tool calls — so "present anywhere in the whole context" is the check that works and the per-turn form would indeed have failed. I later read both files containing the sentinel verbatim and could still tell the injection from the quotations, so the second trap is stated correctly too. One nit only: the doc asserts it is a `system-reminder`; it arrived without a visible tag, so the tag is not itself verifiable from inside — the discriminator the doc already gives (injected session text, never file content you read) is the one that carries.
- **The container containment paragraph now verifies clause by clause against `main`**, apart from must-fix 4 and the scratchpad nuance: bind mount at `:509`, worktrees at `:596-597`, `.agent:ro` at `:512`, §6 `:599-616`, `CLAUDE_CODE_OAUTH_TOKEN` at `:688`, `GH_TOKEN` at `:690`, anonymous `build`/`install`/`log` volumes at `:528-546`.
- **The allowlist and worktree bullets are exact**: 31 `allow` entries and no `deny` in `.claude/settings.json`; no `permissions` key at all in `~/.claude/settings.json`; the untracked local `ask` list is precisely `merge_pr.sh` (three forms), `gh pr merge`, `make merge-pr`, `tmux send-keys`. The ADR-0004/0005 disambiguation added in `052cc0b` is correct — the script header's "convention-only" sentence is about the exit contract.
- **The #604 evidence holds**: nine `##` entries, three `## Implementation` entries each carrying an `**Addressed**` field (address-findings), and the implementation pass unentried between `## Plan Review` and the first `## Local Review (Pre-Push)`.
- **The `review-plan` heuristic fix is sound**: `$AGENT_NAME` is a framework-level constant (`set_git_identity_env.sh:110,134`), so the name comparison could never discriminate; the invocation-based test is one a reader can apply — I applied it to myself.
- **Consequence sweep clean**: no other file in `.claude/`, `.agent/`, `AGENTS.md`, `CLAUDE.md`, `docs/decisions/` still carries the retired "containers for prompt-free dispatch" rule.
- Commit identity correct on all 32 commits (`Claude Code Agent <roland+claude-code@ccom.unh.edu>`); tree clean; nothing pushed.
- Left alone as instructed: the `#606` citation (real, merged as `a00193c`), the safety reasoning covering both dispatch paths, and the `review-plan` fix staying in this PR.

### Governance

| Principle | Verdict | Notes |
|---|---|---|
| Human control and transparency | Concern | Third consecutive round with a false containment clause in the same paragraph — this time understating credential exposure ("no GitHub credentials enter") and overstating the in-process residual guards. Both mislead the agent deciding where untrusted-input work goes. |
| Documentation accuracy (verify against source) | Concern | The fix pass's "every citation re-checked, all verified" is not sound: half were checked against `main`, half against the branch's stale copies, and the 5f claim was inverted by trusting the stale copy over the review. |
| A change includes its consequences | Concern | `.devcontainer/agent/README.md:3` still leads with the "filesystem isolation as the security boundary" framing this PR disproves, and is cited as authority for it; `run-issue/SKILL.md:48` now contradicts the new text. |
| Capture decisions, not just implementations | Pass | Rationale recorded inline in both files with the #545 → #607 lineage; the plan's Implementation Notes are rationale-only per `plan-task` rule 2 (round-2 finding closed). |
| Enforcement over documentation | Watch | Advisory prose only, as #545 was; `dispatch_subagent.sh` requires an explicit `--mode`, so there is no default to enforce. |
| Only what's needed / Improve incrementally | Pass | Prose-only, three files plus the plan; the third is operator-approved. |
| Test what breaks | N/A | No enforced logic. |
| Primary framework first, portability where free | Pass | The non-Claude-runtime case now gets its own bullet and an executable instruction (round-2 must-fix 3 genuinely closed). |

| ADR | Triggered | Compliant | Notes |
|---|---|---|---|
| 0001 — Adopt ADRs | Watch, not required | Yes | Parity with #545's prose-only precedent. |
| 0004 / 0005 — Enforcement hierarchy | Yes | Yes | Now cited only as the lens, with the misattribution corrected. |
| 0013 — progress.md vocabulary | Yes | Yes | Entry types correct throughout. |
| 0015 — Dispatch handoff context contract | Yes | Concern | The fence's condition is described correctly but glossed as container-only (suggestion 2), and the exposed-phase list is short (suggestion 3). |

### Round-2 closure check
- Genuinely closed: must-fix 2 (the tell — verified firsthand, see above); must-fix 3 (non-Claude runtime split into its own bullet with an executable instruction); must-fix 4 (worktree citation + ADR disambiguation); suggestions 6, 7 (partially — see must-fix 3 here), 8, 9, 10, 11, 12, 13, 14.
- Closed in form, not in substance: must-fix 1 (container half rewritten and now accurate on mounts and machine state, but the credential clause is absolute in the wrong direction — must-fix 4 here); suggestion 5 (softened correctly, then re-broken by the 5f inversion — must-fix 1 here).
- **The fix pass's claim to have broken the recurrence pattern is half-true.** It did re-read neighbours and it did find three unflagged defects on its own, two of which were real (the duplicated containment clause in the mode-definition bullet; the plan's stale mechanism claim). But the third — overruling the review on 5f — inverted a fact, because the re-verification used the branch's stale copy of the file rather than `main`. The pattern narrowed (round 1: whole paragraphs false; round 2: half-paragraphs; round 3: single clauses) but did not close.

### Convergence
Another **full** multi-specialist round is not worth the operator's time on a prose diff — the four must-fixes each come with the replacement wording above and none is a design question. What is non-negotiable is the order: **merge `main` into the branch first**, because round 3 proved the branch's own copies are an unreliable verification baseline (they are what produced must-fix 1 and 2), then apply the fixes, then a **targeted** re-check limited to (a) every `file:line` citation re-resolved against the merged tree and (b) the two recurring paragraphs — container containment and in-process containment — read clause by clause with a source line recorded for each clause, not just the ones named here. That is the cheap path to convergence; a fourth full round is not.

## Implementation
**Status**: complete
**When**: 2026-08-24 11:35 -04:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-607 at `70139c9`
**Addressed**: `## Local Review (Pre-Push)` (2026-08-24 10:50 -04:00, branch at `b33e250`, Round 3 — 4 must-fix, 7 suggestions)
**Commits**: 5b3c32e, 36371e6, 638301a, 42cde4f, 1465d59, 70139c9

Round 3's precondition was met before any fix: `main` is merged into the branch
(`0ba4957`, clean) and `git merge-base --is-ancestor main HEAD` now passes, so
every claim and citation below was re-resolved against the **merged** tree, not
against the branch's stale copies. No verification recorded in an earlier entry
was reused.

### Actions
- [x] (must-fix) 5f is **opt-in via `--local`**, not on by default — corrected in both files; `--no-local` named as the deprecated no-op — `run-issue/SKILL.md:139-146`, `skill_workflows.md:154-158`
- [x] (must-fix) Every citation re-resolved against the merged tree (audit table below); `review-code/SKILL.md:333-335` → `:343-346` — `run-issue/SKILL.md:139`
- [x] (must-fix) The in-process permission bullet now deflates containment: `.claude/settings.json` has **no** `ask` rules and exactly **one** `PreToolUse` hook (matcher `AskUserQuestion`, `… || true`, warn-only), which cannot match an edit/commit/shell call — `run-issue/SKILL.md:219-227`
- [x] (must-fix) "no GitHub credentials enter" → GitHub **write** auth, said consistently; the README claim re-attributed (it *leads* with filesystem isolation, and its own absolute is the one the paragraph narrows) — `run-issue/SKILL.md:197-202,212-217,246-248`, `skill_workflows.md:167-170`
- [x] (suggestion) "silently dropping the cross-model pass" removed with the rewritten 5f clause (5f is off by default, so nothing is dropped) — `run-issue/SKILL.md:139-146`
- [x] (suggestion) The "i.e. only on the container path" gloss deleted; `--context-file` is orthogonal to `--mode` and stated as such with sources — `run-issue/SKILL.md:165-179`
- [x] (suggestion) The fence-loss bullet generalized from "`review-issue`'s job too" to *any phase that fetches third-party text itself*, naming `plan-task`/`review-plan`/post-PR `review-code` as further `comments` fetchers — `run-issue/SKILL.md:165-179`
- [x] (suggestion) `.agent/scratchpad` punched back read-write over the `.agent:ro` overlay now stated, with both line cites — `run-issue/SKILL.md:207-208`
- [x] (suggestion) "no GitHub *write* auth **ever**" → dropped "ever"; read-only-ness noted as a filename convention the script does not validate — `skill_workflows.md:171-175`
- [x] (suggestion) Off-by-one closed: the handoff quote is `dispatch_subagent.sh:468-469` — `run-issue/SKILL.md:237`
- [x] (suggestion) Both stale neighbours closed: `:48` no longer says "no GitHub read auth" flatly, and the post-PR heading cite is now `review-code/SKILL.md:1043-1051` — `run-issue/SKILL.md:48-49,344`

### Targeted re-check (a): every `file:line` in the diff, re-resolved

Each was read at the cited line in the merged tree; the "resolves to" column is
what is actually there.

| Citation | Resolves to | OK |
|---|---|---|
| `docker_run_agent.sh:509` | `MOUNT_ARGS+=(-v "$ROOT_DIR:$ROOT_DIR")` | yes |
| `docker_run_agent.sh:512` | `.agent` re-mount with `:ro` | yes |
| `docker_run_agent.sh:526` | `.agent/scratchpad` read-write mount (was `:515` pre-merge — #602 moved it) | yes, corrected |
| `docker_run_agent.sh:546` / `:591` | anonymous `build`/`install`/`log` volume, main layers / worktree layers | yes, newly cited |
| `docker_run_agent.sh:596-597` | both worktree trees mounted read-write | yes |
| `docker_run_agent.sh:599-616` | §6 — `~/.claude.json`, `~/.claude/settings.json`, `~/.claude/.credentials.json` | yes |
| `docker_run_agent.sh:651-665` | `AGENT_GH_TOKEN` override → `gh-readonly-token` file → `export GH_TOKEN`; no scope validation | yes |
| `docker_run_agent.sh:688` / `:690` | `CLAUDE_CODE_OAUTH_TOKEN` / `AGENT_GH_TOKEN:+-e GH_TOKEN` | yes |
| `dispatch_subagent.sh:312-317` | the "`--context-file` is ORTHOGONAL to the task source" comment | yes |
| `dispatch_subagent.sh:461-463` / `:498` | `CONTEXT_SECTION` spliced into `HANDOFF` / `if [ "$MODE" = "in-process" ]` — splice precedes the branch | yes |
| `dispatch_subagent.sh:468-469` | "You are a fresh-context sub-agent … Work only / within this issue's worktree…" | yes, corrected from `:469` |
| `review-code/SKILL.md:343-346` | "use the `Agent` tool … otherwise evaluate / sequentially." | yes, corrected from `:333-335` |
| `review-code/SKILL.md:256` | `gh pr view … ,comments,reviews,…` | yes |
| `review-code/SKILL.md:1043-1051` | post-PR header prose + the `## Local Review` template heading | yes, corrected from `:864-866, 872` |
| `plan-task/SKILL.md:36` | `gh issue view <N> --json …,comments,url` | yes |
| `review-plan/SKILL.md:112` | `gh issue view "$ISSUE_NUM" … --json …,comments,url` | yes (both `review-plan` hunks in this PR start at `:368`, so `:112` is unshifted) |
| `AGENTS.md:585` | `check_question_context.py` row — "Warn-only … never blocks a checkpoint" | yes |
| `.devcontainer/agent/README.md:3-5` | "container filesystem isolation as the security boundary. No GitHub credentials enter the container…" | yes |

### Targeted re-check (b): the two recurring paragraphs, clause by clause

**Container containment** (`run-issue/SKILL.md:194-214`):

| Clause | Source |
|---|---|
| separates the modes = GitHub write auth + machine state, not file access | `docker_run_agent.sh:509` (whole root rw) vs. `:690` (only optional read token) |
| isolates OS/dependency state | container image; no host `/` mount — mount list is `:505-616` |
| build artifacts are anonymous volumes | `:546`, `:591` |
| withholds GitHub **write** auth → cannot push / open a PR | `:648-665` (only a nominally read-only token is ever exported); push runs on the host via the gateway, `:797` |
| "not *all* GitHub credentials" | `:690` forwards `GH_TOKEN` when configured |
| README states the absolute; leads with filesystem isolation | `.devcontainer/agent/README.md:3-5` |
| workspace root bind-mounted rw at the same absolute path | `:509` |
| both worktree trees mounted rw again | `:596-597` |
| only `.agent/` re-mounted `:ro` | `:512` |
| `.agent/scratchpad` punched back rw over it | `:526` |
| host Claude credentials mounted | `:599-616` |
| `CLAUDE_CODE_OAUTH_TOKEN` forwarded | `:688` |
| optional read-only GitHub token forwarded as `-e GH_TOKEN` | `:690` |
| read-only-ness is filename convention, unvalidated ⇒ "no write auth" is configuration, not enforcement | `:651-665` (`AGENT_GH_TOKEN` override accepted verbatim; no scope check) |
| does not put host files out of reach | `:509`, `:596-597` |

**In-process containment** (`run-issue/SKILL.md:216-248`):

| Clause | Source |
|---|---|
| auto mode approves the routine calls | host session permission mode; `~/.claude/settings.json` has an `autoMode` key and **no** `permissions` key |
| tracked baseline defines **no** `ask` rules | `.claude/settings.json` → `permissions` has only `allow` (31 entries) |
| the machine's `ask` rules are untracked | `.claude/settings.local.json` → `permissions.ask` = `tmux send-keys`, `merge_pr.sh` (three path forms), `make merge-pr`, `gh pr merge` |
| exactly **one** `PreToolUse` hook, matcher `AskUserQuestion`, `… \|\| true` | `.claude/settings.json` → `hooks.PreToolUse` (single entry) |
| warn-only, "never blocks" | `AGENTS.md:585` |
| cannot match an edit / commit / shell call | its matcher is `AskUserQuestion` — no Edit/Write/Bash matcher exists in the file |
| allowlist carries no `deny`; `~/.claude/settings.json` neither | `.claude/settings.json` (`allow` only); user settings has no `permissions` key at all |
| worktree scoping is handoff prose, unenforced | `dispatch_subagent.sh:468-469` |
| `skill_workflows.md` says "no filesystem isolation" | `skill_workflows.md:103` |
| the script header's "convention-only … ADR-0004/0005" is about the exit contract | `dispatch_subagent.sh:7-8` |
| phase runs with the host's own GitHub **write** auth | host `gh` auth; the container's counterpart is `:648-665` |

### Not re-opened (per operator instruction)
- The `#606` citation (real PR, merged as `a00193c`) — left as-is.
- The safety reasoning covering both dispatch paths — left as-is.
- The `review-plan` self-review heuristic fix staying in this PR — left as-is.

### Plan sync
`plan.md` needed no edit: it carries no `file:line` citations and its
Implementation Notes already describe the sandbox's contribution as "a clean
machine plus the absence of GitHub write auth", which is what the corrected
prose now says.

### Local checks
Pre-commit ran clean on all six commits (no `--no-verify`); no linter profile
applies to `.md`. Tree clean; nothing pushed. Residual-claim sweep over both
files finds no surviving "credentials enter", "on by default", "silently", or
"no GitHub read auth" text.

## Integrated Review
**Status**: complete
**When**: 2026-08-24 14:53 -04:00
**By**: Claude Code Agent (Claude Opus 5)

**PR**: #608 at `517faa9`
**Sources**: 3 (Copilot R1 @ `517faa9`, Local Review (Pre-Push) R3 @ `517faa9`, CI rollup)
**Cross-source confirmations**: 2
**CI**: all-pass (9 checks: Lint, Script tests, Validate Documentation, commit identity)

11 inline comments, 0 conversation comments, resolving to 9 distinct findings
(two are the same finding mirrored across both files). **No false positives** —
every comment was checked against the local file at the cited line and holds.

### Findings
- [x] (cross-confirmed) The credential boundary is still stated as an absolute in two places despite `AGENT_GH_TOKEN` being accepted and forwarded with no scope validation: "it withholds GitHub **write** auth: a container run that goes wrong cannot push" and "Containers have no GitHub *write* auth". Each is qualified later in its own paragraph, so the text argues with itself rather than being simply wrong — but the flat claim is what a skimming reader takes. Round 3 raised this and the fix pass corrected the mirrors while leaving these two leads — `.claude/skills/run-issue/SKILL.md:199`, `.agent/knowledge/skill_workflows.md:175`
- [x] (cross-confirmed) `.devcontainer/agent/README.md:3-5` still tells users "No GitHub credentials enter the container". This PR cites that line as the false absolute it is correcting, but does not fix it — so anyone following `make agent-run` still reads a credential boundary we now document as untrue. Round 3 asked for re-attribution; that was done in run-issue's prose, not in the README itself — `.devcontainer/agent/README.md`
- [x] (must-fix, Copilot) "Choose `container` regardless of mode when isolation is the actual requirement: anything processing untrusted input" overstates the boundary, and contradicts this PR's own containment paragraph 40 lines later: the launcher bind-mounts the workspace and both worktree trees read-write and forwards `CLAUDE_CODE_OAUTH_TOKEN`, so a prompt-injected phase can still alter host-visible files and use the host credential. Container dispatch is partial OS/dependency isolation, not sufficient containment on its own; the data-fence requirement should stay — `.claude/skills/run-issue/SKILL.md:160`
- [x] (must-fix, Copilot) The fail-safe branch reintroduces "or the operator has permission prompts enabled" as a decision input immediately after the same section establishes that the operator's permission-mode indicator is NOT visible to the agent. The executable rule is the sentinel check alone. This is a recurrence of Round 3's must-fix 1, in both files — `.claude/skills/run-issue/SKILL.md:134`, `.agent/knowledge/skill_workflows.md:151`
- [x] (valid, Copilot) "so it stays in-process in every mode" is unfollowable on a non-Claude runtime, where the same paragraph has just said there is no in-process option at all. Say it stays in-process where an `Agent` tool is available, with container as the degraded fallback — `.agent/knowledge/skill_workflows.md:158`
- [x] (valid, Copilot) "(`triage-reviews` needs both)" is wrong: it fetches reviews, comments and check-runs and writes local progress.md, and its own Guidelines say it performs no GitHub review actions. It needs **read** access only, which is also what `run-issue/SKILL.md:159` says — the two files disagree — `.agent/knowledge/skill_workflows.md:170`
- [x] (valid, Copilot) Plan says `review-code` "cannot be containerized in any mode"; the landed guidance treats container execution as supported but degraded. The plan contradicts the implementation it is meant to track. (Traceable to the host's dispatch brief, not to the fix pass) — `.agent/work-plans/issue-607/plan.md:61`
- [x] (valid, Copilot) Consequences row records "no inline citation update for skill_workflows.md", but the landed diff adds the #545 → #607 citation at `skill_workflows.md:163-164` — `.agent/work-plans/issue-607/plan.md:141`
- [x] (valid, Copilot) Plan describes #606 as an open PR; it merged as `a00193c`. Date it as historical rather than restating it as current — `.agent/work-plans/issue-607/plan.md:33`

### False positives
- None. All 11 comments were verified against the local file at the cited line; each describes a real defect in the current head.

## Implementation
**Status**: complete
**When**: 2026-08-24 15:03 -04:00
**By**: Claude Code Agent (Claude Opus)

**PR**: #608 at `3544e84`
**Addressed**: `## Integrated Review` (2026-08-24 14:53 -04:00, PR #608 @ `517faa9`) — 9 open findings, 0 false positives
**Commits**: `025e66a`, `c2d8853`, `5aee510`, `eca0faa`, `f15fff3`, `a0d6294`, `3ca6d4f`, `d6c4e46`, `3544e84`

All nine actioned; none deferred. Two extra neighbouring claims were corrected
in the same pass (listed below) because the round-3 → round-4 recurrence in this
issue was exactly the "fixed the named clause, left the adjacent one" pattern.

### Actions
- [x] (cross-confirmed) Credential-boundary lead in `run-issue` no longer states
  the absolute: it now reads "*configured* without GitHub **write** auth — no
  SSH keys and no `~/.config/gh` are mounted", with "configured" glossed
  inline as what the launcher forwards rather than what the container enforces
  (verified: `docker_run_agent.sh` mounts neither path; token forwarded at
  `:690`, unvalidated at `:651-665`) — `.claude/skills/run-issue/SKILL.md`
  (`c2d8853`)
- [x] (cross-confirmed) `skill_workflows.md`'s mirror of the same claim
  rewritten. The bullet had stated the boundary **twice** — once in the main
  flow, once in a parenthetical — so it was restructured into a lead plus two
  sub-bullets that say it once, correctly — `.agent/knowledge/skill_workflows.md`
  (`a0d6294`)
- [x] `.devcontainer/agent/README.md` fixed rather than only cited. The opening
  no longer claims filesystem isolation as the security boundary or "No GitHub
  credentials enter the container"; § Security Model now states that
  `AGENT_GH_TOKEN` is forwarded as `GH_TOKEN` with no scope validation, and that
  the host's Claude Code credentials (`:599-616`) and `CLAUDE_CODE_OAUTH_TOKEN`
  (`:688`) do enter — `.devcontainer/agent/README.md` (`025e66a`)
- [x] (must-fix, Copilot) The container-for-isolation rule no longer implies a
  container is sufficient containment for untrusted input. It now names the
  clean OS/dependency set as the isolation containers really give, states the
  bind-mount + forwarded-credential gap, and keeps the **data fence** as the
  requirement the phase holds in either mode — `.claude/skills/run-issue/SKILL.md`
  (`5aee510`), mirrored in `.agent/knowledge/skill_workflows.md` (`a0d6294`)
- [x] (must-fix, Copilot) The non-observable "operator has permission prompts
  enabled" condition removed from the fail-safe branch in **both** files; the
  rule is now the sentinel check alone, said explicitly —
  `.claude/skills/run-issue/SKILL.md`, `.agent/knowledge/skill_workflows.md`
  (`eca0faa`)
- [x] (valid, Copilot) "stays in-process in every mode" replaced with "run it
  in-process wherever an `Agent` tool exists … on a runtime that has none, the
  degraded container run is the fallback, not a preference" —
  `.agent/knowledge/skill_workflows.md` (`f15fff3`)
- [x] (valid, Copilot) `triage-reviews` needs GitHub **read**, not both; both
  files now say read, with the source cite (`triage-reviews/SKILL.md:412`,
  "No GitHub review actions") — `.agent/knowledge/skill_workflows.md`,
  `.claude/skills/run-issue/SKILL.md` (`a0d6294`)
- [x] (valid, Copilot) Plan's "cannot be containerized in any mode" corrected to
  the landed position: a container run of `review-code` is **degraded** (no
  `Agent` tool, no specialist 5f), not impossible, with in-process preferred
  where an `Agent` tool exists — `.agent/work-plans/issue-607/plan.md:61`
  (`3ca6d4f`)
- [x] (valid, Copilot) Consequences row corrected inline: the landed edit *did*
  add the `#545 → #607` citation to `skill_workflows.md`, so the row now records
  that as a revision rather than claiming none was needed —
  `.agent/work-plans/issue-607/plan.md:141` (`3ca6d4f`)
- [x] (valid, Copilot) #606 re-dated as historical — merged as `a00193c`
  (verified via `git log`), not an open PR — `.agent/work-plans/issue-607/plan.md:33`
  (`3ca6d4f`)

### Neighbouring claims corrected in the same pass (not separately flagged)
- The closing caution still called the sandbox "the safeguard you are relying
  on" for untrusted input — the exact claim finding 3 removed 100 lines above.
  Rewritten to keep the residual caution (round-3's suggestion) while resting it
  on credentials and machine state, with the data fence named as what actually
  guards untrusted input — `.claude/skills/run-issue/SKILL.md` (`d6c4e46`)
- The `dispatch_subagent.sh` example comment in `skill_workflows.md` still read
  "use for isolation" — narrowed to "a clean dependency environment"
  (`3544e84`). The `run-issue` sub-bullet's "rather than delegating it to a
  sandbox" was likewise reworded, since the parent bullet now says a sandbox
  never held that fence for you (`5aee510`).
- Three further README inaccuracies of the same class, found while fixing the
  credential claim: Prerequisites listed only `ANTHROPIC_API_KEY` (the launcher
  accepts `CLAUDE_CODE_OAUTH_TOKEN`, `ANTHROPIC_API_KEY`, or a host
  `.credentials.json`, and errors out with none — `:317-329`); the
  "Container won't start" checklist tested only the API key; and "Agents still
  **cannot** push" was stated as a property rather than a consequence of minting
  the PAT read-only (`025e66a`)

### Checks
Pre-commit ran clean on every commit (no `--no-verify`, no rule suppressed).
Changes are Markdown only — no build or package test applies. New README anchor
links (`#mount-strategy`, `#security-model`, `#read-only-github-access`) resolve
to existing headings.

## Integrated Review
**Status**: complete
**When**: 2026-08-25 22:17 -04:00
**By**: Claude Code Agent (Claude Opus 5 (1M context))

**PR**: #608 at `b6e94f3`
**Sources**: 3 (Copilot R2 @ `b6e94f3`, prior `## Integrated Review` R4 @ `517faa9` + its `## Implementation` @ `3544e84`, CI rollup)
**Cross-source confirmations**: 0 same-SHA; **2 cross-round recurrences** (a round-4 finding fixed at the cited site and re-stated at an uncited one in the same pass)
**CI**: failures-noted — 9 checks at this head; the `pull_request` run is fully green, the `push` run's `Lint (pre-commit)` failed in bootstrap with "Failed to determine ROS apt source version from GitHub API" (infra/rate-limit, not this diff). `main` carries no branch protection, so nothing is blocking; AGENTS.md § Merging still says don't merge the workspace repo on a red check — re-run that job.

Copilot's earlier review (11 comments @ `517faa9`) is fully triaged and closed by
the round-4 entry and its fix pass; it is stale and not re-litigated here. This
round covers the 4 live comments at `b6e94f3`. **No false positives** — each was
checked against the local file at the cited line and holds.

### Findings
- [x] (must-fix, Copilot; recurrence of R4 finding 3) `review-plan`'s Next-step block still offers `--mode container` "when the work is isolation-worthy (untrusted input, ...)" — the exact claim this PR removed from `run-issue/SKILL.md:160` and `skill_workflows.md` in the same pass. A plan reviewer following this line is routed into the mode the PR now documents as *not* sufficient containment (workspace and both worktrees bind-mounted rw; `CLAUDE_CODE_OAUTH_TOKEN` forwarded). Restrict the container reason to the clean OS/dependency environment and point untrusted input at the data fence — `.claude/skills/review-plan/SKILL.md:463`
- [x] (must-fix, Copilot; recurrence of R4 cross-confirmed finding 1) § Security Model closes with the flat "What does *not* enter is GitHub **write** credentials" — three lines after the same section states that a write-capable PAT in `AGENT_GH_TOKEN` is forwarded as `GH_TOKEN` unvalidated and *does* give the container write auth. Verified: no `gh auth setup-git`, no credential helper, no SSH keys, so `git push` does fail as the bullet says — but `gh pr create` / `gh api -X POST` would not. Qualify the sentence to the genuinely read-only configuration — `.devcontainer/agent/README.md:212`
- [x] (valid, Copilot) The troubleshooting check prints the first 10 characters of `CLAUDE_CODE_OAUTH_TOKEN` — a long-lived credential this PR's own prerequisites newly recommend — into terminal scrollback and screenshots, for no diagnostic gain over a presence test. It also does not cover the `~/.claude/.credentials.json` fallback it names in the same line. Replace with a non-printing presence check across all three sources — `.devcontainer/agent/README.md:416`
- [x] (valid, Copilot) The "Container auth not ready either?" fallback says "run in-process anyway", 6 lines after the bullet establishing that a non-Claude runtime has no in-process option at all (`the Agent tool *is* in-process dispatch`). For that reader the stated fallback is unexecutable; the real one is the manual drive named in the preceding bullet. Add the clause — `.claude/skills/run-issue/SKILL.md:155`

### False positives
- None. All 4 comments were verified against the local file at the cited line; each describes a real defect at the current head, and all 4 sit on lines this PR added or changed.

### Note for the next fix pass
Both must-fix findings are the *same failure mode* the round-4 `## Implementation`
entry named in its own closing note — "fixed the named clause, left the adjacent
one". Round 4 corrected the untrusted-input claim in two files and the
credential absolute in three, each time at the site Copilot cited. The two sites
above were the uncited mirrors. Sweep for both claims repo-wide, not at the cited
lines only.

## Implementation
**Status**: complete
**When**: 2026-08-25 22:23 -04:00
**By**: Claude Code Agent (Claude Opus)

**PR**: #608 at `264f4d1`
**Addressed**: `## Integrated Review` (2026-08-25 22:17 -04:00, PR #608 @ `b6e94f3`) — 4 open findings, 0 false positives
**Commits**: `4b7bb8d`, `d47ad83`, `113551b`, `fc742c8`, `73962bd`, `264f4d1`

All four actioned; none deferred. Per the review entry's "Note for the next fix
pass", both claims were swept repo-wide before fixing rather than at the cited
lines only — the sweep is recorded below, and it turned up one further mirror of
the credential absolute (in `run-issue/SKILL.md` itself), corrected in the same
commit.

### Actions
- [x] (must-fix, Copilot; R4 finding 3 recurrence) `review-plan`'s Next-step
  block no longer offers `--mode container` for "isolation-worthy (untrusted
  input …)" work. The container reason is now the clean OS/dependency
  environment; the bind-mount + forwarded-`CLAUDE_CODE_OAUTH_TOKEN` gap is
  stated, and untrusted input is pointed at the **data fence**, held by the
  phase in either mode, cross-referenced to `run-issue/SKILL.md`
  § How phases are dispatched, *What contains a dispatched agent* —
  `.claude/skills/review-plan/SKILL.md:462-471` (`4b7bb8d`)
- [x] (must-fix, Copilot; R4 cross-confirmed finding 1 recurrence) § Security
  Model's flat "What does *not* enter is GitHub **write** credentials" replaced
  with the boundary as it actually holds: `git push` fails unconditionally
  (verified — no SSH keys, no `~/.config/gh`, and no credential helper or
  `gh auth setup-git` anywhere in the launcher or entrypoint), while `gh`
  authenticates from `GH_TOKEN` alone, so a write-capable PAT in
  `AGENT_GH_TOKEN` makes `gh pr create` / `gh api -X POST` succeed. The
  following paragraph's "All pushes and PR creation happen on the host" was
  re-scoped to match — `.devcontainer/agent/README.md:212-220` (`d47ad83`)
- [x] (valid, Copilot) The "Container won't start" auth check no longer prints
  the first 10 characters of `CLAUDE_CODE_OAUTH_TOKEN`. It is now a
  non-printing presence test across all three sources the launcher accepts —
  env token, `ANTHROPIC_API_KEY`, and `~/.claude/.credentials.json` — using
  the launcher's own `-n`/`-f` predicates, with the reason for not echoing
  stated inline — `.devcontainer/agent/README.md:426-437` (`113551b`,
  `264f4d1`)
- [x] (valid, Copilot) The "Container auth not ready either?" fallback now names
  the executable route for a runtime with no `Agent` tool: "there, drive the
  phase manually, as the bullet above says" — `.claude/skills/run-issue/SKILL.md:152-157`
  (`fc742c8`)

### Repo-wide sweep (per the review's note)
- **Container-as-containment-for-untrusted-input**: `grep` over `.md`/`.sh`
  for `isolation-worthy`, `untrusted`, and `--mode container` across the
  worktree. The only live surface was `review-plan/SKILL.md:463` (finding 1).
  `run-issue/SKILL.md` and `skill_workflows.md` already carry the corrected
  form; remaining hits are `review-code`'s unrelated
  `--allow-untrusted-copilot` gate, `dispatch_subagent.sh`'s data fence, and
  historical work-plan text.
- **The GitHub-write absolute**: `grep` for `write credential`,
  `GitHub credential`, `no credentials`, `credential-free`, `write auth`,
  `write-level`. Two live sites: `README.md:212` (finding 2) and one *mirror
  not separately flagged* — `run-issue/SKILL.md:207` still concluded "cannot
  push and cannot open a PR" as an absolute. Corrected in `d47ad83` to "cannot
  `git push` at all, and cannot open a PR either unless the optional
  `GH_TOKEN` was minted with write scopes". `docs/decisions/0015`'s "no
  GitHub write auth" was left alone deliberately: an accepted ADR is a
  historical record of the decision, not live guidance.

### Plan sync
The Files-to-Change table listed three files while the branch touches five —
`.devcontainer/agent/README.md` (added by the round-4 fix pass) and
`.claude/skills/review-code/SKILL.md` (round 2) were never recorded. Both rows
added inline with why they entered scope, and Estimated Scope updated from
"three files" to five (`73962bd`). Not a listed finding; it is the same
plan-drift class rounds 3-4 flagged and a direct consequence of this pass
editing the README again.

### Checks
Pre-commit ran clean on all six commits (no `--no-verify`, no rule suppressed).
Markdown only — no build or package test applies. Verified: the three README
anchors cited by other files (`#mount-strategy`, `#security-model`,
`#read-only-github-access`) still resolve to existing headings; the new
`review-plan` cross-reference resolves to the real `## How phases are
dispatched` heading (`run-issue/SKILL.md:38`); `docker_run_agent.sh:317-328`
re-read to confirm the three-source auth check and its error path.


## Local Review
**Status**: complete
**When**: 2026-08-25 22:41 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**PR**: #608 at `53cebac`
**Mode**: post-PR
**Depth**: Standard (reason: governance-touching instruction files other agents read as authority; +383/-63 prose across five files plus the plan)
**Must-fix**: 8 | **Suggestions**: 14
**Round**: 6 | **Ship**: continue — must-fix rose 4 → 8, and two are not wording: the cannot-confirm branch leaves three phases with no mode at all, and the data fence the PR installs as the primary countermeasure for untrusted input does not exist in any of the skills it assigns it to. The recurrence the round-5 note warned about DID recur, in three places.

Specialists: static analysis (no linter profile for `.md`; no trailing whitespace on added lines; all README anchors resolve; pre-commit clean), governance, plan drift, Claude Adversarial x2 (Lens A + Lens B, fresh-context sub-agents). Copilot off (default) — its 15 inline comments are all from `517faa9`/`b6e94f3`, fully triaged by the two `## Integrated Review` entries; no new review at this head. Local Adversarial off (default). **CI: all-green at `53cebac`** (both the `pull_request` and `push` runs; the round-5 Lint bootstrap failure did not recur).

### Findings
- [x] (must-fix) The GitHub-write absolute recurs a third time, in the section that exists to deflate it: `:274-276` concludes "the one arrangement in which a phase that goes wrong reaches neither GitHub **write** auth nor the host's machine state" — flatly, 60 lines after `:210-211` says it "cannot open a PR either **unless** the optional `GH_TOKEN` was minted with write scopes". `:192-193`'s "it has **no GitHub *write* auth**" is the same construction Copilot ruled valid twice (R4 finding 1, R5 must-fix 2). Lead only — neither adversarial pass flagged it — `.claude/skills/run-issue/SKILL.md:192-193,274-276`
- [x] (must-fix) The same absolute is live in a **script** this PR points readers at: the `--context-file` fence ends "— this dispatch has no GitHub read auth." The PR itself establishes `--context-file` is orthogonal to `--mode`, so an **in-process** `review-issue` under the new default is told it has no read auth while sitting in a session that does — and told to avoid `gh`. Triple cross-confirmed (Lens A + Lens B + lead). A docs-only PR may discharge this by filing a follow-up, but not by silence — `.agent/scripts/dispatch_subagent.sh:436-437` (deferred: operator scope decision — the script is not edited in this PR; the host is filing it as a follow-up issue. **PLACEHOLDER: the host must splice the follow-up issue number in here and in the PR body.** In-repo prose mirroring the same claim *was* corrected — `review-issue/SKILL.md:87`)
- [x] (must-fix) Internal contradiction across two adjacent bullets, and a divergence from the mirror file: `:167` bolds "Choose containers for a clean dependency environment, **not for prompt volume**" while `:150-151` — the bullet directly above — routes implement/address-findings to containers precisely for prompt volume ("go to containers, which run prompt-free"). `run-issue/SKILL.md:158` carries no such clause. Intended meaning is presumably "not for prompt volume *under auto mode*" — `.agent/knowledge/skill_workflows.md:167`
- [x] (must-fix) Unrouted branch: on the "cannot confirm auto mode" leaf, `review-issue`, `plan-task` and `review-plan` are assigned **no mode at all**. The diff deleted both surfaces that used to cover them (the old "**in-process** for **quick / cheap phases**" bullet head, and "Use **`in-process`** for quick/cheap phases"). The branch header reads "the container-leaning guidance is in force", so a reader can plausibly containerize `review-issue` — which then needs `--context-file` — `.claude/skills/run-issue/SKILL.md:131-157`
- [x] (must-fix) "What genuinely survives is `run-issue`'s **checkpoints** … host-enforced and **mode-independent**" over-claims for the mode this PR makes the default. The checkpoints are the orchestrator's own `AskUserQuestion` calls (`:432-443`), and the only hook on `AskUserQuestion` is the warn-only `check_question_context.py … || true`. In **container** mode the push/PR gate is backed materially (no transport). **In-process** the only barrier to `git push` / `gh pr create` is handoff prose (`dispatch_subagent.sh:475-477`) — the same documentation layer the section just demoted for worktree scoping — `.claude/skills/run-issue/SKILL.md:270-272`
- [x] (must-fix) The **data fence** — which both files now name as *the* countermeasure for untrusted input in either mode, and which the new default makes the common path — does not exist in any of the skills the PR assigns it to. `grep` for `untrusted|data, not authority|never as instructions|data fence` returns **zero** hits in `review-issue/SKILL.md`, `plan-task/SKILL.md` and `triage-reviews/SKILL.md`. A dispatched sub-agent loads its own SKILL.md, not the orchestrator's. Either add the sentence to those three, or downgrade the claim to "no fence exists for these phases yet" and file it — `.claude/skills/run-issue/SKILL.md:170-188`, `.agent/knowledge/skill_workflows.md:191-198`
- [x] (must-fix) **Recurrence (a) survives in this PR's own work plan.** `plan.md:56` still frames container's remaining case as "isolation or dependency environment … **untrusted input**, and a clean OS/dependency set", and `:83` still says "reach for container fan-out when the work needs OS-level isolation **(untrusted input)**" — the exact claim the PR removes from all three skill files. Neither bullet's superseding parenthetical retracts it. Cross-confirmed (plan drift + lead) — `.agent/work-plans/issue-607/plan.md:56,83`
- [x] (must-fix) Four further plan claims are false about what landed: `:70-72` + `:192` say the `--context-file` paragraphs "were left alone" (they were edited — `run-issue/SKILL.md:45-49`, commit `42cde4f`); `:150` says "No further instances found" of downstream `--mode container` advice (this PR changed two — `review-code/SKILL.md:878-883` and `review-plan/SKILL.md:453-470`); `:131` says "Three files" where the table and Estimated Scope both say five; `:29-35` instructs leaving the `#606` citation "as-is **in both target files**" — neither target file contains `#606` at all — `.agent/work-plans/issue-607/plan.md:29-35,70-72,131,150,192`
- [x] (must-fix) Two **accepted ADRs** still assert invariants this PR disproves, in the layer agents are told to consult first, with no pointer to the correction: ADR-0004:34 ranks "Container isolation | No | **Prevention by construction**" as the unbypassable layer, and ADR-0015 states the credential boundary absolutely at `:30`, `:41`, `:93` ("**zero GitHub auth inside the sandbox**") and `:103` ("the container touches neither"). Rewriting them is forbidden; ADR-0012 permits a cross-reference addendum, but only pointing at **another ADR** — so the remedy is an ADR (or a filed follow-up issue), not a two-line edit. Cross-confirmed (governance + Lens B) — `docs/decisions/0004-…:34`, `docs/decisions/0015-…:30,41,93,103`
- [x] (suggestion) `triage-reviews/SKILL.md:360` hard-codes `--mode in-process` for the `address-findings` hop, contradicting this PR's own fail-safe branch, which sends `address-findings` to a container when auto mode cannot be confirmed. The other four inter-skill hops are all consistent — `.claude/skills/triage-reviews/SKILL.md:360`
- [x] (suggestion) PR-relative narration baked into a durable doc: "corrected to match **in this PR** — before it, that README led with …" quotes two strings that, post-merge, exist nowhere. Keep the cross-reference, drop the before/after — `.claude/skills/run-issue/SKILL.md:215-219`
- [x] (suggestion) run-issue's `review-code` bullet ends "Run it in-process and accept the prompts" with no fallback for a runtime that has no `Agent` tool; `skill_workflows.md:158-160` states that fallback explicitly. A reader of run-issue alone hits a dead end — `.claude/skills/run-issue/SKILL.md:137-147`
- [x] (suggestion) "Use `in-process` regardless when a phase needs … host GitHub read auth (`triage-reviews`)" reads as if a container has none, which `:48-50` and `:193-195` correctly deny. The real blocker is stated right at `:73-76` (body-only `--context-file` cannot carry reviews / check-runs) — `.claude/skills/run-issue/SKILL.md:167-169`
- [x] (suggestion) The canonical `--context-file` snippet still shows `--mode container` as its only worked example — the same shape as the round-3 finding already fixed in `review-plan` — `.claude/skills/run-issue/SKILL.md:54-57`
- [x] (suggestion) The sentinel is described as arriving "attached to an early tool result". In this dispatched sub-agent it arrived in the **session preamble**, before any tool call. The operative rule ("present anywhere in this session") still holds; the mechanism sentence does not — `.claude/skills/run-issue/SKILL.md:104-105`
- [x] (suggestion) Both files reproduce the `While auto mode is active:` sentinel verbatim and then instruct the reader to discount that very quotation, so every agent making this decision carries a false positive. Consider describing the string rather than printing it — `.claude/skills/run-issue/SKILL.md:110-113`, `.agent/knowledge/skill_workflows.md:138-140` (deferred: the verbatim string is what makes the sentinel recognizable, and both files already instruct the reader to discount a quotation of these docs — describing it risks a reader failing to match the real reminder)
- [x] (suggestion) The new self-review test discards a mechanical signal that exists: `dispatch_subagent.sh:468` writes "You are a fresh-context sub-agent dispatched for **issue #N**" into every handoff — present ⇒ dispatched ⇒ independent. Naming it restores a checkable test and shrinks the "genuinely unsure" branch to near-zero (same fail-safe direction) — `.claude/skills/review-plan/SKILL.md:371-395`
- [x] (suggestion) "state in one line what left you unsure" has no destination — the entry template at `:409` has only the `**By**` line and the annotation. Say where the line goes — `.claude/skills/review-plan/SKILL.md:391-396`
- [x] (suggestion) "Conversely" introduces a restatement, not a contrast: the preceding sentence already says the sandbox holds back "(by configuration) GitHub write auth" — `.agent/knowledge/skill_workflows.md:180`
- [x] (suggestion) Antecedent drift vs. the mirror: the container-auth fallback sits directly after two `review-code` sentences and reads as review-code's fallback, where `run-issue/SKILL.md:152-157` makes it a general branch of the cannot-confirm leaf — `.agent/knowledge/skill_workflows.md:160-162`
- [x] (suggestion) "With none of the three the launcher exits with an error before starting the container" is not unconditional: the guard at `docker_run_agent.sh:317-319` also requires `[ "$SHELL_MODE" = false ]`, so `--shell` launches uncredentialed — `.devcontainer/agent/README.md:146-150,426-427`
- [x] (suggestion) The three framework adapters say only "no auto-dispatch — drive the next skill yourself"; `run-issue/SKILL.md:148-151` now also offers those runtimes `--mode container`. Nothing is falsified — a completeness gap, one line each — `.github/copilot-instructions.md:56-62`, `.agent/instructions/gemini-cli.instructions.md:59-65`, `.agent/AGENT_ONBOARDING.md:88-93` (deferred: **Ask-First** instruction files (`.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md`) — flagged to the operator, not edited)
- [x] (suggestion) `AGENTS.md:560`'s `dispatch_subagent.sh` row still reads "so a **no-GitHub-auth** container phase reads it instead of `gh issue view`" — the phrasing this PR spent three rounds qualifying. **Ask-First file**: flag to the operator, do not edit. Cross-confirmed (governance + Lens B) — `AGENTS.md:560` (deferred: **Ask-First** instruction file — flagged to the operator, not edited)
- [x] (suggestion) Plan completeness: `.devcontainer/agent/README.md` and `review-code/SKILL.md` have no Approach step (step 3 is still headed "Third file"); the `run-issue` row omits the `:45-49` and `:383` edits; the `skill_workflows` row omits the rewritten `# In-process (…)` comment; the README row omits the § Read-only GitHub Access edit; Documentation Impact lists only the two original targets — `.agent/work-plans/issue-607/plan.md:99,116-120,138,154-157`

### Verified, not flagged
- **Every `file:line` citation in the diff resolves**, and resolves *identically* in the branch tree and in the current `origin/main` — checked independently by the lead and by both adversarial passes: `docker_run_agent.sh` 317-328, 333-339, 509, 512, 526, 546, 591, 596-597, 599-616, 651-665, 688, 690; `dispatch_subagent.sh` 312-317, 461-463, 468-469, 498; `review-code/SKILL.md` 256, 343-346, 1043-1051; `plan-task/SKILL.md:36`; `review-plan/SKILL.md:112`; `triage-reviews/SKILL.md:412`; `AGENTS.md:585`. `main` has moved through #609/#611 since the branch merged it, but none of those commits shifted a cited line — the round-3 stale-baseline hazard did not recur.
- **The settings claims are exact**: tracked `.claude/settings.json` has `permissions.allow` (31 entries) and **no** `deny` and **no** `ask`; `~/.claude/settings.json` has no `permissions` key at all; exactly one `PreToolUse` hook, matcher `AskUserQuestion`, `… || true`; the untracked local `ask` list is `merge_pr.sh`, `gh pr merge`, `make merge-pr`, `tmux send-keys`.
- **The README's "`git push` fails unconditionally" verifies**: no SSH keys, no `~/.config/gh`, no credential helper and no `gh auth setup-git` anywhere in the launcher, Dockerfile or entrypoint.
- **The #604 evidence holds**: `.agent/work-plans/issue-604/progress.md` has exactly nine typed entries in the stated shape, with the unentried implementation pass between `## Plan Review` and the first `## Local Review (Pre-Push)`.
- **Live retired advice: none.** Independent sweeps by the lead, governance and Lens B over `.claude/`, `.agent/`, `.devcontainer/`, `.github/`, `docs/`, `AGENTS.md`, `CLAUDE.md`, `Makefile`, `README.md`, `presentations/` found no surviving "lean toward container" / "isolation-worthy" / "implementation-heavy" guidance. Every inter-skill hop already dispatches `--mode in-process`. `presentations/raising-agents/slides.md:308` (delivered March 2026) and the completed work plans for #490/#545/#592/#594 are historical records, correctly untouched.
- Commit identity correct on all commits (`Claude Code Agent <roland+claude-code@ccom.unh.edu>`); atomic commits; tree clean; pre-commit clean, nothing suppressed.

### Existing review comments
All 15 Copilot inline comments are outdated (posted at `517faa9` and `b6e94f3`), fully triaged by the two `## Integrated Review` entries and addressed by the fix passes at `3544e84` and `264f4d1`. No comment is unread and none is live at `53cebac`. No human review comments.

### Governance

| Principle | Verdict | Notes |
|---|---|---|
| Human control and transparency | Pass | The strongest part of the change: the PR deflates its *own* safety story rather than inflating it, and every clause of the container paragraph verifies against source. Two residual over-claims remain (must-fix 1, 5). |
| Documentation accuracy (verify against source) | Concern | The diff's own claims verify; the **plan** does not (must-fix 7, 8), and the credential absolute recurs a third time (must-fix 1, 2). |
| A change includes its consequences | Concern | Live guidance sweep is genuinely clean, but three consequences are unclosed: the fence has no home in the phases assigned it (must-fix 6), two accepted ADRs contradict the PR with no pointer (must-fix 9), and the `address-findings` hop was not aligned. |
| Capture decisions, not just implementations | Concern | A standing default reversed *and* a corrected security model, recorded prose-only across three skill files. #545 being prose-only is the failure being repeated, not the precedent to match — see the ADR recommendation below. |
| Enforcement over documentation | Watch | Unenforceable by construction (an agent reading a sentinel out of its own context), but it fail-safes toward container, which is the right direction. `review-plan`'s self-review test is downgraded from mechanical to introspective when a mechanical marker exists. |
| Only what's needed / Improve incrementally | Pass | Prose-only, five files; the two additions beyond the original two were review-driven and operator-approved, and are recorded as such. |
| Primary framework first, portability where free | Pass | The auto-mode sentinel is unapologetically Claude-Code-specific and the non-Claude runtime gets its own branch. |
| Test what breaks | N/A | No enforced logic. |
| Workspace vs. project separation | Pass | Workspace infra only. |

| ADR | Triggered | Compliant | Notes |
|---|---|---|---|
| 0001 — Adopt ADRs | Yes | Partial | A decision made twice in opposite directions, recorded prose-only both times. Recommendation, not a gate — see below. |
| 0004 — Enforcement hierarchy | Yes | **Contradicted, un-addendumed** | `:34` still ranks container isolation as "Prevention by construction"; this PR proves it prevents neither host-file writes nor host-credential spend. Must-fix 9. |
| 0005 — Layered enforcement | Yes | Pass | The new rule sits in the weakest layer and says so. |
| 0012 — Cross-reference addendums | Yes | **Unused** | The sanctioned mechanism for must-fix 9 — but it points only at another ADR, so it needs one written. |
| 0013 — progress.md vocabulary | Yes | Pass | All entries use canonical headings; frontmatter present. |
| 0014 — Deployment mode | No | N/A | Verified by sweep: `start-deployment` / `deployment_mode.md` "prompt-free" references are about `dlog.sh` allowlisting, a different subject. |
| 0015 — Dispatch handoff contract | Yes | **Contradicted, un-addendumed** | Four flat statements of the credential absolute. Must-fix 9. |

| Changed | Required update | Status |
|---|---|---|
| `.claude/skills/*` (framework skills) | Framework adapter files | Partial — nothing is falsified, but the new non-Claude `--mode container` branch is missing from all three adapters (suggestion) |
| Retired dispatch-mode advice | Every surface repeating it | Done — three independent sweeps found no live surface |
| Corrected credential boundary | Every surface stating the absolute | **Missing** — `dispatch_subagent.sh:436-437`, `AGENTS.md:560`, ADR-0004, ADR-0015 (must-fix 2, 9; suggestion) |
| New "data fence" duty assigned to phases | The phases' own SKILL.md files | **Missing** — zero fence text in `review-issue`, `plan-task`, `triage-reviews` (must-fix 6) |
| Implementation diverged from plan | Plan synced inline per `plan-task` § During implementation | **Missing** — inline in form (Files-to-Change, Estimated Scope, per-bullet parentheticals), but six Approach/Consequences claims remain false (must-fix 7, 8) |

### Recommendation on an ADR
The plan argues no ADR is needed, on parity with #545's prose-only precedent. That symmetry holds if the default flip were the whole change — it isn't. The durable part is the *corrected containment model*, which took six review rounds to get right and directly qualifies two accepted ADRs. Recommend one ADR covering **what actually contains a dispatched agent, and how the mode default follows from it**, with ADR-0012 cross-reference addendums on ADR-0004 and ADR-0015 pointing at it. That is a separate issue and a separate PR; blocking #608 on it would be disproportionate — but must-fix 9 stays open until the follow-up is filed.

### Convergence
Round 6, and must-fix rose 4 → 8, so the mechanical "low and not rising" ship test does not fire. But the composition has changed in a way the count hides: **only five of the eight are defects in the edited prose** (must-fix 1, 3, 4, 5 and the plan pair 7/8), and every one of those comes with its replacement wording. The other three are scope questions — a live absolute in an unedited script, a fence with no home, and two contradicted ADRs — each dischargeable by a filed follow-up issue rather than by more editing here.

The recurrence the round-5 note warned about **did recur, in three places**: the credential absolute at `run-issue/SKILL.md:274-276` and `:192-193` (uncited mirrors of the two sites Copilot cited), the same absolute in `dispatch_subagent.sh:436-437`, and the untrusted-input claim in the PR's own `plan.md:56,83`. The round-5 sweep grepped for `write auth` and `untrusted` and reported these as "historical work-plan text" / already-corrected — the sweep ran, but its *classification* was wrong, not its reach. So the fix this round is not another sweep: it is to re-read the two paragraphs and the plan's Approach section **as a reader would**, and to treat this PR's own plan as live text, not as a historical record.

Cheapest path: one targeted fix pass over the five in-prose must-fixes plus the plan, file two follow-up issues (the script string; the ADR + addendums), and re-check only those sites. A seventh full multi-specialist round is not warranted on a prose diff.

## Implementation
**Status**: complete
**When**: 2026-08-25 22:56 -04:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-607 at `713b24e`
**Addressed**: `## Local Review` (round 6, 2026-08-25 22:41 -04:00, at `53cebac`) — 9 must-fix, 15 suggestions
**Commits**: `a03df26`, `149cb85`, `a379c56`, `40fa769`, `f6749da`, `183be95`, `8e67d45`, `e15b623`, `ce041c4`, `87adade`, `b148626`, `bd05738`, `4a43e1c`, `713b24e`

**Operator scope decisions this pass followed** (they override the review's own
recommendations): (a) the fix scope is **in-repo prose and this PR's own work
plan only** — `dispatch_subagent.sh` is not edited, the host files that as a
follow-up; (b) must-fix 6 is discharged by **adding the fence** to the three
skills, not by downgrading the claim; (c) **must-fix 9's ADR is written in this
PR**, not deferred to a follow-up; (d) **no Ask-First instruction file is
touched**.

### Actions
- [x] (must-fix 1) GitHub-write absolute at both remaining sites — `run-issue/SKILL.md:192-196` now reads "*configured* without GitHub write auth — cannot `git push` at all, and cannot open a PR either unless the optional `GH_TOKEN` was minted with write scopes"; the § closing summary (`:301-311`) bounds **both** halves explicitly (files still reachable via the bind mount; write-auth claim holds only for the read-only token configuration) — `a03df26`
- [x] (must-fix 2) `dispatch_subagent.sh:436-437` "this dispatch has no GitHub read auth" (deferred: operator scope decision — the script is not edited in this PR; the host is filing it as a follow-up issue. **PLACEHOLDER — the host must splice the follow-up issue number into this line and into the PR body before merge.** The same claim in *in-repo prose* was corrected: `review-issue/SKILL.md:87` now says the dispatch *may* have no read auth, and why)
- [x] (must-fix 3) `skill_workflows.md:167` no longer contradicts `:150-151` — prompt volume is a reason **only on the cannot-confirm branch**; under auto mode it is not a reason at all; containment is never one — `40fa769`
- [x] (must-fix 4) The cannot-confirm branch now routes `review-issue`, `plan-task` and `review-plan` explicitly (stay `in-process` — few tool calls, few prompts; a containerized `review-issue` additionally costs a host-side `--context-file` fetch). Added to **both** files so the mirror does not diverge — `a379c56`
- [x] (must-fix 5) The checkpoints claim is no longer mode-independent: `run-issue/SKILL.md:270-282` now states that container backs the push/PR gate **materially** (no transport), while in-process the only barrier is the handoff prose at `dispatch_subagent.sh:475-477` — the same documentation layer the section just demoted — `149cb85`
- [x] (must-fix 6) The **data fence now exists in the phases assigned it**: `review-issue/SKILL.md` step 1, `plan-task/SKILL.md` step 1, `triage-reviews/SKILL.md` step 5, each at the step that fetches third-party text, each stating that no dispatch mode holds it for them. Both orchestrator files now name where it lives and why (a dispatched sub-agent loads its own SKILL.md) — `f6749da`, `183be95`, `8e67d45`
- [x] (must-fix 7) `plan.md:56,83` — the "untrusted input" grounds for containers are gone from **both** Approach bullets, each with a superseding parenthetical that says what replaced them (the data fence). The Implementation Notes bullet that reached the same conclusion at `:283` is corrected too — `87adade`, `713b24e`
- [x] (must-fix 8) The four false plan claims: the `--context-file` paragraphs are no longer described as "left alone" (both the round-3 and round-6 edits are recorded, with the reason the fence came down); "No further instances found" is replaced by the three that were found and changed (`review-code:878-883`, `review-plan:453-470`, `triage-reviews:360`) plus why the original grep was too narrow; "Three files" → eight, itemized; the #606 note is re-scoped to the *issue* text, where it always belonged — `87adade`, `713b24e`
- [x] (must-fix 9) **ADR-0019 written in this PR** (operator decision) — `docs/decisions/0019-what-contains-a-dispatched-agent.md`: what a container isolates (OS/dependency state, build artifacts), what it does not (bind-mounted workspace rw, forwarded Claude credentials, `GH_TOKEN` scopes unvalidated), and that the **data fence, not the sandbox**, is the countermeasure for untrusted input. ADR-0012 cross-reference addendums added to ADR-0004 (Status note + new References section) and ADR-0015 (Status note + References entry); neither ADR's existing text is rewritten. Both orchestrator files and `review-plan` now link the ADR — `b148626`, `bd05738`, `4a43e1c`

### Suggestions taken
- [x] `triage-reviews/SKILL.md:360` — the `address-findings` hop no longer hard-codes `--mode in-process`; it points at this PR's mode choice — `f6749da`
- [x] `run-issue/SKILL.md:215-219` — PR-relative before/after narration dropped; the cross-reference to the README kept — `183be95`
- [x] `run-issue/SKILL.md:137-147` — the `review-code` bullet now carries the no-`Agent`-tool fallback (degraded container run), matching `skill_workflows.md` — `183be95`
- [x] `run-issue/SKILL.md:167-169` — the reason to run `triage-reviews` in-process is restated as the real one (body-only `--context-file` cannot carry reviews / check-runs), not "a container has no read auth" — `183be95`
- [x] `run-issue/SKILL.md:54-57` — the `--context-file` snippet's container example is now labelled the common case, with the flag's orthogonality to `--mode` stated — `183be95`
- [x] `run-issue/SKILL.md:104-105` — the sentinel arrives "in the session preamble or attached to an early tool result" (it arrived in the preamble for this dispatch) — `183be95`
- [x] `review-plan/SKILL.md:371-395` — the one **mechanical** independence marker is named: `dispatch_subagent.sh:468`'s "You are a fresh-context sub-agent dispatched for issue #N" handoff header, a positive test (its absence still proves nothing) — `e15b623`
- [x] `review-plan/SKILL.md:391-396` — the "what left you unsure" line now has a destination (first line under the `**Plan**:`/`**PR**:` block), with an example — `e15b623`
- [x] `skill_workflows.md:180` — "Conversely" replaced; it introduced a restatement, not a contrast — `8e67d45`
- [x] `skill_workflows.md:160-162` — the container-auth fallback is re-attached to the cannot-confirm branch as a whole, not to `review-code` — `8e67d45`
- [x] `.devcontainer/agent/README.md:146-150,426-427` — the auth guard is bounded to non-`--shell` launches (`[ "$SHELL_MODE" = false ]`, verified at `docker_run_agent.sh:317-319`) — `ce041c4`
- [x] Plan completeness — Approach now has steps 4-6 (README, `review-code`, and the three fence files); the `run-issue` row records the `:45-49` / `:58-67` / `:411` edits; the `skill_workflows` row the `# In-process (…)` comment; the README row § Read-only GitHub Access; Documentation Impact lists every review-added file plus the ADR; ADR Compliance revises 0001 (ADR now required and written) and 0015 (triggered), and adds 0012 — `87adade`

### Suggestions deferred
- [x] Print-vs-describe the `While auto mode is active:` sentinel (deferred: the verbatim string is what makes the real reminder recognizable, and both files already tell the reader a quotation of these docs does not count — describing it risks a false *negative*, which lands on the wrong branch)
- [x] `AGENTS.md:560` "no-GitHub-auth container phase" phrasing (deferred: **Ask-First** instruction file — flagged to the operator, not edited)
- [x] The three framework adapters' missing non-Claude `--mode container` branch (deferred: `.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`, `.agent/AGENT_ONBOARDING.md` are all **Ask-First** — flagged to the operator, not edited)

### Recurrence check (rounds 4/5/6 found the same two claims each time)
Verified by **reading**, not grep alone: the whole "Choosing a mode" section, the
whole "What contains a dispatched agent" section, both `skill_workflows.md`
bullets, and the plan's Approach + Implementation Notes were re-read end to end
as a reader would. Grep then confirmed: every surviving `untrusted` /
`no GitHub … auth` string in `.claude/`, `.agent/knowledge/`, `.devcontainer/`,
`docs/` and `README.md` is either correct-direction (the fence, or an explicitly
qualified "configured, not enforced") or about an unrelated subject (field mode,
`--allow-untrusted-copilot`, `triage-reviews`' no-GitHub-review-actions rule).
One extra tightening beyond the findings: "**not sufficient** containment for
untrusted input" → "**not** containment for untrusted input **at all**" in
`run-issue` and `review-plan`, since "on its own" implies partial containment.

### Checks
`pre-commit run --files <every changed file>` — all hooks pass or skip (no
Python/YAML/shell in the diff); nothing suppressed. No tests apply (prose + one
ADR, no enforced logic). All 14 commits carry `Claude Code Agent
<roland+claude-code@ccom.unh.edu>`. Not pushed — the host publishes.

### Next step
Re-review (`review-code`) on the round-6 fixes. Two things the re-review should
know: **must-fix 2 is discharged by a filed follow-up, not by an edit** — the
placeholder above is unfilled and needs the host's issue number; and **ADR-0019
is new text in this PR**, so it is in scope for review rather than settled
background.
