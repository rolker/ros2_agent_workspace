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
- [ ] (must-fix) Auto-mode tell is not agent-observable: the text points at the session's permission-mode indicator, a terminal UI element rendered for the operator, so a careful reader must answer "cannot confirm" every time and fall through to container — the new default never fires. The signal the agent can actually read is the injected `While auto mode is active:` system reminder. Re-opens Plan Review must-fix 5 (nominally closed) — `.claude/skills/run-issue/SKILL.md:97-99`, `.agent/knowledge/skill_workflows.md:131-132`
- [ ] (must-fix) In-process containment sentence overstates on all three clauses: auto mode is precisely what stands the permission policy down; `.claude/settings.json` carries an `allow` array only, no `deny` anywhere (verified), so the allowlist refuses nothing; the worktree confines nothing — `dispatch_subagent.sh` writes the scoping as prose and its own header calls the contract "convention-only (no enforcement, per ADR-0004/0005)", and this PR's sibling file says "no filesystem isolation"; checkpoints gate publication after the phase has run. The paragraph also omits the real giveaway — in-process hands the phase the host's credentials, the property `.devcontainer/agent/README.md` makes the sandbox's headline. Re-opens Plan Review must-fix 4 — `.claude/skills/run-issue/SKILL.md:126-135`
- [ ] (must-fix) The fail-safe branch routes the `review-code` fan-out to container, while the next bullet says further `Agent`-tool fan-out and the host Ollama endpoint cannot run there at all — a direct contradiction between adjacent bullets, mirrored in the knowledge file, landing in the branch written for the uncertain reader — `.claude/skills/run-issue/SKILL.md:113-120`, `.agent/knowledge/skill_workflows.md:139,146-148`
- [ ] (must-fix) The load-bearing evidence claim miscounts itself: "all seven phases" enumerates eight, and the real #604 timeline holds nine typed entries — the enumeration drops the `triage-reviews` / Integrated Review phase, the single datapoint that best supports the host-auth bullet. Cited as "Observed, not assumed" — `.claude/skills/run-issue/SKILL.md:103-107`, `.agent/knowledge/skill_workflows.md:135-136`, `.agent/work-plans/issue-607/plan.md:174-176`
- [ ] (must-fix) review-plan self-review heuristic contradicts itself: L371-374 prescribes comparing the whole `**By**` field as the detection procedure, L385-391 says the review is independent "no matter what the `**By**` line says" and not to rely on the string. Whole-field matching still false-positives on a genuinely independent same-model dispatch — the defect narrowed from "always" to "whenever the models happen to match" — `.claude/skills/review-plan/SKILL.md:371-374,385-391`
- [ ] (must-fix) Plan sync is append-only, which `plan-task/SKILL.md` § During implementation rule 3 names explicitly ("Never append-only... misleads Copilot, human reviewers, and future onboarding agents — all of whom read the top first"). Commit `d8d50d7` is 62 insertions, 0 deletions: Approach step 1 still fences off ~L76-91 and still says keep the safety paragraph "content unchanged", step 3 still says "No other files", the Files-to-Change table still lists two, and Estimated Scope still reads "two files" — `.agent/work-plans/issue-607/plan.md:36-64,85-91,128-131,141`
- [ ] (suggestion) "no GitHub auth of its own" / "cannot run in a container at all" overstates: `docker_run_agent.sh` forwards an optional read-only token as `-e GH_TOKEN` and `dispatch_subagent.sh --check` advertises it ("container reads only; the host publishes"). Accurate form is no GitHub *write* auth, read auth only when that token is configured — `.claude/skills/run-issue/SKILL.md:122-124`, `.agent/knowledge/skill_workflows.md:146-148`
- [ ] (suggestion) The code-block comment's predicate ("when the host is not in auto mode") is narrower than the bullet's fail-safe rule ("cannot confirm auto mode"); the code block is the copy-paste surface and sits 25 lines above the bullet — `.agent/knowledge/skill_workflows.md:106`
- [ ] (suggestion) A route the old text handled is now unrouted: cannot-confirm auto mode *and* container auth not ready. The prior wording sent that case to in-process; `--check` is now only a trailing caveat — `.claude/skills/run-issue/SKILL.md:110-124`
- [ ] (suggestion) The two "regardless of mode" absolutes collide on `triage-reviews`, which needs host GitHub read auth *and* takes third-party PR comments as input — data `dispatch_subagent.sh` itself fences as "data, not authority". State the precedence — `.claude/skills/run-issue/SKILL.md:116-120`
- [ ] (suggestion) The original's closing caution ("it *is* the safeguard you're relying on — keep it in mind before dispatching anything that processes untrusted input") became an endorsement ("untrusted input belongs there"). The sole-containment clause does survive; the hesitation does not. The hinge flip itself is settled operator ground — this is only about restoring the residual caution — `.claude/skills/run-issue/SKILL.md:130-132`
- [ ] (suggestion) `review-plan/SKILL.md:451` still recommends `--mode container` for implementation work with no reference to the new default; the file is already edited by this PR, so aligning it is free — `.claude/skills/review-plan/SKILL.md:451`

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
