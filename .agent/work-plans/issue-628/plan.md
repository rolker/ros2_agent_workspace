# Plan: Planning-document vocabulary + two-root rule (umbrella #628) — sub-issue (1): the ADR + the workspace roadmap

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/628

## Context

Surveyed 2026-09-14 after the first hand-run of `/janitor-sweep` (#569): each
scope in this workspace carries a different subset of the same planning
structure (vision / roadmap / decisions / health), and only one of them —
the BizzyBoat roadmap
([`unh_echoboats_project11/docs/roadmap.md`](https://github.com/rolker/unh_echoboats_project11/blob/main/docs/roadmap.md))
— has a working loop, because deployments force a periodic read of it. The
workspace has a README § Vision, 19 ADRs, and no roadmap; the framework repo
has a `VISION.md` untouched since January and no roadmap. Folded in: the
publish + trigger decision deferred from #569, because where a sweep's
findings live is a question about the structure they sit beside.

Per the operator's decisions comment (2026-09-14), **#628 is an umbrella**
and the deliverables are sequenced into six sub-issues (`Part of #628`) — four
independently mergeable in this repo, plus two filed in the project repos the
issue routes them to. **This worktree delivers sub-issue (1) only.**

Factual correction for the record: the issue body describes #249 as "tracked
on a draft PR, idle since Feb 26". [PR #257](https://github.com/rolker/ros2_agent_workspace/pull/257)
in fact **merged** 2026-02-26 (created 2026-02-25); what has been idle since
is [#249](https://github.com/rolker/ros2_agent_workspace/issues/249) itself,
which has had no follow-on work. The supersede-and-close conclusion is
unchanged.

## Sub-issue sequence (filed as part of this work)

| # | Sub-issue | Scope (one paragraph) |
|---|---|---|
| **(1)** | **ADR + workspace `docs/roadmap.md`** | **This worktree.** See Approach below. |
| (2) | Discovery schema + the skills reading it | Define the workspace-owned schema for the project's self-declaration file (the `.agents/deployment.yaml` precedent): roadmap path and health path, nothing more. Add the resolver (a small script beside `field_mode.sh` / `resolve_repo_checkout.sh`, with its own `AGENTS.md` Script Reference row) and make `janitor-sweep` / `audit-project` read it instead of guessing paths. **Graceful absence is the headline requirement and gets an explicit test case**: a repo with no discovery file behaves exactly as it does today, never errors — ADR-0017's incremental-rollout stance. Ships the kind-marker convention and the `.agent/templates/` kind marker. |
| (3) | Sweep split by scope + commit-via-PR publish + run-over-run diff + finding tiers | Split the sweep report by scope (check 1 = workspace, check 2 = per project) and commit each part as the health document beside that root's roadmap, via PR, replaced each run — git history is the run-over-run diff, rendered as explicit "new / resolved / unchanged" sections. Add the finding tiers (work that can be lost → unowned safety bugs → rules that have bitten with no enforcement → contradictions in the record → drift). Carries the consequence the issue review flagged: the `janitor-sweep` exception clause in `.agent/knowledge/principles_review_guide.md`'s Consequences Map ("a local report file under `.agent/scratchpad/janitor/`") goes stale here. Also adds `janitor-sweep` to `worktree_create.sh`'s `ALLOWED_SKILLS` so the skill-worktree branch convention applies. |
| (4) | The trigger | Wire the mechanism the ADR decides, under the dedicated bot identity, at the chosen cadence; provision its credentials (a GH Actions cron needs its own token). Record the outcome on [#569](https://github.com/rolker/ros2_agent_workspace/issues/569) and close it — #569 stays open until the trigger lands. |
| (5) | **`unh_marine_autonomy`: `docs/roadmap.md` + kind markers** — *filed in [rolker/unh_marine_autonomy](https://github.com/rolker/unh_marine_autonomy), not here* | A `docs/roadmap.md` under `VISION.md`, instantiated from `.agent/templates/roadmap.md`, naming the BizzyBoat roadmap and `docs/sonar_ecosystem.md` as its children (the two-root rule read top-down); a one-line `kind:` marker on the eight existing flat pages in `docs/` (`autonomy_modes.md`, `data_flows.md`, `interfaces.md`, `launch_manager.md`, `sonar_ecosystem.md`, `sonar_processing_chain.md`, `sonar_reference.md`, `survey_index_schema.md`), which today are distinguishable only by their opening paragraph. Verified 2026-09-14: that repo has no `docs/roadmap.md`. |
| (6) | **`unh_echoboats_project11`: roadmap parent line + health placement** — *filed in [rolker/unh_echoboats_project11](https://github.com/rolker/unh_echoboats_project11), not here* | `docs/roadmap.md` (703 lines, verified 2026-09-14, no parent line today) gains the "parent" line the two-root rule requires, pointing at the `unh_marine_autonomy` roadmap from (5); the health document lands beside it once the sweep can write one, so this issue is gated on (3). |

Sub-issues **(5)** and **(6)** are the issue's "Deliverables (project repos —
separate issues there, linked from here)". They are filed in those repos (both
are GitHub-origin: `git@github.com:rolker/…`, verified 2026-09-14) and linked
from #628, so closing the umbrella does not lose them — see step 7 and step 8.

## Approach (sub-issue (1))

1. **Write `docs/decisions/0020-planning-document-kinds-and-the-two-root-rule.md`** (Status: Accepted) covering, in one document:
   - **The vocabulary of kinds** — *vision* (why; rarely edited), *roadmap* (what next / what is deferred; carries its own loop), *decision* (ADRs, with a status line), *health* (generated by the sweep, replaced each run), plus supporting kinds used as needed (reference, contract, design draft, architecture). A vocabulary, **not a directory tree**.
   - **The two-root rule** — the workspace root and the project root each carry the first four kinds, and that is the *only* structure workspace skills may assume. Below the project root nothing is presumed: **a roadmap names the roadmaps beneath it**. This is the operator's standing "sub-project tier stays LOOSE" constraint, stated as a rule (ADR-0003: workspace infrastructure stays project-agnostic; ADR-0017: project repos carry thin, referencing files, never forks).
   - **Discovery, in principle** — the project declares its own layout in a file whose schema the workspace defines, the way `.agents/deployment.yaml` already declares deployment config (ADR-0014). The ADR states the principle and the graceful-absence requirement; **the schema itself and the skills reading it are sub-issue (2)**.
   - **Publish means commit, not post** — the sweep's durable output is a health document committed beside the roadmap of the repo it grades, replaced each run via PR. Cites **ADR-0015** (the container produces, the host publishes) and **ADR-0019** (what the container actually isolates — the "no GitHub write auth" line is a property of the launcher's token configuration, not something the sandbox enforces, so the publish path's identity has to be stated rather than assumed).
   - **Gate, stated in the ADR's Consequences: the commit-publish path does not go live until the redaction class on [#626](https://github.com/rolker/ros2_agent_workspace/issues/626) is closed.** #626 carries open, unfixed items in exactly the class that would be published: absolute host paths and credentials reaching stderr that the sweep report transcribes — the resolver's two `redact.sh` refusals (`.agent/scripts/resolve_repo_checkout.sh:131,141`), a failed `exec 9>`/`exec 8>` lock open that bash reports before the `say()` funnel runs (`resolve_repo_checkout.sh:436`, `manifest_fallback.sh:181`), and the `mkdir -p "$REPORT_DIR"` arm (`.claude/skills/janitor-sweep/SKILL.md:108`) — plus two `redact.sh` defects that leak on hostile input (a password containing `@`; a workspace path containing `=`). Deciding "publish means commit" must not read as authorising an unredacted host path into a *public* repo. The ADR decides the destination; #626 is the gate on switching it on, and (3)/(4) must not ship the path before it closes.
   - **Commit identity for the unattended publish path** — a dedicated bot identity (`Janitor Sweep Agent`) committing on a `skill/janitor-*` branch, the convention the `research` skill already uses, so both `check-commit-identity.py` and `check_pr_authors.py` pass and the PR is unmistakably automated. This answers the question #569's 2026-09-11 comment deferred out of that slice.
   - **An unattended PR is still reviewed by a human before merge** — "green CI is not review" (AGENTS.md § Merging) applies unchanged; stated explicitly because this is the workspace's first fully unattended PR-opening path.
   - **The trigger** — the ADR weighs the three candidates (table below) and **records the operator's choice made at plan review**; ADR-0001's bar is to capture the decision, not to leave it open.
   - **The one rule stated in every roadmap** — before choosing the next piece of work, read the roadmap and the health document together: the roadmap says where you want to go, health says what will stop you; work appearing in both goes first.

   **Recorded option, not taken (operator, 2026-09-14): splitting ADR-0020 in two** — a taxonomy ADR (kinds, two-root rule, discovery) and a mechanism ADR (publish=commit, bot identity, trigger, human-review gate), so that superseding the trigger later does not churn the taxonomy. The operator declined the split for now; it stays one ADR. Noted here so a future supersede knows the seam was seen and where it runs: the mechanism half is the "Publish means commit" / "Commit identity" / "The trigger" bullets above.
2. **Land the ADR-0012 cross-reference addendum on `docs/decisions/0015-dispatch-handoff-context-contract.md` in this PR** — decided here, not left conditional. ADR-0015's Decision is a two-actor dichotomy: the container produces, **the host publishes**, and "nothing publishes from inside the sandbox". An unattended, scheduled trigger has **no host session** — it *is* the publisher — so it is not a gap ADR-0020 can merely cite 0015 for; it is a case 0015 does not contemplate. What the addendum says (and nothing more, so it stays inside ADR-0012's permitted class — a Status-line note plus a References entry, no rewording of the Decision, no new Consequences):

   > Extended by [ADR-0020](0020-planning-document-kinds-and-the-two-root-rule.md): a scheduled, unattended trigger is a **third actor** this ADR's host/container dichotomy does not contemplate — it has no host session and is itself the publisher. ADR-0020 states that actor's commit identity, credentials and human-review-before-merge gate; the host/container reading here is unchanged for dispatched phases.

   ADR-0020 in turn names the unattended publisher explicitly as a third actor rather than presenting it as an instance of "the host". (ADR-0012 permits exactly this: Status-line cross-reference and a References section on an accepted ADR; substantive change would still require superseding.)

3. **Add `.agent/templates/roadmap.md`** — the roadmap kind's template, whose closing **loop section is copied from the BizzyBoat roadmap's "How this roadmap stays useful"** (when it is read, when leftovers land, when deferred items are pruned), generalised from "deployment start / wrap-up" to "the scope's forcing function". Also carries the "What's not on this roadmap" section and the parent/children lines the two-root rule needs.
4. **Write `docs/roadmap.md`** — the workspace roadmap, instantiated from that template, superseding #249. Content is drawn from what is already on record, not invented: the README § Vision as its end goal, the open direction-setting threads (#569 sweep cadence, #626, #627, #610, #609 follow-ups), the ADR backlog (3 ADRs still `Proposed` while cited as binding), and the deferred items #249 never closed out. **First-cut size bound: 150 lines.** The BizzyBoat roadmap it is modelled on is 703 lines (verified 2026-09-14) — but that is five months of accreted campaign history, not a first cut. A first roadmap that is a survey rather than a short list of next moves is hard to keep current, which is the failure mode this issue diagnoses; anything that does not fit goes to "What's not on this roadmap" or stays in its issue.

   Its loop section names **the periodic sweep** as the workspace's forcing function — the counterpart to BizzyBoat's deployments — and says so **in the future tense, with the gap named in the document itself**: the sweep exists and has been hand-run once (#569), but nothing schedules it until sub-issue (4). The loop section therefore states both, explicitly: *until (4) lands, the roadmap's forcing function is the plan-review / hand-run sweep that exists today; once (4) lands, it is the scheduled sweep.* Declaring the loop ahead of its trigger is deliberate — the trigger in (4) is being wired **to** this declared cadence, so writing the roadmap without it would mean rewriting the section three PRs later — but a loop section that quietly described a cadence nobody runs would be the exact failure this issue diagnoses in the framework's `VISION.md`, so the pending state is written down and linked to (4) rather than implied.
5. **Point README § Vision at the roadmap** — one line, so the vision document names the roadmap beneath it (the two-root rule read top-down).
6. **Add the Consequences Map row** in `.agent/knowledge/principles_review_guide.md`: changing a roadmap or health document → the kinds table in the ADR, the roadmap template, and the parent roadmap that names it. (The `janitor-sweep` exception-clause rewrite is **(3)**'s, not this PR's — the sweep's output does not change here.)
7. **Add the `AGENTS.md` rows, and the matching `CLAUDE.md` References row** — in `AGENTS.md`, a "Planning documents" pointer plus `docs/roadmap.md` in the References list; in `CLAUDE.md`, `docs/roadmap.md` added to its own References list (`CLAUDE.md:23-34`), which parallels `AGENTS.md`'s and lists `README.md` § Vision, `ARCHITECTURE.md` and `docs/decisions/` today — it would otherwise be the one References list in the repo that does not name the roadmap. This is the Consequences Map's own "`AGENTS.md` → framework adapters if affected". **Both files are Ask First**; the operator's approval of this plan at review is the approval for exactly these three additions (two in `AGENTS.md`, one in `CLAUDE.md`) and nothing more.
8. **File sub-issues (2)-(6)** with `Part of #628` in each body, using `.agent/scripts/gh_create_issue.sh`, each carrying the paragraph from the table above. **(2), (3), (4) are filed in this repo; (5) and (6) are filed in `rolker/unh_marine_autonomy` and `rolker/unh_echoboats_project11` respectively** (`gh issue create --repo <owner/repo>`; both are GitHub-origin, so this is a normal cross-repo file) and referenced from #628 with the full `owner/repo#N` syntax, in the issue body, so GitHub side-bar-links them. Cross-repo bodies say `Part of rolker/ros2_agent_workspace#628`. Post the five links as a comment on #628 so the umbrella's own record names them.
9. **PR body**: `Part of #628` and `Closes #249` — the workspace roadmap is the direction #249 asked for. Per AGENTS.md § Issue-closing keywords, #249 is the *only* issue this PR closes; every other reference is "Part of" / "addresses".

   **#628 does not close here, and does not close with (4) alone.** The umbrella closes when (2), (3) and (4) have merged **and** (5) and (6) have been filed and linked — their completion is then tracked in their own repos, but they must exist before the umbrella is closed, or the issue's project-repo deliverables are lost at close.

   **What else #249 asks for, checked 2026-09-14 before using the keyword.** Its body asks for four workspace properties (simple to use — "a small number of clear entry points, not 42 scripts"; self-documenting; durable — decisions captured where they cannot be silently reverted; sustainable — improvement as a continuing practice, not periodic rework). Its one operator comment adds three items the body does not: (i) "the README containing everything concisely" is *under consideration*, not decided; (ii) per-framework instruction duplication was **deliberate** (frameworks may prefer different layouts) and is named as an improvement at risk of accidental revert; (iii) "maybe we need a central architecture document ... consulted when an issue is considered, when a plan is made, and updated if needed when an issue is implemented" — for the workspace **and** the project. Disposition, each stated in `docs/roadmap.md` so the close is honest:
   - The four body properties → roadmap entries (the entry-point count and the durability/sustainability threads are exactly what the roadmap + sweep loop address).
   - (iii) is **largely already answered**: `ARCHITECTURE.md` exists at the workspace root; what #249 asks for beyond it is the *consult-and-update cadence*, which is what the Consequences Map and this ADR's roadmap+health rule provide. Recorded in the roadmap rather than dropped.
   - (i) and (ii) are **explicitly carried forward, not closed by this PR** — listed in the roadmap's "What's not on this roadmap" / deferred section with (ii) flagged as a decision to protect, not revisit. If the operator would rather #249 stay open to hold (i), drop the keyword to "Part of #249" — the Open Question below.

## Trigger comparison — for the ADR, operator decides at plan review

| Mechanism | Reach | Cadence reliability | Commit identity | Credential surface | Cost |
|---|---|---|---|---|---|
| **GH Actions weekly cron** (workspace repo) | GitHub repos only, on a fresh clone — works because the sweep already resolves repos via `resolve_repo_checkout.sh` + the manifest fallback. **Cannot reach gitcloud repos or `layers/`.** | Exact and unconditional — independent of whether the laptop is on | Bot identity set per-commit with `git -c user.name/user.email` (AGENTS.md § Agent Commit Identity). Same chore as the other two rows | **Largest.** Two standing secrets on a **public** repo: the Claude subscription token (`CLAUDE_CODE_OAUTH_TOKEN`), and a **cross-repo write credential** (PAT or GitHub App token) — health documents are committed to the repos they grade, and the built-in `GITHUB_TOKEN` is scoped to the repo running the workflow | Actions minutes free on a public repo; the agent run itself draws the same subscription usage as the other two |
| **Claude Code cloud Routine** | Same GitHub-only reach — **cannot reach gitcloud or `layers/`** (recorded on #569; independently confirmed in the #624 research digest, which notes Routines are cloud-hosted and GitHub-repo-scoped) | Scheduled; daily run caps by plan (Pro 5 / Max 15 / Team 25 per the digest — the tier floor is reported inconsistently across secondary sources, so verify against current docs before designing around it) | Same per-commit `-c` chore, and it satisfies the same checks: `check_pr_authors.py` inspects each commit's **primary author email**, not the account that opened the PR | **Smallest.** Uses the operator's existing Claude auth and its existing GitHub connection; no new secret stored in any repo | Draws subscription usage; no separate infra charge |
| **anacron on the dev laptop** | **Full reach** — the only option that sees gitcloud repos *and* a real `layers/` tree, which is what project-repo health documents for the field/mirror repos need | Runs missed jobs at next boot, so laptop-off is tolerated — but the interval is boot-dependent, not calendar-exact | Same per-commit `-c` chore | Mints nothing new — the host's existing `gh` auth and the OAuth token already at `~/.config/ros2-agent/claude-oauth-token` | No infra charge; draws the same subscription usage |

**What is *not* a discriminator.** All three run the same Claude Code agent over
the same sweep, so all three draw subscription usage and all three need
`CLAUDE_CODE_OAUTH_TOKEN` (or an API key) available to the run — the Actions row
is not "free" in the sense that matters. And **commit identity is the same
per-commit `-c` chore in all three rows**: the dedicated bot identity is set on
the commit, and `check_pr_authors.py` reads commit authors, so no mechanism is
disqualified on identity grounds. A *separate* question, which applies only to
the Routine, is the **GitHub account that opens the PR** — the operator's
connected user rather than a bot account. That is a legibility point ("is this
PR unmistakably automated?"), not a compliance one; it fails no check in this
repo. A previous revision of this table scored those two as one near-
disqualifying objection against the Routine; corrected here.

**The real discriminators** are therefore three: **reach** (only anacron sees
gitcloud and `layers/`), **cadence exactness** (only Actions is calendar-exact
and laptop-independent), and **credential surface** (Actions adds a standing
cross-repo write credential to a public repo; the Routine and anacron add
nothing new).

Weekly is the cadence the diff argument points to (the issue's own "weekly is a
diff, quarterly is another baseline"). A split — Actions weekly for the
workspace scope, anacron for the project scopes that need `layers/` and gitcloud
— is available if the operator wants full reach without depending on the laptop,
at the cost of two mechanisms to maintain. **This plan deliberately does not
pre-decide.** The operator chooses at plan review from the table above, and the
choice becomes ADR-0020's Decision (ADR-0001's bar: capture the decision, do not
leave it open).

## Files to Change

| File | Change |
|------|--------|
| `docs/decisions/0020-planning-document-kinds-and-the-two-root-rule.md` | New ADR (kinds, two-root rule, discovery in principle, publish=commit, bot identity, human-review gate, trigger decision, roadmap+health rule) |
| `.agent/templates/roadmap.md` | New — roadmap kind template; loop section copied from the BizzyBoat roadmap's closing section |
| `docs/roadmap.md` | New — the workspace roadmap, superseding #249 |
| `README.md` | One-line pointer from § Vision to `docs/roadmap.md` |
| `.agent/knowledge/principles_review_guide.md` | New Consequences Map row for roadmap/health documents |
| `docs/decisions/0015-dispatch-handoff-context-contract.md` | ADR-0012 cross-reference addendum: Status-line note + References entry naming ADR-0020's unattended publisher as a third actor (no change to the Decision) |
| `AGENTS.md` | Planning-documents pointer + `docs/roadmap.md` in References (**Ask First** — approved by this plan's review) |
| `CLAUDE.md` | `docs/roadmap.md` added to its own References list (`CLAUDE.md:23-34`), which parallels `AGENTS.md`'s (**Ask First** — approved by this plan's review) |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Capture decisions, not just implementations | The ADR is the deliverable; it settles trigger and commit identity rather than deferring them again (ADR-0001's bar). |
| Only what's needed / Improve incrementally | The umbrella is split into four independently mergeable sub-issues; (1) ships two documents and a template, not a structure. This is the direct answer to the #249/#257 stall the issue itself cites. |
| Workspace vs. project separation | The two-root rule *is* this principle expressed as structure: the workspace defines a schema and assumes two roots; everything below the project root is discovered (ADR-0003). |
| A change includes its consequences | Consequences Map row and AGENTS.md rows land in this PR; the `janitor-sweep` exception-clause rewrite lands in (3), where the sweep's output actually changes — recorded here so it cannot be lost. |
| Human control and transparency | The ADR states the human-review-before-merge gate for the first unattended PR-opening path, and the trigger is the operator's choice, not the plan's. |
| Enforcement over documentation | Honest limit: this sub-issue is documentation. The enforcement is the sweep itself — the forcing function that makes the roadmap get read — and it lands in (3)/(4). Stated in the ADR's Consequences as a known gap until then. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0001 (ADRs) | Yes | This is the ADR; it decides rather than surveys. |
| ADR-0003 (project-agnostic workspace) | Yes | Two-root rule + discovery keep the workspace from presuming any project's layout. |
| ADR-0006 / ADR-0017 (AGENTS.md two-tier) | Yes | AGENTS.md rows are pointers; project-repo planning documents are referenced, never forked; (2)'s graceful absence mirrors ADR-0017's incremental rollout. |
| ADR-0004 (enforcement hierarchy) / ADR-0005 (layered enforcement) | Yes | **Triggered and deliberately deferred, stated rather than silent.** ADR-0020 states two new compliance rules — "a roadmap names the roadmaps beneath it" and "read the roadmap and the health document together before choosing work" — with **no hook, CI check or guardrail in this sub-issue**; by ADR-0005's own test ("if a rule isn't in CI, it's a suggestion, not a rule") they are suggestions until then. The enforcement is the sweep: the two-root rule becomes checkable when the discovery resolver lands in **(2)**, and the read-both rule gets its forcing function when the trigger lands in **(4)**. ADR-0020's Consequences records this as a named gap with those two sub-issues as the closers, so the deferral is on the record and not discovered later. (ADR-0005 is itself still `Proposed` while cited as binding — one of the three the workspace roadmap lists.) |
| ADR-0012 (cross-reference addendums) | Yes | **Decided here, not conditional** (step 2): the addendum lands on ADR-0015 in this PR — a Status-line cross-reference plus a References entry, which is exactly ADR-0012's permitted class; the Decision text is untouched, so no supersede is required. |
| ADR-0013 (progress.md vocabulary) | Indirect | The health document changes the janitor exception's premise — flagged, and handled in (3). |
| ADR-0014 (deployment mode) | Yes | The BizzyBoat cadence being generalised; `.agents/deployment.yaml` is the precedent the discovery file follows. |
| ADR-0015 / ADR-0019 (dispatch handoff / containment) | Yes | Cited for publish=commit (container produces, host publishes) and for why the unattended path's identity and auth must be stated, not assumed. |
| ADR-0018 (local-first CI) | No | Workspace repo; hosted checks stay required. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Add an ADR in `docs/decisions/` | The ADR table in `.agent/knowledge/principles_review_guide.md` | Yes |
| Add a template in `.agent/templates/` | Docs/skills that reference it | Yes — the ADR and `docs/roadmap.md` are its only referents today |
| `AGENTS.md` | Framework adapters if affected | Yes — `CLAUDE.md` gains the same `docs/roadmap.md` References row in this PR (step 7). An earlier revision dismissed this as "adapters carry no roadmap rows to drift": true but beside the point — the adapter's own References list is precisely what should gain the row |
| The sweep's durable output moves out of the scratchpad | The `janitor-sweep` exception clause in the Consequences Map | No — belongs to (3); recorded in (3)'s scope so it cannot be dropped |
| A new shared script for discovery resolution | `AGENTS.md` Script Reference table | No — belongs to (2); named in (2)'s scope |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): `README.md` § Vision (gains the roadmap pointer); `.agent/knowledge/principles_review_guide.md` (Consequences Map row + ADR table row for ADR-0020); `AGENTS.md` (References); `CLAUDE.md` (References — the adapter's parallel list). Nothing else in the repo asserts that the workspace has no roadmap.
- **Agent-instruction candidates** (proposals only — the operator decides): the "read the roadmap and the health document together before choosing work" rule is an instruction-level behavior, proposed for `AGENTS.md` once a health document actually exists — i.e. with (3), not now. Proposing it early would point agents at a file that is not there.

## Open Questions

- **Trigger mechanism** — GH Actions weekly cron / Claude Code cloud Routine / anacron on the laptop (or the Actions+anacron split). The comparison table above — **corrected in this revision** so that cost, commit identity and credential surface are scored on the same basis in all three rows — is the ADR's input; the operator's answer becomes the ADR's Decision. The plan does not pre-decide.
- **Is `.agent/templates/roadmap.md` in (1) or a later sub-issue?** The plan puts it in (1) because `docs/roadmap.md` is its first instance and the loop section would otherwise be written twice. Reversible at review.
- **`AGENTS.md` *and* `CLAUDE.md` are Ask First** — approving this plan approves exactly the three References-level additions in step 7 (two in `AGENTS.md`, one in `CLAUDE.md`) and nothing more.
- **Does this PR close #249?** The plan says yes, on the checked basis in step 9: the roadmap carries #249's four body properties plus the disposition of the three items from its operator comment, two of which are carried forward rather than answered. If the operator would rather #249 stay open to hold the "README contains everything concisely" question, the keyword drops to `Part of #249` and nothing else in the plan changes.
- **Splitting ADR-0020 into a taxonomy ADR and a mechanism ADR** was offered at plan review and **declined by the operator (2026-09-14)**. Recorded as an option in Approach step 1, not taken; re-openable if the trigger decision later proves volatile.

## Estimated Scope

Umbrella #628: four PRs in this repo, plus two issues filed in project repos ((5), (6)). **This worktree: one PR** — three new documents, five small edits (`README.md`, `.agent/knowledge/principles_review_guide.md`, `AGENTS.md`, `CLAUDE.md`, the ADR-0015 addendum), plus filing sub-issues (2)–(6).
