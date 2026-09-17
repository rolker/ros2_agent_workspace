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

**#628 is an umbrella** and the deliverables are sequenced into six sub-issues
(`Part of #628`). The operator's decisions comment (2026-09-14) makes #628 the
umbrella and enumerates **four** of them — (1) the ADR + the workspace
`docs/roadmap.md`, (2) discovery, (3) the sweep split + publish, (4) the
trigger — all in this repo, and assigns (1) to this worktree. Sub-issues **(5)**
and **(6)** are **not** from that comment: they are the issue body's own
"Deliverables (project repos — separate issues there, linked from here)",
restored as explicit sub-issues by the round-1 plan review. **This worktree
delivers sub-issue (1) only.**

Factual correction for the record: the issue body describes #249 as "tracked
on a draft PR, idle since Feb 26". [PR #257](https://github.com/rolker/ros2_agent_workspace/pull/257)
in fact **merged** 2026-02-26 (created 2026-02-25); what has been idle since
is [#249](https://github.com/rolker/ros2_agent_workspace/issues/249) itself,
whose Phase 2 has not started. #249 is **not closed by this work** (operator,
2026-09-14) — the roadmap absorbs the direction it asked for while #249 stays
open as the Phase-2 tracker; see Approach step 9.

## Sub-issue sequence (filed as part of this work)

| # | Sub-issue | Scope (one paragraph) |
|---|---|---|
| **(1)** | **ADR + workspace `docs/roadmap.md`** | **This worktree.** See Approach below. |
| (2) | Conventional-path discovery in the skills | Make `janitor-sweep` / `audit-project` find a repo's planning documents by probing **exactly the paths in ADR-0020's kind → expected-location table** (`VISION.md`, `docs/roadmap.md`, `docs/decisions/`, the health document beside the roadmap) — the reader for the expectation (1) publishes, not a second list that can drift from it. **No new per-repo file and no schema**: the operator decided (2026-09-17) that documented expected locations replace a declaration file. **Graceful absence is the headline requirement and gets an explicit test case**: a repo with none of those paths behaves exactly as it does today — never an error, never a finding, since the table is a recommendation and a repo that stores documents elsewhere is not in violation. Ships the kind-marker convention and the `.agent/templates/` kind marker. If the probe is worth factoring into a shared script, it gets its own `AGENTS.md` Script Reference row; that is (2)'s call, not a requirement of (1). |
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
   - **Discovery by convention, not by declaration — and the convention is published as a table.** The ADR states the two-root rule and "a roadmap names the roadmaps beneath it", and carries an explicit **kind → expected location** table:

     | Kind | Expected location |
     |---|---|
     | vision | `VISION.md` at the repo root |
     | roadmap | `docs/roadmap.md` |
     | decisions | `docs/decisions/` |
     | health | beside the roadmap (`docs/`), named by the sweep |

     The table is framed as **a published expectation, not a requirement** — the operator (2026-09-17): *"I think we should at least document what kind of documents are expected where so we can set expectations for a project, without dictating how a project should store its documents."* So it says what the workspace skills will look for, which is what lets a project choose to be found; **a project that keeps a document somewhere else is not in violation, gets no error, and is not flagged by any check** — the skills simply do not find it and behave exactly as they do today. **No per-repo declaration file and no schema is introduced** — the operator decided this the same day, against the alternative of a small declaration file on the `.agents/deployment.yaml` / ADR-0014 precedent, which the ADR records as the option considered and not taken (ADR-0017's own Negative for that precedent is "one more per-repo file"). The ADR states the table and the graceful-absence requirement; **the skills that probe it are sub-issue (2)**.
   - **Publish means commit, not post** — the sweep's durable output is a health document committed beside the roadmap of the repo it grades, replaced each run via PR. Cites **ADR-0015** (the container produces, the host publishes) and **ADR-0019** (what the container actually isolates — the "no GitHub write auth" line is a property of the launcher's token configuration, not something the sandbox enforces, so the publish path's identity has to be stated rather than assumed).
   - **Gate, stated in the ADR's Consequences: the commit-publish path does not go live until the redaction class on [#626](https://github.com/rolker/ros2_agent_workspace/issues/626) is closed.** #626 (OPEN, verified 2026-09-17) carries open, unfixed items in exactly the class that would be published: absolute host paths and credentials reaching stderr that the sweep report transcribes — the resolver's two `redact.sh` refusals, the failed lock-open sites that bash reports before the `say()` funnel runs, the sweep's `mkdir -p "$REPORT_DIR"` arm, and two `redact.sh` defects that leak on hostile input (a password containing `@`; a workspace path containing `=`). **The ADR cites #626 as the gate and does not restate its `file:line` references** — #626 stamps them "as of `d7b8baf`" and two of them have already moved since, so a copy in an accepted ADR would be stale on arrival. #626 is the single place those sites are enumerated. Deciding "publish means commit" must not read as authorising an unredacted host path into a *public* repo: the ADR decides the destination, #626 is the gate on switching it on, and (3)/(4) must not ship the path before it closes.
   - **Commit identity for the unattended publish path** — a dedicated bot identity (`Janitor Sweep Agent`) committing on a `skill/janitor-*` branch, the convention the `research` skill already uses, so both `check-commit-identity.py` and `check_pr_authors.py` pass and the PR is unmistakably automated. This answers the question #569's 2026-09-11 comment deferred out of that slice.
   - **An unattended PR is still reviewed by a human before merge** — "green CI is not review" (AGENTS.md § Merging) applies unchanged; stated explicitly because this is the workspace's first fully unattended PR-opening path.
   - **The trigger — decided: a weekly Claude Code cloud Routine** (operator, 2026-09-14 round-2 plan-review comment on #628). The ADR records that as its Decision, with the three candidates and the axes they differ on (table below) as its Context, so a later supersede can see what was weighed. ADR-0001's bar is to capture the decision, not to leave it open — it is captured here, not deferred to (4); (4) wires what this ADR decides.
   - **The one rule stated in every roadmap** — before choosing the next piece of work, read the roadmap and the health document together: the roadmap says where you want to go, health says what will stop you; work appearing in both goes first.

   **Recorded option, not taken (operator, 2026-09-14): splitting ADR-0020 in two** — a taxonomy ADR (kinds, two-root rule, discovery) and a mechanism ADR (publish=commit, bot identity, trigger, human-review gate), so that superseding the trigger later does not churn the taxonomy. The operator declined the split for now; it stays one ADR. Noted here so a future supersede knows the seam was seen and where it runs: the mechanism half is the "Publish means commit" / "Commit identity" / "The trigger" bullets above.
2. **Land the ADR-0012 cross-reference addendum on `docs/decisions/0015-dispatch-handoff-context-contract.md` in this PR** — decided here, not left conditional. ADR-0015's Decision is a two-actor dichotomy: the container produces, **the host publishes**, and "nothing publishes from inside the sandbox". An unattended, scheduled trigger has **no host session** — it *is* the publisher — so it is not a gap ADR-0020 can merely cite 0015 for; it is a case 0015 does not contemplate.

   **The reasoning lives in ADR-0020; ADR-0015 gets pointers only.** ADR-0012's permitted class is "purely navigational": a Status-line note and a References entry, which "don't restate, reword, or reverse the original Decision". A three-sentence paragraph asserting what ADR-0015's Decision *does not contemplate* is a scope qualification a later reader could contest — that is an argument, not a pointer, so it belongs in the ADR making it. Which sentence lands where:

   - **On ADR-0015, Status line** (one sentence, a pointer, no characterisation of 0015's scope):

     > See also [ADR-0020](0020-planning-document-kinds-and-the-two-root-rule.md), which decides the commit identity, credentials and human-review gate for a scheduled, unattended publisher.

   - **On ADR-0015, References section**: an entry for ADR-0020 with the same one-line gloss.

   - **In ADR-0020's own Decision/Context**: the whole third-actor argument — that a scheduled trigger has no host session and is itself the publisher, so it is a third actor ADR-0015's host/container dichotomy does not contemplate rather than an instance of "the host"; and that 0015's reading is unchanged for dispatched phases. ADR-0020 is the document asserting it, so ADR-0020 carries it.

   ADR-0012's own test — *"does this change what was decided?"* — is answered no for both ADR-0015 edits: the Decision text is untouched, no Consequences are added, so no supersede is required.

3. **Add `.agent/templates/roadmap.md`** — the roadmap kind's template, whose closing **loop section is copied from the BizzyBoat roadmap's "How this roadmap stays useful"** (when it is read, when leftovers land, when deferred items are pruned), generalised from "deployment start / wrap-up" to "the scope's forcing function". Also carries the "What's not on this roadmap" section and the parent/children lines the two-root rule needs.

   **Second source, not reinvented: the sibling `rolker/agent_workspace`** (public, default branch `main`; verified 2026-09-17). It already carries [`docs/ROADMAP.md`](https://github.com/rolker/agent_workspace/blob/main/docs/ROADMAP.md) — a living document with a stated guiding goal, user scenarios that items are prioritised against, and a named maintenance loop (`/brainstorm` sessions fed by `/inspiration-tracker`) — and `.agent/scripts/update_roadmap.sh`, which on issue close searches `ROADMAP.md` / `docs/ROADMAP.md` for an explicit `#<N>` and flips that item's status (table `Status` column → `done`, or `- [ ]` → `- [x]`), with no fuzzy matching and an always-exit-0 contract so it never blocks a merge. Read both before writing the template and **adopt what transfers rather than re-deriving it**: the guiding-goal header, the prioritise-against-scenarios shape, and — the part that matters for the loop this issue is about — **an item format that carries an explicit `#<N>`, so the same kind of status-flip automation is possible later** without reformatting the roadmap. This plan does **not** port `update_roadmap.sh` itself (no issue-close hook exists here to call it, and the mechanism that closes the loop in this repo is the sweep, per the ADR); the template just does not foreclose it. Note the path difference — the sibling uses `docs/ROADMAP.md`, while ADR-0020's expected-location table says `docs/roadmap.md`, matching the BizzyBoat roadmap this workspace actually reads; the ADR states the one it recommends rather than both.
4. **Write `docs/roadmap.md`** — the workspace roadmap, instantiated from that template, superseding #249. Content is drawn from what is already on record, not invented: the README § Vision as its end goal, the open direction-setting threads (#569 sweep cadence, #626, #627, #610, #609 follow-ups), the ADR backlog (3 ADRs still `Proposed` while cited as binding), and the deferred items #249 never closed out. **First-cut size bound: 150 lines.** The BizzyBoat roadmap it is modelled on is 703 lines (verified 2026-09-14) — but that is five months of accreted campaign history, not a first cut. A first roadmap that is a survey rather than a short list of next moves is hard to keep current, which is the failure mode this issue diagnoses; anything that does not fit goes to "What's not on this roadmap" or stays in its issue.

   Its loop section names **the periodic sweep** as the workspace's forcing function — the counterpart to BizzyBoat's deployments — and says so **in the future tense, with the gap named in the document itself**: the sweep exists and has been hand-run once (#569), but nothing schedules it until sub-issue (4). The loop section therefore states both, explicitly: *until (4) lands, the roadmap's forcing function is the plan-review / hand-run sweep that exists today; once (4) lands, it is the scheduled sweep.* Declaring the loop ahead of its trigger is deliberate — the trigger in (4) is being wired **to** this declared cadence, so writing the roadmap without it would mean rewriting the section three PRs later — but a loop section that quietly described a cadence nobody runs would be the exact failure this issue diagnoses in the framework's `VISION.md`, so the pending state is written down and linked to (4) rather than implied.
5. **Point README § Vision at the roadmap** — one line, so the vision document names the roadmap beneath it (the two-root rule read top-down).
6. **Make both edits in `.agent/knowledge/principles_review_guide.md`** — the Consequences Map row *and* the ADR Applicability row. (a) **Consequences Map row**: changing a roadmap or health document → the kinds table in the ADR, the roadmap template, and the parent roadmap that names it. (b) **ADR Applicability row for ADR-0020** — trigger: *writing or restructuring a planning document, or a skill reading one*; required: *use the kind vocabulary; a roadmap names the roadmaps beneath it; probe conventional paths and treat absence as normal*. **This is decided deliberately, because the table is not current**: it runs 0001–0010 and then 0013, so 0011, 0012 and 0014–0019 have **no row at all** (verified 2026-09-17) and an ADR-0020 row would be the first added in six ADRs. The guide's own Consequences Map says "An ADR in `docs/decisions/` → this review guide's ADR table", so the row is added rather than the omission continued; **backfilling the eight missing rows is out of scope for this PR** and goes on `docs/roadmap.md` as a named backlog entry — it is exactly the sort of quiet drift the roadmap-plus-health loop exists to surface, so leaving it undocumented would undercut the ADR being written. (The `janitor-sweep` exception-clause rewrite is **(3)**'s, not this PR's — the sweep's output does not change here.)
7. **Add the `AGENTS.md` rows, and the matching `CLAUDE.md` References row** — in `AGENTS.md`, a "Planning documents" pointer plus `docs/roadmap.md` in the References list; in `CLAUDE.md`, `docs/roadmap.md` added to its own References list (`CLAUDE.md:23-34`), which parallels `AGENTS.md`'s and lists `README.md` § Vision, `ARCHITECTURE.md` and `docs/decisions/` today — it would otherwise be the one References list in the repo that does not name the roadmap. This is the Consequences Map's own "`AGENTS.md` → framework adapters if affected". **Both files are Ask First**; the operator's approval of this plan at review is the approval for exactly these three additions (two in `AGENTS.md`, one in `CLAUDE.md`) and nothing more.
8. **File sub-issues (2)-(6)** with `Part of #628` in each body, using `.agent/scripts/gh_create_issue.sh`, each carrying the paragraph from the table above. **(2), (3), (4) are filed in this repo; (5) and (6) are filed in `rolker/unh_marine_autonomy` and `rolker/unh_echoboats_project11` respectively** (`gh issue create --repo <owner/repo>`; both are GitHub-origin, so this is a normal cross-repo file) and referenced from #628 with the full `owner/repo#N` syntax, in the issue body, so GitHub side-bar-links them. Cross-repo bodies say `Part of rolker/ros2_agent_workspace#628`. Post the five links as a comment on #628 so the umbrella's own record names them.
9. **PR body**: `Part of #628` and `Part of #249` — **no closing keyword on either** (operator, 2026-09-14: *"#249 stays open — the PR says `Part of #249`. The workspace roadmap absorbs its four properties and links #263/#264/#265/#266; #249 remains the Phase-2 tracker until those close. No closing keyword."*). Per AGENTS.md § Issue-closing keywords this PR closes **no** issue, and the keyword hazard applies to plan text pasted into the PR body, so this paragraph is the one to scrub-check before pasting.

   **#628 does not close here, and does not close with (4) alone.** The umbrella closes when (2), (3) and (4) have merged **and** (5) and (6) have been filed and linked — their completion is then tracked in their own repos, but they must exist before the umbrella is closed, or the issue's project-repo deliverables are lost at close.

   **What #249 asks for, checked 2026-09-14 and re-checked 2026-09-17.** Its body asks for four workspace properties (simple to use — "a small number of clear entry points, not 42 scripts"; self-documenting; durable — decisions captured where they cannot be silently reverted; sustainable — improvement as a continuing practice, not periodic rework). **#249 carries nine comments** (an earlier revision said "its one operator comment" — wrong); the three items below come from the one that adds requirements the body does not have: (i) "the README containing everything concisely" is *under consideration*, not decided; (ii) per-framework instruction duplication was **deliberate** (frameworks may prefer different layouts) and is named as an improvement at risk of accidental revert; (iii) "maybe we need a central architecture document ... consulted when an issue is considered, when a plan is made, and updated if needed when an issue is implemented" — for the workspace **and** the project. Disposition, each stated in `docs/roadmap.md` so the close is honest:
   - The four body properties → roadmap entries (the entry-point count and the durability/sustainability threads are exactly what the roadmap + sweep loop address).
   - (iii) is **largely already answered**: `ARCHITECTURE.md` exists at the workspace root; what #249 asks for beyond it is the *consult-and-update cadence*, which is what the Consequences Map and this ADR's roadmap+health rule provide. Recorded in the roadmap rather than dropped.
   - (i) and (ii) are **explicitly carried forward, not closed by this PR** — listed in the roadmap's "What's not on this roadmap" / deferred section with (ii) flagged as a decision to protect, not revisit.

   **#249's live children stay named.** Its last comment (2026-02-26) records Phase 1 complete via merged PR #257 and **Phase 2 as not started**, naming where to resume: [#263](https://github.com/rolker/ros2_agent_workspace/issues/263) (simplify scripts and Makefile — the same "not 42 scripts" body property the roadmap absorbs) and [#264](https://github.com/rolker/ros2_agent_workspace/issues/264) (consolidate documentation overlap), with [#265](https://github.com/rolker/ros2_agent_workspace/issues/265) (CI enforcement checks) and [#266](https://github.com/rolker/ros2_agent_workspace/issues/266) (evaluate MCP servers) spun off. **All four are OPEN** (verified 2026-09-17), as is #249 itself. `docs/roadmap.md` names all four in its #249 disposition, so the roadmap absorbing #249's properties does not orphan the issues that actually do the work — and since the PR uses no closing keyword, #249 remains the Phase-2 tracker until they close.

## Trigger comparison — the ADR's input; the decision is recorded below

**Decided by the operator at the round-2 plan-review checkpoint (2026-09-14,
comment on #628): the trigger is a weekly Claude Code cloud Routine.** Their
stated grounds, quoted: *"Smallest credential surface on a public repo (no new
secret stored anywhere), exact cadence, GitHub-only reach (already the rotation
rule)."* ADR-0020 records that as its Decision; the table below stays in this
plan as the input the decision was made from, so the ADR's Context can carry
the alternatives and why they were not taken (ADR-0001's bar).

| Mechanism | Reach | Cadence reliability | Commit identity | Credential surface (new stored secrets) | Cost |
|---|---|---|---|---|---|
| **GH Actions weekly cron** (workspace repo) | The GitHub-origin repos the manifests declare, on a fresh clone — works because the sweep already resolves repos via `resolve_repo_checkout.sh` + the manifest fallback. No `layers/` tree | Exact and unconditional — independent of whether the laptop is on | Bot identity set per-commit with `git -c user.name/user.email` (AGENTS.md § Agent Commit Identity). Same chore as the other two rows | **2 new.** Both stored on a **public** repo: the Claude subscription token (`CLAUDE_CODE_OAUTH_TOKEN`), and a **cross-repo write credential** (PAT or GitHub App token) — health documents are committed to the repos they grade, and the built-in `GITHUB_TOKEN` is scoped to the repo running the workflow | Actions minutes free on a public repo; the agent run itself draws the same subscription usage as the other two |
| **Claude Code cloud Routine** ← **chosen** | Same set of repos; no `layers/` tree (Routines are cloud-hosted and GitHub-repo-scoped — `.agent/knowledge/research_digest.md:124,132`) | Scheduled; daily run caps by plan (Pro 5 / Max 15 / Team 25 per the digest — the tier floor is reported inconsistently across secondary sources, so verify against current docs before designing around it) | Same per-commit `-c` chore, and it satisfies the same checks: `check_pr_authors.py` inspects each commit's **primary author email** (`check_pr_authors.py:115-127`), not the account that opened the PR | **0 new.** Runs on Anthropic-managed cloud infrastructure under the operator's existing Claude auth and its existing GitHub connection; nothing is stored in any repo, and no credential moves onto the dev host that is not there already | Draws subscription usage; no separate infra charge |
| **anacron on the dev laptop** | Same set of repos, **plus the local `layers/` tree** — see "what `layers/` buys a sweep" below | Runs missed jobs at next boot, so laptop-off is tolerated — but the interval is boot-dependent, not calendar-exact | Same per-commit `-c` chore | **0 new.** Every credential stays on the dev host, where it already is: the host's `gh` auth (`~/.config/gh/hosts.yml`) and the OAuth token at `~/.config/ros2-agent/claude-oauth-token` | No infra charge; draws the same subscription usage |

**No mechanism reaches the gitcloud repos, and none would.** `janitor-sweep`
excludes non-GitHub origins by URL before the rotation is built
(`is_field_url`, `.claude/skills/janitor-sweep/SKILL.md` step 2 of the
repo-enumeration section), and every one of the 44 `url:` entries in
`configs/manifest/repos/*.repos` is on `github.com` (verified 2026-09-17). An
earlier revision of this table credited anacron with "full reach — the only
option that sees gitcloud repos *and* a real `layers/` tree"; the gitcloud half
of that was wrong and is corrected here.

**What `layers/` actually buys a sweep** — the whole of anacron's unique reach,
and it is narrow. The sweep and `audit-project` are built not to assume
`layers/` exists (`janitor-sweep/SKILL.md:38,197`), and
`resolve_repo_checkout.sh` falls back to a manifest-pinned clone. Two things
differ when a layer checkout is present:

1. **The two layer-dependent `audit-project` checks run instead of reporting
   SKIPPED** — the optional `colcon test` run (step 5, which needs a built
   layer workspace) and the "is it in the expected layer?" cross-reference
   (step 7). In `clone` mode both are SKIPPED, never OK
   (`audit-project/SKILL.md:152-156,254`).
2. **What is graded is the operator's working tree as it stands** — feature
   branches, uncommitted edits and all — rather than the ref the manifest pins.
   `resolve_repo_checkout.sh` documents this as a deliberate difference in
   guarantee, not a better or worse one: a `clone` was decided by the manifests,
   a `layer` checkout is accepted as is.

Whether a health document *should* grade one developer's working tree is a
design question, not just a reach one. The chosen Routine answers it by
default: health documents describe the pinned, pushed state, which is also the
state a reader of the committed document can check.

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
repo. The `skill/janitor-*` branch and the `Janitor Sweep Agent` commit author
are what make the PR legible as automated under the chosen mechanism.

**The real discriminators**, with the superlatives dropped and each axis stated
plainly, are two: **cadence exactness** (Actions is calendar-exact and
laptop-independent; the Routine is scheduled but rate-capped per plan; anacron
is boot-dependent) and **new stored secrets** (Actions 2, both on a public repo;
Routine 0; anacron 0) — the third, **whether the sweep should see the local
`layers/` tree at all**, is the design question above rather than a straight
capability win. Weekly is the cadence the diff argument points to (the issue's
own "weekly is a diff, quarterly is another baseline"), and weekly is what the
operator chose.

## Files to Change

| File | Change |
|------|--------|
| `docs/decisions/0020-planning-document-kinds-and-the-two-root-rule.md` | New ADR (kinds, two-root rule, discovery by convention, publish=commit, bot identity, human-review gate, trigger decision, roadmap+health rule) |
| `.agent/templates/roadmap.md` | New — roadmap kind template; loop section copied from the BizzyBoat roadmap's closing section |
| `docs/roadmap.md` | New — the workspace roadmap, superseding #249 |
| `README.md` | One-line pointer from § Vision to `docs/roadmap.md` |
| `.agent/knowledge/principles_review_guide.md` | **Two edits**: a new Consequences Map row for roadmap/health documents, and a new ADR Applicability row for ADR-0020 (the table currently stops at 0013 — see Approach step 6; the eight missing rows are a roadmap backlog entry, not this PR's work) |
| `docs/decisions/0015-dispatch-handoff-context-contract.md` | ADR-0012 cross-reference addendum: a one-sentence Status-line pointer to ADR-0020 + a References entry with the same gloss. The third-actor reasoning stays in ADR-0020 (Approach step 2); the Decision text is untouched |
| `AGENTS.md` | Planning-documents pointer + `docs/roadmap.md` in References (**Ask First** — approved by this plan's review) |
| `CLAUDE.md` | `docs/roadmap.md` added to its own References list (`CLAUDE.md:23-34`), which parallels `AGENTS.md`'s (**Ask First** — approved by this plan's review) |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Capture decisions, not just implementations | The ADR is the deliverable; it settles trigger and commit identity rather than deferring them again (ADR-0001's bar). |
| Only what's needed / Improve incrementally | The umbrella is split into four independently mergeable sub-issues; (1) ships two documents and a template, not a structure. This is the direct answer to the #249/#257 stall the issue itself cites. |
| Workspace vs. project separation | The two-root rule *is* this principle expressed as structure: the workspace assumes two roots and nothing else; everything below the project root is discovered by convention, and its absence is normal (ADR-0003). |
| A change includes its consequences | Consequences Map row and AGENTS.md rows land in this PR; the `janitor-sweep` exception-clause rewrite lands in (3), where the sweep's output actually changes — recorded here so it cannot be lost. |
| Human control and transparency | The ADR states the human-review-before-merge gate for the first unattended PR-opening path, and the trigger is the operator's choice, not the plan's. |
| Enforcement over documentation | Honest limit: this sub-issue is documentation. The enforcement is the sweep itself — the forcing function that makes the roadmap get read — and it lands in (3)/(4). Stated in the ADR's Consequences as a known gap until then. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0001 (ADRs) | Yes | This is the ADR; it decides rather than surveys. |
| ADR-0003 (project-agnostic workspace) | Yes | Two-root rule + discovery keep the workspace from presuming any project's layout. |
| ADR-0006 / ADR-0017 (AGENTS.md two-tier) | Yes | AGENTS.md rows are pointers; project-repo planning documents are referenced, never forked; (2)'s graceful absence mirrors ADR-0017's incremental rollout. |
| ADR-0004 (enforcement hierarchy) / ADR-0005 (layered enforcement) | Yes | **Triggered and deliberately deferred, stated rather than silent.** ADR-0020 states two new compliance rules — "a roadmap names the roadmaps beneath it" and "read the roadmap and the health document together before choosing work" — with **no hook, CI check or guardrail in this sub-issue**; by ADR-0005's own test ("if a rule isn't in CI, it's a suggestion, not a rule") they are suggestions until then. The enforcement is the sweep: the two-root rule becomes checkable when the conventional-path probe lands in **(2)**, and the read-both rule gets its forcing function when the trigger lands in **(4)**. ADR-0020's Consequences records this as a named gap with those two sub-issues as the closers, so the deferral is on the record and not discovered later. (ADR-0005 is itself still `Proposed` while cited as binding — one of the three the workspace roadmap lists.) |
| ADR-0012 (cross-reference addendums) | Yes | **Decided here, not conditional** (step 2): the addendum lands on ADR-0015 in this PR as a one-sentence Status-line pointer plus a References entry — ADR-0012's "purely navigational" class, restated so the addendum characterises nothing about 0015's scope; the third-actor argument sits in ADR-0020, which is the document making it. The Decision text is untouched, so no supersede is required. |
| ADR-0013 (progress.md vocabulary) | Indirect | The health document changes the janitor exception's premise — flagged, and handled in (3). |
| ADR-0014 (deployment mode) | Yes | The BizzyBoat cadence being generalised. `.agents/deployment.yaml` is the per-repo-declaration precedent the ADR **considered and did not follow** for discovery (operator, 2026-09-17: conventional paths instead) — recorded in the ADR as the option not taken. |
| ADR-0015 / ADR-0019 (dispatch handoff / containment) | Yes | Cited for publish=commit (container produces, host publishes) and for why the unattended path's identity and auth must be stated, not assumed. |
| ADR-0018 (local-first CI) | No | Workspace repo; hosted checks stay required. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Add an ADR in `docs/decisions/` | The ADR table in `.agent/knowledge/principles_review_guide.md` | Yes — the ADR-0020 row, named in Files to Change and Approach step 6. The table's eight pre-existing gaps (0011, 0012, 0014–0019) are recorded on the roadmap, not backfilled here |
| Add a template in `.agent/templates/` | Docs/skills that reference it | Yes — the ADR and `docs/roadmap.md` are its only referents today |
| `AGENTS.md` | Framework adapters if affected | Yes — `CLAUDE.md` gains the same `docs/roadmap.md` References row in this PR (step 7). An earlier revision dismissed this as "adapters carry no roadmap rows to drift": true but beside the point — the adapter's own References list is precisely what should gain the row |
| The sweep's durable output moves out of the scratchpad | The `janitor-sweep` exception clause in the Consequences Map | No — belongs to (3); recorded in (3)'s scope so it cannot be dropped |
| A new shared script, if (2) factors the conventional-path probe into one | `AGENTS.md` Script Reference table | No — belongs to (2), and only if (2) writes a script at all; named in (2)'s scope |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): `README.md` § Vision (gains the roadmap pointer); `.agent/knowledge/principles_review_guide.md` (Consequences Map row + ADR table row for ADR-0020); `AGENTS.md` (References); `CLAUDE.md` (References — the adapter's parallel list). Nothing else in the repo asserts that the workspace has no roadmap.
- **Agent-instruction candidates** (proposals only — the operator decides): the "read the roadmap and the health document together before choosing work" rule is an instruction-level behavior, proposed for `AGENTS.md` once a health document actually exists — i.e. with (3), not now. Proposing it early would point agents at a file that is not there.

## Open Questions

- **Trigger mechanism — DECIDED (operator, 2026-09-14): a weekly Claude Code cloud Routine.** No longer open. The comparison table above is retained as the ADR's Context — corrected in this revision so that reach, cost, commit identity and credential surface are scored on the same basis in all three rows (the anacron row no longer claims gitcloud reach, which no mechanism has, and the credential column states new-stored-secret counts instead of superlatives).
- **Discovery mechanism — DECIDED (operator, 2026-09-17): expected locations following common conventions, published as a table in the ADR; no new per-repo file, no schema.** Quoted: *"Maybe the skill can look at expected locations for files, and those expected locations should probably follow common conventions."* And, refining it the same day: *"I think we should at least document what kind of documents are expected where so we can set expectations for a project, without dictating how a project should store its documents."* So ADR-0020 carries an explicit **kind → expected location** table (Approach step 1) as a **recommendation that sets expectations**, never a requirement: it tells a project what the skills will look for, and a project that stores a document elsewhere is not in violation and produces no error or finding. This closes the round-1 suggestion that was deferred to the operator at plan review and, per the round-2 review, was recorded only in `progress.md` and not here. Sub-issue (2) shrinks to the reader that probes that table's paths plus its graceful-absence test, and the synthetic-fixture and sibling-manifest-comparison items proposed at the round-2 checkpoint are dropped with it.
- **Is `.agent/templates/roadmap.md` in (1) or a later sub-issue?** The plan puts it in (1) because `docs/roadmap.md` is its first instance and the loop section would otherwise be written twice. Reversible at review.
- **`AGENTS.md` *and* `CLAUDE.md` are Ask First** — approving this plan approves exactly the three References-level additions in step 7 (two in `AGENTS.md`, one in `CLAUDE.md`) and nothing more.
- **Does this PR close #249? — DECIDED (operator, 2026-09-14): no.** The PR says `Part of #249`; #249 stays open as the Phase-2 tracker until #263/#264/#265/#266 close. The roadmap still absorbs its four body properties and states the disposition of the three items from its comment, and now names those four children as well (step 9).
- **Splitting ADR-0020 into a taxonomy ADR and a mechanism ADR** was offered at plan review and **declined by the operator (2026-09-14)**. Recorded as an option in Approach step 1, not taken; re-openable if the trigger decision later proves volatile.

## Estimated Scope

Umbrella #628: four PRs in this repo, plus two issues filed in project repos ((5), (6)). **This worktree: one PR** — three new documents and six small edits (`README.md`; **two** rows in `.agent/knowledge/principles_review_guide.md`; `AGENTS.md`; `CLAUDE.md`; the ADR-0015 addendum), plus filing sub-issues (2)–(6).
