---
issue: 605
---

# Issue #605 — Local Model Adversarial Specialist (5f) cannot run on the dev host: 35b model vs 8GB VRAM, and diffs exceed num_ctx

## Issue Review
**Status**: complete
**When**: 2026-08-23 10:00 -04:00
**By**: Claude Code Agent (Claude Sonnet 5 (1M context))

**Issue**: #605
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: needs-more-detail

### Scope Assessment

**Well-scoped?** Mostly — the diagnosis (two failure modes, in direct
tension) is precise and evidence-backed. The four proposed changes are all
plausibly necessary, and AGENTS.md's filing discipline favors bundling
related changes into one PR with atomic commits, so keeping this as one
issue is fine. But two of the four items are structurally different from
the other two and the plan should treat them that way:

- Item 2 (chunk large diffs in `local_review.sh`) is a **host-independent
  correctness fix** — no `num_ctx` on any single-GPU box swallows an 88k-token
  diff. It belongs in this repo's script regardless of which model ships as
  default.
- Items 1, 3, 4 (model default, `OLLAMA_KV_CACHE_TYPE`, `OLLAMA_KEEP_ALIVE`)
  are **this-host tuning** for an 8GB-VRAM box. `local_review.sh`'s model/URL
  are already env-overridable per-operator (`LOCAL_REVIEW_MODEL`,
  `LOCAL_REVIEW_URL`), which is the right layer for host-specific defaults —
  but item 3 (see below) proposes reaching past that into systemd, which is
  a different kind of change with different approval needs.

Recommend the plan keep chunking as its own atomic commit(s) so it lands
even if the host-tuning discussion runs long, and call out in the plan
which of the 4 items are "generically correct" vs. "true for this operator's
current hardware" (a second dev host with more VRAM would want different
defaults for 1/3/4 but the same chunking).

**Right repo?** Yes — `.agent/scripts/local_review.sh` is a workspace-repo
script; this is workspace infra, not project content.

**Dependencies**: Directly related to #585 (container Ollama reachability)
and #590 (make 5f opt-in), both currently open. See Recommendations below —
the issue's framing that #585 is "downstream of this" is not quite right and
should be corrected before or during planning.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Only what's needed | Watch | 4 changes bundled under one acceptance list; recommend the plan sequence them as separable atomic commits (see Scope Assessment) so a partial land (e.g. chunking merges, host-tuning stalls on operator sign-off) still leaves the workspace better off. |
| Human control and transparency | Action needed | Item 3 (`OLLAMA_KV_CACHE_TYPE`) needs a systemd drop-in under `/etc` on the operator's own dev workstation. AGENTS.md's hard "Never" on host administration is explicitly scoped to *remote/field* hosts (salmon, gabby, boats) — this dev host isn't covered by that hard stop. But it's still an unattended host-level `/etc` change with sudo, so it should not be applied without the operator's explicit, specific approval (not inferred from approving the issue/plan in general) at plan or implementation time. The plan should also check whether Ollama's `/api/chat` `options` accept a per-request KV-cache-type override (mirroring how the issue already scopes `keep_alive` per-request specifically to avoid a global host change) — if so, item 3 could avoid the systemd drop-in entirely. |
| Enforcement over documentation | Watch | Acceptance criteria say the host-level change should be "documented so a rebuilt or second dev host can be brought to the same state." Prefer an idempotent setup script (e.g. under `.agent/scripts/`, run manually with the operator's consent) over prose-only documentation, consistent with this principle — flag as a plan-time choice, not a blocker. |
| Capture decisions, not just implementations | Watch | The empirical model-quality check (planted-defects spot-check) is exactly the kind of decision that should leave a record — an ADR is probably overkill for a model-tag choice, but the plan should say where the spot-check result and chosen tag's rationale get written down (progress.md entry, or a short note in `.agent/knowledge/`) so it doesn't evaporate once #605 closes. |
| A change includes its consequences | OK | Acceptance criteria already cover verification (no OOM, chunking exercised, quality spot-check, host-change documented). |
| Test what breaks | OK | Chunking and context-overflow handling are exactly the kind of edge-case logic worth a regression test in `local_review.sh`'s own test coverage (if any exists — plan should check) rather than only manual verification. |
| Workspace vs. project separation | OK | Workspace-only change. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| 0001 — Adopt ADRs | No | Model-tag/tuning choice is operational, not architectural; a progress.md/knowledge-note record (see above) is proportionate. |
| 0004/0005 — Enforcement hierarchy | Watch | If item 3 becomes a documented manual step rather than a script, note that this is documentation without enforcement (drift risk on a rebuilt host) — acceptable given it's a one-time host config, but worth a line in the plan. |
| 0009 — Python packaging | No | Not applicable — no Python packages installed here. |

### Consequences

- `local_review.sh`'s doc comment block (env var descriptions, the
  `qwen3.5:35b` default reference, the `num_ctx`/reasoning-headroom
  rationale) will need updating if items 1/2/3/4 land — the script's own
  header currently documents the exact tension this issue describes, so it
  is effectively the spec and must move in lockstep with the code.
  `AGENTS.md`'s script reference table only needs an update if the script's
  external usage/flags change (chunking should stay internal, so this is
  likely a no-op — verify at plan time).
- If a new setup script is added for the systemd drop-in, add it to
  `AGENTS.md`'s script reference table per the consequences map.
- Consider proposing a `.agent/knowledge/` note on sizing local models
  against VROM-constrained dev hosts (per "Implementation surfaces a
  reusable pattern" in the consequences map) — this failure mode will recur
  on any 8GB-class box, not just this one.

### Recommendations

- Correct the issue's dependency framing before/while planning: #605 fixes
  capacity/context on the *current dispatch path* (5f run directly against
  `localhost:11434` from the host or an in-process agent). #585 is about
  *container-dispatched* reviews reaching the host's Ollama at all over
  Docker's bridge network — a pure connectivity problem, unaffected by
  model size or `num_ctx`. Fixing #605 does not fix #585; they are
  independent, not root-cause/downstream. The plan should scope #605 as
  fixing the non-container path and leave #585 open on its own merits.
- #590 (make 5f opt-in) is the one genuinely resolved by #605 succeeding:
  if the acceptance criteria are met (5f completes without OOM, on
  PR-#350-scale diffs, with a checked quality floor), #590's premise — 5f
  is unhelpfully slow/unreliable so it should default off — goes away.
  Recommend the plan for #605 explicitly re-triage #590 at the end (close
  as superseded, or re-scope to "opt-in only on hosts where the 5f
  preflight can't confirm capacity") rather than leaving both open issues
  making opposite default-on/off arguments indefinitely.
- Before committing to a default model tag, the plan should also record
  *why* the chosen tag's context headroom is sufficient for chunked
  per-file passes (chunking changes what "big enough context" means — a
  smaller model with a smaller context might be fine per-chunk even though
  it couldn't hold the full 88k-token diff).
- Verify whether `local_review.sh` currently has any automated tests; if
  the chunking logic is nontrivial (per-file pass + synthesis step), it
  should get direct test coverage rather than relying only on the
  acceptance-criteria manual exercise against a real large diff.

### Actions
- [ ] Sequence the four proposed changes as separable atomic commits: chunking (host-independent correctness) vs. model default / KV-cache-type / keep-alive (host-specific tuning).
- [ ] Get explicit operator sign-off before applying the `OLLAMA_KV_CACHE_TYPE` systemd drop-in specifically (host-level `/etc` change with sudo); check first whether a per-request API option can avoid the host change entirely.
- [ ] Prefer an idempotent setup script over prose-only documentation for the host-level change, if the operator approves making it.
- [ ] Record the planted-defects quality spot-check result and chosen model tag's rationale somewhere durable (progress.md or `.agent/knowledge/`).
- [ ] Correct the issue's "#585 is downstream of this" framing — #605 and #585 are independent problems (context/capacity vs. container networking); do not treat #605 as blocking or resolving #585.
- [ ] Re-triage #590 once #605's acceptance criteria are met — likely close as superseded rather than leaving both default-on and default-off issues open.
- [ ] Verify `local_review.sh`'s existing test coverage and add tests for the chunking logic, not just manual verification against a real diff.

## Plan Authored
**Status**: complete
**When**: 2026-08-23 23:46 -04:00
**By**: Claude Code Agent (Claude Sonnet 5 (1M context))

**Plan**: `.agent/work-plans/issue-605/plan.md` at `2ef2541`
**Branch**: feature/issue-605 at `2ef2541`
**Phases**: single (5 sequenced commits within the one PR: chunking+tests, per-request keep_alive+tests, model-default change pending empirical check, knowledge doc, conditional systemd drop-in script written-but-not-executed)

### Open questions
- [ ] Does `qwen3.5:4b-q8_0` clear the planted-defect recall check well enough to ship alone, or is the `9b-q4_K_M` stretch candidate (needing KV-cache-type) necessary for acceptable review quality? Determines whether item 4 (systemd drop-in) is needed at all.
- [ ] If the drop-in is needed, operator sign-off on running `setup_ollama_kv_cache.sh` is a separate ask at implementation time — not covered by plan approval.
- [ ] Is `LOCAL_REVIEW_KEEP_ALIVE` default of `30s` right, or should it be tuned after observing real chunked-request timing?

## Plan Review
**Status**: complete
**When**: 2026-08-23 23:48 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-605/plan.md` at `2ef2541`
**PR**: PR-less (reviewed via worktree/issue number)
**Verdict**: changes-requested

### Findings
- [ ] (must-fix) Chunking's VRAM budget arithmetic (item 3) assumes a "modest per-chunk `num_ctx`," but no commit actually changes `NUM_CTX`/`LOCAL_REVIEW_NUM_CTX` — it stays a single global value (default 32768) used identically for every chunk's `options.num_ctx` (`local_review.sh:224`). Ollama/llama.cpp allocates the KV cache buffer sized to the requested `num_ctx` at model-load/request time, not to the tokens actually used, so chunking alone does not shrink the KV-cache VRAM footprint the way item 3's table implies — the 9b/4b candidates' "headroom for KV cache" column is only accurate if a smaller `num_ctx` is actually requested per chunk. The plan should either (a) size each chunk's request `num_ctx` dynamically from that chunk's estimated tokens (extending the per-chunk budget check in item 1 to also pick the request's `num_ctx`, with a documented floor/ceiling), or (b) explicitly lower `LOCAL_REVIEW_NUM_CTX`'s default alongside the model change and say why that's sufficient — and add this to the Files to Change / commit list, not leave it implicit in the item-3 narrative. As written, the empirical planted-defect check (item 3, step 2) will exercise the real value, which may simply surface this gap live rather than the plan preempting it. — `plan.md` items 1 and 3
- [ ] (suggestion) The plan never states that per-file (and per-hunk) chunking is a systematic blind spot for defects that span files or depend on a caller/callee relationship the chunk cannot see (e.g., a signature change in file A whose caller in file B is now wrong, or duplicated logic introduced across two files). Item 7 discusses only the cross-chunk *duplicate-finding* cost of skipping an LLM synthesis pass, not this correctness gap. Given `review-code`'s other specialists retain full-repo context, this may be an acceptable v1 tradeoff for 5f specifically — but per "Human control and transparency" it should be stated as a known limitation (script header comment and/or `.agent/knowledge/local_model_sizing.md`), not left silent. — `plan.md` item 1, step 7
- [ ] (suggestion) Verified independently: `run_script_tests.sh`'s glob (`"$TESTS_DIR"/test_*.sh`, `.agent/scripts/tests/run_script_tests.sh:24`) does auto-discover a new `.agent/scripts/tests/test_local_review.sh` with no Makefile change — the plan's claim checks out. No action needed; noting this was checked per the review brief rather than trusted.

### Other dimensions checked, no findings
- Loud-failure guarantee (probe 2): preserved at every level — per-file, per-hunk, and the irreducible-single-hunk case all fail loud with a specific file/hunk name, and per-chunk send/parse failures abort the whole review rather than silently dropping a file's findings (item 1 steps 4/6).
- Scope discipline on the systemd drop-in (probe 4): item 4 is explicit that the plan does not authorize execution, the script requires confirmation/`--yes` before touching `/etc`, and the gate is a stated stop-and-ask, not a code comment. Consistent with the operator's binding decision recorded in the plan's investigation section.
- Model-selection gate (probe 3, partial): the empirical planted-defect check (fixed defect set, objective scoring doc, `ollama ps` PROCESSOR=100%GPU + no-OOM as the real acceptance gate rather than the registry-size arithmetic) is well-specified and not a rubber stamp — the arithmetic table is correctly framed as a first-pass filter only. The one gap is the `num_ctx` interaction above.
- Issue Review follow-through: all seven `review-issue` action items are addressed in the plan (separable commits, per-request `keep_alive` investigated and confirmed, drop-in gated on separate sign-off, planted-defect check specified, #585 framing corrected, #590 re-triage scheduled as a follow-up, existing test coverage checked — zero, addressed with new tests).
- Documentation & Instruction Impact section: present, non-silent, correctly frames the new knowledge doc as a candidate for operator approval rather than an auto-applied edit.

## Plan Authored
**Status**: complete
**When**: 2026-08-23 12:10 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-605/plan.md` at `5d3f2a9` (revision of `2ef2541`)
**Branch**: feature/issue-605 at `5d3f2a9`
**Phases**: single (5 sequenced commits within the one PR, unchanged from prior authoring — chunking+tests now includes per-chunk `num_ctx` sizing, per-request keep_alive+tests, model-default change pending empirical check, knowledge doc, conditional systemd drop-in script written-but-not-executed)

### Revision summary (responds to Plan Review changes-requested, `91321a8`)

- **Must-fix closed**: item 1 (chunking) gains an explicit new step 5,
  "Per-chunk `num_ctx` sizing" — each chunk's request now sizes
  `options.num_ctx` from that chunk's estimated tokens via a fixed bucket
  set (`8192 16384 24576 32768`), capped at `LOCAL_REVIEW_NUM_CTX`, instead
  of every chunk reusing the single global 32768 default
  (`local_review.sh:224` today). `ANSWER_HEADROOM` (currently 12288,
  tuned for `qwen3.5:35b`) is flagged as needing re-measurement per
  candidate model — folded into item 3's empirical check as a new step 1
  ("measure before scoring quality"). Item 3's candidate table is redone
  to show the `num_ctx` bucket each candidate would actually request and
  is explicit that the headroom-GB figures are order-of-magnitude
  placeholders pending that measurement, not verified numbers. A new
  "Arithmetic method" subsection states the general
  weights+KV-cache-VRAM formula (`kv_bytes_per_token` from `ollama show
  --verbose`, `kv_GB(num_ctx)`, `headroom_GB`) so it can be re-derived on
  different hardware or a different model tag, per the review's ask.
- **Suggestion closed**: item 1 gains step 9, documenting per-file/per-hunk
  chunking's cross-file/caller-callee blind spot as a known v1 limitation
  — added to the `local_review.sh` header comment and to the new
  `.agent/knowledge/local_model_sizing.md` (item 5), explicitly noting 5f
  is the only specialist affected (other `review-code` specialists retain
  full-repo context).
- Files to Change, Consequences, Documentation & Instruction Impact, and
  Open Questions sections updated to reflect the num_ctx-sizing code work
  and the new limitation-documentation content. Commit structure stays at
  5 (num_ctx sizing folded into commit 1, not a new commit) — the review
  did not require a 6th commit and the change is additive to the existing
  chunking commit's own scope.
- Not re-litigated, per the caller's instruction: the no-per-request
  `OLLAMA_KV_CACHE_TYPE` investigation, the `run_script_tests.sh` glob
  verification, the loud-failure guarantee, the drop-in gate, and the
  planted-defect quality check all carry forward unchanged from the prior
  plan version.

### Open questions
- [ ] Does `qwen3.5:4b-q8_0` clear the planted-defect recall check well enough to ship alone, or is the `9b-q4_K_M` stretch candidate necessary for acceptable review quality? Determines whether item 4 (systemd drop-in) is needed at all.
- [ ] Item 3's candidate headroom numbers are placeholders pending the empirical check's new step 1 (`kv_bytes_per_token` measurement per candidate) — `9b-q4_K_M` may not clear the gate even with chunking once measured.
- [ ] The `num_ctx` bucket set (`8192 16384 24576 32768`) is a first proposal, not a measured optimum — may need adjusting once real per-chunk token counts are observed.
- [ ] If the drop-in is needed, operator sign-off on running `setup_ollama_kv_cache.sh` is a separate ask at implementation time — not covered by plan approval.
- [ ] Is `LOCAL_REVIEW_KEEP_ALIVE` default of `30s` right, or should it be tuned after observing real chunked-request timing?
