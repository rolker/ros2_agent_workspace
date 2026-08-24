# Plan: Local Model Adversarial Specialist (5f) cannot run on the dev host

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/605

## Context

`.agent/scripts/local_review.sh` (specialist 5f) has never produced a
finding against `unh_marine_autonomy` PR #350 in four review rounds, on the
dev host (deadpool). Two independent failure modes, in tension with each
other:

- **Capacity**: default model `qwen3.5:35b` is 23 GB against 8 GB VRAM (RTX
  2080 Super). Ollama spills ~16 GB to CPU RAM (31 GB total, 1 GB swap,
  full). 4 `llama-server` OOM kills in 7 days, with collateral kills of
  `chrome` and `CCOMAutonomousM`. Raising `num_ctx` to fit a large diff makes
  this worse (KV cache scales with context).
- **Context**: PR #350's unreviewed portion alone is ~40k tokens; full
  branch diff ~87.6k, against default `num_ctx=32768`. `local_review.sh`
  already fails loud on overflow rather than let Ollama silently truncate
  (`local_review.sh:200-207`, backstopped by the `prompt_eval_count` check
  at `local_review.sh:266-272`) — that guard must survive this change, just
  scoped to a smaller unit of work.

Per the Issue Review (`.agent/work-plans/issue-605/progress.md`), the four
proposed changes split into a **host-independent correctness fix**
(chunking) and **this-host tuning** (model default, KV-cache type,
keep-alive). This plan sequences them as separable commits so chunking can
land even if host-tuning needs more operator back-and-forth.

**Operator decision (already made, binding on this plan)**: on
`OLLAMA_KV_CACHE_TYPE`, try a per-request option first; only propose a
systemd drop-in if that's impossible, and come back and ask before applying
one. See item 4 below — the investigation is done and the answer is
negative, so the drop-in path is retained but gated on a separate ask.

## Investigation: does Ollama 0.32.0 support a per-request KV-cache-type override?

Checked before committing to the systemd path, per the operator's stated
preference. Inspected the installed binary (`/usr/local/bin/ollama`,
`ollama version 0.32.0`):

- `ollama serve --help` lists `OLLAMA_KV_CACHE_TYPE` only as a **server
  environment variable**, alongside `LLAMA_ARG_FIT_TARGET` and other
  process-launch settings.
- `strings /usr/local/bin/ollama | grep -i cache` surfaces
  `--cache-type-k` / `--cache-type-v` — these are `llama-server` CLI flags
  the Ollama runner passes at **subprocess launch**, derived from
  `OLLAMA_KV_CACHE_TYPE` in the runner's own environment
  (`server.LlmRequest`, `kvCacheType`, `GGML.SupportsKVCacheType` in the
  binary's Go symbol table).
- The only per-request JSON option confirmed present via symbol/tag search
  (`json:"num_ctx,omitempty"`, `NumCtx`, `numCtxAuto`) is `num_ctx`. No
  `kv_cache_type` (or similar) JSON tag exists anywhere in the binary.

**Conclusion: no per-request override exists in Ollama 0.32.0.** KV cache
quantization is fixed at model-load time by the server process's
environment, full stop — unlike `keep_alive`, which genuinely is a
per-request `/api/chat` field (confirmed the same way: `keep_alive` has a
JSON tag and is read per-request in the handler). Item 4 below implements
`keep_alive` per-request as planned; item 3 (KV-cache type) has no
per-request path and stays gated behind explicit operator approval for a
systemd drop-in — this plan does **not** authorize applying it.

## Approach

### 1. Chunk large diffs in `local_review.sh` (host-independent correctness — Commit 1)

Rewrite the diff-gathering/prompt-building section (`local_review.sh:132-217`
in the current script) to operate per-file with a deterministic merge, not a
single monolithic prompt:

1. **Split**: parse the full diff (from `--base` or stdin) into per-file
   segments on `^diff --git a/.* b/.*$` boundaries — each segment is one
   file's complete hunk set with its own header.
2. **Per-file budget check**: estimate tokens for each file segment the same
   way the current script estimates the whole prompt (`${#PROMPT} / 3 +
   headroom`). If a file segment alone fits under `NUM_CTX`, send it as one
   chunk.
3. **Sub-file splitting**: if a single file's segment still exceeds
   `NUM_CTX` even alone (a very large generated file, a big data diff),
   split further on `^@@ .* @@$` hunk boundaries and group consecutive hunks
   into sub-chunks that fit the budget, each retaining the file's `diff
   --git` / `---`/`+++` header so the model has file identity.
4. **Irreducible-oversize guard (loud-failure preserved)**: if a *single
   hunk* alone (the smallest splittable unit) still exceeds `NUM_CTX` minus
   headroom, fail loud exactly as today — same error shape, but naming the
   specific file/hunk that can't be reduced further, and the `NUM_CTX` that
   would be needed. This is strictly rarer than today's whole-diff check
   (a lone 40k-token hunk is a different, worse problem than a 40k-token
   diff), so the guarantee gets *stronger*, not weaker.
5. **Per-chunk `num_ctx` sizing (closes the KV-cache gap flagged in Plan
   Review).** The chunking in steps 1-4 only shrinks *prompt content* — it
   does nothing to VRAM unless the per-request `options.num_ctx` actually
   shrinks too. Ollama/llama.cpp sizes the KV-cache buffer to the
   *requested* `num_ctx`, not to tokens actually consumed
   (`local_review.sh:224` currently sends the single global `NUM_CTX` on
   every request, chunked or not — verified by reading the `build_body()`
   function). So this step is a **required code change**, not a
   side-effect of chunking:
   - For each chunk, after estimating its tokens the same way the
     whole-diff estimate works today (`${#CHUNK_PROMPT} / 3 + headroom`),
     pick that request's `num_ctx` from a small set of fixed buckets
     (e.g. `8192 16384 24576 32768`, ordered ascending) — the smallest
     bucket that is `>= chunk_estimated_tokens`, capped at
     `LOCAL_REVIEW_NUM_CTX` (which remains the hard ceiling / overflow
     guard, see step 4 above). Fixed buckets (not a value computed to the
     token) avoid a distinct KV-cache allocation shape per request while
     still landing most single-file chunks well under the 32768 default.
   - The `ANSWER_HEADROOM` constant (today `12288`, sized for
     `qwen3.5:35b`'s observed ~7k-token reasoning on a 100-line diff) is
     itself close to the smallest useful bucket, so it must be re-measured
     for whichever candidate model item 3 selects — a smaller model that
     reasons in fewer tokens lets smaller buckets be viable, which is the
     actual mechanism that makes the item-3 headroom numbers real rather
     than aspirational. Record the measured value in
     `.agent/knowledge/local_model_sizing.md` (item 5).
   - This makes the item-3 candidate table's VRAM-headroom numbers
     accurate to what the code actually requests, instead of assuming a
     "modest" `num_ctx` that no commit set. See item 3 for the redone
     arithmetic and the general method for re-deriving it on other
     hardware.
6. **Per-chunk request**: send each chunk through the existing
   `build_body`/`send_request` machinery (reasoning retry, timeout, HTTP-code
   handling, `keep_alive` — see item 4 below) with `num_ctx` from step 5
   substituted per-request, and the prompt template noting "reviewing chunk
   N of M — file `<path>`" so the model doesn't need cross-chunk context it
   doesn't have.
7. **Per-chunk failure stays loud**: if any chunk times out, gets a non-200,
   or fails the truncation/`done_reason` checks, the whole review aborts
   with that chunk's specific error — a file that failed to review is never
   silently dropped from the findings list. (No partial-results-on-error
   mode; that would let a failure masquerade as "no findings" for the
   skipped file.)
8. **Synthesis = deterministic merge, not a second LLM call.** Concatenate
   each chunk's returned findings under a `### <file path>` sub-heading and
   renumber findings 1..N globally in the final `## Local Adversarial
   (<model>)` output. Chunks are non-overlapping by file, so cross-chunk
   duplicate findings are not expected; a second synthesis LLM pass would
   double the token/time cost with no demonstrated need. Document this as a
   deliberate v1 tradeoff — revisit with an LLM synthesis pass only if
   manual review of chunked output shows real cross-chunk noise (e.g. the
   same systemic issue re-reported once per file in a way that drowns out
   distinct findings).
9. **Document the cross-file blind spot as a known v1 limitation
   (closes the Plan Review suggestion).** Per-file (and per-hunk) chunking
   means the model reviewing chunk N never sees any other chunk — it
   cannot catch a defect that spans files or depends on a caller/callee
   relationship split across chunks (e.g. a signature change in file A
   whose now-mismatched caller sits in file B; an invariant maintained in
   one file and broken in another). This is a systematic blind spot, not
   an edge case, and it is distinct from the duplicate-finding cost
   discussed in step 8. State it explicitly in two places so a reader of
   a chunked review's "No findings" output does not mistake silence for
   an all-clear on cross-file defects: (a) the `local_review.sh` header
   comment block, next to the existing chunking/env-var documentation,
   and (b) `.agent/knowledge/local_model_sizing.md` (item 5), framed as a
   known limitation of the chunked-review mode specifically — 5f's
   cross-file blind spot does not affect `review-code`'s other specialists,
   which retain full-repo/full-diff context.

This item does not depend on which model ships as default and should be
usable today with `qwen3.5:35b` too (it just makes each request smaller and
faster, in addition to enabling smaller-context models).

### 2. Per-request `keep_alive` (host-independent — Commit 2)

Add `keep_alive` to `build_body()`'s JSON payload (`local_review.sh:222-231`)
rather than `OLLAMA_KEEP_ALIVE` in the environment, so the workspace never
reaches into the operator's machine-wide Ollama config (matches the existing
`LOCAL_REVIEW_*` per-invocation override pattern, and confirmed above that
`keep_alive` genuinely is a per-request field in 0.32.0).

- New env var `LOCAL_REVIEW_KEEP_ALIVE` (default `"30s"`) documented in the
  header comment block alongside the existing `LOCAL_REVIEW_*` vars.
- Rationale for the default: chunked reviews make multiple sequential
  requests to the same model (item 1), so unloading immediately after each
  chunk (`keep_alive: "0"`) would force a reload per chunk — expensive on a
  23 GB (or even a right-sized 4-6 GB) model. `30s` keeps the model warm
  across a chunk sequence without squatting in RAM for the Ollama default
  `5m` after the whole review finishes.

### 3. Model tag selection (this-host tuning — Commit 3, after empirical check)

**Budget arithmetic** (8 GB VRAM total budget; ~0.5 GB reserved for
CUDA/driver overhead → ~7.5 GB usable for weights + KV cache).

**Arithmetic method** (so it can be re-derived on different hardware or a
different model tag, rather than trusted as a fixed table):

1. `weights_GB` — read from the registry (`ollama show <tag>`) or measured
   VRAM at load; this is the only number in the table below actually
   sourced from the registry.
2. `kv_bytes_per_token = 2 (K and V) × num_layers × num_kv_heads ×
   head_dim × bytes_per_element` — read `num_layers`, `num_kv_heads`,
   `head_dim` from `ollama show <tag> --verbose` (or the GGUF metadata)
   for the *specific candidate tag*, not assumed from a different model
   size in the same family. `bytes_per_element` is 2 for the default
   `f16` KV cache (`OLLAMA_KV_CACHE_TYPE` unset — item 4 is not applied by
   this plan) or 1 for `q8_0` (only relevant if item 4's drop-in is later
   approved and applied).
3. `kv_GB(num_ctx) = kv_bytes_per_token × num_ctx / 2^30`.
4. `headroom_GB(num_ctx) = 7.5 − weights_GB − kv_GB(num_ctx)`.

The `num_ctx` plugged into step 3/4 is **not** the 32768 default — it is
the per-chunk bucket item 1 step 5 will actually request for that
candidate (bucket set `8192 16384 24576 32768`, chosen from the
candidate's measured `ANSWER_HEADROOM` + a typical single-file chunk's
token count). This is the specific gap the Plan Review flagged: without
item 1 step 5's code change, every request — chunked or not — would still
request the 32768 default and the table below would be arithmetic fiction.

| Tag | Weights | `num_ctx` assumed (bucket, from item 1 step 5) | Headroom after weights + KV cache | Fits fully GPU-resident? |
|---|---|---|---|---|
| `qwen3.5:35b` (current default) | 23 GB | 32768 (no per-chunk shrink helps — weights alone exceed budget) | negative | No — CPU spill (today's OOM path) |
| `qwen3.5:9b-q4_K_M` | 6.6 GB | 16384 (smallest bucket ≥ measured headroom + typical single-file chunk, pending item 3 step 1's measurement) | ~0.9 GB, **order-of-magnitude — exact `kv_bytes_per_token` for this tag not yet read off `ollama show --verbose`; this is a placeholder pending step 1 of the empirical check, not a verified number** | Tight — only clears the gate if the measured KV-cache size at bucket 16384 actually fits; disqualified outright if not, per the standing "smaller model that runs beats a bigger one that OOMs" rule |
| `qwen3.5:4b-q8_0` | 5.3 GB | 16384 (same bucket reasoning as above, smaller model → likely shorter reasoning → plausibly fits the 8192 bucket instead, to be confirmed) | ~2.2 GB at 16384, more if 8192 clears | Yes, comfortably, even at default `f16` KV cache |
| `qwen3.5:4b` | 3.4 GB | 16384 (same bucket reasoning) | ~4.1 GB | Yes, most headroom |
| `qwen3.5:2b` / `qwen3.5:0.8b` | 2.7 GB / 1.0 GB | 8192-16384 | large | Yes, but quality risk — not carried forward as primary candidates given 5f's adversarial-review role |

The arithmetic above is a **first-pass filter**, not the acceptance gate —
weights-on-disk size isn't the same as loaded VRAM footprint (context
buffers, CUDA graph memory, batch buffers add overhead the registry number
doesn't capture), and the `num_ctx` bucket assumptions above are themselves
estimates pending the measurement in the empirical check's step 1 below. The
gate is `ollama ps`
after a real chunked request: `PROCESSOR` column must read `100% GPU` (no
`/CPU` split) and `journalctl -k` must show no OOM kill across a full
review round — this is acceptance criterion 1/2 verbatim, applied
per-candidate during selection, not just post-hoc.

**Candidates carried to the empirical check**: `qwen3.5:4b-q8_0` (primary —
fits with headroom to spare even without the KV-cache-type change) and
`qwen3.5:9b-q4_K_M` (stretch — only if `4b-q8_0`'s review quality is
measurably worse and `9b` clears the VRAM gate at chunked, non-88k-token
`num_ctx`).

**Empirical quality check (planted-defect diff)**:

1. **Measure, per candidate, before scoring quality**: run each candidate
   (`qwen3.5:9b-q4_K_M`, `qwen3.5:4b-q8_0`) through `local_review.sh` on a
   single representative file-sized chunk and record (a) its actual
   reasoning-token count (replaces the `qwen3.5:35b`-derived
   `ANSWER_HEADROOM=12288` assumption for that candidate), (b) the
   `num_ctx` bucket item 1 step 5 selects for that chunk, and (c) `ollama
   ps`'s reported VRAM/`PROCESSOR` split at that bucket. This is what
   turns the item-3 table's placeholder headroom numbers into verified
   ones — fill in the table (or note the correction) before proceeding to
   recall scoring, since a candidate that fails this step is disqualified
   before spending time on steps 2-4 below.
2. Construct a synthetic diff fixture in
   `.agent/scripts/tests/fixtures/local_review_planted_defects.diff`
   (new file) embedding a fixed, enumerated set of known defects
   spanning the categories `local_review.sh`'s own prompt asks for: an
   off-by-one, a race condition (unsynchronized shared state), a resource
   leak (unclosed file/socket), a silently-swallowed exception, and a
   misleading comment that contradicts the code below it. Document the
   defect list (file, line, what it is) in a companion
   `local_review_planted_defects.md` so scoring is objective, not
   re-adjudicated each run.
3. Run each candidate model through `local_review.sh` (post-chunking, with
   per-chunk `num_ctx` from item 1 step 5) against the fixture, on the dev
   host, with `journalctl -k` monitored for OOM.
4. Score recall (planted defects the model actually flagged, matched by file
   + approximate line) and false-positive rate (findings that don't
   correspond to a planted defect — informational only, not a gate, since a
   real defect-finder legitimately surfaces things we didn't plant).
5. Selection rule: prefer the smaller/faster candidate (`4b-q8_0`) unless its
   recall is materially worse than `9b-q4_K_M`'s (no fixed numeric bar is set
   here — this is a judgment call for whoever runs the check, recorded with
   its reasoning, not a pass/fail threshold invented in advance). A model
   that OOMs during the check is disqualified regardless of recall — "smaller
   model that runs beats a bigger one that's OOM-killed" is the standing
   principle from the issue.
6. Record the result (chosen tag, measured `ANSWER_HEADROOM` and `num_ctx`
   bucket from step 1, recall/false-positive counts, VRAM behavior
   observed) in `.agent/knowledge/local_model_sizing.md` (new file — see
   item 5) — not just progress.md, since this is a reusable pattern for
   any 8GB-class dev host, not a one-off decision log entry.

Change `MODEL="${LOCAL_REVIEW_MODEL:-qwen3.5:35b}"` (`local_review.sh:76`) to
the chosen tag, and update the header doc comment's default-model reference
and the `num_ctx` rationale paragraph (which currently justifies 32768 by
citing `qwen3.5:35b`'s ~7k-token reasoning on a 100-line diff — a smaller
model's reasoning-token behavior needs re-measuring, not assumed to carry
over).

### 4. `OLLAMA_KV_CACHE_TYPE` systemd drop-in (this-host tuning — NOT authorized by this plan)

Per the investigation above, no per-request path exists. If, after item 3's
empirical check, VRAM headroom is still tight (e.g. `9b-q4_K_M` is chosen and
needs the ~2x KV-cache saving to clear the gate), write an **idempotent setup
script** — e.g. `.agent/scripts/setup_ollama_kv_cache.sh` — that:

- Writes a systemd drop-in (`/etc/systemd/system/ollama.service.d/kv-cache.conf`
  or equivalent) setting `Environment=OLLAMA_KV_CACHE_TYPE=q8_0`.
- Is idempotent (safe to re-run; checks current state before writing).
- Prints what it's about to do and requires explicit confirmation (or a
  `--yes` flag) before touching `/etc` — this is host administration on the
  operator's own dev workstation, which AGENTS.md's hard "Never" doesn't
  cover (that's scoped to remote/field hosts), but it's still an unattended
  `/etc` write with sudo and needs the operator's explicit, specific
  sign-off, not inferred from approving this plan generally.
- Documents the change in `.agent/knowledge/local_model_sizing.md` so a
  rebuilt or second dev host can be brought to the same state
  (enforcement-over-documentation: a script beats a prose-only runbook step).

**This plan writes the script but does not run it.** Whether it's needed at
all depends on item 3's outcome — if `4b-q8_0` clears the empirical check
without it, this item may turn out to be unnecessary and can be dropped
entirely rather than applied speculatively. Either way: **stop and ask the
operator before executing the drop-in**, per the standing decision.

### 5. Durable record of decisions

New file `.agent/knowledge/local_model_sizing.md` capturing:

- The VRAM budget arithmetic method (weights + KV cache + ~0.5 GB overhead,
  gated by `ollama ps` PROCESSOR=100%GPU + no-OOM, not registry size alone)
  as a reusable pattern for any 8GB-class (or other VRAM-constrained) dev
  host — this failure mode will recur. Include the step-by-step method from
  item 3 (`kv_bytes_per_token` from `ollama show --verbose`, bucket-based
  `num_ctx` selection, `headroom_GB` formula) so it is re-derivable for a
  future model tag or a different VRAM budget, not just the specific
  numbers measured for this host.
- The chosen model tag and why, plus the planted-defect recall/false-positive
  numbers and the measured `ANSWER_HEADROOM`/`num_ctx` bucket from item 3's
  check.
- Whether the `OLLAMA_KV_CACHE_TYPE` drop-in was needed/applied, and on which
  host(s), for reproducing the state on a rebuild.
- The known v1 limitation from item 1 step 9: per-file/per-hunk chunking
  cannot see cross-file or caller/callee-dependent defects — "no findings"
  from a chunked 5f run is not a clean bill of health for those cases.

Add the new `setup_ollama_kv_cache.sh` script (if item 4's script is written)
to `AGENTS.md`'s script reference table, per the consequences map — new
scripts belong there.

### 6. Re-triage #590; leave #585 alone

- Once this issue's acceptance criteria are met (5f completes without OOM at
  PR-#350 scale, quality floor checked), re-triage
  [#590](https://github.com/rolker/ros2_agent_workspace/issues/590) (make 5f
  opt-in) — its premise (5f is too unreliable to be default-on) is resolved.
  Likely disposition: close as superseded by #605, with a comment
  cross-linking. This is a follow-up action after acceptance, not part of
  this plan's implementation — noting it here so it isn't dropped.
- [#585](https://github.com/rolker/ros2_agent_workspace/issues/585)
  (container-dispatched reviews can't reach host Ollama over the Docker
  bridge network) is **independent** — a connectivity problem, not a
  capacity/context problem. This plan does not touch it, and completing
  #605 does not resolve it. (The issue body's original "#585 is downstream
  of this" framing was already corrected during Issue Review; this plan
  carries the corrected framing forward.)

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/local_review.sh` | Chunking (split/per-chunk-budget/merge), **per-chunk `num_ctx` bucket sizing**, per-request `keep_alive`, new default model tag, updated header doc comments (env vars, `num_ctx`/`ANSWER_HEADROOM` rationale, **cross-file blind-spot limitation note**) |
| `.agent/scripts/tests/test_local_review.sh` (new) | Hermetic tests for chunking + **per-chunk `num_ctx` bucket selection** + keep_alive (see below) |
| `.agent/scripts/tests/fixtures/local_review_planted_defects.diff` (new) | Synthetic diff with known planted defects, for the empirical quality check |
| `.agent/scripts/tests/fixtures/local_review_planted_defects.md` (new) | Enumerated defect list + line numbers, for objective scoring |
| `.agent/knowledge/local_model_sizing.md` (new) | VRAM sizing method, chosen tag + rationale, quality-check result, drop-in status |
| `.agent/scripts/setup_ollama_kv_cache.sh` (new, conditional on item 4 being needed) | Idempotent systemd drop-in writer — written but not executed by this plan |
| `AGENTS.md` | Script reference table: add `setup_ollama_kv_cache.sh` if item 4 lands; update `local_review.sh` row's default-model note (currently says "default `qwen3.5:35b`") |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | 4 changes sequenced as 3 host-independent commits (chunking, keep_alive, this-plan documentation) plus one this-host commit (model default) that depends on the empirical check; the systemd drop-in is written but gated separately and may not land at all. |
| Human control and transparency | Systemd drop-in requires explicit operator sign-off before execution — this plan does not authorize applying it, matching the operator's stated decision. The investigation confirming no per-request KV-cache override exists is recorded above so the "why not just do it per-request" question doesn't recur. |
| Enforcement over documentation | The drop-in, if needed, ships as an idempotent script, not a runbook step. |
| Capture decisions, not just implementations | New `.agent/knowledge/local_model_sizing.md` records the VRAM arithmetic method, chosen tag rationale, and planted-defect check result — durable, not just a progress.md line that evaporates once the issue closes. |
| A change includes its consequences | `AGENTS.md` script table updated if the setup script lands; header doc comments in `local_review.sh` updated in the same commits that change the behavior they describe. |
| Test what breaks | New `test_local_review.sh` covers chunking (split boundaries, per-chunk budget, per-chunk `num_ctx` bucket selection, irreducible-hunk failure, merge/renumbering) and `keep_alive` payload construction — auto-discovered by `run_script_tests.sh`'s glob, no Makefile change needed. |
| Workspace vs. project separation | Workspace-only change (`.agent/scripts/`, `.agent/knowledge/`). |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0001 — Adopt ADRs | No | Model-tag/tuning choice is operational, not architectural; recorded in `.agent/knowledge/` per the plan, not an ADR. |
| 0004/0005 — Enforcement hierarchy | Yes (partial) | Drop-in as a script (enforcement) rather than prose-only doc, per item 4 — but execution itself stays a manual, approved step (can't be auto-enforced without violating the "ask first" decision). |
| 0009 — Python packaging | No | Not applicable — no Python packages touched. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `local_review.sh`'s prompt-building/chunking logic | Header doc comment block (env vars, `num_ctx` rationale, chunking behavior) | Yes — item 1/3 |
| `local_review.sh`'s default model | Header doc comment's default-model reference; `AGENTS.md` script table row | Yes — item 3/5 |
| Add `setup_ollama_kv_cache.sh` | `AGENTS.md` script reference table | Yes, conditionally — item 5 |
| Chunking changes what counts as "context overflow" | Loud-failure guard semantics (now per-hunk, not per-diff) | Yes — item 1, explicitly re-derived, not just carried over |
| Chunking's `num_ctx` becomes per-chunk instead of global | The item-3 VRAM headroom arithmetic (must reflect the actual bucket requested, not the 32768 default) | Yes — item 1 step 5 (code) + item 3 (arithmetic redone against it) |
| Per-file/per-hunk chunking loses cross-file review context | Documented as a known v1 limitation, not silently shipped | Yes — item 1 step 9, script header + `.agent/knowledge/local_model_sizing.md` |
| 5f becomes reliable | #590 (make 5f opt-in) re-triage | Yes — item 6, as a follow-up action, not implementation scope |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): `local_review.sh`'s own header
  comment block (env var list, default model, `num_ctx`/`ANSWER_HEADROOM`
  rationale — now per-chunk-bucket-based, not a single global value) goes
  stale the moment chunking or the model default changes — update in the
  same commits, not as an afterthought. The header must also gain the new
  cross-file-blind-spot limitation note (item 1 step 9) — this is new
  content the diff itself introduces (chunking didn't exist before), so it
  lands as part of this PR, not as a proposal. `AGENTS.md`'s
  `local_review.sh` script-table row currently says "default `qwen3.5:35b`"
  — update when item 3 lands.
- **Agent-instruction candidates** (proposals only — operator decides): a
  `.agent/knowledge/local_model_sizing.md` note (item 5) on sizing local
  models against VRAM-constrained dev hosts — including the re-derivable
  arithmetic method and the cross-file blind-spot limitation — framed as a
  reusable pattern for any future 8GB-class (or smaller) host, not just
  this one. This is a new knowledge doc being proposed, not an existing
  one being corrected — operator can decline or ask for a different
  location.

## Open Questions

- Does `qwen3.5:4b-q8_0` clear the planted-defect recall check well enough to
  ship as the sole default, or does the stretch candidate (`9b-q4_K_M`, which
  needs the KV-cache-type change) turn out to be necessary for acceptable
  review quality? This determines whether item 4 (systemd drop-in) is even
  needed — cannot be answered without running the empirical check on the dev
  host during implementation.
- Item 3's candidate table's exact headroom numbers are placeholders pending
  step 1 of the empirical check (reading `kv_bytes_per_token` off `ollama
  show --verbose` for each candidate tag and measuring its actual reasoning
  length). If the measured KV-cache size at the selected bucket is larger
  than the placeholder implies, `9b-q4_K_M` may not clear the gate even with
  chunking — in which case `4b-q8_0` (or a smaller tag) becomes the only
  viable candidate, and item 4's drop-in becomes moot regardless of quality
  preference. Cannot be resolved without running the measurement.
- The `num_ctx` bucket set (`8192 16384 24576 32768`) in item 1 step 5 is a
  first proposal, not a measured optimum — it may need adjusting once real
  per-chunk token counts and reasoning lengths are observed during
  implementation (e.g. if most single-file chunks cluster well under 8192,
  a smaller bottom bucket saves more VRAM than proposed here).
- If the drop-in does turn out to be needed: operator sign-off on applying
  `setup_ollama_kv_cache.sh` is a separate ask at implementation time, not
  covered by approval of this plan.
- Is `LOCAL_REVIEW_KEEP_ALIVE` default of `30s` right, or should it be
  shorter/longer given the chunked request sequence's actual per-chunk
  latency? No strong evidence either way yet — flagged as a value worth
  revisiting after the first real chunked run's timing is observed.

## Estimated Scope

Single PR, multiple atomic commits on `feature/issue-605`:
1. Chunking + tests (host-independent correctness)
2. Per-request `keep_alive` + tests (host-independent)
3. Model default change, pending the empirical planted-defect check
   (this-host tuning)
4. `.agent/knowledge/local_model_sizing.md` + doc updates
5. (Conditional, may be dropped) `setup_ollama_kv_cache.sh`, written but not
   executed — execution is out of scope for this PR pending separate
   operator approval

No breakdown into separate PRs needed — AGENTS.md's filing discipline
favors bundling related changes with atomic commits over fanning out
issues, and the host-independent chunking commit landing first means a
partial merge (if host-tuning stalls on quality-check results or operator
approval) still leaves the workspace strictly better off.
