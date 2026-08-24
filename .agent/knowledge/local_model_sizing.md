# Sizing a local model against a VRAM-constrained host

How to pick a model tag and context budget for
[`.agent/scripts/local_review.sh`](../scripts/local_review.sh) (review-code
specialist 5f) on a host whose GPU cannot hold a large model. Written from
[#605](https://github.com/rolker/ros2_agent_workspace/issues/605), where
the default model was 23 GB against an 8 GB card, but the method is the
point — this failure mode recurs on any 8 GB-class box, and the numbers
below are this host's, not universal.

## The failure mode

Ollama does not refuse a model that does not fit. It loads what fits in
VRAM and spills the rest to system RAM, then runs. On the dev host
(deadpool, RTX 2080 Super, 8 GB VRAM, 31 GB RAM) `qwen3.5:35b` loaded at
24 GB with `PROCESSOR` reading `76%/24% CPU/GPU`, filled RAM and swap, and
`llama-server` was OOM-killed four times in seven days — taking unrelated
processes (`chrome`, a running simulator) with it, because the kernel OOM
killer does not respect whose fault it is.

The result was not a slow reviewer. It was no reviewer: specialist 5f
produced zero findings across four review rounds.

Two knobs pull against each other, which is why this needs arithmetic
rather than a guess:

- A **bigger model** needs more VRAM for weights.
- A **bigger `num_ctx`** needs more VRAM for the KV cache, and a large
  diff appears to demand one.

Chunking the diff (see the script's header) removes the second pressure.
This document covers the first.

## The arithmetic — first-pass filter only

```
usable_GB      = total_VRAM_GB - ~0.5   # CUDA context, driver, display
kv_bytes_token = 2 (K and V) x n_full_attention_layers
                   x n_kv_heads x head_dim x bytes_per_element
kv_GB(num_ctx) = kv_bytes_token x num_ctx / 2^30
headroom_GB    = usable_GB - weights_GB - kv_GB(num_ctx)
```

- `weights_GB` — from `ollama show <tag>` (the registry size).
- `n_full_attention_layers`, `n_kv_heads`, `head_dim` — from
  `ollama show <tag> --verbose`, **for the specific candidate tag**, never
  assumed from another size in the same family.
- `bytes_per_element` — 2 for the default `f16` KV cache, 1 for `q8_0`
  (see the drop-in section below).

Two traps worth naming:

- **`num_ctx` here is the bucket actually requested, not the ceiling.**
  llama.cpp sizes the KV cache to the *requested* `num_ctx`, not to tokens
  consumed. Before #605 the script sent one global `num_ctx` on every
  request, so chunking alone would have saved nothing. `local_review.sh`
  now picks a per-chunk bucket, which is what makes the smaller numbers
  in this table real.
- **Hybrid architectures break the naive layer count.** `qwen3.5` sets
  `full_attention_interval=4`: only every 4th layer is full attention and
  keeps a growing KV cache; the rest are SSM (`ssm.state_size`,
  `ssm.conv_kernel` in the metadata) with fixed-size state. Counting all
  32 blocks overestimates the KV cache roughly 4x. `ollama show --verbose`
  also truncates the `head_count_kv` array in its display, so deriving
  `kv_bytes_token` empirically (below) is more reliable than reading it
  off the metadata.

**Registry size is not loaded footprint.** Context buffers, CUDA graph
memory and batch buffers add overhead the registry number does not
capture. Treat the arithmetic as a filter that rules candidates *out*, and
measure to rule one *in*.

### Deriving `kv_bytes_token` by measurement

More robust than the metadata, and only takes two requests: load the model
at two different `num_ctx` values and difference the reported size.

```bash
# request at num_ctx N, then:
ollama ps   # read the SIZE column while the model is resident
```

For `qwen3.5:4b-q8_0` on this host:

| `num_ctx` | `ollama ps` SIZE | `nvidia-smi` used | PROCESSOR |
|---|---|---|---|
| 16384 | 5.5 GB | 6272 MiB | 100% GPU |
| 24576 | 5.8 GB | 6536 MiB | 100% GPU |
| 32768 | 6.1 GB | — | 100% GPU |

Linear, ~0.6 GB per 16384 tokens → **~39 KB/token**. Against 5.3 GB of
weights and ~7.5 GB usable, even the top bucket leaves over 1 GB spare.

## The acceptance gate is measured, not calculated

A candidate is accepted only when, on a **real chunked review run**:

1. `ollama ps` shows `PROCESSOR` = `100% GPU`, with no `/CPU` split; and
2. `journalctl -k` shows no new OOM kill across the whole run; and
3. it actually answers — `done_reason` is `stop`, not `length`; and
4. the planted-defect score is recorded (below).

Standing principle from #605: **a smaller model that runs beats a bigger
one that is OOM-killed.** Its converse holds too — a model that reasons
until the window is exhausted and returns an empty answer fails just as
hard as one that is killed.

## Answer headroom must be measured, and does not track model size

`LOCAL_REVIEW_ANSWER_HEADROOM` is the tokens reserved in every request for
the model's reasoning plus its answer. It is added into the same estimate
that selects the `num_ctx` bucket, so it directly determines which buckets
are reachable — a headroom above a rung makes that rung unselectable.
`local_review.sh` prunes such rungs rather than advertising them, so the
ladder never contains a dead entry at any headroom.

Measure it by sending one representative single-file chunk and reading
`eval_count` (thinking + answer tokens) off the response:

```bash
curl -s -H 'Content-Type: application/json' -d @body.json \
     http://localhost:11434/api/chat \
  | jq '{prompt_eval_count, eval_count, done_reason}'
```

**Do not assume a smaller model reasons less.** On one ~180-line source
file:

| Model | Generated tokens (thinking + answer) |
|---|---|
| `qwen3.5:35b` | ~7000 |
| `qwen3.5:4b-q8_0` | **14835** |

The 4b model is more than twice as verbose as the 35b. At `num_ctx=16384`
it exhausted the window mid-reasoning and returned `done_reason: length`
with `content` empty — a silent-looking non-answer. So the shipped
headroom went **up** to 16384 for the smaller model, not down. Parameter
count does not predict verbosity; measure the tag you intend to ship.

### Bucket coarseness is a deliberate tradeoff

Requests ask for a rung of a fixed ladder (`8192 16384 24576 32768`,
filtered to what is reachable and capped at `LOCAL_REVIEW_NUM_CTX`) rather
than a value computed to the token, so the server sees a handful of
distinct KV-cache allocation shapes instead of one per request. A chunk
just over a boundary rounds up — as much as ~1.5x at the low end. That
waste is accepted rather than tuned away: the acceptance gate above
measures the real allocation, so any overrun surfaces directly instead of
depending on the arithmetic being tight.

## Result for this host (2026-08-24)

**Chosen: `qwen3.5:4b-q8_0`** (5.3 GB weights), headroom 16384.

Planted-defect check against
[`.agent/scripts/tests/fixtures/local_review_planted_defects.diff`](../scripts/tests/fixtures/local_review_planted_defects.diff)
(5 defects; scoring method in the companion `.md`):

| Planted defect | Found? |
|---|---|
| Resource leak (unclosed file handle) | Yes — correct, well-reasoned |
| Race on unsynchronized shared state | Yes — correct |
| Off-by-one slice (`end + 1`) | Yes — identified, reasoning muddled |
| Misleading comment | Flagged, but with an incorrect rationale |
| Silently swallowed exception (`except Exception: pass`) | **No — missed entirely** |

Three further findings did not correspond to planted defects; one was
substantially confabulated.

**This is not a clean quality bar, and it should not be read as one.**
Missing the swallowed exception is the worst of the five to miss in a
workspace whose standard is that silent failures are unacceptable, and the
confabulation means 5f's output needs reading with judgement rather than
trust. It was adopted anyway because the comparison is not against a
perfect reviewer: the incumbent found nothing at all, four rounds running,
because it never completed. Treat 5f as a cheap extra pair of eyes whose
misses are expected — not as a gate.

`qwen3.5:9b-q4_K_M` (6.6 GB), the stretch candidate, was **not
evaluated**: the root filesystem is 100% full with ~7 GB free, and the
pull would have taken it to zero. Worth re-checking when disk allows —
the VRAM arithmetic above says it would fit at the 24576 bucket, and it
may well score better.

## Known limitation of the chunked mode — cross-file blind spot

Chunking reviews each file (or hunk group, or line range) in isolation, so
the model never sees any other chunk. It **cannot** catch a defect that
spans files or depends on a caller/callee relationship split across
chunks: a signature change in file A whose now-mismatched caller sits in
file B, an invariant maintained in one file and broken in another,
duplicated logic introduced across two files.

This is systematic, not an edge case. **"No findings." from a chunked 5f
run is not a clean bill of health for cross-file defects.** The script
says so in its own header and in the chunked output itself. review-code's
other specialists retain full-diff/full-repo context and are unaffected —
5f is the only one with this blind spot.

## `OLLAMA_KV_CACHE_TYPE` — available, not applied

Halving the KV cache (`f16` → `q8_0`) is a real lever, but **Ollama 0.32.0
has no per-request override**. Verified against the installed binary:
`ollama serve --help` lists `OLLAMA_KV_CACHE_TYPE` only as a server
environment variable; the `--cache-type-k` / `--cache-type-v` strings in
the binary are `llama-server` CLI flags the runner derives from its own
environment at subprocess launch; and no `kv_cache_type` JSON tag exists
anywhere in the binary — unlike `keep_alive` and `num_ctx`, which
`local_review.sh` does send per request. KV-cache quantization is fixed at
model-load time by the server process's environment.

So it needs a systemd drop-in, which is a sudo write under `/etc` on the
operator's own workstation.
[`.agent/scripts/setup_ollama_kv_cache.sh`](../scripts/setup_ollama_kv_cache.sh)
exists for that (idempotent, dry-run by default, refuses to act without an
explicit `--yes`), **but it has not been run on any host.** It was not
needed: `qwen3.5:4b-q8_0` clears the gate with over 1 GB spare at the top
bucket on the default `f16` cache. Reach for it only if a future candidate
needs the headroom, and get the operator's specific sign-off first —
approving a plan that mentions the script is not approval to execute it.

| Host | Drop-in applied? |
|---|---|
| deadpool (dev) | No |

## Model residency

`local_review.sh` sends `keep_alive` as a per-request field (default
`30s`, `LOCAL_REVIEW_KEEP_ALIVE`) rather than setting a global
`OLLAMA_KEEP_ALIVE`. A chunked review makes several sequential requests to
the same model, so unloading immediately would force a reload per chunk;
`30s` keeps it warm across the sequence without squatting in RAM for
Ollama's 5m default once the review has finished. Scoping it per request
keeps the workspace out of the operator's machine-wide Ollama config.

## Checklist for a new host or a new tag

1. `nvidia-smi` → total VRAM; subtract ~0.5 GB.
2. `ollama show <tag>` → weights; rule out anything close to the budget.
3. `ollama show <tag> --verbose` → attention layout; watch for a hybrid.
4. Pull one candidate at a time and check free disk first — these are
   multi-GB pulls and a full root filesystem breaks far more than Ollama.
5. Measure `eval_count` on one representative chunk → set
   `LOCAL_REVIEW_ANSWER_HEADROOM` above it (round up to a rung).
6. Run a real chunked review; check `ollama ps` (100% GPU) and
   `journalctl -k` (no OOM) *during* it, not after.
7. Score the planted-defect fixture; record the misses honestly.
8. Record the outcome here — including a candidate that failed and why.
