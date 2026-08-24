#!/bin/bash
# Local Model Adversarial Review
# Sends a diff to a locally served Ollama model and prints its review
# findings to stdout. Used by the review-code skill (specialist 5f) and
# usable standalone — including offline / field mode, where the
# GitHub- and Claude-based review pipeline is unavailable.
#
# Usage:
#   git diff main...HEAD | local_review.sh            # diff on stdin
#   local_review.sh --base main                        # diff computed here
#   local_review.sh --base main --context ctx.txt      # extra task context
#   local_review.sh --base main --plan                 # show the chunk plan only
#
# Chunking (why the script does not send one big prompt):
#   No single-GPU context window swallows a real PR-sized diff (an 88k-token
#   branch diff against a 32k window is routine), and raising num_ctx to fit
#   one inflates the KV cache until the model spills to CPU and gets
#   OOM-killed. So the diff is split into chunks that each fit:
#     1. per file, on `diff --git` boundaries;
#     2. if one file alone still overflows, per group of consecutive hunks
#        on `@@` boundaries, each group re-carrying the file header;
#     3. if a single hunk alone still overflows (a large added file is one
#        hunk), per line range within that hunk, each part re-carrying the
#        file header and a banner naming the part.
#   Every level is a lossless split: every diff line lands in exactly one
#   chunk. Only a single *line* too large for the window is irreducible,
#   and that FAILS LOUD naming the file and hunk — it is never silently
#   truncated.
#   Each chunk is sent as its own request, with its own `options.num_ctx`
#   picked as the smallest bucket that fits that chunk (see
#   LOCAL_REVIEW_ANSWER_HEADROOM below): a small chunk allocates a small KV
#   cache instead of always paying for the ceiling. Any chunk that times
#   out, errors, or comes back truncated aborts the whole review — a file
#   that failed to review is never silently dropped from the findings.
#   Findings are merged deterministically (concatenated under a per-file
#   heading and renumbered), not by a second LLM synthesis pass.
#
# KNOWN LIMITATION — cross-file blind spot:
#   Because each chunk is reviewed in isolation, the model never sees any
#   other chunk. It therefore CANNOT catch a defect that spans files or
#   depends on a caller/callee relationship split across chunks: a
#   signature change in file A whose now-mismatched caller sits in file B,
#   an invariant maintained in one file and broken in another, logic
#   duplicated across two files. This is a systematic blind spot of the
#   chunked mode, not an edge case. "No findings." from this script is not
#   a clean bill of health for cross-file defects. review-code's other
#   specialists retain full-diff/full-repo context and are unaffected.
#
# Environment:
#   LOCAL_REVIEW_MODEL    model tag (default: qwen3.5:35b)
#   LOCAL_REVIEW_URL      Ollama base URL (default: http://localhost:11434)
#   LOCAL_REVIEW_TIMEOUT  per-request timeout seconds (default: 900; scale
#                         up for large chunks — reasoning time grows with
#                         chunk size. Chunking makes each request smaller
#                         than the old whole-diff request, but a chunked
#                         review makes several of them in sequence.)
#   LOCAL_REVIEW_NUM_CTX  context-window CEILING in tokens (default: 32768).
#                         Per-chunk requests ask for the smallest bucket
#                         from 8192/16384/24576/32768 that fits the chunk,
#                         capped at this value; buckets at or below the
#                         answer headroom are dropped as unreachable, and
#                         this ceiling is always itself a bucket. The
#                         script fails loud, with the size it needed, when
#                         even a single indivisible line would overflow
#                         this — Ollama would otherwise truncate silently.
#                         A post-response check on the server's measured
#                         prompt_eval_count backstops the byte-based
#                         estimate per chunk.
#   LOCAL_REVIEW_ANSWER_HEADROOM
#                         tokens reserved in every request for the model's
#                         reasoning + answer (default: 12288). This is a
#                         PER-MODEL value, not a universal constant: the
#                         default is measured for the default model
#                         (qwen3.5:35b thinks for ~7k tokens on even a
#                         100-line diff). A model that reasons in fewer
#                         tokens should lower it — that is what makes the
#                         smaller num_ctx buckets reachable and is the
#                         mechanism by which a right-sized model actually
#                         saves VRAM. See
#                         .agent/knowledge/local_model_sizing.md for how to
#                         measure it for a new tag.
#   LOCAL_REVIEW_KEEP_ALIVE
#                         how long the server keeps the model loaded after
#                         each request (default: 30s), sent as a
#                         per-request field. A chunked review makes
#                         several sequential requests to the same model,
#                         so unloading immediately ("0") would force a
#                         reload per chunk; 30s keeps it warm across the
#                         sequence without squatting in RAM for Ollama's
#                         5m default once the review is done. Set
#                         per-invocation rather than via a global
#                         OLLAMA_KEEP_ALIVE, so this script never reaches
#                         into the operator's machine-wide Ollama config.
#
# Exit codes:
#   0  review produced (findings on stdout)
#   1  invocation error (timeout, HTTP error, empty answer, oversized
#      indivisible diff line, bad configuration)
#   2  unavailable (server down, model not pulled, or jq missing) —
#      callers should treat this as skip-with-notice, not failure
#   Callers must treat any other exit status as 1 (defensive: an
#   unhandled tool failure under `set -e` can surface its own code).
#
# Reasoning models: requests are sent with think:true so the model can
# reason before answering; if the server rejects that for a
# non-reasoning LOCAL_REVIEW_MODEL, the request is retried once with
# think:false, so non-reasoning models work without configuration.
#
# Uses the HTTP API rather than `ollama run`: with reasoning models the
# CLI's thinking handling can swallow the entire answer (observed with
# qwen3.5:35b + --hidethinking on ollama 0.32.0), while /api/chat
# returns thinking and content as separate fields.

if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
    echo "Error: This script should be executed, not sourced." >&2
    echo "  Run: ${BASH_SOURCE[0]} $*" >&2
    return 1
fi

set -euo pipefail

MODEL="${LOCAL_REVIEW_MODEL:-qwen3.5:35b}"
BASE_URL="${LOCAL_REVIEW_URL:-http://localhost:11434}"
TIMEOUT="${LOCAL_REVIEW_TIMEOUT:-900}"
NUM_CTX="${LOCAL_REVIEW_NUM_CTX:-32768}"
ANSWER_HEADROOM="${LOCAL_REVIEW_ANSWER_HEADROOM:-12288}"
KEEP_ALIVE="${LOCAL_REVIEW_KEEP_ALIVE:-30s}"

BASE_BRANCH=""
CONTEXT_FILE=""
PLAN_ONLY=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --base)
            if [[ -z "${2:-}" ]]; then
                echo "Error: --base requires a branch name" >&2
                exit 1
            fi
            BASE_BRANCH="$2"
            shift 2
            ;;
        --context)
            if [[ -z "${2:-}" || ! -r "${2:-}" ]]; then
                echo "Error: --context requires a readable file" >&2
                exit 1
            fi
            CONTEXT_FILE="$2"
            shift 2
            ;;
        --model)
            if [[ -z "${2:-}" ]]; then
                echo "Error: --model requires a model tag" >&2
                exit 1
            fi
            MODEL="$2"
            shift 2
            ;;
        --plan)
            # Print the chunk plan (chunk index, chosen num_ctx bucket,
            # estimated tokens, label) and exit without contacting the
            # server. Useful for sizing a review before paying for it, and
            # the seam the chunking tests drive.
            PLAN_ONLY=true
            shift
            ;;
        -h|--help)
            # Print the header comment block (everything from line 2 to
            # the first non-comment line) so the help text cannot drift
            # from the documentation above as lines are added/removed.
            awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "${BASH_SOURCE[0]}"
            exit 0
            ;;
        *)
            echo "Error: unknown argument: $1" >&2
            exit 1
            ;;
    esac
done

for _var in TIMEOUT NUM_CTX ANSWER_HEADROOM; do
    if ! [[ "${!_var}" =~ ^[0-9]+$ ]]; then
        echo "Error: LOCAL_REVIEW_$_var must be a plain integer (got '${!_var}')" >&2
        exit 1
    fi
done

# --- num_ctx bucket ladder ---
#
# Requests ask for a bucket rather than a value computed to the token, so
# the server sees a handful of distinct KV-cache allocation shapes instead
# of one per request. The ladder is filtered so every bucket in it is
# actually reachable: a bucket at or below ANSWER_HEADROOM can never be
# selected (the headroom alone would fill it), and a bucket above NUM_CTX
# would breach the ceiling. The ceiling itself is always a bucket, so a
# custom LOCAL_REVIEW_NUM_CTX is never rounded past.
#
# Bucket coarseness is a deliberate tradeoff: a chunk just over a boundary
# rounds up to the next bucket (up to ~1.5x at the low end, e.g. 16385 ->
# 24576). The acceptance gate for a model/host pairing is the measured
# `ollama ps` PROCESSOR=100% GPU + no kernel OOM, not this arithmetic, so
# the rounding waste shows up directly in that check rather than needing a
# tighter ladder.
NUM_CTX_LADDER=(8192 16384 24576 32768)
BUCKETS=()
for _b in "${NUM_CTX_LADDER[@]}"; do
    if (( _b > ANSWER_HEADROOM && _b <= NUM_CTX )); then
        BUCKETS+=("$_b")
    fi
done
if (( ${#BUCKETS[@]} == 0 )) || (( BUCKETS[${#BUCKETS[@]} - 1] != NUM_CTX )); then
    if (( NUM_CTX > ANSWER_HEADROOM )); then
        BUCKETS+=("$NUM_CTX")
    fi
fi
if (( ${#BUCKETS[@]} == 0 )); then
    echo "Error: no usable context budget — answer headroom ($ANSWER_HEADROOM tokens) is not smaller than num_ctx ($NUM_CTX);" >&2
    echo "  raise LOCAL_REVIEW_NUM_CTX or lower LOCAL_REVIEW_ANSWER_HEADROOM" >&2
    exit 1
fi

# --- Gather the diff ---

if [[ -n "$BASE_BRANCH" ]]; then
    DIFF=$(git diff --merge-base "$BASE_BRANCH" HEAD) || {
        echo "Error: git diff failed for base '$BASE_BRANCH' (not a ref?)" >&2
        exit 1
    }
elif [[ ! -t 0 ]]; then
    DIFF=$(cat)
else
    echo "Error: pipe a diff on stdin or pass --base <branch>" >&2
    exit 1
fi

if [[ -z "$DIFF" ]]; then
    echo "Error: diff is empty — nothing to review" >&2
    exit 1
fi

# --- Availability probes (exit 2 = skip-with-notice) ---
#
# --plan does no I/O beyond the diff, so it must not require a server.

if ! command -v jq >/dev/null 2>&1; then
    echo "local review unavailable: jq not installed" >&2
    exit 2
fi

if [[ "$PLAN_ONLY" != "true" ]]; then
    if ! curl -sf --max-time 5 "$BASE_URL/api/version" >/dev/null 2>&1; then
        echo "local review unavailable: no Ollama server at $BASE_URL" >&2
        exit 2
    fi

    # Accept both fully-tagged names and the bare form users pass to
    # `ollama run` (Ollama registers untagged pulls as "<name>:latest").
    if ! curl -sf --max-time 10 "$BASE_URL/api/tags" 2>/dev/null \
            | jq -e --arg m "$MODEL" \
                '.models[] | select(.name == $m or .name == ($m + ":latest"))' >/dev/null; then
        echo "local review unavailable: model '$MODEL' not pulled (ollama pull $MODEL)" >&2
        exit 2
    fi
fi

WORK_DIR=$(mktemp -d /tmp/local_review.XXXXXX)
trap 'rm -rf "$WORK_DIR"' EXIT

# --- Split the diff into chunks ---

printf '%s\n' "$DIFF" > "$WORK_DIR/full.diff"

# Anything before the first `diff --git` is not part of any file's diff —
# for `git show` output it is the commit message. Carry it as context on
# every chunk rather than dropping it, so no input is silently lost.
DIFF_PREAMBLE=$(awk '/^diff --git /{exit} {print}' "$WORK_DIR/full.diff")

SEG_DIR="$WORK_DIR/segments"
mkdir -p "$SEG_DIR"
awk -v dir="$SEG_DIR" '
    /^diff --git / { n++; f = sprintf("%s/seg_%05d.diff", dir, n) }
    n > 0 { print > f }
' "$WORK_DIR/full.diff"

shopt -s nullglob
SEGMENTS=("$SEG_DIR"/seg_*.diff)
shopt -u nullglob

if (( ${#SEGMENTS[@]} == 0 )); then
    # No `diff --git` header at all (a hand-rolled or unified-format diff).
    # Treat the whole input as one unnamed segment rather than guessing —
    # the per-chunk overflow guard still applies.
    cp "$WORK_DIR/full.diff" "$SEG_DIR/seg_00001.diff"
    SEGMENTS=("$SEG_DIR/seg_00001.diff")
    DIFF_PREAMBLE=""
fi

CONTEXT=""
if [[ -n "$CONTEXT_FILE" ]]; then
    CONTEXT=$(cat "$CONTEXT_FILE")
fi
if [[ -n "$DIFF_PREAMBLE" ]]; then
    CONTEXT="${CONTEXT:+$CONTEXT

}$DIFF_PREAMBLE"
fi

# Path shown to the model (and used as the findings heading). `+++ b/path`
# is authoritative when present; fall back to the `diff --git` line, then
# to a placeholder for headerless input.
segment_label() {
    local seg="$1" path
    path=$(awk '/^\+\+\+ b\//{sub(/^\+\+\+ b\//,""); print; exit}' "$seg")
    if [[ -z "$path" ]]; then
        path=$(awk '/^diff --git /{print $NF; exit}' "$seg" | sed 's|^b/||')
    fi
    if [[ -z "$path" || "$path" == "/dev/null" ]]; then
        path=$(awk '/^--- a\//{sub(/^--- a\//,""); print; exit}' "$seg")
    fi
    printf '%s' "${path:-(unnamed diff)}"
}

# The full prompt for one chunk. Built identically for estimation and for
# the real request, so the estimate can never drift from what is sent.
build_prompt() {  # $1 = label, $2 = chunk index, $3 = chunk total, $4 = diff body
    local label="$1" idx="$2" total="$3" body="$4"
    cat <<EOF
You are a rigorous code reviewer for a ROS 2 robotics workspace
(autonomous survey boats — robustness is mandatory, silent failures are
unacceptable). Review the following diff.

${CONTEXT:+Task context: $CONTEXT

}Focus areas: missed edge cases and boundary conditions; assumption
violations; subtle bugs (off-by-one, race conditions, resource leaks);
logic errors (does the code do what it claims?); security implications;
concurrency and lifecycle ordering; cross-cutting effects; portability
(GNU vs BSD tools); silent-failure paths; misleading comments.

Rules: review only what the diff changes (lines starting with + or -);
do not flag unchanged context lines. Only report defects you can trace
to specific changed lines — no speculation about environments or inputs
the code visibly does not target. Number the findings. For each give:
file, line(s), why it is wrong, and a concrete failure scenario. Do not
pad with style nits. If you find nothing, say "No findings."

This is chunk $idx of $total of a larger diff, covering $label. The
other chunks are not shown to you; do not speculate about code you
cannot see, and do not ask for it.

DIFF:

$body
EOF
}

est_tokens() {  # $1 = prompt length in characters
    # ~3 chars/token is typical for diff text, not a guaranteed bound —
    # the post-response prompt_eval_count check backstops it.
    echo $(( $1 / 3 + ANSWER_HEADROOM ))
}

pick_bucket() {  # $1 = estimated tokens; prints the bucket, or nothing if none fits
    local est="$1" b
    for b in "${BUCKETS[@]}"; do
        if (( b >= est )); then
            printf '%s' "$b"
            return 0
        fi
    done
    return 1
}

# Characters the prompt costs before any diff body is added, for this
# label. Used only to decide how to group hunks; the authoritative check
# is the per-chunk estimate against the ceiling below.
overhead_chars() {  # $1 = label
    local p
    p=$(build_prompt "$1" 999 999 "")
    printf '%s' "${#p}"
}

CHUNK_BODIES=()   # file paths holding each chunk's diff body
CHUNK_LABELS=()   # human label for each chunk (file, plus hunk range if split)

# Banner prefixed to every continuation part of a split hunk. Its length
# is charged to the part budget, so it is a constant, not built inline.
CONTINUATION_SUFFIX='    [continuation of this hunk; the preceding lines are in the previous chunk]'

emit_chunk() {  # $1 = body file, $2 = label
    CHUNK_BODIES+=("$1")
    CHUNK_LABELS+=("$2")
}

# Split one file segment into as few chunks as fit the ceiling.
#
# Three levels, each used only when the level above still overflows:
#   file -> groups of consecutive hunks -> line ranges within one hunk.
# The third level is still a lossless split, not a truncation: every line
# of the hunk lands in exactly one chunk, each carrying the file header
# and a banner naming which slice of which hunk it is. Only a single
# *line* that alone overflows the ceiling is genuinely irreducible, and
# that fails loud in the sizing pass below.
split_segment() {  # $1 = segment file, $2 = label
    local seg="$1" label="$2" overhead budget seg_len
    overhead=$(overhead_chars "$label")
    # est = (overhead + body)/3 + headroom <= NUM_CTX
    budget=$(( (NUM_CTX - ANSWER_HEADROOM) * 3 - overhead ))
    seg_len=$(wc -c < "$seg")

    if (( budget > 0 && seg_len <= budget )); then
        emit_chunk "$seg" "$label"
        return 0
    fi

    # From here on the chunk labels grow (they gain a hunk/part suffix) and
    # split parts gain a continuation banner. Both go into the prompt, so
    # both must come out of the budget — sizing against the bare label
    # would let a part overshoot the ceiling by exactly the text the split
    # itself added.

    local -a hunk_starts
    mapfile -t hunk_starts < <(grep -n '^@@' "$seg" | cut -d: -f1)
    local total_lines
    total_lines=$(wc -l < "$seg")

    if (( ${#hunk_starts[@]} == 0 )); then
        # No hunks at all (binary file, pure rename, mode change): there is
        # nothing to split on. Emit as-is; the sizing pass fails loud with
        # this file named if it does not fit.
        emit_chunk "$seg" "$label"
        return 0
    fi

    local seg_idx=${#CHUNK_BODIES[@]}
    local header_file
    header_file="$WORK_DIR/header_$(printf '%05d' "$seg_idx")"
    if (( hunk_starts[0] > 1 )); then
        sed -n "1,$(( hunk_starts[0] - 1 ))p" "$seg" > "$header_file"
    else
        : > "$header_file"
    fi
    local header_len
    header_len=$(wc -c < "$header_file")

    local n=${#hunk_starts[@]}
    local out_idx=0

    # Budget for hunk-group chunks, sized against the widest label a group
    # can carry.
    local group_overhead
    group_overhead=$(overhead_chars "$label (hunks $n-$n of $n)")
    # Characters of hunk content that fit alongside the file header.
    local body_budget=$(( (NUM_CTX - ANSWER_HEADROOM) * 3 - group_overhead - header_len ))
    if (( body_budget <= 0 )); then
        # The file header alone does not fit — nothing to gain by splitting.
        emit_chunk "$seg" "$label"
        return 0
    fi

    # Writes one chunk: file header + the given "start,end" sed ranges,
    # optionally preceded by a continuation banner.
    write_chunk() {  # $1 = label suffix, $2 = banner (may be empty), $3.. = ranges
        local suffix="$1" banner="$2"
        shift 2
        out_idx=$(( out_idx + 1 ))
        # Name by position, not by label: a path may contain characters
        # that are not safe in a filename.
        local out
        out="$WORK_DIR/chunk_$(printf '%05d_%05d' "$seg_idx" "$out_idx").diff"
        cat "$header_file" > "$out"
        [[ -n "$banner" ]] && printf '%s\n' "$banner" >> "$out"
        local r
        for r in "$@"; do
            sed -n "${r}p" "$seg" >> "$out"
        done
        emit_chunk "$out" "$label$suffix"
    }

    # A single hunk too big even on its own: split it on line boundaries.
    split_hunk() {  # $1 = hunk number (1-based), $2 = start line, $3 = end line
        local hnum="$1" hstart="$2" hend="$3"
        local hdr banner_probe part_overhead part_budget
        hdr=$(sed -n "${hstart}p" "$seg")
        # Every part but the first also carries a continuation banner, and
        # all parts carry the wider "part i/N" label. Size the budget
        # against both so a part cannot overshoot the ceiling by exactly
        # the text the split added.
        banner_probe="$hdr$CONTINUATION_SUFFIX"
        part_overhead=$(overhead_chars "$label (hunk $n of $n, part $n/$n)")
        part_budget=$(( (NUM_CTX - ANSWER_HEADROOM) * 3 - part_overhead \
                        - header_len - ${#banner_probe} - 1 ))
        if (( part_budget <= 0 )); then
            # Not even the scaffolding fits; emit whole and fail loud below.
            write_chunk " (hunk $hnum of $n)" "" "${hstart},${hend}"
            return 0
        fi
        # Line at which each piece ends. A line longer than the whole
        # budget still gets its own piece — that is the irreducible case
        # the sizing pass rejects loudly.
        local -a piece_ends
        mapfile -t piece_ends < <(awk -v s="$hstart" -v e="$hend" -v budget="$part_budget" '
            NR < s || NR > e { next }
            {
                len = length($0) + 1
                if (acc > 0 && acc + len > budget) { print NR - 1; acc = 0 }
                acc += len
            }
            END { if (e >= s) print e }
        ' "$seg")
        local parts=${#piece_ends[@]}
        local pstart="$hstart" pnum=0 pend
        for pend in "${piece_ends[@]}"; do
            pnum=$(( pnum + 1 ))
            if (( pnum == 1 )); then
                write_chunk " (hunk $hnum of $n, part $pnum/$parts)" "" "${pstart},${pend}"
            else
                write_chunk " (hunk $hnum of $n, part $pnum/$parts)" \
                    "$hdr$CONTINUATION_SUFFIX" \
                    "${pstart},${pend}"
            fi
            pstart=$(( pend + 1 ))
        done
    }

    local i start end hunk_len
    local -a group_ranges=()
    local group_len=0 first_in_group=1

    flush_group() {  # $1 = 1-based index of the last hunk in the group
        (( ${#group_ranges[@]} == 0 )) && return 0
        write_chunk " (hunks $first_in_group-$1 of $n)" "" "${group_ranges[@]}"
        group_ranges=()
        group_len=0
    }

    for (( i = 0; i < n; i++ )); do
        start=${hunk_starts[i]}
        if (( i + 1 < n )); then
            end=$(( hunk_starts[i + 1] - 1 ))
        else
            end=$total_lines
        fi
        hunk_len=$(sed -n "${start},${end}p" "$seg" | wc -c)

        if (( hunk_len > body_budget )); then
            # Does not fit even alone: close any open group, then split it.
            flush_group "$i"
            split_hunk "$(( i + 1 ))" "$start" "$end"
            first_in_group=$(( i + 2 ))
            continue
        fi

        if (( ${#group_ranges[@]} > 0 && group_len + hunk_len > body_budget )); then
            flush_group "$i"
            first_in_group=$(( i + 1 ))
        fi
        group_ranges+=("${start},${end}")
        group_len=$(( group_len + hunk_len ))
    done
    flush_group "$n"
}

for seg in "${SEGMENTS[@]}"; do
    split_segment "$seg" "$(segment_label "$seg")"
done

TOTAL_CHUNKS=${#CHUNK_BODIES[@]}
if (( TOTAL_CHUNKS == 0 )); then
    echo "Error: diff produced no reviewable chunks" >&2
    exit 1
fi

# --- Size every chunk, failing loud on an indivisible overflow ---
#
# This is the same loud-failure guarantee the whole-diff check gave, moved
# down to the smallest splittable unit. It is strictly stronger: an
# oversized 40k-token *diff* is now routine and chunked, while a single
# *line* too large for the window is a genuinely different (and much
# rarer) problem that still refuses to run rather than letting Ollama
# truncate silently. Nothing below this point can shrink the input, so a
# chunk that does not fit here is reported, not sent.

CHUNK_PROMPTS=()
CHUNK_CTX=()
CHUNK_EST=()

for (( c = 0; c < TOTAL_CHUNKS; c++ )); do
    label="${CHUNK_LABELS[c]}"
    body=$(cat "${CHUNK_BODIES[c]}")
    prompt=$(build_prompt "$label" "$(( c + 1 ))" "$TOTAL_CHUNKS" "$body")
    est=$(est_tokens "${#prompt}")
    if ! bucket=$(pick_bucket "$est"); then
        echo "Error: chunk $(( c + 1 ))/$TOTAL_CHUNKS — $label — is indivisible and too large:" >&2
        echo "  ~$est tokens (incl. $ANSWER_HEADROOM answer headroom) exceeds the num_ctx ceiling of $NUM_CTX." >&2
        echo "  It could not be split further — file, hunk-group and line-range splitting were all exhausted," >&2
        echo "  which means one diff line (or an unsplittable segment, e.g. a binary blob) exceeds the window on its own." >&2
        echo "  Re-run with LOCAL_REVIEW_NUM_CTX=$est (needs RAM/VRAM) or review a smaller diff." >&2
        exit 1
    fi
    prompt_file="$WORK_DIR/prompt_$(printf '%05d' "$(( c + 1 ))").txt"
    printf '%s' "$prompt" > "$prompt_file"
    CHUNK_PROMPTS+=("$prompt_file")
    CHUNK_CTX+=("$bucket")
    CHUNK_EST+=("$est")
done

if [[ "$PLAN_ONLY" == "true" ]]; then
    echo "# chunk	num_ctx	est_tokens	label"
    for (( c = 0; c < TOTAL_CHUNKS; c++ )); do
        printf '%d\t%d\t%d\t%s\n' "$(( c + 1 ))" "${CHUNK_CTX[c]}" "${CHUNK_EST[c]}" "${CHUNK_LABELS[c]}"
    done
    echo "total_chunks: $TOTAL_CHUNKS"
    exit 0
fi

# --- Invoke, one request per chunk ---

BODY_FILE="$WORK_DIR/body.json"
RESPONSE_FILE="$WORK_DIR/response.json"

build_body() {  # $1 = "true" | "false" (think flag), $2 = prompt file, $3 = num_ctx
    # keep_alive is a per-request field in Ollama's /api/chat, so the
    # model's residency is scoped to this review rather than set globally
    # via OLLAMA_KEEP_ALIVE in the operator's server environment.
    jq -n \
        --arg model "$MODEL" \
        --rawfile prompt "$2" \
        --argjson num_ctx "$3" \
        --argjson think "$1" \
        --arg keep_alive "$KEEP_ALIVE" \
        '{model: $model,
          messages: [{role: "user", content: $prompt}],
          stream: false, think: $think,
          keep_alive: $keep_alive,
          options: {num_ctx: $num_ctx}}' > "$BODY_FILE"
}

send_request() {
    # curl runs alone here (jq already wrote $BODY_FILE) so a failure
    # is attributable: exit 28 = timeout, anything else = transport.
    CURL_EXIT=0
    HTTP_CODE=$(curl -s --max-time "$TIMEOUT" \
        -o "$RESPONSE_FILE" -w '%{http_code}' \
        -H 'Content-Type: application/json' \
        -d @"$BODY_FILE" "$BASE_URL/api/chat") || CURL_EXIT=$?
    if [[ "$CURL_EXIT" == "28" ]]; then
        echo "local review failed: request timed out (${TIMEOUT}s limit; LOCAL_REVIEW_TIMEOUT to raise)" >&2
        exit 1
    elif [[ "$CURL_EXIT" != "0" ]]; then
        echo "local review failed: connection to $BASE_URL failed (curl exit $CURL_EXIT)" >&2
        exit 1
    fi
}

# Reviews one chunk; prints its findings on stdout. Any failure exits 1 —
# a chunk that could not be reviewed must never be silently dropped, which
# would read downstream as "this file had no findings".
review_chunk() {  # $1 = chunk index (1-based), $2 = prompt file, $3 = num_ctx, $4 = label
    local idx="$1" prompt_file="$2" ctx="$3" label="$4"
    local where="chunk $idx/$TOTAL_CHUNKS ($label)"

    build_body true "$prompt_file" "$ctx"
    send_request

    # Non-reasoning models reject think:true with a 400; retry once without.
    if [[ "$HTTP_CODE" == "400" ]] \
            && grep -qi "does not support thinking" "$RESPONSE_FILE"; then
        build_body false "$prompt_file" "$ctx"
        send_request
    fi

    if [[ "$HTTP_CODE" != "200" ]]; then
        echo "local review failed on $where: HTTP $HTTP_CODE: $(head -c 200 "$RESPONSE_FILE")" >&2
        exit 1
    fi

    local content
    content=$(jq -r '.message.content // empty' "$RESPONSE_FILE" 2>/dev/null) || {
        echo "local review failed on $where: non-JSON response from $BASE_URL (proxy in the way?)" >&2
        exit 1
    }

    # Silent-truncation guard: the pre-flight size check estimates tokens
    # from bytes, and dense tokenization can beat the estimate — the server
    # then silently drops the head of the prompt (the instructions) and the
    # model confidently reviews a tail fragment. Truncation is detectable
    # post-hoc from the tokens the server actually ingested: the pre-check
    # guarantees an untruncated prompt leaves >= ANSWER_HEADROOM tokens of
    # window free, so an ingested count at the ceiling can only mean the
    # prompt was cut down to fit (or the estimate undershot so badly the
    # answer is untrustworthy anyway). Caveat: server-side prompt caching
    # may report fewer ingested tokens than the full prompt, so this check
    # can miss — it is a backstop for the pre-check, not a replacement.
    local prompt_eval
    prompt_eval=$(jq -r '.prompt_eval_count // 0' "$RESPONSE_FILE" 2>/dev/null || echo 0)
    if (( prompt_eval >= ctx - 1024 )); then
        echo "local review failed on $where: prompt filled the context window (server ingested $prompt_eval of num_ctx=$ctx tokens) — input was likely silently truncated;" >&2
        echo "  raise LOCAL_REVIEW_NUM_CTX (needs RAM) or review a smaller diff" >&2
        exit 1
    fi

    # done_reason must be checked independent of content emptiness: a
    # window exhausted mid-answer yields non-empty content that reads as a
    # complete review but is a truncated findings list — as untrustworthy
    # as an empty one.
    local done_reason
    done_reason=$(jq -r '.done_reason // "unknown"' "$RESPONSE_FILE" 2>/dev/null || echo "unparseable")
    if [[ "$done_reason" == "length" ]]; then
        if [[ -z "$content" ]]; then
            echo "local review failed on $where: reasoning consumed the context window before an answer was produced (done_reason: length); raise LOCAL_REVIEW_NUM_CTX (chunk requested $ctx, ceiling $NUM_CTX)" >&2
        else
            echo "local review failed on $where: answer was cut off mid-stream by the context window (done_reason: length) — a partial findings list is not trustworthy; raise LOCAL_REVIEW_NUM_CTX (chunk requested $ctx, ceiling $NUM_CTX)" >&2
        fi
        exit 1
    fi

    if [[ -z "$content" ]]; then
        echo "local review failed on $where: model returned an empty answer (done_reason: $done_reason)" >&2
        exit 1
    fi

    printf '%s' "$content"
}

# --- Deterministic merge ---
#
# Chunks are non-overlapping, so findings are concatenated under a
# per-chunk heading and renumbered globally. No second LLM pass: a
# synthesis call would double token/time cost with no demonstrated need,
# and a deterministic merge cannot invent or drop a finding.

MERGED="$WORK_DIR/merged.md"
: > "$MERGED"
CHUNKS_WITH_FINDINGS=0

for (( c = 0; c < TOTAL_CHUNKS; c++ )); do
    chunk_content=$(review_chunk "$(( c + 1 ))" "${CHUNK_PROMPTS[c]}" "${CHUNK_CTX[c]}" "${CHUNK_LABELS[c]}")
    # A chunk whose entire answer is the sentinel contributes nothing;
    # anything else is kept verbatim (a "no findings, but note that ..."
    # answer is a finding).
    trimmed=$(printf '%s' "$chunk_content" | tr -d '[:space:]')
    if [[ "${trimmed,,}" == "nofindings" || "${trimmed,,}" == "nofindings." ]]; then
        continue
    fi
    CHUNKS_WITH_FINDINGS=$(( CHUNKS_WITH_FINDINGS + 1 ))
    {
        printf '### %s\n\n' "${CHUNK_LABELS[c]}"
        printf '%s\n\n' "$chunk_content"
    } >> "$MERGED"
done

echo "## Local Adversarial ($MODEL)"
echo
if (( TOTAL_CHUNKS > 1 )); then
    echo "_Reviewed in $TOTAL_CHUNKS chunks; each chunk saw only its own file(s)."
    echo "Defects spanning chunks (e.g. a caller in another file) are outside this specialist's view._"
    echo
fi

if (( CHUNKS_WITH_FINDINGS == 0 )); then
    echo "No findings."
    exit 0
fi

# Renumber top-level findings 1..N across all chunks. Lines inside fenced
# code blocks are left alone so a numbered snippet is not rewritten.
awk '
    /^```/ { fence = !fence; print; next }
    !fence && /^[0-9]+\./ {
        n++
        sub(/^[0-9]+\./, n ".")
        print
        next
    }
    { print }
' "$MERGED"
