#!/bin/bash
# .agent/scripts/tests/test_local_review.sh
# Tests the chunking, num_ctx bucket selection, overflow guard and
# deterministic merge in .agent/scripts/local_review.sh (#605).
#
# Why these cases: 5f's two failure modes were (a) a 23 GB model against
# 8 GB of VRAM and (b) real diffs several times larger than num_ctx. The
# fix is to split the diff into chunks that fit and to size each request's
# num_ctx to the chunk rather than always asking for the ceiling. Both are
# arithmetic that is easy to get subtly wrong and impossible to notice at
# runtime — a mis-sized bucket just costs VRAM silently, and a bad split
# just loses review coverage silently. So the split boundaries, the bucket
# ladder (including that every bucket in it is reachable), the loud-failure
# guard, and the merge/renumbering are all pinned here.
#
# Hermetic: --plan mode needs no server, and the end-to-end cases run
# against a stub `curl` on PATH. No network, no Ollama, no GPU.
#
# Run: bash .agent/scripts/tests/test_local_review.sh

set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LR="$SCRIPT_DIR/../local_review.sh"
TEST_PASS=0
TEST_FAIL=0

pass() { echo "PASS: $1"; TEST_PASS=$((TEST_PASS + 1)); }
fail() { echo "FAIL: $1"; TEST_FAIL=$((TEST_FAIL + 1)); }

TMPD="$(mktemp -d)"
trap 'rm -rf "$TMPD"' EXIT

# Neutralise any operator settings leaking in from the environment.
unset LOCAL_REVIEW_MODEL LOCAL_REVIEW_URL LOCAL_REVIEW_TIMEOUT \
      LOCAL_REVIEW_NUM_CTX LOCAL_REVIEW_ANSWER_HEADROOM LOCAL_REVIEW_KEEP_ALIVE

# --- fixtures ---------------------------------------------------------

# One hunk of `count` added lines for `path`, starting at `@@` line 1.
make_file_diff() {  # $1 = path, $2 = line count, $3 = payload width
    local path="$1" count="$2" width="${3:-40}" i
    printf 'diff --git a/%s b/%s\n' "$path" "$path"
    printf 'index 1111111..2222222 100644\n'
    printf -- '--- a/%s\n+++ b/%s\n' "$path" "$path"
    printf '@@ -0,0 +1,%d @@\n' "$count"
    for (( i = 0; i < count; i++ )); do
        # Payload includes the path so lines are unique across files —
        # the losslessness check counts duplicates.
        printf '+%s_%0*d\n' "${path//\//_}" "$width" "$i"
    done
}

# `hunks` separate hunks in one file, each `count` added lines.
make_multi_hunk_diff() {  # $1 = path, $2 = hunks, $3 = lines per hunk
    local path="$1" hunks="$2" count="$3" h i
    printf 'diff --git a/%s b/%s\n' "$path" "$path"
    printf 'index 1111111..2222222 100644\n'
    printf -- '--- a/%s\n+++ b/%s\n' "$path" "$path"
    for (( h = 0; h < hunks; h++ )); do
        printf '@@ -%d,%d +%d,%d @@ ctx%d\n' \
            $(( h * 100 + 1 )) "$count" $(( h * 100 + 1 )) "$count" "$h"
        for (( i = 0; i < count; i++ )); do
            printf '+hunk%d_line%040d\n' "$h" "$i"
        done
    done
}

THREE_FILES="$TMPD/three_files.diff"
{
    make_file_diff "src/alpha.py" 20
    make_file_diff "src/beta.py" 20
    make_file_diff "docs/gamma.md" 20
} > "$THREE_FILES"

plan() { "$LR" --plan < "$1" 2>&1; }

# --- 1. per-file splitting -------------------------------------------

out=$(plan "$THREE_FILES"); rc=$?
if [ "$rc" -eq 0 ] \
    && [ "$(echo "$out" | grep -c $'^[0-9]\t')" -eq 3 ] \
    && echo "$out" | grep -qP '^1\t\d+\t\d+\tsrc/alpha\.py$' \
    && echo "$out" | grep -qP '^2\t\d+\t\d+\tsrc/beta\.py$' \
    && echo "$out" | grep -qP '^3\t\d+\t\d+\tdocs/gamma\.md$' \
    && echo "$out" | grep -q '^total_chunks: 3$'; then
    pass "splits a 3-file diff into one chunk per file, labelled by path"
else
    fail "splits a 3-file diff into one chunk per file (rc=$rc)
$out"
fi

# A one-file diff must stay one chunk — chunking must not fragment what
# already fits.
ONE_FILE="$TMPD/one_file.diff"
make_file_diff "src/alpha.py" 20 > "$ONE_FILE"
out=$(plan "$ONE_FILE")
if echo "$out" | grep -q '^total_chunks: 1$'; then
    pass "a diff that already fits stays a single chunk"
else
    fail "a diff that already fits stays a single chunk
$out"
fi

# --- 2. bucket selection ---------------------------------------------

# With the default 12288 headroom, the 8192 rung is arithmetically
# unreachable (headroom alone exceeds it) and must be pruned rather than
# advertised — every bucket the script can print has to be selectable.
out=$(plan "$THREE_FILES")
if ! echo "$out" | grep -qP '^[0-9]+\t8192\t'; then
    pass "8192 bucket is not selected at the default 12288 headroom (pruned as unreachable)"
else
    fail "8192 bucket selected despite a 12288 headroom
$out"
fi

# Lowering the headroom is what unlocks the small bucket — the mechanism
# by which a smaller model actually saves VRAM. If this fails, the
# headroom is not really wired into bucket selection.
out=$(LOCAL_REVIEW_ANSWER_HEADROOM=4096 plan "$THREE_FILES")
if echo "$out" | grep -qP '^1\t8192\t'; then
    pass "lowering the answer headroom makes the 8192 bucket reachable"
else
    fail "lowering the answer headroom makes the 8192 bucket reachable
$out"
fi

# The bucket must track chunk size, not be a constant.
SMALL="$TMPD/small.diff"; make_file_diff "s.py" 5 > "$SMALL"
BIG="$TMPD/big.diff";    make_file_diff "b.py" 900 > "$BIG"
small_ctx=$(LOCAL_REVIEW_ANSWER_HEADROOM=4096 plan "$SMALL" | awk -F'\t' '/^1\t/{print $2}')
big_ctx=$(LOCAL_REVIEW_ANSWER_HEADROOM=4096 plan "$BIG" | awk -F'\t' '/^1\t/{print $2}')
if [ -n "$small_ctx" ] && [ -n "$big_ctx" ] && [ "$small_ctx" -lt "$big_ctx" ]; then
    pass "num_ctx is sized per chunk (small=$small_ctx < big=$big_ctx), not fixed at the ceiling"
else
    fail "num_ctx is sized per chunk (small='$small_ctx' big='$big_ctx')"
fi

# Every selected bucket must actually hold that chunk's estimate.
bad=$(LOCAL_REVIEW_ANSWER_HEADROOM=4096 plan "$THREE_FILES" \
        | awk -F'\t' '/^[0-9]+\t/ && $3 > $2 {print}')
if [ -z "$bad" ]; then
    pass "every chunk's estimate fits inside its selected bucket"
else
    fail "a chunk's estimate exceeds its bucket: $bad"
fi

# A custom ceiling must be honoured as the top bucket rather than rounded
# past to the next ladder rung.
out=$(LOCAL_REVIEW_NUM_CTX=20000 LOCAL_REVIEW_ANSWER_HEADROOM=4096 plan "$BIG")
top=$(echo "$out" | awk -F'\t' '/^1\t/{print $2}')
if [ "$top" = "20000" ] || [ "$top" = "16384" ]; then
    pass "a non-ladder LOCAL_REVIEW_NUM_CTX ceiling is honoured (got $top)"
else
    fail "a non-ladder ceiling is honoured (got '$top')
$out"
fi

# --- 3. sub-file hunk splitting --------------------------------------

MULTI="$TMPD/multi.diff"
make_multi_hunk_diff "src/big.py" 8 200 > "$MULTI"
out=$(LOCAL_REVIEW_NUM_CTX=16384 LOCAL_REVIEW_ANSWER_HEADROOM=4096 plan "$MULTI")
n=$(echo "$out" | grep -c $'^[0-9]\t')
if [ "$n" -gt 1 ] && echo "$out" | grep -q 'src/big.py (hunks '; then
    pass "one oversized file is split into hunk groups ($n chunks), labelled with the hunk range"
else
    fail "one oversized file is split into hunk groups (n=$n)
$out"
fi

# The hunk ranges must tile 1..8 with no gap and no overlap. (Full
# line-level losslessness is asserted in section 6b against the actual
# request bodies.)
ranges=$(echo "$out" | grep -oP '\(hunks \d+-\d+ of 8\)' | grep -oP '\d+-\d+')
first=$(echo "$ranges" | head -1 | cut -d- -f1)
last=$(echo "$ranges" | tail -1 | cut -d- -f2)
gap=0
prev=0
while read -r r; do
    [ -z "$r" ] && continue
    s=${r%%-*}; e=${r##*-}
    [ "$s" -ne $(( prev + 1 )) ] && gap=1
    prev=$e
done <<< "$ranges"
if [ "$first" = "1" ] && [ "$last" = "8" ] && [ "$gap" -eq 0 ]; then
    pass "hunk groups tile the file's hunks contiguously (1..8, no gap, no overlap)"
else
    fail "hunk groups tile the file's hunks (first=$first last=$last gap=$gap)
$ranges"
fi

# --- 4. line-range splitting of one oversized hunk -------------------

# A large *added file* is a single hunk, so hunk-level splitting alone
# cannot save it — this is the common case that used to abort the run.
ONE_HUNK="$TMPD/one_hunk.diff"
make_file_diff "src/generated.py" 2000 > "$ONE_HUNK"
out=$(LOCAL_REVIEW_NUM_CTX=16384 LOCAL_REVIEW_ANSWER_HEADROOM=4096 plan "$ONE_HUNK"); rc=$?
n=$(echo "$out" | grep -c $'^[0-9]\t')
if [ "$rc" -eq 0 ] && [ "$n" -gt 1 ] && echo "$out" | grep -q 'part 1/'; then
    pass "a single oversized hunk is split into line-range parts ($n chunks), not rejected"
else
    fail "a single oversized hunk is split into line-range parts (rc=$rc n=$n)
$out"
fi

# --- 5. loud-failure guard -------------------------------------------

# One diff line longer than the whole window is genuinely irreducible.
# It must abort with the offending file named — never be truncated.
IRREDUCIBLE="$TMPD/irreducible.diff"
{
    printf 'diff --git a/src/blob.py b/src/blob.py\n'
    printf -- '--- a/src/blob.py\n+++ b/src/blob.py\n'
    printf '@@ -0,0 +1,1 @@\n'
    printf '+%0*d\n' 200000 1
} > "$IRREDUCIBLE"
out=$(LOCAL_REVIEW_NUM_CTX=16384 LOCAL_REVIEW_ANSWER_HEADROOM=4096 plan "$IRREDUCIBLE"); rc=$?
if [ "$rc" -eq 1 ] \
    && echo "$out" | grep -q 'src/blob.py' \
    && echo "$out" | grep -qi 'could not be split further' \
    && echo "$out" | grep -q 'LOCAL_REVIEW_NUM_CTX='; then
    pass "an irreducible oversized line fails loud, naming the file and the num_ctx needed"
else
    fail "an irreducible oversized line fails loud (rc=$rc)
$out"
fi

# A headroom that swallows the whole window is a misconfiguration, not a
# silent no-op: it must be rejected up front rather than leaving an empty
# bucket ladder.
out=$(LOCAL_REVIEW_NUM_CTX=8192 LOCAL_REVIEW_ANSWER_HEADROOM=8192 plan "$ONE_FILE"); rc=$?
if [ "$rc" -eq 1 ] && echo "$out" | grep -qi 'no usable context budget'; then
    pass "headroom >= num_ctx is rejected up front (no empty bucket ladder)"
else
    fail "headroom >= num_ctx is rejected up front (rc=$rc)
$out"
fi

# Non-integer configuration must be rejected, not silently coerced.
out=$(LOCAL_REVIEW_ANSWER_HEADROOM=lots plan "$ONE_FILE"); rc=$?
if [ "$rc" -eq 1 ] && echo "$out" | grep -q 'LOCAL_REVIEW_ANSWER_HEADROOM'; then
    pass "a non-integer LOCAL_REVIEW_ANSWER_HEADROOM is rejected"
else
    fail "a non-integer LOCAL_REVIEW_ANSWER_HEADROOM is rejected (rc=$rc)
$out"
fi

# An empty diff is still an error, not a vacuous "No findings."
out=$(printf '' | "$LR" --plan 2>&1); rc=$?
if [ "$rc" -eq 1 ] && echo "$out" | grep -qi 'empty'; then
    pass "an empty diff is an error, not a vacuous pass"
else
    fail "an empty diff is an error (rc=$rc)
$out"
fi

# --- 6. end-to-end against a stub Ollama -----------------------------
#
# The stub records every request body so the per-chunk num_ctx and the
# keep_alive field can be asserted on what is actually sent, not on what
# the plan says it will send.

STUB_DIR="$TMPD/bin"
mkdir -p "$STUB_DIR"
REQ_DIR="$TMPD/requests"
mkdir -p "$REQ_DIR"
cat > "$STUB_DIR/curl" <<'STUB'
#!/bin/bash
# Minimal stand-in for curl against Ollama's HTTP API.
url="${@: -1}"
out=""; body=""
args=("$@")
for (( i = 0; i < ${#args[@]}; i++ )); do
    case "${args[i]}" in
        -o) out="${args[i+1]}" ;;
        -d) body="${args[i+1]#@}" ;;
    esac
done
case "$url" in
    */api/version) echo '{"version":"0.0.0-stub"}'; exit 0 ;;
    */api/tags)    echo '{"models":[{"name":"stub-model:latest"}]}'; exit 0 ;;
    */api/chat)
        n=$(ls "$REQ_DIR" | wc -l)
        cp "$body" "$REQ_DIR/req_$(printf '%03d' "$((n+1))").json"
        answer=$(cat "$STUB_ANSWER_FILE")
        code=$(cat "$STUB_HTTP_CODE_FILE" 2>/dev/null || echo 200)
        if [ "$code" != "200" ] && [ "$((n+1))" -eq "${STUB_FAIL_ON_CHUNK:-0}" ]; then
            printf '%s' '{"error":"stub failure"}' > "$out"
            printf '%s' "$code"; exit 0
        fi
        jq -n --arg c "$answer" \
            '{message:{content:$c},done_reason:"stop",prompt_eval_count:10}' > "$out"
        printf '200'; exit 0 ;;
esac
exit 1
STUB
chmod +x "$STUB_DIR/curl"

export REQ_DIR
STUB_ANSWER_FILE="$TMPD/answer.txt"; export STUB_ANSWER_FILE
STUB_HTTP_CODE_FILE="$TMPD/http_code.txt"; export STUB_HTTP_CODE_FILE
echo 200 > "$STUB_HTTP_CODE_FILE"

run_stubbed() {  # stdin = diff
    PATH="$STUB_DIR:$PATH" LOCAL_REVIEW_MODEL="stub-model" \
        LOCAL_REVIEW_ANSWER_HEADROOM="${HR:-4096}" \
        LOCAL_REVIEW_NUM_CTX="${NC:-32768}" \
        "$LR" 2>&1
}

printf '1. Off-by-one in the loop bound.\n2. Unclosed file handle.\n' > "$STUB_ANSWER_FILE"
rm -f "$REQ_DIR"/*
out=$(run_stubbed < "$THREE_FILES"); rc=$?
nreq=$(ls "$REQ_DIR" | wc -l)
if [ "$rc" -eq 0 ] && [ "$nreq" -eq 3 ]; then
    pass "one HTTP request is sent per chunk (3 files -> 3 requests)"
else
    fail "one HTTP request is sent per chunk (rc=$rc nreq=$nreq)
$out"
fi

# Each request must carry its own bucket, and none may exceed the ceiling.
ctxs=$(cat "$REQ_DIR"/*.json | jq -r '.options.num_ctx')
overs=$(echo "$ctxs" | awk '$1 > 32768')
if [ -n "$ctxs" ] && [ -z "$overs" ]; then
    pass "every request carries its own options.num_ctx, none above the ceiling ($(echo "$ctxs" | tr '\n' ' '))"
else
    fail "per-request num_ctx (got '$ctxs', over-ceiling '$overs')"
fi

# The prompt actually sent must be the chunk, not the whole diff — a chunk
# reviewing beta.py must not contain alpha.py's diff.
p1=$(jq -r '.messages[0].content' "$REQ_DIR/req_001.json")
p2=$(jq -r '.messages[0].content' "$REQ_DIR/req_002.json")
if echo "$p1" | grep -q 'src/alpha.py' && ! echo "$p1" | grep -q 'src/beta.py' \
   && echo "$p2" | grep -q 'src/beta.py' && ! echo "$p2" | grep -q 'src/alpha.py'; then
    pass "each request's prompt contains only its own chunk's diff"
else
    fail "each request's prompt contains only its own chunk's diff"
fi

# keep_alive must travel as a per-request JSON field, never as a global
# OLLAMA_KEEP_ALIVE -- the workspace must not reach into the operator's
# machine-wide Ollama config.
ka=$(jq -r '.keep_alive' "$REQ_DIR/req_001.json")
if [ "$ka" = "30s" ]; then
    pass "requests carry the default keep_alive (30s) as a per-request field"
else
    fail "requests carry the default keep_alive as a per-request field (got '$ka')"
fi

rm -f "$REQ_DIR"/*
LOCAL_REVIEW_KEEP_ALIVE=5m PATH="$STUB_DIR:$PATH" LOCAL_REVIEW_MODEL="stub-model" \
    LOCAL_REVIEW_ANSWER_HEADROOM=4096 "$LR" < "$ONE_FILE" > /dev/null 2>&1
ka=$(jq -r '.keep_alive' "$REQ_DIR/req_001.json")
if [ "$ka" = "5m" ]; then
    pass "LOCAL_REVIEW_KEEP_ALIVE overrides the default"
else
    fail "LOCAL_REVIEW_KEEP_ALIVE overrides the default (got '$ka')"
fi

# --- 6b. splitting is lossless ---------------------------------------
#
# The strongest property of the split: every changed line of the original
# diff reaches the model exactly once. Asserted against the prompts that
# were actually sent, across all three split levels (file, hunk group,
# line range) at once.

LOSSLESS="$TMPD/lossless.diff"
{
    make_file_diff "src/small.py" 10
    make_multi_hunk_diff "src/many_hunks.py" 8 200
    make_file_diff "src/one_big_hunk.py" 2000
} > "$LOSSLESS"

rm -f "$REQ_DIR"/*
printf 'No findings.\n' > "$STUB_ANSWER_FILE"
HR=4096 NC=16384 run_stubbed < "$LOSSLESS" > /dev/null 2>&1
sent="$TMPD/sent.txt"
: > "$sent"
for f in "$REQ_DIR"/*.json; do
    jq -r '.messages[0].content' "$f" >> "$sent"
done
# Changed lines only (the '+' lines the fixtures emit); '+++' header lines
# are excluded on both sides.
grep '^+' "$LOSSLESS" | grep -v '^+++' | sort > "$TMPD/expected_lines.txt"
grep '^+' "$sent"     | grep -v '^+++' | sort > "$TMPD/sent_lines.txt"
missing=$(comm -23 "$TMPD/expected_lines.txt" "$TMPD/sent_lines.txt" | wc -l)
dupes=$(sort "$TMPD/sent_lines.txt" | uniq -d | wc -l)
total=$(wc -l < "$TMPD/expected_lines.txt")
if [ "$missing" -eq 0 ] && [ "$dupes" -eq 0 ] && [ "$total" -gt 2000 ]; then
    pass "splitting is lossless: all $total changed lines sent exactly once, none duplicated"
else
    fail "splitting is lossless (total=$total missing=$missing duplicated=$dupes)"
fi

# --- 7. deterministic merge and renumbering --------------------------

if echo "$out" | grep -q '^## Local Adversarial (stub-model)$' \
   && [ "$(echo "$out" | grep -c '^### ')" -eq 3 ]; then
    pass "findings are merged under one per-chunk heading each"
else
    fail "findings are merged under per-chunk headings
$out"
fi

nums=$(echo "$out" | grep -oP '^\d+(?=\.)' | tr '\n' ' ')
if [ "$nums" = "1 2 3 4 5 6 " ]; then
    pass "findings are renumbered globally across chunks (got: $nums)"
else
    fail "findings are renumbered globally across chunks (got: '$nums')"
fi

# The cross-file blind spot must be stated in the output a reader actually
# sees, not only in the script header.
if echo "$out" | grep -qi 'chunk'; then
    pass "chunked output carries the cross-chunk limitation notice"
else
    fail "chunked output carries the cross-chunk limitation notice
$out"
fi

# A chunk that genuinely found nothing contributes no heading; an all-clear
# review still says so once.
printf 'No findings.\n' > "$STUB_ANSWER_FILE"
rm -f "$REQ_DIR"/*
out=$(run_stubbed < "$THREE_FILES")
if [ "$(echo "$out" | grep -c '^### ')" -eq 0 ] \
   && echo "$out" | grep -q '^No findings\.$'; then
    pass "an all-clear review reports 'No findings.' once, with no empty headings"
else
    fail "an all-clear review reports 'No findings.' once
$out"
fi

# Numbered lines inside a fenced code block must not be renumbered as
# findings — the merge is textual and must not corrupt quoted code.
printf '1. Bad bound.\n\n```\n1. not a finding\n```\n' > "$STUB_ANSWER_FILE"
rm -f "$REQ_DIR"/*
out=$(run_stubbed < "$ONE_FILE")
if [ "$(echo "$out" | grep -c '^1\. not a finding$')" -eq 1 ]; then
    pass "numbered lines inside fenced code blocks are not renumbered"
else
    fail "numbered lines inside fenced code blocks are not renumbered
$out"
fi

# --- 8. per-chunk failure is loud ------------------------------------

# A chunk that errors must abort the whole review. Silently dropping it
# would read downstream as "that file had no findings" — the exact silent
# failure this script exists to avoid.
printf '1. Something.\n' > "$STUB_ANSWER_FILE"
echo 500 > "$STUB_HTTP_CODE_FILE"
export STUB_FAIL_ON_CHUNK=2
rm -f "$REQ_DIR"/*
out=$(run_stubbed < "$THREE_FILES"); rc=$?
if [ "$rc" -eq 1 ] \
   && echo "$out" | grep -q 'chunk 2/3' \
   && echo "$out" | grep -q 'src/beta.py'; then
    pass "a failing chunk aborts the review, naming the chunk and file"
else
    fail "a failing chunk aborts the review (rc=$rc)
$out"
fi
unset STUB_FAIL_ON_CHUNK
echo 200 > "$STUB_HTTP_CODE_FILE"

# --- summary ----------------------------------------------------------

echo ""
echo "Passed: $TEST_PASS  Failed: $TEST_FAIL"
[ "$TEST_FAIL" -eq 0 ]
