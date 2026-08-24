#!/bin/bash
# .agent/scripts/tests/test_ci_verification_helpers.sh
# Tests for ci_local_attestation_status() in _ci_verification_helpers.sh (#610).
#
# The helper decides whether a PR head carries a full-scope ci_local attestation
# (ADR-0018 decisions 1/2 + the #577 upstream.repos completeness rule), which is
# what lets merge_pr.sh merge a no-CI project repo on evidence rather than on a
# warning. Every case here is hermetic: throwaway git repos, notes written by
# hand in ci_local.sh's exact format, and a bare "origin" reached over a local
# path — no network, no container, no gh.
#
# Run: bash .agent/scripts/tests/test_ci_verification_helpers.sh

set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HELPERS="$SCRIPT_DIR/../_ci_verification_helpers.sh"

PASS=0; FAIL=0
ok()  { echo "  ✅ $1"; PASS=$((PASS+1)); }
bad() { echo "  ❌ $1"; FAIL=$((FAIL+1)); }

TMPROOT=$(mktemp -d /tmp/test_ci_verif.XXXXXX)
trap 'rm -rf "$TMPROOT"' EXIT

# shellcheck source=../_ci_verification_helpers.sh
source "$HELPERS"

GIT_ID=(-c user.email=t@t -c user.name=t)

# A repo with one commit; echoes its path.
new_repo() {
    local d="$TMPROOT/$1"
    mkdir -p "$d"
    git -C "$d" init -q
    echo "x" > "$d/file"
    git -C "$d" add -A
    git -C "$d" "${GIT_ID[@]}" commit -q -m init
    echo "$d"
}

head_of() { git -C "$1" rev-parse HEAD; }

add_note() {  # add_note <repo> <sha> <body>
    git -C "$1" "${GIT_ID[@]}" notes --ref=ci-local add -m "$3" "$2"
}
append_note() {
    git -C "$1" "${GIT_ID[@]}" notes --ref=ci-local append -m "
---
$3" "$2"
}

# The note body ci_local.sh writes, parameterized on the fields that matter.
note_body() {  # note_body <pass-label> <scope> [extra lines...]
    printf '%s\nrepo: testrepo\ncommit: deadbeef\nimage: img\nimage-id: sha256:0\npackages: all\nscope: %s\n' "$1" "$2"
    shift 2
    for line in "$@"; do printf '%s\n' "$line"; done
    printf 'steps: template\ndate: 2026-08-24T00:00:00Z\nhost: testhost\nlog-sha256: 0\n'
}

# assert_verdict <desc> <expect: attested|no-attestation> <repo> <sha> [msg-substring]
assert_verdict() {
    local desc="$1" expect="$2" repo="$3" sha="$4" want_msg="${5:-}"
    local msg rc
    msg=$(ci_local_attestation_status "$repo" "$sha"); rc=$?
    local got="no-attestation"; [[ $rc -eq 0 ]] && got="attested"
    if [[ "$got" != "$expect" ]]; then
        bad "$desc (got $got, msg: $msg)"; return
    fi
    if [[ -n "$want_msg" ]] && ! grep -qF "$want_msg" <<<"$msg"; then
        bad "$desc (verdict right but message lacks '$want_msg': $msg)"; return
    fi
    ok "$desc"
}

echo "Test: helper is syntactically valid"
bash -n "$HELPERS" && ok "bash -n clean" || bad "bash -n failed"

echo "Test: no note at all → no-attestation"
r=$(new_repo no_note)
assert_verdict "bare repo, no note" no-attestation "$r" "$(head_of "$r")" "no ci-local note"

echo "Test: scope: partial → no-attestation (ADR-0018 decision 2)"
r=$(new_repo partial_scope); s=$(head_of "$r")
add_note "$r" "$s" "$(note_body 'ci-local: pass' partial)"
assert_verdict "partial scope rejected" no-attestation "$r" "$s" "scope: full"

echo "Test: 'pass (partial)' label → no-attestation (exact match required)"
r=$(new_repo partial_label); s=$(head_of "$r")
add_note "$r" "$s" "$(note_body 'ci-local: pass (partial)' full)"
assert_verdict "pass (partial) rejected" no-attestation "$r" "$s"

echo "Test: pass + scope: full on the exact head → attested"
r=$(new_repo good); s=$(head_of "$r")
add_note "$r" "$s" "$(note_body 'ci-local: pass' full)"
assert_verdict "full-scope pass accepted" attested "$r" "$s" "full-scope ci-local attestation"

echo "Test: multi-block note (partial appended, then full) → attested"
# ci_local.sh APPENDS records; a later full run must not be masked by an
# earlier partial one, so every block has to be considered.
r=$(new_repo multiblock); s=$(head_of "$r")
add_note "$r" "$s" "$(note_body 'ci-local: pass (partial)' partial)"
append_note "$r" "$s" "$(note_body 'ci-local: pass' full)"
assert_verdict "any qualifying block accepted" attested "$r" "$s"

echo "Test: note on an ANCESTOR only → no-attestation"
# An attestation for an earlier commit is not evidence for a head that may
# carry unverified changes since (ADR-0018: the note must be on the head).
r=$(new_repo ancestor); anc=$(head_of "$r")
add_note "$r" "$anc" "$(note_body 'ci-local: pass' full)"
git -C "$r" "${GIT_ID[@]}" commit -q --allow-empty -m second
assert_verdict "ancestor-only note rejected" no-attestation "$r" "$(head_of "$r")"

echo "Test: upstream.repos entries not all covered → no-attestation (#577)"
r=$(new_repo upstream_incomplete)
cat > "$r/upstream.repos" <<'YAML'
repositories:
  alpha:
    type: git
    url: https://example.invalid/alpha.git
    version: main
  beta:
    type: git
    url: https://example.invalid/beta.git
    version: main
YAML
git -C "$r" add -A && git -C "$r" "${GIT_ID[@]}" commit -q -m upstream
s=$(head_of "$r")
add_note "$r" "$s" "$(note_body 'ci-local: pass' full 'upstream-repo: alpha@1111111111111111111111111111111111111111')"
assert_verdict "incomplete upstream coverage rejected" no-attestation "$r" "$s" "upstream-repo"

echo "Test: upstream.repos entries all covered → attested"
r=$(new_repo upstream_complete)
cat > "$r/upstream.repos" <<'YAML'
repositories:
  alpha:
    type: git
    url: https://example.invalid/alpha.git
    version: main
  beta:
    type: git
    url: https://example.invalid/beta.git
    version: main
YAML
git -C "$r" add -A && git -C "$r" "${GIT_ID[@]}" commit -q -m upstream
s=$(head_of "$r")
add_note "$r" "$s" "$(note_body 'ci-local: pass' full \
    'upstream-repo: alpha@1111111111111111111111111111111111111111' \
    'upstream-repo: beta@2222222222222222222222222222222222222222')"
assert_verdict "complete upstream coverage accepted" attested "$r" "$s"

# ---- fetch fallback ---------------------------------------------------------
# The note may live only on origin (attested on another machine, or pushed by
# ci_local.sh from a different checkout). Fetching must reach it WITHOUT
# clobbering a local, not-yet-pushed refs/notes/ci-local.
setup_origin_case() {   # echoes "<clone> <sha>"
    local name="$1"
    local up; up=$(new_repo "${name}_up")
    local sha; sha=$(head_of "$up")
    add_note "$up" "$sha" "$(note_body 'ci-local: pass' full)"
    local bare="$TMPROOT/${name}_bare.git"
    git init -q --bare "$bare"
    git -C "$up" push -q "$bare" HEAD:refs/heads/main
    git -C "$up" push -q "$bare" refs/notes/ci-local
    local clone="$TMPROOT/${name}_clone"
    git clone -q "$bare" "$clone"
    echo "$clone $sha"
}

echo "Test: attestation only on origin → attested via scratch-ref fetch"
read -r clone sha <<<"$(setup_origin_case fetchcase)"
assert_verdict "origin-only note fetched" attested "$clone" "$sha" "from origin note"

echo "Test: scratch-ref fetch leaves a local unpushed refs/notes/ci-local intact"
read -r clone sha <<<"$(setup_origin_case localsafe)"
git -C "$clone" "${GIT_ID[@]}" commit -q --allow-empty -m local-only
localsha=$(head_of "$clone")
add_note "$clone" "$localsha" "LOCAL UNPUSHED SENTINEL"
before=$(git -C "$clone" rev-parse refs/notes/ci-local)
ci_local_attestation_status "$clone" "$sha" >/dev/null
after=$(git -C "$clone" rev-parse refs/notes/ci-local)
{ [[ "$before" == "$after" ]] && \
  git -C "$clone" notes --ref=ci-local show "$localsha" 2>/dev/null | grep -q SENTINEL; } \
    && ok "local note untouched by the fetch" \
    || bad "local refs/notes/ci-local was perturbed ($before → $after)"

echo "Test: leftover scratch ref from an interrupted run → still attested"
# Regression guard: without the forced refspec + up-front delete, a stale
# scratch ref makes the fetch fail non-fast-forward, and the helper would report
# a FALSE 'no attestation' — merging on a warning instead of on evidence.
# (Per-call ref names now also make a leftover unlikely; the forced refspec is
# still the guard that makes it harmless.)
read -r clone sha <<<"$(setup_origin_case leftover)"
git -C "$clone" "${GIT_ID[@]}" commit -q --allow-empty -m unrelated
unrelated=$(head_of "$clone")
planted=()
for stale in refs/notes/ci-local-merge-check "$(_ci_local_scratch_ref)"; do
    git -C "$clone" "${GIT_ID[@]}" notes --ref="$stale" add -m "stale junk" "$unrelated"
    planted+=("$stale")
done
assert_verdict "stale scratch ref does not break the fetch" attested "$clone" "$sha"
# Every scratch ref still present must be one we planted — the helper must
# leave none of its own behind.
leftover=$(git -C "$clone" for-each-ref --format='%(refname)' "$CI_LOCAL_SCRATCH_REF_PREFIX*" \
    | grep -vxF "$(printf '%s\n' "${planted[@]}")" || true)
[[ -z "$leftover" ]] \
    && ok "per-call scratch ref cleaned up after the call" \
    || bad "a scratch ref was left behind: $leftover"

echo "Test: concurrent lookups in one repo do not sabotage each other"
# The scratch ref name must be PER CALL. With a constant name, a second
# merge_pr.sh running in the same repo deletes the ref mid-fetch and the first
# one reports a false 'no attestation' — i.e. an ATTESTED PR falls through to
# merge-with-no-verification. The operator runs several agent sessions at once.
read -r clone sha <<<"$(setup_origin_case concurrent)"
conc_fail=0
for _ in 1 2 3 4 5; do
    ( ci_local_attestation_status "$clone" "$sha" >"$TMPROOT/conc_a.txt" ) & pa=$!
    ( ci_local_attestation_status "$clone" "$sha" >"$TMPROOT/conc_b.txt" ) & pb=$!
    wait "$pa" || conc_fail=$((conc_fail+1))
    wait "$pb" || conc_fail=$((conc_fail+1))
done
[[ $conc_fail -eq 0 ]] \
    && ok "10/10 concurrent lookups still attested" \
    || bad "$conc_fail of 10 concurrent lookups lost the attestation"

echo "Test: head commit absent + note WITHOUT upstream-repo lines → no-attestation"
# The hole this closes: completeness was only enforced when the note itself
# carried upstream-repo lines, so a note that omitted them skipped the #577
# rule entirely and attested — the inverse of the guard's intent. Absent the
# head commit, upstream.repos cannot be read at all, so nothing is checkable.
r=$(new_repo absent_head_plain)
absent2="0123456789012345678901234567890123456789"
add_note "$r" "$absent2" "$(note_body 'ci-local: pass' full)"
assert_verdict "head-absent note without upstream lines rejected" no-attestation "$r" "$absent2" "cannot be checked"

echo "Test: 'pass' and 'scope: full' split across two records → no-attestation"
# Each appended record describes ONE run. A pass in one and full scope in
# another never together describe a full-scope passing run.
r=$(new_repo split_records); s=$(head_of "$r")
add_note "$r" "$s" "$(note_body 'ci-local: pass' partial)"
append_note "$r" "$s" "$(note_body 'ci-local: fail' full)"
assert_verdict "cross-record pass/scope split rejected" no-attestation "$r" "$s"

echo "Test: a regex-metacharacter upstream.repos key cannot satisfy completeness"
# `${entry}` is interpolated into a grep -E pattern; an entry of `.*` would
# otherwise be matched by any single upstream-repo line.
r=$(new_repo upstream_regex)
cat > "$r/upstream.repos" <<'YAML'
repositories:
  ".*":
    type: git
    url: https://example.invalid/x.git
    version: main
YAML
git -C "$r" add -A && git -C "$r" "${GIT_ID[@]}" commit -q -m upstream
s=$(head_of "$r")
add_note "$r" "$s" "$(note_body 'ci-local: pass' full \
    'upstream-repo: alpha@1111111111111111111111111111111111111111')"
assert_verdict "regex-metacharacter entry rejected" no-attestation "$r" "$s" "upstream-repo"

echo "Test: note claims upstream-repo but the commit isn't local → no-attestation"
# We cannot read upstream.repos at that commit, so completeness is unverifiable;
# it must fail closed with a fetch hint rather than pass by default.
r=$(new_repo absent_head)
absent="0123456789012345678901234567890123456789"
add_note "$r" "$absent" "$(note_body 'ci-local: pass' full 'upstream-repo: alpha@1111111111111111111111111111111111111111')"
assert_verdict "unverifiable upstream coverage rejected" no-attestation "$r" "$absent" "fetch origin"

# ---- publishing the attestation (ADR-0018 decision 5) -----------------------
echo "Test: note read from origin → nothing to push, and no false 'only on this machine'"
# The old bare `git push origin refs/notes/ci-local` errored with "src refspec
# does not match any" here, and the caller then told the operator the evidence
# existed only on the merging machine while prescribing a command that failed
# the same way. It is in fact already published.
read -r clone sha <<<"$(setup_origin_case pushorigin)"
msg=$(ci_local_push_attestation "$clone"); rc=$?
{ [[ $rc -eq 0 ]] && grep -qF "already published" <<<"$msg"; } \
    && ok "origin-sourced note reports already-published" \
    || bad "origin-sourced push (rc=$rc, msg: $msg)"

echo "Test: local note + origin holding records we never fetched → push succeeds"
# git does NOT fetch refs/notes by default, so origin's notes ref routinely
# holds records this checkout has never seen — a bare push is then rejected
# non-fast-forward. Reconciling with cat_sort_uniq first makes it a fast-forward
# and keeps BOTH sides' records.
read -r clone sha <<<"$(setup_origin_case pushdiverged)"
bare="$TMPROOT/pushdiverged_bare.git"
# Another machine attests a different commit and pushes it.
other="$TMPROOT/pushdiverged_other"
git clone -q "$bare" "$other"
git -C "$other" "${GIT_ID[@]}" commit -q --allow-empty -m other
othersha=$(head_of "$other")
git -C "$other" push -q origin HEAD:refs/heads/other-branch
# That machine has origin's notes (it fetched them) and appends to them.
git -C "$other" fetch -q origin '+refs/notes/ci-local:refs/notes/ci-local'
add_note "$other" "$othersha" "$(note_body 'ci-local: pass' full)"
git -C "$other" push -q origin refs/notes/ci-local
# Meanwhile this checkout has its own local record it has not published.
git -C "$clone" "${GIT_ID[@]}" commit -q --allow-empty -m local
localsha=$(head_of "$clone")
git -C "$clone" push -q origin HEAD:refs/heads/local-branch
add_note "$clone" "$localsha" "$(note_body 'ci-local: pass' full)"
msg=$(ci_local_push_attestation "$clone"); rc=$?
{ [[ $rc -eq 0 ]] && grep -qF "pushed" <<<"$msg"; } \
    && ok "diverged notes ref reconciled and pushed" \
    || bad "diverged push (rc=$rc, msg: $msg)"
# Both records must survive the union merge.
git clone -q "$bare" "$TMPROOT/pushdiverged_verify"
git -C "$TMPROOT/pushdiverged_verify" fetch -q origin '+refs/notes/ci-local:refs/notes/ci-local'
{ git -C "$TMPROOT/pushdiverged_verify" notes --ref=ci-local show "$othersha" >/dev/null 2>&1 \
  && git -C "$TMPROOT/pushdiverged_verify" notes --ref=ci-local show "$localsha" >/dev/null 2>&1; } \
    && ok "both sides' records survive on origin" \
    || bad "a record was lost by the reconciliation"

echo "Test: an unreachable origin → push reports the failure, never a false success"
r=$(new_repo pushfail); s=$(head_of "$r")
git -C "$r" remote add origin "$TMPROOT/does_not_exist.git"
add_note "$r" "$s" "$(note_body 'ci-local: pass' full)"
msg=$(ci_local_push_attestation "$r"); rc=$?
{ [[ $rc -ne 0 ]] && grep -qF "could not push" <<<"$msg"; } \
    && ok "push failure surfaced with git's own reason" \
    || bad "push failure (rc=$rc, msg: $msg)"

echo "Test: missing arguments → no-attestation (never a silent pass)"
assert_verdict "no args rejected" no-attestation "" "" "internal error"

echo ""
echo "========================================"
echo "Passed: $PASS   Failed: $FAIL"
echo "========================================"
[[ $FAIL -eq 0 ]]
