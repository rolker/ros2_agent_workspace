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
# refs/notes/ci-local-merge-check makes the fetch fail non-fast-forward, and the
# helper would report a FALSE 'no attestation' — merging on a warning instead of
# on evidence.
read -r clone sha <<<"$(setup_origin_case leftover)"
git -C "$clone" "${GIT_ID[@]}" commit -q --allow-empty -m unrelated
unrelated=$(head_of "$clone")
git -C "$clone" "${GIT_ID[@]}" notes --ref=ci-local-merge-check add -m "stale junk" "$unrelated"
assert_verdict "stale scratch ref does not break the fetch" attested "$clone" "$sha"
git -C "$clone" rev-parse --verify -q refs/notes/ci-local-merge-check >/dev/null \
    && bad "scratch ref left behind after the call" \
    || ok "scratch ref cleaned up after the call"

echo "Test: note claims upstream-repo but the commit isn't local → no-attestation"
# We cannot read upstream.repos at that commit, so completeness is unverifiable;
# it must fail closed with a fetch hint rather than pass by default.
r=$(new_repo absent_head)
absent="0123456789012345678901234567890123456789"
add_note "$r" "$absent" "$(note_body 'ci-local: pass' full 'upstream-repo: alpha@1111111111111111111111111111111111111111')"
assert_verdict "unverifiable upstream coverage rejected" no-attestation "$r" "$absent" "fetch origin"

echo "Test: missing arguments → no-attestation (never a silent pass)"
assert_verdict "no args rejected" no-attestation "" "" "internal error"

echo ""
echo "========================================"
echo "Passed: $PASS   Failed: $FAIL"
echo "========================================"
[[ $FAIL -eq 0 ]]
