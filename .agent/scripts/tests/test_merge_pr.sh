#!/bin/bash
# .agent/scripts/tests/test_merge_pr.sh
# Tests for merge_pr.sh resolution + guard logic.
#
# Scope: the deterministic, no-network paths — argument handling, the
# worktree/issue resolution dispatch (the bug-prone part flagged in the #488
# plan review), the field-mode guard (exercised with a stubbed `gh` to prove no
# GitHub call escapes on a non-GitHub origin), and — since #610 — the
# verification classification and its precedence rules, driven end-to-end
# through a stubbed `gh` (and a stubbed `make` for the one case that runs to
# completion). A live `gh pr checks --watch` against real Actions remains out
# of scope.
#
# Run: bash .agent/scripts/tests/test_merge_pr.sh

set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MERGE_PR="$(cd "$SCRIPT_DIR/.." && pwd)/merge_pr.sh"
ROOT_DIR="$(git -C "$SCRIPT_DIR" worktree list --porcelain | head -n1 | sed 's/^worktree //')"
# Fixtures below are created INSIDE the real workspace root (merge_pr.sh only
# looks for project repos under $ROOT_DIR/layers/main). An empty or wrong
# ROOT_DIR would make the cleanup `rm -rf` operate on a path built from nothing,
# so refuse to run at all unless it resolved to a real git worktree root.
if [[ -z "$ROOT_DIR" || ! -e "$ROOT_DIR/.git" ]]; then
    echo "FATAL: could not resolve the workspace root (got '${ROOT_DIR:-<empty>}') — refusing to run." >&2
    exit 1
fi

PASS=0; FAIL=0
ok()  { echo "  ✅ $1"; PASS=$((PASS+1)); }
bad() { echo "  ❌ $1"; FAIL=$((FAIL+1)); }

# Assert: running $MERGE_PR with given args (from given cwd) exits non-zero and
# its combined output contains the expected substring.
assert_err() {
    local desc="$1" cwd="$2" expect="$3"; shift 3
    local out rc
    out=$(cd "$cwd" && "$MERGE_PR" "$@" 2>&1); rc=$?
    if [[ $rc -ne 0 ]] && grep -qF "$expect" <<<"$out"; then
        ok "$desc"
    else
        bad "$desc (rc=$rc, output: $(head -1 <<<"$out"))"
    fi
}

echo "Test: merge_pr.sh is executable + syntactically valid"
[[ -x "$MERGE_PR" ]] && ok "executable" || bad "not executable"
bash -n "$MERGE_PR" && ok "bash -n clean" || bad "bash -n failed"

echo "Test: unknown argument → usage error (exit 2)"
out=$("$MERGE_PR" --bogus 2>&1); rc=$?
{ [[ $rc -eq 2 ]] && grep -qF "unknown argument" <<<"$out" && grep -qF "Usage:" <<<"$out"; } \
    && ok "unknown arg rejected with usage" || bad "unknown arg (rc=$rc)"

echo "Test: run from main tree (not a feature worktree) → error"
assert_err "main-tree cwd rejected" "$ROOT_DIR" "not inside a feature worktree"

echo "Test: bogus --issue (no matching worktree) → error"
assert_err "bogus --issue rejected" "$ROOT_DIR" "no worktree found for issue #99999" --issue 99999

echo "Test: bogus --issue with --repo-slug → error names the slug"
assert_err "bogus --issue+slug rejected" "$ROOT_DIR" "repo-slug nonesuch" --issue 99999 --repo-slug nonesuch

echo "Test: cwd on a non-feature branch → can't-derive-issue error"
tmp=$(mktemp -d)
git -C "$tmp" init -q
git -C "$tmp" -c user.email="t@t" -c user.name="t" commit -q --allow-empty -m init
assert_err "non-feature branch rejected" "$tmp" "is not 'feature/issue-"
rm -rf "$tmp"

echo "Test: missing value for --issue → usage error (exit 2)"
out=$("$MERGE_PR" --issue 2>&1); rc=$?
{ [[ $rc -eq 2 ]] && grep -qF "missing value for --issue" <<<"$out"; } \
    && ok "missing --issue value rejected" || bad "missing --issue value (rc=$rc)"

echo "Test: --pr with unknown --repo-slug → error (no silent workspace fallback)"
assert_err "unknown --repo-slug rejected" "$ROOT_DIR" "not found under layers/main" --pr 1 --repo-slug nonesuch_slug_xyz

echo "Test: --pr without --repo-slug → usage error (PR #s are per-repo, exit 2)"
out=$("$MERGE_PR" --pr 1 2>&1); rc=$?
{ [[ $rc -eq 2 ]] && grep -qF "requires --repo-slug" <<<"$out"; } \
    && ok "bare --pr requires --repo-slug" || bad "bare --pr (rc=$rc, out: $(head -1 <<<"$out"))"

# A multi-repo layer worktree (`--packages a,b` spanning different repos) holds
# >1 inner git repo. The inner-repo pick must NOT silently grab the first .git
# (wrong-repo merge/branch-delete); it must error deterministically. Fabricate a
# fake layer worktree (two inner .git dirs) under layers/worktrees so the no-slug
# issue-glob (`issue-*-<N>`) matches exactly it and reaches repo_path_in_worktree.
echo "Test: --issue on a multi-repo layer worktree → deterministic ambiguity error"
fakewt="$ROOT_DIR/layers/worktrees/issue-faketest-88888"
mkdir -p "$fakewt/overlay_ws/src/repo_a/.git" "$fakewt/overlay_ws/src/repo_b/.git"
assert_err "multi-repo worktree errors, not first-.git" "$ROOT_DIR" "holds multiple package repos" --issue 88888
rm -rf "$fakewt"

# worktree dirs are named with the SANITIZED slug (issue-my_pkg-N), so a hyphenated
# --repo-slug must be sanitized before lookup or the dir is never found. Fabricate
# the sanitized-name dir and pass the hyphenated slug; the lookup must MATCH it
# (proven by NOT getting "no worktree found" — it then fails later, off-slug).
echo "Test: --issue + hyphenated --repo-slug → sanitized before worktree lookup"
fakewt="$ROOT_DIR/layers/worktrees/issue-my_pkg-88888"
mkdir -p "$fakewt/overlay_ws/src/my_pkg/.git"
out=$(cd "$ROOT_DIR" && "$MERGE_PR" --issue 88888 --repo-slug my-pkg 2>&1); rc=$?
{ [[ $rc -ne 0 ]] && ! grep -qF "no worktree found for issue #88888" <<<"$out"; } \
    && ok "hyphenated slug sanitized for lookup" || bad "hyphenated slug lookup (rc=$rc, out: $(head -1 <<<"$out"))"
rm -rf "$fakewt"

# Legacy bare workspace worktree (.workspace-worktrees/issue-<N>, no slug) predates
# the issue-workspace-<N> convention. From cwd-mode, merge-pr must fail BEFORE the
# merge (worktree_remove --repo-slug workspace can't target it; dropping the slug
# risks the R4 collision). Fabricate a real bare worktree-shaped git repo on a
# feature branch and assert the pre-merge guard fires.
echo "Test: cwd in legacy bare workspace worktree → fail before merge"
legacywt="$ROOT_DIR/.workspace-worktrees/issue-88888"
mkdir -p "$legacywt"
git -C "$legacywt" init -q
git -C "$legacywt" -c user.email="t@t" -c user.name="t" commit -q --allow-empty -m init
git -C "$legacywt" checkout -q -b feature/issue-88888 2>/dev/null
out=$(cd "$legacywt" && "$MERGE_PR" 2>&1); rc=$?
{ [[ $rc -ne 0 ]] && grep -qF "legacy worktree dir 'issue-88888'" <<<"$out"; } \
    && ok "legacy bare worktree rejected pre-merge" || bad "legacy guard (rc=$rc, out: $(head -1 <<<"$out"))"
rm -rf "$legacywt"

# The field-mode guard is the key safety feature: on a non-GitHub origin (field
# repo, no GitHub PR) merge-pr must refuse BEFORE making any `gh` call. Exercise
# it without network — a temp git repo shaped like a cwd worktree (feature branch
# + gitcloud origin), with `gh` stubbed to a loud failure on PATH. The guard must
# fire (field-mode error, non-zero) and the stub's sentinel must NOT appear.
echo "Test: field-mode origin (cwd) → refuses before any gh call"
fieldwt=$(mktemp -d "/tmp/mergepr_fieldtest.XXXXXX")
git -C "$fieldwt" init -q
git -C "$fieldwt" -c user.email="t@t" -c user.name="t" commit -q --allow-empty -m init
git -C "$fieldwt" checkout -q -b feature/issue-88888 2>/dev/null
git -C "$fieldwt" remote add origin "git@gitcloud:field/mergepr_fieldtest.git"
ghstub=$(mktemp -d)
cat >"$ghstub/gh" <<'STUB'
#!/bin/bash
echo "GH_WAS_CALLED" >&2
exit 99
STUB
chmod +x "$ghstub/gh"
out=$(cd "$fieldwt" && PATH="$ghstub:$PATH" "$MERGE_PR" 2>&1); rc=$?
{ [[ $rc -ne 0 ]] && grep -qF "is a field-mode repo" <<<"$out" && ! grep -qF "GH_WAS_CALLED" <<<"$out"; } \
    && ok "field-mode refused before any gh call" || bad "field-mode guard (rc=$rc, out: $(head -1 <<<"$out"))"
rm -rf "$fieldwt" "$ghstub"

# A `colcon build` creates artifact dirs at <layer>_ws/install/<pkg> and
# <layer>_ws/build/<pkg> with the SAME depth+name as the source repo at
# <layer>_ws/src/<pkg>. The slug `find` must resolve the source under src/, not
# an artifact (issue #514). Contrast test (no reliance on filesystem ordering):
# make install/testpkg a *github*-origin repo and src/testpkg a *gitcloud*
# (field-mode) repo, then assert the field-mode guard FIRES — which can only
# happen if src/testpkg was resolved. A regression to install/testpkg (github
# origin) would proceed PAST the field-mode guard, a distinguishable outcome.
echo "Test: --repo-slug resolves src/ project repo, not colcon install/ artifact (#514)"
artwt="$ROOT_DIR/layers/main/testlayer_ws"
mkdir -p "$artwt/build/testpkg"
git -C "$ROOT_DIR" init -q "$artwt/install/testpkg"
git -C "$artwt/install/testpkg" remote add origin "git@github.com:test/testpkg.git"
git -C "$ROOT_DIR" init -q "$artwt/src/testpkg"
git -C "$artwt/src/testpkg" remote add origin "git@gitcloud:field/testpkg.git"
ghstub514=$(mktemp -d)
cat >"$ghstub514/gh" <<'STUB'
#!/bin/bash
echo "GH_WAS_CALLED" >&2
exit 99
STUB
chmod +x "$ghstub514/gh"
out=$(cd "$ROOT_DIR" && PATH="$ghstub514:$PATH" "$MERGE_PR" --pr 1 --repo-slug testpkg 2>&1); rc=$?
{ [[ $rc -ne 0 ]] && grep -qF "is a field-mode repo" <<<"$out" && ! grep -qF "GH_WAS_CALLED" <<<"$out"; } \
    && ok "src/ repo resolved (field-mode guard fired), not install/ artifact" \
    || bad "src/ vs install/ resolution (#514) (rc=$rc, out: $(head -1 <<<"$out"))"
rm -rf "$artwt" "$ghstub514"

echo "Test: --issue and --pr together → mutually-exclusive usage error (exit 2)"
out=$("$MERGE_PR" --issue 5 --pr 5 --repo-slug workspace 2>&1); rc=$?
{ [[ $rc -eq 2 ]] && grep -qF "mutually exclusive" <<<"$out"; } \
    && ok "conflicting --issue/--pr rejected" || bad "conflicting flags (rc=$rc, out: $(head -1 <<<"$out"))"

# ---------------------------------------------------------------------------
# Verification classification (#610). merge_pr.sh no longer treats every
# non-zero `gh pr checks` as "CI failed": it classifies four states and only
# one of them merges without hosted checks. These assert the OUTCOMES, not
# merely that the old call is skipped — the outcomes are what will rot.
#
# Shape: a temp project repo under layers/main/<layer>_ws/src/<slug> with a
# github origin, reached via `--pr <N> --repo-slug <slug>`, with `gh` stubbed on
# PATH. The stub's `pr merge` prints MERGE_ATTEMPTED and fails, so each case
# stops right after the verification block — no real merge, worktree removal, or
# `make sync`. Settle polling is driven to zero so nothing sleeps.
# ---------------------------------------------------------------------------
CI_TESTLAYER="$ROOT_DIR/layers/main/citest_ws"
ci_stubdir=""
ci_repo=""

# Never clobber something real: the fixture layer must not already exist.
if [[ -e "$CI_TESTLAYER" ]]; then
    echo "FATAL: $CI_TESTLAYER already exists — refusing to run the #610 cases." >&2
    exit 1
fi
cleanup_ci_case() {
    [[ -n "$CI_TESTLAYER" ]] && rm -rf "$CI_TESTLAYER"
    [[ -n "$ci_stubdir"  ]] && rm -rf "$ci_stubdir"
    return 0
}
# A failed assertion, an abort, or a Ctrl-C must not leave a fake layer behind
# inside the real workspace.
trap cleanup_ci_case EXIT INT TERM

make_ci_stub() {
    ci_stubdir=$(mktemp -d)
    cat >"$ci_stubdir/gh" <<'STUB'
#!/bin/bash
# Minimal gh stub for merge_pr.sh's verification block. Behavior is driven by
# STUB_* env vars; anything unexpected fails loudly rather than silently.
args="$*"
case "$1 $2" in
    "pr view")
        if [[ "$args" == *statusCheckRollup* ]]; then
            [[ "${STUB_VIEW_FAILS:-0}" == 1 ]] && { echo "gh: auth required" >&2; exit 1; }
            echo "{\"statusCheckRollup\":${STUB_ROLLUP:-[]}}"; exit 0
        fi
        if [[ "$args" == *headRefOid* ]]; then echo "${STUB_HEAD_SHA:-}"; exit 0; fi
        if [[ "$args" == *--json*url* ]]; then echo "https://example.invalid/pr/1"; exit 0; fi
        echo "{\"number\":1,\"state\":\"OPEN\",\"headRefName\":\"feature/issue-88888\"}"; exit 0 ;;
    "api repos"*|"api "*)
        if [[ "$args" == *".github/workflows"* ]]; then
            case "${STUB_WORKFLOWS:-404}" in
                listing) echo '[{"name":"validate.yml","type":"file"}]'; exit 0 ;;
                error)   echo "gh: connection refused" >&2; exit 1 ;;
                *)       echo "gh: Not Found (HTTP 404)" >&2; exit 1 ;;
            esac
        fi
        # Control probe: the repository ROOT at the same ref. A 404 here means
        # the workflows 404 proves nothing (no Contents read, or an unknown ref).
        case "${STUB_ROOT_CONTENTS:-ok}" in
            ok) echo '[{"name":"README.md","type":"file"}]'; exit 0 ;;
            *)  echo "gh: Not Found (HTTP 404)" >&2; exit 1 ;;
        esac ;;
    "pr checks")
        echo "CHECKS_WATCHED"; exit "${STUB_CHECKS_RC:-0}" ;;
    "pr merge")
        echo "MERGE_ATTEMPTED"; exit "${STUB_MERGE_RC:-1}" ;;
esac
echo "UNEXPECTED_GH_CALL: $args" >&2; exit 99
STUB
    chmod +x "$ci_stubdir/gh"
    # `make -C <root> sync` runs only in the cases that go all the way to the
    # end of the script; stub it so no test ever syncs the real workspace.
    cat >"$ci_stubdir/make" <<'MSTUB'
#!/bin/bash
echo "SYNC_STUBBED: $*"
MSTUB
    chmod +x "$ci_stubdir/make"
}

make_ci_repo() {   # a github-origin project repo with one commit
    ci_repo="$CI_TESTLAYER/src/citestpkg"
    mkdir -p "$ci_repo"
    git -C "$ci_repo" init -q
    git -C "$ci_repo" remote add origin "git@github.com:test/citestpkg.git"
    git -C "$ci_repo" -c user.email="t@t" -c user.name="t" commit -q --allow-empty -m init
}

# A full-scope ci-local note on the given commit, in ci_local.sh's format.
attest_ci_repo() {   # attest_ci_repo <sha>
    git -C "$ci_repo" -c user.email="t@t" -c user.name="t" notes --ref=ci-local add -m \
        "ci-local: pass
repo: citestpkg
commit: $1
scope: full
steps: template" "$1"
}

# Run merge_pr.sh against the fake project repo with the stub active.
# GIT_SSH_COMMAND=/bin/false keeps every git operation that would reach the
# network (the attested case's notes fetch/push, the remote branch delete) off
# the wire — they fail fast into their documented paths.
run_ci_case() {
    ( cd "$ROOT_DIR" && \
      PATH="$ci_stubdir:$PATH" \
      GIT_SSH_COMMAND=/bin/false \
      MERGE_PR_SETTLE_ATTEMPTS=1 MERGE_PR_SETTLE_SECONDS=0 \
      "$@" "$MERGE_PR" --pr 1 --repo-slug citestpkg 2>&1 )
}

echo "Test: no CI workflows at head + no attestation → warns loudly, still merges (#610)"
make_ci_stub; make_ci_repo
sha=$(git -C "$ci_repo" rev-parse HEAD)
out=$(STUB_HEAD_SHA="$sha" STUB_WORKFLOWS=404 run_ci_case env)
{ grep -qF "NO automated verification" <<<"$out" \
  && grep -qF "ci_local.sh $ci_repo" <<<"$out" \
  && grep -qF "MERGE_ATTEMPTED" <<<"$out"; } \
    && ok "third state: named warning naming the recourse, merge proceeds" \
    || bad "third state (out: $(head -3 <<<"$out" | tr '\n' '|'))"
cleanup_ci_case

echo "Test: the recourse never names a checkout that is not on the PR head (#610)"
# The bug: it printed `ci_local.sh $BRANCH_REPO` — the MAIN checkout, which sits
# on the default branch — so following the instruction attests the wrong commit
# and the re-run still reports no attestation.
make_ci_stub; make_ci_repo
sha=$(git -C "$ci_repo" rev-parse HEAD)
git -C "$ci_repo" -c user.email="t@t" -c user.name="t" commit -q --allow-empty -m moved-on
out=$(STUB_HEAD_SHA="$sha" STUB_WORKFLOWS=404 run_ci_case env)
{ grep -qF "sitting on $sha" <<<"$out" \
  && ! grep -qF "ci_local.sh $ci_repo" <<<"$out"; } \
    && ok "recourse points at a checkout of the head, not the default branch" \
    || bad "recourse path (out: $(head -6 <<<"$out" | tr '\n' '|'))"
cleanup_ci_case

echo "Test: no CI workflows at head + full-scope attestation → merges on the note (#610)"
make_ci_stub; make_ci_repo
sha=$(git -C "$ci_repo" rev-parse HEAD)
attest_ci_repo "$sha"
out=$(STUB_HEAD_SHA="$sha" STUB_WORKFLOWS=404 run_ci_case env)
{ grep -qF "full-scope ci-local attestation" <<<"$out" \
  && grep -qF "Pushing refs/notes/ci-local" <<<"$out" \
  && grep -qF "MERGE_ATTEMPTED" <<<"$out"; } \
    && ok "attested state: merges on the note and pushes it (ADR-0018 d5)" \
    || bad "attested state (out: $(head -3 <<<"$out" | tr '\n' '|'))"
cleanup_ci_case

echo "Test: workflows exist but no checks registered → fail-closed, no merge (#610)"
# The empty-rollup race: a head pushed moments ago, or paths-filtered workflows.
# This must NOT be read as "no CI configured" — that would be fail-open.
make_ci_stub; make_ci_repo
sha=$(git -C "$ci_repo" rev-parse HEAD)
out=$(STUB_HEAD_SHA="$sha" STUB_WORKFLOWS=listing run_ci_case env)
{ grep -qF "no checks have" <<<"$out" && ! grep -qF "MERGE_ATTEMPTED" <<<"$out"; } \
    && ok "empty-rollup race stays fail-closed" \
    || bad "empty-rollup race (out: $(head -3 <<<"$out" | tr '\n' '|'))"
cleanup_ci_case

echo "Test: checks present → watched as before, then merges (#610 leaves this path alone)"
make_ci_stub; make_ci_repo
sha=$(git -C "$ci_repo" rev-parse HEAD)
out=$(STUB_HEAD_SHA="$sha" STUB_ROLLUP='[{"name":"validate"}]' run_ci_case env)
{ grep -qF "CHECKS_WATCHED" <<<"$out" && grep -qF "MERGE_ATTEMPTED" <<<"$out"; } \
    && ok "checks-present path unchanged" \
    || bad "checks-present path (out: $(head -3 <<<"$out" | tr '\n' '|'))"
cleanup_ci_case

echo "Test: red checks → refuses (#610 must not weaken the existing gate)"
make_ci_stub; make_ci_repo
sha=$(git -C "$ci_repo" rev-parse HEAD)
out=$(STUB_HEAD_SHA="$sha" STUB_ROLLUP='[{"name":"validate"}]' STUB_CHECKS_RC=1 run_ci_case env)
{ grep -qF "CI checks failed" <<<"$out" && ! grep -qF "MERGE_ATTEMPTED" <<<"$out"; } \
    && ok "failed checks still block the merge" \
    || bad "failed checks (out: $(head -3 <<<"$out" | tr '\n' '|'))"
cleanup_ci_case

echo "Test: gh failure is an error, never 'no CI configured' (#610)"
# Auth expiry / network loss / rate limiting must fail closed. Classifying them
# as an empty rollup would be a second fail-open path.
make_ci_stub; make_ci_repo
sha=$(git -C "$ci_repo" rev-parse HEAD)
out=$(STUB_HEAD_SHA="$sha" STUB_VIEW_FAILS=1 run_ci_case env)
{ grep -qF "could not read check status" <<<"$out" \
  && ! grep -qF "NO automated verification" <<<"$out" \
  && ! grep -qF "MERGE_ATTEMPTED" <<<"$out"; } \
    && ok "gh failure errors out, no merge" \
    || bad "gh failure (out: $(head -3 <<<"$out" | tr '\n' '|'))"
cleanup_ci_case

echo "Test: probe failure (not a 404) is an error, never 'no CI configured' (#610)"
make_ci_stub; make_ci_repo
sha=$(git -C "$ci_repo" rev-parse HEAD)
out=$(STUB_HEAD_SHA="$sha" STUB_WORKFLOWS=error run_ci_case env)
{ grep -qF "could not probe .github/workflows" <<<"$out" \
  && ! grep -qF "MERGE_ATTEMPTED" <<<"$out"; } \
    && ok "unrunnable probe errors out, no merge" \
    || bad "probe failure (out: $(head -3 <<<"$out" | tr '\n' '|'))"
cleanup_ci_case

echo "Test: workspace repo with an empty rollup → fail-closed (ADR-0018 decision 4)"
# The workspace repo's hosted checks are REQUIRED; a ci-local attestation never
# substitutes. Without this gate the empty-rollup race would warn-and-merge on
# this very repo — most likely right after a push, which is when it fires.
make_ci_stub
WS_SHA=$(printf 'd%.0s' {1..40})
out=$( cd "$ROOT_DIR" && PATH="$ci_stubdir:$PATH" \
       GIT_SSH_COMMAND=/bin/false \
       MERGE_PR_SETTLE_ATTEMPTS=1 MERGE_PR_SETTLE_SECONDS=0 \
       STUB_HEAD_SHA="$WS_SHA" STUB_WORKFLOWS=404 \
       "$MERGE_PR" --pr 1 --repo-slug workspace 2>&1 )
{ grep -qF "hosted checks are required" <<<"$out" \
  && ! grep -qF "NO automated verification" <<<"$out" \
  && ! grep -qF "MERGE_ATTEMPTED" <<<"$out"; } \
    && ok "workspace repo never takes the substitution path" \
    || bad "workspace-repo exemption (out: $(head -3 <<<"$out" | tr '\n' '|'))"
cleanup_ci_case


# ---------------------------------------------------------------------------
# ADR-0018 decision 1, in full (#610): a full-scope attestation satisfies the
# gate for a PROJECT repo whether or not the repo has workflows — but it never
# overrides a RED hosted signal ("never merge past a red signal from whichever
# verification applies"). These pin the precedence.
# ---------------------------------------------------------------------------
FAILING_ROLLUP='[{"status":"COMPLETED","conclusion":"FAILURE"}]'
PENDING_ROLLUP='[{"status":"IN_PROGRESS","conclusion":null}]'
PASSING_ROLLUP='[{"status":"COMPLETED","conclusion":"SUCCESS"}]'

echo "Test: RED hosted checks + a valid attestation → refuses (#610 precedence)"
# The case that must never regress: an attestation is not a way past a red
# signal. If this ever merges, the widening has become a loophole.
make_ci_stub; make_ci_repo
sha=$(git -C "$ci_repo" rev-parse HEAD)
attest_ci_repo "$sha"
out=$(STUB_HEAD_SHA="$sha" STUB_ROLLUP="$FAILING_ROLLUP" run_ci_case env)
{ grep -qF "CI checks failed" <<<"$out" \
  && grep -qF "does NOT override a red hosted signal" <<<"$out" \
  && ! grep -qF "MERGE_ATTEMPTED" <<<"$out"; } \
    && ok "attestation cannot override red hosted checks" \
    || bad "red+attestation (out: $(head -4 <<<"$out" | tr '\n' '|'))"
cleanup_ci_case

echo "Test: PENDING hosted checks + a valid attestation → merges without waiting"
make_ci_stub; make_ci_repo
sha=$(git -C "$ci_repo" rev-parse HEAD)
attest_ci_repo "$sha"
out=$(STUB_HEAD_SHA="$sha" STUB_ROLLUP="$PENDING_ROLLUP" run_ci_case env)
{ grep -qF "hosted checks are still pending" <<<"$out" \
  && ! grep -qF "CHECKS_WATCHED" <<<"$out" \
  && grep -qF "MERGE_ATTEMPTED" <<<"$out"; } \
    && ok "attestation satisfies the gate while checks are pending (ADR-0018 d1)" \
    || bad "pending+attestation (out: $(head -4 <<<"$out" | tr '\n' '|'))"
cleanup_ci_case

echo "Test: PENDING hosted checks, no attestation → waits on them as before"
make_ci_stub; make_ci_repo
sha=$(git -C "$ci_repo" rev-parse HEAD)
out=$(STUB_HEAD_SHA="$sha" STUB_ROLLUP="$PENDING_ROLLUP" run_ci_case env)
{ grep -qF "CHECKS_WATCHED" <<<"$out" && grep -qF "MERGE_ATTEMPTED" <<<"$out"; } \
    && ok "pending without evidence still waits" \
    || bad "pending, no attestation (out: $(head -4 <<<"$out" | tr '\n' '|'))"
cleanup_ci_case

echo "Test: PASSING hosted checks + an attestation → normal merge, no shortcut"
make_ci_stub; make_ci_repo
sha=$(git -C "$ci_repo" rev-parse HEAD)
attest_ci_repo "$sha"
out=$(STUB_HEAD_SHA="$sha" STUB_ROLLUP="$PASSING_ROLLUP" run_ci_case env)
{ grep -qF "CHECKS_WATCHED" <<<"$out" && grep -qF "MERGE_ATTEMPTED" <<<"$out" \
  && ! grep -qF "still pending" <<<"$out"; } \
    && ok "green checks path unchanged by the widening" \
    || bad "passing+attestation (out: $(head -4 <<<"$out" | tr '\n' '|'))"
cleanup_ci_case

echo "Test: workspace repo, pending checks → never shortcut (ADR-0018 decision 4)"
make_ci_stub
out=$( cd "$ROOT_DIR" && PATH="$ci_stubdir:$PATH" \
       GIT_SSH_COMMAND=/bin/false \
       MERGE_PR_SETTLE_ATTEMPTS=1 MERGE_PR_SETTLE_SECONDS=0 \
       STUB_HEAD_SHA="$(printf 'd%.0s' {1..40})" STUB_ROLLUP="$PENDING_ROLLUP" \
       "$MERGE_PR" --pr 1 --repo-slug workspace 2>&1 )
{ grep -qF "CHECKS_WATCHED" <<<"$out" && ! grep -qF "still pending" <<<"$out"; } \
    && ok "workspace repo always waits on its hosted checks" \
    || bad "workspace pending (out: $(head -4 <<<"$out" | tr '\n' '|'))"
cleanup_ci_case

echo "Test: a workflows 404 that could be a permission/bad-ref 404 → error, no merge"
# The 404 for "path absent", "no Contents read", and "unknown ref" are
# byte-identical. Without the control probe, a token with PR read but no
# Contents read classifies EVERY project repo as no-CI and merges it.
make_ci_stub; make_ci_repo
sha=$(git -C "$ci_repo" rev-parse HEAD)
out=$(STUB_HEAD_SHA="$sha" STUB_WORKFLOWS=404 STUB_ROOT_CONTENTS=404 run_ci_case env)
{ grep -qF "cannot tell whether" <<<"$out" \
  && ! grep -qF "NO automated verification" <<<"$out" \
  && ! grep -qF "MERGE_ATTEMPTED" <<<"$out"; } \
    && ok "unconfirmable 404 fails closed" \
    || bad "ambiguous 404 (out: $(head -4 <<<"$out" | tr '\n' '|'))"
cleanup_ci_case

echo "Test: a non-40-hex head sha is rejected before it is used as a ref"
make_ci_stub; make_ci_repo
out=$(STUB_HEAD_SHA="deadbeef" STUB_WORKFLOWS=404 run_ci_case env)
{ grep -qF "could not resolve a valid head commit" <<<"$out" \
  && ! grep -qF "MERGE_ATTEMPTED" <<<"$out"; } \
    && ok "short/garbage head sha fails closed" \
    || bad "head-sha validation (out: $(head -4 <<<"$out" | tr '\n' '|'))"
cleanup_ci_case

echo "Test: a non-integer settle knob is rejected (exit 2), nothing merged"
make_ci_stub; make_ci_repo
sha=$(git -C "$ci_repo" rev-parse HEAD)
out=$( cd "$ROOT_DIR" && PATH="$ci_stubdir:$PATH" GIT_SSH_COMMAND=/bin/false \
       MERGE_PR_SETTLE_ATTEMPTS='1$(touch /tmp/pwned610)' MERGE_PR_SETTLE_SECONDS=0 \
       STUB_HEAD_SHA="$sha" "$MERGE_PR" --pr 1 --repo-slug citestpkg 2>&1 ); rc=$?
{ [[ $rc -eq 2 ]] && grep -qF "must be a non-negative integer" <<<"$out" \
  && [[ ! -e /tmp/pwned610 ]]; } \
    && ok "settle knobs validated as integers" \
    || bad "settle knob validation (rc=$rc, out: $(head -2 <<<"$out" | tr '\n' '|'))"
rm -f /tmp/pwned610
cleanup_ci_case

echo "Test: merged with nothing verifying it → distinct exit code 42 + banner (#610)"
# Operator decision: this state stays warn-and-merge (refusing would block the
# 58% of project repos that have no CI, with no path forward), but it must be
# impossible to miss in a transcript.
make_ci_stub; make_ci_repo
sha=$(git -C "$ci_repo" rev-parse HEAD)
out=$(STUB_HEAD_SHA="$sha" STUB_WORKFLOWS=404 STUB_MERGE_RC=0 run_ci_case env); rc=$?
{ [[ $rc -eq 42 ]] \
  && grep -qF "MERGED WITHOUT VERIFICATION" <<<"$out" \
  && grep -qF "SYNC_STUBBED" <<<"$out"; } \
    && ok "unverified merge exits 42 after completing cleanup" \
    || bad "unverified exit code (rc=$rc, out: $(tail -6 <<<"$out" | tr '\n' '|'))"
cleanup_ci_case

echo "Test: a verified merge still exits 0"
make_ci_stub; make_ci_repo
sha=$(git -C "$ci_repo" rev-parse HEAD)
attest_ci_repo "$sha"
out=$(STUB_HEAD_SHA="$sha" STUB_WORKFLOWS=404 STUB_MERGE_RC=0 run_ci_case env); rc=$?
{ [[ $rc -eq 0 ]] && ! grep -qF "MERGED WITHOUT VERIFICATION" <<<"$out"; } \
    && ok "attested merge exits 0" \
    || bad "attested exit code (rc=$rc, out: $(tail -4 <<<"$out" | tr '\n' '|'))"
cleanup_ci_case

echo ""
echo "========================================"
echo "Passed: $PASS   Failed: $FAIL"
echo "========================================"
[[ $FAIL -eq 0 ]]
