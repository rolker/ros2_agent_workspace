#!/bin/bash
# .agent/scripts/tests/test_merge_pr.sh
# Tests for merge_pr.sh resolution + guard logic.
#
# Scope: the deterministic, no-network paths — argument handling, the
# worktree/issue resolution dispatch (the bug-prone part flagged in the #488
# plan review), and the field-mode guard (exercised with a stubbed `gh` to prove
# no GitHub call escapes on a non-GitHub origin). The CI-wait and the actual
# `gh pr merge` are integration-level and intentionally NOT exercised here —
# mocking `gh pr checks --watch` / a live merge is drift-prone for little value
# (same call the upstream merge_pr test made).
#
# Run: bash .agent/scripts/tests/test_merge_pr.sh

set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MERGE_PR="$(cd "$SCRIPT_DIR/.." && pwd)/merge_pr.sh"
ROOT_DIR="$(git -C "$SCRIPT_DIR" worktree list --porcelain | head -n1 | sed 's/^worktree //')"

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

echo ""
echo "========================================"
echo "Passed: $PASS   Failed: $FAIL"
echo "========================================"
[[ $FAIL -eq 0 ]]
