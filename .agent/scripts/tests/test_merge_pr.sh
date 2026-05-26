#!/bin/bash
# .agent/scripts/tests/test_merge_pr.sh
# Tests for merge_pr.sh resolution + guard logic.
#
# Scope: the deterministic, no-network paths — argument handling and the
# worktree/issue resolution dispatch (the bug-prone part flagged in the #488
# plan review). The CI-wait, the actual `gh pr merge`, and the field-mode guard
# (which needs a real field-origin repo) are integration-level and intentionally
# NOT exercised here — mocking `gh pr checks --watch` / a live merge is
# drift-prone for little value (same call the upstream merge_pr test made).
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

echo ""
echo "========================================"
echo "Passed: $PASS   Failed: $FAIL"
echo "========================================"
[[ $FAIL -eq 0 ]]
