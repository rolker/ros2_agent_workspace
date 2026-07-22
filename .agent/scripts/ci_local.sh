#!/bin/bash
# ci_local.sh — run a project repo's CI equivalent locally in a container and
# record an auditable attestation (workspace#573, Phase 1 of workspace#572).
#
# Mirrors the workspace CI template semantics (.agent/templates/ci_workflow.yml):
# overlay workspace around the repo, rosdep install, colcon build (BUILD_TESTING),
# colcon test (overlay sourced), colcon test-result (always, like if: always()) —
# inside a throwaway container.
#
# What gets built:
#   attestable run (clean HEAD)  a pristine `git archive HEAD` snapshot — the
#                                exact commit content, no gitignored/untracked
#                                files — mirroring hosted CI's actions/checkout
#   dirty tree / --no-attest     the live working tree, mounted READ-ONLY
#                                (useful for pre-commit iteration; never attested)
#
# Images:
#   default            ros2-agent-workspace-agent:latest if present (rosdep deps
#                      baked at image build, #520 — fast path), else clean room
#   --clean-room       force ros:jazzy-ros-core, replicating hosted CI exactly
#                      (apt-installs ros-dev-tools at runtime; needs network)
#
# Per-repo extras beyond the template (e.g. URDF validation): executable hook
# at <repo>/.agents/ci_local_extra.sh, run inside the container from the
# workspace root after tests pass, with the overlay sourced.
#
# Attestation: on success of an attestable run, a record is added to a git note
# on the tested commit under refs/notes/ci-local (repo-local; never perturbs
# branches; push with `git push origin refs/notes/ci-local` if wanted). Records
# are APPENDED, never overwritten, so a clean-room record survives later
# agent-image re-runs. A run narrowed with --packages below the discovered set
# is recorded as `ci-local: pass (partial)` — consumers must check for the
# unqualified `ci-local: pass`. Inspect with: git notes --ref=ci-local show <commit>
# Full log: $CI_LOCAL_LOG_DIR (default <workspace>/.agent/scratchpad/ci_local/).
#
# Usage:
#   ci_local.sh <project_repo_path> [options]
#
# Options:
#   --image IMG      container image (overrides the default selection)
#   --clean-room     use ros:jazzy-ros-core (exact hosted-CI mirror)
#   --packages "A B" packages to build/test (default: all packages in the repo;
#                    a narrowed set is attested as partial)
#   --no-attest      run against the live tree; never write the attestation note
#   -n, --dry-run    print the run plan (image, packages, steps); run nothing
#   -h, --help       this help
#
# Exit codes: 0 = CI passed (attested if eligible); non-zero = failed/refused.

if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
    echo "Error: This script should be executed, not sourced."
    return 1
fi
set -euo pipefail

AGENT_IMAGE="ros2-agent-workspace-agent:latest"
CLEAN_IMAGE="ros:jazzy-ros-core"
NOTES_REF="ci-local"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
LOG_DIR="${CI_LOCAL_LOG_DIR:-$WORKSPACE_ROOT/.agent/scratchpad/ci_local}"

err() { echo "ci_local: ERROR: $*" >&2; }
warn() { echo "ci_local: WARN: $*" >&2; }

REPO=""
IMAGE=""
CLEAN_ROOM=0
PACKAGES=""
NO_ATTEST=0
DRY_RUN=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --image)      IMAGE="${2:?--image needs a value}"; shift 2 ;;
    --clean-room) CLEAN_ROOM=1; shift ;;
    --packages)   PACKAGES="${2:?--packages needs a value}"; shift 2 ;;
    --no-attest)  NO_ATTEST=1; shift ;;
    -n|--dry-run) DRY_RUN=1; shift ;;
    -h|--help)    sed -n '2,/^$/p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    -*)           err "unknown option: $1 (try --help)"; exit 1 ;;
    *)            [[ -z "$REPO" ]] || { err "unexpected argument: $1"; exit 1; }
                  REPO="$1"; shift ;;
  esac
done

[[ -n "$REPO" ]] || { err "usage: ci_local.sh <project_repo_path> [options]"; exit 1; }
[[ -d "$REPO" ]] || { err "repo path not found: $REPO"; exit 1; }
REPO="$(cd "$REPO" && pwd)"
REPO_NAME="$(basename "$REPO")"
# The path is embedded in a docker -v spec (colon/comma-delimited) and the name
# in the attestation record — whitespace/control chars or mount metacharacters
# would produce ambiguous mounts or malformed note records.
if [[ "$REPO" =~ [[:space:]:,] ]]; then
  err "repo path contains whitespace, ':' or ',' — unsupported for container mounts: $REPO"; exit 1
fi
git -C "$REPO" rev-parse --git-dir >/dev/null 2>&1 || { err "$REPO is not a git repository"; exit 1; }
# Refuse subdirectories: HEAD/attestation belong to the whole repo — attesting
# the repo's commit while mounting only a subtree would forge full-repo CI.
TOPLEVEL="$(git -C "$REPO" rev-parse --show-toplevel)"
[[ "$REPO" == "$TOPLEVEL" ]] || { err "$REPO is inside repo $TOPLEVEL — pass the repo root"; exit 1; }
command -v docker >/dev/null 2>&1 || { err "docker not found on PATH"; exit 1; }

# ---- image selection --------------------------------------------------------
if [[ -z "$IMAGE" ]]; then
  if [[ $CLEAN_ROOM -eq 1 ]]; then
    IMAGE="$CLEAN_IMAGE"
  elif docker image inspect "$AGENT_IMAGE" >/dev/null 2>&1; then
    IMAGE="$AGENT_IMAGE"
  else
    warn "agent image $AGENT_IMAGE not found — falling back to clean room ($CLEAN_IMAGE)"
    IMAGE="$CLEAN_IMAGE"
  fi
fi
# The image string is written verbatim into the attestation record; embedded
# whitespace/newlines could inject a spurious 'ci-local: pass' line and fool
# note consumers.
if [[ "$IMAGE" =~ [[:space:]] ]]; then
  err "--image must not contain whitespace: '$IMAGE'"; exit 1
fi

# ---- package discovery ------------------------------------------------------
# Mirror colcon discovery: skip hidden dirs and any subtree holding a
# COLCON_IGNORE marker (fixture/template packages colcon itself won't see).
discover_packages() {
  local pkgxml dir names=()
  while IFS= read -r pkgxml; do
    dir="$(dirname "$pkgxml")"
    local d="$dir" ignored=0
    while [[ "$d" == "$REPO"* ]]; do
      [[ -e "$d/COLCON_IGNORE" ]] && { ignored=1; break; }
      d="$(dirname "$d")"
    done
    [[ $ignored -eq 1 ]] && continue
    names+=("$(sed -n 's/.*<name>\(.*\)<\/name>.*/\1/p' "$pkgxml" | head -1)")
  done < <(find "$REPO" -name package.xml -not -path '*/.*' | sort)
  printf '%s\n' "${names[@]:-}" | sort -u | grep -v '^$' | tr '\n' ' '
}
DISCOVERED="$(discover_packages)"; DISCOVERED="${DISCOVERED% }"
PARTIAL=0
if [[ -z "$PACKAGES" ]]; then
  PACKAGES="$DISCOVERED"
else
  norm_req="$(tr ' ' '\n' <<<"$PACKAGES" | grep -v '^$' | sort -u | tr '\n' ' ')"
  norm_disc="$(tr ' ' '\n' <<<"$DISCOVERED" | grep -v '^$' | sort -u | tr '\n' ' ')"
  [[ "$norm_req" != "$norm_disc" ]] && PARTIAL=1
fi
[[ -n "$PACKAGES" ]] || { err "no ROS packages found under $REPO (no package.xml)"; exit 1; }

# Validate names before they are interpolated into the root inner script: a
# package.xml <name> (or --packages arg) containing shell metacharacters must
# never reach the container shell (attestation forgery via injection).
for p in $PACKAGES; do
  [[ "$p" =~ ^[A-Za-z0-9_-]+$ ]] || { err "invalid package name: '$p'"; exit 1; }
done

EXTRA_HOOK=""
[[ -x "$REPO/.agents/ci_local_extra.sh" ]] && EXTRA_HOOK=".agents/ci_local_extra.sh"

# ---- attestation eligibility ------------------------------------------------
HEAD_SHA="$(git -C "$REPO" rev-parse HEAD)"
DIRTY=0
if [[ -n "$(git -C "$REPO" status --porcelain)" ]]; then
  DIRTY=1
fi
ATTEST=1
[[ $NO_ATTEST -eq 1 || $DIRTY -eq 1 ]] && ATTEST=0

STEPS="template${EXTRA_HOOK:++extra-hook}"
PASS_LABEL="ci-local: pass"
[[ $PARTIAL -eq 1 ]] && PASS_LABEL="ci-local: pass (partial)"

if [[ $DRY_RUN -eq 1 ]]; then
  echo "=== ci_local plan (dry run) ==="
  echo "repo     : $REPO"
  echo "commit   : $HEAD_SHA$( [[ $DIRTY -eq 1 ]] && echo ' (DIRTY — will not attest)' )"
  echo "image    : $IMAGE"
  echo "packages : $PACKAGES$( [[ $PARTIAL -eq 1 ]] && echo ' (PARTIAL of: '"$DISCOVERED"')' )"
  echo "steps    : $STEPS"
  echo "source   : $( [[ $ATTEST -eq 1 ]] && echo "pristine snapshot of $HEAD_SHA" || echo 'live working tree (read-only)' )"
  echo "attest   : $( [[ $ATTEST -eq 1 ]] && echo "append '$PASS_LABEL' record, refs/notes/$NOTES_REF on $HEAD_SHA" || echo no )"
  echo "log dir  : $LOG_DIR"
  exit 0
fi

# ---- build source: pristine snapshot when attesting -------------------------
# Attestable runs must test exactly the commit content (hosted CI checks out a
# pristine tree): a live mount would let mid-run edits and gitignored files
# leak into an attested result. Dirty/no-attest runs use the live tree.
INNER="$(mktemp)"
SNAP=""
cleanup() { rm -f "$INNER"; if [[ -n "$SNAP" ]]; then rm -rf "$SNAP"; fi; }
trap cleanup EXIT
MOUNT_SRC="$REPO"
if [[ $ATTEST -eq 1 ]]; then
  SNAP="$(mktemp -d -t ci_local_snap.XXXXXX)"
  git -C "$REPO" archive "$HEAD_SHA" | tar -x -C "$SNAP"
  MOUNT_SRC="$SNAP"
fi

# ---- inner step script ------------------------------------------------------
# Same step sequence as the CI template; conditionals make it a no-op-fast on
# the agent image (tools and deps baked) and a faithful mirror on ros-core.
cat > "$INNER" <<EOF
# no -u: ROS setup.bash files reference unbound vars (AMENT_TRACE_SETUP_FILES)
set -eo pipefail
export DEBIAN_FRONTEND=noninteractive
if ! command -v colcon >/dev/null 2>&1; then
  apt-get update
  apt-get -y install ros-dev-tools
fi
source /opt/ros/jazzy/setup.bash
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi
# Tolerate rosdep update failure (offline field host with baked deps, #520):
# rosdep install -r below still resolves against the existing cache/apt state.
if [ ! -d "\$HOME/.ros/rosdep/sources.cache" ]; then
  rosdep update || echo "ci_local(inner): WARN: rosdep update failed (offline?) — proceeding with baked/existing deps"
fi
cd /ci
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to $PACKAGES --cmake-args -DBUILD_TESTING=ON
# local_setup (not setup): ROS is already sourced above; ADR-0016
source install/local_setup.bash
# Mirror the template's 'if: always()' test-result step: summarize even when
# tests fail, then propagate the test exit code.
test_rc=0
colcon test --packages-select $PACKAGES --event-handlers console_direct+ --return-code-on-test-failure || test_rc=\$?
colcon test-result --verbose || true
if [ "\$test_rc" -ne 0 ]; then exit "\$test_rc"; fi
EOF
if [[ -n "$EXTRA_HOOK" ]]; then
  cat >> "$INNER" <<EOF
bash "src/$REPO_NAME/$EXTRA_HOOK"
EOF
fi

# ---- run --------------------------------------------------------------------
mkdir -p "$LOG_DIR"
TS="$(date -u +%Y%m%dT%H%M%SZ)"
LOG="$LOG_DIR/${REPO_NAME}-${HEAD_SHA:0:7}-${TS}.log"
IMAGE_ID="$(docker image inspect --format '{{.Id}}' "$IMAGE" 2>/dev/null || echo unknown)"

echo "=== ci_local: $REPO_NAME @ ${HEAD_SHA:0:7} in $IMAGE ==="
echo "log: $LOG"
rc=0
# --entrypoint bash: the agent image's entrypoint is the agent-container
# harness (mount validation, ROS2_AGENT_WORKSPACE_ROOT guard); CI only needs
# the baked toolchain. Harmless on plain ros images.
docker run --rm --user root --entrypoint bash \
  -v "$MOUNT_SRC:/ci/src/$REPO_NAME:ro" \
  -v "$INNER:/ci_local_steps.sh:ro" \
  "$IMAGE" /ci_local_steps.sh 2>&1 | tee "$LOG" || rc=$?

if [[ $rc -ne 0 ]]; then
  err "CI FAILED (exit $rc) — log: $LOG"
  exit "$rc"
fi

# ---- attestation ------------------------------------------------------------
if [[ $NO_ATTEST -eq 1 ]]; then
  echo "=== ci_local: PASS (attestation skipped: --no-attest) ==="
  exit 0
fi
if [[ $DIRTY -eq 1 ]]; then
  warn "working tree dirty — PASS is not attestable (commit first, re-run to attest)"
  echo "=== ci_local: PASS (unattested) ==="
  exit 0
fi

LOG_HASH="$(sha256sum "$LOG" | cut -d' ' -f1)"
NOTE="$PASS_LABEL
repo: $REPO_NAME
commit: $HEAD_SHA
image: $IMAGE
image-id: $IMAGE_ID
packages: $PACKAGES
scope: $( [[ $PARTIAL -eq 1 ]] && echo partial || echo full )
steps: $STEPS
date: $(date -u +%Y-%m-%dT%H:%M:%SZ)
host: $(hostname)
log-sha256: $LOG_HASH"
# Append, never overwrite: a clean-room record must survive later fast-path
# re-runs on the same commit. Explicit tool identity: notes need a committer,
# and fresh hosts/containers/CI runners have no ambient git config — the tool
# is the attester (the host is already recorded in the note body).
NOTES_ID=(-c user.name=ci_local -c user.email=ci-local@localhost)
if git -C "$REPO" notes --ref="$NOTES_REF" show "$HEAD_SHA" >/dev/null 2>&1; then
  git -C "$REPO" "${NOTES_ID[@]}" notes --ref="$NOTES_REF" append -m "
---
$NOTE" "$HEAD_SHA"
else
  git -C "$REPO" "${NOTES_ID[@]}" notes --ref="$NOTES_REF" add -m "$NOTE" "$HEAD_SHA"
fi
echo "=== ci_local: PASS — attested on $HEAD_SHA (refs/notes/$NOTES_REF, scope: $( [[ $PARTIAL -eq 1 ]] && echo partial || echo full )) ==="
