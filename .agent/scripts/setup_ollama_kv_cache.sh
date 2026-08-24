#!/bin/bash
# setup_ollama_kv_cache.sh
# Writes a systemd drop-in setting OLLAMA_KV_CACHE_TYPE for the ollama
# service, halving KV-cache VRAM (f16 -> q8_0) on a VRAM-constrained host.
#
# WHY A DROP-IN AND NOT A PER-REQUEST OPTION: Ollama 0.32.0 has no
# per-request KV-cache-type override. Verified against the installed
# binary: `ollama serve --help` lists OLLAMA_KV_CACHE_TYPE only as a
# server environment variable; the `--cache-type-k` / `--cache-type-v`
# flags visible in the binary are llama-server CLI flags the runner
# derives from its own environment at subprocess launch; and no
# kv_cache_type JSON tag exists anywhere in the binary (unlike
# keep_alive and num_ctx, which local_review.sh does send per request).
# KV-cache quantization is fixed at model-load time by the server
# process's environment, full stop. See #605.
#
# THIS SCRIPT REQUIRES EXPLICIT OPERATOR SIGN-OFF. It writes under /etc
# with sudo and restarts a system service. That is host administration on
# the operator's own workstation, and it is not implied by approving the
# issue or plan that produced this script. It therefore refuses to do
# anything without --yes: a dry run is the default, and there is no
# interactive prompt to fat-finger through.
#
# Usage:
#   setup_ollama_kv_cache.sh                # dry run: show what would change
#   setup_ollama_kv_cache.sh --yes          # actually apply (needs sudo)
#   setup_ollama_kv_cache.sh --yes --type q4_0
#   setup_ollama_kv_cache.sh --revert --yes # remove the drop-in
#
# Idempotent: re-running with the same type is a no-op that neither
# rewrites the file nor restarts the service.
#
# Exit codes:
#   0  applied, already in the requested state, or dry run completed
#   1  usage / environment error (no systemd, no ollama unit, bad type)

if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
    echo "Error: This script should be executed, not sourced." >&2
    echo "  Run: ${BASH_SOURCE[0]} $*" >&2
    return 1
fi

set -euo pipefail

DROPIN_DIR="/etc/systemd/system/ollama.service.d"
DROPIN_FILE="$DROPIN_DIR/kv-cache.conf"
CACHE_TYPE="q8_0"
APPLY=false
REVERT=false

# q4_0 is offered but is a quality tradeoff, not a free win — KV cache at
# 4 bits measurably degrades long-context recall. f16 is the Ollama
# default and is what "revert" restores by removing the drop-in.
VALID_TYPES="f16 q8_0 q4_0"

while [[ $# -gt 0 ]]; do
    case $1 in
        --yes) APPLY=true; shift ;;
        --revert) REVERT=true; shift ;;
        --type)
            if [[ -z "${2:-}" ]]; then
                echo "Error: --type requires one of: $VALID_TYPES" >&2
                exit 1
            fi
            CACHE_TYPE="$2"
            shift 2
            ;;
        -h|--help)
            awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "${BASH_SOURCE[0]}"
            exit 0
            ;;
        *)
            echo "Error: unknown argument: $1" >&2
            exit 1
            ;;
    esac
done

if [[ " $VALID_TYPES " != *" $CACHE_TYPE "* ]]; then
    echo "Error: --type must be one of: $VALID_TYPES (got '$CACHE_TYPE')" >&2
    exit 1
fi

if ! command -v systemctl >/dev/null 2>&1; then
    echo "Error: systemctl not found — this host does not use systemd." >&2
    echo "  Set OLLAMA_KV_CACHE_TYPE=$CACHE_TYPE in the Ollama server's environment by whatever means this host uses." >&2
    exit 1
fi

if ! systemctl list-unit-files ollama.service >/dev/null 2>&1 \
        || ! systemctl cat ollama.service >/dev/null 2>&1; then
    echo "Error: no ollama.service unit on this host — nothing to configure." >&2
    exit 1
fi

DESIRED=$(printf '[Service]\nEnvironment="OLLAMA_KV_CACHE_TYPE=%s"\n' "$CACHE_TYPE")

CURRENT=""
if [[ -r "$DROPIN_FILE" ]]; then
    CURRENT=$(cat "$DROPIN_FILE")
fi

if [[ "$REVERT" == "true" ]]; then
    if [[ ! -e "$DROPIN_FILE" ]]; then
        echo "Already reverted: $DROPIN_FILE does not exist (Ollama uses its f16 default)."
        exit 0
    fi
    if [[ "$APPLY" != "true" ]]; then
        echo "DRY RUN — would remove $DROPIN_FILE and restart ollama.service."
        echo "Re-run with --yes to apply. Requires sudo."
        exit 0
    fi
    sudo rm -f "$DROPIN_FILE"
    sudo rmdir --ignore-fail-on-non-empty "$DROPIN_DIR"
    sudo systemctl daemon-reload
    sudo systemctl restart ollama.service
    echo "Reverted: removed $DROPIN_FILE; ollama.service restarted (back to the f16 default)."
    exit 0
fi

if [[ "$CURRENT" == "$DESIRED" ]]; then
    echo "Already configured: $DROPIN_FILE already sets OLLAMA_KV_CACHE_TYPE=$CACHE_TYPE."
    echo "No change made, service not restarted."
    exit 0
fi

echo "Would write $DROPIN_FILE:"
echo "---"
printf '%s\n' "$DESIRED"
echo "---"
if [[ -n "$CURRENT" ]]; then
    echo "Replacing existing contents:"
    echo "---"
    printf '%s\n' "$CURRENT"
    echo "---"
fi
echo "Then: systemctl daemon-reload && systemctl restart ollama.service"
echo "(Restarting unloads any resident model; in-flight requests will fail.)"

if [[ "$APPLY" != "true" ]]; then
    echo
    echo "DRY RUN — nothing was changed."
    echo "This writes under /etc with sudo and restarts a system service."
    echo "Re-run with --yes only with the operator's explicit sign-off."
    exit 0
fi

sudo mkdir -p "$DROPIN_DIR"
printf '%s\n' "$DESIRED" | sudo tee "$DROPIN_FILE" >/dev/null
sudo systemctl daemon-reload
sudo systemctl restart ollama.service
echo "Applied: OLLAMA_KV_CACHE_TYPE=$CACHE_TYPE; ollama.service restarted."
echo "Verify with: systemctl show ollama.service -p Environment"
