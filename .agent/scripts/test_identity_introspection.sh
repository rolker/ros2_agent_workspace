#!/bin/bash
# Test script to validate agent identity introspection system
# This validates all components of the identity introspection strategy

if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
    echo "Error: This script should be executed, not sourced."
    echo "  Run: ${BASH_SOURCE[0]} $*"
    return 1
fi
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$(dirname "$SCRIPT_DIR")/.."

echo "=== Testing Agent Identity Introspection System ==="
echo ""

# Test 1: Framework configuration loads
echo "Test 1: Framework configuration..."
source "$SCRIPT_DIR/framework_config.sh"
if [ -n "${FRAMEWORK_NAMES[copilot]}" ] && [ -n "${FRAMEWORK_MODELS[copilot]}" ]; then
    echo "✅ Framework configuration loaded successfully"
else
    echo "❌ Framework configuration failed"
    exit 1
fi
echo ""

# Test 2: Detection script runs
echo "Test 2: Identity detection script..."
OUTPUT=$("$SCRIPT_DIR/detect_agent_identity.sh" 2>&1)
if echo "$OUTPUT" | grep -q "Detected Agent Identity"; then
    echo "✅ Detection script runs successfully"
else
    echo "❌ Detection script failed"
    echo "$OUTPUT"
    exit 1
fi
echo ""

# Test 3: Identity file can be written
echo "Test 3: Writing identity to file..."
# Clean up any existing runtime file first
rm -f .agent/.identity
"$SCRIPT_DIR/detect_agent_identity.sh" --write > /dev/null 2>&1
if [ -f .agent/.identity ]; then
    echo "✅ Identity file written successfully"
    # Verify it's marked as runtime/git-ignored
    if grep -q "RUNTIME" .agent/.identity && grep -q "git-ignored" .agent/.identity; then
        echo "   (Correctly marked as runtime file)"
    fi
else
    echo "❌ Identity file creation failed"
    exit 1
fi
echo ""

# Test 4: Identity file can be sourced
echo "Test 4: Sourcing identity file..."
if [ -f .agent/.identity ]; then
    source .agent/.identity
    if [ -n "$AGENT_NAME" ] && [ -n "$AGENT_MODEL" ]; then
        echo "✅ Identity file sourced successfully"
        echo "   Name: $AGENT_NAME"
        echo "   Model: $AGENT_MODEL"
    else
        echo "❌ Identity file sourcing failed"
        exit 1
    fi
else
    echo "❌ Identity file doesn't exist"
    exit 1
fi
echo ""

# Test 5: set_git_identity_env.sh works with --agent flag (in subshell)
echo "Test 5: Setting identity with --agent flag (subshell test)..."
EXIT_CODE=0
(
    source "$SCRIPT_DIR/set_git_identity_env.sh" --agent copilot > /dev/null 2>&1
    if [ "$AGENT_NAME" = "Copilot CLI Agent" ] && [ "$AGENT_MODEL" = "GPT-4o" ]; then
        echo "✅ Identity set successfully in subshell"
        exit 0
    else
        echo "❌ Identity setting with --agent flag failed in subshell"
        exit 1
    fi
) || EXIT_CODE=$?

if [ $EXIT_CODE -ne 0 ]; then
    exit $EXIT_CODE
fi
echo ""

# Test 5b: Variables persist in current shell when properly sourced
echo "Test 5b: Variables available in current shell when sourced..."
# Clear any existing variables
unset AGENT_NAME AGENT_EMAIL AGENT_MODEL AGENT_FRAMEWORK
# Source in current shell
source "$SCRIPT_DIR/set_git_identity_env.sh" --agent copilot > /dev/null 2>&1
if [ "$AGENT_NAME" = "Copilot CLI Agent" ] && [ "$AGENT_MODEL" = "GPT-4o" ]; then
    echo "✅ Identity variables available in current shell after sourcing"
else
    echo "❌ Identity variables not available in current shell"
    echo "   AGENT_NAME=$AGENT_NAME (expected: Copilot CLI Agent)"
    echo "   AGENT_MODEL=$AGENT_MODEL (expected: GPT-4o)"
    exit 1
fi
echo ""

# Test 5c: Manual 2-parameter usage (backward compatibility)
#
# The 2-arg path in set_git_identity_env.sh runs framework detection and looks
# up the matching fallback model from framework_config.sh. The contract is:
#   - detection miss → (AGENT_FRAMEWORK=custom,            AGENT_MODEL="Unknown Model")
#   - detection hit  → (AGENT_FRAMEWORK=<detected>,        AGENT_MODEL=FRAMEWORK_MODELS[normalized key])
# Framework normalization mirrors set_git_identity_env.sh: strip trailing -cli,
# lowercase. AGENT_FRAMEWORK must never be the raw sentinel "unknown".
echo "Test 5c: Manual 2-parameter usage (backward compatibility)..."
# Clear any existing variables
unset AGENT_NAME AGENT_EMAIL AGENT_MODEL AGENT_FRAMEWORK
# Source with 2 parameters
source "$SCRIPT_DIR/set_git_identity_env.sh" "Test Agent" "test@example.com" > /dev/null 2>&1

test_5c_ok=true
test_5c_reason=""
if [ "$AGENT_NAME" != "Test Agent" ] || [ "$AGENT_EMAIL" != "test@example.com" ]; then
    test_5c_ok=false
    test_5c_reason="name/email mismatch"
elif [ "$AGENT_FRAMEWORK" = "unknown" ] || [ -z "$AGENT_FRAMEWORK" ]; then
    test_5c_ok=false
    test_5c_reason="AGENT_FRAMEWORK must not be empty or 'unknown' (should map to 'custom' on detection miss)"
elif [ "$AGENT_FRAMEWORK" = "custom" ]; then
    if [ "$AGENT_MODEL" != "Unknown Model" ]; then
        test_5c_ok=false
        test_5c_reason="framework=custom implies model='Unknown Model', got '$AGENT_MODEL'"
    fi
else
    # Detected framework: model must match FRAMEWORK_MODELS[normalized key]
    fwkey="${AGENT_FRAMEWORK%-cli}"
    fwkey="${fwkey,,}"
    expected_model="${FRAMEWORK_MODELS[$fwkey]:-Unknown Model}"
    if [ "$AGENT_MODEL" != "$expected_model" ]; then
        test_5c_ok=false
        test_5c_reason="framework='$AGENT_FRAMEWORK' (key='$fwkey') implies model='$expected_model', got '$AGENT_MODEL'"
    fi
fi

if [ "$test_5c_ok" = "true" ]; then
    echo "✅ 2-parameter usage works correctly (framework=$AGENT_FRAMEWORK, model=$AGENT_MODEL)"
else
    echo "❌ 2-parameter usage failed: $test_5c_reason"
    echo "   AGENT_NAME=$AGENT_NAME (expected: Test Agent)"
    echo "   AGENT_EMAIL=$AGENT_EMAIL (expected: test@example.com)"
    echo "   AGENT_MODEL=$AGENT_MODEL"
    echo "   AGENT_FRAMEWORK=$AGENT_FRAMEWORK"
    exit 1
fi
echo ""

# Test 5d: Manual 3-parameter usage (self-report form)
#
# The 3-arg form preserves the caller-supplied model verbatim; AGENT_FRAMEWORK
# comes from framework detection with the raw sentinel "unknown" collapsed to
# "custom" (see set_git_identity_env.sh:154-164). Contract:
#   - model matches the 3rd arg exactly (self-report)
#   - framework is non-empty AND is never the raw sentinel "unknown"
echo "Test 5d: Manual 3-parameter usage with model (new feature)..."
# Clear any existing variables
unset AGENT_NAME AGENT_EMAIL AGENT_MODEL AGENT_FRAMEWORK
# Source with 3 parameters
source "$SCRIPT_DIR/set_git_identity_env.sh" "Test Agent" "test@example.com" "Test Model 1.0" > /dev/null 2>&1
if [ "$AGENT_NAME" = "Test Agent" ] \
   && [ "$AGENT_EMAIL" = "test@example.com" ] \
   && [ "$AGENT_MODEL" = "Test Model 1.0" ] \
   && [ -n "$AGENT_FRAMEWORK" ] \
   && [ "$AGENT_FRAMEWORK" != "unknown" ]; then
    echo "✅ 3-parameter usage works correctly (model=$AGENT_MODEL, framework=$AGENT_FRAMEWORK)"
else
    echo "❌ 3-parameter usage failed"
    echo "   AGENT_NAME=$AGENT_NAME (expected: Test Agent)"
    echo "   AGENT_EMAIL=$AGENT_EMAIL (expected: test@example.com)"
    echo "   AGENT_MODEL=$AGENT_MODEL (expected: Test Model 1.0)"
    echo "   AGENT_FRAMEWORK=$AGENT_FRAMEWORK (expected: non-empty and not 'unknown')"
    exit 1
fi
echo ""

# Test 6: Documentation files exist and reference introspection
echo "Test 6: Documentation consistency..."
DOCS_OK=true

if ! grep -q "AGENT_MODEL" .agent/rules/common/ai-signature.md; then
    echo "❌ ai-signature.md missing AGENT_MODEL reference"
    DOCS_OK=false
fi

if ! grep -iq "do not copy" .agent/rules/common/ai-signature.md; then
    echo "❌ ai-signature.md missing anti-copying warning"
    DOCS_OK=false
fi

if ! grep -q "Model Identity Introspection" .agent/AI_IDENTITY_STRATEGY.md; then
    echo "❌ AI_IDENTITY_STRATEGY.md missing introspection section"
    DOCS_OK=false
fi

if ! grep -q "AGENT_MODEL" .agent/AI_RULES.md; then
    echo "❌ AI_RULES.md missing AGENT_MODEL reference"
    DOCS_OK=false
fi

if [ "$DOCS_OK" = true ]; then
    echo "✅ Documentation is consistent"
else
    echo "⚠️  Some documentation issues found (non-critical)"
    # Note: We don't fail the test for documentation warnings
    # as the core functionality still works
fi
echo ""

echo "=== All Core Tests Passed! ==="
echo ""
echo "The agent identity introspection system is working correctly."
echo "Agents can now:"
echo "  1. Auto-detect their framework and model"
echo "  2. Read identity from .agent/.identity file"
echo "  3. Use \$AGENT_MODEL in GitHub signatures"
echo "  4. Avoid copying example model names from documentation"
