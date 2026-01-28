#!/bin/bash
# Test script to validate agent identity introspection system
# This validates all components of the identity introspection strategy

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
echo "Test 5c: Manual 2-parameter usage (backward compatibility)..."
# Clear any existing variables
unset AGENT_NAME AGENT_EMAIL AGENT_MODEL AGENT_FRAMEWORK
# Source with 2 parameters
source "$SCRIPT_DIR/set_git_identity_env.sh" "Test Agent" "test@example.com" > /dev/null 2>&1
if [ "$AGENT_NAME" = "Test Agent" ] && [ "$AGENT_EMAIL" = "test@example.com" ] && [ "$AGENT_MODEL" = "Unknown Model" ] && [ "$AGENT_FRAMEWORK" = "custom" ]; then
    echo "✅ 2-parameter usage works correctly (model = 'Unknown Model')"
else
    echo "❌ 2-parameter usage failed"
    echo "   AGENT_NAME=$AGENT_NAME (expected: Test Agent)"
    echo "   AGENT_EMAIL=$AGENT_EMAIL (expected: test@example.com)"
    echo "   AGENT_MODEL=$AGENT_MODEL (expected: Unknown Model)"
    echo "   AGENT_FRAMEWORK=$AGENT_FRAMEWORK (expected: custom)"
    exit 1
fi
echo ""

# Test 5d: Manual 3-parameter usage (new feature)
echo "Test 5d: Manual 3-parameter usage with model (new feature)..."
# Clear any existing variables
unset AGENT_NAME AGENT_EMAIL AGENT_MODEL AGENT_FRAMEWORK
# Source with 3 parameters
source "$SCRIPT_DIR/set_git_identity_env.sh" "Test Agent" "test@example.com" "Test Model 1.0" > /dev/null 2>&1
if [ "$AGENT_NAME" = "Test Agent" ] && [ "$AGENT_EMAIL" = "test@example.com" ] && [ "$AGENT_MODEL" = "Test Model 1.0" ] && [ "$AGENT_FRAMEWORK" = "custom" ]; then
    echo "✅ 3-parameter usage works correctly (model specified)"
else
    echo "❌ 3-parameter usage failed"
    echo "   AGENT_NAME=$AGENT_NAME (expected: Test Agent)"
    echo "   AGENT_EMAIL=$AGENT_EMAIL (expected: test@example.com)"
    echo "   AGENT_MODEL=$AGENT_MODEL (expected: Test Model 1.0)"
    echo "   AGENT_FRAMEWORK=$AGENT_FRAMEWORK (expected: custom)"
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
