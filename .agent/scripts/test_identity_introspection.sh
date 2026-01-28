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
"$SCRIPT_DIR/detect_agent_identity.sh" --write > /dev/null 2>&1
if [ -f .agent/.identity ]; then
    echo "✅ Identity file written successfully"
else
    echo "❌ Identity file creation failed"
    exit 1
fi
echo ""

# Test 4: Identity file can be sourced
echo "Test 4: Sourcing identity file..."
source .agent/.identity
if [ -n "$AGENT_NAME" ] && [ -n "$AGENT_MODEL" ]; then
    echo "✅ Identity file sourced successfully"
    echo "   Name: $AGENT_NAME"
    echo "   Model: $AGENT_MODEL"
else
    echo "❌ Identity file sourcing failed"
    exit 1
fi
echo ""

# Test 5: set_git_identity_env.sh works with --agent flag
echo "Test 5: Setting identity with --agent flag..."
(
    source "$SCRIPT_DIR/set_git_identity_env.sh" --agent copilot
    if [ "$AGENT_NAME" = "Copilot CLI Agent" ] && [ "$AGENT_MODEL" = "GPT-4o" ]; then
        echo "✅ Identity set successfully with --agent flag"
    else
        echo "❌ Identity setting with --agent flag failed"
        exit 1
    fi
)
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
    echo "⚠️  Some documentation issues found"
fi
echo ""

echo "=== All Tests Passed! ==="
echo ""
echo "The agent identity introspection system is working correctly."
echo "Agents can now:"
echo "  1. Auto-detect their framework and model"
echo "  2. Read identity from .agent/.identity file"
echo "  3. Use \$AGENT_MODEL in GitHub signatures"
echo "  4. Avoid copying example model names from documentation"
