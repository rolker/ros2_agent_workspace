# Agent Instruction Testing

This directory contains automated tests for agent instructions and workflows using [Promptfoo](https://www.promptfoo.dev/).

## Overview

Agent instructions (in `.agent/`, `.github/copilot-instructions.md`, etc.) are tested like code to prevent regressions and ensure agents follow documented workflows.

## Environment Setup

Set your Anthropic API key:

```bash
export ANTHROPIC_API_KEY="your-api-key-here"
```

Or set it in the GitHub Actions workflow as a secret.

```bash
# Run all tests
cd .agent/testing
npx promptfoo@latest eval

# Run specific test file
npx promptfoo@latest eval -c tests/test-environment-sourcing.yaml

# View results in UI
npx promptfoo@latest view
```

## Test Suite

### Regression Tests

1. **Environment Sourcing** (`tests/test-environment-sourcing.yaml`)
   - Verifies agents source ROS environment before builds
   - Prevents: Building without proper environment setup

2. **Worktree Workflow** (`tests/test-worktree-workflow.yaml`)
   - Verifies agents use worktrees instead of feature branches in main
   - Prevents: Direct commits to main, branch switching conflicts

3. **Git Identity** (`tests/test-git-identity.yaml`)
   - Verifies agents configure ephemeral git identity
   - Prevents: Copying example model names, using wrong identity method

4. **Issue-First Workflow** (`tests/test-issue-first-workflow.yaml`)
   - Verifies agents create/reference issues before coding
   - Prevents: Untracked work, lost context

5. **AI Signature** (`tests/test-ai-signature.yaml`)
   - Verifies agents sign GitHub content with actual identity
   - Prevents: Missing signatures, copied example model names

## How It Works

### Test Structure

Each test file defines scenarios that simulate user requests and check if the agent's response follows documented workflows:

```yaml
tests:
  - description: "Test name"
    vars:
      task: "User request"
      context: "Additional context"
    assert:
      - type: contains
        value: "expected output"
      - type: llm-rubric
        value: "Behavior check description"
```

### Assertion Types

- **contains/not-contains**: Check for specific text in response
- **matches**: Regex pattern matching
- **llm-rubric**: LLM-based evaluation of behavior
- **contains-all/any**: Multiple value checks

### LLM Evaluation

Tests use LLM-as-a-judge to evaluate complex behaviors that can't be checked with simple text matching.

## Running Tests

### Locally

```bash
# From workspace root
cd .agent/testing

# Run all tests
npx promptfoo@latest eval

# Run with specific config
npx promptfoo@latest eval -c promptfooconfig.yaml

# Run single test file
npx promptfoo@latest eval -c tests/test-worktree-workflow.yaml

# View results in web UI
npx promptfoo@latest view
```

### In CI/CD

Tests run automatically on:
- Pull requests that modify `.agent/` or instruction files
- Manual trigger via GitHub Actions
- Nightly schedule (detects drift)

See: `.github/workflows/test-agent-instructions.yml`

## Adding New Tests

See: [WRITING_TESTS.md](WRITING_TESTS.md)

## Configuration

Main config: `promptfooconfig.yaml`
- Defines instruction files to test
- Configures test providers (models)
- Sets default assertions

## Test Results

Results are stored in `.promptfoo/output/` (gitignored).

View results:
```bash
npx promptfoo@latest view
```

Or check JSON output:
```bash
cat .promptfoo/output.json | jq
```

## Troubleshooting

### Tests are failing locally but pass in CI
- Check your instruction files match the repository version
- Ensure you're using the same Promptfoo version

### LLM-rubric tests are flaky
- LLM evaluation can be non-deterministic
- Consider adding more specific text-based assertions
- Adjust rubric descriptions to be more precise

### Tests are slow
- Use `--no-cache` flag if needed
- Consider reducing test scenarios
- Use text-based assertions over LLM-rubric when possible

## Resources

- [Promptfoo Documentation](https://www.promptfoo.dev/docs/intro/)
- [Writing Tests Guide](WRITING_TESTS.md)
- [Running Tests Guide](RUNNING_TESTS.md)
