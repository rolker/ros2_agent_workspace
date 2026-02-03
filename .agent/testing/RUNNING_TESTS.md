# Running Agent Instruction Tests

Guide for running Promptfoo tests locally and in CI/CD.

## Prerequisites

- Node.js 20+ (check with `node --version`)
- No installation needed (uses npx)

## Quick Start

```bash
# Navigate to testing directory
cd .agent/testing

# Run all tests
npx promptfoo@latest eval

# View results in browser
npx promptfoo@latest view
```

## Running Tests

### All Tests

```bash
cd .agent/testing
npx promptfoo@latest eval
```

Output shows pass/fail for each test case.

### Specific Test File

```bash
cd .agent/testing
npx promptfoo@latest eval -c tests/test-worktree-workflow.yaml
```

### With Options

```bash
# No caching (fresh evaluation)
npx promptfoo@latest eval --no-cache

# Verbose output
npx promptfoo@latest eval --verbose

# Specific config file
npx promptfoo@latest eval -c custom-config.yaml

# Output to specific file
npx promptfoo@latest eval -o results.json
```

## Viewing Results

### Interactive UI

```bash
npx promptfoo@latest view
```

Opens web interface at `http://localhost:15500` showing:
- Pass/fail status
- Test outputs
- Assertion details
- Comparison across providers

### JSON Output

```bash
# View raw JSON
cat .promptfoo/output.json | jq

# Check test summary
cat .promptfoo/output.json | jq '.results.summary'

# List failing tests
cat .promptfoo/output.json | jq '.results.table[] | select(.pass == false)'
```

### Command Line Summary

Default `eval` command shows summary:
```
✓ Test passed: Environment sourcing (3/3 assertions)
✗ Test failed: Worktree workflow (2/3 assertions)
  - Failed: Should use worktree_create.sh
```

## Test Development Workflow

### 1. Create New Test

```bash
cd .agent/testing/tests
cp test-environment-sourcing.yaml test-my-new-test.yaml
# Edit the file...
```

### 2. Run Single Test

```bash
npx promptfoo@latest eval -c tests/test-my-new-test.yaml
```

### 3. Debug Failures

```bash
# Run with verbose output
npx promptfoo@latest eval -c tests/test-my-new-test.yaml --verbose

# View in UI for detailed inspection
npx promptfoo@latest view
```

### 4. Iterate

Adjust test assertions based on results, re-run until passing.

## CI/CD Integration

Tests run automatically in GitHub Actions.

### Trigger Manually

```bash
# Via GitHub CLI
gh workflow run test-agent-instructions.yml

# Or via GitHub UI
# Actions → Test Agent Instructions → Run workflow
```

### View CI Results

```bash
# List recent runs
gh run list --workflow=test-agent-instructions.yml

# View specific run
gh run view <run-id>

# Download artifacts
gh run download <run-id>
```

### CI Test Output

Workflow uploads results as artifacts:
- `promptfoo-results/` - Full test output
- Available for 30 days

## Common Commands

```bash
# Run tests with fresh cache
npx promptfoo@latest eval --no-cache

# Run tests and view results immediately
npx promptfoo@latest eval && npx promptfoo@latest view

# Run specific test file
npx promptfoo@latest eval -c tests/test-worktree-workflow.yaml

# Run tests for specific provider only
npx promptfoo@latest eval --filter-providers "openai:gpt-4o"

# Generate HTML report
npx promptfoo@latest eval -o results.html

# Check Promptfoo version
npx promptfoo@latest --version

# Get help
npx promptfoo@latest eval --help
```

## Interpreting Results

### Pass/Fail Criteria

Each test case has multiple assertions. All must pass for the test to pass.

Example output:
```
Test: Environment sourcing before builds
  ✓ Should mention sourcing environment
  ✓ Should not build without sourcing
  ✓ Workflow is correct
Result: PASS (3/3 assertions)
```

### Common Failures

**"Agent should mention X but didn't"**
- Agent's response doesn't contain required text
- Check if wording differs but meaning is correct
- May need to adjust assertion to be less strict

**"LLM rubric failed"**
- Agent's behavior doesn't match expected workflow
- Review the rubric criteria
- Check agent's actual response in UI

**"Agent should NOT mention Y but did"**
- Agent violated a policy
- Real issue that needs fixing in instructions

## Debugging Tips

### 1. Use Interactive UI

```bash
npx promptfoo@latest view
```

Click on failed tests to see:
- Full prompt sent to agent
- Agent's complete response
- Which assertions passed/failed
- Assertion details

### 2. Check Exact Output

```bash
cat .promptfoo/output.json | jq '.results.table[0].outputs'
```

### 3. Test Assertions Manually

Copy test scenario, run through agent manually, verify behavior.

### 4. Adjust Test Sensitivity

If test is too strict:
```yaml
# Before (too strict)
- type: contains
  value: "exact wording required"

# After (more flexible)
- type: contains-any
  value:
    - "option 1"
    - "option 2"
    - "option 3"
```

## Performance

### Speed Optimization

- Use text-based assertions when possible (faster than LLM-rubric)
- Cache results (default, use `--no-cache` when needed)
- Reduce number of test scenarios if needed

### Cost Optimization

LLM-rubric assertions use API calls:
- Each rubric = 1 API call per provider
- Monitor usage if testing frequently
- Use mock providers for development if needed

## Troubleshooting

### "Command not found: npx"

Install Node.js 20+: https://nodejs.org/

### "Timeout waiting for response"

- Check internet connection
- API keys may be rate-limited
- Increase timeout in config

### Tests pass locally but fail in CI

- Check instruction files match (run from clean checkout)
- Verify same Promptfoo version
- Check environment variables

### Flaky test results

- LLM responses can vary
- Add more specific text assertions
- Make rubric criteria more precise
- Consider multiple runs to confirm

### Port 15500 already in use

```bash
# Kill existing Promptfoo view
pkill -f promptfoo

# Or use different port
npx promptfoo@latest view --port 8080
```

## Best Practices

1. **Run tests before committing**
   ```bash
   cd .agent/testing && npx promptfoo@latest eval
   ```

2. **Use no-cache for verification**
   ```bash
   npx promptfoo@latest eval --no-cache
   ```

3. **Check specific failures in UI**
   ```bash
   npx promptfoo@latest view
   ```

4. **Add regression test for each bug fix**
   - Bug found → Create test that would catch it
   - Fix bug → Verify test now passes
   - Commit both test and fix

5. **Review test output regularly**
   - Check for patterns in failures
   - Adjust tests if instructions legitimately change
   - Update rubrics to be clearer

## Next Steps

- [Writing Tests](WRITING_TESTS.md) - How to create new tests
- [Main README](README.md) - Overview and test suite description
- [Promptfoo Docs](https://www.promptfoo.dev/docs/) - Full documentation
