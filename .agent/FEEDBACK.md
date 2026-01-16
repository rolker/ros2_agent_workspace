# Feedback Loop

This log tracks "Friction Points" encountered by agents or developers.
Framework Engineers review this file to improve tools and rules.

**Format**: `[Date] [Category] Description of friction -> Suggestion`

---

## Friction Log

<!-- Add new entries below -->
- [2026-01-16] [Process] CI Failed on PR because local paths changed but validation wasn't run locally. -> Suggestion: Update `submit-pr` workflow to run `make lint` or `pre-commit` automatically.
- [2026-01-16] [Workflow] Repositories left on feature branches after task. -> Suggestion: Add mechanism to return all repos to default branch (main/jazzy) to reset workspace state.
