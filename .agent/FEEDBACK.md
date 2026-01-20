# Feedback Loop

This log tracks "Friction Points" encountered by agents or developers.
Framework Engineers review this file to improve tools and rules.

**Format**: `[Date] [Category] Description of friction -> Suggestion`

---

## Friction Log

<!-- Add new entries below -->
- [2026-01-16] [Process] CI Failed on PR because local paths changed but validation wasn't run locally. -> Suggestion: Update `submit-pr` workflow to run `make lint` or `pre-commit` automatically.
- [2026-01-16] [Workflow] Repositories left on feature branches after task. -> Suggestion: Add mechanism to return all repos to default branch (main/jazzy) to reset workspace state.
- [2026-01-20] [Context] 'project11' is a legacy/alias name for 'unh_marine_autonomy'. -> Agents should treat 'project11' and 'unh_marine_autonomy' as equivalent.
- [2026-01-20] [Migration] Script breakage due to `.repos` file relocation. ->
    - **Issue**: `list_overlay_repos.py` only scans `configs/`. `health_check.sh` fails if `configs/` is missing. `setup.sh` hardcodes `configs/bootstrap.repos`.
    - **Context**: Migration of `.repos` files to `unh_marine_autonomy` moves them out of the expected root `configs/` location.
    - **Suggestion**: Update scripts to support dual-path lookup or "distributed configuration". `bootstrap.repos` may need to remain in root or have a special handling mechanism.
