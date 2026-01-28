---
description: Check if your feature branch needs to be updated with latest changes from default branch
---

# Check Branch Updates Workflow

Use this workflow to check if the default branch (e.g., `main`, `jazzy`) has new commits that you should merge or rebase into your feature branch.

## When to Use

✅ **Before committing** - Ensure you're working on an up-to-date base  
✅ **Before creating a PR** - Minimize merge conflicts  
✅ **During long-running work** - Stay in sync with team changes  
✅ **After returning to a feature branch** - Catch up with recent updates  

## Quick Check

```bash
.agent/scripts/check_branch_updates.sh
```

This will:
- Fetch latest commits from default branch
- Compare your branch against it
- Show how many commits behind/ahead you are
- Provide specific merge or rebase recommendations
- Display recent commits you're missing

## Automatic Check (Pre-Commit Hook)

The check runs automatically before every commit when using `pre-commit`:

```bash
# Install pre-commit hooks (one-time setup)
pip install pre-commit
pre-commit install

# Now every commit will check for updates
git commit -m "feat: my changes"
# Output will include branch update status
```

## Strict Mode (Block Commits)

To prevent commits when your branch is behind:

```bash
.agent/scripts/check_branch_updates.sh --strict
```

**Exit codes:**
- `0` - Branch is up-to-date or on default branch
- `1` - Branch needs updates
- `2` - Error occurred

## Understanding the Output

### Scenario 1: Up-to-Date ✅

```
✓ Feature branch is up-to-date with main
```

**Action:** None needed. You're good to commit!

### Scenario 2: Behind (No Local Commits) 📥

```
⚠️  Default branch has new commits!
  Current branch:  feature/my-feature
  Default branch:  main
  Commits behind:  3
  Commits ahead:   0
```

**Recommendation:** Fast-forward merge (no conflicts)

```bash
git merge origin/main
```

### Scenario 3: Diverged (Both Have Commits) 🔀

```
⚠️  Default branch has new commits!
  Current branch:  feature/my-feature
  Default branch:  main
  Commits behind:  5
  Commits ahead:   2

📊 Branches have diverged
```

**Option 1 - Merge (Recommended):**
```bash
git merge origin/main
```
- Preserves complete history
- Creates merge commit
- Safe and reversible

**Option 2 - Rebase (Cleaner History):**
```bash
git rebase origin/main
```
- Replays your commits on top of latest main
- Creates linear history
- ⚠️ Requires force-push if already shared
- May need to resolve conflicts

## Best Practices

### For AI Agents

1. **Check before starting work**
   ```bash
   .agent/scripts/checkout_default_branch.sh  # Get latest main
   git checkout -b feature/my-feature          # Create branch
   ```

2. **Check periodically during long tasks**
   ```bash
   # Every few hours or after major milestones
   .agent/scripts/check_branch_updates.sh
   ```

3. **Check before final commit**
   ```bash
   .agent/scripts/check_branch_updates.sh
   git commit -m "feat: complete implementation"
   ```

### For Human Developers

1. **Morning routine**
   ```bash
   git checkout main
   git pull
   git checkout feature/my-feature
   .agent/scripts/check_branch_updates.sh
   ```

2. **Before PR creation**
   ```bash
   .agent/scripts/check_branch_updates.sh
   # If updates needed:
   git merge origin/main
   # Re-run tests after merge
   .agent/scripts/test.sh
   ```

## Merge vs Rebase Decision Tree

```
Do you need to update your feature branch?
├─ Have you already pushed your feature branch?
│  ├─ YES → Use MERGE (safer for shared branches)
│  │   └─ git merge origin/main
│  └─ NO → Either option is fine
│      ├─ MERGE → Preserves exact history
│      │   └─ git merge origin/main
│      └─ REBASE → Cleaner linear history
│          └─ git rebase origin/main
└─ Are you working on a long-running feature?
   ├─ YES → Use MERGE (easier conflict resolution)
   │   └─ git merge origin/main
   └─ NO → Use REBASE for cleaner history
       └─ git rebase origin/main
```

## Handling Merge Conflicts

### After Merge

```bash
git merge origin/main
# If conflicts occur:
git status  # See conflicting files
# Edit files to resolve conflicts
git add <resolved-files>
git commit  # Complete the merge
```

### After Rebase

```bash
git rebase origin/main
# If conflicts occur:
git status  # See conflicting files
# Edit files to resolve conflicts
git add <resolved-files>
git rebase --continue  # Continue rebase
# Repeat for each conflicting commit
```

## Troubleshooting

### "Could not fetch from remote"

```bash
# Check network/auth
git fetch origin --dry-run

# Check remote configuration
git remote -v
```

### "Could not determine merge base" (Shallow Clone)

```bash
# Unshallow the repository
git fetch --unshallow

# Or just work with limited information
# The script will still show commit counts
```

### "Branches have diverged" but you don't remember changing anything

```bash
# Check what commits are different
git log origin/main..HEAD  # Your commits
git log HEAD..origin/main  # Their commits

# See the actual changes
git diff origin/main...HEAD
```

## Integration with Workflows

This check integrates with:

- **`/start-feature`** - Ensures clean starting point
- **`/finish-feature`** - Validates branch is current before PR
- **`/submit-pr`** - Pre-submission validation
- **Pre-commit hooks** - Automatic checking on every commit

## See Also

- `.agent/workflows/dev/start-feature.md` - Starting new work
- `.agent/workflows/dev/submit-pr.md` - Creating pull requests
- `.agent/rules/common/git-hygiene.md` - Git best practices
- `.agent/scripts/checkout_default_branch.sh` - Return to default branch
