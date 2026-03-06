# Plan: Bootstrap script should show sudo commands and skip already-installed packages

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/352

## Context

`bootstrap.sh` runs up to 12 `sudo` commands without showing the user what
will be executed. It also unconditionally installs most packages even when
they're already present (only locale, `ros-jazzy-ros-base`, and `rosdep init`
have skip logic today). The Makefile prompt (lines 115–130) says "You may be
prompted for your sudo password" but doesn't list specific commands.

The review comment recommended: (1) a dynamic `--dry-run` flag over a
hardcoded summary, (2) an `ensure_installed` helper, and (3) a
`make skip-bootstrap` convenience target coordinated with #332.

## Approach

### Step 1: Add idempotent checks to `bootstrap.sh`

Wrap each installation section in a check-before-install pattern. Introduce
a helper function and track whether any `sudo` commands are actually needed:

```bash
is_installed() { dpkg -s "$1" 2>/dev/null | grep -q 'Status: install ok installed'; }
```

Checks to add (extending the 3 existing ones to cover all sections):

| Section | Check | Skip if true |
|---------|-------|-------------|
| Locale (line 12) | Already has check | _(no change)_ |
| `software-properties-common` (line 26) | `is_installed software-properties-common` | Skip install |
| Universe repo (line 27) | `grep -q universe /etc/apt/sources.list` | Skip `add-apt-repository` |
| `curl` (line 30) | `is_installed curl` | Skip install |
| ROS 2 apt source (line 46) | `ls /etc/apt/sources.list.d/ros2*.list 2>/dev/null` | Skip `.deb` download + install |
| `apt update` (line 51) | Needed only if any apt install is pending | Conditional |
| `ros-dev-tools` (line 52) | `is_installed ros-dev-tools` | Skip install |
| `ros-jazzy-ros-base` (line 55) | Already has check | _(no change)_ |
| `python3-pip` (line 64) | `is_installed python3-pip` | Skip install |
| pip packages (line 65) | `python3 -c "import empy" 2>/dev/null` etc. | Skip pip install |
| `rosdep init` (line 69) | Already has check | _(no change)_ |

If all checks pass, print "All prerequisites already installed — nothing to
do." and exit 0 without any `sudo` call.

### Step 2: Add `--dry-run` flag to `bootstrap.sh`

When `--dry-run` is passed, the script runs through the same check logic but
instead of executing `sudo` commands, it collects them into a list and prints
them. This keeps the summary in sync with actual behavior — no hardcoded list
that can drift.

```bash
if [ "${1:-}" = "--dry-run" ]; then
    DRY_RUN=true
fi
```

Each `sudo` call becomes:
```bash
if [ "$DRY_RUN" = true ]; then
    PENDING_COMMANDS+=("sudo apt install -y ros-dev-tools")
else
    sudo apt install -y ros-dev-tools
fi
```

At the end of a dry run, print the collected commands and exit.

### Step 3: Update Makefile first-run prompt

Replace the current generic "You may be prompted for your sudo password"
message (lines 121–122) with a call to `bootstrap.sh --dry-run` so the
user sees exactly which commands will require `sudo`:

```makefile
$(STAMP)/bootstrap.done:
	@if [ ! -f "$(STAMP)/bootstrap.done" ] && [ -z "$$CI" ] && [ -z "$$NONINTERACTIVE" ]; then \
		echo ""; \
		echo "========================================"; \
		echo "  First-run setup detected"; \
		echo "========================================"; \
		echo ""; \
		./.agent/scripts/bootstrap.sh --dry-run; \
		echo ""; \
		echo "Set CI=1 or NONINTERACTIVE=1 to skip this message."; \
		echo ""; \
		read -r -p "Continue? [Y/n] " response; \
		case "$$response" in \
			[nN]*) echo "Aborted. Run 'make build' when ready."; exit 1 ;; \
		esac; \
	fi
```

### Step 4: Add `make skip-bootstrap` target

A simple convenience target that creates the stamp file, coordinated with
the existing stamp-file pattern from ADR-0007:

```makefile
skip-bootstrap: ## Skip bootstrap (system already configured)
	@mkdir -p $(STAMP)
	@touch $(STAMP)/bootstrap.done
	@echo "Bootstrap marked as done. Run 'make clean' to reset."
```

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/bootstrap.sh` | Add `is_installed` helper, idempotent checks, `--dry-run` flag, early exit when nothing needed |
| `Makefile` | Update first-run prompt to use `--dry-run`; add `skip-bootstrap` target |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Core driver. `--dry-run` in the Makefile prompt shows exact `sudo` commands before asking for confirmation. Users make an informed decision. |
| Only what's needed | Idempotent checks skip already-installed packages, avoiding unnecessary `sudo` prompts and install time. |
| Enforcement over documentation | `--dry-run` is generated from the same code path as the real install — can't drift from reality. |
| A change includes its consequences | Makefile prompt updated alongside script. `make help` auto-updates via the `##` comment pattern. |
| Improve incrementally | Single PR with two files. Each improvement (checks, dry-run, skip target) is independently useful. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0007 — Retain Make with dependency tracking | Yes | `skip-bootstrap` uses the existing stamp-file pattern. Compatible with #332's refactor. |
| 0004 — Enforcement hierarchy | No | No new compliance rules. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `bootstrap.sh` interface (add `--dry-run`) | Makefile that calls it | Yes |
| Add `skip-bootstrap` Make target | `make help` (auto via `##` comment) | Yes (automatic) |
| Add `skip-bootstrap` Make target | `make generate-skills` (new `.PHONY`) | Yes — run after adding target |

## Open Questions

- **`apt update` consolidation**: Currently `apt update` runs in 3 places
  (lines 14, 30, 51). Should we consolidate into a single `apt update` that
  runs only if any apt installs are pending? This simplifies the script but
  changes behavior. Recommend yes — a single conditional `apt update` is
  cleaner.

## Estimated Scope

Single PR, 2 files changed.
