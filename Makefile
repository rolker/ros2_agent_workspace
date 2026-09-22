# ROS2 Agent Workspace — Makefile
#
# Uses stamp-file dependencies so `make build` works from a fresh clone.
# See ADR-0007 for rationale: docs/decisions/0007-retain-make-with-dependency-tracking.md
#
# Target tiers (see #335):
#   Tier 1 — Setup chain (stamp-based, mostly invisible)
#   Tier 2 — Developer workflow (what users type)
#   Tier 3 — Agent/maintenance (container + utilities)

# Bootstrap recipe uses bash-specific `read -r -p`.
SHELL := /bin/bash

# --- Workspace root resolution ---
# Stamps are workspace-global. When running from a worktree, resolve back
# to the main workspace root so stamps are shared.
#   .workspace-worktrees/<name>/  → 2 levels up
#   layers/worktrees/<name>/      → 3 levels up
ifneq ($(findstring /layers/worktrees/,$(CURDIR)),)
  MAIN_ROOT := $(abspath $(CURDIR)/../../..)
else ifneq ($(findstring /.workspace-worktrees/,$(CURDIR)),)
  MAIN_ROOT := $(abspath $(CURDIR)/../..)
else
  MAIN_ROOT := $(CURDIR)
endif

# --- Stamp directory ---
STAMP := $(MAIN_ROOT)/.make
VENV_DIR := $(MAIN_ROOT)/.venv
VENV_BIN := $(VENV_DIR)/bin
PRE_COMMIT := $(VENV_BIN)/pre-commit

# --- Layer list (read from manifest config after bootstrap) ---
# On a fresh clone, layers.txt doesn't exist yet (created by manifest bootstrap).
# Targets that need LAYER_STAMPS use recursive make after manifest.done so that
# LAYERS is re-evaluated with the real layers.txt. See: build, setup-all.
LAYERS_FILE := $(MAIN_ROOT)/configs/manifest/layers.txt
ifneq ($(wildcard $(LAYERS_FILE)),)
  LAYERS := $(shell awk '!/^[[:space:]]*($$|#)/ {$$1=$$1; print}' $(LAYERS_FILE) | tr '\n' ' ')
else
  LAYERS :=
endif
LAYER_STAMPS := $(patsubst %,$(STAMP)/layer-%.done,$(LAYERS))

# --- Phony targets ---
.PHONY: help build _build-layers test test-scripts lint clean setup-all _setup-all-layers dashboard dashboard-ui test-dashboard validate sync lock unlock revert-feature pr-triage generate-skills skip-bootstrap skip-git-bug agent-build agent-run agent-shell add-remote push-remote pull-remote merge-pr

# =============================================================================
# Tier 2 — Developer workflow
# =============================================================================

help:
	@echo "ROS2 Agent Workspace - Makefile"
	@echo ""
	@echo "Core workflow:"
	@echo "  build         - Build all layers (auto-setup on first run)"
	@echo "  test          - Run tests on all layers (builds first if needed)"
	@echo "  lint          - Run pre-commit hooks on all files"
	@echo "  clean         - Clean build artifacts and reset setup stamps"
	@echo "  setup-all     - Run full setup without building"
	@echo ""
	@echo "Status & info:"
	@echo "  dashboard     - Unified workspace status (worktrees, PRs, health)"
	@echo "  dashboard QUICK=1 - Quick mode (skip sync and GitHub API)"
	@echo "  dashboard-ui  - Start web-based dashboard (http://localhost:3000)"
	@echo "  test-dashboard - Run dashboard unit/integration tests (ephemeral port)"
	@echo "  test-scripts  - Run .agent/scripts/tests/ (shell + pytest, no ROS build)"
	@echo "  validate      - Validate workspace config + layer sourcing + local rosdep keys (CI-oriented)"
	@echo "                  make reports 2 for ANY failure here; run"
	@echo "                  validate_workspace.py directly for 0 match / 1 drift / 3 unconfigured / 4 unreadable repo"
	@echo ""
	@echo "Remote sync:"
	@echo "  add-remote REMOTE=<name> URL_PREFIX=<prefix> - Add remote to all repos"
	@echo "  push-remote REMOTE=<name> [ALL=1] [SET_DEFAULT=1] - Push to named remote"
	@echo "  pull-remote REMOTE=<name> [PULL=1|BRANCH=<b>] - Fetch/pull from remote"
	@echo ""
	@echo "Maintenance:"
	@echo "  sync          - Safely sync all workspace repositories"
	@echo "  merge-pr (ISSUE=<n> [REPO=<slug>] | PR=<n> REPO=<slug>) [NO_WAIT=1] - Merge a PR, clean up worktree/branches, then sync (PR= needs REPO=, e.g. REPO=workspace; NO_WAIT=1 skips the CI wait)"
	@echo "  lock          - Lock workspace for exclusive access"
	@echo "  unlock        - Unlock workspace"
	@echo "  pr-triage     - Cross-repo PR triage (all workspace repos)"
	@echo "  skip-bootstrap - Skip bootstrap (system already configured)"
	@echo "  skip-git-bug  - Skip git-bug setup (optional)"
	@echo "  revert-feature ISSUE=<number> - Revert all commits for a specific issue"
	@echo "  generate-skills - Regenerate Claude Code /make_* slash commands"
	@echo ""
	@echo "Agent container:"
	@echo "  agent-build   - Build the sandboxed agent Docker image"
	@echo "  agent-run ISSUE=<number> - Launch agent container for a worktree"
	@echo "  agent-shell ISSUE=<number> - Launch agent container with bash (debug)"
	@echo ""

# Ensure manifest is bootstrapped first, then re-invoke make so LAYERS is
# read from the now-existing layers.txt (see layer list comment above).
build: $(STAMP)/manifest.done
	@$(MAKE) --no-print-directory _build-layers

_build-layers: $(LAYER_STAMPS) $(STAMP)/rosdep-local.done
	@./.agent/scripts/build.sh

test: build
	@./.agent/scripts/test.sh

lint: $(STAMP)/setup-dev.done
	@$(PRE_COMMIT) run --all-files

clean:
	@echo "Cleaning build artifacts..."
	@find $(MAIN_ROOT)/layers -type d -name "build" -exec rm -rf {} + 2>/dev/null || true
	@find $(MAIN_ROOT)/layers -type d -name "install" -exec rm -rf {} + 2>/dev/null || true
	@find $(MAIN_ROOT)/layers -type d -name "log" -exec rm -rf {} + 2>/dev/null || true
	@echo "Resetting setup stamps..."
	@rm -rf $(STAMP)
	@echo "Done. Run 'make build' to re-setup and rebuild."

setup-all: $(STAMP)/manifest.done
	@$(MAKE) --no-print-directory _setup-all-layers

_setup-all-layers: $(LAYER_STAMPS) $(STAMP)/git-bug.done
	@echo "Setup complete. All layers are ready."

dashboard:
ifdef QUICK
	@./.agent/scripts/dashboard.sh --quick
else
	@./.agent/scripts/dashboard.sh
endif

dashboard-ui:
	@PYTHON=$$([ -x "$(VENV_BIN)/python3" ] && echo "$(VENV_BIN)/python3" || echo "python3"); \
	$$PYTHON ./.agent/tools/dashboard/server.py

test-dashboard:
	@PYTHON=$$([ -x "$(VENV_BIN)/python3" ] && echo "$(VENV_BIN)/python3" || echo "python3"); \
	$$PYTHON -m unittest discover .agent/tools/dashboard/tests -v

test-scripts:
	@PYTHON=$$([ -x "$(VENV_BIN)/python3" ] && echo "$(VENV_BIN)/python3" || echo "python3"); \
	PYTHON=$$PYTHON ./.agent/scripts/tests/run_script_tests.sh

# Both checks always run, and the first failure's status is what the recipe
# returns. Left as two plain lines, validate_workspace.py's new exit 3 for an
# un-bootstrapped workspace (#609) aborted the recipe before
# test_layer_sourcing.sh — retiring ADR-0016's named enforcement path in every
# workspace worktree and every un-bootstrapped clone, which is exactly where
# agents work. The layer-sourcing guard needs no configs/manifest, so it has
# something to say in that state.
# The third check, rosdep_local_staleness_check.sh (#654), reports exit 3 for
# SKIPPED — it could not run the isolated `rosdep update` its verdict depends
# on (offline, no rosdep). That is not a finding, so it is printed and cleared;
# only its 1 (a stale or unmarked local key) and 2 (usage) fail the recipe.
# Its exit codes are distinct precisely so this accumulation can tell them
# apart instead of failing every offline workspace.
#
# GNU make flattens whatever this returns to its own 2; the code is preserved
# for a direct `./.agent/scripts/validate_workspace.py` call, not for `make`.
validate:
	@vrc=0; python3 ./.agent/scripts/validate_workspace.py || vrc=$$?; \
	lrc=0; ./.agent/scripts/test_layer_sourcing.sh || lrc=$$?; \
	rrc=0; ./.agent/scripts/rosdep_local_staleness_check.sh || rrc=$$?; \
	if [ "$$rrc" -eq 3 ]; then \
		echo "  (rosdep-local check SKIPPED — not a validation failure)"; rrc=0; \
	fi; \
	if [ "$$vrc" -ne 0 ]; then exit $$vrc; fi; \
	if [ "$$lrc" -ne 0 ]; then exit $$lrc; fi; \
	exit $$rrc

# =============================================================================
# Tier 1 — Setup chain (stamp-based dependencies)
# =============================================================================

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
	@mkdir -p $(STAMP)
	@./.agent/scripts/bootstrap.sh
	@touch $@

$(STAMP)/setup-dev.done: $(STAMP)/bootstrap.done $(MAIN_ROOT)/requirements.txt
	@mkdir -p $(STAMP)
	@python3 -m venv $(VENV_DIR) \
		|| { echo "Error: python3-venv is required. Install with:"; \
		     echo "  sudo apt install python3-venv"; exit 1; }
	@$(VENV_BIN)/pip install --upgrade pip -r requirements.txt
	@$(PRE_COMMIT) install
	@echo "Dev-tools venv ready at $(VENV_DIR)/. Git hooks installed."
	@touch $@

# Manifest stamp: bootstrap manifest repo only. Guard: if configs/manifest
# symlink is missing, the stamp is stale — delete it so Make re-runs.
$(STAMP)/manifest.done: $(STAMP)/bootstrap.done
	@mkdir -p $(STAMP)
	@if [ -f "$(STAMP)/manifest.done" ] && [ ! -L "$(MAIN_ROOT)/configs/manifest" ]; then \
		echo "Manifest symlink missing — re-running setup..."; \
		rm -f "$(STAMP)/manifest.done"; \
	fi
	@./.agent/scripts/setup_layers.sh $(if $(BOOTSTRAP_URL),--bootstrap-url "$(BOOTSTRAP_URL)") --manifest-only
	@touch $@

# Workspace-owned rosdep sources (#654): aggregate every project repo's root
# rosdep.yaml into $(MAIN_ROOT)/.rosdep/sources.list.d/ and refresh the cache.
# The wildcard is a PLAIN prerequisite list — no .SECONDEXPANSION needed, since
# this target has no `%` (unlike $(STAMP)/layer-%.done, whose prerequisite
# embeds $*). Makefiles are re-parsed every invocation, so the wildcard
# re-evaluates and the stamp goes stale when a rosdep.yaml is edited, added, or
# first appears with a newly checked-out repo.
#
# DELETION needs more than the wildcard, though, and deletion is the DOCUMENTED
# end of a local key's lifecycle (the upstream entry lands → delete the local
# file). A shrinking prerequisite list never makes a stamp stale, so the
# generated source list would keep a `yaml file://…` line for a file that no
# longer exists. The stamp therefore also depends on a file holding the CURRENT
# set of paths, rewritten (and so made newer than the stamp) only when that set
# actually changes — added, renamed or removed alike.
ROSDEP_LOCAL_YAMLS := $(wildcard $(MAIN_ROOT)/layers/main/*_ws/src/*/rosdep.yaml)

# FORCE (an ordinary target with no recipe and no file behind it) makes this
# rule run every invocation; the cmp keeps the file's MTIME unchanged unless
# its content differs, so it only triggers the stamp when the set really moved.
# Deliberately not .PHONY: .PHONY targets in this Makefile are published as
# /make_* slash commands.
FORCE:

$(STAMP)/rosdep-local.list: FORCE
	@mkdir -p $(STAMP)
	@printf '%s\n' $(ROSDEP_LOCAL_YAMLS) > $@.tmp
	@if cmp -s $@.tmp $@; then rm -f $@.tmp; else mv $@.tmp $@; fi

# `rosdep update` is best-effort: an offline host must not fail `make build`,
# and the generated source list is still correct for the next online run.
#
# The generator's exit 3 ("rosdep is not initialized" — no *.list under
# /etc/ros/rosdep/sources.list.d) is RECOVERABLE and must not break `make
# build`: it is the normal state of a clone that has not run `sudo rosdep init`
# yet, and the same condition bootstrap.sh treats as a note. The stamp is then
# deliberately NOT touched, so the next `make build` retries once rosdep is
# initialized (the prerequisite list would otherwise still be up to date and
# the generator would never run again). Any OTHER non-zero status — notably
# exit 4, a project repo's rosdep.yaml rejected by the shape rules — fails the
# build: those keys feed a root-level `rosdep install`, so a rejected file is a
# policy violation to fix, not a condition to build past.
$(STAMP)/rosdep-local.done: $(STAMP)/manifest.done $(STAMP)/rosdep-local.list $(ROSDEP_LOCAL_YAMLS)
	@mkdir -p $(STAMP)
	@rc=0; ./.agent/scripts/rosdep_local_sources.sh $(MAIN_ROOT) || rc=$$?; \
	if [ "$$rc" -eq 3 ]; then \
		echo "  (workspace-local rosdep sources not generated — rosdep is not"; \
		echo "   initialized; run .agent/scripts/bootstrap.sh. Using system defaults.)"; \
	elif [ "$$rc" -ne 0 ]; then \
		exit $$rc; \
	else \
		ROSDEP_SOURCE_PATH=$(MAIN_ROOT)/.rosdep/sources.list.d rosdep update \
			|| echo "  (rosdep update failed — offline? generated source list is still current)"; \
		touch $@; \
	fi

# Enable secondary expansion for the layer stamp rule below.
.SECONDEXPANSION:

# Per-layer stamps: re-run setup for a layer when its .repos file changes.
# Depends on manifest being bootstrapped first. Uses secondary expansion +
# $(wildcard) so a missing .repos file on fresh clones doesn't hard-fail,
# but once present, changes to the file will re-trigger setup.
$(STAMP)/layer-%.done: $(STAMP)/manifest.done $$(wildcard $(MAIN_ROOT)/configs/manifest/repos/$$*.repos)
	@mkdir -p $(STAMP)
	@./.agent/scripts/setup_layers.sh $(if $(BOOTSTRAP_URL),--bootstrap-url "$(BOOTSTRAP_URL)") "$*"
	@touch $@

$(STAMP)/git-bug.done: $(STAMP)/bootstrap.done
	@mkdir -p $(STAMP)
	@./.agent/scripts/git_bug_setup.sh
	@touch $@

# =============================================================================
# Tier 3 — Maintenance & agent container targets
# =============================================================================

skip-bootstrap:
	@mkdir -p $(STAMP)
	@touch $(STAMP)/bootstrap.done
	@echo "Bootstrap marked as done. Run 'make clean' to reset."

skip-git-bug:
	@mkdir -p $(STAMP)
	@touch $(STAMP)/git-bug.done
	@echo "git-bug setup marked as done. Run 'make clean' to reset."

sync:
	@python3 ./.agent/scripts/sync_repos.py

# Merge a PR, remove its worktree, delete branches, and sync. Keyed on the
# worktree/issue (PR# derived), not a global PR number. With no ISSUE=/PR=, the
# script uses the current worktree (cwd) — so run it directly from a worktree
# for the zero-arg path; via make, pass ISSUE= (or PR=) [+ REPO=<slug>].
merge-pr:
	@./.agent/scripts/merge_pr.sh $(if $(ISSUE),--issue $(ISSUE),) $(if $(PR),--pr $(PR),) $(if $(REPO),--repo-slug $(REPO),) $(if $(filter 1,$(NO_WAIT)),--no-wait,)

add-remote:
	@if [ -z "$(REMOTE)" ] || [ -z "$(URL_PREFIX)" ]; then \
		echo "Error: REMOTE and URL_PREFIX required"; \
		echo "Usage: make add-remote REMOTE=gitcloud URL_PREFIX=git@gitcloud:field/"; \
		exit 1; \
	fi
	@python3 ./.agent/scripts/add_remote.py --remote $(REMOTE) --url-prefix "$(URL_PREFIX)"

push-remote:
	@if [ -z "$(REMOTE)" ]; then \
		echo "Error: REMOTE parameter required"; \
		echo "Usage: make push-remote REMOTE=gitcloud [ALL=1] [SET_DEFAULT=1]"; \
		exit 1; \
	fi
	@python3 ./.agent/scripts/push_remote.py --remote $(REMOTE) $(if $(filter 1,$(ALL)),--all-branches,) $(if $(filter 1,$(SET_DEFAULT)),--set-default-branch,)

pull-remote:
	@if [ -z "$(REMOTE)" ]; then \
		echo "Error: REMOTE parameter required"; \
		echo "Usage: make pull-remote REMOTE=gitcloud [PULL=1] [BRANCH=<name>]"; \
		exit 1; \
	fi
	@python3 ./.agent/scripts/pull_remote.py --remote $(REMOTE) $(if $(filter 1,$(PULL)),--pull,) $(if $(BRANCH),--branch $(BRANCH),)

lock:
	@./.agent/scripts/lock.sh "Manual lock via Makefile"

unlock:
	@./.agent/scripts/unlock.sh

pr-triage:
	@./.agent/scripts/pr_status.sh --all-repos --simple

revert-feature:
	@if [ -z "$(ISSUE)" ]; then \
		echo "Error: ISSUE parameter required"; \
		echo "Usage: make revert-feature ISSUE=<number>"; \
		exit 1; \
	fi
	@./.agent/scripts/revert_feature.sh --issue $(ISSUE)

generate-skills:
	@./.agent/scripts/generate_make_skills.sh

# --- Agent container targets ---

agent-build:
	@# Single build path (#604): docker_run_agent.sh --build-only stages the
	@# rosdep manifests, runs `docker build`, and stamps the startup-scripts
	@# digest. Duplicating any of that here is what let the two paths hash
	@# different directories and produce a permanent false "stale" warning.
	@#
	@# The build resolves the MAIN workspace root even when run from a
	@# worktree — that is the tree the launcher mounts and hashes. So a
	@# worktree edit to agent-entrypoint.sh / fix-volume-ownership.sh is NOT
	@# baked until it is merged; the build prints a notice when that applies.
	@./.agent/scripts/docker_run_agent.sh --build-only

agent-run:
	@if [ -z "$(ISSUE)" ]; then \
		echo "Error: ISSUE parameter required"; \
		echo "Usage: make agent-run ISSUE=<number>"; \
		exit 1; \
	fi
	@./.agent/scripts/docker_run_agent.sh --issue $(ISSUE)

agent-shell:
	@if [ -z "$(ISSUE)" ]; then \
		echo "Error: ISSUE parameter required"; \
		echo "Usage: make agent-shell ISSUE=<number>"; \
		exit 1; \
	fi
	@./.agent/scripts/docker_run_agent.sh --issue $(ISSUE) --shell
