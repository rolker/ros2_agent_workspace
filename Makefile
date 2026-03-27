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

# --- Layer list (read from manifest config if available) ---
LAYERS_FILE := $(MAIN_ROOT)/configs/manifest/layers.txt
ifneq ($(wildcard $(LAYERS_FILE)),)
  LAYERS := $(shell awk '!/^[[:space:]]*($$|#)/ {$$1=$$1; print}' $(LAYERS_FILE) | tr '\n' ' ')
else
  LAYERS := underlay core platforms sensors simulation ui
endif
LAYER_STAMPS := $(patsubst %,$(STAMP)/layer-%.done,$(LAYERS))

# --- Phony targets ---
.PHONY: help build test lint clean setup-all dashboard dashboard-ui test-dashboard validate sync lock unlock revert-feature pr-triage generate-skills skip-bootstrap skip-git-bug agent-build agent-run agent-shell push-gateway add-remote push-remote pull-remote

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
	@echo "  validate      - Validate workspace config (CI-oriented, pass/fail)"
	@echo ""
	@echo "Remote sync:"
	@echo "  add-remote REMOTE=<name> URL_PREFIX=<prefix> - Add remote to all repos"
	@echo "  push-remote REMOTE=<name> [ALL=1]            - Push to named remote"
	@echo "  pull-remote REMOTE=<name> [PULL=1|BRANCH=<b>] - Fetch/pull from remote"
	@echo ""
	@echo "Maintenance:"
	@echo "  sync          - Safely sync all workspace repositories"
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
	@echo "  push-gateway  - Process pending push requests from containers"
	@echo ""

build: $(LAYER_STAMPS)
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

setup-all: $(LAYER_STAMPS) $(STAMP)/git-bug.done
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

validate:
	@python3 ./.agent/scripts/validate_workspace.py

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
	@./.agent/scripts/setup_layers.sh --manifest-only
	@touch $@

# Enable secondary expansion for the layer stamp rule below.
.SECONDEXPANSION:

# Per-layer stamps: re-run setup for a layer when its .repos file changes.
# Depends on manifest being bootstrapped first. Uses secondary expansion +
# $(wildcard) so a missing .repos file on fresh clones doesn't hard-fail,
# but once present, changes to the file will re-trigger setup.
$(STAMP)/layer-%.done: $(STAMP)/manifest.done $$(wildcard $(MAIN_ROOT)/configs/manifest/repos/$$*.repos)
	@mkdir -p $(STAMP)
	@./.agent/scripts/setup_layers.sh "$*"
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
		echo "Usage: make push-remote REMOTE=gitcloud [ALL=1]"; \
		exit 1; \
	fi
	@python3 ./.agent/scripts/push_remote.py --remote $(REMOTE) $(if $(ALL),--all-branches,)

pull-remote:
	@if [ -z "$(REMOTE)" ]; then \
		echo "Error: REMOTE parameter required"; \
		echo "Usage: make pull-remote REMOTE=gitcloud [PULL=1] [BRANCH=<name>]"; \
		exit 1; \
	fi
	@python3 ./.agent/scripts/pull_remote.py --remote $(REMOTE) $(if $(PULL),--pull,) $(if $(BRANCH),--branch $(BRANCH),)

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
	@docker build \
		--build-arg USER_UID=$$(id -u) \
		--build-arg USER_GID=$$(id -g) \
		-t ros2-agent-workspace-agent:latest \
		-f .devcontainer/agent/Dockerfile \
		.devcontainer/agent/

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

push-gateway:
	@./.agent/scripts/push_gateway.sh
