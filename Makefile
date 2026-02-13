.PHONY: help bootstrap setup-core setup-all build test clean status status-quick lock unlock format lint setup-dev health-check sync validate revert-feature

VENV_DIR := .venv
VENV_BIN := $(VENV_DIR)/bin
PRE_COMMIT := $(VENV_BIN)/pre-commit

# Default target
help:
	@echo "ROS2 Agent Workspace - Makefile"
	@echo ""
	@echo "Available targets:"
	@echo "  help          - Show this help message"
	@echo "  health-check  - Run comprehensive workspace health check"
	@echo "  bootstrap     - Install ROS2 and dependencies"
	@echo "  setup-core    - Setup core workspace layer"
	@echo "  setup-all     - Setup all workspace layers"
	@echo "  build         - Build all workspace layers"
	@echo "  test          - Run tests on all layers"
	@echo "  clean         - Clean build artifacts"
	@echo "  status        - Show full status (sync + repos + GitHub PRs/issues)"
	@echo "  status-quick  - Show quick local-only status (no sync, no GitHub)"
	@echo "  lock          - Lock workspace for exclusive access"
	@echo "  unlock        - Unlock workspace"
	@echo "  sync          - Safely sync all workspace repositories"
	@echo "  validate      - Validate workspace configuration"
	@echo "  setup-dev     - Create dev-tools venv and install pre-commit"
	@echo "  format        - Format Python code with black"
	@echo "  lint          - Run pre-commit hooks on all files"
	@echo "  revert-feature ISSUE=<number> - Revert all commits for a specific issue"
	@echo ""

health-check:
	@./.agent/scripts/health_check.sh

bootstrap:
	@./.agent/scripts/bootstrap.sh

setup-core:
	@./.agent/scripts/setup.sh core

setup-all: setup-underlay setup-core setup-platforms setup-sensors setup-simulation setup-ui

setup-underlay:
	@./.agent/scripts/setup.sh underlay

setup-platforms:
	@./.agent/scripts/setup.sh platforms

setup-sensors:
	@./.agent/scripts/setup.sh sensors

setup-simulation:
	@./.agent/scripts/setup.sh simulation

setup-ui:
	@./.agent/scripts/setup.sh ui

build:
	@./.agent/scripts/build.sh

test:
	@./.agent/scripts/test.sh

clean:
	@echo "Cleaning build artifacts..."
	@find layers -type d -name "build" -exec rm -rf {} + 2>/dev/null || true
	@find layers -type d -name "install" -exec rm -rf {} + 2>/dev/null || true
	@find layers -type d -name "log" -exec rm -rf {} + 2>/dev/null || true
	@echo "Done."

status:
	@./.agent/scripts/status_report.sh

status-quick:
	@./.agent/scripts/status_report.sh --quick

lock:
	@./.agent/scripts/lock.sh "Manual lock via Makefile"

unlock:
	@./.agent/scripts/unlock.sh

sync:
	@python3 ./.agent/scripts/sync_repos.py

validate:
	@python3 ./.agent/scripts/validate_workspace.py

revert-feature:
	@if [ -z "$(ISSUE)" ]; then \
		echo "Error: ISSUE parameter required"; \
		echo "Usage: make revert-feature ISSUE=<number>"; \
		exit 1; \
	fi
	@./.agent/scripts/revert_feature.sh --issue $(ISSUE)

format: $(PRE_COMMIT)
	@echo "Formatting Python code with black..."
	@$(PRE_COMMIT) run black --all-files
	@echo "Done."

$(PRE_COMMIT):
	@python3 -m venv $(VENV_DIR) \
		|| { echo "Error: python3-venv is required.  Install it with:"; \
		     echo "  sudo apt install python3-venv"; exit 1; }
	@$(VENV_BIN)/pip install --upgrade pip pre-commit
	@echo "Dev-tools venv ready at $(VENV_DIR)/"

setup-dev: $(PRE_COMMIT)
	@$(PRE_COMMIT) install
	@echo "Dev-tools venv is ready. Git hooks installed."

lint: $(PRE_COMMIT)
	@$(PRE_COMMIT) run --all-files
