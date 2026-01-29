.PHONY: help bootstrap setup-core setup-all build test clean status lock unlock install-deps format lint health-check

# Default target
help:
	@echo "ROS2 Agent Workspace - Makefile"
	@echo ""
	@echo "Available targets:"
	@echo "  help          - Show this help message"
	@echo "  health-check  - Run comprehensive workspace health check"
	@echo "  bootstrap     - Install ROS2 and dependencies"
	@echo "  install-deps  - Install Python dependencies"
	@echo "  setup-core    - Setup core workspace layer"
	@echo "  setup-all     - Setup all workspace layers"
	@echo "  build         - Build all workspace layers"
	@echo "  test          - Run tests on all layers"
	@echo "  clean         - Clean build artifacts"
	@echo "  status        - Show workspace status report"
	@echo "  lock          - Lock workspace for exclusive access"
	@echo "  unlock        - Unlock workspace"
	@echo "  format        - Format Python code with black"
	@echo "  lint          - Run linters on all code"
	@echo ""

health-check:
	@./.agent/scripts/health_check.sh

bootstrap:
	@./.agent/scripts/bootstrap.sh

install-deps:
	@echo "Installing Python dependencies..."
	@pip3 install -r requirements.txt
	@echo "Done."

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

lock:
	@./.agent/scripts/lock.sh "Manual lock via Makefile"

unlock:
	@./.agent/scripts/unlock.sh

format:
	@echo "Formatting Python code with black..."
	@black --line-length 100 .agent/scripts/*.py
	@echo "Done."

lint: lint-shell lint-python

lint-shell:
	@echo "Running shellcheck on shell scripts..."
	@shellcheck .agent/scripts/*.sh || true
	@echo "Done."

lint-python:
	@echo "Running flake8 on Python scripts..."
	@flake8 --max-line-length=100 --extend-ignore=E203,W503 .agent/scripts/*.py || true
	@echo "Running pylint on Python scripts..."
	@pylint --max-line-length=100 --disable=C0114,C0115,C0116 .agent/scripts/*.py || true
	@echo "Done."
