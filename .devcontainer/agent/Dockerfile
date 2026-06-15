# Sandboxed Agent DevContainer
# For running Claude Code in YOLO mode with container-level isolation.
# No SSH keys — pushes happen on the host. Optional read-only gh token via GH_TOKEN.

FROM ros:jazzy-perception

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in base image with same UID
RUN if id -u $USER_UID >/dev/null 2>&1; then userdel -r $(id -un $USER_UID); fi

# Create non-root user (no sudo — entrypoint runs as root, drops privileges)
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME

# Enable universe repo (needed for some rosdep dependencies)
RUN apt-get update && apt-get install -y software-properties-common \
    && add-apt-repository universe

# Install dev tools
RUN apt-get update && apt-get install -y \
    git \
    jq \
    curl \
    python3-pip \
    python3-vcstool \
    python3-venv \
    python3-yaml \
    shellcheck \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# Install Node.js 22.x via signed apt repo (no curl|bash)
RUN curl -fsSL https://deb.nodesource.com/gpgkey/nodesource-repo.gpg.key \
        | gpg --dearmor -o /usr/share/keyrings/nodesource.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/nodesource.gpg] https://deb.nodesource.com/node_22.x nodistro main" \
        > /etc/apt/sources.list.d/nodesource.list \
    && apt-get update && apt-get install -y nodejs \
    && rm -rf /var/lib/apt/lists/*

# Install GitHub CLI via signed apt repo (for read-only access inside container)
RUN curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg \
        | gpg --dearmor -o /usr/share/keyrings/githubcli.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli.gpg] https://cli.github.com/packages stable main" \
        > /etc/apt/sources.list.d/github-cli.list \
    && apt-get update && apt-get install -y gh \
    && rm -rf /var/lib/apt/lists/*

# Install git-bug (local-first issue tracking, see ADR-0010)
# Pinned SHA256 per arch detects tampering or asset substitution.
# Supports linux amd64 and armhf only; arm64 (Apple Silicon) and other arches
# are not available upstream for v0.10.1 and fail explicitly below.
ARG GITBUG_VERSION=v0.10.1
ARG GITBUG_SHA256_AMD64=3ba2f8b41e526fef1b6e825d5030823be65bb6521a287b1139bd609fed0d54a1
ARG GITBUG_SHA256_ARM=308b1d17bc4d12685a95c2908baf2b14b77e81737f34b3bd82a8baf10850fd87
RUN set -eux; \
    case "$(dpkg --print-architecture)" in \
        amd64) gitbug_arch=amd64; gitbug_sha256="${GITBUG_SHA256_AMD64}" ;; \
        armhf) gitbug_arch=arm;   gitbug_sha256="${GITBUG_SHA256_ARM}"   ;; \
        *)  echo "Unsupported architecture for git-bug ${GITBUG_VERSION}: $(dpkg --print-architecture)" >&2; \
            echo "Upstream provides amd64, 386, arm only (no arm64)." >&2; \
            exit 1 ;; \
    esac; \
    curl -fsSL "https://github.com/git-bug/git-bug/releases/download/${GITBUG_VERSION}/git-bug_linux_${gitbug_arch}" \
        -o /usr/local/bin/git-bug; \
    echo "${gitbug_sha256}  /usr/local/bin/git-bug" | sha256sum -c -; \
    chmod 755 /usr/local/bin/git-bug

# Install Claude Code CLI globally
RUN npm install -g @anthropic-ai/claude-code

# Initialize rosdep (as root, then update as user)
RUN rosdep init 2>/dev/null || true

# Bake workspace layer dependencies into the image (#520, #522).
# stage_rosdep_manifests.sh stages the layer package.xml manifests into
# .rosdep-manifests/ (build context); rosdep install them here so the system
# deps live in a read-only image layer. Each container FS is ephemeral (--rm),
# so this is what lets the launch-time `rosdep check` in agent-entrypoint.sh
# actually skip — only deps added since the last image build install at launch.
# Runs as root (apt needs root; the image has no sudo) and needs its own rosdep
# cache, hence the root-side `rosdep update`. The dev-tools blocks above each
# end with `rm -rf /var/lib/apt/lists/*`, so refresh the apt index first.
#
# All layers are installed in a SINGLE rosdep pass so --ignore-src ignores
# local packages across layers (an overlay package may depend on a local
# package in a lower layer — a per-layer pass would treat it as an
# unresolvable system key). But `rosdep install` is all-or-nothing on key
# resolution: a single unresolvable EXTERNAL key aborts the whole install.
# So first compute the genuinely-unresolvable keys via `rosdep check` and
# pass them as --skip-keys — one bad key then only skips itself instead of
# nuking the entire bake (#522). Still best-effort overall: anything skipped
# or failed falls through to the launch-time install path.
# The image must be built via docker_run_agent.sh --build or `make agent-build`,
# which create .rosdep-manifests/; a bare `docker build` won't have it.
COPY .rosdep-manifests/ /tmp/rosdep-manifests/
RUN apt-get update || echo "WARNING: apt-get update failed — skipping dep bake"; \
    rosdep update || echo "WARNING: rosdep update failed — skipping dep bake"; \
    skip_keys="$(rosdep check --from-paths /tmp/rosdep-manifests --ignore-src --rosdistro jazzy 2>&1 \
        | sed -n 's/.*Cannot locate rosdep definition for \[\([^]]*\)\].*/\1/p' \
        | sort -u | tr '\n' ' ')"; \
    if [ -n "$skip_keys" ]; then \
        echo "rosdep bake: skipping unresolvable keys: $skip_keys"; \
    fi; \
    rosdep install --from-paths /tmp/rosdep-manifests --ignore-src -y --rosdistro jazzy \
        ${skip_keys:+--skip-keys "$skip_keys"} \
        || echo "WARNING: some layer deps did not resolve at bake time — they will install at launch"; \
    rm -rf /tmp/rosdep-manifests /var/lib/apt/lists/*

USER $USERNAME

# Update rosdep as non-root user (the launch-time `rosdep check` in the
# entrypoint runs under this user and needs its own populated cache)
RUN rosdep update

# Entrypoint runs as root to fix volume ownership, then drops to $USERNAME
# via setpriv (no setuid bits — compatible with --security-opt no-new-privileges)
USER root
COPY --chmod=755 agent-entrypoint.sh /usr/local/bin/agent-entrypoint.sh
ENTRYPOINT ["agent-entrypoint.sh"]
CMD ["claude", "--dangerously-skip-permissions"]
