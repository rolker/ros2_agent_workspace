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
# Upstream v0.10.1 publishes linux binaries for amd64, 386, and arm (32-bit) only;
# arm64 (Apple Silicon) is not available upstream and fails explicitly below.
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

USER $USERNAME

# Update rosdep as non-root user
RUN rosdep update

# Entrypoint runs as root to fix volume ownership, then drops to $USERNAME
# via setpriv (no setuid bits — compatible with --security-opt no-new-privileges)
USER root
COPY --chmod=755 agent-entrypoint.sh /usr/local/bin/agent-entrypoint.sh
ENTRYPOINT ["agent-entrypoint.sh"]
CMD ["claude", "--dangerously-skip-permissions"]
