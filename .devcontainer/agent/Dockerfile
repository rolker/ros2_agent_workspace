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
