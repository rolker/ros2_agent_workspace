FROM ros:jazzy-perception

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in base image (e.g. ubuntu) with same UID
RUN if id -u $USER_UID >/dev/null 2>&1; then userdel -r $(id -un $USER_UID); fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install basic dev tools
RUN apt-get update && apt-get install -y \
    git \
    openssh-client \
    python3-pip \
    shellcheck \
    vim \
    nano \
    # VCSTool for repo management
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Pre-install python deps
COPY requirements.txt /tmp/requirements.txt

USER $USERNAME
RUN pip3 install --user --break-system-packages -r /tmp/requirements.txt
CMD ["/bin/bash"]
