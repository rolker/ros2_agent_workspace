# Document ROS2 Code

Generate or update documentation for ROS2 packages.

## Steps

1. **README.md**:
   - Ensure every package has a `README.md`.
   - **Sections Required**:
     - `## Overview`: What the package does.
     - `## Installation`: How to build/install.
     - `## Usage`: Launch commands or run verification.
     - `## key Topics/Services`: List of interface endpoints.
     - `## Parameters`: Table of ROS parameters.

2. **Code Comments**:
   - **Python**: Ensure Google-style docstrings for classes and functions.
   - **C++**: Ensure Doxygen-compatible comments (`///`) for public headers.

3. **package.xml**:
   - Verify `<description>`, `<maintainer>`, and `<license>` are filled out.
