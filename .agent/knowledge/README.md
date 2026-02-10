# Workspace Knowledge Index

This directory contains symbolic links to key documentation found throughout the workspace. Agents should use these files to understand the system architecture, component responsibilities, and usage instructions.

## System Overview
- **[Autonomy Overview](system__autonomy_overview.md)**: General architecture of the UNH Marine Autonomy system (`marine_autonomy`).

## Core Components
- **[Mission Manager](component__mission_manager.md)**: Logic for high-level mission state machines.
- **[Helm Manager](component__helm_manager.md)**: Control arbitration and behavior execution.
- **[Marine AIS](component__marine_ais.md)**: Handling of AIS data and messages.

## User Interface
- **[CAMP UI](component__camp_ui.md)**: The Common Autonomy Mission Planner (CAMP) interface.
- **[CAMP Architecture](architecture__camp_docs)**: Detailed design docs for the UI.

## Simulation
- **[Marine Simulation](component__simulation.md)**: VRX and Gazebo simulation environments.

---
*Note: This index is manually maintained. If you add new major components, please update this file and create the corresponding symlinks.*

# 🔍 Search Tips for Agents
> **Important**: The source code in `layers/` is **gitignored** (managed by `vcs`).
> Tools like `find_by_name` will **not** find files in `src/` if searching from the root.
>
> **Best Practice**:
> 1. Use `list_dir` to explore directories (it sees everything).
> 2. Search specifically in subdirectories if needed.
> 3. Rely on this Knowledge Index for high-level navigation.
