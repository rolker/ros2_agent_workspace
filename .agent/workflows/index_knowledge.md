# Index Knowledge

Populate the `.agent/knowledge` directory with relevant context from the codebase.

## Purpose
Agents work better when high-level documentation (architecture, design docs) is readily available without searching deep into packages.

## Steps

1.  **Identify Key Docs**:
    - Look for `architecture.md`, `design.md`, or detailed `README.md` files in key packages.
    - *Example*: `workspaces/ui_ws/src/camp/docs/architecture_design.md`

2.  **Create Symbolic Links**:
    - Link them into `.agent/knowledge/` with descriptive names.
    - *Naming Convention*: `<topic>__<original_name>.md`
    - *Example*:
      ```bash
      ln -sf ../../workspaces/ui_ws/src/camp/docs/architecture_design.md .agent/knowledge/architecture__camp.md
      ```

3.  **Create Summaries (Optional)**:
    - If a document is too large, create a summary file `summary__<topic>.md` in `.agent/knowledge/`.
