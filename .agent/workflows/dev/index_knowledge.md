# Index Knowledge

Populate the `.agent/knowledge` directory with relevant context from the codebase.

## Purpose
Agents work better when high-level documentation (architecture, design docs) is readily available without searching deep into packages.

## Steps

1.  **Run Generation Script**:
    ```bash
    ./scripts/generate_knowledge.sh
    ```

2.  **Verify**:
    - Check `.agent/knowledge/` to see that new links were created.
    - If you added new repositories, you may need to edit `scripts/generate_knowledge.sh` to include them.

3.  **Update Index**:
    - If you added new types of documentation, update `.agent/knowledge/README.md` to describe them.
