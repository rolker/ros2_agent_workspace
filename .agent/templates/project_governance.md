# Project Governance Layout

Suggested layout for project repos that want their governance documents
discoverable by workspace tools. **This layout is optional** — the discovery
script searches broadly and will find governance documents regardless of
where they live. This template is for projects that want a consistent,
recommended structure.

## Suggested Structure

```
<project-repo>/
├── .agents/
│   ├── README.md               # Agent guide (see project_agents_guide.md template)
│   ├── work-plans/             # Task plans for this repo
│   └── workspace-context/      # Symlinked into workspace as .agent/project_knowledge/
├── PRINCIPLES.md               # Project-level guiding principles
├── ARCHITECTURE.md             # System design and component relationships
└── docs/
    └── decisions/              # Architecture Decision Records (ADRs)
        └── 0001-example.md
```

## What Each File Does

| File | Purpose | When to Add |
|------|---------|-------------|
| `.agents/README.md` | Agent onboarding: packages, layout, build, pitfalls | Always — first file agents read |
| `PRINCIPLES.md` | Project-specific guiding principles | When the project has domain rules beyond workspace principles |
| `ARCHITECTURE.md` | Design overview, data flows, component relationships | When the project has multiple packages or non-obvious structure |
| `docs/decisions/` | ADRs recording significant design choices | When making decisions worth preserving for future contributors |
| `.agents/work-plans/` | Task implementation plans | Created automatically by planning workflow |
| `.agents/workspace-context/` | Content shared with the workspace | When the project participates in a ROS 2 Agent Workspace |

## Adoption Levels

Not every project needs full governance. Start minimal and add as needed:

1. **Minimal**: `.agents/README.md` only — enough for agents to navigate the repo
2. **Standard**: Add `ARCHITECTURE.md` when structure isn't self-evident
3. **Full**: Add `PRINCIPLES.md` and `docs/decisions/` for projects with domain-specific
   rules or significant design choices

## Relationship to Workspace Governance

- **Workspace principles** govern *process* — how work is done across all projects
- **Project principles** govern *domain* — what is built and why
- Both apply; neither is optional. Project rules take precedence on domain questions.
- The discovery script aggregates governance docs from all levels.

## Related Templates

- [`project_agents_guide.md`](project_agents_guide.md) — Template for `.agents/README.md`

## Instructions for Use

1. Share this document with project maintainers as a reference — don't copy it into
   every repo.
2. Start with `.agents/README.md` using the
   [`project_agents_guide.md`](project_agents_guide.md) template.
3. Add other files as the project's needs grow.
4. The discovery script will find governance docs wherever they are — this layout
   just makes them predictable.
