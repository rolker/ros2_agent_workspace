# Universal AI Agent Rules

**This file is a redirect.** All shared workspace rules now live in
[`AGENTS.md`](../AGENTS.md) at the repository root (see ADR-0006).

## Framework Entry Points

| Framework | Instruction File |
|-----------|-----------------|
| Claude Code | [`CLAUDE.md`](../CLAUDE.md) (auto-loaded) |
| GitHub Copilot | [`.github/copilot-instructions.md`](../.github/copilot-instructions.md) |
| Gemini CLI | [`.agent/instructions/gemini-cli.instructions.md`](instructions/gemini-cli.instructions.md) |
| Other / Unknown | [`AGENT_ONBOARDING.md`](AGENT_ONBOARDING.md) |

All framework files reference `AGENTS.md` for shared rules and add only
framework-specific configuration.
