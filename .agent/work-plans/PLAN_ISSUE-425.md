# Plan: Allow bootstrap URL override for alternate deployments

## Issue

https://github.com/rolker/ros2_agent_workspace/issues/425

## Context

`setup_layers.sh` reads the bootstrap URL from the committed file
`configs/project_bootstrap.url` with no override mechanism. This blocks
deploying the workspace to environments where the manifest is hosted on a
different server (e.g., Forgejo/gitcloud) because the raw file URL format
differs and the committed URL returns 404.

This is a focused first step toward the broader first-run UX in #330.

## Approach

1. **Add `--bootstrap-url` flag parsing to `setup_layers.sh`** — parse it
   early (before `LAYER_NAME` assignment) and store in a variable. Must
   coexist with existing flags (`--manifest-only`).

2. **Add URL resolution logic in `bootstrap_manifest_repo()`** — priority
   order:
   1. `BOOTSTRAP_URL` env var (highest)
   2. `--bootstrap-url <url>` CLI flag
   3. Interactive prompt (if not `CI`/`NONINTERACTIVE` and stdin is a
      terminal) — shows default from file if present, allows override
   4. `configs/project_bootstrap.url` file contents (current behavior)
   5. Error with clear message if none of the above

3. **Add interactive prompt** — when running interactively (no
   `CI`/`NONINTERACTIVE`, stdin is a TTY), show the default URL from the
   file (if present) and let the user confirm or provide an alternative.
   If no file exists, prompt without a default. Uses `read -r -p`.

4. **Pass `BOOTSTRAP_URL` through Makefile** — the manifest stamp rule
   (line 164) calls `setup_layers.sh --manifest-only`. If `BOOTSTRAP_URL`
   is set, pass it as `--bootstrap-url` arg. Similarly for per-layer stamp
   rules (line 176).

5. **Update script header comments** — document the new flag and env var
   in the usage block at the top of `setup_layers.sh`.

## Files to Change

| File | Change |
|------|--------|
| `.agent/scripts/setup_layers.sh` | Add flag parsing, env var check, interactive prompt in `bootstrap_manifest_repo()` |
| `Makefile` | Pass `BOOTSTRAP_URL` to `setup_layers.sh` calls |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Workspace vs. project separation | This strengthens it — the workspace no longer silently assumes a specific project's URL format works |
| Only what's needed | Minimal change: one env var, one flag, one prompt. No new files or abstractions |
| Human control and transparency | Interactive prompt makes the bootstrap URL visible and overridable |
| Improve incrementally | Focused step toward #330's broader first-run UX |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0003 (project-agnostic) | Yes | This change makes the workspace more project-agnostic by not hardcoding URL assumptions |
| ADR-0007 (Make + stamps) | Yes | `BOOTSTRAP_URL` passes through the existing stamp-based flow; no new stamps needed |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `setup_layers.sh` CLI interface | Script reference in `AGENTS.md` | No — script is not individually documented there; usage header suffices |
| Makefile manifest rule | `make help` output | No — no new target, just env var passthrough |

## Open Questions

- Should the interactive prompt also offer to save the user's choice back
  to `configs/project_bootstrap.url`? (Leaning no — that's #330 territory
  and would modify a tracked file.)

## Estimated Scope

Single PR.
