# ADR-0014: Deployment Mode — An Urgency-Aware Behavioral Mode for Live Field Operations

## Status

Proposed (Draft) — recorded ahead of full implementation so the design has a
stable reference; some components (activation marker, lifecycle skills) may land
before this ADR is marked Accepted.

## Context

Field deployments of autonomous robot boats put the agent in a **live-assistant**
role: troubleshooting, hot-fixes, and situational response while the operator runs
the boat. On-water operating/testing time is the scarce resource, and the pre-class
deployment calendar (June 2026 freeze) makes cycle latency the bottleneck.

Two recurring problems motivated this (see [#495](https://github.com/rolker/ros2_agent_workspace/issues/495),
[#477](https://github.com/rolker/ros2_agent_workspace/issues/477), and the
`unh_echoboats_project11` deployment logs):

1. **Agents rabbit-hole under time pressure** — deep-diving an investigation or
   over-polishing setup when the operator needed a fast response, consuming
   irreplaceable operating time.
2. **The deployment lifecycle** (start → session → recovery → wrap-up → next) has
   accumulated as a project-local convention with no workspace tooling.

The workspace already has a precedent for a context-scoped exception to default
agent behavior: **field mode** ([ADR-0011](0011-field-mode-for-non-github-origins.md)),
detected by origin URL, relaxing commit/PR ceremony. Deployment mode is a sibling
on a different axis — not *where commits go*, but *how the agent behaves under live
time pressure*.

The behavioral half is grounded in established operational practice (incident
command / SRE, the aviation sterile-cockpit rule, supervisory control / adjustable
autonomy) — see the workspace research digest entry "Operational-Assistant Behavior
Under Live Time Pressure."

## Decision

Define **deployment mode**: a behavioral operating mode, active during a live field
deployment, with two co-equal halves — (1) an **urgency contract** (behavioral
adjustments) and (2) the **lifecycle tooling** that runs inside it.

### Activation

An **operator-set marker**: a slash command / skill writes a state marker at
deployment start and clears it at wrap-up. It works identically on dev and field
hosts (no GitHub dependency — field hosts have no GitHub access). The operator may
also dial the autonomy level (adjustable autonomy); deployment mode is a setting,
not a fixed behavior.

### The urgency contract (phase-aware behavioral adjustments)

- **Sterile cockpit** — during live on-water ops, do *only* operation-essential
  work; no doc polish, refactors, or speculative deep-dives.
- **Mitigate before diagnose** — stabilize / hot-fix now; defer root-cause analysis
  to wrap-up.
- **Confidence-threshold handoff** — below confidence, or outside a known failure
  class, stop and surface a prepared summary to the operator instead of spiraling;
  run cheap deterministic checks before expensive investigation.
- **Phase-aware** — the contract is strongest in live ops and **relaxes in wrap-up**,
  where deep analysis belongs. Lifecycle: [0] start → [1] session → [2] recovery →
  [3] wrap-up → [4] next (≈ the SRE Prepare → Detect → Respond → Recover → Learn loop).

### Invariants (what deployment mode does NOT relax)

- **Safety-critical correctness** (boat control, collision avoidance) is never
  traded for speed.
- Pre-commit hooks, AI signature, atomic commits, no secrets, no destructive ops
  without approval — same as field mode.
- **Defer is not skip** — deferred RCA / tests / docs are *tracked* (follow-up
  issues, roadmap, wrap-up checklist), not dropped. The Quality Standard's "never
  table when the permanent solve is near" still holds; deployment mode only defers
  *within a live session*, and wrap-up is where completeness is restored.
- **Human control** — the operator is continuously in the loop, so deployment mode
  *increases* responsiveness precisely because real-time oversight is higher. This
  is the "tight by default, relaxable as confidence grows" principle, not a
  weakening of it.

### Content split (per [ADR-0003](0003-workspace-infrastructure-is-project-agnostic.md))

| Tier | Content | Home |
|---|---|---|
| Generic | mode concept, urgency contract, lifecycle skeleton, log naming, recovery machinery, activation skill | workspace `.agent/` + skills + this ADR |
| Project-general (marine) | tides-in-metres, OTH, gitcloud reconciliation, marine nav rates, dev-vs-`hostname` label rule | `unh_marine_autonomy/.agents/workspace-context/` |
| Platform-specific | host inventory (gabby/salmon/mercat/dora), bag paths, sensor specifics | `unh_echoboats_project11` (README becomes a thin pointer) |

### Deployment identity (multi-platform / multi-day)

A deployment is **one issue**, not bound to a single day or platform. Hosts across
platforms each keep a per-host log; the filename's date is the deployment **start**
date; "Hosts active" spans platforms (e.g. dev, salmon, gabby [BizzyBoat], dora
[IzzyBoat]).

### Modes family (framing, not a framework)

Deployment mode is one of an emerging family of agent operating modes:

| Mode | Trigger | Adjusts |
|---|---|---|
| Development (default) | the normal case | full worktree/PR ceremony |
| Field ([ADR-0011](0011-field-mode-for-non-github-origins.md)) | origin not on GitHub | where commits go |
| Deployment (this ADR) | operator-set marker | behavior under live time pressure |
| Device/tmux troubleshooting (informal, [#420](https://github.com/rolker/ros2_agent_workspace/issues/420)) | interactive hardware session | collaboration rules |

They share a structure — **trigger / behavioral adjustments / invariants**. This ADR
*names* the pattern but does not create a general framework; a unifying "agent
operating modes" ADR may follow once ≥3 modes are formalized.

## Consequences

**Positive:**

- Directly attacks the rabbit-hole / lost-operating-time failure mode.
- Reuses the field-mode precedent; composes with existing skills rather than
  replacing them.
- Gives the lifecycle tooling a stable behavioral contract to build against.
- The three-tier split makes the capability adoptable by other marine platforms.

**Negative / risks:**

- A *behavioral* mode is harder to enforce mechanically than field mode (which is
  origin-detected). It relies on the agent reading the marker and honoring the
  contract — mitigated by making the marker explicit and surfaced in context, with
  a possible future hook.
- "Defer to wrap-up" carries risk if wrap-up is skipped — mitigated by the recovery
  checklist ([#496](https://github.com/rolker/ros2_agent_workspace/issues/496)) and
  wrap-up orchestration.
- Drawing and maintaining the three-tier boundary takes ongoing discipline.
- The activation marker is one more thing to set/clear — owned by the start/wrap-up
  skills so the operator isn't doing it by hand.

## References

- [ADR-0011](0011-field-mode-for-non-github-origins.md) — Field Mode (sibling; this
  ADR follows its trigger/adjustments/invariants shape)
- [ADR-0003](0003-workspace-infrastructure-is-project-agnostic.md) — Workspace
  Infrastructure Is Project-Agnostic (the three-tier split)
- [#495](https://github.com/rolker/ros2_agent_workspace/issues/495) — Deployment-mode
  umbrella · [#477](https://github.com/rolker/ros2_agent_workspace/issues/477) —
  logging convention · [#496](https://github.com/rolker/ros2_agent_workspace/issues/496) —
  recovery checklist
- Workspace research digest — "Operational-Assistant Behavior Under Live Time Pressure"
