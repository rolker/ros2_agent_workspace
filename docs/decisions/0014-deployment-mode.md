# ADR-0014: Deployment Mode — An Urgency-Aware Behavioral Mode for Live Field Operations

## Status

Accepted — both lifecycle skills have shipped (`/start-deployment`,
[#501](https://github.com/rolker/ros2_agent_workspace/issues/501); `/wrap-up-deployment`,
[#530](https://github.com/rolker/ros2_agent_workspace/issues/530)) and the urgency
contract has been exercised across multiple live deployments
(`unh_echoboats_project11` #277 / #289 / #295 / #299).

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

**Per-agent-session**: invoking the activation skill (`/start-deployment`,
[#501](https://github.com/rolker/ros2_agent_workspace/issues/501)) puts the
current agent session in deployment mode. Other agent sessions on the same host
doing unrelated work are unaffected. No filesystem marker and no host-wide
hook — on-disk evidence of an ongoing deployment is what already exists
naturally: an open `deployment`-labeled issue, plus a worktree (dev side) or a
host log file (field side). This works identically on dev and field hosts (no
GitHub dependency — field hosts have no GitHub access, so the field
`issue_sync` mechanism — e.g. git-bug + a shared field remote — surfaces the
issue there). The operator may also dial the autonomy level (adjustable
autonomy); deployment mode is a setting, not a fixed behavior.

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
| Deployment (this ADR) | per-session, operator-invoked via `/start-deployment` | behavior under live time pressure |
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
  origin-detected). It relies on the operator invoking the activation skill and
  the agent honoring the embedded urgency contract — mitigated by embedding the
  contract verbatim in the skill (so invocation puts the rules in context) and
  by leaving the door open for a future framework-level hook if the
  skill-invocation channel proves insufficient.
- "Defer to wrap-up" carries risk if wrap-up is skipped — mitigated by the recovery
  checklist ([#496](https://github.com/rolker/ros2_agent_workspace/issues/496)) and
  wrap-up orchestration.
- Drawing and maintaining the three-tier boundary takes ongoing discipline.
- Per-session activation means a second agent session on the same host won't
  inherit deployment mode automatically — the operator has to invoke
  `/start-deployment` in each session that should run under the contract. This
  is by design (other sessions doing unrelated work shouldn't be constrained),
  but it does require operator awareness.

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
