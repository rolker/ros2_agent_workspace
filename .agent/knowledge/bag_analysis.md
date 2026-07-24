# Bag & Deployment Data Analysis

Conventions for analyzing recorded deployment data (rosbags, router stats,
deployment logs). Field-earned; each rule exists because its violation
produced a wrong conclusion at least once.

## Pipeline

- **Use the standard `bag_analysis` pipeline**: `bag_to_sqlite` → SQLite DB →
  analysis scripts. Don't hand-roll ad-hoc mcap readers per question — the
  pipeline handles topic extraction, typing, and indexing once, and
  downstream scripts stay comparable across analyses.
- **Read bags offline for batch data products** — never replay a bag through
  a live ROS node to regenerate products. Offline reads are faster,
  deterministic, and can't cross-contaminate a live graph.

## Time discipline

- **Derive event times from the data, not from logs**: dock departure/return
  and other physical events come from bag GPS (and battery/actuator
  signals), not from launch/log timestamps.
- **Agent-log timestamps are unreliable while the operator is away** — an
  idle field agent's log lines cluster on wake-ups, not on events. Confirm
  any operationally meaningful time against the bag.
- **Exclude launch settling**: drop the first ~30 s after a stack launch
  (and a buffer after recoveries) when analyzing estimator-derived values —
  estimates haven't converged and will skew statistics.
- **Stamp retrieval time** on any fetched, time-sensitive data you record
  (weather forecasts, tide predictions, API pulls) — the fetch time is part
  of the datum.

## Units & sources

- **MikroTik and udp_bridge `*_bytes_per_second` counters are bytes/s** —
  multiply by 8 for Mbps. **Starlink reports bits/s.** Mixing these up
  produces 8× link-utilization errors (it has happened).

## Deployment review products

- **Review videos**: build one synchronized multi-panel video (all cameras +
  segmentation + costmap) rather than per-topic clips; verify event timing
  against the bag before annotating.
- **Recording is selective and split across locations** — see the
  platform's data-locations doc (for BizzyBoat:
  `unh_echoboats_project11/docs/logs/README.md`) before concluding
  something "wasn't recorded".
