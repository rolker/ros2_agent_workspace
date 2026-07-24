# Diagnosis Discipline

Rules for root-causing failures in a field-robotics workspace, where
partial data, multi-host systems, and human actions make plausible-but-wrong
narratives cheap to construct. Each rule traces to a real misdiagnosis.

## Verify before narrating

- **Check the suspect component's *current* state before declaring "same
  bug as last time"** — precedent plus a similar symptom is a hypothesis,
  not a diagnosis. The config/code may have changed since the precedent.
- **Identical observations prove identity, not causation.** Two matching
  values/artifacts don't establish that one derived from the other; say
  "these match", not "A caused B", until you have a mechanism.
- **Verify "X is missing from Y" against a known-good case first.** Tooling
  artifacts (filters, truncation, wrong query) masquerade as data gaps;
  reproduce a case where X is known present through the same tooling before
  claiming absence.
- **Only verified-deployed components belong on a suspect list.** Naming a
  component that isn't actually in the running stack sends the
  investigation down a dead end; check what's deployed or ask.

## Respect the observer

- **A repeated operator eyewitness observation outranks a partial-data
  conclusion.** If the operator saw it twice and your analysis says it
  can't happen, the analysis is missing data — reproduce the observation
  before dismissing it.

## Human actions first

- **A `SIGKILL`/-9 in a launch log usually means an operator restart** (plus
  rmw_zenoh's slow close making the supervisor escalate), **not a crash or
  OOM.** Before narrating a failure from process exits, confirm what humans
  did at that time and whether shutdown ordering explains the signals.

## Measured data is looked up, never estimated

- **Never estimate a quantity the project has measured** — mounting offsets
  live in the URDF and `/tf_static`; rates, limits, and calibrations live in
  configs. Fabricating a "reasonable" value for known data is a serious
  error even when it's close.
