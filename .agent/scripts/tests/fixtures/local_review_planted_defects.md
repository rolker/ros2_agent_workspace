# Planted defects — `local_review_planted_defects.diff`

Scoring fixture for the local adversarial reviewer (specialist 5f,
`.agent/scripts/local_review.sh`). Used to check that a candidate model
still finds real defects after being right-sized for the host's VRAM
(#605) — a model that runs but cannot review is not an improvement over a
model that OOMs.

The defect set is fixed and enumerated here so scoring is objective and
repeatable, not re-adjudicated each run. One defect per category that
`local_review.sh`'s own prompt asks the model to look for.

The fixture is deliberately a single self-contained added file: it is
scored per-model, and chunking must not change the score between runs.

## The defects

| # | Category | Method / line (in the added file) | What is wrong |
|---|---|---|---|
| 1 | Off-by-one | `samples_for_window`, `return samples[start:end + 1]` | `window_bounds` documents a half-open range where sample `end` belongs to the *next* window, but the slice adds `+ 1`, so every window duplicates its first sample into the previous one. The `+ 1` and the docstring directly contradict. |
| 2 | Misleading comment | `samples_for_window`, `# Include the boundary sample so no data is lost between windows.` | The comment claims the `+ 1` prevents data loss. It causes duplication, not loss prevention — nothing is lost without it, because the range is half-open by construction. The comment is what makes defect 1 look intentional. |
| 3 | Race condition | `_pings` / `_dropped` shared between `on_ping` (subscription thread) and `report` (timer thread) | `_pings` is appended from the subscription callback, replaced wholesale in `flush`, and read in `report`; `_dropped` is read-and-reset in `report` with no lock. `threading` is imported but never used — no mutex is taken anywhere. A flush concurrent with a report can read a list that is being swapped. |
| 4 | Resource leak | `flush`, `f = open(path, 'wb')` | The file handle is opened without a context manager and never closed, on a path called once per completed window. A long survey leaks one descriptor per window until the process hits its `ulimit -n`. |
| 5 | Silently swallowed exception | `load_calibration`, `except Exception: pass` | A missing, unreadable or corrupt calibration file returns empty bytes indistinguishable from a legitimately empty file. The caller cannot tell calibration failed, and the boat runs uncalibrated with no diagnostic. |

## Scoring method

1. Run `local_review.sh` against the fixture with the candidate model.
2. **Recall**: count how many of the 5 planted defects the model flags,
   matched by method name or line content. A finding must identify the
   actual defect, not merely mention the method.
3. **False positives**: count findings that do not correspond to a planted
   defect. Informational only, not a gate — a genuine defect-finder
   legitimately surfaces things that were not planted (this file has other
   imperfections by construction).
4. A model that OOMs, or that returns `done_reason: length` without an
   answer, is disqualified regardless of recall: "a smaller model that
   runs beats a bigger one that gets OOM-killed" is the standing principle
   from #605, and its converse — a model that never answers — fails just
   as hard.

Record the result in `.agent/knowledge/local_model_sizing.md`, not only in
a progress.md entry: the sizing method and the score are reusable for the
next VRAM-constrained host, and progress entries evaporate once the issue
closes.
