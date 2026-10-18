# Changelog

## 1.0.0

Reorganisation of the original experiment scripts (`AVR_SFA.m`, `AVR_MSFA.m`, `AVR_NLSFA.m`, `AVR_CMSFA.m`, `PIDAVR.m`, `runGain.m`, `SigmoidTest.m`).

### Changed

- The four optimiser scripts were about 99% identical and are now one function, `avr_optimize`.
- **SFA and NLSFA acceptance rules.** In the earlier `AVR_SFA.m` and `AVR_NLSFA.m` the acceptance logic was commented out, so the walk never moved from the initial point. `avr_optimize` applies the intended rules: SFA and NLSFA accept each new point. The MSFA and CMSFA trajectories are unchanged (same `rng default` stream and evaluation order).
- Results are saved per method (`SFA_AVR.mat`, `MSFA_AVR.mat`, and so on). The earlier scripts all saved to `NLSFA_AVR.mat`.
- Plotting during the optimisation is suppressed, which makes a 148-evaluation run about ten times faster. Plots remain available through `doPlot` and `PIDAVR(..., 'plotStepBode', ...)`.
- `PIDAVR` returns real analysis data (`bodeAnalysis = [pm wg td]` and a `PZ` struct of poles and zeros) in place of stub zeros.
- The variable `title` no longer shadows the built-in function, `display` was replaced by `fprintf`, bound clipping is vectorised, and `SigmoidTest` guards against a missing `AVRsim` model and uses `Output(end)`.

### Fixed

- The `catch` branch of `PIDAVR` referenced undefined variables. It now returns the penalty `J = 1e10`.
- `plot(Jf(:,9))` indexed column 9 with only one run. The convergence figure now plots the best run.

### Added

- `tests/smoke_test.m`, `LICENSE`, `CITATION.cff`.
