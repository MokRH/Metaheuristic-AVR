# AVR FOPID Tuning Toolbox

Tuning of **fractional-order PID (FOPID)** controllers for the classical
**Automatic Voltage Regulator (AVR)** plant with the SFA family of stochastic
search algorithms, plus utilities for comparing published controller gains.

This is a reorganized version of the original scripts
(`AVR_SFA.m`, `AVR_MSFA.m`, `AVR_NLSFA.m`, `AVR_CMSFA.m`, `PIDAVR.m`,
`runGain.m`, `SigmoidTest.m`). The four optimizer scripts were ~99% identical
(copies that differed only in a few commented lines) and are now a single
function, `avr_optimize`.

## Files

| File | Purpose | Replaces |
|------|---------|----------|
| `avr_optimize.m` | Unified optimizer: `avr_optimize("SFA" / "MSFA" / "NLSFA" / "CMSFA")` | `AVR_SFA.m`, `AVR_MSFA.m`, `AVR_NLSFA.m`, `AVR_CMSFA.m` |
| `PIDAVR.m` | Objective function: FOPID + AVR closed-loop metrics, cost J, margins, pole/zero | same (cleaned) |
| `runGain.m` | Compare published gains: step, Bode, margins, pole/zero | same (cleaned) |
| `SigmoidTest.m` | Simulink-based robustness test (needs `AVRsim.slx`) | same (cleaned + guarded) |
| `README.md` | This file | – |

## Requirements

- MATLAB **R2019b** or newer (`arguments` block, string arrays)
- Control System Toolbox (`tf`, `stepinfo`, `step`, `bode`, `margin`)
- **FOTF toolbox** (A. Tepljakov) providing `fotf` and `oustapp` for the
  fractional-order part
- Simulink only for `SigmoidTest.m` (model `AVRsim.slx`, not included here)

## Quick start

```matlab
% One optimization, defaults reproduce the original scripts:
results = avr_optimize("CMSFA");

% Statistics over 25 runs, no figures (batch mode):
results = avr_optimize("NLSFA", 'runs', 25, 'doPlot', false);

% Longer run with custom bounds and a different starting point:
results = avr_optimize("MSFA", 'Nc', 99, 'lb', 0, 'ub', 5);

% Compare published gains (fills figure(11) step + figure(12) Bode):
runGain
```

The optimizer prints one line per step (`run / k / Jbar`), draws the
convergence curve in figure(1), adds the best controller's step response and
Bode diagram to figure(11)/figure(12), and saves everything to
`<METHOD>_AVR.mat` (e.g. `CMSFA_AVR.mat`, variable `results`).

## Options (name-value pairs)

| Option   | Default              | Meaning |
|----------|----------------------|---------|
| `runs`   | `1`                  | number of independent runs (use e.g. 25 for statistics) |
| `Nc`     | `49`                 | update steps per run; the cost history has `Nc+1` = 50 points (original code: `Nc = 50-1`) |
| `lb,ub`  | `0.001`, `2`         | parameter bounds |
| `p0`     | `[1 0.01 0.01 1 1]`  | initial `[kp ki kd lambda mu]` |
| `a0`     | `0.5`                | step-size sequence `ak = a0/(k+1)` |
| `c0`     | `0.05`               | perturbation size `ck = c0/(k+1)` |
| `delta`  | `0.01`               | step-norm limit (NLSFA only) |
| `rngSeed`| `0`                  | `0` reproduces `rng default`; `[]` keeps the current stream |
| `doPlot` | `true`               | convergence figure + step/Bode of the best controller |
| `verbose`| `true`               | per-step console output |
| `saveName`| auto                | `.mat` file name; auto = `<METHOD>_AVR.mat` |

## The four algorithm variants

All variants share the same SPSA-style search. Each step k draws one random
direction `eta = sign(randn(5,1))`, evaluates the two perturbed candidates
`P±ck*eta` (their costs give the gradient estimate
`ghat = (J+ - J-)*eta/(2*ck)`), and computes the new point
`Pnew = clip(P - ak*ghat)`. The variants differ **only** in the acceptance rule:

| Method | Step limit          | Acceptance rule |
|--------|---------------------|-----------------|
| SFA    | no                  | always accept `Pnew` |
| MSFA   | no                  | accept `Pnew` only if J improves on the best-so-far |
| NLSFA  | `\|step\| <= delta` | always accept `Pnew` |
| CMSFA  | no                  | move to the best of the candidates `{Pminus, Pplus, Pnew}` seen so far |

## Cost function

```
J = (1 - exp(-B)) * ( (Mp - 1) + |y(t_end) - 1| ) + (Ts - Tr) * exp(-B),   B = 1
```

where `Mp` is the peak of the closed-loop step response, `Ts` the 5% settling
time, `Tr` the 10–90% rise time and `y(t_end)` the response value at t = 5 s.
Unstable or invalid candidates receive the penalty `J = 1e10`.

## Reproducibility

- With default options, `avr_optimize("CMSFA")` reproduces the trajectory of
  the original `AVR_CMSFA.m` and `avr_optimize("MSFA")` that of `AVR_MSFA.m`
  (same `rng default` stream, same evaluation order, same math).
- The uploaded `AVR_SFA.m` / `AVR_NLSFA.m` had their acceptance logic fully
  commented out, so as uploaded they never improved the cost (the walk stayed
  at the initial point). The refactored versions implement the intended
  algorithms: SFA and NLSFA accept each new point.
- Results are saved per method (`SFA_AVR.mat`, `MSFA_AVR.mat`, ...). The
  original scripts all saved to the same `NLSFA_AVR.mat`, overwriting each
  other.

## What was fixed or changed (nothing that alters the search math)

1. **Crash fix:** the `catch` branch of `PIDAVR` referenced undefined
   `Ess/Ts/Tr` — it now returns the penalty `J = 1e10`.
2. **Crash fix:** `plot(Jf(:,9))` indexed column 9 although only 1 run existed;
   the convergence figure now plots the best run.
3. **Bug fix:** all four optimizer scripts saved to `NLSFA_AVR.mat`; results
   are now saved per method.
4. **Speed:** per-evaluation plotting during the optimization is suppressed
   (it did not consume the RNG stream, so the trajectory is unchanged);
   148 evaluations now run ~10x faster. Plots are still available via
   `doPlot` and `PIDAVR(...,'plotStepBode',...)`.
5. **Real analysis data:** `PIDAVR` now returns `bodeAnalysis = [pm wg td]`
   (phase margin, gain-crossover frequency, delay margin) and a `PZ` struct
   with the closed-loop poles/zeros, instead of stub zeros.
6. **Style/robustness:** the variable `title` no longer shadows the built-in
   function; `display(...)` replaced by `fprintf`; vectorized bound clipping
   (identical result); `SigmoidTest` guards against a missing `AVRsim` model
   and uses `Output(end)` instead of the hard-coded `Output(100000)`.
7. **SFA/NLSFA semantics:** acceptance logic re-enabled as intended (see
   Reproducibility).

## Notes

- `figure(11)` and figure(12)` keep their original roles: step responses and
  Bode diagrams accumulate there via `hold on` across calls (e.g. the 9
  controllers of `runGain`).
- `runGain.m` keeps the published gain table; uncomment the classical PID
  baseline rows to include them in the comparison.
- `SigmoidTest.m` needs `AVRsim.slx` on the path; the model must export
  `Output, Input, Disturbance, ISE, IAE, ITAE, ITSE` to the base workspace
  (To Workspace blocks, array format, "Single simulation output" OFF).
