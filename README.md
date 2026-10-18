# Publication
Fast and optimal tuning of fractional order PID controller for AVR system based on memorizable-smoothed functional algorithm
https://doi.org/10.1016/j.jestch.2022.101264

# AVR FOPID Tuning Toolbox

Tuning of fractional-order PID (FOPID) controllers for the classical
Automatic Voltage Regulator (AVR) plant with the SFA family of stochastic
search algorithms, plus utilities for comparing published controller gains
and for a Simulink-based robustness study.

The four search variants (SFA, MSFA, NLSFA, CMSFA) share a single
optimizer, `avr_optimize`; the plant/cost model and the comparison
utilities are provided by `PIDAVR`, `runGain` and `SigmoidTest`.

## Requirements

- MATLAB R2019b or newer (`arguments` block, string arrays)
- Control System Toolbox (`tf`, `step`, `stepinfo`, `bode`, `margin`)
- FOTF toolbox (A. Tepljakov) providing `fotf` and `oustapp` for the
  fractional-order terms
- Simulink, only for `SigmoidTest.m` (model `AVRsim.slx`, not included)

## Contents

| File | Purpose |
|------|---------|
| `avr_optimize.m` | Unified optimizer: `avr_optimize(method, ...)`, method = SFA / MSFA / NLSFA / CMSFA |
| `PIDAVR.m` | Objective function: FOPID + AVR closed-loop metrics, cost J, margins, poles/zeros |
| `runGain.m` | Comparison of published controller gains: step responses, Bode diagrams, margins, poles/zeros |
| `SigmoidTest.m` | Simulink-based robustness study of published gains against plant-parameter drift |
| `README.md` | This documentation |

## Quick start

```matlab
results = avr_optimize("CMSFA");                   % single run, default options
results = avr_optimize("NLSFA", 'runs', 25, 'doPlot', false);  % 25-run statistics
results = avr_optimize("MSFA", 'Nc', 99, 'lb', 0, 'ub', 5);    % custom settings
runGain                                            % published-gain comparison
```

With default options `avr_optimize` prints one line per step, draws the
convergence curve in `figure(1)`, adds the best controller's step response
and Bode diagram to `figure(11)`/`figure(12)`, and saves the `results`
struct to `<METHOD>_AVR.mat` (e.g. `CMSFA_AVR.mat`).

## Optimizer reference

`results = avr_optimize(method, Name, Value, ...)`

### Options

| Option | Default | Meaning |
|--------|---------|---------|
| `runs` | `1` | number of independent runs; use e.g. 25 for statistics |
| `Nc` | `49` | update steps per run; the cost history has `Nc + 1` points |
| `lb`, `ub` | `0.001`, `2` | parameter bounds applied to every candidate |
| `p0` | `[1 0.01 0.01 1 1]` | initial `[kp ki kd lambda mu]` |
| `a0` | `0.5` | step-size scale, `ak = a0/(k+1)` |
| `c0` | `0.05` | perturbation scale, `ck = c0/(k+1)` |
| `delta` | `0.01` | step-norm limit (NLSFA only) |
| `rngSeed` | `0` | `0` seeds `rng(0)` ("rng default"); `[]` keeps the current stream |
| `doPlot` | `true` | convergence figure + step/Bode of the best controller |
| `verbose` | `true` | per-step console output |
| `saveName` | `""` | `.mat` file name; `""` selects `<METHOD>_AVR.mat` |

### Results struct

| Field | Size | Meaning |
|-------|------|---------|
| `method`, `options` | – | method name and the option values used |
| `P` | 5×(Nc+1)×runs | accepted parameter history `[kp ki kd lambda mu]` |
| `Jf` | (Nc+1)×runs | cost of the accepted point at every step |
| `OS`, `Tr`, `Ts`, `Mp` | (Nc+1)×runs | step-response metrics of the accepted points |
| `Jplus`, `Jminus` | Nc×runs | costs of the two perturbation probes |
| `Pgood`, `Jgood`, `treq` | 5×runs, 1×runs | final parameters, final cost, wall-clock time per run |
| `ibest`, `bestP`, `bestJ` | – | index, parameters and cost of the best run |
| `bestOS`, `bestTr`, `bestTs`, `bestMp`, `bestRespond` | – | metrics of the best controller |
| `AvgJ`, `BestJgood`, `WorstJgood`, `stdJ` | – | statistics of `Jgood` over the runs |

## Search algorithm

All four variants share the same SPSA-style stochastic search. At every
step `k` each run draws one random direction `eta = sign(randn(5,1))`,
evaluates the two perturbed candidates `P ± ck*eta` (their costs provide
the gradient estimate `ghat = (Jplus - Jminus) * eta / (2*ck)`), and
computes the proposal `Pnew = clip(P - ak*ghat)`. The variants differ only
in the acceptance rule:

| Method | Step limit | Acceptance rule |
|--------|-----------|-----------------|
| SFA | no | always accept `Pnew` |
| MSFA | no | accept `Pnew` only if it improves the best cost so far |
| NLSFA | `\|step\| <= delta` | always accept `Pnew` |
| CMSFA | no | move to the best of the candidates `{Pminus, Pplus, Pnew}` seen so far |

`Jbar`/`Pbar` track the current walk position for SFA/NLSFA and the best
position seen so far for MSFA/CMSFA, so the convergence plot shows a raw
(non-monotone) walk for the former and a monotone curve for the latter.
The default settings (`Nc = 49`, `a0 = 0.5`, `c0 = 0.05`, bounds
`[0.001, 2]`, `p0 = [1 0.01 0.01 1 1]`) reproduce the configuration of the
reference experiment.

## Cost function (PIDAVR)

For a candidate `P = [kp ki kd lambda mu]`, `PIDAVR` builds the FOPID
controller (Oustaloup approximation, 5th order, band [1e-5, 1e5] rad/s)
and the classical AVR plant

```
Amplifier 10/(1+0.1*s) · Exciter 1/(1+0.4*s) · Generator 1/(1+1.0*s) · Sensor 1/(1+0.01*s)
```

and evaluates the closed loop over a 5 s simulation:

```
J = (1 - exp(-B)) * ( (Mp - 1) + |y(t_end) - 1| ) + (Ts - Tr) * exp(-B),   B = 1
```

where `Mp` is the peak of the closed-loop step response, `Ts` the 5%
settling time, `Tr` the 10–90% rise time and `y(t_end)` the response value
at t = 5 s. Unstable or invalid candidates receive the penalty `J = 1e10`
with all metrics set to `1e10`.

Additional outputs: `bodeAnalysis = [pm wg td]` (phase margin in degrees,
gain-crossover frequency in rad/s, delay margin in s) and `PZ` (struct
with the closed-loop poles and zeros). With `plotFlag = 'plotStepBode'`
the step response is drawn in `figure(11)` and the Bode diagram in
`figure(12)`; plots accumulate across calls via `hold on` (line color and
style are optional arguments).

## Output files and figures

- `figure(1)` — convergence: all runs in gray, best run in bold blue.
- `figure(11)` / `figure(12)` — step responses / Bode diagrams of the best
  controller and of the controllers compared by `runGain`.
- `<METHOD>_AVR.mat` — the `results` struct described above; each method
  writes its own file.

## Reproducibility

- Results are deterministic for a given seed: `rngSeed = 0` (default)
  seeds `rng(0)` before the search; `rngSeed = []` leaves the current
  stream untouched.
- The RNG stream is consumed at a fixed rate: exactly one `randn(5,1)`
  draw per optimization step, in a fixed evaluation order.
- Per-method `.mat` files keep the variants' results separate.

## Notes

- `runGain.m` contains the published gain table (SCA / SA / CAS / PSO /
  NSGA-II FOPIDs and the four SFA-family FOPIDs); the classical PID
  baseline rows are included by uncommenting them.
- `SigmoidTest.m` requires `AVRsim.slx` on the MATLAB path. The model must
  read `Kp, Ki, Kd, lambda, mu` from the base workspace and export
  `Output, Input, Disturbance, ISE, IAE, ITAE, ITSE` to the base workspace
  (To Workspace blocks, array format, "Single simulation output" OFF).
  Plant time constants are scaled through the `*_ROC` arrays (±25% / ±50%)
  to study parameter drift.

## Reference

The published gain set in `runGain.m` and the AVR plant parameters follow
the comparison study referenced in the `PIDAVR.m` / `runGain.m` headers
(journals.sagepub.com, doi 10.1177/0142331218811453).
