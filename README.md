# Metaheuristic-AVR

MATLAB toolbox for tuning a **fractional-order PID (FOPID)** controller on the classical **automatic voltage regulator (AVR)** plant with four variants of the **smoothed functional algorithm (SFA)**: SFA, MSFA, NLSFA and CMSFA.

It accompanies:

- *Fast and optimal tuning of fractional order PID controller for AVR system based on memorizable-smoothed functional algorithm.* Engineering Science and Technology, an International Journal, 2022, 35, 101264. <https://doi.org/10.1016/j.jestch.2022.101264>
- PhD thesis: *Data-Driven Tool Using Enhanced Smoothed Functional Algorithm for Nonlinear Modelling and Advanced PID Control Problems*, Universiti Malaysia Pahang Al-Sultan Abdullah (UMPSA), 2025 (source of the NLSFA and CMSFA variants).

## Relation to the published results

TODO(author): replace this paragraph before publishing. State whether the tables in the paper were regenerated with this code and report any numerical differences, or name the scripts that produced them. Until this is verified, do not describe the repository as reproducing the paper.

## Quick start

Requirements:

- MATLAB R2019b or newer
- Control System Toolbox
- [FOTF toolbox](https://www.mathworks.com/matlabcentral/fileexchange/60874-fotf-toolbox) (A. Tepljakov), which provides `fotf` and `oustapp`
- Simulink, only for `SigmoidTest.m`

Run one optimisation from the repository folder:

```matlab
results = avr_optimize("CMSFA");                                   % one run, default settings
results = avr_optimize("NLSFA", 'runs', 25, 'doPlot', false);      % statistics over 25 runs
runGain                                                            % compare published gains
```

Check the installation (runs all four variants for a few steps):

```matlab
runtests('tests/smoke_test.m')
```

Each run prints one line per step (`run / k / Jbar`), draws the convergence curve in figure 1, adds the best controller's step response and Bode diagram to figures 11 and 12, and saves `results` to `<METHOD>_AVR.mat`.

## Options

`avr_optimize(method, Name, Value, ...)` where `method` is `"SFA"`, `"MSFA"`, `"NLSFA"` or `"CMSFA"`.

| Option | Default | Meaning |
|---|---|---|
| `runs` | `1` | independent runs (use 25 for statistics) |
| `Nc` | `49` | update steps per run; the cost history has `Nc+1` points |
| `lb`, `ub` | `0.001`, `2` | parameter bounds |
| `p0` | `[1 0.01 0.01 1 1]` | initial `[kp ki kd lambda mu]` |
| `a0` | `0.5` | step-size sequence `ak = a0/(k+1)` |
| `c0` | `0.05` | perturbation size `ck = c0/(k+1)` |
| `delta` | `0.01` | step-norm limit (NLSFA only) |
| `rngSeed` | `0` | `0` uses `rng default`; `[]` keeps the current stream |
| `doPlot` | `true` | convergence figure and step/Bode of the best controller |
| `verbose` | `true` | per-step console output |
| `saveName` | auto | `.mat` file name; auto = `<METHOD>_AVR.mat` |

## Algorithm variants

All variants share one search engine. At step k, a random direction `eta = sign(randn(5,1))` is drawn, the two perturbed candidates `P ± ck*eta` are evaluated, the gradient estimate is `ghat = (J+ - J-)*eta/(2*ck)`, and the proposal is `Pnew = clip(P - ak*ghat)`. The variants differ only in the acceptance rule.

<!-- TODO(author): add one sentence on how this estimate relates to the smoothed functional gradient estimate used in the paper (readers familiar with SPSA will ask). -->

| Method | Step limit | Acceptance rule |
|---|---|---|
| SFA | none | always accept `Pnew` |
| MSFA | none | accept `Pnew` only if J improves on the best so far |
| NLSFA | `abs(step) <= delta` | always accept `Pnew` |
| CMSFA | none | move to the best of `{Pminus, Pplus, Pnew}` seen so far |

## Cost function

```
J = (1 - exp(-B)) * ( (Mp - 1) + |y(t_end) - 1| ) + (Ts - Tr) * exp(-B),   B = 1
```

`Mp` is the peak of the closed-loop step response, `Ts` the 5% settling time, `Tr` the 10-90% rise time and `y(t_end)` the response at t = 5 s. Unstable or invalid candidates receive the penalty `J = 1e10`.

## Repository layout

| File | Purpose |
|---|---|
| `avr_optimize.m` | Unified optimiser for the four variants |
| `PIDAVR.m` | Objective function: FOPID + AVR closed-loop metrics, cost J, margins, poles and zeros |
| `runGain.m` | Compare published gains: step, Bode, margins, poles and zeros |
| `SigmoidTest.m` | Simulink robustness test (needs `AVRsim.slx`, which is not included) |
| `tests/smoke_test.m` | Runs each variant for a few steps |
| `CHANGELOG.md` | History of changes to the original scripts |

## Notes

- Step responses and Bode diagrams accumulate in figures 11 and 12 across calls (`hold on`), for example the nine controllers compared by `runGain`.
- `runGain.m` keeps the published gain table. Uncomment the classical PID baseline rows to include them.
- `SigmoidTest.m` needs `AVRsim.slx` on the path. The model must export `Output, Input, Disturbance, ISE, IAE, ITAE, ITSE` to the base workspace (To Workspace blocks, array format, "Single simulation output" off).

## Citation

If you use this code, please cite the paper above. See also `CITATION.cff`.

## Licence

MIT, see `LICENSE`.

## Author

Mok Ren Hao, [ORCID 0000-0002-6937-0603](https://orcid.org/0000-0002-6937-0603)
