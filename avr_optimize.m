function results = avr_optimize(method, varargin)
%AVR_OPTIMIZE  Tune a FOPID controller for the AVR plant with the SFA family.
%
%   results = avr_optimize(method)
%   results = avr_optimize(method, Name, Value, ...)
%
%   Optimizes the five FOPID parameters P = [kp ki kd lambda mu] of the
%   classical AVR plant (see PIDAVR) with one of four variants of the SFA
%   (Stochastic Filtering Algorithm) family. All variants share the same
%   SPSA-style search; they differ only in the acceptance rule:
%
%     "SFA"    always accept the new point
%     "MSFA"   accept the new point only if it improves the best cost
%     "NLSFA"  SFA + update step limited to |step| <= delta
%     "CMSFA"  move to the best of the three evaluated candidates {P-,P+,Pnew}
%
%   Name-Value options (defaults reproduce the original scripts):
%     runs     = 1                 number of independent runs (25 for statistics)
%     Nc       = 49                update steps per run; cost history has Nc+1
%                                  = 50 points (original: Nc = 50-1)
%     lb       = 0.001             lower parameter bound
%     ub       = 2                 upper parameter bound
%     p0       = [1 0.01 0.01 1 1] initial [kp ki kd lambda mu]
%     a0       = 0.5               step size sequence ak = a0/(k+1)
%     c0       = 0.05              perturbation size   ck = c0/(k+1)
%     delta    = 0.01              step-norm limit (NLSFA only)
%     rngSeed  = 0                 0 reproduces "rng default"; [] keeps the
%                                  current random stream
%     doPlot   = true              convergence figure(1) + step/Bode of best
%     verbose  = true              per-step console output
%     saveName = ""                .mat file name; "" = auto: <METHOD>_AVR.mat
%
%   Examples:
%     results = avr_optimize("CMSFA");                 % like the original AVR_CMSFA.m
%     results = avr_optimize("NLSFA", 'runs', 25);     % 25-run statistics
%     results = avr_optimize("MSFA", 'doPlot', false); % batch mode, no figures
%
%   The returned struct contains the full cost history (results.Jf), the
%   parameter history (results.P), the perturbation costs (results.Jplus /
%   results.Jminus), per-run bests (results.Pgood / results.Jgood) and a
%   summary of the best run (results.bestP, results.bestJ, ...).
%
%   See also PIDAVR, RUNGAIN.

% ---- parse options ---------------------------------------------------
arguments
    method                                                       % 'SFA' | 'MSFA' | 'NLSFA' | 'CMSFA'
    runs     (1,1) double {mustBePositive, mustBeInteger} = 1
    Nc       (1,1) double {mustBePositive, mustBeInteger} = 49
    lb       (1,1) double = 0.001
    ub       (1,1) double = 2
    p0       (1,5) double = [1 0.01 0.01 1 1]
    a0       (1,1) double {mustBeNonnegative} = 0.5
    c0       (1,1) double {mustBePositive}    = 0.05
    delta    (1,1) double {mustBePositive}    = 0.01
    rngSeed            double = 0
    doPlot   (1,1) logical = true
    verbose  (1,1) logical = true
    saveName            = ""
end

methodU = upper(char(method));
switch methodU
    case 'SFA'
        acceptMode = 'always';  limitStep = false;   % accept every new point
    case 'MSFA'
        acceptMode = 'best';    limitStep = false;   % monotone (elitist) walk
    case 'NLSFA'
        acceptMode = 'always';  limitStep = true;    % SFA + limited step size
    case 'CMSFA'
        acceptMode = 'best3';   limitStep = false;   % best of {P-, P+, Pnew}
    otherwise
        error('avr_optimize:badMethod', ...
            'Unknown method "%s". Use SFA, MSFA, NLSFA or CMSFA.', methodU);
end

if ub <= lb
    error('avr_optimize:badBounds', 'ub must be greater than lb.');
end

if ~isempty(rngSeed)
    rng(rngSeed);    % seed 0 reproduces the "rng default" of the original scripts
end

p     = numel(p0);      % 5 parameters: kp ki kd lambda mu
nHist = Nc + 1;         % cost-history length (original scripts stored 50 points)

% ---- preallocate history arrays --------------------------------------
P      = zeros(p,    nHist, runs);
Jf     = zeros(nHist, runs);
OS     = zeros(nHist, runs);   Tr = zeros(nHist, runs);
Ts     = zeros(nHist, runs);   Mp = zeros(nHist, runs);
Jplus  = zeros(Nc, runs);
Jminus = zeros(Nc, runs);
Pgood  = zeros(p, runs);
Jgood  = zeros(1, runs);
treq   = zeros(1, runs);

% ---- main loop over independent runs ----------------------------------
for iter = 1:runs
    tic

    % initialize: evaluate the starting point
    P(:, 1, iter) = p0;
    [OS(1,iter), Tr(1,iter), Ts(1,iter), Mp(1,iter), Jf(1,iter)] = PIDAVR(p0, 'none');

    Jbar = Jf(1, iter);     % best cost seen so far
    Pbar = p0(:);           % corresponding parameter vector

    if verbose
        fprintf('run %d/%d | k = %3d/%3d | Jbar = %.6g\n', iter, runs, 1, nHist, Jbar);
    end

    % optimization loop
    for k = 1:Nc
        ak  = a0/(k+1);          % step-size sequence  (original: 0.5/(k+1)^1)
        ck  = c0/(k+1);          % perturbation size   (original: 0.05/(k+1)^1)
        eta = sign(randn(p,1));  % random direction: exactly one draw per step,
                                 % in the same order as the original scripts

        % +/- perturbations (their costs also estimate the gradient)
        Pplus = min(max(P(:,k,iter) + ck*eta, lb), ub);
        [~, ~, ~, ~, Jplus(k,iter)] = PIDAVR(Pplus, 'none');

        Pminus = min(max(P(:,k,iter) - ck*eta, lb), ub);
        [~, ~, ~, ~, Jminus(k,iter)] = PIDAVR(Pminus, 'none');

        % gradient estimate and parameter update
        ghat = (Jplus(k,iter) - Jminus(k,iter)) .* eta ./ (2*ck);
        stepVec = ak .* ghat;
        if limitStep
            stepVec = min(max(stepVec, -delta), delta);   % NLSFA: norm-limited step
        end
        Pnew = min(max(P(:,k,iter) - stepVec, lb), ub);
        [OS(k+1,iter), Tr(k+1,iter), Ts(k+1,iter), Mp(k+1,iter), Jnew] = ...
            PIDAVR(Pnew, 'none');

        % ---- acceptance rule: this is the only difference between variants
        switch acceptMode
            case 'always'      % SFA / NLSFA: keep the new point
                Jbar = Jnew;
                Pbar = Pnew;
            case 'best'        % MSFA: keep the new point only if it improves
                if Jnew < Jbar
                    Jbar = Jnew;
                    Pbar = Pnew;
                end
            case 'best3'       % CMSFA: best of the three evaluated candidates
                if Jminus(k,iter) < Jbar
                    Jbar = Jminus(k,iter);  Pbar = Pminus;
                end
                if Jplus(k,iter)  < Jbar
                    Jbar = Jplus(k,iter);   Pbar = Pplus;
                end
                if Jnew           < Jbar
                    Jbar = Jnew;            Pbar = Pnew;
                end
        end

        % record the elite point in the histories
        Jf(k+1, iter)  = Jbar;
        P(:, k+1, iter) = Pbar;

        if verbose
            fprintf('run %d/%d | k = %3d/%3d | Jbar = %.6g\n', iter, runs, k+1, nHist, Jbar);
        end
    end

    treq(iter)      = toc;
    Pgood(:, iter)  = Pbar;
    Jgood(iter)     = Jbar;
end

% ---- pick the best run and evaluate its controller ---------------------
[Jbest, ibest] = min(Jgood);   %#ok<NASGU>  Jbest kept for clarity
bestP = Pgood(:, ibest);

if doPlot
    % adds the step response and Bode diagram of the best controller to
    % figure(11) / figure(12)
    [bestOS, bestTr, bestTs, bestMp, bestJ] = PIDAVR(bestP, 'plotStepBode', 'b', '-');
else
    [bestOS, bestTr, bestTs, bestMp, bestJ] = PIDAVR(bestP, 'none');
end

bestRespond = ["Mp%" "Mp" "Ts" "Tr" "J" "1/J"];
bestRespond(2,:) = [bestOS bestMp bestTs bestTr bestJ 1/bestJ];

% ---- convergence figure -------------------------------------------------
if doPlot
    figure(1)
    clf
    hold on
    box on
    grid on
    Jhist = squeeze(Jf);                       % nHist x runs
    if runs > 1
        hGray = plot(Jhist, 'Color', [0.7 0.7 0.7], 'LineWidth', 0.7);
    end
    hBest = plot(Jhist(:, ibest), 'b-', 'LineWidth', 1.8);
    xlabel('optimization step k')
    ylabel('cost J (best so far)')
    if runs > 1
        title(sprintf('%s on the AVR plant: convergence, best of %d runs', methodU, runs))
        legend([hGray(1) hBest], ...
            {'other runs', sprintf('best run (\\#%d)', ibest)}, 'Location', 'northeast')
    else
        title(sprintf('%s on the AVR plant: convergence', methodU))
    end
end

% ---- summary statistics (as displayed by the original scripts) ---------
AvgJ       = mean(Jgood);
BestJgood  = min(Jgood);
WorstJgood = max(Jgood);
stdJ       = std(Jgood);

fprintf('\n%s: Avg J = %.6g | Best J = %.6g | Worst J = %.6g | Std J = %.6g\n', ...
    methodU, AvgJ, BestJgood, WorstJgood, stdJ)
if runs > 1
    fprintf('best run: #%d | best parameters [kp ki kd lambda mu]:\n', ibest)
    fprintf('    %.4f  %.4f  %.4f  %.4f  %.4f\n', bestP)
end

% ---- pack results and save ----------------------------------------------
results = struct();
results.method   = methodU;
results.options  = struct('runs',runs, 'Nc',Nc, 'lb',lb, 'ub',ub, 'p0',p0, ...
    'a0',a0, 'c0',c0, 'delta',delta, 'limitStep',limitStep, ...
    'acceptMode',acceptMode, 'rngSeed',rngSeed);
results.P        = P;
results.Jf       = Jf;
results.OS       = OS;
results.Tr       = Tr;
results.Ts       = Ts;
results.Mp       = Mp;
results.Jplus    = Jplus;
results.Jminus   = Jminus;
results.Pgood    = Pgood;
results.Jgood    = Jgood;
results.treq     = treq;
results.ibest    = ibest;
results.bestP    = bestP;
results.bestJ    = bestJ;
results.bestOS   = bestOS;
results.bestTr   = bestTr;
results.bestTs   = bestTs;
results.bestMp   = bestMp;
results.bestRespond = bestRespond;
results.AvgJ       = AvgJ;
results.BestJgood  = BestJgood;
results.WorstJgood = WorstJgood;
results.stdJ       = stdJ;

if isempty(saveName)
    saveName = string(methodU) + "_AVR";    % e.g. CMSFA_AVR.mat
end
save(char(saveName), 'results')
if verbose
    fprintf('results saved to %s.mat\n', saveName)
end
end
