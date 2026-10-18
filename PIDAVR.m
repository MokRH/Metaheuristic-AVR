function [OS,Tr,Ts,Mp,J,bodeAnalysis,PZ] = PIDAVR(P, plotFlag, color, style)
%PIDAVR  Closed-loop performance and cost of a FOPID controller for the AVR plant.
%
%   [OS,Tr,Ts,Mp,J] = PIDAVR(P)
%   [OS,Tr,Ts,Mp,J,bodeAnalysis,PZ] = PIDAVR(P, plotFlag, color, style)
%
%   INPUTS
%     P        [kp ki kd lambda mu] : FOPID gains and fractional orders
%     plotFlag (optional) 'none' (default) | 'plotStepBode'
%              'plotStepBode' adds the closed-loop step response to figure(11)
%              and the Bode magnitude/phase to figure(12) (they accumulate via
%              hold on, exactly like the original code).
%     color    line color when plotting: char ('b', 'm', ...) or RGB triplet
%     style    line style when plotting: char ('-', '--', ...)
%
%   OUTPUTS
%     OS           overshoot [%]               (stepinfo Overshoot)
%     Tr           rise time [s], 10-90%       (stepinfo RiseTime)
%     Ts           settling time [s], 5% band  (stepinfo SettlingTime)
%     Mp           peak value of the response  (stepinfo Peak)
%     J            cost, with B = 1:
%                      J = (1-exp(-B))*((Mp-1) + |y(t_end)-1|) + (Ts-Tr)*exp(-B)
%     bodeAnalysis [pm wg td]: phase margin [deg], gain-crossover (phase-
%                  crossover) frequency wg [rad/s] and delay margin
%                  td = pm[rad]/wg [s]. [NaN NaN NaN] if unavailable.
%     PZ           struct with fields .zeros and .poles of the closed loop
%
%   Unstable / invalid parameter sets return the penalty J = 1e10 with all
%   metrics set to 1e10. (BUGFIX: the original catch branch referenced the
%   undefined variables Ess/Ts/Tr and crashed instead of returning a penalty.)
%
%   Plant (classic AVR model):
%       Amplifier 10/(1+0.1*s), Exciter 1/(1+0.4*s),
%       Generator 1/(1+1.0*s),  Sensor 1/(1+0.01*s)
%   The fractional-order PID is approximated with the Oustaloup recursion
%   (5th order, band [1e-5, 1e5] rad/s) from the FOTF toolbox.
%
%   Reference: https://journals.sagepub.com/doi/abs/10.1177/0142331218811453
%
%   Requires: Control System Toolbox, FOTF toolbox (fotf, oustapp).
%
%   See also AVR_OPTIMIZE, RUNGAIN.

% ---- parse optional plotting arguments --------------------------------
if nargin < 2 || isempty(plotFlag)
    plotFlag = 'none';
end
doPlot = strcmpi(char(plotFlag), 'plotStepBode');
if doPlot
    if nargin < 3 || isempty(color), color = 'b'; end
    if nargin < 4 || isempty(style), style = '-'; end
    if isstring(color), color = char(color); end
    if isstring(style), style = char(style); end
end

% ---- controller and plant models ---------------------------------------
kp     = P(1);
ki     = P(2);
kd     = P(3);
lambda = P(4);
mu     = P(5);

s  = tf('s');
Amplifier = 10/(1+0.1*s);    % kA/(1+Ta*s)
Exciter   = 1/(1+0.4*s);     % kE/(1+Te*s)
Generator = 1/(1+1.0*s);     % kG/(1+Tg*s)
Sensor    = 1/(1+0.01*s);    % kS/(1+Ts*s)

S = fotf('s');
FOPID_long = kp + (ki/S^lambda) + (kd*S^mu);
% convert fotf to a rational tf using the Oustaloup method
FOPID = minreal(oustapp(FOPID_long, 1e-5, 1e5, 5, 'oust'));

G = FOPID*Amplifier*Exciter*Generator;

% ---- evaluate the closed loop, with a penalty on failure ----------------
B      = 1;
PENALTY = 1e10;

try
    % equivalent to feedback(G, Sensor)
    CL = G/(1 + G*Sensor);

    Q = stepinfo(CL, 'SettlingTimeThreshold', 0.05, 'RiseTimeThreshold', [0.1 0.9]);
    [Ess, t] = step(CL, 5);

    OS = Q.Overshoot;
    Tr = Q.RiseTime;
    Ts = Q.SettlingTime;
    Mp = Q.Peak;
    J  = (1-exp(-B))*((Mp-1) + abs(Ess(end)-1)) + (Ts-Tr)*exp(-B);

    % margin and pole/zero analysis (the original returned stub zeros here)
    try
        [~, pm, ~, wcp] = margin(CL);
        if isnan(wcp) || wcp <= 0
            td = NaN;
        else
            td = (pm*pi/180)/wcp;      % delay margin [s]
        end
        bodeAnalysis = [pm, wcp, td];

        PZ = struct();
        PZ.zeros = zero(CL);
        PZ.poles = pole(CL);
    catch
        bodeAnalysis = [NaN NaN NaN];
        PZ = struct('zeros', [], 'poles', []);
    end

    if doPlot
        % step response -> figure(11)
        figure(11)
        hold on
        plot(t, Ess, 'Color', color, 'LineStyle', style, 'LineWidth', 1.5);

        % Bode magnitude & phase -> figure(12)
        figure(12)
        hold on
        hBefore = findobj(gcf, 'Type', 'line');
        ww = linspace(1, 100*pi, 1e5);      % same frequency band as the original
        bode(CL, ww)
        hNew = setdiff(findobj(gcf, 'Type', 'line'), hBefore);
        set(hNew, 'Color', color, 'LineStyle', style, 'LineWidth', 1.5);
    end
catch
    % BUGFIX: return a large penalty so the optimizer can discard the
    % candidate (the original crashed on undefined Ess/Ts/Tr here).
    OS = PENALTY;
    Tr = PENALTY;
    Ts = PENALTY;
    Mp = PENALTY;
    J  = PENALTY;
    bodeAnalysis = [NaN NaN NaN];
    PZ = struct('zeros', [], 'poles', []);
end
end
