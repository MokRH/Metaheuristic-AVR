%RUNGAIN  Compare published FOPID (and classical PID) gains on the AVR plant.
%
%   For every gain vector in BESTP, PIDAVR is evaluated and
%     * the closed-loop step response is added to figure(11),
%     * the Bode magnitude/phase is added to figure(12),
%     * margins are collected in BODEDATA = [pm [deg], wg [rad/s], td [s]],
%     * pole/zero sets are stored in PZDATA (cell array of structs).
%
%   The gain vectors below come from the published comparison in
%   https://journals.sagepub.com/doi/abs/10.1177/0142331218811453
%   (rows 6-9 are the SFA-family controllers tuned by AVR_OPTIMIZE).

close all
clear
clc

bestP = [...
    1.4509 0.6567 0.3076 1.1442 1.2145;...  % 1 SCA-FOPID (Mustafa)
    0.7837 0.5027 0.2307 1.0103 1.0727;...  % 2 SA-FOPID
    1.0537 0.4418 0.2510 1.0624 1.1122;...  % 3 CAS-FOPID
    1.6264 0.2956 0.3226 1.3183 1.1980;...  % 4 PSO-FOPID
    0.8399 1.3359 0.3512 0.9147 0.7107;...  % 5 NSGA II-FOPID
    1.2605 0.9855 0.3264 1.0578 1.1615;...  % 6 SFA-FOPID
    1.1662 0.3779 0.2923 0.9648 1.1765;...  % 7 NLSFA-FOPID
    1.6889 0.0142 0.3847 1.4372 1.3339;...  % 8 MSFA-FOPID
    1.7251 0.0010 0.3905 1.4370 1.3414];... % 9 CMSFA-FOPID

% Classical PID baselines (lambda = mu = 1). Uncomment to include:
%     0.9826 0.8337 0.4982 1 1;...  % SCA-PID
%     1.2464 0.5893 0.4596 1 1;...  % BBO-PID
%     1.6524 0.4083 0.3654 1 1;...  % ABC-PID
%     0.6746 0.6009 0.2618 1 1;...  % CAS-PID
%     0.7722 0.7201 0.3196 1 1;...  % GA-PID (Gaing), Ts 2%
%     0.6751 0.5980 0.2630 1 1;...  % PSO-PID (Gaing), Ts 2%

labels = ["SCA-FOPID"; "SA-FOPID"; "CAS-FOPID"; "PSO-FOPID"; ...
          "NSGA II-FOPID"; "SFA-FOPID"; "NLSFA-FOPID"; ...
          "MSFA-FOPID"; "CMSFA-FOPID"];
color  = ["m", "y", "c", "g", "r", "y", "c", "g", "r"];
style  = ["--","--","--","--","--","-", "-", "-", "-" ];

n = size(bestP, 1);

bestRespond(1,:) = ["Mp%" "Mp" "Ts" "Tr" "J" "1/J"];
bodeData(1,:)    = ["pm"  "wg" "td"];
PZData = cell(1, n);

for i = 1:n
    [bestOS, bestTr, bestTs, bestMp, bestJ, bodeAnalysis, PZ] = ...
        PIDAVR(bestP(i,:), 'plotStepBode', color(i), style(i));

    bestRespond(i+1,:) = [bestOS bestMp-1 bestTs bestTr bestJ 1/bestJ];
    bodeData(i+1,:)    = bodeAnalysis;
    PZData{i}          = PZ;
end

% unity reference line on the step-response figure
figure(11)
plot(0:10, ones(1,11), 'k--', 'LineWidth', 1.5);
try
    legend([labels, "unity"], 'Location', 'best');
catch
    % the legend is cosmetic only - ignore any handle-count mismatch
end

% comparison table (overshoot %, overshoot amplitude, settling/rise time, cost)
bestRespond

% settling-time improvement of the best (CMSFA) over the worst published
% controller, in percent (values taken from the paper's results)
Ts_improvement_pct = (179.8169 - 43.7524)/179.8169*100
