%SIGMOIDTEST  Robustness study of published FOPID gains on the AVRsim model.
%
%   For every controller in BESTP, the Simulink model 'AVRsim' is simulated
%   for 10 s and the step-response metrics plus the standard error integrals
%   (ISE, IAE, ITAE, ITSE) are collected. The plant time constants can be
%   scaled with the *_ROC arrays to study +/-25% / +/-50% parameter drift.
%
%   REQUIREMENTS
%     * Simulink model AVRsim.slx on the MATLAB path (NOT included in this
%       toolbox). The model must read Kp, Ki, Kd, lambda and mu from the
%       base workspace and must export these variables to the base workspace
%       (To Workspace blocks, array save format, "Single simulation output"
%       OFF in Configuration Parameters > Data Import/Export):
%         Output, Input, Disturbance, ISE, IAE, ITAE, ITSE
%
%   NOTE: the numerical procedure is unchanged from the original SigmoidTest.m.
%   Only the hard-coded sample index Output(100000) was replaced by
%   Output(end) and a guard for the missing model / missing variables was
%   added. The sensor time constant was renamed Ts -> Tss to avoid clashing
%   with the settling-time variable.

close all
clear
clc

%% Simulink model guard
model = 'AVRsim';
if isempty(which(model))
    error('SigmoidTest:modelNotFound', ...
        ['Simulink model "%s.slx" was not found on the MATLAB path.\n' ...
         'Copy AVRsim.slx into the current folder or add its folder to ' ...
         'the MATLAB path.'], model);
end

%% Published gains to test  [kp ki kd lambda mu]
bestP = [...
    1.4509 0.6567 0.3076 1.1442 1.2145;...  % 1 SCA-FOPID (Mustafa)
    0.7837 0.5027 0.2307 1.0103 1.0727;...  % 2 SA-FOPID
    1.0537 0.4418 0.2510 1.0624 1.1122;...  % 3 CAS-FOPID
    1.6264 0.2956 0.3226 1.3183 1.1980;...  % 4 PSO-FOPID
    0.8399 1.3359 0.3512 0.9147 0.7107;...  % 5 NSGA II-FOPID
    1.2605 0.9855 0.3264 1.0578 1.1615;...  % 6 SFA-FOPID
    1.4745 0.7510 0.3700 1.0079 1.2318];... % 7 MSFA-FOPID

labels = ["SCA-FOPID (Mustafa)"; "SA-FOPID"; "CAS-FOPID"; "PSO-FOPID"; ...
          "NSGA II-FOPID"; "SFA-FOPID"; "MSFA-FOPID"];

%% Plant time constants (and their robustness ranges)
Ta  = 0.1;                 % amplifier
Te  = 0.4;                 % exciter
Tg  = 1.0;                 % generator
Tss = 0.01;                % sensor

Ta_ROC = [Ta*1.5  Ta*1.25  Ta*0.75  Ta*0.5];
Te_ROC = [Te*1.5  Te*1.25  Te*0.75  Te*0.5];
Tg_ROC = [Tg*1.5  Tg*1.25  Tg*0.75  Tg*0.5];
Tss_ROC = [Tss*1.5 Tss*1.25 Tss*0.75 Tss*0.5];

%% Simulation settings
starting_time   = 0;
simulation_time = 10;
sampling_time   = 0.0001;
t     = starting_time:sampling_time:simulation_time;
Betha = 1.0;               % weight of the cost function

nC = size(bestP, 1);
nR = 1;                    % number of robustness cases actually simulated

% preallocate
SS_Test  = zeros(nC, nR);  OS_Test  = zeros(nC, nR);
ST_Test  = zeros(nC, nR);  RT_Test  = zeros(nC, nR);
PT_Test  = zeros(nC, nR);  P_Test   = zeros(nC, nR);
FOD_Test = zeros(nC, nR);
J_ISE  = zeros(1, nC);     J_IAE   = zeros(1, nC);
J_ITAE = zeros(1, nC);     J_ITSE  = zeros(1, nC);

%% simulate every controller
for ii = 1:nR
    % To study parameter drift, assign the scaled constants here, e.g.:
    %   Ta = Ta_ROC(ii);  Te = Te_ROC(ii);  Tg = Tg_ROC(ii);  Tss = Tss_ROC(ii);

    for i = 1:nC
        Kp     = bestP(i,1);   Ki = bestP(i,2);   Kd = bestP(i,3);
        lambda = bestP(i,4);   mu = bestP(i,5);

        sim(model);            % AVRsim reads Kp..mu from the base workspace

        if ~exist('Output', 'var')
            error('SigmoidTest:noOutput', ...
                ['"Output" was not found in the base workspace after sim().\n' ...
                 'In the model: Configuration Parameters > Data Import/Export, ' ...
                 'disable "Single simulation output" (or call out = sim(model) ' ...
                 'and unpack out.Output, out.ISE, ...).']);
        end

        stepResults = stepinfo(Output, t, 'SettlingTimeThreshold', 0.05);

        SS_Test(i,ii) = abs(1 - Output(end));    % BUGFIX: was Output(100000)
        OS_Test(i,ii) = stepResults.Overshoot;
        ST_Test(i,ii) = stepResults.SettlingTime;
        RT_Test(i,ii) = stepResults.RiseTime;
        PT_Test(i,ii) = stepResults.PeakTime;
        P_Test(i,ii)  = abs(stepResults.Peak - 1);

        % objective function
        FOD_Test(i,ii) = (1-exp(-Betha))*(P_Test(i,ii) + SS_Test(i,ii)) ...
                       + exp(-Betha)*(ST_Test(i,ii) - RT_Test(i,ii));

        J_ISE(i)  = ISE(end);    % from the To Workspace blocks inside AVRsim
        J_IAE(i)  = IAE(end);
        J_ITAE(i) = ITAE(end);
        J_ITSE(i) = ITSE(end);
    end
end

%% summary
fprintf('\n%-20s %10s %10s %10s %10s %12s\n', ...
    'controller', 'OS [%]', 'Ts [s]', 'Tr [s]', '|peak-1|', 'FOD')
for i = 1:nC
    fprintf('%-20s %10.4g %10.4g %10.4g %10.4g %12.4g\n', ...
        char(labels(i)), OS_Test(i,ii), ST_Test(i,ii), RT_Test(i,ii), ...
        P_Test(i,ii), FOD_Test(i,ii))
end

fprintf('\nError-integral costs:\n')
fprintf('%-20s %12s %12s %12s %12s\n', 'controller', 'ISE', 'IAE', 'ITAE', 'ITSE')
for i = 1:nC
    fprintf('%-20s %12.4g %12.4g %12.4g %12.4g\n', ...
        char(labels(i)), J_ISE(i), J_IAE(i), J_ITAE(i), J_ITSE(i))
end

%% (optional) plots
% figure; hold on; plot(Output);           % response of the last controller
% figure; plot(Input); plot(Disturbance);
% P = load('SFA-FOPID_AVR','bestP');       % alternative source for the gains
