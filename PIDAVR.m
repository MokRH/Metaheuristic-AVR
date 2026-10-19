%
% Please refer the following manuscript: https://journals.sagepub.com/doi/abs/10.1177/0142331218811453
%
% S=fotf('s');
% PID_long = 1 + (2/S^1.2) + (3*S^0.8)
% PID = minreal(oustapp(PID_long, 1e-5, 1e5, 3, 'oust'))

function [OS,Tr,Ts,Mp,J,bodeAnalysis,PZ] = PIDAVR(P,~,color,style)
kp = P(1);
ki = P(2);
kd = P(3);
lambda = P(4);
mew = P(5);

s=tf('s');
Amplifier=10/(1+0.1*s);%kA/(1+TAs)
Exciter=1/(1+0.4*s);   %kE/(1+TEs)
Generator=1/(1+1.0*s); %kG/(1+TGs)
Sensor=1/(1+0.01*s);   %kS/(1+TSs)

S=fotf('s');
FOPID_long = kp + (ki/S^lambda) + (kd*S^mew);
FOPID = minreal(oustapp(FOPID_long, 1e-5, 1e5, 5, 'oust')); % convert fotf to tf using Oustaloup method

G = (FOPID*Amplifier*Exciter*Generator);

er=1;
try
    % it is proved that CL=CL2
    CL=G/(1+G*Sensor);
    % CL2=feedback(G,Sensor);
catch
    er=2;
    msg=lasterr;
    %disp(msg)
end
if er==2
    OS=1e10;
    Tr=1e10;
    Ts=1e10;
    Mp=1e10;
    B=1;

    J=( 1-exp(-B) )*( (Mp-1)+abs(Ess(end)-1) )+( Ts-Tr )*exp(-B);
else
    Q = stepinfo(CL,'SettlingTimeThreshold',0.05,'RiseTimeThreshold',[0.1 0.9]); % Transfer function dari mfile
    % Q=stepinfo(CL);
    [Ess,t] = step(CL,5);

    if color ~= "0" && style ~= "0"
        figure(11)
        hold on
        plot(t,Ess,color,'LineWidth',1.5,'LineStyle',style)

        ww = linspace(1e-0,1e2*pi,1e5);
        figure(12)
        hold on
        bode(CL, ww);
        lineHandle = findobj(gcf,'Type','line','-and',...
            '-not',{'Color','m','-or','Color','y','-or','Color','c','-or','Color','g','-or','Color','r'});
        set(lineHandle,'Color',color,'LineStyle',style,'LineWidth',1.5);
    end

    OS=Q.Overshoot;
    Tr=Q.RiseTime;
    Ts=Q.SettlingTime;
    Mp=Q.Peak;
    B=1;

    J=( 1-exp(-B) )*( (Mp-1)+abs(Ess(end)-1) )+( Ts-Tr )*exp(-B);
end

allmargin(CL);
[gm,pm,wg,wp]=margin(CL); % gain margin, phase margin, gain freq, phase freq
bodeAnalysis = [pm,wp,(pm/wp)*(pi/180)];
PZ=0;
end

