rng default
close all
clear all
clc

title = 'NLSFA_AVR';
tic
clear P

for iter=1:5  % <---- 25 try
    %% initialize
    lb=0.001;
    ub=2;

    k=1;
    Nc=50-1;
    p = 5;
    P(1:p,k) = [1 0.03 0.01 1 1]; % highlight simple initialization
    [OS(k,iter),Tr(k,iter),Ts(k,iter),Mp(k,iter),Jf(k,iter)] = ...
        PIDAVR(P(:,k),'plotStepBode',"0","0");

    Pbar = P(:,k);
    Jbar = Jf(k,iter);
    display(['iter: ',num2str(iter),', k: ',num2str(k),', Jbar: ',num2str(Jbar)])

    %% optimization
    while k <= Nc

        ak = 0.5/(k+1)^1;
        ck = 0.05/(k+1)^1;
        delta = 0.01;
        eta = sign(randn(p,1));

        %% Learning
        Pplus = P(:,k)+ck.*eta(:,1);
        for i=1:5
            Pplus(i)=min(max(Pplus(i),lb),ub);
        end
        [OSplus,Trplus,Tsplus,Mpplus,Jplus(k)] = PIDAVR(Pplus,'plotStepBode',"0","0");

        Pminus = P(:,k)-ck.*eta(:,1);
        for i=1:5
            Pminus(i)=min(max(Pminus(i),lb),ub);
        end
        [OSminus,Trminus,Tsminus,Mpminus,Jminus(k)] = PIDAVR(Pminus,'plotStepBode',"0","0");

        %% Gradient
        ghat=(Jplus(k)-Jminus(k)).*eta(:,1)./(2*ck);
        var = ak*ghat;
        var = min(max(var,-delta),delta); % norm-limited

        %% Step
        P(:,k+1)=P(:,k)-var;
        for i=1:5
            P(i,k+1)=min(max(P(i,k+1),lb),ub);
        end
        [OS(k+1,iter),Tr(k+1,iter),Ts(k+1,iter),Mp(k+1,iter),Jf(k+1,iter)] = ...
            PIDAVR( P(:,k+1) ,'plotStepBode',"0","0");

        Jbar = Jf(k+1,iter);
        Pbar(:) = P(:,k+1);
        k=k+1;
        display(['iter: ',num2str(iter),', k: ',num2str(k),', Jbar: ',num2str(Jbar)])
    end

    toc

    Jgood(iter)=Jbar;
    Pgood(:,iter)=Pbar;
    treq(iter)=toc;

end

Pbest=Pgood(:,find(Jgood==min(Jgood)));
[bestOS,bestTr,bestTs,bestMp,bestJ] = PIDAVR( Pbest ,'plotStepBode',"r","-");
bestRespond(1,:) = ["Mp%" "Mp" "Ts" "Tr" "J" "1/J"];
bestRespond(2,:) = [bestOS bestMp bestTs bestTr bestJ 1/bestJ];

figure(1)
hold on
plot(Jf(:,find(Jgood==min(Jgood))),'r');

AvgJ=mean(Jgood)            % Mean of the cost function
BestJgood = min(Jgood);     % Best
WorstJgood = max(Jgood);    % Worst
stdJ=std(Jgood);            % Std

% save(title);