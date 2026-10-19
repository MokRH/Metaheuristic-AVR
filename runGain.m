close all
clear
clc

bestP = [...
    1.4509 0.6567 0.3076 1.1442 1.2145;... %1 SCA-FOPID (Mustafa)
    0.7837 0.5027 0.2307 1.0103 1.0727;... %2 SA-FOPID
    1.0537 0.4418 0.2510 1.0624 1.1122;... %3 CAS-FOPID
    1.6264 0.2956 0.3226 1.3183 1.1980;... %4 PSO-FOPID
    0.8399 1.3359 0.3512 0.9147 0.7107;... %5 NSGA II-FOPID
    1.2605,0.9855,0.3264,1.0578,1.1615;... %6 SFA-FOPID
    1.1662,0.3779,0.2923,0.9648,1.1765;... %7 NLSFA-FOPID
    1.6889,0.0142,0.3847,1.4372,1.3339;... %8 MSFA-FOPID
    1.7251,0.0010,0.3905,1.4370,1.3414;... %9 CMSFA-FOPID
    
    %     0.9826 0.8337 0.4982 1 1;... %8 SCA-PID
    %     1.2464 0.5893 0.4596 1 1;... %9 BBO-PID
    %     1.6524 0.4083 0.3654 1 1;... %10 ABC-PID
    %     0.6746 0.6009 0.2618 1 1;... %11 CAS-PID
    %     0.7722 0.7201 0.3196 1 1;... %12 GA-PID (Gaing) Ts_2%
    %     0.6751 0.5980 0.2630 1 1;... %13 PSO-PID (Gaing) Ts_2%
    ];
color = ["m","y" ,"c" ,"g" ,"r" ,"y","c","g","r"];
style = ["--","--","--","--","--","-","-","-","-"];
bestRespond(1,:) = ["Mp%" "Mp" "Ts" "Tr" "J" "1/J"];
bodeData(1,:) = ["pm","wg","td"];
for i = 1:9
    [bestOS,bestTr,bestTs,bestMp,bestJ,bodeAnalysis,PZ] = PIDAVR(bestP(i,:),'plotStepBode',color(i),style(i));
    bestRespond(i+1,:) = [bestOS bestMp-1 bestTs bestTr bestJ 1/bestJ];
    bodeData(i+1,:)=bodeAnalysis(:);
    PZData(:,i)=PZ;
end

figure(11)
plot(0:10,[1 1 1 1 1 1 1 1 1 1 1],'k','LineWidth',1.5,'LineStyle','--');

(179.8169-43.7524)/179.8169*100
