close all
clear
clc

addpath('../../../bag')

%% Parameters
sampling = 1;
Ts_sim = 0.001;

%% Save and plot data
TRAJ = [1 4 7 10 17 20,   3 5 11 15 19 21,   23 26,   29 30]; 
nTraj = 16; 
nSamples = [924 864 862 1100 756 882,   1168 926 857 1190 865 756,  780 1167,    1000 1000];

for i=1:nTraj
    realTraj = load(append('../../experimental_data_analysis/licas/experiments_testbed_2022-09-06/finalNatRes',int2str(TRAJ(i)),'.txt'));                  % Retrieve data from file
    t = realTraj(1:nSamples(i),1);
    time = t-t(1);
    meanTs(i) = mean(time(2:end)-time(1:end-1));
    qRef = realTraj(1:nSamples(i),10:17);
    q = realTraj(1:nSamples(i),10:17);
    qd = realTraj(1:nSamples(i),18:25);
    dualArmPosition = realTraj(1:nSamples(i),58:60);
    dualArmQuaternion = realTraj(1:nSamples(i),61:64);
    a = quat2eul(dualArmQuaternion(:,1:4), 'ZYX');
    a(:,1:2) = a(:,1:2) * (-1);
    for j = 1:size(a(:,1))
         if a(j,1) < 0
             a(j,1) = a(j,1) + pi;
         else
             a(j,1) = a(j,1) - pi;
         end
    end
    dualArmOrientation = a;
    
    % Set to zero small angular variations
    %     if max(abs(dualArmPosition(:,1)))<0.05
    %         dualArmPosition(:,1) = zeros(length(dualArmPosition(:,1)),1);        
    %     end
    %     if max(abs(dualArmPosition(:,2)))<0.05
    %         dualArmPosition(:,2) = zeros(length(dualArmPosition(:,2)),1);        
    %     end
    %     if max(abs(dualArmOrientation(:,3)))<0.2
    %         dualArmOrientation(:,3) = zeros(length(dualArmOrientation(:,3)),1);        
    %     end

    % Save data in data structures
    shoulderX = dualArmPosition(:,1);
    shoulderY = dualArmPosition(:,2);
    shoulderYaw = filloutliers(dualArmOrientation(:,3),'makima');

    timeRealVec{i} = time;
    shoulderPoseRealVec{i} = [shoulderX shoulderY shoulderYaw];
    qRefVec{i} = qRef;

    qRealVec{i} = q;
    qdRealVec{i} = qd;
    
    % Compute velocity commands
    for k=2:nSamples(i)-1
        qdRefVec{i}(k-1,:) = downsample((qRefVec{i}(k,:)-qRefVec{i}(k-1,:))/meanTs(i),sampling);
    end
    qdRefVec{i}(nSamples(i),:) = qRefVec{i}(end,:);
    
    % Plot
%     figure(1), sgtitle('Shoulder Position');
%     subplot(2,ceil(nTraj/2),i), plot(time,[shoulderX shoulderY]), grid, hold on, xlim([time(1) time(end)]), ylim([-0.4 0.4]), xlabel('time [s]'), ylabel('position [m]'), title(append('Trajectory ',int2str(i))), legend('X Position','Y Position')
%     figure(2), sgtitle('Shoulder Orientation')
%     subplot(2,ceil(nTraj/2),i), plot(time,180/pi*shoulderYaw), grid, hold on, xlim([time(1) time(end)]), ylim([-100 100]), xlabel('time [s]'), ylabel('orientation [deg]'), title(append('Trajectory ',int2str(i)))

end

%% Eliminare eventuali campioni doppioni nel vettore dei tempi
for i=1:nTraj
    [timeRealVec{i},ia,ic] = unique(timeRealVec{i});
    for k=ic(1):length(ic)
        if (~ismember(k,ia))
            qRefVec{i}(k,:) = [];
            qdRefVec{i}(k,:) = [];
            qRealVec{i}(k,:) = [];
            qdRealVec{i}(k,:) = [];
            shoulderPoseRealVec{i}(k,:) = [];
        end
    end
end

%% Interpolare comandi e dati
for i=1:nTraj
    tModel{i} = (0:Ts_sim*sampling:meanTs(i)*nSamples(i))';

    qRefVecInterp{i} = interp1(timeRealVec{i},qRefVec{i},tModel{i},'spline');
    qdRefVecInterp{i} = interp1(timeRealVec{i},qdRefVec{i},tModel{i},'spline');
    qRealVecInterp{i} = interp1(timeRealVec{i},qRealVec{i},tModel{i},'spline');
    qdRealVecInterp{i} = interp1(timeRealVec{i},qdRealVec{i},tModel{i},'spline');
    shoulderPoseRealVecInterp{i} = interp1(timeRealVec{i},shoulderPoseRealVec{i},tModel{i},'spline');

end

%% Eliminare eventuali NaN
for i=1:nTraj
    TF = isnan(qRefVecInterp{i});
    qRefVecInterp{i}(TF) = 0;
    TF = isnan(qdRefVecInterp{i});
    qdRefVecInterp{i}(TF) = 0;
    TF = isnan(qRealVecInterp{i});
    qRealVecInterp{i}(TF) = 0;
    TF = isnan(qdRealVecInterp{i});
    qdRealVecInterp{i}(TF) = 0;
    TF = isnan(shoulderPoseRealVecInterp{i});
    shoulderPoseRealVecInterp{i}(TF) = 0;
 
end

%% Definire le starting conditions
for i=1:nTraj
    q0{i} = qRealVecInterp{i}(1,:)';
    qd0{i} = qdRealVecInterp{i}(1,:)';
    qdd0{i} = zeros(8,1);
    shoulderPoseReal0{i} = shoulderPoseRealVec{i}(1,:)';
end

%% Model dynamics

p = [3.5463    0.7471    0.5882   0.5679,      1.9343    0.9081    0.7135   0.3563,     0.7222    0.8343    19.2797];


L = p(1);
for i=1:nTraj
    
    disp(append('Simulating dynamics for trajectory ',int2str(i)));
    
    qModel{i}(1,1:3) = [shoulderPoseReal0{i}(3) asin(shoulderPoseReal0{i}(2)/L) asin(shoulderPoseReal0{i}(1)/L) ];
    qModel{i}(1,4:5) = [-qModel{i}(1,2) -qModel{i}(1,3)];
    qModel{i}(1,6:13) = q0{i};
    qdModel{i}(1,1:5) = zeros(1,5);
    qdModel{i}(1,6:13) = qd0{i};
    shoulderPoseModel{i}(1,:) = shoulderPoseReal0{i};

    for k=2:(length(tModel{i}))
        [qModel{i}(k,:), qdModel{i}(k,:)] = qFunModelLicas(p,qModel{i}(k-1,:),qdModel{i}(k-1,:),Ts_sim,qRefVecInterp{i}(k-1,:),qdRefVecInterp{i}(k-1,:));
    end
    shoulderPoseModel{i} = [L*sin(qModel{i}(:,3)) L*sin(qModel{i}(:,2)) qModel{i}(:,1)];                                                           % Compute the position of the shoulders again

end

%% Plot

A = {'Shoulder X position','Shoulders Y position','Shoulder Z Orientation'};
Y = {'position [m]', 'position [m]', 'angle [rad]'};
YlimL = [-0.4 -0.3 -1.5];
YlimU = [0.35 0.3 1.7];
YErrlimU = [0.15 0.1 0.6];


set(0, 'DefaultTextInterpreter', 'latex')
set(0, 'DefaultLegendInterpreter', 'latex')
set(0, 'DefaultAxesTickLabelInterpreter', 'latex')
lw = 2;

%% IDENTIFICATION: X EXPERIMENT #1
j=1;  i=1;
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);

plot(tModel{i}, shoulderPoseRealVecInterp{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]); hold on
plot(tModel{i}, shoulderPoseModel{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
plot(tModel{i},abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j)), 'k--', 'Linewidth', 1.5 ,'Color', [0.2, 0.2, 0.2])
legend('Real','Model','Error','Orientation','horizontal'); xlabel('t [s]'); ylabel('[m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
set(gca, 'YLim', [-0.3 0.3], 'YTick', [-0.3:0.1:0.3]);
grid on; box on; set(gcf,'color','w');
% exportgraphics(h, 'LicasX1Id.pdf');

%% IDENTIFICATION: X EXPERIMENT #2
j=1;  i=2;
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);

plot(tModel{i}, shoulderPoseRealVecInterp{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]); hold on
plot(tModel{i}, shoulderPoseModel{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
plot(tModel{i},abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j)), 'k--', 'Linewidth', 1.5 ,'Color', [0.2, 0.2, 0.2])
legend('Real','Model','Error','Orientation','horizontal'); xlabel('t [s]'); ylabel('[m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
set(gca, 'YLim', [-0.3 0.3], 'YTick', [-0.3:0.1:0.3]);
grid on; box on; set(gcf,'color','w');
% exportgraphics(h, 'LicasX2Id.pdf');

%% IDENTIFICATION: Y EXPERIMENT #3
j=2;  i=3;
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);

plot(tModel{i}, shoulderPoseRealVecInterp{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]); hold on
plot(tModel{i}, shoulderPoseModel{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
plot(tModel{i},abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j)), 'k--', 'Linewidth', 1.5 ,'Color', [0.2, 0.2, 0.2])
legend('Real','Model','Error','Orientation','horizontal'); xlabel('t [s]'); ylabel('[m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
set(gca, 'YLim', [-0.3 0.3], 'YTick', [-0.3:0.1:0.3]);
grid on; box on; set(gcf,'color','w');
% exportgraphics(h, 'LicasY1Id.pdf');

%% IDENTIFICATION: Y EXPERIMENT #4
j=2;  i=4;
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);

plot(tModel{i}, shoulderPoseRealVecInterp{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]); hold on
plot(tModel{i}, shoulderPoseModel{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
plot(tModel{i},abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j)), 'k--', 'Linewidth', 1.5 ,'Color', [0.2, 0.2, 0.2])
legend('Real','Model','Error','Orientation','horizontal'); xlabel('t [s]'); ylabel('[m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
set(gca, 'YLim', [-0.3 0.3], 'YTick', [-0.3:0.1:0.3]);
grid on; box on; set(gcf,'color','w');
% exportgraphics(h, 'LicasY2Id.pdf');

%% IDENTIFICATION: Z EXPERIMENT #5
j=3;  i=5;
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);

plot(tModel{i}, 180/pi*shoulderPoseRealVecInterp{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]); hold on
plot(tModel{i}, 180/pi*shoulderPoseModel{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
plot(tModel{i},180/pi*abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j)), 'k--', 'Linewidth', 1.5 ,'Color', [0.2, 0.2, 0.2])
legend('Real','Model','Error','Orientation','horizontal'); xlabel('t [s]'); ylabel('[deg]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
set(gca, 'YLim', [-60 60], 'YTick', [-60:20:60]);
grid on; box on; set(gcf,'color','w');
% exportgraphics(h, 'LicasZ1Id.pdf');

%% IDENTIFICATION: Z EXPERIMENT #6
j=3;  i=6;
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);

plot(tModel{i}, 180/pi*shoulderPoseRealVecInterp{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]); hold on
plot(tModel{i}, 180/pi*shoulderPoseModel{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
plot(tModel{i},180/pi*abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j)), 'k--', 'Linewidth', 1.5 ,'Color', [0.2, 0.2, 0.2])
legend('Real','Model','Error','Orientation','horizontal'); xlabel('t [s]'); ylabel('[deg]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
set(gca, 'YLim', [-80 80], 'YTick', [-80:20:80]);
grid on; box on; set(gcf,'color','w');
% exportgraphics(h, 'LicasZ2Id.pdf');

%% VALIDATION: X EXPERIMENT #1
j=1;  i=7;
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);

plot(tModel{i}, shoulderPoseRealVecInterp{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]); hold on
plot(tModel{i}, shoulderPoseModel{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
plot(tModel{i},abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j)), 'k--', 'Linewidth', 1.5 ,'Color', [0.2, 0.2, 0.2])
legend('Real','Model','Error','Orientation','horizontal'); xlabel('t [s]'); ylabel('[m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
set(gca, 'YLim', [-0.3 0.3], 'YTick', [-0.3:0.1:0.3]);
grid on; box on; set(gcf,'color','w');
% exportgraphics(h, 'LicasX1Val.pdf');

%% VALIDATION: X EXPERIMENT #2
j=1;  i=8;
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);

plot(tModel{i}, shoulderPoseRealVecInterp{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]); hold on
plot(tModel{i}, shoulderPoseModel{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
plot(tModel{i},abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j)), 'k--', 'Linewidth', 1.5 ,'Color', [0.2, 0.2, 0.2])
legend('Real','Model','Error','Orientation','horizontal'); xlabel('t [s]'); ylabel('[m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
set(gca, 'YLim', [-0.3 0.3], 'YTick', [-0.3:0.1:0.3]);
grid on; box on; set(gcf,'color','w');
% exportgraphics(h, 'LicasX2Val.pdf');

%% VALIDATION: Y EXPERIMENT #3
j=2;  i=9;
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);

plot(tModel{i}, shoulderPoseRealVecInterp{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]); hold on
plot(tModel{i}, shoulderPoseModel{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
plot(tModel{i},abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j)), 'k--', 'Linewidth', 1.5 ,'Color', [0.2, 0.2, 0.2])
legend('Real','Model','Error','Orientation','horizontal'); xlabel('t [s]'); ylabel('[m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
set(gca, 'YLim', [-0.3 0.3], 'YTick', [-0.3:0.1:0.3]);
grid on; box on; set(gcf,'color','w');
% exportgraphics(h, 'LicasY1Val.pdf');

%% VALIDATION: Y EXPERIMENT #4
j=2;  i=10;
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);

plot(tModel{i}, shoulderPoseRealVecInterp{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]); hold on
plot(tModel{i}, shoulderPoseModel{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
plot(tModel{i},abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j)), 'k--', 'Linewidth', 1.5 ,'Color', [0.2, 0.2, 0.2])
legend('Real','Model','Error','Orientation','horizontal'); xlabel('t [s]'); ylabel('[m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
set(gca, 'YLim', [-0.3 0.3], 'YTick', [-0.3:0.1:0.3]);
grid on; box on; set(gcf,'color','w');
% exportgraphics(h, 'LicasY2Val.pdf');

%% VALIDATION: Z EXPERIMENT #5
j=3;  i=11;
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);

plot(tModel{i}, 180/pi*shoulderPoseRealVecInterp{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]); hold on
plot(tModel{i}, 180/pi*shoulderPoseModel{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
plot(tModel{i},180/pi*abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j)), 'k--', 'Linewidth', 1.5 ,'Color', [0.2, 0.2, 0.2])
legend('Real','Model','Error','Orientation','horizontal'); xlabel('t [s]'); ylabel('[deg]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
set(gca, 'YLim', [-60 60], 'YTick', [-60:20:60]);
grid on; box on; set(gcf,'color','w');
% exportgraphics(h, 'LicasZ1Val.pdf');

%% VALIDATION: Z EXPERIMENT #6
j=3;  i=12;
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);

plot(tModel{i}, 180/pi*shoulderPoseRealVecInterp{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]); hold on
plot(tModel{i}, 180/pi*shoulderPoseModel{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
plot(tModel{i},180/pi*abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j)), 'k--', 'Linewidth', 1.5 ,'Color', [0.2, 0.2, 0.2])
legend('Real','Model','Error','Orientation','horizontal'); xlabel('t [s]'); ylabel('[deg]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
set(gca, 'YLim', [-60 60], 'YTick', [-60:20:60]);
grid on; box on; set(gcf,'color','w');
% exportgraphics(h, 'LicasZ2Val.pdf');

%% VALIDATION: XY EXPERIMENT #7
j=1;  i=13;
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);

subplot(1,2,1);
plot(tModel{i}, shoulderPoseRealVecInterp{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]); hold on
plot(tModel{i}, shoulderPoseModel{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
plot(tModel{i}, abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j)), 'k--', 'Linewidth', 1.5 ,'Color', [0.2, 0.2, 0.2])
legend('Real','Model','Error','Orientation','horizontal');  ylabel('[m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
set(gca, 'YLim', [-0.3 0.3], 'YTick', [-0.3:0.1:0.3]);
grid on; box on; set(gcf,'color','w');

j=2;
subplot(1,2,2);
plot(tModel{i}, shoulderPoseRealVecInterp{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]); hold on
plot(tModel{i}, shoulderPoseModel{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
plot(tModel{i}, abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j)), 'k--', 'Linewidth', 1.5 ,'Color', [0.2, 0.2, 0.2])
legend('Real','Model','Error','Orientation','horizontal'); xlabel('t [s]'); ylabel('[m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
set(gca, 'YLim', [-0.3 0.3], 'YTick', [-0.3:0.1:0.3]);
grid on; box on; set(gcf,'color','w');
% exportgraphics(h, 'LicasY2Val.pdf');


%% VALIDATION: XY EXPERIMENT #8
j=1;  i=14;
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);

subplot(1,2,1);
plot(tModel{i}, shoulderPoseRealVecInterp{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]); hold on
plot(tModel{i}, shoulderPoseModel{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
plot(tModel{i}, abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j)), 'k--', 'Linewidth', 1.5 ,'Color', [0.2, 0.2, 0.2])
legend('Real','Model','Error','Orientation','horizontal');  ylabel('[m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
set(gca, 'YLim', [-0.3 0.3], 'YTick', [-0.3:0.1:0.3]);
grid on; box on; set(gcf,'color','w');

j=2;
subplot(1,2,2);
plot(tModel{i}, shoulderPoseRealVecInterp{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]); hold on
plot(tModel{i}, shoulderPoseModel{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
plot(tModel{i}, abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j)), 'k--', 'Linewidth', 1.5 ,'Color', [0.2, 0.2, 0.2])
legend('Real','Model','Error','Orientation','horizontal'); xlabel('t [s]'); ylabel('[m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
set(gca, 'YLim', [-0.3 0.3], 'YTick', [-0.3:0.1:0.3]);
grid on; box on; set(gcf,'color','w');
% exportgraphics(h, 'LicasY2Val.pdf');

%% VALIDATION: XYZ EXPERIMENT #9
j=1;  i=15;
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);

subplot(1,3,1);
plot(tModel{i}, shoulderPoseRealVecInterp{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]); hold on
plot(tModel{i}, shoulderPoseModel{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
plot(tModel{i}, abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j)), 'k--', 'Linewidth', 1.5 ,'Color', [0.2, 0.2, 0.2])
legend('Real','Model','Error','Orientation','horizontal');  ylabel('[m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
set(gca, 'YLim', [-0.3 0.3], 'YTick', [-0.3:0.1:0.3]);
grid on; box on; set(gcf,'color','w');

j=2;
subplot(1,3,2);
plot(tModel{i}, shoulderPoseRealVecInterp{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]); hold on
plot(tModel{i}, shoulderPoseModel{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
plot(tModel{i}, abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j)), 'k--', 'Linewidth', 1.5 ,'Color', [0.2, 0.2, 0.2])
legend('Real','Model','Error','Orientation','horizontal'); xlabel('t [s]'); ylabel('[m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
set(gca, 'YLim', [-0.3 0.3], 'YTick', [-0.3:0.1:0.3]);
grid on; box on; set(gcf,'color','w');

j=3;
subplot(1,3,3);
plot(tModel{i}, 180/pi*shoulderPoseRealVecInterp{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]); hold on
plot(tModel{i}, 180/pi*shoulderPoseModel{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
plot(tModel{i}, 180/pi*abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j)), 'k--', 'Linewidth', 1.5 ,'Color', [0.2, 0.2, 0.2])
legend('Real','Model','Error','Orientation','horizontal'); xlabel('t [s]'); ylabel('[deg]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
set(gca, 'YLim', [-60 60], 'YTick', [-60:20:60]);
grid on; box on; set(gcf,'color','w');
% exportgraphics(h, 'LicasY2Val.pdf');

%% VALIDATION: XYZ EXPERIMENT #10
j=1;  i=16;
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);

subplot(1,3,1);
plot(tModel{i}, shoulderPoseRealVecInterp{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]); hold on
plot(tModel{i}, shoulderPoseModel{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
plot(tModel{i}, abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j)), 'k--', 'Linewidth', 1.5 ,'Color', [0.2, 0.2, 0.2])
legend('Real','Model','Error','Orientation','horizontal');  ylabel('[m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
set(gca, 'YLim', [-0.3 0.3], 'YTick', [-0.3:0.1:0.3]);
grid on; box on; set(gcf,'color','w');

j=2;
subplot(1,3,2);
plot(tModel{i}, shoulderPoseRealVecInterp{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]); hold on
plot(tModel{i}, shoulderPoseModel{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
plot(tModel{i}, abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j)), 'k--', 'Linewidth', 1.5 ,'Color', [0.2, 0.2, 0.2])
legend('Real','Model','Error','Orientation','horizontal'); xlabel('t [s]'); ylabel('[m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
set(gca, 'YLim', [-0.3 0.3], 'YTick', [-0.3:0.1:0.3]);
grid on; box on; set(gcf,'color','w');

j=3;
subplot(1,3,3);
plot(tModel{i}, 180/pi*shoulderPoseRealVecInterp{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]); hold on
plot(tModel{i}, 180/pi*shoulderPoseModel{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
plot(tModel{i}, 180/pi*abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j)), 'k--', 'Linewidth', 1.5 ,'Color', [0.2, 0.2, 0.2])
legend('Real','Model','Error','Orientation','horizontal'); xlabel('t [s]'); ylabel('[deg]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
set(gca, 'YLim', [-60 60], 'YTick', [-60:20:60]);
grid on; box on; set(gcf,'color','w');
% exportgraphics(h, 'LicasY2Val.pdf');