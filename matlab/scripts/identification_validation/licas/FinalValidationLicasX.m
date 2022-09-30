close all
clear
clc

addpath('../../../bag')

%% Parameters
sampling = 1;
Ts_sim = 0.001;

%% Save and plot data
TRAJ = [1 2 3 4 5 6]; nTraj = 6; nSamples = [924 924 926 926 926 921];          % natRes

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
    
%     figure(6), sgtitle('Shoulder Position');
%     subplot(2,ceil(nTraj/2),i), plot(tModel{i},shoulderPoseRealVecInterp{i}(:,1:2)), grid
%     figure(7), sgtitle('Shoulder Orientation')
%     subplot(2,ceil(nTraj/2),i), plot(tModel{i},180/pi*shoulderPoseRealVecInterp{i}(:,3)), grid
%     figure(8), sgtitle('Left Arm References')
%     subplot(2,ceil(nTraj/2),i), plot(tModel{i},180/pi*qRefVecInterp{i}), grid
%     figure(9), sgtitle('Left Arm Position')
%     subplot(2,ceil(nTraj/2),i), plot(tModel{i},180/pi*qRealVecInterp{i}), grid
%     figure(10), sgtitle('Left Arm Position')
%     subplot(2,ceil(nTraj/2),i), plot(tModel{i},180/pi*qdRealVecInterp{i}), grid

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
    
%     figure(11), subplot(2,ceil(nTraj/2),i);
%     plot(tModel{i},qRefVecInterp{i}.*180/pi); grid, hold on; xlim([tModel{i}(1) tModel{i}(end)]), xlabel('time [s]'), ylabel('angular position [deg]')
%     figure(12), subplot(2,ceil(nTraj/2),i);
%     plot(tModel{i},qRefVecInterp{i}.*180/pi); grid, hold on; xlim([tModel{i}(1) tModel{i}(end)]), xlabel('time [s]'), ylabel('angular velocity [deg/s]')
%     figure(13), subplot(2,ceil(nTraj/2),i);
%     plot(tModel{i},qRealVecInterp{i}.*180/pi); grid, hold on; xlim([tModel{i}(1) tModel{i}(end)]), xlabel('time [s]'), ylabel('angular position [deg]')
%     figure(14), subplot(2,ceil(nTraj/2),i);
%     plot(tModel{i},qdRealVecInterp{i}.*180/pi); grid, hold on; xlim([tModel{i}(1) tModel{i}(end)]), xlabel('time [s]'), ylabel('angular velocity [deg/s]')
%     figure(15), subplot(2,ceil(nTraj/2),i);
%     plot(tModel{i},shoulderPoseRealVecInterp{i}.*180/pi); grid, hold on; xlim([tModel{i}(1) tModel{i}(end)]), xlabel('time [s]')
end

%% Definire le starting conditions
for i=1:nTraj
    q0{i} = qRealVecInterp{i}(1,:)';
    qd0{i} = qdRealVecInterp{i}(1,:)';
    qdd0{i} = zeros(8,1);
    shoulderPoseReal0{i} = shoulderPoseRealVec{i}(1,:)';
end

% %% Load Simulation Data
% 
% simDataVid1X = load('simDataVid1NatRespX.txt');
% simDataVid1Y = load('simDataVid1NatRespY.txt');
% simDataVid1Z = load('simDataVid1NatRespZ.txt');
% simDataVid2X = load('simDataVid2NatRespX.txt');
% simDataVid2Y = load('simDataVid2NatRespY.txt');
% simDataVid2Z = load('simDataVid2NatRespZ.txt');
% simDataVid2XY = load('simDataVid2NatRespXY.txt');
% 
% timeSimVid1X = simDataVid1X(2:end,1)-simDataVid1X(1,1);
% dualArmPositionSimVid1X = simDataVid1X(2:end,2);
% timeSimVid1Y = simDataVid1Y(2:end,1)-simDataVid1Y(1,1);
% dualArmPositionSimVid1Y = simDataVid1Y(2:end,3);
% timeSimVid1Z = simDataVid1Z(2:end,1)-simDataVid1Z(1,1);
% dualArmQuaternionSim = simDataVid1Z(2:end,4:7);
% a = quat2eul(dualArmQuaternionSim(:,1:4), 'XYZ');
% a(:,1:2) = a(:,1:2) * (-1);
% for i = 1:length(a)
%      if a(i,1) < 0
%          a(i,1) = a(i,1) + pi;
%      else
%          a(i,1) = a(i,1) - pi;
%      end
% end
% dualArmOrientationSimVid1 = a(:,3);
% 
% timeSimVid2X = simDataVid2X(2:end,1)-simDataVid2X(1,1);
% dualArmPositionSimVid2X = simDataVid2X(2:end,2);
% 
% timeSimVid2Y = simDataVid2Y(2:end,1)-simDataVid2Y(1,1);
% dualArmPositionSimVid2Y = simDataVid2Y(2:end,3);
% 
% timeSimVid2Z = simDataVid2Z(2:end,1)-simDataVid2Z(1,1);
% dualArmQuaternionSim = simDataVid2Z(2:end,4:7);
% a = quat2eul(dualArmQuaternionSim(:,1:4), 'XYZ');
% a(:,1:2) = a(:,1:2) * (-1);
% for i = 1:length(a)
%      if a(i,1) < 0
%          a(i,1) = a(i,1) + pi;
%      else
%          a(i,1) = a(i,1) - pi;
%      end
% end
% dualArmOrientationSimVid2 = a(:,3);
% 
% 
% timeSimVid2XY = simDataVid2XY(2:end,1)-simDataVid2XY(1,1);
% dualArmPositionSimVid2XY = simDataVid2XY(2:end,2:3);

%% Model dynamics

% p = [1.1378   0.3958   2.5000   0.4000   0.4000   0.1   0.1   0.7872   3.39e-3   0.2   0.5232   1.0000   0.4686   16.8314];  % MIGLIORE
% p = [1.05   0.3958   2.3   0.4000   0.4000   0.1   0.1   0.7872   3.39e-3   0.2   0.5232   0.5   0.4686   16.8314];  % MIGLIORE

% p = [1.5928    0.5166];

p = [2.7964    0.4592    0.1675   0.1];


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

%% X EXPERIMENT #1
j=1;  i=1;
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);

plot(tModel{i}, shoulderPoseRealVecInterp{i}(:,j), 'k--', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
hold on
plot(tModel{i}, shoulderPoseModel{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]);
legend('Real','Model'); xlabel('t [s]'); ylabel('x [m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
grid on; box on; set(gcf,'color','w');
exportgraphics(h, 'LicasX1.pdf');

% ERROR
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);
plot(tModel{i},abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j)), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2])
xlabel('t [s]'); ylabel('$\mid e \mid$ [m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
grid on; box on; set(gcf,'color','w');
exportgraphics(h, 'ErrorLicasX1.pdf');

%% X EXPERIMENT #2
j=1;  i=2;
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);

plot(tModel{i}, shoulderPoseRealVecInterp{i}(:,j), 'k--', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
hold on
plot(tModel{i}, shoulderPoseModel{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]);
legend('Real','Model'); xlabel('t [s]'); ylabel('x [m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
grid on; box on; set(gcf,'color','w');
exportgraphics(h, 'LicasX2.pdf');

% Error
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);
plot(tModel{i},abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j)), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
xlabel('t [s]'); ylabel('$\mid e \mid$ [m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
grid on; box on; set(gcf,'color','w');
exportgraphics(h, 'ErrorLicasX2.pdf');

%% X EXPERIMENT #3
j=1;  i=3;
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);

plot(tModel{i}, shoulderPoseRealVecInterp{i}(:,j), 'k--', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
hold on
plot(tModel{i}, shoulderPoseModel{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]);
legend('Real','Model'); xlabel('t [s]'); ylabel('x [m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
grid on; box on; set(gcf,'color','w');
exportgraphics(h, 'LicasX3.pdf');

% Error
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);
plot(tModel{i},abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j)), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
xlabel('t [s]'); ylabel('$\mid e \mid$ [m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
grid on; box on; set(gcf,'color','w');
exportgraphics(h, 'ErrorLicasX3.pdf');  

%% X EXPERIMENT #4
j=1;  i=4;
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);

plot(tModel{i}, shoulderPoseRealVecInterp{i}(:,j), 'k--', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
hold on
plot(tModel{i}, shoulderPoseModel{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]);
legend('Real','Model'); xlabel('t [s]'); ylabel('x [m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
grid on; box on; set(gcf,'color','w');
exportgraphics(h, 'LicasX4.pdf');

% Error
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);
plot(tModel{i},abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j)), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
xlabel('t [s]'); ylabel('$\mid e \mid$ [m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
grid on; box on; set(gcf,'color','w');
exportgraphics(h, 'ErrorLicasX4.pdf');

%% X EXPERIMENT #5
j=1;  i=5;
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);

plot(tModel{i}, shoulderPoseRealVecInterp{i}(:,j), 'k--', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
hold on
plot(tModel{i}, shoulderPoseModel{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]);
legend('Real','Model'); xlabel('t [s]'); ylabel('x [m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
grid on; box on; set(gcf,'color','w');
exportgraphics(h, 'LicasX5.pdf');

% Error
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);
plot(tModel{i},abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j)), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
xlabel('t [s]'); ylabel('$\mid e \mid$ [m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
grid on; box on; set(gcf,'color','w');
exportgraphics(h, 'ErrorLicasX5.pdf');  

%% X EXPERIMENT #6
j=1;  i=6;
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);

plot(tModel{i}, shoulderPoseRealVecInterp{i}(:,j), 'k--', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
hold on
plot(tModel{i}, shoulderPoseModel{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]);
legend('Real','Model'); xlabel('t [s]'); ylabel('x [m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
grid on; box on; set(gcf,'color','w');
exportgraphics(h, 'LicasX6.pdf');

% Error
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);
plot(tModel{i},abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j)), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
xlabel('t [s]'); ylabel('$\mid e \mid$ [m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
grid on; box on; set(gcf,'color','w');
exportgraphics(h, 'ErrorLicasX6.pdf'); 

