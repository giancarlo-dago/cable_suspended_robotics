close all
clear
clc

addpath('../../../bag')

%% Parameters
sampling = 1;
Ts_sim = 0.001;

%% Save and plot data
% TRAJ = [1 2 3 4 5 6 7]; nTraj = 7; nSamples = [220 220 220 220 220 220 220];      % natRes
TRAJ = [1 2 3 4 5 6 7]; nTraj = 7; nSamples = [220 250 250 220 260 215 310];      % natRes
% TRAJ = [1]; nTraj = 1; nSamples = [220];      % natRes
% TRAJ = [1 2 3 4 5 6 7 8]; nTraj = 8; nSamples = [258 343 310 426 380 350 4265 4096];      % Other data
% TRAJ = [1 2 3 4 5 6 8]; nTraj = 7; nSamples = [258 343 310 426 380 350 4096];     % Other data
% TRAJ = [7 8]; nTraj = 2; nSamples = [4265 4096];                                  % Other data
% TRAJ = 1; nTraj = 1; nSamples = 1962;                                               % Other data

for i=1:nTraj
    realTraj = load(append('natResCut',int2str(TRAJ(i)),'.txt'));                 % Retrieve data from file
%     realTraj = load(append('dataMod',int2str(TRAJ(i)),'.txt'));                   % Retrieve data from file
%     realTraj = load(append('dataTb1NoLoadMod.txt'));                                   % Retrieve data from file
    t = realTraj(1:nSamples(i),1);
    time = t-t(1);
    meanTs(i) = mean(time(2:end)-time(1:end-1));
%     qRef = pi/180*realTraj(1:nSamples(i),2:9);
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
    
%     % Plot
%     figure(1), sgtitle('Shoulder Position');
%     %     subplot(2,ceil(nTraj/2),i), plot(time,[shoulderX shoulderY]), grid, hold on, xlim([time(1) time(end)]), ylim([-0.4 0.4]), xlabel('time [s]'), ylabel('position [m]'), title(append('Trajectory ',int2str(i))), legend('X Position','Y Position')
%     subplot(2,ceil(nTraj/2),i), plot(time,[shoulderX shoulderY]), grid, hold on, xlim([time(1) time(end)]),  xlabel('time [s]'), ylabel('position [m]'), title(append('Trajectory ',int2str(i))), legend('X Position','Y Position')
%     figure(2), sgtitle('Shoulder Orientation')
%     %     subplot(2,ceil(nTraj/2),i), plot(time,180/pi*shoulderYaw), grid, hold on, xlim([time(1) time(end)]), ylim([-100 100]), xlabel('time [s]'), ylabel('orientation [deg]'), title(append('Trajectory ',int2str(i)))
%     subplot(2,ceil(nTraj/2),i), plot(time,180/pi*shoulderYaw), grid, hold on, xlim([time(1) time(end)]), xlabel('time [s]'), ylabel('orientation [deg]'), title(append('Trajectory ',int2str(i)))
%     figure(3), sgtitle('Arm Position References')
%     subplot(2,ceil(nTraj/2),i), plot(time,180/pi*qRef), grid, hold on, xlim([time(1) time(end)]), title(append('Trajectory ',int2str(i))), legend('qL1', 'qL2', 'qL3', 'qL4', 'qR1', 'qR2', 'qR3', 'qR4')
%     figure(4), sgtitle('Arm Position')
%     subplot(2,ceil(nTraj/2),i), plot(time,180/pi*q), grid, hold on, xlim([time(1) time(end)]), title(append('Trajectory ',int2str(i))), legend('qL1', 'qL2', 'qL3', 'qL4', 'qR1', 'qR2', 'qR3', 'qR4')
%     figure(5), sgtitle('Arm Velocity')
%     subplot(2,ceil(nTraj/2),i), plot(time,180/pi*qd), grid, hold on, xlim([time(1) time(end)]), title(append('Trajectory ',int2str(i))), legend('qdL1', 'qdL2', 'qdL3', 'qdL4', 'qdR1', 'qdR2', 'qdR3', 'qdR4')

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

%% Load Simulation Data

simDataVid1X = load('simDataVid1NatRespX.txt');
simDataVid1Y = load('simDataVid1NatRespY.txt');
simDataVid1Z = load('simDataVid1NatRespZ.txt');
simDataVid2X = load('simDataVid2NatRespX.txt');
simDataVid2Y = load('simDataVid2NatRespY.txt');
simDataVid2Z = load('simDataVid2NatRespZ.txt');
simDataVid2XY = load('simDataVid2NatRespXY.txt');

timeSimVid1X = simDataVid1X(2:end,1)-simDataVid1X(1,1);
dualArmPositionSimVid1X = simDataVid1X(2:end,2);
timeSimVid1Y = simDataVid1Y(2:end,1)-simDataVid1Y(1,1);
dualArmPositionSimVid1Y = simDataVid1Y(2:end,3);
timeSimVid1Z = simDataVid1Z(2:end,1)-simDataVid1Z(1,1);
dualArmQuaternionSim = simDataVid1Z(2:end,4:7);
a = quat2eul(dualArmQuaternionSim(:,1:4), 'XYZ');
a(:,1:2) = a(:,1:2) * (-1);
for i = 1:length(a)
     if a(i,1) < 0
         a(i,1) = a(i,1) + pi;
     else
         a(i,1) = a(i,1) - pi;
     end
end
dualArmOrientationSimVid1 = a(:,3);

timeSimVid2X = simDataVid2X(2:end,1)-simDataVid2X(1,1);
dualArmPositionSimVid2X = simDataVid2X(2:end,2);

timeSimVid2Y = simDataVid2Y(2:end,1)-simDataVid2Y(1,1);
dualArmPositionSimVid2Y = simDataVid2Y(2:end,3);

timeSimVid2Z = simDataVid2Z(2:end,1)-simDataVid2Z(1,1);
dualArmQuaternionSim = simDataVid2Z(2:end,4:7);
a = quat2eul(dualArmQuaternionSim(:,1:4), 'XYZ');
a(:,1:2) = a(:,1:2) * (-1);
for i = 1:length(a)
     if a(i,1) < 0
         a(i,1) = a(i,1) + pi;
     else
         a(i,1) = a(i,1) - pi;
     end
end
dualArmOrientationSimVid2 = a(:,3);


timeSimVid2XY = simDataVid2XY(2:end,1)-simDataVid2XY(1,1);
dualArmPositionSimVid2XY = simDataVid2XY(2:end,2:3);

%% Model dynamics

% p = [1.1378 0.3958 2.5000 0.4000 0.4000 0 0 0.0910 0.1220 0.5232 1.0000 2.2614]; % MIGLIORE
% p = [1.1084    0.4000    2.8000    0.4200    0.3007    0.0001    0.0005    0.0920    0.1220    0.5232    0.9000    2.1764];
% p = [1.1105    0.4000    2.8000    0.4200    0.3072    0.0001    0.0005    0.0970    0.1208    0.5232    0.9000    0.0146    2.2721];
% p = [1.1197    0.4000    3.0204    0.4200    0.3000    0.0001    0.0005    0.25    3.39e-3 0.1220    0.5232    0.9000    0.0689    2.3838];
% p = [    1.1199    0.4000    3.0216    0.4200    0.3000    0.0001    0.0005    3.39e-3 0.1030    0.1139    0.5232    0.9000    0.0855    2.5000];
% p = [    1.1370    0.4000    3.2491    0.4200    0.3000    0.0001    0.0005    3.39e-3 0.5000    0.1220    0.5232    0.9000    0.7359   10.8046];
% p = [    1.0833    0.4000    2.4000    0.4200    0.3000    0.0001    0.0005    3.39e-3 0.7839    0.5473    0.5109    0.5000    0.9167   16.8052];
% p = [    1.0833    0.4000    2.4000    0.4200    0.3000    0.0001    0.0005    0.7872    3.39e-3    0.2    0.5108    0.5000    0.4686   16.8314];

p = [1.1378   0.3958   2.5000   0.4000   0.4000   0.1   0.1   0.7872   3.39e-3   0.2   0.5232   1.0000   0.4686   16.8314];  % MIGLIORE

% p = [1.1378  0.3958    2.5000    0.0     0.0   0.0001    0.0005    0.7872    3.39e-3     0.2     0.5232    1.0000    0.4686   16.8314]; 
% p = [1.1378     3.5      3.5     0.1     0.1   0.01    0.01    0.01    3.39e-3     0.2     0.2    0.2    0.4686   16.8314];
% p = [0.9485  0.5645    1.2588    0.1000    0.1000         0         0    0.0259    0.0500    0.3159    0.5011    0.9221];
% p = [0.7000       0         0    0.6833    0.0827    0.1275    0.4684         0    0.0213    0.2235    0.8262    0.3758];
% p = [0.9500  1.8747    0.5456    0.8282    0.3000         0    0.0120    0.0844    0.1397         0    0.2530    2.0746];
% p = [1.0265  1.8339    0.5893    0.1506    0.1592    0.0647    0.1092    0.3672    0.4725    1.6513];

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
j=1;  i=4;
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

%% Y EXPERIMENT #1
j=2;  i=2;
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);

plot(tModel{i}, shoulderPoseRealVecInterp{i}(:,j), 'k--', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
hold on
plot(tModel{i}, shoulderPoseModel{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]);
legend('Real','Model'); xlabel('t [s]'); ylabel('x [m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
grid on; box on; set(gcf,'color','w');
exportgraphics(h, 'LicasY1.pdf');

% Error
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);
plot(tModel{i},abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j)), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
xlabel('t [s]'); ylabel('$\mid e \mid$ [m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
grid on; box on; set(gcf,'color','w');
exportgraphics(h, 'ErrorLicasY1.pdf');  

%% Y EXPERIMENT #2
j=2;  i=5;
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);

plot(tModel{i}, shoulderPoseRealVecInterp{i}(:,j), 'k--', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
hold on
plot(tModel{i}, shoulderPoseModel{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]);
legend('Real','Model'); xlabel('t [s]'); ylabel('x [m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
grid on; box on; set(gcf,'color','w');
exportgraphics(h, 'LicasY2.pdf');

% Error
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);
plot(tModel{i},abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j)), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
xlabel('t [s]'); ylabel('$\mid e \mid$ [m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
grid on; box on; set(gcf,'color','w');
exportgraphics(h, 'ErrorLicasY2.pdf');  

%% Z EXPERIMENT #1
j=3;  i=3;
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);

plot(tModel{i}, shoulderPoseRealVecInterp{i}(:,j), 'k--', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
hold on
plot(tModel{i}, shoulderPoseModel{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]);
legend('Real','Model'); xlabel('t [s]'); ylabel('x [m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
grid on; box on; set(gcf,'color','w');
exportgraphics(h, 'LicasZ1.pdf');

% Error
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);
plot(tModel{i},abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j)), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
xlabel('t [s]'); ylabel('$\mid e \mid$ [m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
grid on; box on; set(gcf,'color','w');
exportgraphics(h, 'ErrorLicasZ1.pdf');  

%% Z EXPERIMENT #2
j=3;  i=6;
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);

plot(tModel{i}, shoulderPoseRealVecInterp{i}(:,j), 'k--', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
hold on
plot(tModel{i}, shoulderPoseModel{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]);
legend('Real','Model'); xlabel('t [s]'); ylabel('x [m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
grid on; box on; set(gcf,'color','w');
exportgraphics(h, 'LicasZ2.pdf');

% Error
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);
plot(tModel{i},abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j)), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
xlabel('t [s]'); ylabel('$\mid e \mid$ [m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
grid on; box on; set(gcf,'color','w');
exportgraphics(h, 'ErrorLicasZ2.pdf');  

% %% PLOT X7
% j=3;  i=7;
% h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);
% 
% plot(tModel{i}, shoulderPoseRealVecInterp{i}(:,j), 'k--', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
% hold on
% plot(tModel{i}, shoulderPoseModel{i}(:,j), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]);
% legend('Real','Model'); xlabel('t [s]'); ylabel('x [m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
% grid on; box on; set(gcf,'color','w');
% exportgraphics(h, 'Licas_X7.pdf');
% 
% %% ERROR X7
% h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);
% plot(tModel{i},abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j)), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
% xlabel('t [s]'); ylabel('e [m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) tModel{i}(end)]);
% grid on; box on; set(gcf,'color','w');
% exportgraphics(h, 'Error_Licas_X7.pdf');  


% figure(4)
% 
%     i = 7;
%     subplot(2,2,1);
%     j = 1; plot(tModel{i},shoulderPoseRealVecInterp{i}(:,j)); grid, hold on, xlim([tModel{i}(j) tModel{i}(end)]), xlabel('t [s]'), ylabel(Y{j})
%     j = 1; plot(tModel{i},shoulderPoseModel{i}(:,j))
%     legend('Real X','Model X')
%     subplot(2,2,2);
%     j = 2; plot(tModel{i},shoulderPoseRealVecInterp{i}(:,j)); grid, hold on, xlim([tModel{i}(j) tModel{i}(end)]), xlabel('t [s]'), ylabel(Y{j})
%     j = 2; plot(tModel{i},shoulderPoseModel{i}(:,j))
%     legend('Real Y','Model Y')
%     subplot(2,2,3);
%     j = 1; plot(tModel{i},abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j))), grid, xlabel('t [s]'), ylabel(Y{j})
%     subplot(2,2,4);
%     j = 2; plot(tModel{i},abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j))), grid, xlabel('t [s]'), ylabel(Y{j})




% --------------------------------------------------------------------------------------------
    
    
% % Shoulders horizontality
% figure('WindowState','Maximized')
% sgtitle('Shoulders horizontality')
% for i=1:nTraj                                       % For each trajectory
%     subplot(2,ceil(nTraj/2),i);
%     plot(tModel{i},(qModel{i}(:,4)+qModel{i}(:,2))*180/pi); grid, hold on; xlim([tModel{i}(1) tModel{i}(end)]), xlabel('time [s]'), ylabel('angle [deg]'), title(append('Trajectory ',int2str(i)));
%     plot(tModel{i},(qModel{i}(:,5)+qModel{i}(:,3))*180/pi)
%     legend('Horizontality X','Horizontality Y')
% end

% % Shoulder Pose
% for j=1:3                                               % For each joint
%     figure(j)
%     sgtitle(A{j})
%     for i=1:nTraj                                       % For each trajectory
% 
%         if i<=ceil(nTraj/2)
%             subplot(4,ceil(nTraj/2),i);
%             plot(tModel{i},shoulderPoseRealVecInterp{i}(:,j)); grid, hold on, xlim([tModel{i}(1) tModel{i}(end)]), xlabel('time [s]'), ylabel(Y{j}), title(append('Trajectory ',int2str(i)));
% %             plot(tModel{i},shoulderPoseRealVecInterp{i}(:,j)); grid, hold on; xlim([tModel{i}(1) tModel{i}(end)]), ylim([YlimL(j) YlimU(j)]), xlabel('time [s]'), ylabel(Y{j}), title(append('Trajectory ',int2str(i)));
%             plot(tModel{i},shoulderPoseModel{i}(:,j))
%             legend('Real','Model')
%             
%             subplot(4,ceil(nTraj/2),i+ceil(nTraj/2));
%             plot(tModel{i},abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j))), grid, xlabel('time [s]'), ylabel(Y{j}), title(append('Error - Trajectory ',int2str(i)));     
% %             plot(tModel{i},abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j))), grid, ylim([0 YErrlimU(j)]), xlabel('time [s]'), ylabel(Y{j}), title(append('Error - Trajectory ',int2str(i)));     
%         elseif i>ceil(nTraj/2)
%             subplot(4,ceil(nTraj/2),i+ceil(nTraj/2));
%             plot(tModel{i},shoulderPoseRealVecInterp{i}(:,j)); grid, hold on; xlim([tModel{i}(1) tModel{i}(end)]), xlabel('time [s]'), ylabel(Y{j}), title(append('Trajectory ',int2str(i)));
% %             plot(tModel{i},shoulderPoseRealVecInterp{i}(:,j)); grid, hold on; xlim([tModel{i}(1) tModel{i}(end)]), ylim([YlimL(j) YlimU(j)]), xlabel('time [s]'), ylabel(Y{j}), title(append('Trajectory ',int2str(i)));
%             plot(tModel{i},shoulderPoseModel{i}(:,j))
%             legend('Real','Model')
%             
%             subplot(4,ceil(nTraj/2),i+ceil(nTraj)+1);
%             plot(tModel{i},abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j))), grid, xlabel('time [s]'), ylabel(Y{j}), title(append('Error - Trajectory ',int2str(i)));     
% %             plot(tModel{i},abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j))), grid, ylim([0 YErrlimU(j)]), xlabel('time [s]'), ylabel(Y{j}), title(append('Error - Trajectory ',int2str(i)));     
%         end
%     end
% end
% 
% figure(1)
% subplot(4,ceil(nTraj/2),1); plot(timeSimVid1X,dualArmPositionSimVid1X,'--k'); legend('Real','Model','Gazebo')
% subplot(4,ceil(nTraj/2),4); plot(timeSimVid2X,dualArmPositionSimVid2X,'--k'); legend('Real','Model','Gazebo')
% subplot(4,ceil(nTraj/2),11); plot(timeSimVid2XY,dualArmPositionSimVid2XY(:,1),'--k'); legend('Real','Model','Gazebo')
% 
% figure(2)
% subplot(4,ceil(nTraj/2),2); plot(timeSimVid1Y,dualArmPositionSimVid1Y,'--k'); legend('Real','Model','Gazebo')
% subplot(4,ceil(nTraj/2),9); plot(timeSimVid2Y,dualArmPositionSimVid2Y,'--k'); legend('Real','Model','Gazebo')
% subplot(4,ceil(nTraj/2),11); plot(timeSimVid2XY,dualArmPositionSimVid2XY(:,2),'--k'); legend('Real','Model','Gazebo')
% 
% figure(3)
% subplot(4,ceil(nTraj/2),3); plot(timeSimVid1Z,dualArmOrientationSimVid1,'--k'); legend('Real','Model','Gazebo')
% subplot(4,ceil(nTraj/2),10); plot(timeSimVid2Z,dualArmOrientationSimVid2,'--k'); legend('Real','Model','Gazebo')


% % Active Joints
% for j=1:8                                               % For each joint
%     figure('WindowState','Maximized')
%     sgtitle(append('Joint ',int2str(j)))
%     for i=1:nTraj                                       % For each trajectory
%         if i<=ceil(nTraj/2)
%             subplot(4,ceil(nTraj/2),i);
%             plot(tModel{i},qRealVecInterp{i}(:,j)*180/pi), grid, hold on, xlim([tModel{i}(1) tModel{i}(end)]), xlabel('time [s]'), ylabel('angle [deg]'), title(append('Trajectory ',int2str(i)));
%             plot(tModel{i},qModel{i}(:,j+5)*180/pi)            
%             plot(tModel{i},qRefVecInterp{i}(:,j)*180/pi,'k--')
%             legend('Real','Model','Command')
%             
%             subplot(4,ceil(nTraj/2),i+ceil(nTraj/2));
%             plot(tModel{i},abs(qRealVecInterp{i}(:,j)-qModel{i}(:,j+5))*180/pi), grid, xlim([tModel{i}(1) tModel{i}(end)]), xlabel('time [s]'), ylabel('angle [deg]'), title(append('Error - Trajectory ',int2str(i)));     
%         elseif i>ceil(nTraj/2)
%             subplot(4,ceil(nTraj/2),i+ceil(nTraj/2));
%             plot(tModel{i},qRefVecInterp{i}(:,j)*180/pi), grid, hold on; xlim([tModel{i}(1) tModel{i}(end)]), xlabel('time [s]'), ylabel('angle [deg]'), title(append('Trajectory ',int2str(i)));
%             plot(tModel{i},qModel{i}(:,j+5)*180/pi)
%             plot(tModel{i},qRefVecInterp{i}(:,j)*180/pi,'k--')
%             legend('Real','Model','Command')
%             
%             subplot(4,ceil(nTraj/2),i+ceil(nTraj));
%             plot(tModel{i},abs(qRealVecInterp{i}(:,j)-qModel{i}(:,j+5))*180/pi), grid, xlim([tModel{i}(1) tModel{i}(end)]), xlabel('time [s]'), ylabel('angle [deg]'), title(append('Error - Trajectory ',int2str(i)));     
%         end
%     end
% end
