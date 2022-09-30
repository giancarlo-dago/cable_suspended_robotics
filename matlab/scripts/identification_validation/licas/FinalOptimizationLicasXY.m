close all
clear
clc

%% Parameters
sampling = 1;
Ts_sim = 0.001;

%% Save and plot data
% TRAJ = [1 7]; nTraj = 2; nSamples = [924 864];
% TRAJ = [1 2 7 8]; nTraj = 4; nSamples = [924 ];

TRAJ = [1 4 7 10]; nTraj = 4; nSamples = [500 500 500 500];

for i=1:nTraj
    realTraj = load(append('../../experimental_data_analysis/licas/experiments_testbed_2022-09-06/finalNatRes',int2str(TRAJ(i)),'.txt'));                  % Retrieve data from file
    t = realTraj(1:nSamples(i),1);
    time = t-t(1);
    meanTs(i) = mean(time(2:end)-time(1:end-1));
    qRef = pi/180*realTraj(1:nSamples(i),2:9);
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
    shoulderYaw = dualArmOrientation(:,3);
    
    shoulderRoll = dualArmOrientation(:,1);
    shoulderPitch = dualArmOrientation(:,2);

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
    figure(1), sgtitle('Shoulder Position');
    subplot(2,ceil(nTraj/2),i), plot(time,[shoulderX shoulderY]), grid, hold on, xlim([time(1) time(end)]), ylim([-0.4 0.4]), xlabel('time [s]'), ylabel('position [m]'), title(append('Trajectory ',int2str(i))), legend('X Position','Y Position')
    figure(2), sgtitle('Shoulder Orientation')
    subplot(2,ceil(nTraj/2),i), plot(time,180/pi*shoulderYaw), grid, hold on, xlim([time(1) time(end)]), ylim([-100 100]), xlabel('time [s]'), ylabel('orientation [deg]'), title(append('Trajectory ',int2str(i)))

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

%% Creare una corrFun e testarla con dei parametri a caso

% p = [1.277 ...
%      2.5 2.5 2.5 0.3195 0.3195 ...
%      0.5639 0.5639 0.5639 ...
%      0.1 0.01 0.01 ...
%      0.04 0.23 0.3 ...
%      0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 ...
%      50 1.5 1.5 20 50 1.5 1.5 20 ...
%      0.58, ...
%      1 1 700 700];
% cost = corrcoefFunLicas(p, qRealVecInterp, shoulderPoseRealVecInterp, q0, qd0, shoulderPoseReal0, qRefVecInterp, qdRefVecInterp, Ts_sim, meanTs, sampling, nSamples, nTraj);

%% Definire il problema di minimizzazione 
pause(3);
fun = @(p) corrcoefFunLicas(p, qRealVecInterp, shoulderPoseRealVecInterp, q0, qd0, shoulderPoseReal0, qRefVecInterp, qdRefVecInterp, Ts_sim, meanTs, sampling, nSamples, nTraj);

% ----------------------------------------------------------------------------------------------------

p0 = [1.0  0.1  0.1  0,       1.0  0.1  0.1  0];
LB = [0 0 0 0,      0 0 0 0];
UB = [5 3 1 2,     10 3 1 2];

%     1.1131    0.4768    0.5998    0.0201    1.0221    0.5082    0.5225    0.0703


% p0 = [1.0  0.1  0, 0.1 0];
% LB = [0 0 0,  0 0];
% UB = [5 3 2,  3 2];

% ----------------------------------------------------------------------------------------------------

tic
options = optimoptions('fmincon','PlotFcn',{@optimplotx,@optimplotfval,@optimplotfirstorderopt},'Display','iter-detailed','Algorithm','active-set','UseParallel',true);          
[p,fval] = fmincon(fun,p0,[],[],[],[],LB,UB,[],options)
toc

%% Model dynamics

% p = p0;
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

% for j=1:3                                               % For each joint
%     figure()
%     sgtitle('Parameters identification')
%     for i=1:nTraj   
%         subplot(2,ceil(nTraj/2),i);
%         plot(tModel,shoulderPoseRealVecInterp{i}(:,j)); grid, hold on; xlim([tModel(1) tModel(end)]), xlabel('time [s]'), ylabel('angle [deg]'), title(append('Joint ',int2str(j),' Angle - Trajectory ',int2str(i)));
%         plot(tModel,shoulderPoseModel{i}(:,j))
%         legend('Real','Model')
%     end
% end

%% Plot

A = {'Shoulder X position','Shoulders Y position','Shoulder Z Orientation'};
Y = {'position [m]', 'position [m]', 'angle [rad]'};
YlimL = [-0.4 -0.3 -1.5];
YlimU = [0.35 0.3 1.7];
YErrlimU = [0.15 0.1 0.6];

% Shoulder Pose
for j=1:3                                               % For each joint
    figure()
    sgtitle(A{j})
    for i=1:nTraj                                       % For each trajectory

        if i<=ceil(nTraj/2)
            subplot(4,ceil(nTraj/2),i);
            plot(tModel{i},shoulderPoseRealVecInterp{i}(:,j)); grid, hold on, xlim([tModel{i}(1) tModel{i}(end)]), xlabel('time [s]'), ylabel(Y{j}), title(append('Trajectory ',int2str(i)));
%             plot(tModel{i},shoulderPoseRealVecInterp{i}(:,j)); grid, hold on; xlim([tModel{i}(1) tModel{i}(end)]), ylim([YlimL(j) YlimU(j)]), xlabel('time [s]'), ylabel(Y{j}), title(append('Trajectory ',int2str(i)));
            plot(tModel{i},shoulderPoseModel{i}(:,j))
            legend('Real','Model')
            
            subplot(4,ceil(nTraj/2),i+ceil(nTraj/2));
            plot(tModel{i},abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j))), grid, xlabel('time [s]'), ylabel(Y{j}), title(append('Error - Trajectory ',int2str(i)));     
%             plot(tModel{i},abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j))), grid, ylim([0 YErrlimU(j)]), xlabel('time [s]'), ylabel(Y{j}), title(append('Error - Trajectory ',int2str(i)));     
        elseif i>ceil(nTraj/2)
            subplot(4,ceil(nTraj/2),i+ceil(nTraj/2));
            plot(tModel{i},shoulderPoseRealVecInterp{i}(:,j)); grid, hold on; xlim([tModel{i}(1) tModel{i}(end)]), xlabel('time [s]'), ylabel(Y{j}), title(append('Trajectory ',int2str(i)));
%             plot(tModel{i},shoulderPoseRealVecInterp{i}(:,j)); grid, hold on; xlim([tModel{i}(1) tModel{i}(end)]), ylim([YlimL(j) YlimU(j)]), xlabel('time [s]'), ylabel(Y{j}), title(append('Trajectory ',int2str(i)));
            plot(tModel{i},shoulderPoseModel{i}(:,j))
            legend('Real','Model')
            
            subplot(4,ceil(nTraj/2),i+ceil(nTraj));
            plot(tModel{i},abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j))), grid, xlabel('time [s]'), ylabel(Y{j}), title(append('Error - Trajectory ',int2str(i)));     
%             plot(tModel{i},abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j))), grid, ylim([0 YErrlimU(j)]), xlabel('time [s]'), ylabel(Y{j}), title(append('Error - Trajectory ',int2str(i)));     
        end
    end
end

