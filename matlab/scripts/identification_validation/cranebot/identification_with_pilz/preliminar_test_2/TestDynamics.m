close all
clear
clc

addpath("/home/giancarlo/ros_ws/src/cable_suspended_robotics/matlab/functions/trajectory_generation_functions/")

%% Parameters
sampling = 1;
Ts_sim = 0.001;

%%
TRAJ = {'ModNatResExpY2'};
nTraj = length(TRAJ);

for i=1:nTraj
    realTraj = load(append(TRAJ{i},'.txt'));
%     nSamples(i) = length(realTraj);
    nSamples(i) = 200;
    time = realTraj(1:nSamples(i),1);           %     time = t-t(1);
    meanTs(i) = mean(time(2:end)-time(1:end-1));
    platformPosition = realTraj(1:nSamples(i),2:3);
    platformOrientation = realTraj(1:nSamples(i),4);
    
    platformX = platformPosition(:,1);
    platformY = platformPosition(:,2);
    firstPassiveAngleZ = platformOrientation(:,1);
    timeRealVec{i} = time;
    platformPoseRealVec{i} = [platformX platformY firstPassiveAngleZ];

end

%% Interpolare comandi e dati
for i=1:nTraj
    tModel{i} = (0:Ts_sim*sampling:meanTs(i)*nSamples(i))';
    platformPoseRealVecInterp{i} = interp1(timeRealVec{i},platformPoseRealVec{i},tModel{i},'spline');
end

%% Eliminare eventuali NaN
for i=1:nTraj
    TF = isnan(platformPoseRealVecInterp{i});
    platformPoseRealVecInterp{i}(TF) = 0;
end

%% Definire le starting conditions
for i=1:nTraj
    qActive0{i} = [0 0 0 0 0 0 0 0 0 0 0 0]';
    qdActive0{i} = [0 0 0 0 0 0 0 0 0 0 0 0]';
    platformPoseReal0{i} = platformPoseRealVec{i}(1,:)';
end

for i=1:nTraj
    qRef{i} = ones(length(tModel{i}),1)*[0 0 0 0 0 0 0 0 0 0 0 0];
    qdRef{i} = ones(length(tModel{i}),1)*[0 0 0 0 0 0 0 0 0 0 0 0];
end

%% Model dynamics

p = [3.286 ...
      36 ...
      2.9 ...
      35.64 ...
      30 50];

L = p(1);
for i=1:nTraj
    disp(append('Simulating dynamics for trajectory ',int2str(i)));
    
    qModel(1,1) = asin(platformPoseReal0{i}(2)/L)                          % Initialize the cable joint from the starting condition of the shoulder pose
    qModel(1,2) = 0;                          % Initialize the cable joint from the starting condition of the shoulder pose
    qModel(1,3:14) = reshape(qActive0{i},1,12);                         % Initialize qModel with the starting condition of the active joints
    qdModel(1,1:14) = zeros(1,14);                                      % Initialize qdModel with zero velocity for all the joints

    qRef_i = qRef{i};                                                   % Define qRef for the i-th trajectory
    qdRef_i = qdRef{i};                                                 % Define qdRef_i for the i-th trajectory

    WB = waitbar(0);
    for k=2:(length(tModel{i}))                                                                                                % For each sample
        [qModel(k,:), qdModel(k,:)] = qFunCranebot(p,qModel(k-1,:),qdModel(k-1,:),Ts_sim,qRef_i(k-1,:),qdRef_i(k-1,:));        % Compute the dynamics iteratively
        waitbar(k/(length(tModel{i})), WB, append('Dynamic model computation',' - ',num2str(k/(length(tModel{i}))*100),'%') );
    end                                                                                                                        % End-for
    
    qModelVec{i} = qModel;
    platformPoseModel{i} = L*sin(qModel(:,1));
%         platformPoseModel = [zeros(length(qModel(:,1)),1) L*sin(qModel(:,1)) L*sin(qModel(:,2))];                                                    % Compute the position of the shoulders again

end

%% Plot

% Platform position comparison
figure()
sgtitle('Platform position')
plot(tModel{1},qModelVec{1}), hold on
% plot(time,platformPoseRealVec{1}(:,2)), grid, hold on, xlim([time(1) time(end)]),  xlabel('time [s]'), ylabel('position [m]'), title(append('Trajectory ',int2str(i))), legend('X Position','Y Position','Yaw')
