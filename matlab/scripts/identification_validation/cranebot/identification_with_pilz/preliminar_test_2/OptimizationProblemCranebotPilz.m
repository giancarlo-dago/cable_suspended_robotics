close all
clear
clc

addpath("/home/giancarlo/ros_ws/src/cable_suspended_robotics/matlab/functions/trajectory_generation_functions/")

%% Parameters
sampling = 1;
Ts_sim = 0.001;

%%
% TRAJ = {'ModNatResExpX1', 'ModNatResExpX2', 'ModNatResExpX3', ... 
%         'ModNatResExpY1', 'ModNatResExpY2', 'ModNatResExpY3', ... 
%         'ModNatResExpZ1', 'ModNatResExpZ2', 'ModNatResExpZ3'};

% TRAJ = {'ModNatResExpY1', 'ModNatResExpY2', 'ModNatResExpY3'};
TRAJ = {'ModNatResExpY1','ModNatResExpY2'};
nTraj = length(TRAJ);
figure, 
for i=1:nTraj
    realTraj = load(append(TRAJ{i},'.txt'));
%     nSamples(i) = length(realTraj);
    nSamples(i) = 500;
    time = realTraj(1:nSamples(i),1);           %     time = t-t(1);
    meanTs(i) = mean(time(2:end)-time(1:end-1));
    platformPosition = realTraj(1:nSamples(i),2:3);
    platformOrientation = realTraj(1:nSamples(i),4);
    
    platformX = platformPosition(:,1);
    platformY = platformPosition(:,2);
    firstPassiveAngleZ = platformOrientation(:,1);
    timeRealVec{i} = time;
    platformPoseRealVec{i} = [platformX platformY firstPassiveAngleZ];

%     plot(timeRealVec{i},platformPoseRealVec{i}), hold on

end
  
% for traj=1:1
% 
%     Fs = round(1/meanTs(traj));
%     Y = fft(platformPoseRealVec{traj}(:,2));
%     l = length(platformPoseRealVec{traj}(:,2));
%     P2 = abs(Y/l);
%     P1 = P2(1:l/2+1);
%     P1(2:end-1) = 2*P1(2:end-1);
%     f = Fs*(0:(l/2))/l;
% 
%     MX = max(P1);
%     k = find(P1>MX-0.001);
%     T = 1/f(k)
% 
%     figure
%     plot(f,P1)
%     title("Single-Sided Amplitude Spectrum of S(t)")
%     xlabel("f (Hz)")
%     ylabel("|P1(f)|")
% end

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

%% Definire i riferimenti per le braccia

% dt = 0.001;
% amp = 0.05;
% tTrajectories = [2 2 2 2 2 2 2 2 2 2 2]';
% sViaPoints = [0 0 0 0 0 0;
%               0 amp 0 0 0 0; 
%               0 -amp 0 0 0 0;
%               0 amp 0 0 0 0;
%               0 -amp 0 0 0 0;
%               0 amp 0 0 0 0;
%               0 -amp 0 0 0 0;
%               0 amp 0 0 0 0;
%               0 -amp 0 0 0 0;
%               0 amp 0 0 0 0;
%               0 -amp 0 0 0 0;
%               0 0 0 0 0 0];
% sDotViaPoints = zeros(12,6);
% [qRef, qdRef, qddRef, t] = concatenatedMultiJointCubicTraj(dt, tTrajectories, sViaPoints, sDotViaPoints);
% 
% figure()
% for i=1:6
%     
%     subplot 311
%     plot(t{i},qRef{i}), legend, grid on, hold on
%     subplot 312
%     plot(t{i},qdRef{i}), legend, grid on, hold on
%     subplot 313
%     plot(t{i},qddRef{i}), legend, grid on, hold on
%     
% end

for i=1:nTraj
    qRef{i} = ones(length(tModel{i}),1)*[0 0 0 0 0 0 0 0 0 0 0 0];
    qdRef{i} = ones(length(tModel{i}),1)*[0 0 0 0 0 0 0 0 0 0 0 0];
end

%% Creare una corrFun e testarla con dei parametri a caso

% p = [L, massCablesPulleys, lCablesPulleys, ixxCablesPulleys, iyyCablesPulleys, izzCablesPulleys,...
%         fricCablesJz, fricCablesJx, fricCablesJy, fricPlatformJz];

% L, massCablesPulleys, lCablesPulleys, ixxCablesPulleys, ...
%         fricCablesJx

% p = [2  ...
%      10 ...
%      1  ...
%      0.1 0.1 0.1 ...
%      10 10 10 10];

% p = [2.5  ...
%      36 ...
%      2  ...
%      0.1 ...
%      10];
% 
% cost = corrcoefFunCranebotPilz(p, platformPoseRealVecInterp, qActive0, qdActive0, platformPoseReal0, qRef, qdRef, Ts_sim, meanTs, sampling, nSamples, nTraj)

%% Definire il problema di minimizzazione 
pause(3);
% p = [3.286 36 param(1) 35.64 param 0];
fun = @(param) corrcoefFunCranebotPilz(param, platformPoseRealVecInterp, qActive0, qdActive0, platformPoseReal0, qRef, qdRef, Ts_sim, meanTs, sampling, nSamples, nTraj);

% ----------------------------------------------------------------------------------------------------

% p0 = [3.286 ...
%       36 ...
%       1 ...
%       35.64 ...
%       300 0];
% LB = [3.286 10 0, 10 , 200 0];
% UB = [3.286 50 3.286,  50, 500 50];

param0 = [1 300];
LB = [0 100];
UB = [3.286 500];
% ----------------------------------------------------------------------------------------------------

tic
options = optimoptions('fmincon','PlotFcn',{@optimplotx,@optimplotfval,@optimplotfirstorderopt},'Display','iter-detailed','Algorithm','active-set','UseParallel',true);          
[param,fval] = fmincon(fun,param0,[],[],[],[],LB,UB,[],options)
toc

%% Model dynamics

% p = [3.286 ...
%       36 ...
%       1 ...
%       35.64 ...
%       300 0];

    p = [3.286 36 param(1) 35.64 param(2) 0];


L = p(1);
f = 0.414;
for i=1:nTraj
    disp(append('Simulating dynamics for trajectory ',int2str(i)));

    qModel = zeros(length(tModel),13);                                  % Define vector for the position
        qdModel = zeros(length(tModel),13);                                 % Define vector for the velocity
        qModel(1,1) = asin(platformPoseReal0{i}(2)/(L+f));                          % Initialize the cable joint from the starting condition of the shoulder pose
        qModel(1,2:13) = reshape(qActive0{i},1,12);                         % Initialize qModel with the starting condition of the active joints
        qdModel(1,1:13) = zeros(1,13);                                      % Initialize qdModel with zero velocity for all the joints

        qRef_i = qRef{i};                                                   % Define qRef for the i-th trajectory
        qdRef_i = qdRef{i};                                                 % Define qdRef_i for the i-th trajectory

    WB = waitbar(0);
    for k=2:(length(tModel{i}))                                                                                                % For each sample
        [qModel(k,:), qdModel(k,:)] = qFunCranebotPilz(p,qModel(k-1,:),qdModel(k-1,:),Ts_sim,qRef_i(k-1,:),qdRef_i(k-1,:));        % Compute the dynamics iteratively
        waitbar(k/(length(tModel{i})), WB, append('Dynamic model computation',' - ',num2str(k/(length(tModel{i}))*100),'%') );
    end                                                                                                                        % End-for
    
    qModelVec{i} = qModel;
    platformPoseModel{i} = (L+f)*sin(qModel(:,1));
end

%% Plot

% Platform position comparison
figure()
sgtitle('Platform position')
plot(tModel{1},platformPoseModel{1}), hold on
plot(time,platformPoseRealVec{1}(:,2)), grid, hold on, xlim([time(1) time(end)]),  xlabel('time [s]'), ylabel('position [m]'), title(append('Trajectory ',int2str(i))), legend('Model','Measured')


figure()
sgtitle('Platform position')
plot(tModel{2},platformPoseModel{2}), hold on
plot(time,platformPoseRealVec{2}(:,2)), grid, hold on, xlim([time(1) time(end)]),  xlabel('time [s]'), ylabel('position [m]'), title(append('Trajectory ',int2str(i))), legend('Model','Measured')
