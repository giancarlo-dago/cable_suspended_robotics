close all
clear
clc

%% Parameters
sampling = 1;
Ts_sim = 0.001;

%%
% TRAJ = [1 2 4 6 7]; nTraj = 5; nSamples = [14070 13760 13411 60790 79473];            % natRes
% TRAJ = [4 7]; nTraj = 2; nSamples = [13411 79473];                                    % natRes
% TRAJ = 4; nTraj = 1; nSamples = 13411;                                                % natRes
% TRAJ = 4; nTraj = 1; nSamples = 2000;                                                 % natRes
TRAJ = 7; nTraj = 1; nSamples = 79473;                                                  % natRes
% TRAJ = 7; nTraj = 1; nSamples = 1000;                                                 % natRes

for i=1:nTraj
    realTraj = load(append('../../../experimental_data_analysis/data_modification/cranebot/dataMod',int2str(TRAJ(i)),'.txt'));
    t = realTraj(1:nSamples(i),1);
    time = t-t(1);
    meanTs(i) = mean(time(2:end)-time(1:end-1));
    platformPosition = realTraj(1:nSamples(i),2:4);
    platformOrientation = realTraj(1:nSamples(i),8:10);
    
    platformX = platformPosition(:,1);
    platformY = platformPosition(:,2);
    firstPassiveAngleZ = platformOrientation(:,1);
    firstPassiveAngleY = platformOrientation(:,2);
    firstPassiveAngleX = platformOrientation(:,3);
    timeRealVec{i} = time;
    platformPoseRealVec{i} = [platformX platformY];
    firstPassiveAngleVec{i} = [firstPassiveAngleX firstPassiveAngleY firstPassiveAngleZ];
    if TRAJ(i)==4
        qRealVec{i} = [pi/180*firstPassiveAngleX zeros(nSamples,1) pi/2*ones(nSamples,1) zeros(nSamples,5) pi/2*ones(nSamples,1) zeros(nSamples,5)];
    elseif TRAJ(i)==7
        qRealVec{i} = [pi/180*firstPassiveAngleY zeros(nSamples,1) pi/2*ones(nSamples,1) zeros(nSamples,5) pi/2*ones(nSamples,1) zeros(nSamples,5)];
    end

    % Plot
%     figure(1), sgtitle('Platform Position');
%     subplot(2,ceil(nTraj/2),i), plot(time,platformPoseRealVec{i}), grid, hold on, xlim([time(1) time(end)]),  xlabel('time [s]'), ylabel('position [m]'), title(append('Trajectory ',int2str(i))), legend('X Position','Y Position')
%     figure(2), sgtitle('Platform Orientation')
%     subplot(2,ceil(nTraj/2),i), plot(time,firstPassiveAngleZ), grid, hold on, xlim([time(1) time(end)]), xlabel('time [s]'), ylabel('orientation [deg]'), title(append('Trajectory ',int2str(i)))
%     figure(3), sgtitle('First Passive Joint X')
%     subplot(2,ceil(nTraj/2),i), plot(time,[-firstPassiveAngleX 180/pi*asin(platformX/4.3330)]), grid, hold on, xlim([time(1) time(end)]), xlabel('time [s]'), ylabel('angle [deg]'), title(append('Trajectory ',int2str(i))), legend('Measured','Reconstructed')
%     figure(4), sgtitle('First Passive Joint Y')
%     subplot(2,ceil(nTraj/2),i), plot(time,[-firstPassiveAngleY 180/pi*asin(platformY/4.3330)]), grid, hold on, xlim([time(1) time(end)]), xlabel('time [s]'), ylabel('angle [deg]'), title(append('Trajectory ',int2str(i))), legend('Measured','Reconstructed')
    figure(5), sgtitle('Joints')
    subplot(2,ceil(nTraj/2),i), plot(time,qRealVec{i}), grid, hold on, xlim([time(1) time(end)]), xlabel('time [s]'), ylabel('angle [rad]'), title(append('Trajectory ',int2str(i))), legend
end

%% Interpolare comandi e dati
for i=1:nTraj
    tModel{i} = (0:Ts_sim*sampling:meanTs(i)*nSamples(i))';
    qRealVecInterp{i} = interp1(timeRealVec{i},qRealVec{i},tModel{i},'spline');
    platformPoseRealVecInterp{i} = interp1(timeRealVec{i},platformPoseRealVec{i},tModel{i},'spline');
end

%% Eliminare eventuali NaN
for i=1:nTraj
    TF = isnan(platformPoseRealVecInterp{i});
    platformPoseRealVecInterp{i}(TF) = 0;
    TF = isnan(qRealVecInterp{i});
    qRealVecInterp{i}(TF) = 0;
end

%% Definire le starting conditions
for i=1:nTraj
%     qPassive0{i} = [qRealVec{i}(1,1) deg2rad(6)]';
%     qPassive0{i} = [deg2rad(-4.4) deg2rad(-1.6)]';
    qPassive0{i} = [qRealVec{i}(1,1) deg2rad(0)]';
    qActive0{i} = [pi/2 0 0 0 0 0 -pi/2 0 0 0 0 0]';
    qdActive0{i} = [0 0 0 0 0 0 0 0 0 0 0 0]';
    platformPoseReal0{i} = platformPoseRealVec{i}(1,:)';
end

%% Definire i riferimenti per le braccia
for i=1:nTraj
    qRef{i} = ones(length(tModel{i}),1)*[pi/2 0 0 0 0 0 -pi/2 0 0 0 0 0];
    qdRef{i} = ones(length(tModel{i}),1)*[0 0 0 0 0 0 0 0 0 0 0 0];
end

%% Creare una corrFun e testarla con dei parametri a caso

% L = 1;
% m_cables = 1;
% l_cables = 1;
% ixx_cables = 1;
% fv1p = 1;
% fv2p = 1;
% 
% p = [L,m_cables,l_cables,ixx_cables,fv1p,fv2p];

% p = [3.286 ...
%       10 ...
%       1.5 ...
%       0.01 ...
%       30 100];
% 
% p = [3.286 ...           % TROLLEY
%       36 ...
%       2.9 ...
%       35.64 ...
%       30 0];
% cost = corrcoefFunCranebot(p, qRealVecInterp, platformPoseRealVecInterp, qActive0, qdActive0, qPassive0, platformPoseReal0, qRef, qdRef, Ts_sim, meanTs, sampling, nSamples, nTraj)

%% Definire il problema di minimizzazione 
fun = @(p) corrcoefFunCranebot(p, qRealVecInterp, platformPoseRealVecInterp, qActive0, qdActive0, qPassive0, platformPoseReal0, qRef, qdRef, Ts_sim, meanTs, sampling, nSamples, nTraj);

% ----------------------------------------------------------------------------------------------------

% L
% m_cables
% l_cables l_platform
% ixx_cables
% fv1p fv2p
% 
% p0 = [3.286 ...
%       10 ...
%       1.5 ...
%       0.01 ...
%       30 100];
% 
% LB = [1 ...
%       0.3 ...
%       0 ...
%       0.001 ...
%       0 0];
% 
% UB = [5 ...
%       20 ...
%       3 ...
%       10 ...
%       500 500];

%----------------------------------------

p0 = [3.286 ...           % TROLLEY
      36 ...
      2.9 ...
      35.64 ...
      30 100];

LB = [3.286 ...
      10 ...
      1 ...
      20 ...
      0 0];

UB = [3.286 ...
      50 ...
      3 ...
      50 ...
      500 500];
  
%----------------------------------------

% p0 = [3.286 ...             % BRIDGE
%       36 ...
%       2.9 0.45 ...
%       35.64 ...
%       102 0];
% 
% LB = [3.286 ...
%       10 ...
%       1.5 0.35 ...
%       20 ...
%       0 0];
%   
% UB = [3.286 ...
%       50 ...
%       3 0.55 ...
%       50 ...
%       500 0];
 
% ----------------------------------------------------------------------------------------------------

tic
options = optimoptions('fmincon','PlotFcn',{@optimplotx,@optimplotfval,@optimplotfirstorderopt},'Display','iter-detailed','Algorithm','active-set','UseParallel',true);          
[p,fval] = fmincon(fun,p0,[],[],[],[],LB,UB,[],options)
toc

%% Model dynamics

% p = [3.2377 9.4888 0 0.0628 36.1293 100.0032];  % TROLLEY
% p = [3.2860 29.4504 1.5000 0.0628 36.1293 100.0032];  % TROLLEY
% p = [3.0573 4.3369 0 1.8558 129.7731      0];  % BRIDGE
% p = [3.286 36 2.9 0.45 35.64 102 0];                 % BRIDGE
% p = [3.2860  29.4504  1.5000  0.3754  33.6430  142.0760  0]; % BRIDGE


L = p(1);
for i=1:nTraj
    disp(append('Simulating dynamics for trajectory ',int2str(i)));
    
    qModel(1,1:2) =  reshape(qPassive0{i},1,2);                         % Initialize the passive joint
    qModel(1,3:14) = reshape(qActive0{i},1,12);                         % Initialize qModel with the starting condition of the active joints
    qdModel(1,1:2) = zeros(1,2);                                        % Initialize qdModel with zero velocity for the passive joints
    qdModel(1,3:14) = reshape(qdActive0{i},1,12);                       % Initialize qdModel with the starting condition of the active joints

    qRef_i = qRef{i};                                                   % Define qRef for the i-th trajectory
    qdRef_i = qdRef{i};                                                 % Define qdRef_i for the i-th trajectory

    WB = waitbar(0);
    for k=2:(length(tModel{i}))                                                                                                % For each sample
        [qModel(k,:), qdModel(k,:)] = qFunCranebot(p,qModel(k-1,:),qdModel(k-1,:),Ts_sim,qRef_i(k-1,:),qdRef_i(k-1,:));        % Compute the dynamics iteratively
        waitbar(k/(length(tModel{i})), WB, append('Dynamic model computation',' - ',num2str(k/(length(tModel{i}))*100),'%') );
    end                                                                                                                        % End-for
    
    qModelVec{i} = qModel;
    platformPoseModel{i} = L*sin(qModel(:,1)) + 1*sin(qModel(:,2));

end

%% Plot

% Platform position comparison
figure()
sgtitle('Platform position')
plot(tModel{1},L*sin(qModelVec{1}(:,1))+1*sin(qModelVec{1}(:,1)+qModelVec{1}(:,2)))

% Second passive joint
figure()
sgtitle('Second passive joint')
plot(tModel{1},qModelVec{1}(:,2))

% Platform position comparison
figure()
sgtitle('Platform position')
for i=1:nTraj                                       % For each trajectory
    subplot(2,ceil(nTraj/2),i);
    plot(tModel{i},-platformPoseRealVecInterp{i}(:,2)); grid, hold on, xlim([tModel{i}(1) tModel{i}(end)]), xlabel('time [s]'), ylabel('position [m]'), title(append('Trajectory ',int2str(i)));
    plot(tModel{1},L*sin(qModelVec{1}(:,1))+0.8*sin(qModelVec{1}(:,1)+qModelVec{1}(:,2)))
    legend('Real','Model')
    subplot(2,ceil(nTraj/2),i+ceil(nTraj/2));
    plot(tModel{i},abs(-platformPoseRealVecInterp{i}(:,2) - (L*sin(qModelVec{1}(:,1))+1*sin(qModelVec{1}(:,1)+qModelVec{1}(:,2))))), grid, xlabel('time [s]'), ylabel('position [m]'), title(append('Error - Trajectory ',int2str(i)));     
end

% First Joint Identification
for j=1:1                                           % For each joint
    figure()
    sgtitle('First Passive Joint')
    for i=1:nTraj                                       % For each trajectory
          subplot(2,ceil(nTraj/2),i);
          plot(tModel{i},180/pi*qRealVecInterp{i}(:,j)); grid, hold on, xlim([tModel{i}(1) tModel{i}(end)]), xlabel('time [s]'), ylabel('angle [deg]'), title(append('Trajectory ',int2str(i)));
          plot(tModel{i},180/pi*qModelVec{i}(:,j))
          legend('Real','Model')
          subplot(2,ceil(nTraj/2),i+ceil(nTraj/2));
          plot(tModel{i},180/pi*abs(qRealVecInterp{i}(:,j) - qModelVec{i}(:,j))), grid, xlabel('time [s]'), ylabel('angle [deg]'), title(append('Error - Trajectory ',int2str(i)));     
    end
end

%% FFT simulation
LL = length(qModelVec{i}(:,1));
Fs = 1/0.001;

Y = fft(qModelVec{i}(:,1));
P2 = abs(Y/LL);
P1 = P2(1:LL/322+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(LL/322))/LL;

figure();
plot(f,P1); grid; hold on;
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f [Hz]')
ylabel('|P(f)|')
xlim([f(1) f(end)])

%% FFT measurements
LL = length(qRealVecInterp{i}(:,1));
Fs = 1/0.001;

Y = fft(qRealVecInterp{i}(:,1));
P2 = abs(Y/LL);
P1 = P2(1:LL/322+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(LL/322))/LL;

plot(f,P1)
xlabel('f [Hz]')
ylabel('|P(f)|')
xlim([f(1) f(end)])

legend('sim','meas')

% % Joints Plot
% for j=1:14                                           % For each joint
%     figure()
%     sgtitle('Model Joints')
%     for i=1:nTraj                                       % For each trajectory
%           subplot(2,ceil(nTraj/2),i);
%           plot(tModel{i},180/pi*qRealVecInterp{i}(:,j)); grid, hold on, xlim([tModel{i}(1) tModel{i}(end)]), xlabel('time [s]'), ylabel('angle [deg]'), title(append('Trajectory ',int2str(i)));
%           plot(tModel{i},180/pi*qModelVec{i}(:,j))
%           legend('Real','Model')
%           subplot(2,ceil(nTraj/2),i+ceil(nTraj/2));
%           plot(tModel{i},180/pi*abs(qRealVecInterp{i}(:,j) - qModelVec{i}(:,j))), grid, xlabel('time [s]'), ylabel('angle [deg]'), title(append('Error - Trajectory ',int2str(i)));     
%     end
% end

