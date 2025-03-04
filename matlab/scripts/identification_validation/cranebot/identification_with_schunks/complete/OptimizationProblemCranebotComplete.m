close all
clear
clc

%% Parameters
sampling = 1;
Ts_sim = 0.001;

%%
% TRAJ = [1 2 4 6 7]; nTraj = 5; nSamples = [14070 13760 13411 60790 79473];            % natRes
% TRAJ = [4 7]; nTraj = 2; nSamples = [13411 79473];                                    % natRes
% TRAJ = [4 7]; nTraj = 2; nSamples = [1000 1000];                                    % natRes
% TRAJ = 7; nTraj = 1; nSamples = 79473;                                                  % natRes
TRAJ = 7; nTraj = 1; nSamples = 2000;                                                  % natRes

for i=1:nTraj
    realTraj = load(append('dataMod',int2str(TRAJ(i)),'.txt'));
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
        qRealVec{i} = [zeros(nSamples(i),1) pi/180*firstPassiveAngleX zeros(nSamples(i),1) zeros(nSamples(i),1) pi/2*ones(nSamples(i),1) zeros(nSamples(i),5) pi/2*ones(nSamples(i),1) zeros(nSamples(i),5)];
    elseif TRAJ(i)==7
%         qRealVec{i} = [pi/180*firstPassiveAngleY pi/180*firstPassiveAngleX zeros(nSamples(i),1) zeros(nSamples(i),1) pi/2*ones(nSamples(i),1) zeros(nSamples(i),5) pi/2*ones(nSamples(i),1) zeros(nSamples(i),5)];
        qRealVec{i} = [pi/180*firstPassiveAngleX pi/180*firstPassiveAngleY zeros(nSamples(i),1) zeros(nSamples(i),1) pi/2*ones(nSamples(i),1) zeros(nSamples(i),5) pi/2*ones(nSamples(i),1) zeros(nSamples(i),5)];
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
    subplot(2,ceil(nTraj/2),i), plot(time,180/pi*qRealVec{i}), grid, hold on, xlim([time(1) time(end)]), xlabel('time [s]'), ylabel('angle [rad]'), title(append('Trajectory ',int2str(i))), legend('q1','q2','q3','q4','q5','q6','q7','q8','q9','q10','q11','q12','q13','q14','q15','q16')
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
    qPassive0{i} = qRealVec{i}(1,1:4);
    qActive0{i} = [pi/2 0 0 0 0 0 pi/2 0 0 0 0 0]';
    qdActive0{i} = [0 0 0 0 0 0 0 0 0 0 0 0]';
    platformPoseReal0{i} = platformPoseRealVec{i}(1,:)';
end

%% Definire i riferimenti per le braccia
for i=1:nTraj
    qRef{i} = ones(length(tModel{i}),1)*[pi/2 0 0 0 0 0 pi/2 0 0 0 0 0];
    qdRef{i} = ones(length(tModel{i}),1)*[0 0 0 0 0 0 0 0 0 0 0 0];
end

%% Creare una corrFun e testarla con dei parametri a caso

% L = 3.2377;
% m_cables = 9.4888;
% l_cables = 0;
% ixx_cables = 0.0628;
% fv1p = 36.1293;
% fv2p = 100.00320;
% 
% p = [L,m_cables,l_cables,ixx_cables,fv1p,fv2p];
% cost = corrcoefFunCranebotComplete(p, qRealVecInterp, qActive0, qdActive0, qPassive0, qRef, qdRef, Ts_sim, meanTs, sampling, nSamples, nTraj);

%% Definire il problema di minimizzazione 
fun = @(p) corrcoefFunCranebotComplete(p, qRealVecInterp, qActive0, qdActive0, qPassive0, qRef, qdRef, Ts_sim, meanTs, sampling, nSamples, nTraj);

% ----------------------------------------------------------------------------------------------------

% L
% m_cables
% l_cables
% ixx_cables
% fv1p fv2p

p0 = [3.286 ...
      10 ...
      1.5 ...
      0.01 ...
      30 100];

LB = [1 ...
      0.3 ...
      0 ...
      0.001 ...
      0 0];

UB = [5 ...
      20 ...
      3 ...
      10 ...
      500 500];

% ----------------------------------------------------------------------------------------------------

tic
options = optimoptions('fmincon','PlotFcn',{@optimplotx,@optimplotfval,@optimplotfirstorderopt},'Display','iter-detailed','Algorithm','active-set','UseParallel',true);          
[p,fval] = fmincon(fun,p0,[],[],[],[],LB,UB,[],options)
toc

%% Model dynamics

% p = [4.333    9.6948    0.1000    0.0500  100.8375  100.0040];
% p = [3.2994    9.9983    1.5031    0.0109  100.0006  100.0000];
% p = [3.286    9.9983    1.5031    0.0109  30 100.0000];

% p = [3.2377 9.4888 0 0.0628 36.1293 100.00320];  % TROLLEY

L = p(1);
for i=1:nTraj
    disp(append('Simulating dynamics for trajectory ',int2str(i)));
    
    qModel(1,1:4) =  reshape(qPassive0{i},1,4);                        % Initialize the passive joint
    qModel(1,5:16) = reshape(qActive0{i},1,12);                         % Initialize qModel with the starting condition of the active joints
    qdModel(1,1:4) = zeros(1,4);                                        % Initialize qdModel with zero velocity for the passive joints
    qdModel(1,5:16) = reshape(qdActive0{i},1,12);                       % Initialize qdModel with the starting condition of the active joints

    qRef_i = qRef{i};                                                   % Define qRef for the i-th trajectory
    qdRef_i = qdRef{i};                                                 % Define qdRef_i for the i-th trajectory

    WB = waitbar(0);
    for k=2:(length(tModel{i}))                                                                                                   % For each sample
        [qModel(k,:), qdModel(k,:)] = qFunCranebotComplete(p,qModel(k-1,:),qdModel(k-1,:),Ts_sim,qRef_i(k-1,:),qdRef_i(k-1,:));        % Compute the dynamics iteratively
        waitbar(k/(length(tModel{i})), WB, append('Dynamic model computation',' - ',num2str(k/(length(tModel{i}))*100),'%') );
    end                                                                                                                        % End-for
    
    qModelVec{i} = qModel;
end

%% Plot

% First Joint Identification
for j=1:2                                           % For each joint
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


% % Shoulders horizontality
% figure()
% sgtitle('Shoulders horizontality')
% for i=1:nTraj                                       % For each trajectory
%     subplot(2,ceil(nTraj/2),i);
%     plot(tModel{i},(qModel{i}(:,4)+qModel{i}(:,2))*180/pi); grid, hold on; xlim([tModel{i}(1) tModel{i}(end)]), xlabel('time [s]'), ylabel('angle [deg]'), title(append('Trajectory ',int2str(i)));
%     plot(tModel{i},(qModel{i}(:,5)+qModel{i}(:,3))*180/pi)
%     legend('Horizontality X','Horizontality Y')
% end

% Shoulder Pose
% for j=1:3                                               % For each joint
%     figure()
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
%             subplot(4,ceil(nTraj/2),i+ceil(nTraj));
%             plot(tModel{i},abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j))), grid, xlabel('time [s]'), ylabel(Y{j}), title(append('Error - Trajectory ',int2str(i)));     
% %             plot(tModel{i},abs(shoulderPoseRealVecInterp{i}(:,j) - shoulderPoseModel{i}(:,j))), grid, ylim([0 YErrlimU(j)]), xlabel('time [s]'), ylabel(Y{j}), title(append('Error - Trajectory ',int2str(i)));     
%         end
%     end
% end
% 
% % close all
% 
% % Active Joints
% for j=1:8                                               % For each joint
%     figure()
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
%             plot(tModel{i},abs(qRealVecInterp{i}(:,j)-qModel{i}(:,j+5))), grid, xlim([tModel{i}(1) tModel{i}(end)]), xlabel('time [s]'), ylabel('angle [deg]'), title(append('Error - Trajectory ',int2str(i)));     
%         elseif i>ceil(nTraj/2)
%             subplot(4,ceil(nTraj/2),i+ceil(nTraj/2));
%             plot(tModel{i},qRefVecInterp{i}(:,j)*180/pi), grid, hold on; xlim([tModel{i}(1) tModel{i}(end)]), xlabel('time [s]'), ylabel('angle [deg]'), title(append('Trajectory ',int2str(i)));
%             plot(tModel{i},qModel{i}(:,j+5)*180/pi)
%             plot(tModel{i},qRefVecInterp{i}(:,j)*180/pi,'k--')
%             legend('Real','Model','Command')
%             
%             subplot(4,ceil(nTraj/2),i+ceil(nTraj));
%             plot(tModel{i},abs(qRealVecInterp{i}(:,j)-qModel{i}(:,j+5))), grid, xlim([tModel{i}(1) tModel{i}(end)]), xlabel('time [s]'), ylabel('angle [deg]'), title(append('Error - Trajectory ',int2str(i)));     
%         end
%     end
% end
