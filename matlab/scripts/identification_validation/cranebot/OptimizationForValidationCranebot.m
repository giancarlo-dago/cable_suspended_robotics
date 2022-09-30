close all
clear
clc

%% Parameters
sampling = 1;
Ts_sim = 0.001;

%%
TRAJ = 1; nSamples = 12516;                                               % natRes
% TRAJ = 1; nSamples = 500;                                                 % natRes
% TRAJ = 4; nSamples = 13411;                                               % natRes
% TRAJ = 6; nSamples = 58748;                                                 % natRes
% TRAJ = 6; nSamples = 5000;                                                % natRes
% TRAJ = 7; nSamples = 79473;                                               % natRes

realTraj = load(append('dataMod',int2str(TRAJ),'.txt'));
t = realTraj(1:nSamples,1);
time = t-t(1);
meanTs = mean(time(2:end)-time(1:end-1));
platformPosition = realTraj(1:nSamples,2:4);
platformOrientation = realTraj(1:nSamples,8:10);
platformVelocity = realTraj(1:nSamples,11:13);
platformAngVelocity = realTraj(1:nSamples,14:16);

platformX = platformPosition(:,1);
platformY = platformPosition(:,2);
AngleZ = platformOrientation(:,1);
AngleY = platformOrientation(:,2);
AngleX = platformOrientation(:,3);
platformVelX = platformVelocity(:,1);
platformVelY = platformVelocity(:,2);
AngleVelZ = platformAngVelocity(:,1);
AngleVelY = platformAngVelocity(:,2);
AngleVelX = platformAngVelocity(:,3);

if TRAJ==4 || TRAJ==1 || TRAJ==2
    angleReal = pi/180*AngleX;
    angleVelReal = pi/180*AngleVelX;
elseif TRAJ==7 || TRAJ==6
    angleReal = pi/180*AngleX;
    angleVelReal = pi/180*AngleVelX;
end

% Plot
figure(), sgtitle('Measures')
subplot(2,1,1), plot(time,180/pi*angleReal), grid, hold on, xlim([time(1) time(end)]), xlabel('time [s]'), ylabel('position [deg]')
subplot(2,1,2), plot(time,180/pi*angleVelReal), grid, hold on, xlim([time(1) time(end)]), xlabel('time [s]'), ylabel('velocity [deg/s]')

%% Interpolare comandi e dati
tModel = (0:Ts_sim*sampling:meanTs*nSamples)';
angleRealInterp = interp1(time,angleReal,tModel,'spline');
angleVelRealInterp = interp1(time,angleVelReal,tModel,'spline');

%% Eliminare eventuali NaN
TF = isnan(angleRealInterp);
angleRealInterp(TF) = 0;
TF = isnan(angleVelRealInterp);
angleVelRealInterp(TF) = 0;

figure(), sgtitle('Interpolated Measures')
subplot(2,1,1), plot(tModel,180/pi*angleRealInterp), grid, hold on, xlim([tModel(1) tModel(end)]), xlabel('time [s]'), ylabel('position [deg]')
subplot(2,1,2), plot(tModel,180/pi*angleVelRealInterp), grid, hold on, xlim([tModel(1) tModel(end)]), xlabel('time [s]'), ylabel('velocity [deg/s]')

%% Definire le starting conditions

% if  TRAJ==1
%     qPassive0 = [angleReal(1) deg2rad(0)]';
%     qdPassive0 = [angleVelReal(1) deg2rad(0)]';
%     
%     qPassive0 = [deg2rad(-1) deg2rad(1)]';
%     qdPassive0 = [deg2rad(1.8) deg2rad(0)]';
% elseif TRAJ==2
%     qPassive0 = [angleReal(1) deg2rad(0)]';
%     qdPassive0 = [angleVelReal(1) deg2rad(0)]';
% elseif TRAJ==4
%     qPassive0 = [deg2rad(-4.4) deg2rad(-1.6)]';
%     qdPassive0 = [deg2rad(0) deg2rad(0)]';
% elseif TRAJ==6
%     qPassive0 = [angleReal(1) deg2rad(0)]';
%     qdPassive0 = [angleVelReal(1) deg2rad(0)]';
%     qPassive0 = [deg2rad(0) deg2rad(0)]';
%     qdPassive0 = [deg2rad(-4.95) deg2rad(0)]';
% elseif TRAJ==7
%     qPassive0 = [angleReal(1) deg2rad(0)]';
%     qdPassive0 = [deg2rad(0) deg2rad(0)]';
% end
qActive0 = [pi/2 0 0 0 0 0 pi/2 0 0 0 0 0]';
qdActive0 = [0 0 0 0 0 0 0 0 0 0 0 0]';

%% Definire i riferimenti per le braccia
qRef = ones(length(tModel),1)*[pi/2 0 0 0 0 0 pi/2 0 0 0 0 0];
qdRef = ones(length(tModel),1)*[0 0 0 0 0 0 0 0 0 0 0 0];

%% Creare una corrFun e testarla con dei parametri a caso

% qPassive0 = [q1_0 q2_0 qd1_0 qd2_0];
% qPassive0 = deg2rad([-3 2 0 0]);
% p = [3.286 29.3791 1.5 0.3753 36.8108 112.3436 0]; % BRIDGE
% 
% cost = corrcoefValidation(p, angleRealInterp, qActive0, qdActive0, qPassive0, qRef, qdRef, Ts_sim, meanTs, sampling, nSamples)

%% Definire il problema di minimizzazione 
p = [3.286 29.3791 1.5 0.3753 36.8108 112.3436 0]; % BRIDGE

fun = @(qPassive0) corrcoefValidation(p, angleRealInterp, qActive0, qdActive0, qPassive0, qRef, qdRef, Ts_sim, meanTs, sampling, nSamples)

% ----------------------------------------------------------------------------------------------------
% IDENTIFICATION FOR VALIDATION

% paramsToIdentify = [q1_0 q2_0 qd1_0 qd2_0];

startingCondition = deg2rad([0 0 1.8 0]);     % [deg]
LB = deg2rad([-1 -1 1 0]); % [deg]
UB = deg2rad([ 1  1 2.5 2.5]); % [deg]

startingCondi
 
% ----------------------------------------------------------------------------------------------------

tic
options = optimoptions('fmincon','PlotFcn',{@optimplotx,@optimplotfval,@optimplotfirstorderopt},'Display','iter-detailed','Algorithm','active-set','UseParallel',true);          
[qPassiveIdentified,fval] = fmincon(fun,startingCondition,[],[],[],[],LB,UB,[],options)
toc

%% Model dynamics
%     0.0873    0.0114    0.0682   -0.0873

disp(append('Simulating dynamics'));

qModel(1,1:2) =  reshape(qPassiveIdentified(1:2),1,2);           % Initialize the passive joint
qModel(1,3:14) = reshape(qActive0,1,12);                         % Initialize qModel with the starting condition of the active joints
qdModel(1,1:2) =  reshape(qPassiveIdentified(3:4),1,2);          % Initialize qdModel with zero velocity for the passive joints
qdModel(1,3:14) = reshape(qdActive0,1,12);                       % Initialize qdModel with the starting condition of the active joints

qRef_i = qRef;                                                   % Define qRef for the i-th trajectory
qdRef_i = qdRef;                                                 % Define qdRef_i for the i-th trajectory

WB = waitbar(0);
for k=2:(length(tModel))                                                                                                % For each sample
    [qModel(k,:), qdModel(k,:)] = qFunCranebot(p,qModel(k-1,:),qdModel(k-1,:),Ts_sim,qRef_i(k-1,:),qdRef_i(k-1,:));     % Compute the dynamics iteratively
    waitbar(k/(length(tModel)), WB, append('Dynamic model computation',' - ',num2str(k/(length(tModel))*100),'%') );
end                                                                                                                     % End-for
qModelVec = qModel;

% First Joint Identification
figure()
sgtitle('Identification Results')
subplot(2,1,1);
plot(tModel,180/pi*angleRealInterp); grid, hold on, xlim([tModel(1) tModel(end)]), xlabel('time [s]'), ylabel('angle [deg]')
plot(tModel,180/pi*(qModelVec(:,1)+qModelVec(:,2)))
legend('Real','Model')
subplot(2,1,2);
plot(tModel,180/pi*abs(angleRealInterp - (qModelVec(:,1)+qModelVec(:,2)))), grid, xlabel('time [s]'), ylabel('angle [deg]'), title(append('Error'));     

% FFT measurements
LL = length(angleRealInterp);
Fs = 1/0.001;

Y = fft(angleRealInterp);
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

% FFT simulation
LL = length(qModelVec(:,1));
Fs = 1/0.001;

Y = fft(qModelVec(:,1)+qModelVec(:,2));
P2 = abs(Y/LL);
P1 = P2(1:LL/322+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(LL/322))/LL;

plot(f,P1)
legend('Real','Model')

