
close all
clear
clc

if ispc % Windows
    addpath('..\..\..\..\functions\trajectory_generation_functions\')
    addpath('..\..\..\experimental_data_analysis\cranebot\treatNewData\')
else % Linux
    addpath('../../../../functions/trajectory_generation_functions/')
    addpath('../../../experimental_data_analysis/cranebot/treatNewData/')
end

% Parameters
sampling = 1;
Ts_sim = 0.001;

TRAJ = {'ModArmsMovExpZ_amp0.05', 'ModArmsMovExpZ_amp0.1', 'ModArmsMovExpZ_amp0.15', 'ModNatResExpZ1', 'ModNatResExpZ2', 'ModNatResExpZ3'};
nTraj = length(TRAJ);
figure
for i=1:nTraj
    realTraj = load(append(TRAJ{i},'.txt'));
    %     nSamples(i) = length(realTraj);
    nSamples(i) = 700;
    time = realTraj(1:nSamples(i),1);                       %     time = t-t(1);
    meanTs(i) = mean(time(2:end)-time(1:end-1));
    platformPosition = realTraj(1:nSamples(i),2:3);
    platformOrientation = realTraj(1:nSamples(i),4);
    
    platformX = platformPosition(:,1);
    platformY = platformPosition(:,2);
    firstPassiveAngleZ = platformOrientation(:,1);
    timeRealVec{i} = time;
    platformPoseRealVec{i} = [zeros(length(time),1) firstPassiveAngleZ zeros(length(time),1)];

    plot(timeRealVec{i},platformPoseRealVec{i}(:,2)), hold on

end

% Interpolare comandi e dati
for i=1:nTraj
    tModel{i} = (0:Ts_sim*sampling:meanTs(1)*nSamples(i))';
    platformPoseRealVecInterp{i} = interp1(timeRealVec{i},platformPoseRealVec{i},tModel{i},'linear');
end

% Eliminare eventuali NaN
for i=1:nTraj
    TF = isnan(platformPoseRealVecInterp{i});
    for k=1:length(TF)
        if TF(k,2)==1
            platformPoseRealVecInterp{i}(k,2) = platformPoseRealVecInterp{i}(k-1,2);
        end
    end
end

for i=4:6
    qRefMat{i} = ones(length(tModel{i}),1)*[0 0];
    qdRefMat{i} = ones(length(tModel{i}),1)*[0 0];
end


% Definire i riferimenti per le braccia
amp = 0.05;
tTrajectories = 1.5*ones(11,2);
sViaPoints = [0 0;
              -amp -amp;
              amp amp;
              -amp -amp;
              amp amp;
              -amp -amp;
              amp amp;
              -amp -amp;
              amp amp;
              -amp -amp;
              amp amp;
              0 0];
sDotViaPoints = zeros(12,2);
[qRef, qdRef, qddRef, t] = concatenatedMultiJointCubicTraj(Ts_sim, tTrajectories, sViaPoints, sDotViaPoints);

for i=1:2
    qRef{i}(end+1:end+length(tModel{1})-length(t{i})) = zeros(1,length(tModel{1})-length(t{i}));
    qdRef{i}(end+1:end+length(tModel{1})-length(t{i})) = zeros(1,length(tModel{1})-length(t{i}));
%     qdRef{i}(1:length(qRef{1})) =  zeros(1,length(qRef{1}));
end

qRefMat{1} = cell2mat(qRef);
qdRefMat{1} = cell2mat(qdRef);


amp = 0.1;
tTrajectories = 1.5*ones(11,2);
sViaPoints = [0 0;
              -amp -amp;
              amp amp;
              -amp -amp;
              amp amp;
              -amp -amp;
              amp amp;
              -amp -amp;
              amp amp;
              -amp -amp;
              amp amp;
              0 0];
sDotViaPoints = zeros(12,2);
[qRef, qdRef, qddRef, t] = concatenatedMultiJointCubicTraj(Ts_sim, tTrajectories, sViaPoints, sDotViaPoints);

for i=1:2
    qRef{i}(end+1:end+length(tModel{1})-length(t{i})) = zeros(1,length(tModel{1})-length(t{i}));
    qdRef{i}(end+1:end+length(tModel{1})-length(t{i})) = zeros(1,length(tModel{1})-length(t{i}));
%     qdRef{i}(1:length(qRef{1})) =  zeros(1,length(qRef{1}));
end

qRefMat{2} = cell2mat(qRef);
qdRefMat{2} = cell2mat(qdRef);

amp = 0.15;
tTrajectories = 1.5*ones(11,2);
sViaPoints = [0 0;
              -amp -amp;
              amp amp;
              -amp -amp;
              amp amp;
              -amp -amp;
              amp amp;
              -amp -amp;
              amp amp;
              -amp -amp;
              amp amp;
              0 0];
sDotViaPoints = zeros(12,2);
[qRef, qdRef, qddRef, t] = concatenatedMultiJointCubicTraj(Ts_sim, tTrajectories, sViaPoints, sDotViaPoints);

for i=1:2
    qRef{i}(end+1:end+length(tModel{1})-length(t{i})) = zeros(1,length(tModel{1})-length(t{i}));
    qdRef{i}(end+1:end+length(tModel{1})-length(t{i})) = zeros(1,length(tModel{1})-length(t{i}));
%     qdRef{i}(1:length(qRef{1})) =  zeros(1,length(qRef{1}));
end

qRefMat{3} = cell2mat(qRef);
qdRefMat{3} = cell2mat(qdRef);


figure()
for i=1:3
    
    subplot(3,2,i)
    plot(tModel{i},qRefMat{i}), legend, grid on, hold on
    plot(tModel{i},platformPoseRealVecInterp{i}(:,2)), legend, grid on, hold on
 
end
for i=4:6
    
    subplot(3,2,i)
    plot(tModel{i},qRefMat{i}), legend, grid on, hold on
    plot(tModel{i},platformPoseRealVecInterp{i}(:,2)), legend, grid on, hold on
 
end

% Define the signals
t1 = tModel{1};
t2 = tModel{2};
t3 = tModel{3};
t4 = tModel{4};
t5 = tModel{5};
t6 = tModel{6};
x1 = platformPoseRealVecInterp{1}(:,2);
x2 = platformPoseRealVecInterp{2}(:,2);
x3 = platformPoseRealVecInterp{3}(:,2);
x4 = platformPoseRealVecInterp{4}(:,2);
x5 = platformPoseRealVecInterp{5}(:,2);
x6 = platformPoseRealVecInterp{6}(:,2);

platformPose0_1 = platformPoseRealVecInterp{1}(1,2);
platformPose0_2 = platformPoseRealVecInterp{2}(1,2);
platformPose0_3 = platformPoseRealVecInterp{3}(1,2);
platformPose0_4 = platformPoseRealVecInterp{4}(1,2);
platformPose0_5 = platformPoseRealVecInterp{5}(1,2);
platformPose0_6 = platformPoseRealVecInterp{6}(1,2);

Ts_sim = 0.001;

%%%%%%%% TESTING FIT %%%%%%%%%%%

% Define the function to fit
f = fittype( 'modelPlanarArms(lCz,mC,fvC,fsC,Ts_sim,platformPose0,qRef,qdRef,t)', ...
             'independent', 't', 'dependent', 'y' ,...
             'coefficient', {'fvC','fsC'}, 'problem', {'lCz','mC','Ts_sim','qRef','qdRef','platformPose0'})

[f1, gof, output] = fit(t1, x1, f, 'StartPoint', [0 0], 'problem',{3.8,206,Ts_sim,qRefMat,qdRefMat,platformPose0_1}, ...
            'Normalize', 'off', 'Lower', [0 0], 'Upper', [500 3], ...
            'Display', 'iter', 'MaxIter', 25, 'DiffMaxChange', 0.1)
% 
% figure, plot(f1,t1,x1)
% disp('---------------------')
% 
% [f2, gof, output] = fit(t2, x2, f, 'StartPoint', [0 0], 'problem',{3.8,185,Ts_sim,platformPose0_2}, ...
%             'Normalize', 'off', 'Lower', [0 0], 'Upper', [500 3], ...
%             'Display', 'iter', 'MaxIter', 25, 'DiffMaxChange', 0.1, 'Exclude', t2>50)
% 
% figure, plot(f2,t2,x2)
% disp('---------------------')
% 
% [f3, gof, output] = fit(t3, x3, f, 'StartPoint', [0 0], 'problem',{3.8,185,Ts_sim,platformPose0_3}, ...
%             'Normalize', 'off', 'Lower', [0 0], 'Upper', [500 3], ...
%             'Display', 'iter', 'MaxIter', 25, 'DiffMaxChange', 0.1, 'Exclude', t3>80)
% 
% figure, plot(f3,t3,x3)
% disp('---------------------')

%%%%%%%% OPTIMIZATION PROBLEM %%%%%%%%

errorfun = @(p) [x1 - modelPlanarArms('Z',p,Ts_sim,platformPose0_1,qRefMat{1},qdRefMat{1},t1), ...
                 x2 - modelPlanarArms('Z',p,Ts_sim,platformPose0_2,qRefMat{2},qdRefMat{2},t2), ...
                 x3 - modelPlanarArms('Z',p,Ts_sim,platformPose0_3,qRefMat{3},qdRefMat{3},t3)];

% errorfun = @(p) [x4 - modelPlanarArms('W',p,Ts_sim,platformPose0_4,qRefMat{4},qdRefMat{4},t4), ...
%                  x5 - modelPlanarArms('W',p,Ts_sim,platformPose0_5,qRefMat{5},qdRefMat{5},t5), ...
%                  x6 - modelPlanarArms('W',p,Ts_sim,platformPose0_6,qRefMat{6},qdRefMat{6},t6)];

% Fit the model to the first signal, constraining the coefficients
p0 = [5.8072,0.9,0.01,25,25000,0,30.9273];
eps = [0 0 0 25 25000 0 0];

lb = p0 - eps;
ub = p0 + eps;
options = optimoptions('lsqnonlin','Display','iter');
% [f,resnorm,residual] = lsqnonlin(errorfun,p0,lb,ub,options);


%%%%%%%% VALIDATION %%%%%%%%

% [iCzz,fvC,fsC,kd,kp,kdZ,kpZ]
f = [5.8072,0.9,0.01,25,25000,0,30.9273];

x1_sim = modelPlanarArms('Z',f,Ts_sim,platformPose0_1,qRefMat{1},qdRefMat{1},t1);
x2_sim = modelPlanarArms('Z',f,Ts_sim,platformPose0_2,qRefMat{2},qdRefMat{2},t2);
x3_sim = modelPlanarArms('Z',f,Ts_sim,platformPose0_3,qRefMat{3},qdRefMat{3},t3);
x4_sim = modelPlanarArms('Z',f,Ts_sim,platformPose0_4,qRefMat{4},qdRefMat{4},t4);
x5_sim = modelPlanarArms('Z',f,Ts_sim,platformPose0_5,qRefMat{5},qdRefMat{5},t5);
x6_sim = modelPlanarArms('Z',f,Ts_sim,platformPose0_6,qRefMat{6},qdRefMat{6},t6);

% Plot the data and the fit
figure
subplot 321
plot(t1, x1, '--', t1, x1_sim)
legend('Signal 1', 'Fit1'), grid, hold on

subplot 323
plot(t2, x2, '--', t2, x2_sim)
legend('Signal 2', 'Fit2'), grid, hold on

subplot 325
plot(t3, x3, '--', t3, x3_sim)
legend('Signal 3', 'Fit3'), grid, hold on

data = load('dataZ.txt');
time = data(:,1);
x = data(:,2);
for k=1:length(x)
    if (x(k)>1)
        x(k) = x(k)-pi;
    elseif (x(k)<-1)
        x(k) = x(k)+pi;
    end
end
plot(time,x)

subplot 322
plot(t4, x4, '--', t4, x4_sim)
legend('Signal 4', 'Fit4'), grid

subplot 324
plot(t5, x5, '--', t5, x5_sim)
legend('Signal 5', 'Fit5'), grid

subplot 326
plot(t6, x6, '--', t6, x6_sim)
legend('Signal 6', 'Fit6'), grid

corrcoef(x1,x1_sim)
corrcoef(x2,x2_sim)
corrcoef(x3,x3_sim)
corrcoef(x4,x4_sim)
corrcoef(x5,x5_sim)
corrcoef(x6,x6_sim)

%% Plot commands for paper

% set(0, 'DefaultTextInterpreter', 'latex')
% set(0, 'DefaultLegendInterpreter', 'latex')
% set(0, 'DefaultAxesTickLabelInterpreter', 'latex')
% lw = 1.5;
% 
% h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);
% 
% subplot(1,2,2)
% plot(tModel{1},180/pi*qRefMat{1}(:,1), 'k-', 'Linewidth', lw ,'Color', [0.6, 0.6, 0.6]), legend, grid on, hold on
% plot(tModel{2},180/pi*qRefMat{2}(:,1), 'k-', 'Linewidth', lw ,'Color', [0.4, 0.4, 0.4]), legend, grid on, hold on
% plot(tModel{3},180/pi*qRefMat{3}(:,1), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]), legend, grid on, hold on
% legend('$|q_{joint}^d|$','Orientation','horizontal','FontSize',14); xlabel('t [s]'); ylabel('[deg]'); set(gca, 'FontSize',14); set(gca, 'XLim', [tModel{1}(1) 30]);
% set(gca, 'YLim', [-12 12], 'YTick',[-12:4:12]);

%% Plot for paper

set(0, 'DefaultTextInterpreter', 'latex')
set(0, 'DefaultLegendInterpreter', 'latex')
set(0, 'DefaultAxesTickLabelInterpreter', 'latex')
lw = 1.5;

h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);

plot(t3, 180/pi*x3, 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]); hold on
plot(t3, 180/pi*x3_sim, 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
e = abs(180/pi*x3 - 180/pi*x3_sim);
plot(t3,e, 'k--', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2])
legend('${\phi}_m^z$','${\phi}_s^z$','$|e^z|$','Orientation','horizontal'); xlabel('t [s]'); ylabel('$\phi$ [deg]'); set(gca, 'FontSize',18); set(gca, 'XLim', [t3(1) t3(end)]);
set(gca, 'YLim', [-20 20], 'YTick',[-20:10:20]);
grid on; box on; set(gcf,'color','w');
R = corrcoef(x3,x3_sim);
rho = R(1,2);
str = {['$|\overline{e}|=',num2str(round(max(e),3)),'\, ^{\circ}$','\,\,\,\,\,$\rho=',num2str(round(rho,3)),'$']};
annotation('textbox', [0.64, 0.255, 0.1, 0.1], 'String',str,'interpreter','latex', 'BackgroundColor', 'w', 'FitBoxToText','on','FontSize',16,'VerticalAlignment','middle')

exportgraphics(h, 'CranebotArmsMovZ_amp0.15.pdf');

%% Computation MAE (mean absolute error) and MRE (mean relative error)
true_signal = x3*180/pi;
predicted_signal = x3_sim*180/pi;
abs_error = abs(true_signal-predicted_signal);

MAE = mean(abs_error)

abs_error_new = abs_error((isfinite(abs_error)));
true_signal_new = true_signal((isfinite(true_signal)));

ratio = abs_error_new./abs(true_signal_new);
rationew = ratio((isfinite(ratio)));

MRE = mean(rmoutliers(rationew))

figure(), plot(1:length(rationew),rationew)