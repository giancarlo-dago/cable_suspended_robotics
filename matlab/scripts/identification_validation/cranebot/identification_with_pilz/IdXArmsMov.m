
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

TRAJ = {'ModArmsMovExpX_amp0.1', 'ModArmsMovExpX_amp0.2', 'ModArmsMovExpX_amp0.3', 'ModNatResExpX1', 'ModNatResExpX2', 'ModNatResExpX3'};
nTraj = length(TRAJ);
figure
for i=1:nTraj
    realTraj = load(append(TRAJ{i},'.txt'));
    %     nSamples(i) = length(realTraj);
    nSamples(i) = 2400;
%     nSamples(i) = 4000;
    time = realTraj(1:nSamples(i),1);                       %     time = t-t(1);
    meanTs(i) = mean(time(2:end)-time(1:end-1));
    platformPosition = realTraj(1:nSamples(i),2:3);
    platformOrientation = realTraj(1:nSamples(i),4);
    
    platformX = platformPosition(:,1);
    platformY = platformPosition(:,2);
    firstPassiveAngleZ = platformOrientation(:,1);
    timeRealVec{i} = time;
    platformPoseRealVec{i} = [ zeros(length(time),1) platformX zeros(length(time),1)];

    plot(timeRealVec{i},platformPoseRealVec{i}(:,2)), hold on

end

% Interpolare comandi e dati
for i=1:nTraj
    tModel{i} = (0:Ts_sim*sampling:meanTs(1)*nSamples(i))';
    platformPoseRealVecInterp{i} = interp1(timeRealVec{i},platformPoseRealVec{i},tModel{i},'cubic');
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
    qRefMat{i} = ones(length(tModel{i}),1)*[0 0 0 0];
    qdRefMat{i} = ones(length(tModel{i}),1)*[0 0 0 0];
end


% Definire i riferimenti per le braccia
amp = 0.1;
tTrajectories = [2 2 2 2 2 2 2 2 2 2 2]';
sViaPoints = [0 0 0 0;
              -amp -amp -amp -amp;
              amp amp amp amp;
              -amp -amp -amp -amp;
              amp amp amp amp;
              -amp -amp -amp -amp;
              amp amp amp amp;
              -amp -amp -amp -amp;
              amp amp amp amp;
              -amp -amp -amp -amp;
              amp amp amp amp;
              0 0 0 0];
sDotViaPoints = zeros(12,4);
[qRef, qdRef, qddRef, t] = concatenatedMultiJointCubicTraj(Ts_sim, tTrajectories, sViaPoints, sDotViaPoints);

for i=1:4
    qRef{i}(end+1:end+length(tModel{1})-length(t{i})) = zeros(1,length(tModel{1})-length(t{i}));
    qdRef{i}(end+1:end+length(tModel{1})-length(t{i})) = zeros(1,length(tModel{1})-length(t{i}));
%     qdRef{i}(1:length(qRef{1})) =  zeros(1,length(qRef{1}));
end

qRefMat{1} = cell2mat(qRef);
qdRefMat{1} = cell2mat(qdRef);


amp = 0.2;
tTrajectories = [2 2 2 2 2 2 2 2 2 2 2]';
sViaPoints = [0 0 0 0;
              -amp -amp -amp -amp;
              amp amp amp amp;
              -amp -amp -amp -amp;
              amp amp amp amp;
              -amp -amp -amp -amp;
              amp amp amp amp;
              -amp -amp -amp -amp;
              amp amp amp amp;
              -amp -amp -amp -amp;
              amp amp amp amp;
              0 0 0 0];
sDotViaPoints = zeros(12,4);
[qRef, qdRef, qddRef, t] = concatenatedMultiJointCubicTraj(Ts_sim, tTrajectories, sViaPoints, sDotViaPoints);

for i=1:4
    qRef{i}(end+1:end+length(tModel{1})-length(t{i})) = zeros(1,length(tModel{1})-length(t{i}));
    qdRef{i}(end+1:end+length(tModel{1})-length(t{i})) = zeros(1,length(tModel{1})-length(t{i}));
%     qdRef{i}(1:length(qRef{1})) =  zeros(1,length(qRef{1}));
end

qRefMat{2} = cell2mat(qRef);
qdRefMat{2} = cell2mat(qdRef);

amp = 0.3;
tTrajectories = [2 2 2 2 2 2 2 2 2 2 2]';
sViaPoints = [0 0 0 0;
              -amp -amp -amp -amp;
              amp amp amp amp;
              -amp -amp -amp -amp;
              amp amp amp amp;
              -amp -amp -amp -amp;
              amp amp amp amp;
              -amp -amp -amp -amp;
              amp amp amp amp;
              -amp -amp -amp -amp;
              amp amp amp amp;
              0 0 0 0];
sDotViaPoints = zeros(12,4);
[qRef, qdRef, qddRef, t] = concatenatedMultiJointCubicTraj(Ts_sim, tTrajectories, sViaPoints, sDotViaPoints);

for i=1:4
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

% % Define the function to fit
% f = fittype( 'modelPlanarArms(lCz,mC,fvC,fsC,Ts_sim,platformPose0,qRef,qdRef,t)', ...
%              'independent', 't', 'dependent', 'y' ,...
%              'coefficient', {'fvC','fsC'}, 'problem', {'lCz','mC','Ts_sim','qRef','qdRef','platformPose0'})
% 
% [f1, gof, output] = fit(t1, x1, f, 'StartPoint', [0 0], 'problem',{3.8,206,Ts_sim,qRefMat,qdRefMat,platformPose0_1}, ...
%             'Normalize', 'off', 'Lower', [0 0], 'Upper', [500 3], ...
%             'Display', 'iter', 'MaxIter', 25, 'DiffMaxChange', 0.1)
% 
% 
% figure, plot(f1,t1,x1)
% disp('---------------------')
% 
% 
% [f2, gof, output] = fit(t2, x2, f, 'StartPoint', [0 0], 'problem',{3.8,185,Ts_sim,platformPose0_2}, ...
%             'Normalize', 'off', 'Lower', [0 0], 'Upper', [500 3], ...
%             'Display', 'iter', 'MaxIter', 25, 'DiffMaxChange', 0.1, 'Exclude', t2>50)
% 
% 
% figure, plot(f2,t2,x2)
% disp('---------------------')
% 
% 
% [f3, gof, output] = fit(t3, x3, f, 'StartPoint', [0 0], 'problem',{3.8,185,Ts_sim,platformPose0_3}, ...
%             'Normalize', 'off', 'Lower', [0 0], 'Upper', [500 3], ...
%             'Display', 'iter', 'MaxIter', 25, 'DiffMaxChange', 0.1, 'Exclude', t3>80)
% 
% figure, plot(f3,t3,x3)
% disp('---------------------')


%%%%%%%% OPTIMIZATION PROBLEM %%%%%%%%
 
errorfun = @(p) [x1 - modelPlanarArms('X',p,Ts_sim,platformPose0_1,qRefMat{1},qdRefMat{1},t1), ...
                 x2 - modelPlanarArms('X',p,Ts_sim,platformPose0_2,qRefMat{2},qdRefMat{2},t2), ...
                 x3 - modelPlanarArms('X',p,Ts_sim,platformPose0_3,qRefMat{3},qdRefMat{3},t3)];

% errorfun = @(p) [x4 - modelPlanarArms('X',p,Ts_sim,platformPose0_4,qRefMat{4},qdRefMat{4},t4), ...
%                  x5 - modelPlanarArms('X',p,Ts_sim,platformPose0_5,qRefMat{5},qdRefMat{5},t5), ...
%                  x6 - modelPlanarArms('X',p,Ts_sim,platformPose0_6,qRefMat{6},qdRefMat{6},t6)];

% Fit the model to the first signal, constraining the coefficients
eps = [50   0  0  0.8  0.0  0.0  0.0  300];
p0 = [32.7  20  25000    3.64    0.0    0.0    0.0   200];
lb = p0 - eps;
ub = p0 + eps;
options = optimoptions('lsqnonlin','Display','iter');
% [f,resnorm,residual] = lsqnonlin(errorfun,p0,lb,ub,options);

%%%%%%%% VALIDATION %%%%%%%%

% lCz,mC,fvC,fsC,kd,kp,iA1yy,iA2yy,iCyy
f = [3.64 206 52.7 0 20 16000 0.05 0.05 198.5];

x1_sim = modelPlanarArms('X',f,Ts_sim,platformPose0_1,qRefMat{1},qdRefMat{1},t1);
x2_sim = modelPlanarArms('X',f,Ts_sim,platformPose0_2,qRefMat{2},qdRefMat{2},t2);
x3_sim = modelPlanarArms('X',f,Ts_sim,platformPose0_3,qRefMat{3},qdRefMat{3},t3);
x4_sim = modelPlanarArms('X',f,Ts_sim,platformPose0_4,qRefMat{4},qdRefMat{4},t4);
x5_sim = modelPlanarArms('X',f,Ts_sim,platformPose0_5,qRefMat{5},qdRefMat{5},t5);
x6_sim = modelPlanarArms('X',f,Ts_sim,platformPose0_6,qRefMat{6},qdRefMat{6},t6);


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

data = load('dataX.txt');
time = data(:,1);
x = data(:,2);
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

%% Plot for paper

set(0, 'DefaultTextInterpreter', 'latex')
set(0, 'DefaultLegendInterpreter', 'latex')
set(0, 'DefaultAxesTickLabelInterpreter', 'latex')
lw = 1.5;

h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);

plot(t3, x3, 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]); hold on
plot(t3, x3_sim, 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
e = abs(x3 - x3_sim);
plot(t3,e, 'k--', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2])
legend('${r}_m^x$','${r}_s^x$','$|e^x|$','Orientation','horizontal'); xlabel('t [s]'); ylabel('x [m]'); set(gca, 'FontSize',18); set(gca, 'XLim', [t3(1) t3(end)]);
set(gca, 'YLim', [-0.08 0.08], 'YTick',[-0.08:0.04:0.08]);
grid on; box on; set(gcf,'color','w');
R = corrcoef(x3,x3_sim);
rho = R(1,2);
str = {['$|\overline{e}|=',num2str(round(max(e),3)),'\,m$','\,\,\,\,\,$\rho=',num2str(round(rho,3)),'$']};
annotation('textbox', [0.63, 0.255, 0.1, 0.1], 'String',str,'interpreter','latex', 'BackgroundColor', 'w', 'FitBoxToText','on','FontSize',16,'VerticalAlignment','middle')

exportgraphics(h, 'CranebotArmsMovX_amp0.3.pdf');

%% Computation MAE (mean absolute error) and MRE (mean relative error)
true_signal = x3;
predicted_signal = x3_sim;
abs_error = abs(true_signal-predicted_signal);

MAE = mean(abs_error)

abs_error_new = abs_error((isfinite(abs_error)));
true_signal_new = true_signal((isfinite(true_signal)));

ratio = abs_error_new./abs(true_signal_new);
rationew = ratio((isfinite(ratio)));

MRE = mean(rmoutliers(rationew))

figure(), plot(1:length(rationew),rationew)