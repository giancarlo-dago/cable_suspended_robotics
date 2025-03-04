close all
clear
clc

dt = 0.001;
velDes = 0.3;
sInit = 1;
sFinal = 0;

tFinal = 8;
accDes = 0.5;

%----------------------------

% [s, sdot, sdotdot, t] = trapVelTraj(dt, accDes, velDes, sInit, sFinal);
% [s, sdot, sdotdot, t] = trapVelTraj_tf(dt, tFinal, accDes, sInit, sFinal);
% 
% subplot 311
% plot(t,s)
% subplot 312
% plot(t,sdot)
% subplot 313
% plot(t,sdotdot)

%----------------------------

% [s, sdot, sdotdot, t] = multiJointTrapVelTraj_tf(dt, [5 5 5 5 ], [.5 .5 .5 .5 ], [1 2 3 4], [0 0 0 0])

% subplot 311
% plot(t{1},cell2mat(s)), legend, grid
% subplot 312
% plot(t{1},cell2mat(sdot)), legend, grid
% subplot 313
% plot(t{1},cell2mat(sdotdot)), legend, grid

%----------------------------

% [s, sdot, sdotdot, t] = multiJointTrapVelTraj(dt, [.5 .5 .5 .5 ], [.3 .3 .3 .3 ], [1 2 3 4], [0 0 0 0])
% 
% figure()
% for i=1:4
%     
%     subplot 311
%     plot(t{i},s{i}), legend, grid on, hold on
%     subplot 312
%     plot(t{i},sdot{i}), legend, grid on, hold on
%     subplot 313
%     plot(t{i},sdotdot{i}), legend, grid on, hold on
%     
% end

%% ---------------------------

period = 4;
time = 0 : 0.04 : 4;
f = 1/period;
phi = 0;
A = 0.5;
qA = A*cos(2*pi*f*time + phi) + 0.0;
qAd = -(A*2*pi*f)*sin(2*pi*f*time + phi);
qAdd = -A*(2*pi*f)^2*cos(2*pi*f*time + phi);

tFinal = [2; 2]';
sInit = [0.5; -0.5; 0.5];
sFinal = [-0.5; 0.5; -0.5]; 
accDes = [1.0; 1.0; 1.0];
[sT, sdotT, sdotdotT, tT] = concatenatedMultiJointTrapVelTraj_tf(dt, tFinal, accDes, sInit, sFinal);

tTrajectories = [2; 2]';
sViaPoints = [0.5; -0.5; 0.5];
sDotViaPoints = [0; 0; 0];
[sC, sdotC, sdotdotC, tC] = concatenatedMultiJointCubicTraj(dt, tTrajectories, sViaPoints, sDotViaPoints);

tTrajectories = [2; 2]';
sViaPoints = [0.5; -0.5; 0.5];
sDotViaPoints = [0; 0; 0];
sDot2ViaPoints = [0; 0.0; 0];
[sQ, sdotQ, sdotdotQ, tQ] = concatenatedMultiJointQuinticTraj(dt, tTrajectories, sViaPoints, sDotViaPoints, sDot2ViaPoints);

figure()

subplot 311
plot(time,qA), legend, grid on, hold on
plot(tT{1},sT{1})
plot(tC{1},sC{1})
plot(tQ{1},sQ{1})
subplot 312
plot(time,qAd), legend, grid on, hold on
plot(tT{1},sdotT{1})
plot(tC{1},sdotC{1})
plot(tQ{1},sdotQ{1})
subplot 313
plot(time,qAdd), legend, grid on, hold on
plot(tT{1},sdotdotT{1})
plot(tC{1},sdotdotC{1})
plot(tQ{1},sdotdotQ{1})

    

%% ---------------------------

% tTrajectories = [2 2 2 2]';
% sViaPoints = [0 0 0 0; 1 2 3 4; 0 0 0 0; 1 2 3 4; 0 0 0 0];
% sDotViaPoints = [0 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 0];
% [s, sdot, sdotdot, t] = multiJointCubicVelTraj(dt, tTrajectories, sViaPoints, sDotViaPoints);
% 
% figure()
% for i=1:4
%     
%     subplot 311
%     plot(t{i},s{i}), legend, grid on, hold on
%     subplot 312
%     plot(t{i},sdot{i}), legend, grid on, hold on
%     subplot 313
%     plot(t{i},sdotdot{i}), legend, grid on, hold on
%     
% end
