close all
clear
clc

if ispc % Windows
    addpath('..\..\..\functions/trajectory_generation_functions')
else % Linux
    addpath('../../../functions/trajectory_generation_functions')
end

%% Bridge - Long Travel
MinVel_bridge = 0.16666666;                            % [m/s]
MaxVel_bridge = 0.66666666;                            % [m/s]
FirstAcc_bridge = 0.56;                                % [m/s^2]
SecondAcc_bridge = 0.25;                               % [m/s^2]
q0_bridge = 0;                                         % [m]
qf_bridge = 3;                                         % [m]
samplingTime = 0.001;
[q_bridge,qd_bridge,qdd_bridge,time_bridge] = doubleRampTrapVelTraj(samplingTime, FirstAcc_bridge, SecondAcc_bridge, MinVel_bridge, MaxVel_bridge, q0_bridge, qf_bridge);

%% Trolley - Cross Travel
MinVel_trolley = 0.1666666;                            % [m/s]
MaxVel_trolley = 0.3333333;                            % [m/s]
FirstAcc_trolley = 0.40;                               % [m/s^2]
SecondAcc_trolley = 0.22;                              % [m/s^2]
q0_trolley = 0;                                        % [m]
qf_trolley = 3;                                       % [m]
samplingTime = 0.001;
[q_trolley,qd_trolley,qdd_trolley,time_trolley] = doubleRampTrapVelTraj(samplingTime, FirstAcc_trolley, SecondAcc_trolley, MinVel_trolley, MaxVel_trolley, q0_trolley, qf_trolley);

%% Plot bridge velocity profile
figure('NumberTitle','off','Name','Building 927 overhead crane - Bridge Motion Profile','WindowState','Maximized')
% sgtitle('Building 927 overhead crane - Bridge Motion Profile','FontSize',15,'FontWeight','bold')
subplot(3,1,1);
plot(time_bridge, q_bridge); grid; title('Position'), xlabel('[s]'), ylabel('[m]'); xlim([time_bridge(1) time_bridge(end)])
subplot(3,1,2);
plot(time_bridge, qd_bridge); grid; title('Velocity'), xlabel('[s]'), ylabel('[m/s]'); xlim([time_bridge(1) time_bridge(end)])
subplot(3,1,3);
plot(time_bridge, qdd_bridge); grid; title('Acceleration'), xlabel('[s]'), ylabel('[m/s^2]'); xlim([time_bridge(1) time_bridge(end)])

%% Plot trolley velocity profile
figure('NumberTitle','off','Name','Building 927 overhead crane - Trolley Motion Profile','WindowState','Maximized')
% sgtitle('Building 927 overhead crane - Trolley Motion Profile','FontSize',15,'FontWeight','bold')
subplot(3,1,1);
plot(time_trolley, q_trolley); grid; title('Position'), xlabel('[s]'), ylabel('[m]'); xlim([time_trolley(1) time_trolley(end)])
subplot(3,1,2);
plot(time_trolley, qd_trolley); grid; title('Velocity'), xlabel('[s]'), ylabel('[m/s]'); xlim([time_trolley(1) time_trolley(end)])
subplot(3,1,3);
plot(time_trolley, qdd_trolley); grid; title('Acceleration'), xlabel('[s]'), ylabel('[m/s^2]'); xlim([time_trolley(1) time_trolley(end)])



