close all
clear
clc

if ispc % Windows
    addpath('..\..\..\functions/trajectory_generation_functions')
else % Linux
    addpath('../../../functions/trajectory_generation_functions')
end

%% Bridge - Long Travel
PeakVelocity_bridge = 0.266666666;                         % [m/s]
MaxAcceleration_bridge = 0.067;                            % [m/s^2]
q0_bridge = 0;                                             % [m]
qf_bridge = 2;                                             % [m]
samplingTime = 0.001;
[q_bridge,qd_bridge,qdd_bridge,time_bridge] = trapVelTraj(samplingTime, MaxAcceleration_bridge, PeakVelocity_bridge, q0_bridge, qf_bridge);

%% Trolley - Cross Travel
PeakVelocity_trolley = 0.266666666;                        % [m/s]
MaxAcceleration_trolley = 0.089;                           % [m/s^2]
q0_trolley = 0;                                            % [m]
qf_trolley = 2;                                            % [m]
samplingTime = 0.001;
[q_trolley,qd_trolley,qdd_trolley,time_trolley] = trapVelTraj(samplingTime, MaxAcceleration_trolley, PeakVelocity_trolley, q0_trolley, qf_trolley);

%% Plot bridge velocity profile
figure('NumberTitle','off','Name','CMS Cavern overhead crane - Bridge Velocity Profile'); axis square
% sgtitle('CMS Cavern overhead crane - Bridge Velocity Profile','FontSize',15,'FontWeight','bold')
subplot(3,1,1);
plot(time_bridge, q_bridge); grid; title('Position'), xlabel('[s]'), ylabel('[m]'); xlim([time_bridge(1) time_bridge(end)])
subplot(3,1,2);
plot(time_bridge, qd_bridge); grid; title('Velocity'), xlabel('[s]'), ylabel('[m/s]'); xlim([time_bridge(1) time_bridge(end)])
subplot(3,1,3);
plot(time_bridge, qdd_bridge); grid; title('Acceleration'), xlabel('[s]'), ylabel('[m/s^2]'); xlim([time_bridge(1) time_bridge(end)])

%% Plot trolley velocity profile
figure('NumberTitle','off','Name','CMS Cavern overhead crane - Trolley Velocity Profile'); axis square
% sgtitle('CMS Cavern overhead crane - Trolley Velocity Profile','FontSize',15,'FontWeight','bold')
subplot(3,1,1);
plot(time_trolley, q_trolley); grid; title('Position'), xlabel('[s]'), ylabel('[m]'); xlim([time_trolley(1) time_trolley(end)])
subplot(3,1,2);
plot(time_trolley, qd_trolley); grid; title('Velocity'), xlabel('[s]'), ylabel('[m/s]'); xlim([time_trolley(1) time_trolley(end)])
subplot(3,1,3);
plot(time_trolley, qdd_trolley); grid; title('Acceleration'), xlabel('[s]'), ylabel('[m/s^2]'); xlim([time_trolley(1) time_trolley(end)])

%% Plot all together 
figure('NumberTitle','off','Name','CMS Cavern overhead crane - Motion profile','WindowState','Maximized')
% sgtitle('CMS Cavern overhead crane - Trolley Velocity Profile','FontSize',15,'FontWeight','bold')
subplot(3,2,1);
plot(time_bridge, q_bridge); grid; title('Position (Bridge)'), xlabel('[s]'), ylabel('[m]'); xlim([time_bridge(1) time_bridge(end)])
subplot(3,2,3);
plot(time_bridge, qd_bridge); grid; title('Velocity (Bridge)'), xlabel('[s]'), ylabel('[m/s]'); xlim([time_bridge(1) time_bridge(end)])
subplot(3,2,5);
plot(time_bridge, qdd_bridge); grid; title('Acceleration (Bridge)'), xlabel('[s]'), ylabel('[m/s^2]'); xlim([time_bridge(1) time_bridge(end)])
subplot(3,2,2);
plot(time_trolley, q_trolley); grid; title('Position (Trolley)'), xlabel('[s]'), ylabel('[m]'); xlim([time_trolley(1) time_trolley(end)])
subplot(3,2,4);
plot(time_trolley, qd_trolley); grid; title('Velocity (Trolley)'), xlabel('[s]'), ylabel('[m/s]'); xlim([time_trolley(1) time_trolley(end)])
subplot(3,2,6);
plot(time_trolley, qdd_trolley); grid; title('Acceleration (Trolley)'), xlabel('[s]'), ylabel('[m/s^2]'); xlim([time_trolley(1) time_trolley(end)])


