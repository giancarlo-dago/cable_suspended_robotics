close all
clear
clc

addpath('../../../../data/cranebot')

% Read file and save data
filename = '2021-06-29-01.csv';
M = readmatrix(filename);

% Build the time vector
numberOfSamples = M(end,1)-M(1,1);
samplingTime = 0.01;
time = 0 : samplingTime : samplingTime*numberOfSamples;

% From sperical coordinates to cartesian coordinates
H = M(:,2)./100;                                            % Angle about the horizontal axis of the sensor
V = M(:,3)./100;                                            % Angle about the vertical axis of the sensor
D = M(:,4);                                                 % Radial distance from the sensor
[X,Y,Z] = sph2cart(deg2rad(V),deg2rad(H),D);                % From spherical coordinates to cartesian coordinates

% From [mm] to [m] 
X = X./1000;                                    
Y = Y./1000;
Z = Z./1000;

% Plots Position over the time (X, Y and Z)
figure('WindowState','Maximized');
% sgtitle('Movement of the overhead crane (Bridge) - Cartesian coordinates','FontWeight','bold')
subplot(3,1,1);
plot(time,X); grid; xlim([time(1) time(end)]); title('X'); xlabel('[s]'); ylabel('[m]'); title('X Position','FontWeight','bold','FontSize',14)
subplot(3,1,2);
plot(time,Y); grid; xlim([time(1) time(end)]); title('Y'); xlabel('[s]'); ylabel('[m]'); title('Y Position','FontWeight','bold','FontSize',14)
subplot(3,1,3);
plot(time,Z); grid; xlim([time(1) time(end)]); title('Z'); xlabel('[s]'); ylabel('[m]'); title('Z Position','FontWeight','bold','FontSize',14)

% Plots Position over the time (X and Y)
figure('WindowState','Maximized');
% sgtitle('Movement of the overhead crane (Bridge) - Cartesian coordinates','FontWeight','bold')
subplot(2,1,1);
plot(time,X); grid; xlim([time(1) time(end)]); title('X'); xlabel('[s]'); ylabel('[m]'); title('X Position','FontWeight','bold','FontSize',14)
subplot(2,1,2);
plot(time,Y); grid; xlim([time(1) time(end)]); title('Y'); xlabel('[s]'); ylabel('[m]'); title('Y Position','FontWeight','bold','FontSize',14)

% Plot spherical coordinates over time
figure('WindowState','Maximized');
% sgtitle('Movement of the overhead crane (Bridge) - Spherical coordinates','FontWeight','bold')
subplot(3,1,1);
plot(time,V); grid; xlim([time(1) time(end)]); title('V'); xlabel('Time [s]'); ylabel('Azimuth [deg]'); title('Azimuth','FontWeight','normal','FontSize',14)
subplot(3,1,2);
plot(time,H); grid; xlim([time(1) time(end)]); title('H'); xlabel('Time [s]'); ylabel('Elevation [deg]'); title('Elevation','FontWeight','normal','FontSize',14)
subplot(3,1,3);
plot(time,D); grid; xlim([time(1) time(end)]); title('D'); xlabel('Time [s]'); ylabel('Radius [m]'); title('Radius','FontWeight','normal','FontSize',14)

% Plot X-Y in the last part of the trajectory (to show the oscillation plane)
figure('WindowState','Maximized')
index = find(time>17.4,1,'first');
plot(X(index:end,:),Y(index:end,:)); grid;
title('X-Y Plot','FontSize',14)
axis([2 2.45 -0.2 0.25]); axis square
xlabel('X [m]'); ylabel('Y [m]')

% % Animated Plot X-Y 
% figure('WindowState','Maximized')
% h = animatedline('LineStyle','none','Marker','o','MarkerSize',10,'MaximumNumPoint',5); grid; 
% title('Movement of the overhead crane (Bridge) - Animation','FontSize',14);
% xlabel('X Position [m]'); ylabel('Y Position [m]')
% axis([1 8 -3.5 3.5]); axis square
% 
% for k = 1:length(X)
%     addpoints(h,X(k),Y(k));
%     drawnow
% end

% From sperical coordinates to cartesian coordinates
Q0 = M(:,10);
Q1 = M(:,11);
Q2 = M(:,12);
Q3 = M(:,13);
E = rad2deg(quat2eul([Q0 Q1 Q2 Q3]));

% Plot Euler angles over time
figure('WindowState','Maximized');
subplot(3,1,1);
plot(time,E(:,1)); grid; xlim([time(1) time(end)]); title('E_z'); xlabel('[s]'); ylabel('[deg]'); title('Euler Z','FontWeight','bold','FontSize',14)
subplot(3,1,2);
plot(time,E(:,2)); grid; xlim([time(1) time(end)]); title('E_y'); xlabel('[s]'); ylabel('[deg]'); title('Euler Y','FontWeight','bold','FontSize',14)
subplot(3,1,3);
plot(time,E(:,3)); grid; xlim([time(1) time(end)]); title('E_x'); xlabel('[s]'); ylabel('[deg]'); title('Euler X','FontWeight','bold','FontSize',14)


