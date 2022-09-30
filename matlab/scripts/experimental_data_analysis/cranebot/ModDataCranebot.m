close all
clear
clc

addpath('../../../../data/cranebot')

%% Experiment #1

% Read file and save data
M = readmatrix('2021-06-29-01.csv');

% Build the time vector
numberOfSamples = M(end,1)-M(1,1);
samplingTime = 0.01;
time = (0 : samplingTime : samplingTime*numberOfSamples);

% Manipulate data
initialCutTime = 15.54;
initialCutTime = 17.16;
finalCutTime = 140.69;
index1 = find(time>=initialCutTime,1,'first');
index2 = find(time>=finalCutTime,1,'first');
time = time(index1:index2)-time(index1);

% From sperical coordinates to cartesian coordinates
H = M(index1:index2,2)./100;                                            % Angle about the horizontal axis of the sensor
V = M(index1:index2,3)./100;                                            % Angle about the vertical axis of the sensor
D = M(index1:index2,4);                                                 % Radial distance from the sensor
[X,Y,Z] = sph2cart(deg2rad(V),deg2rad(H),D);                            % From spherical coordinates to cartesian coordinates

% From [mm] to [m]
X = X./1000 - 2.23706;
Y = Y./1000 - 0.0355304;
Z = Z./1000 - 0.0432746;

% Compute velocity
for i=1:length(X)-1
    dX(i) = (X(i+1)-X(i))/samplingTime;
    dY(i) = (Y(i+1)-Y(i))/samplingTime;
    dZ(i) = (Z(i+1)-Z(i))/samplingTime;
end
dX(length(X)) = dX(length(X)-1); dY(length(X)) = dY(length(X)-1); dZ(length(X)) = dZ(length(X)-1);
dX = dX'; dY = dY'; dZ = dZ';

% From sperical coordinates to cartesian coordinates
Q0 = M(index1:index2,10);
Q1 = M(index1:index2,11);
Q2 = M(index1:index2,12);
Q3 = M(index1:index2,13);
E = rad2deg(quat2eul([Q0 Q1 Q2 Q3]));
E(:,1) = E(:,1) - 71.5294;
E(:,2) = E(:,2) + 0.05;
E(:,3) = E(:,3) + 89.435;

% Compute angular velocity
for i=1:length(X)-1
    dE(i,1) = (E(i+1,1)-E(i,1))/samplingTime;
    dE(i,2) = (E(i+1,2)-E(i,2))/samplingTime;
    dE(i,3) = (E(i+1,3)-E(i,3))/samplingTime;
end
dE(length(X),1) = dE(length(X)-1,1); dE(length(X),2) = dE(length(X)-1,2); dE(length(X),3) = dE(length(X)-1,3);

% Plots Position over the time (X, Y and Z)
figure('WindowState','Maximized');
% sgtitle('Movement of the overhead crane (Bridge) - Cartesian coordinates','FontWeight','bold')
subplot(3,1,1);
plot(time,X); grid; xlim([time(1) time(end)]); xlabel('[s]'); ylabel('[m]'); title('X Position','FontWeight','bold','FontSize',14)
subplot(3,1,2);
plot(time,Y); grid; xlim([time(1) time(end)]); xlabel('[s]'); ylabel('[m]'); title('Y Position','FontWeight','bold','FontSize',14)
subplot(3,1,3);
plot(time,Z); grid; xlim([time(1) time(end)]); xlabel('[s]'); ylabel('[m]'); title('Z Position','FontWeight','bold','FontSize',14)

% Plots Velocity over the time (dX, dY and dZ)
figure('WindowState','Maximized');
% sgtitle('Movement of the overhead crane (Bridge) - Cartesian coordinates','FontWeight','bold')
subplot(3,1,1);
plot(time,dX); grid; xlim([time(1) time(end)]); xlabel('[s]'); ylabel('[m/s]'); title('X Velocity','FontWeight','bold','FontSize',14)
subplot(3,1,2);
plot(time,dY); grid; xlim([time(1) time(end)]); xlabel('[s]'); ylabel('[m/s]'); title('Y Velocity','FontWeight','bold','FontSize',14)
subplot(3,1,3);
plot(time,dZ); grid; xlim([time(1) time(end)]); xlabel('[s]'); ylabel('[m/s]'); title('Z Velocity','FontWeight','bold','FontSize',14)

% Plot Euler angles over time
figure('WindowState','Maximized');
subplot(3,1,1);
plot(time,E(:,1)); grid; xlim([time(1) time(end)]); xlabel('[s]'); ylabel('[deg]'); title('Euler Z','FontWeight','bold','FontSize',14)
subplot(3,1,2);
plot(time,E(:,2)); grid; xlim([time(1) time(end)]); xlabel('[s]'); ylabel('[deg]'); title('Euler Y','FontWeight','bold','FontSize',14)
subplot(3,1,3);
plot(time,E(:,3)); grid; xlim([time(1) time(end)]); xlabel('[s]'); ylabel('[deg]'); title('Euler X','FontWeight','bold','FontSize',14)

% Plot Angular Velocity over time
figure('WindowState','Maximized');
subplot(3,1,1);
plot(time,dE(:,1)); grid; xlim([time(1) time(end)]); xlabel('[s]'); ylabel('[deg/s]'); title('Euler Velocity Z','FontWeight','bold','FontSize',14)
subplot(3,1,2);
plot(time,dE(:,2)); grid; xlim([time(1) time(end)]); xlabel('[s]'); ylabel('[deg/s]'); title('Euler Velocity Y','FontWeight','bold','FontSize',14)
subplot(3,1,3);
plot(time,dE(:,3)); grid; xlim([time(1) time(end)]); xlabel('[s]'); ylabel('[deg/s]'); title('Euler Velocity X','FontWeight','bold','FontSize',14)

% Print on file
fileID = fopen(append('dataMod1.txt'),'w');
for j=1:length(time)
    fprintf(fileID,'%8.3f ',time(j));
    fprintf(fileID,'%8.3f ',X(j));
    fprintf(fileID,'%8.3f ',Y(j));
    fprintf(fileID,'%8.3f ',Z(j));
    fprintf(fileID,'%8.3f ',H(j));
    fprintf(fileID,'%8.3f ',V(j));
    fprintf(fileID,'%8.3f ',D(j));
    fprintf(fileID,'%8.3f ',E(j,1));
    fprintf(fileID,'%8.3f ',E(j,2));
    fprintf(fileID,'%8.3f ',E(j,3));
    fprintf(fileID,'%8.3f ',dX(j));
    fprintf(fileID,'%8.3f ',dY(j));
    fprintf(fileID,'%8.3f ',dZ(j));
    fprintf(fileID,'%8.3f ',dE(j,1));
    fprintf(fileID,'%8.3f ',dE(j,2));
    fprintf(fileID,'%8.3f ',dE(j,3));
    fprintf(fileID,'\n');
end
fclose(fileID);
% clear

%% Experiment #2

% Read file and save data
M = readmatrix('2021-06-29-02.csv');

% Build the time vector
numberOfSamples = M(end,1)-M(1,1);
samplingTime = 0.01;
time = (0 : samplingTime : samplingTime*numberOfSamples);

% Manipulate data
initialCutTime = 16.78;
finalCutTime = 137.59;
index1 = find(time>=initialCutTime,1,'first');
index2 = find(time>=finalCutTime,1,'first');
time = time(index1:index2)-time(index1);

% From sperical coordinates to cartesian coordinates
H = M(index1:index2,2)./100;                                            % Angle about the horizontal axis of the sensor
V = M(index1:index2,3)./100;                                            % Angle about the vertical axis of the sensor
D = M(index1:index2,4);                                                 % Radial distance from the sensor
[X,Y,Z] = sph2cart(deg2rad(V),deg2rad(H),D);                % From spherical coordinates to cartesian coordinates

% From [mm] to [m]
X = X./1000 - 2.01643;
Y = Y./1000 - 0.0320667;
Z = Z./1000 - 0.0390441;

% Compute velocity
for i=1:length(X)-1
    dX(i) = (X(i+1)-X(i))/samplingTime;
    dY(i) = (Y(i+1)-Y(i))/samplingTime;
    dZ(i) = (Z(i+1)-Z(i))/samplingTime;
end
dX(length(X)) = dX(length(X)-1); dY(length(X)) = dY(length(X)-1); dZ(length(X)) = dZ(length(X)-1);
dX = dX'; dY = dY'; dZ = dZ';

% From sperical coordinates to cartesian coordinates
Q0 = M(index1:index2,10);
Q1 = M(index1:index2,11);
Q2 = M(index1:index2,12);
Q3 = M(index1:index2,13);
E = rad2deg(quat2eul([Q0 Q1 Q2 Q3]));
E(:,1) = E(:,1) - 71.56;
E(:,2) = E(:,2) + 0.06;
E(:,3) = E(:,3) + 89.4306;

% Compute angular velocity
dE(1,1) = 0; dE(1,1) = 0; dE(1,1) = 0;
for i=1:length(X)-1
    dE(i+1,1) = (E(i+1,1)-E(i,1))/samplingTime;
    dE(i+1,2) = (E(i+1,2)-E(i,2))/samplingTime;
    dE(i+1,3) = (E(i+1,3)-E(i,3))/samplingTime;
end

% Plots Position over the time (X, Y and Z)
figure('WindowState','Maximized');
% sgtitle('Movement of the overhead crane (Bridge) - Cartesian coordinates','FontWeight','bold')
subplot(3,1,1);
plot(time,X); grid; xlim([time(1) time(end)]); title('X'); xlabel('[s]'); ylabel('[m]'); title('X Position','FontWeight','bold','FontSize',14)
subplot(3,1,2);
plot(time,Y); grid; xlim([time(1) time(end)]); title('Y'); xlabel('[s]'); ylabel('[m]'); title('Y Position','FontWeight','bold','FontSize',14)
subplot(3,1,3);
plot(time,Z); grid; xlim([time(1) time(end)]); title('Z'); xlabel('[s]'); ylabel('[m]'); title('Z Position','FontWeight','bold','FontSize',14)

% Plots Velocity over the time (dX, dY and dZ)
figure('WindowState','Maximized');
% sgtitle('Movement of the overhead crane (Bridge) - Cartesian coordinates','FontWeight','bold')
subplot(3,1,1);
plot(time,dX); grid; xlim([time(1) time(end)]); title('dX'); xlabel('[s]'); ylabel('[m/s]'); title('X Velocity','FontWeight','bold','FontSize',14)
subplot(3,1,2);
plot(time,dY); grid; xlim([time(1) time(end)]); title('dY'); xlabel('[s]'); ylabel('[m/s]'); title('Y Velocity','FontWeight','bold','FontSize',14)
subplot(3,1,3);
plot(time,dZ); grid; xlim([time(1) time(end)]); title('dZ'); xlabel('[s]'); ylabel('[m/s]'); title('Z Velocity','FontWeight','bold','FontSize',14)

% Plot Euler angles over time
figure('WindowState','Maximized');
subplot(3,1,1);
plot(time,E(:,1)); grid; xlim([time(1) time(end)]); title('E_z'); xlabel('[s]'); ylabel('[deg]'); title('Euler Z','FontWeight','bold','FontSize',14)
subplot(3,1,2);
plot(time,E(:,2)); grid; xlim([time(1) time(end)]); title('E_y'); xlabel('[s]'); ylabel('[deg]'); title('Euler Y','FontWeight','bold','FontSize',14)
subplot(3,1,3);
plot(time,E(:,3)); grid; xlim([time(1) time(end)]); title('E_x'); xlabel('[s]'); ylabel('[deg]'); title('Euler X','FontWeight','bold','FontSize',14)

% Plot Angular Velocity over time
figure('WindowState','Maximized');
subplot(3,1,1);
plot(time,dE(:,1)); grid; xlim([time(1) time(end)]); xlabel('[s]'); ylabel('[deg/s]'); title('Euler Velocity Z','FontWeight','bold','FontSize',14)
subplot(3,1,2);
plot(time,dE(:,2)); grid; xlim([time(1) time(end)]); xlabel('[s]'); ylabel('[deg/s]'); title('Euler Velocity Y','FontWeight','bold','FontSize',14)
subplot(3,1,3);
plot(time,dE(:,3)); grid; xlim([time(1) time(end)]); xlabel('[s]'); ylabel('[deg/s]'); title('Euler Velocity X','FontWeight','bold','FontSize',14)

% Print on file
fileID = fopen(append('dataMod1.txt'),'w');
for j=1:length(time)
    fprintf(fileID,'%8.3f ',time(j));
    fprintf(fileID,'%8.3f ',X(j));
    fprintf(fileID,'%8.3f ',Y(j));
    fprintf(fileID,'%8.3f ',Z(j));
    fprintf(fileID,'%8.3f ',H(j));
    fprintf(fileID,'%8.3f ',V(j));
    fprintf(fileID,'%8.3f ',D(j));
    fprintf(fileID,'%8.3f ',E(j,1));
    fprintf(fileID,'%8.3f ',E(j,2));
    fprintf(fileID,'%8.3f ',E(j,3));
    fprintf(fileID,'%8.3f ',dX(j));
    fprintf(fileID,'%8.3f ',dY(j));
    fprintf(fileID,'%8.3f ',dZ(j));
    fprintf(fileID,'%8.3f ',dE(j,1));
    fprintf(fileID,'%8.3f ',dE(j,2));
    fprintf(fileID,'%8.3f ',dE(j,3));
    fprintf(fileID,'\n');
end
fclose(fileID);
clear

%% Experiment #4

M = readmatrix(append('2021-06-29-04.csv'));

% Build the time vector
numberOfSamples = M(end,1) - M(1,1);
samplingTime = 0.01;
time = (0 : samplingTime : samplingTime*numberOfSamples)';

% Manipulate data
initialCutTime = 11.69;
finalCutTime = 145.79;
index1 = find(time>=initialCutTime,1,'first');
index2 = find(time>=finalCutTime,1,'first');
time = time(index1:index2)-time(index1);

% From sperical coordinates to cartesian coordinates
H = M(index1:index2,2)./100;                                            % Angle about the horizontal axis of the sensor
V = M(index1:index2,3)./100;                                            % Angle about the vertical axis of the sensor
D = M(index1:index2,4);                                                 % Radial distance from the sensor
[X,Y,Z] = sph2cart(deg2rad(V),deg2rad(H),D);                            % From spherical coordinates to cartesian coordinates

% From [mm] to [m] 
X = X./1000;
Y = Y./1000;
Z = Z./1000;

% From sperical coordinates to cartesian coordinates
Q0 = M(index1:index2,10);
Q1 = M(index1:index2,11);
Q2 = M(index1:index2,12);
Q3 = M(index1:index2,13);
E = rad2deg(quat2eul([Q0 Q1 Q2 Q3]));  % Euler ZYX


% % Print on file
% fileID = fopen(append('dataMod4.txt'),'w');
% for j=1:length(time)
%     fprintf(fileID,'%8.3f ',time(j));
%     fprintf(fileID,'%8.3f ',X(j)-2.598);
%     fprintf(fileID,'%8.3f ',Y(j)-0.039);
%     fprintf(fileID,'%8.3f ',Z(j));
%     fprintf(fileID,'%8.3f ',H(j));
%     fprintf(fileID,'%8.3f ',V(j));
%     fprintf(fileID,'%8.3f ',D(j));
%     fprintf(fileID,'%8.3f ',E(j,1)-71.594);
%     fprintf(fileID,'%8.3f ',E(j,2));
%     fprintf(fileID,'%8.3f ',E(j,3)+89.4237);
%     fprintf(fileID,'\n');
% end
% fclose(fileID);
clear

%% Experiment #6

% Read file and save data
M = readmatrix('2021-06-29-06.csv');

% Build the time vector
numberOfSamples = M(end,1)-M(1,1);
samplingTime = 0.01;
time = (0 : samplingTime : samplingTime*numberOfSamples);

% % Manipulate data
initialCutTime = 18.4 + 2.02;
finalCutTime = 607.89;
index1 = find(time>=initialCutTime,1,'first');
index2 = find(time>=finalCutTime,1,'first');
time = time(index1:index2)-time(index1);

% From sperical coordinates to cartesian coordinates
H = M(index1:index2,2)./100;                                            % Angle about the horizontal axis of the sensor
V = M(index1:index2,3)./100;                                            % Angle about the vertical axis of the sensor
D = M(index1:index2,4);                                                 % Radial distance from the sensor
[X,Y,Z] = sph2cart(deg2rad(V),deg2rad(H),D);                % From spherical coordinates to cartesian coordinates

% From [mm] to [m]
X = X./1000 - 2.59546;
Y = Y./1000 - 0.0411612;
Z = Z./1000 + 0.0442166;

% Compute velocity
for i=1:length(X)-1
    dX(i) = (X(i+1)-X(i))/samplingTime;
    dY(i) = (Y(i+1)-Y(i))/samplingTime;
    dZ(i) = (Z(i+1)-Z(i))/samplingTime;
end
dX(length(X)) = dX(length(X)-1); dY(length(X)) = dY(length(X)-1); dZ(length(X)) = dZ(length(X)-1);
dX = dX'; dY = dY'; dZ = dZ';

% From sperical coordinates to cartesian coordinates
Q0 = M(index1:index2,10);
Q1 = M(index1:index2,11);
Q2 = M(index1:index2,12);
Q3 = M(index1:index2,13);
E = rad2deg(quat2eul([Q0 Q1 Q2 Q3]));
E(:,1) = E(:,1) + 79.677;
E(:,2) = E(:,2) + 0.0356;
E(:,3) = E(:,3) + 89.742;

% Compute angular velocity
dE(1,1) = 0; dE(1,1) = 0; dE(1,1) = 0;
for i=1:length(X)-1
    dE(i+1,1) = (E(i+1,1)-E(i,1))/samplingTime;
    dE(i+1,2) = (E(i+1,2)-E(i,2))/samplingTime;
    dE(i+1,3) = (E(i+1,3)-E(i,3))/samplingTime;
end

% Plots Position over the time (X, Y and Z)
figure('WindowState','Maximized');
% sgtitle('Movement of the overhead crane (Bridge) - Cartesian coordinates','FontWeight','bold')
subplot(3,1,1);
plot(time,X); grid; xlim([time(1) time(end)]); title('X'); xlabel('[s]'); ylabel('[m]'); title('X Position','FontWeight','bold','FontSize',14)
subplot(3,1,2);
plot(time,Y); grid; xlim([time(1) time(end)]); title('Y'); xlabel('[s]'); ylabel('[m]'); title('Y Position','FontWeight','bold','FontSize',14)
subplot(3,1,3);
plot(time,Z); grid; xlim([time(1) time(end)]); title('Z'); xlabel('[s]'); ylabel('[m]'); title('Z Position','FontWeight','bold','FontSize',14)

% Plots Velocity over the time (dX, dY and dZ)
figure('WindowState','Maximized');
% sgtitle('Movement of the overhead crane (Bridge) - Cartesian coordinates','FontWeight','bold')
subplot(3,1,1);
plot(time,dX); grid; xlim([time(1) time(end)]); title('dX'); xlabel('[s]'); ylabel('[m/s]'); title('X Velocity','FontWeight','bold','FontSize',14)
subplot(3,1,2);
plot(time,dY); grid; xlim([time(1) time(end)]); title('dY'); xlabel('[s]'); ylabel('[m/s]'); title('Y Velocity','FontWeight','bold','FontSize',14)
subplot(3,1,3);
plot(time,dZ); grid; xlim([time(1) time(end)]); title('dZ'); xlabel('[s]'); ylabel('[m/s]'); title('Z Velocity','FontWeight','bold','FontSize',14)

% Plot Euler angles over time
figure('WindowState','Maximized');
subplot(3,1,1);
plot(time,E(:,1)); grid; xlim([time(1) time(end)]); title('E_z'); xlabel('[s]'); ylabel('[deg]'); title('Euler Z','FontWeight','bold','FontSize',14)
subplot(3,1,2);
plot(time,E(:,2)); grid; xlim([time(1) time(end)]); title('E_y'); xlabel('[s]'); ylabel('[deg]'); title('Euler Y','FontWeight','bold','FontSize',14)
subplot(3,1,3);
plot(time,E(:,3)); grid; xlim([time(1) time(end)]); title('E_x'); xlabel('[s]'); ylabel('[deg]'); title('Euler X','FontWeight','bold','FontSize',14)

% Plot Angular Velocity over time
figure('WindowState','Maximized');
subplot(3,1,1);
plot(time,dE(:,1)); grid; xlim([time(1) time(end)]); xlabel('[s]'); ylabel('[deg/s]'); title('Euler Velocity Z','FontWeight','bold','FontSize',14)
subplot(3,1,2);
plot(time,dE(:,2)); grid; xlim([time(1) time(end)]); xlabel('[s]'); ylabel('[deg/s]'); title('Euler Velocity Y','FontWeight','bold','FontSize',14)
subplot(3,1,3);
plot(time,dE(:,3)); grid; xlim([time(1) time(end)]); xlabel('[s]'); ylabel('[deg/s]'); title('Euler Velocity X','FontWeight','bold','FontSize',14)

% Print on file
fileID = fopen(append('dataMod6.txt'),'w');
for j=1:length(time)
    fprintf(fileID,'%8.3f ',time(j));
    fprintf(fileID,'%8.3f ',X(j));
    fprintf(fileID,'%8.3f ',Y(j));
    fprintf(fileID,'%8.3f ',Z(j));
    fprintf(fileID,'%8.3f ',H(j));
    fprintf(fileID,'%8.3f ',V(j));
    fprintf(fileID,'%8.3f ',D(j));
    fprintf(fileID,'%8.3f ',E(j,1));
    fprintf(fileID,'%8.3f ',E(j,2));
    fprintf(fileID,'%8.3f ',E(j,3));
    fprintf(fileID,'%8.3f ',dX(j));
    fprintf(fileID,'%8.3f ',dY(j));
    fprintf(fileID,'%8.3f ',dZ(j));
    fprintf(fileID,'%8.3f ',dE(j,1));
    fprintf(fileID,'%8.3f ',dE(j,2));
    fprintf(fileID,'%8.3f ',dE(j,3));
    fprintf(fileID,'\n');
end
fclose(fileID);
% clear

%% Experiment #7

M = readmatrix(append('2021-06-29-07.csv'));

% Build the time vector
numberOfSamples = M(end,1) - M(1,1);
samplingTime = 0.01;
time = (0 : samplingTime : samplingTime*numberOfSamples)';

% Manipulate data
initialCutTime = 7.67;
finalCutTime = 802.39;
index1 = find(time>=initialCutTime,1,'first');
index2 = find(time>=finalCutTime,1,'first');
time = time(index1:index2)-time(index1);

% From sperical coordinates to cartesian coordinates
H = M(index1:index2,2)./100;                                            % Angle about the horizontal axis of the sensor
V = M(index1:index2,3)./100;                                            % Angle about the vertical axis of the sensor
D = M(index1:index2,4);                                                 % Radial distance from the sensor
% [X,Y,Z] = sph2cart(deg2rad(V),deg2rad(H),D);                          % From spherical coordinates to cartesian coordinates
[Y,X,Z] = sph2cart(deg2rad(V),deg2rad(H),D);                            % From spherical coordinates to cartesian coordinates

% From [mm] to [m] 
X = X./1000;
Y = Y./1000;
Z = Z./1000;

% From sperical coordinates to cartesian coordinates
Q0 = M(index1:index2,10);
Q1 = M(index1:index2,11);
Q2 = M(index1:index2,12);
Q3 = M(index1:index2,13);
E = rad2deg(quat2eul([Q0 Q1 Q2 Q3]));  % Euler ZYX

% % Print on file
% fileID = fopen(append('dataMod7.txt'),'w');
% for j=1:length(time)
%     fprintf(fileID,'%8.3f ',time(j));
% %     fprintf(fileID,'%8.3f ',X(j)-4.544);
% %     fprintf(fileID,'%8.3f ',Y(j)-0.072);
%     fprintf(fileID,'%8.3f ',X(j)-0.072);
%     fprintf(fileID,'%8.3f ',Y(j)-4.544);
%     fprintf(fileID,'%8.3f ',Z(j));
%     fprintf(fileID,'%8.3f ',H(j));
%     fprintf(fileID,'%8.3f ',V(j));
%     fprintf(fileID,'%8.3f ',D(j));
% %     fprintf(fileID,'%8.3f ',E(j,1)+79.38);
% %     fprintf(fileID,'%8.3f ',E(j,2));
% %     fprintf(fileID,'%8.3f ',E(j,3)+89.7268);
%     fprintf(fileID,'%8.3f ',E(j,1)+79.38);
%     fprintf(fileID,'%8.3f ',E(j,3)+89.7268);
%     fprintf(fileID,'%8.3f ',E(j,2));
%     fprintf(fileID,'\n');
% end
% fclose(fileID);
clear
