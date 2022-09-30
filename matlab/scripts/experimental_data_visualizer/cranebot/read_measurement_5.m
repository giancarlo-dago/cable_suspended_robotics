close all
clear
clc

addpath('../../../data/cranebot')

%%
filename = '2021-06-29-05.csv';
M = readmatrix(filename);

numberOfSamples = M(end,1)-M(1,1);
samplingTime = 0.01;
time = 0 : samplingTime : samplingTime*numberOfSamples;

H = deg2rad(M(:,2)./100);
V = deg2rad(M(:,3)./100);
D = M(:,4);
tilt = M(:,9);
[X,Y,Z] = sph2cart(H,V,D);


figure('WindowState','Maximized');
plot(time,X); grid; xlim([time(1) time(end)]); title('X')
figure('WindowState','Maximized');
plot(time,Y); grid; xlim([time(1) time(end)]); title('Y')
figure('WindowState','Maximized');
plot(time,Z); grid; xlim([time(1) time(end)]); title('Z')
figure('WindowState','Maximized');
plot(time,rad2deg(H)); grid; xlim([time(1) time(end)]); title('H')
figure('WindowState','Maximized');
plot(time,rad2deg(V)); grid; xlim([time(1) time(end)]); title('V')
figure('WindowState','Maximized');
plot(time,D); grid; xlim([time(1) time(end)]); title('D')


% figure('WindowState','Maximized')
% h = animatedline('LineStyle','none','Marker','o','MaximumNumPoint',3); grid; 
% title('Test 1 - Movement of the overhead crane - Bridge');
% xlabel('X Position [m]'); ylabel('Y Position [m]')
% axis([-3.5 3.5 1 8]); axis square
% 
% for k = 1:length(X)
%     addpoints(h,X(k)./1000,Y(k)./1000);
%     drawnow
% end



