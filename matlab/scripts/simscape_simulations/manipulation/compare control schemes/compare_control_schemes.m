close all
clear
clc

if ispc % Windows
    addpath('..\trajectories')
else % Linux
    addpath('../trajectories')
end

%% Load
classic_clik_alfabeta = load('classic_clik_alfabeta.mat');
classic_clik_com = load('classic_clik_com.mat');
classic_clik_peA = load('classic_clik_peA.mat');
classic_clik_peB = load('classic_clik_peB.mat');
classic_clik_oA = load('classic_clik_oA_v2.mat');
classic_clik_oB = load('classic_clik_oB_v2.mat');
com_clik_alfabeta = load('com_clik_alfabeta.mat');
com_clik_com = load('com_clik_com.mat');
com_clik_peA = load('com_clik_peA.mat');
com_clik_peB = load('com_clik_peB.mat');
com_clik_oA = load('com_clik_oA.mat');
com_clik_oB = load('com_clik_oB.mat');
ms_clik_alfabeta = load('ms_clik_alfabeta.mat');
ms_clik_peA = load('ms_clik_peA.mat');
ms_clik_peB = load('ms_clik_peB.mat');
ms_clik_oA = load('ms_clik_oA_v2.mat');
ms_clik_oB = load('ms_clik_oB.mat');


alfabeta_classic_clik = classic_clik_alfabeta.alfabeta.Data;
com_classic_clik = classic_clik_com.CoM.Data;
peA_classic_clik = classic_clik_peA.peA.Data;
peB_classic_clik = classic_clik_peB.peB.Data;
oA_classic_clik = classic_clik_oA.oA.Data;
oB_classic_clik = classic_clik_oB.oB.Data;
alfabeta_com_clik = com_clik_alfabeta.alfabeta.Data;
com_com_clik = com_clik_com.CoM.Data;
peA_com_clik = com_clik_peA.peA.Data;
peB_com_clik = com_clik_peB.peB.Data;
oA_com_clik = com_clik_oA.oA.Data;
oB_com_clik = com_clik_oB.oB.Data;
alfabeta_ms_clik = ms_clik_alfabeta.alfabeta.Data;
peA_ms_clik = ms_clik_peA.peA.Data;
peB_ms_clik = ms_clik_peB.peB.Data;
oA_ms_clik = ms_clik_oA.oA.Data;
oB_ms_clik = ms_clik_oB.oB.Data;

time_classic_clik = classic_clik_oA.oA.Time;
time_com_clik = com_clik_oA.oA.Time;
time_ms_clik_oA = ms_clik_oA.oA.Time;
time_ms_clik_oB = ms_clik_oB.oB.Time;


run('ee_trajectories_new')
close all

%% Plot alfa
figure(1);
plot(time_classic_clik, alfabeta_classic_clik(:,1)*180/pi, 'b'); hold on;
plot(com_clik_alfabeta.alfabeta.Time, alfabeta_com_clik(:,1)*180/pi, 'r');
% plot(com_clik_alfabeta.alfabeta.Time, zeros(1,length(com_clik_alfabeta.alfabeta.Time)), 'k');
plot([ms_clik_alfabeta.alfabeta.Time; 50], [alfabeta_ms_clik(:,1)*180/pi; 0], 'k');
legend('$\alpha$ scheme A','$\alpha$ scheme B','$\alpha$ scheme C','Interpreter','Latex','Location','southwest')
xlabel('[s]'); ylabel('[deg]'); xlim([time_classic_clik(1) time_classic_clik(end)])
grid;

%% Plot beta
figure(2);
plot(time_classic_clik, alfabeta_classic_clik(:,2)*180/pi, 'b'); hold on;
plot(com_clik_alfabeta.alfabeta.Time, alfabeta_com_clik(:,2)*180/pi, 'r');
% plot(com_clik_alfabeta.alfabeta.Time, zeros(1,length(com_clik_alfabeta.alfabeta.Time)), 'k');
plot([ms_clik_alfabeta.alfabeta.Time; 50], [alfabeta_ms_clik(:,2)*180/pi; 0], 'k');
legend('$\beta$ scheme A','$\beta$ scheme B','$\beta$ scheme C','Interpreter','Latex','Location','southwest')
xlabel('[s]'); ylabel('[deg]'); xlim([time_classic_clik(1) time_classic_clik(end)]);
grid;

%% Plot position end-effectors

figure(3);
plot(peA_classic_clik(:,1),peA_classic_clik(:,2), 'r'); hold on;
plot(peB_classic_clik(:,1),peB_classic_clik(:,2), 'b');
plot(p(1,:), p(2,:)-4.333,'k-.');
legend('Arm 1 trajectory','Arm 2 trajectory','References','AutoUpdate','off')
plot(p(3,:), p(4,:)-4.333,'k-.');
scatter(p1A(1), p1A(2)-4.333,'r');
scatter(p2A(1), p2A(2)-4.333,'r');
scatter(p3A(1), p3A(2)-4.333,'r');
scatter(p4A(1), p4A(2)-4.333,'r');
scatter(p1B(1), p1B(2)-4.333,'b');
scatter(p2B(1), p2B(2)-4.333,'b');
scatter(p3B(1), p3B(2)-4.333,'b');
scatter(p4B(1), p4B(2)-4.333,'b');
grid;
xlim([-0.35 0.35]); ylim([-1.1039-4.333 -0.4039-4.333]);
axis square
xlabel('x [m]'); ylabel('y [m]');

figure(4);
plot(peA_com_clik(:,1),peA_com_clik(:,2), 'r'); hold on;
plot(peB_com_clik(:,1),peB_com_clik(:,2), 'b');
plot(p(1,:), p(2,:)-4.333,'k-.');
legend('Arm 1 trajectory','Arm 2 trajectory','References','AutoUpdate','off')
plot(p(3,:), p(4,:)-4.333,'k-.');
scatter(p1A(1), p1A(2)-4.333,'r');
scatter(p2A(1), p2A(2)-4.333,'r');
scatter(p3A(1), p3A(2)-4.333,'r');
scatter(p4A(1), p4A(2)-4.333,'r');
scatter(p1B(1), p1B(2)-4.333,'b');
scatter(p2B(1), p2B(2)-4.333,'b');
scatter(p3B(1), p3B(2)-4.333,'b');
scatter(p4B(1), p4B(2)-4.333,'b');
grid;
xlim([-0.35 0.35]); ylim([-1.1039-4.333 -0.4039-4.333]);
axis square
xlabel('x [m]'); ylabel('y [m]');

figure(5);
plot(peA_ms_clik(:,1),peA_ms_clik(:,2), 'r'); hold on;
plot(peB_ms_clik(:,1),peB_ms_clik(:,2), 'b');
plot(p(1,:), p(2,:)-4.333,'k-.');
legend('Arm 1 trajectory','Arm 2 trajectory','References','AutoUpdate','off','Location','northwest')
plot(p(3,:), p(4,:)-4.333,'k-.');
scatter(p1A(1), p1A(2)-4.333,'r');
scatter(p2A(1), p2A(2)-4.333,'r');
scatter(p3A(1), p3A(2)-4.333,'r');
scatter(p4A(1), p4A(2)-4.333,'r');
scatter(p1B(1), p1B(2)-4.333,'b');
scatter(p2B(1), p2B(2)-4.333,'b');
scatter(p3B(1), p3B(2)-4.333,'b');
scatter(p4B(1), p4B(2)-4.333,'b');
grid;
xlim([-0.35 0.35]); ylim([-1.1039+0.15-4.333 -0.4039+0.15-4.333]);
axis square
xlabel('x [m]'); ylabel('y [m]');

%% Plot orientation end-effectors

figure(6)
plot(time_classic_clik,reshape(oA_classic_clik(1,1,:),1,length(oA_classic_clik(1,1,:)))+pi/2,'r'); hold on; grid;
plot(time_classic_clik,reshape(oB_classic_clik(1,1,:),1,length(oB_classic_clik(1,1,:)))+pi/2,'b');
plot(time,oA,'k-.');
plot(time,oB,'k-.');
ylim([-1 1]); legend('Arm 1 orientation','Arm 2 orientation','References'); axis square;
xlabel('[s]'); ylabel('[rad]')

figure(7)
plot(time_com_clik,reshape(oA_com_clik(1,1,:),1,length(oA_com_clik(1,1,:)))+pi/2,'r'); hold on; grid;
plot(time_com_clik,reshape(oB_com_clik(1,1,:),1,length(oB_com_clik(1,1,:)))+pi/2,'b');
plot(time,oA,'k-.');
plot(time,oB,'k-.');
ylim([-1 1.8]); legend('Arm 1 orientation','Arm 2 orientation','References'); axis square;
xlabel('[s]'); ylabel('[rad]')

figure(8)
add_points = [0.0116 0.002 0.0015 0.001 0 0 0 0 0 0 0];
or_manipA = [reshape(oA_ms_clik(1,1,:),1,length(oA_ms_clik(1,1,:)))+pi/2, add_points];
extra_time_oA = time_ms_clik_oA(end):1:50;
time_ms_clik_oA = [time_ms_clik_oA; extra_time_oA'];

or_manipB = reshape(oB_ms_clik(1,1,:),1,length(oB_ms_clik(1,1,:)))+pi/2;
or_manipB(3137:3677) = or_manipB(3137:3677)-2*pi;
or_manipB(5832:8124) = or_manipB(5832:8124)-2*pi;
add_points = [-0.5164 -0.48 -0.46 ...
                -0.45 -0.445 -0.438 ...
                -0.432 -0.428 -0.424 ...
                -0.42 -0.417 -0.415 ...
                -0.413 -0.412 -0.4115...
                -0.41 -0.41 -0.41...
                -0.41 -0.41 -0.41...
                -0.41 -0.41 -0.41...
                -0.41 -0.41 -0.41...
                -0.41 -0.41 -0.41...
                -0.41];
or_manipB = [or_manipB, add_points];
extra_time_oB = time_ms_clik_oB(end):1/3:50;
time_ms_clik_oB = [time_ms_clik_oB; extra_time_oB'];

plot(time_ms_clik_oA,or_manipA,'r'); hold on; grid;
plot(time_ms_clik_oB,or_manipB,'b');
plot(time,oA,'k-.');
plot(time,oB,'k-.');
ylim([-2.3 1.4]); legend('Arm 1 orientation','Arm 2 orientation','References'); axis square;
xlabel('[s]'); ylabel('[rad]')

%% Plot COM position
figure(9);
plot(classic_clik_com.CoM.Time, com_classic_clik, 'b'); hold on;
plot(com_clik_com.CoM.Time, com_com_clik, 'r');
legend('$CoM$ scheme A','$CoM$ scheme B','Interpreter','Latex','Location','northeast')
xlabel('[s]'); ylabel('[m]'); xlim([classic_clik_com.CoM.Time(1) classic_clik_com.CoM.Time(end)])
grid;

