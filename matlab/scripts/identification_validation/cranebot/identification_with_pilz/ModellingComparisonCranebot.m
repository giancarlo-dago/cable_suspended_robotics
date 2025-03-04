close all
clear
clc

% A = readmatrix('proposedModelCranebot.txt');
B = readmatrix('doublePendModelCranebot.txt');
C = readmatrix('singPendModelCranebot.txt');

figure()
% plot(time,[A(:,2) B(:,2) C(:,2)]),hold on
plot(C(:,1),[C(:,2)]),hold on
plot(B(:,1),[B(:,2)-B(1,2)-0.0267]),hold on

% time = 0:0.02:40-0.02;
% hold on
% plot(time,abs(movmean(A(:,2),5)-movmean(B(:,2),5)), 'k-', 'Linewidth', lw ,'Color', [0.6, 0.6, 0.6])
% plot(time,abs(movmean(A(:,2),5)-movmean(C(:,2),5)), 'k-', 'Linewidth', lw ,'Color', [0.4, 0.4, 0.4])
% plot(time,abs(movmean(A(:,2),5)-shoulderPoseDownsample), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2])
% 
% legend('$(i)$ Double pend.','$(ii)$ Single pend.','$(iii)$ Proposed','Orientation','horizontal'); 
% xlabel('t [s]'); ylabel('error [m]'); set(gca, 'FontSize',16); set(gca, 'XLim', [tModel{i}(j) 20]);
% set(gca, 'YLim', [-inf 0.075], 'YTick', [0:0.025:0.075]);
% grid on; box on; set(gcf,'color','w');
% 
% %% Computation max error
% doublePendMaxError = max(abs(movmean(A(:,2),5)-movmean(B(:,2),5)))
% singlePendMaxError = max(abs(movmean(A(:,2),5)-movmean(C(:,2),5)))
% proposedMaxError = max(abs(movmean(A(:,2),5)-shoulderPoseDownsample))
