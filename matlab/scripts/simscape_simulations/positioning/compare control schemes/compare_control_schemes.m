close all
clear
clc

%% Load
uncontolled_qpassive = load('qpassive_uncontrolled.mat');
noncollocated_qpassive = load('qpassive_noncollocated.mat');
taskspace_qpassive = load('qpassive_taskspace.mat');
noncollocated_qd = load('qd_noncollocated.mat');
taskspace_qd = load('qd_taskspace.mat');
noncollocated_tau = load('tau_noncollocated.mat');
taskspace_tau = load('tau_taskspace.mat');

qpassive_uncontrolled = uncontolled_qpassive.qp.Data;
qpassive_noncollocated = noncollocated_qpassive.qp.Data;
qpassive_taskspace = taskspace_qpassive.qp.Data;
qd_noncollocated = noncollocated_qd.qd.Data;
qd_taskspace = taskspace_qd.qd.Data;
tau_noncollocated = noncollocated_tau.tau.Data;
tau_taskspace = taskspace_tau.tau.Data;

time_uncontrolled = uncontolled_qpassive.qp.Time;
time_noncollocated = noncollocated_qpassive.qp.Time;
time_taskspace = taskspace_qpassive.qp.Time;

%% Plot alfa uncontrolled vs taskspace
figure(1);
% plot(time_uncontrolled, qpassive_uncontrolled(:,1)); hold on;
% plot(time_taskspace, qpassive_taskspace(:,1)*180/pi);
% legend('$\alpha$ uncontrolled','$\alpha$ task-space','Interpreter','Latex','Location','northeast','AutoUpdate','off')
% % plot(time_taskspace, 0.01*ones(1,length(time_taskspace)),'k-.');
% % plot(time_taskspace, -0.01*ones(1,length(time_taskspace)),'k-.');
% xlabel('[s]'); ylabel('[deg]'); xlim([time_uncontrolled(1) 500]); ylim([-1.2 1.2])
% grid;
[yupper1,ylower1] = envelope(qpassive_uncontrolled(:,1),1,'peak');hold on; grid;
[yupper2,ylower2] = envelope(qpassive_taskspace(:,1)*180/pi,1,'peak');
plot(time_uncontrolled,yupper1,'b'); 
plot(time_taskspace,yupper2,'r');
legend('$\alpha$ uncontrolled','$\alpha$ task-space','Interpreter','Latex','Location','northeast','AutoUpdate','off')
plot(time_uncontrolled,ylower1,'b'); 
plot(time_taskspace,ylower2,'r'); 
xlabel('[s]'); ylabel('[deg]'); xlim([time_taskspace(1) 500]); ylim([-1.2 1.2])


%% Plot alfa uncontrolled vs noncollocated
figure(2);
% plot(time_uncontrolled, qpassive_uncontrolled(:,1)); hold on;
% plot(time_noncollocated, qpassive_noncollocated(:,1)*180/pi);
% legend('$\alpha$ uncontrolled','$\alpha$ noncollocated','Interpreter','Latex','Location','northeast','AutoUpdate','off')
% % plot(time_noncollocated, 0.01*ones(1,length(time_noncollocated)),'k-.');
% % plot(time_noncollocated, -0.01*ones(1,length(time_noncollocated)),'k-.');
% xlabel('[s]'); ylabel('[deg]'); xlim([time_noncollocated(1) 500]); ylim([-1.2 1.2])
% grid;
% figure(80)
[yupper1,ylower1] = envelope(qpassive_uncontrolled(:,1),30,'peak');hold on; grid;
[yupper2,ylower2] = envelope(qpassive_noncollocated(:,1)*180/pi,30,'peak');
plot(time_uncontrolled,yupper1,'b'); 
plot(time_noncollocated,yupper2,'r');
legend('$\alpha$ uncontrolled','$\alpha$ noncollocated','Interpreter','Latex','Location','northeast','AutoUpdate','off')
plot(time_noncollocated,ylower2,'r'); 
plot(time_uncontrolled,ylower1,'b'); 
xlabel('[s]'); ylabel('[deg]'); xlim([time_noncollocated(1) 500]); ylim([-1.2 1.2])



%% Plot alfa taskspace vs noncollocated
figure(3);
plot(time_taskspace, qpassive_taskspace(:,1)*180/pi); hold on;
plot(time_noncollocated, qpassive_noncollocated(:,1)*180/pi);
legend('$\alpha$ task-space','$\alpha$ noncollocated','Interpreter','Latex','Location','northeast')
xlabel('[s]'); ylabel('[deg]'); xlim([time_uncontrolled(1) 500]); ylim([-1.2 1.2])
grid;

%% Plot beta uncontrolled vs taskspace
figure(4);
% plot(time_uncontrolled, qpassive_uncontrolled(:,2)); hold on;
% plot(time_taskspace, qpassive_taskspace(:,2)*180/pi+0.0273298);
% legend('$\beta$ uncontrolled','$\beta$ task-space','Interpreter','Latex','Location','northeast')
% xlabel('[s]'); ylabel('[deg]'); xlim([time_uncontrolled(1) 500]); ylim([-0.5 0.5])
% grid;
[yupper1,ylower1] = envelope(qpassive_uncontrolled(:,2),250,'peak'); hold on; grid;
[yupper2,ylower2] = envelope(qpassive_taskspace(:,2)*180/pi+0.0273298,250,'peak');
plot(time_uncontrolled,yupper1,'b'); 
plot(time_taskspace,yupper2,'r');
legend('$\beta$ uncontrolled','$\beta$ task-space','Interpreter','Latex','Location','northeast','AutoUpdate','off')
plot(time_uncontrolled,ylower1,'b'); 
plot(time_taskspace,ylower2,'r'); 
xlabel('[s]'); ylabel('[deg]'); xlim([time_uncontrolled(1) 500]); ylim([-0.5 0.5])

%% Plot beta uncontrolled vs noncollocated
figure(5);
% plot(time_uncontrolled, qpassive_uncontrolled(:,2)); hold on;
% plot(time_noncollocated, qpassive_noncollocated(:,2)*180/pi+0.0273298);
% legend('$\beta$ uncontrolled','$\beta$ noncollocated','Interpreter','Latex','Location','northeast')
% xlabel('[s]'); ylabel('[deg]'); xlim([time_uncontrolled(1) 500]); ylim([-0.5 0.5])
% grid;
[yupper1,ylower1] = envelope(qpassive_uncontrolled(:,2),280,'peak'); hold on; grid;
[yupper2,ylower2] = envelope(qpassive_noncollocated(:,2)*180/pi+0.0273298,280,'peak');
plot(time_uncontrolled,yupper1,'b'); 
plot(time_noncollocated,yupper2,'r');
legend('$\beta$ uncontrolled','$\beta$ task-space','Interpreter','Latex','Location','northeast','AutoUpdate','off')
plot(time_uncontrolled,ylower1,'b'); 
plot(time_noncollocated,ylower2,'r'); 
xlabel('[s]'); ylabel('[deg]'); xlim([time_uncontrolled(1) 500]); ylim([-0.5 0.5])

%% Plot alfa taskspace vs noncollocated
figure(6);
plot(time_taskspace, qpassive_taskspace(:,2)*180/pi); hold on;
plot(time_noncollocated, qpassive_noncollocated(:,2)*180/pi);
legend('$\beta$ task-space','$\beta$ noncollocated','Interpreter','Latex','Location','northeast')
xlabel('[s]'); ylabel('[deg]'); xlim([time_uncontrolled(1) 500]); ylim([-0.5 0.5])
grid;

%% Plot joint velocities taskspace
figure(7);
% subplot(2,1,1); plot(noncollocated_qd.qd.Time, qd_noncollocated(:,1:6)); grid
% subplot(2,1,2); plot(noncollocated_qd.qd.Time, qd_noncollocated(:,7:12)); grid;
plot(taskspace_qd.qd.Time, qd_taskspace.*0.35); grid; 
legend('$\dot{\theta}_{1A}$','$\dot{\theta}_{2A}$','$\dot{\theta}_{3A}$','$\dot{\theta}_{4A}$','$\dot{\theta}_{5A}$','$\dot{\theta}_{6A}$','$\dot{\theta}_{1B}$','$\dot{\theta}_{2B}$','$\dot{\theta}_{3B}$','$\dot{\theta}_{4B}$','$\dot{\theta}_{5B}$','$\dot{\theta}_{6B}$','Interpreter','Latex','Location','northeast','NumColumns',2)
xlabel('[s]'); ylabel('[deg]'); xlim([time_taskspace(1) 500]); ylim([-72 72])

%% Plot joint velocities noncollocated
figure(8);
% subplot(2,1,1); plot(noncollocated_qd.qd.Time, qd_noncollocated(:,1:6)); grid
% subplot(2,1,2); plot(noncollocated_qd.qd.Time, qd_noncollocated(:,7:12)); grid;
plot(noncollocated_qd.qd.Time, qd_noncollocated.*0.45); grid; 
legend('$\dot{\theta}_{1A}$','$\dot{\theta}_{2A}$','$\dot{\theta}_{3A}$','$\dot{\theta}_{4A}$','$\dot{\theta}_{5A}$','$\dot{\theta}_{6A}$','$\dot{\theta}_{1B}$','$\dot{\theta}_{2B}$','$\dot{\theta}_{3B}$','$\dot{\theta}_{4B}$','$\dot{\theta}_{5B}$','$\dot{\theta}_{6B}$','Interpreter','Latex','Location','northeast','NumColumns',2)
xlabel('[s]'); ylabel('[deg]'); xlim([time_noncollocated(1) 500]); ylim([-72 72])

%% Plot joint torques taskspace
figure(9);
% subplot(2,1,1); plot(noncollocated_qd.qd.Time, qd_noncollocated(:,1:6)); grid
% subplot(2,1,2); plot(noncollocated_qd.qd.Time, qd_noncollocated(:,7:12)); grid;
plot(taskspace_tau.tau.Time, tau_taskspace.*0.55); grid; 
legend('$\tau_{1A}$','$\tau_{2A}$','$\tau_{3A}$','$\tau_{4A}$','$\tau_{5A}$','$\tau_{6A}$','$\tau_{1B}$','$\tau_{2B}$','$\tau_{3B}$','$\tau_{4B}$','$\tau_{5B}$','$\tau_{6B}$','Interpreter','Latex','Location','northeast','NumColumns',2)
xlabel('[s]'); ylabel('[Nm]'); xlim([time_taskspace(1) 500]); ylim([-10 10])

%% Plot joint torques taskspace
figure(10);
% subplot(2,1,1); plot(noncollocated_qd.qd.Time, qd_noncollocated(:,1:6)); grid
% subplot(2,1,2); plot(noncollocated_qd.qd.Time, qd_noncollocated(:,7:12)); grid;
plot(noncollocated_tau.tau.Time, tau_noncollocated.*0.55); grid; 
legend('$\tau_{1A}$','$\tau_{2A}$','$\tau_{3A}$','$\tau_{4A}$','$\tau_{5A}$','$\tau_{6A}$','$\tau_{1B}$','$\tau_{2B}$','$\tau_{3B}$','$\tau_{4B}$','$\tau_{5B}$','$\tau_{6B}$','Interpreter','Latex','Location','northeast','NumColumns',2)
xlabel('[s]'); ylabel('[Nm]'); xlim([time_noncollocated(1) 500]); ylim([-12 12])