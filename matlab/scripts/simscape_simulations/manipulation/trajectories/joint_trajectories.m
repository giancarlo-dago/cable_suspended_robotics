
Nc = 100;                                   % Numero di campioni al secondo
time_traj(:,1) = 1/Nc : 1/Nc : T_traj;
[Q_traj,QD_traj,QDD_traj] = jtraj(q0,qf,time_traj);

time_regime(:,1) = time_traj(end)+1/Nc : 1/Nc : time_traj(end)+T_regime;
[Q_reg,QD_reg,QDD_reg] = jtraj(qf,qf,time_regime);

Q = [Q_traj' Q_reg']';
QD = [QD_traj' QD_reg']';
QDD = [QDD_traj' QDD_reg']';

time = [time_traj' time_regime']';

q_ref = timeseries(Q, time);
qdot_ref = timeseries(QD, time);
qddot_ref = timeseries(QDD, time);

% Plot
figure(1);
subplot(3,1,1); plot(time,Q(:,1),'b'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$q(t) [rad]$','Interpreter','Latex'); legend('q_{ref}'); grid;
subplot(3,1,2); plot(time,QD(:,1),'b'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$\dot{q}(t) [\frac{rad}{s}]$','Interpreter','Latex');legend('qd_{ref}'); grid; 
subplot(3,1,3); plot(time,QDD(:,1),'b'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$\ddot{q}(t) [\frac{rad}{s^2}]$','Interpreter','Latex'); legend('qdd_{ref}'); grid;

figure(2);
subplot(3,1,1); plot(time,Q(:,2),'b'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$q(t) [rad]$','Interpreter','Latex'); legend('q_{ref}'); grid;
subplot(3,1,2); plot(time,QD(:,2),'b'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$\dot{q}(t) [\frac{rad}{s}]$','Interpreter','Latex');legend('qd_{ref}'); grid; 
subplot(3,1,3); plot(time,QDD(:,2),'b'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$\ddot{q}(t) [\frac{rad}{s^2}]$','Interpreter','Latex'); legend('qdd_{ref}'); grid;

figure(3);
subplot(3,1,1); plot(time,Q(:,3),'b'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$q(t) [rad]$','Interpreter','Latex'); legend('q_{ref}'); grid;
subplot(3,1,2); plot(time,QD(:,3),'b'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$\dot{q}(t) [\frac{rad}{s}]$','Interpreter','Latex');legend('qd_{ref}'); grid; 
subplot(3,1,3); plot(time,QDD(:,3),'b'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$\ddot{q}(t) [\frac{rad}{s^2}]$','Interpreter','Latex'); legend('qdd_{ref}'); grid;

figure(4);
subplot(3,1,1); plot(time,Q(:,4),'b'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$q(t) [rad]$','Interpreter','Latex'); legend('q_{ref}'); grid;
subplot(3,1,2); plot(time,QD(:,4),'b'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$\dot{q}(t) [\frac{rad}{s}]$','Interpreter','Latex');legend('qd_{ref}'); grid; 
subplot(3,1,3); plot(time,QDD(:,4),'b'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$\ddot{q}(t) [\frac{rad}{s^2}]$','Interpreter','Latex'); legend('qdd_{ref}'); grid;

figure(5);
subplot(3,1,1); plot(time,Q(:,5),'b'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$q(t) [rad]$','Interpreter','Latex'); legend('q_{ref}'); grid;
subplot(3,1,2); plot(time,QD(:,5),'b'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$\dot{q}(t) [\frac{rad}{s}]$','Interpreter','Latex');legend('qd_{ref}'); grid; 
subplot(3,1,3); plot(time,QDD(:,5),'b'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$\ddot{q}(t) [\frac{rad}{s^2}]$','Interpreter','Latex'); legend('qdd_{ref}'); grid;

figure(6);
subplot(3,1,1); plot(time,Q(:,6),'b'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$q(t) [rad]$','Interpreter','Latex'); legend('q_{ref}'); grid;
subplot(3,1,2); plot(time,QD(:,6),'b'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$\dot{q}(t) [\frac{rad}{s}]$','Interpreter','Latex');legend('qd_{ref}'); grid; 
subplot(3,1,3); plot(time,QDD(:,6),'b'); xlabel('$t [s]$','Interpreter','Latex'); ylabel('$\ddot{q}(t) [\frac{rad}{s^2}]$','Interpreter','Latex'); legend('qdd_{ref}'); grid;
