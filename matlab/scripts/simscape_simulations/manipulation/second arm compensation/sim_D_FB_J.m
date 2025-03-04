close all
clear
clc

if ispc % Windows
    addpath('..\trajectories\')
else % Linux
    addpath('../trajectories/')
end

% Condizioni iniziali
alfa_0 = -pi/2;      % posizione iniziale primo giunto passivo
beta_0 = pi/2;       % posizione iniziale secondo giunto passivo
th1A_0 = -pi/2;      % posizione iniziale giunto 1 manipolatore A 
th2A_0 = -pi/8;      % posizione iniziale giunto 2 manipolatore A
th3A_0 = -pi/8;      % posizione iniziale giunto 3 manipolatore A
th1B_0 = -pi/2;      % posizione iniziale giunto 1 manipolatore B
th2B_0 = pi/8;       % posizione iniziale giunto 2 manipolatore B
th3B_0 = pi/8;       % posizione iniziale giunto 3 manipolatore B

q0 = [th1A_0; th2A_0; th3A_0; th1B_0; th2B_0; th3B_0];
qf = [-pi/2; -pi/2; pi/2; -pi/2; pi/4; pi/4];

% Parametri
g0 = -9.81;
ml0 = 1000;          % Massa carroponte (kg)
m_cavo = 45;         % Massa cavo (kg)
m_base = 100;        % Massa base (kg)
ml1A = 5;            % Massa link 1 manipolatore A (kg)
ml2A = 5;            % Massa link 2 manipolatore A (kg)
ml3A = 5;            % Massa link 3 manipolatore A (kg)
ml1B = 5;            % Massa link 1 manipolatore B (kg)
ml2B = 5;            % Massa link 2 manipolatore B (kg)
ml3B = 5;            % Massa link 3 manipolatore B (kg)
L = 3;               % Lunghezza cavo (m)
a1A = 0.5;           % Lunghezza link 1 manipolatore A (m)
a2A = 0.5;           % Lunghezza link 2 manipolatore A (m)
a3A = 0.5;           % Lunghezza link 3 manipolatore A (m)
a1B = 0.5;           % Lunghezza link 1 manipolatore B (m)
a2B = 0.5;           % Lunghezza link 2 manipolatore B (m)
a3B = 0.5;           % Lunghezza link 3 manipolatore B (m)
offA = -0.25;        % Posizionamento manipolatore A sulla base (m)
offB = 0.25;         % Posizionamento manipolatore B sulla base (m)
fv1p = 0;            % Coeff. attrito viscoso primo giunto passivo (N/(m/s))
fv2p = 0;            % Coeff. attrito viscoso primo giunto passivo(N/(m/s))
fv1A = 0;            % Coeff. attrito viscoso giunto 1 A (N/(m/s))
fv2A = 0;            % Coeff. attrito viscoso giunto 2 A (N/(m/s))
fv3A = 0;            % Coeff. attrito viscoso giunto 3 A (N/(m/s))
fv1B = 0;            % Coeff. attrito viscoso giunto 1 B (N/(m/s))
fv2B = 0;            % Coeff. attrito viscoso giunto 2 B (N/(m/s))
fv3B = 0;            % Coeff. attrito viscoso giunto 3 B (N/(m/s))
Iczz = 33.7594;      % kg*m^2
Ibzz = 12.1875;      % kg*m^2
Il1Azz = 0.108333;   % kg*m^2
Il2Azz = 0.108333;   % kg*m^2
Il3Azz = 0.108333;   % kg*m^2
Il1Bzz = 0.108333;   % kg*m^2
Il2Bzz = 0.108333;   % kg*m^2
Il3Bzz = 0.108333;   % kg*m^2

% Simulation Time
T_traj = 5;
T_regime = 5;
T = T_traj + T_regime;

% Controller gains
Kd = 5*eye(6);
Kp = 5*eye(6);

% Simulate
run('joint_trajectories'); close all;
sim('D_FB_J_controlled')
q_controlled = ans.q_controlled;
pause(10)
sim('D_FB_J_simpleinvdyn')
q_uncontrolled = ans.q_uncontrolled;

%% Plot

figure();
plot(q_controlled.Time, q_controlled.Data(:,1));
hold on;
plot(q_uncontrolled.Time, q_uncontrolled.Data(:,1));
legend('contr','uncontr');
title('alfa comparison')
grid;

figure();
plot(q_controlled.Time, q_controlled.Data(:,2));
hold on;
plot(q_uncontrolled.Time, q_uncontrolled.Data(:,2));
legend('contr','uncontr');
title('beta comparison')
grid;

figure();
subplot(2,3,1); plot(q_ref.Time, q_ref.Data(:,1)); hold on; plot(q_controlled.Time, q_controlled.Data(:,3)); legend('reference','effective'); title('th1A'); grid;
subplot(2,3,2); plot(q_ref.Time, q_ref.Data(:,2)); hold on; plot(q_controlled.Time, q_controlled.Data(:,4)); legend('reference','effective'); title('th2A'); grid;
subplot(2,3,3); plot(q_ref.Time, q_ref.Data(:,3)); hold on; plot(q_controlled.Time, q_controlled.Data(:,5)); legend('reference','effective'); title('th3A'); grid;
subplot(2,3,4); plot(q_ref.Time, q_ref.Data(:,4)); hold on; plot(q_controlled.Time, q_controlled.Data(:,6)); legend('reference','effective'); title('th1B'); grid;
subplot(2,3,5); plot(q_ref.Time, q_ref.Data(:,5)); hold on; plot(q_controlled.Time, q_controlled.Data(:,7)); legend('reference','effective'); title('th2B'); grid;
subplot(2,3,6); plot(q_ref.Time, q_ref.Data(:,6)); hold on; plot(q_controlled.Time, q_controlled.Data(:,8)); legend('reference','effective'); title('th3B');grid;
