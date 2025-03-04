close all
clear
clc

if ispc % Windows
    addpath('..\crane_motion\')
    addpath('..\..\..\functions\screw_theory_functions\')
    addpath('..\..\..\parameters\')
    addpath('..\..\..\data\T-Probe')
else % Linux
    addpath('../crane_motion/')
    addpath('../../../functions/trajectory_generation_functions/')
    addpath('../../../parameters/')
    addpath('../../../data/T-Probe')
end

run('cranebot_parameters.m')
run('cms_trolley_trajectory.m')

%% Simulation

% Initial configuration
d0_0 = 0;
alfa_0 = deg2rad(0);                                        % [rad]
beta_0 = deg2rad(0);                                        % [rad]

% Initial configuration vector
q_0 = [d0_0 alfa_0 beta_0];

% Kinematic parameters
a1 = L;                                                     % [m]
a2 = D+L1+L2+L3;                                            % [m]
l1 = 2.474;                                                 % [m]
l2 = 0.674;                                                 % [m]
% l1 = 2.9;                                                 % [m]
% l2 = 0.5;                                                 % [m]

% Dynamic parameters
m1 = m_cables;                                              % [kg]
m2 = m_platform + 2*(ml1+ml2+ml3+ml4+ml5+ml6);              % [kg]
g0 = 9.81;                                                  % [m/s^2]
% I_cm_1 = diag([0 0 35.64]);                               % [kg*m^2]
% I_cm_2 = diag([0 0 30.366057265]);                        % [kg*m^2]
I_cm_0 = diag([0 0 0]);                                     % [kg*m^2]
I_cm_1 = diag([35.64 35.64 35.64]);                         % [kg*m^2]
I_cm_2 = diag([30.366 30.366 30.366]);                      % [kg*m^2]
% I_cm_1 = diag([40 40 40]);                                % [kg*m^2]
% I_cm_2 = diag([30.366 30.366 30.366]);                    % [kg*m^2]

% Friction
fv0 = 0;
fs0 = 0;
fv1 = 35;                                                   
fs1 = 0.001;
fv2 = 0;                                                   
fs2 = 0;

% Gravity vector and e-e force
g = [0 -g0 0]';
F_ee = zeros(6,1);

% Kinematic parameters
omega0 = [0 0 0]';
omega1 = [0 0 1]';
omega2 = [0 0 1]';
v0 = [1 0 0]';
q1 = [0 0 0]';
q2 = [0 -a1 0]';

% Screw axis computation
v1 = cross(-omega1,q1);
v2 = cross(-omega2,q2);
S0 = [omega0; v0];
S1 = [omega1; v1];
S2 = [omega2; v2];

% Computation M_{b,i}
M_b0 = [ 1  0  0  0; 
         0  1  0  0;
         0  0  1  0;
         0  0  0  1];

M_b1 = [ 1  0  0  0; 
         0  1  0  0;
         0  0  1  0;
         0  0  0  1];

M_b2 = [ 1  0  0  0; 
         0  1  0  -a1;
         0  0  1  0;
         0  0  0  1];

M_be = [ 1  0  0  0; 
         0  1  0  -a1-a2;
         0  0  1  0;
         0  0  0  1];
     
% Chain definition
n_joints = 3;

% Store datas in data-structures
S(:,1) = S0;
S(:,2) = S1;
S(:,3) = S2;
M_bi(:,:,1) = M_b0;
M_bi(:,:,2) = M_b1;
M_bi(:,:,3) = M_b2;
M_bi(:,:,4) = M_be;
Inertia(:,:,1) = I_cm_0;
Inertia(:,:,2) = I_cm_1;
Inertia(:,:,3) = I_cm_2;
inertial_disp(1,:) = [0 0 0];
inertial_disp(2,:) = [0 -l1 0];
inertial_disp(3,:) = [0 -l2 0];
viscous_friction = [fv0 fv1 fv2];
static_friction = [fs0 fs1 fs2];
mass = [m_crane m1 m2];

% Store data regarding the frames in a struct
info = struct('n_joints', n_joints, ...
              'S', S, ...
              'M_bi', M_bi, ...
              'Inertia', Inertia, ...
              'mass', mass, ...
              'inertial_disp', inertial_disp);

% Simulation
T = 800;
sim('uncontrolled_simulink.slx')

% Read simulation
timeSimulation = ans.sim_time.Data;
dataSimulation = ans.sim_alfabeta.Data;
alphaSimulation = dataSimulation(:,1);
betaSimulation = dataSimulation(:,2);

%% Plot comparison

figure('NumberTitle','off','Name','Alpha');
plot(timeSimulation,alphaSimulation); grid;
xlabel('[s]');
ylabel('[deg]');
legend('Measured','Simulated')
xlim([timeSimulation(1) timeSimulation(end)])

figure('NumberTitle','off','Name','Beta ');
plot(timeSimulation,betaSimulation); grid;
xlabel('[s]');
ylabel('[deg]');
legend('Measured','Simulated')
xlim([timeSimulation(1) timeSimulation(end)])

% %% FFT simulation
% L = length(alphaSimulation);
% Fs = 1/samplingTime;
% 
% Y = fft(alphaSimulation);
% P2 = abs(Y/L);
% P1 = P2(1:L/32+1);
% P1(2:end-1) = 2*P1(2:end-1);
% f = Fs*(0:(L/32))/L;
% 
% figure();
% plot(f,P1); grid; hold on;
% title('Single-Sided Amplitude Spectrum of X(t)')
% xlabel('f [Hz]')
% ylabel('|P(f)|')
% xlim([f(1) f(end)])
% 
% %% FFT measurements
% L = length(data);
% Fs = 1/samplingTime;
% 
% Y = fft(data);
% P2 = abs(Y/L);
% P1 = P2(1:L/32+1);
% P1(2:end-1) = 2*P1(2:end-1);
% f = Fs*(0:(L/32))/L;
% 
% % figure();
% plot(f,P1)
% % title('Single-Sided Amplitude Spectrum of X(t)')
% xlabel('f [Hz]')
% ylabel('|P(f)|')
% xlim([f(1) f(end)])
% 
% legend('sim','meas')
% 
