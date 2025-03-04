close all
clear
clc

if ispc % Windows
    addpath('..\trajectories\')
    addpath('..\..\meshes\')
    addpath('..\..\..\..\functions\screw_theory_functions\')
    addpath('..\..\..\..\parameters\')
else % Linux
    addpath('../trajectories/')
    addpath('../../meshes/')
    addpath('../../../../functions/screw_theory_functions/')
    addpath('../../../../parameters/')
end

run('parameters_3R.m')

% Chain definition
n_links = 8;
n_ee = 2;
n_joints = 8;

% Dynamic parameters
g = [0 -g0 0]';
F_ee_A = zeros(6,1);
F_ee_B = zeros(6,1);

% Initial conditions
alfa_0 = 0;         % posizione iniziale primo giunto passivo
beta_0 = 0;         % posizione iniziale secondo giunto passivo
th1A_0 = 0;         % posizione iniziale giunto 1 manipolatore A
th2A_0 = -pi/4;     % posizione iniziale giunto 2 manipolatore A
th3A_0 = pi/2;      % posizione iniziale giunto 3 manipolatore A
th1B_0 = 0;         % posizione iniziale giunto 1 manipolatore B
th2B_0 = pi/4;      % posizione iniziale giunto 2 manipolatore B
th3B_0 = -pi/2;     % posizione iniziale giunto 3 manipolatore B

% Initial conditions
alfa_0 = 0;         % posizione iniziale primo giunto passivo
beta_0 = 0;         % posizione iniziale secondo giunto passivo
th1A_0 = -pi/6;         % posizione iniziale giunto 1 manipolatore A
th2A_0 = pi/6;         % posizione iniziale giunto 2 manipolatore A
th3A_0 = pi/6;         % posizione iniziale giunto 3 manipolatore A
th1B_0 = pi/6;         % posizione iniziale giunto 1 manipolatore B
th2B_0 = -pi/6;         % posizione iniziale giunto 2 manipolatore B
th3B_0 = -pi/6;         % posizione iniziale giunto 3 manipolatore B

% Kinematic parameters
omega1 = [0 0 1]';
omega2 = [0 0 1]';
omega3 = [0 0 1]';
omega4 = [0 0 1]';
omega5 = [0 0 1]';
omega6 = [0 0 1]';
omega7 = [0 0 1]';
omega8 = [0 0 1]';
q1 = [0 0 0]';
q2 = [0 -L 0]';
q3 = [offA -L-D 0]';
q4 = [offA -L-D-a1A 0]';
q5 = [offA -L-D-a1A-a2A 0]';
q6 = [offB -L-D 0]';
q7 = [offB -L-D-a1B 0]';
q8 = [offB -L-D-a1B-a2B 0]';

% Screw axis computation
omega = [omega1 omega2 omega3 omega4 omega5 omega6 omega7 omega8];
q = [q1 q2 q3 q4 q5 q6 q7 q8];
S = zeros(6,n_joints);
for i = 1:n_joints
    v = cross(-omega(:,i),q(:,i));
    S(:,i) = [omega(:,i); v];
end
 
% Computation M_{b,i}
M_b1 = [ 0  1  0  0; 
        -1  0  0 -L/2;
         0  0  1  0
         0  0  0  1];

M_b2 = [ 1  0  0  0; 
         0  1  0 -L;
         0  0  1  0
         0  0  0  1];

M_b3 = [ 0  1  0   offA; 
        -1  0  0 -L-D-l1A;
         0  0  1    0
         0  0  0    1];

M_b4 = [ 0  1  0   offA; 
        -1  0  0 -L-D-a1A-l2A;
         0  0  1    0
         0  0  0    1];
   
M_b5 = [ 0  1  0   offA; 
        -1  0  0 -L-D-a1A-a2A-l3A;
         0  0  1    0
         0  0  0    1];

M_b6 = [ 0  1  0   offB; 
        -1  0  0 -L-D-l1B;
         0  0  1    0
         0  0  0    1];

M_b7 = [ 0  1  0   offB; 
        -1  0  0 -L-D-a1B-l2B;
         0  0  1    0
         0  0  0    1];
   
M_b8 = [ 0  1  0   offB; 
        -1  0  0 -L-D-a1B-a2B-l3B;
         0  0  1    0
         0  0  0    1];

M_b9 = [ 0  1  0   offA; 
        -1  0  0 -L-D-a1A-a2A-a3A;
         0  0  1    0
         0  0  0    1];

M_b10 = [ 0  1  0   offB; 
         -1  0  0 -L-D-a1B-a2B-a3B;
          0  0  1    0
          0  0  0    1];

% Store datas in data-structures
M_bi(:,:,1) = M_b1;
M_bi(:,:,2) = M_b2;
M_bi(:,:,3) = M_b3;
M_bi(:,:,4) = M_b4;
M_bi(:,:,5) = M_b5;
M_bi(:,:,6) = M_b6;
M_bi(:,:,7) = M_b7;
M_bi(:,:,8) = M_b8;
M_bi(:,:,9) = M_b9;
M_bi(:,:,10) = M_b10;
Inertia(:,:,1) = I_cables;
Inertia(:,:,2) = I_platform;
Inertia(:,:,3) = Il1A;
Inertia(:,:,4) = Il2A;
Inertia(:,:,5) = Il3A;
Inertia(:,:,6) = Il1B;
Inertia(:,:,7) = Il2B;
Inertia(:,:,8) = Il3B;
mass = [m_cables m_platform ml1A ml2A ml3A ml1B ml2B ml3B];
inertial_disp(1,:) = inertial_disp_cables;
inertial_disp(2,:) = inertial_disp_platform;
inertial_disp(3,:) = inertial_disp_1A;
inertial_disp(4,:) = inertial_disp_2A;
inertial_disp(5,:) = inertial_disp_3A;
inertial_disp(6,:) = inertial_disp_1B;
inertial_disp(7,:) = inertial_disp_2B;
inertial_disp(8,:) = inertial_disp_3B;
friction = [fv1p fv2p fv1A fv2A fv3A fv1B fv2B fv3B];

% Store data regarding the frames in a struct
info = struct('n_joints', n_joints, ...
              'n_links', n_links, ...
              'n_ee', n_ee, ...
              'current_frame_index',    [1  2 3 4 5 6 7  8 9 10], ...
              'previous_frame_index',   [0  1 2 3 4 2 6  7 5  8], ...
              'next_frame_index',       [2 36 4 5 9 7 8 10 0  0], ...
              'next_frame_type',        [0  1 0 0 0 0 0  0 0  0], ...           % 0 for single frame / 1 for duplex next frame (frameA(1 digit) - frameB(1 digit)) / 2 for duplex next frame (frameA(1 digit) - frameB(2 digit))
              'previous_joint_index',   [1  2 3 4 5 6 7  8 0  0], ...
              'frame_type',             [1  1 1 1 1 1 1  1 0  0], ...           % 1 for link, 0 for ee
              'explored',               [0  0 0 0 0 0 0  0 0  0], ...           % 0 for unxeplored, 1 for explored 
              'S', S, ...
              'M_bi', M_bi, ...
              'Inertia', Inertia, ...
              'mass', mass, ...
              'inertial_disp', inertial_disp, ...
              'F', [zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) F_ee_A F_ee_B]);                                 

% Joint regulation
qr = [0 -pi/3 pi/4 0 pi/4 pi/8];
qr_dot = zeros(1,6);
qr_ddot = zeros(1,6);

% Joint tracking 
T_traj = 40;
T_regime = 10;
T = T_traj + T_regime;
q0 = [th1A_0 th2A_0 th3A_0 th1B_0 th2B_0 th3B_0];
qf = [0 -pi/4 pi/4 0 pi/4 -pi/8];
run('joint_trajectories');
close all;

% Task space reference (regulation)
% x_ref = [-0.1 -1.2 0.3 -1.2 0];
x_ref = [-0.25 -0.8 0.19 -0.9 0];
xdot_ref = [0 0 0 0 0];
xddot_ref = [0 0 0 0 0];

% End effectors tracking 
run('ee_trajectories_new'); 
close all;
dq0 = [0 0 0 0 0 0]'; 
q0 = [th1A_0 th2A_0 th3A_0 th1B_0 th2B_0 th3B_0]';

% Control Type 1
%(1: CLIK Classico)
%(2: CLIK with center of mass control)
%(3: CLIK with center of mass control and fyA+fyB minimization)
tipo_controllo = 1;

% Minimization Type
%(1: Minimizzazione forza orizzontale)
%(2: Minimizzazione forza verticale)
%(3: Minimizzazione somma forze orizzontali e somma forze verticali contemporaneamente)
tipo_minimizzazione = 4;

%(1: Minimizzazione f1Ax e f1Bx separatamente)
%(2: Minimizzazione somma f1Ax e f1Bx)
prova = 2;

% Simulation
Kd = 20*eye(6);
Kp = 10*eye(6);
Gd = diag([1000 1000 1000 1000 1000]);
Gp = diag([3000 3000 3000 3000 1000]);
Gd_nullspace = 10*eye(6);
Gp_nullspace = 10*eye(6);
sim('redundancy')
