close all
clear
clc

if ispc % Windows
    addpath('..\..\..\meshes\')
    addpath('..\..\..\..\..\functions\screw_theory_functions\')
    addpath('..\..\..\..\..\parameters\')
else % Linux
    addpath('../../../meshes/')
    addpath('../../../../../functions/screw_theory_functions/')
    addpath('../../../../../parameters/')
end

run('cranebot_parameters.m')

% Chain definition
n_joints = 6;

% Dynamic parameters
g = [0 0 g0]';                                                  % N.B. questo vettore di gravità è riferito ripetto alla frame di base del robot, non del mondo (quindi abbiamo [0 0 g0] e non [0 0 -g0])
F_ee_A = zeros(6,1);
F_ee_B = zeros(6,1);

% Initial conditions
th1A_0 = deg2rad(0);
th2A_0 = deg2rad(0);
th3A_0 = deg2rad(0);
th4A_0 = deg2rad(0);
th5A_0 = deg2rad(0);
th6A_0 = deg2rad(0);
th1B_0 = deg2rad(0);
th2B_0 = deg2rad(0);
th3B_0 = deg2rad(0);
th4B_0 = deg2rad(0);
th5B_0 = deg2rad(0);
th6B_0 = deg2rad(0);

% Kinematic parameters
omega1 = [0 0 1]';
omega2 = [0 -1 0]';
omega3 = [0 1 0]';
omega4 = [0 0 1]';
omega5 = [0 1 0]';
omega6 = [0 0 1]';
q1 = [0 0 0]';
q2 = [0 0 0]';
q3 = [0 0 L1]';
q4 = [0 0 L1]';
q5 = [0 0 L1+L2]';
q6 = [0 0 L1+L2]';

% Screw axis computation
omega = [omega1 omega2 omega3 omega4 omega5 omega6];
q = [q1 q2 q3 q4 q5 q6];
S_A = zeros(6,n_joints);
for i = 1:n_joints
    v = cross(-omega(:,i),q(:,i));
    S_A(:,i) = [omega(:,i); v];
end
S_B = S_A;
S = [S_A S_B];

% Computation M_{b,i}
M_b1 = eye(4);
    
M_b2 = [1  0  0  0;
        0  0 -1  0;
        0  1  0  0;
        0  0  0  1];

M_b3 = [0  1  0  0;
        0  0  1  0;
        1  0  0  L1;
        0  0  0  1];
    
M_b4 = [1  0  0  0;
        0  1  0  0;
        0  0  1  L1;
        0  0  0  1];
    
M_b5 = [1  0  0      0;
        0  0  1      0;
        0 -1  0  L1+L2;
        0  0  0     1];
    
M_b6 = [1  0  0      0;
        0  1  0      0;
        0  0  1  L1+L2;
        0  0  0     1];
    
M_be = [1  0  0         0;
        0  1  0         0;
        0  0  1  L1+L2+L3;
        0  0  0        1];
 
% Store datas in data-structures
M_bi_A(:,:,1) = M_b1;
M_bi_A(:,:,2) = M_b2;
M_bi_A(:,:,3) = M_b3;
M_bi_A(:,:,4) = M_b4;
M_bi_A(:,:,5) = M_b5;
M_bi_A(:,:,6) = M_b6;
M_bi_A(:,:,7) = M_be;
M_bi_B = M_bi_A;
M_bi(:,:,1:7) = M_bi_A;
M_bi(:,:,8:14) = M_bi_B;
Inertia(:,:,1) = Il1;
Inertia(:,:,2) = Il2;
Inertia(:,:,3) = Il3;
Inertia(:,:,4) = Il4;
Inertia(:,:,5) = Il5;
Inertia(:,:,6) = Il6;
Inertia(:,:,7) = Il1;
Inertia(:,:,8) = Il2;
Inertia(:,:,9) = Il3;
Inertia(:,:,10) = Il4;
Inertia(:,:,11) = Il5;
Inertia(:,:,12) = Il6;
mass = [ml1 ml2 ml3 ml4 ml5 ml6 ml1 ml2 ml3 ml4 ml5 ml6];
inertial_disp(1,:) = inertial_disp_1;
inertial_disp(2,:) = inertial_disp_2;         
inertial_disp(3,:) = inertial_disp_3;
inertial_disp(4,:) = inertial_disp_4;         
inertial_disp(5,:) = inertial_disp_5;         
inertial_disp(6,:) = inertial_disp_6;  
inertial_disp(7,:) = inertial_disp_1;
inertial_disp(8,:) = inertial_disp_2;         
inertial_disp(9,:) = inertial_disp_3;
inertial_disp(10,:) = inertial_disp_4;         
inertial_disp(11,:) = inertial_disp_5;         
inertial_disp(12,:) = inertial_disp_6; 
friction = [fv1 fv2 fv3 fv4 fv5 fv6 fv1 fv2 fv3 fv4 fv5 fv6];
F_ee = [F_ee_A F_ee_B];

% Simulation
T = 20;
Kd = 5*eye(12);
Kp = 2*eye(12);
q_ref = [pi/2 pi/4 pi/2 -pi/2 pi/2 -pi/8 pi/2 pi/4 pi/2 -pi/2 pi/2 -pi/8];
qd_ref = zeros(1,12);
qdd_ref = zeros(1,12);
sim('double_lwa4p_platform')
