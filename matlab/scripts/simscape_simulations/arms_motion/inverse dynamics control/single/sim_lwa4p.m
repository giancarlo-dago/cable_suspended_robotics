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
g = [0 0 -g0]';
F_ee = zeros(6,1);

% Initial conditions
th1_0 = deg2rad(0);
th2_0 = deg2rad(0);
th3_0 = deg2rad(0);
th4_0 = deg2rad(0);
th5_0 = deg2rad(0);
th6_0 = deg2rad(0);
 
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
S = zeros(6,n_joints);
for i = 1:n_joints
    v = cross(-omega(:,i),q(:,i));
    S(:,i) = [omega(:,i); v];
end

% Definition M_{b,i}
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
M_bi(:,:,1) = M_b1;
M_bi(:,:,2) = M_b2;
M_bi(:,:,3) = M_b3;
M_bi(:,:,4) = M_b4;
M_bi(:,:,5) = M_b5;
M_bi(:,:,6) = M_b6;
M_bi(:,:,7) = M_be;
Inertia(:,:,1) = Il1;           
Inertia(:,:,2) = Il2;
Inertia(:,:,3) = Il3;
Inertia(:,:,4) = Il4;           
Inertia(:,:,5) = Il5;
Inertia(:,:,6) = Il6;
mass = [ml1 ml2 ml3 ml4 ml5 ml6];
inertial_disp(1,:) = inertial_disp_1;
inertial_disp(2,:) = inertial_disp_2;         
inertial_disp(3,:) = inertial_disp_3;
inertial_disp(4,:) = inertial_disp_4;         
inertial_disp(5,:) = inertial_disp_5;         
inertial_disp(6,:) = inertial_disp_6;   
friction = [fv1 fv2 fv3 fv4 fv5 fv6];

% Store data regarding the frames in a struct
info = struct('n_joints', n_joints, ...
              'S', S, ...
              'M_bi', M_bi, ...
              'Inertia', Inertia, ...
              'mass', mass, ...
              'inertial_disp', inertial_disp);  

% Simulation
T = 20;
Kp = 10*eye(6);
Kd = 10*eye(6);
q_ref = [pi/2 -pi/4 -pi/4 pi/2 -pi/4 pi/8];
qd_ref = zeros(1,6);
qdd_ref = zeros(1,6);
sim('lwa4p')

