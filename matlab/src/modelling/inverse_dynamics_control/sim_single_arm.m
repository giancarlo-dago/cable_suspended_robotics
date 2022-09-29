close all
clear
clc

run('../../../parameters/licas_parameters.m')
addpath('../../../include/screw_theory_functions/')
addpath('../../../include/control_functions/')

% Chain definition
n_joints = 4;

% Dynamic parameters
g = [0 0 -g0]';
F_ee = zeros(6,1);

% Kinematic parameters
omega1 = [0 1 0]';
omega2 = [1 0 0]';
omega3 = [0 0 1]';
omega4 = [0 1 0]';
q1 = [0 off 0]';
q2 = [0 off+L1 0]';
q3 = [0 off+L1 -L2]';
q4 = [0 off+L1 -L2-L3]';

% Screw axis computation
omega = [omega1 omega2 omega3 omega4];
q = [q1 q2 q3 q4];
S = zeros(6,n_joints);
for i = 1:n_joints
    v = cross(-omega(:,i),q(:,i));
    S(:,i) = [omega(:,i); v];
end

% Definition M_{b,i}
M_b1 = [1  0  0  0;
        0  1  0  off;
        0  0  1  0;
        0  0  0  1];
M_b2 = [1  0  0  0;
        0  1  0  off+L1;
        0  0  1  0;
        0  0  0  1];
M_b3 = [1  0  0  0;
        0  1  0  off+L1;
        0  0  1  -L2;
        0  0  0  1];
M_b4 = [1  0  0  0;
        0  1  0  off+L1;
        0  0  1  -L2-L3;
        0  0  0  1];
M_be = [1  0  0  0;
        0  1  0  off+L1;
        0  0  1  -L2-L3-L4;
        0  0  0  1];
    
% Store datas in data-structures
M_bi(:,:,1) = M_b1;
M_bi(:,:,2) = M_b2;
M_bi(:,:,3) = M_b3;
M_bi(:,:,4) = M_b4;
M_bi(:,:,5) = M_be;
Inertia(:,:,1) = Il1_left;
Inertia(:,:,2) = Il2_left; 
Inertia(:,:,3) = Il3_left; 
Inertia(:,:,4) = Il4_left; 
mass = [ml1 ml2 ml3 ml4];
inertial_disp(1,:) = inertial_disp_1_left;
inertial_disp(2,:) = inertial_disp_2_left;
inertial_disp(3,:) = inertial_disp_3_left;
inertial_disp(4,:) = inertial_disp_4_left;
friction = [fv1 fv2 fv3 fv4];

% Store data regarding the frames in a struct
info = struct('n_joints', n_joints, ...
              'S', S, ...
              'M_bi', M_bi, ...
              'Inertia', Inertia, ...
              'mass', mass, ...
              'inertial_disp', inertial_disp);
          
% Control gains
Kp = diag([50 50 50 50]);
Kd = diag([15 15 15 15]);

% Regulation references     
q_ref = [pi/8 pi/8 pi/8 pi/16]';
% q_ref = [0 0 0 0]';
q_dot_ref = zeros(4,1);
q_ddot_ref = zeros(4,1);

% Simulation parameters
Ts = 0.01;                 % Sample Time [s]
Duration = 6;              % Simulation Time [s]

% Start simulation
run('controller_single_arm.m')
