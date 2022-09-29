close all
clear
clc

run('../../../parameters/test_parameters.m')
addpath('../../../include/screw_theory_functions/')
addpath('../../../include/control_functions/test')

% Chain definition
n_links = 4;
n_ee = 2;
n_joints = 4;

% Dynamic parameters
g = [0 0 -g0]';
F_ee_A = zeros(6,1);
F_ee_B = zeros(6,1);

% Kinematic parameters
omega1 = [1 0 0]';              % (first passive x joint)
omega2 = [1 0 0]';              % (second passive x joint)
omega3 = [1 0 0]';              % (left arm)
omega4 = [-1 0 0]';             ]]% (right arm)

q1 = [0 0 0]';                  % (first passive x joint)
q2 = [0 0 -L]';                 % (second passive x joint)
q3 = [0 -off -L]';               % (left arm)
q4 = [0 off -L]';              % (right arm)

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
        0  1  0  0;
        0  0  1  0;
        0  0  0  1];
M_b2 = [1  0  0  0;
        0  1  0  0;
        0  0  1  -L;
        0  0  0  1];
M_b3 = [1  0  0  0;
        0  1  0  -off;
        0  0  1  -L;
        0  0  0  1];
M_b4 = [-1   0  0  0;
         0  -1  0  off;
         0   0  1  -L;
         0   0  0  1];
M_b5 = [1  0  0  0;
        0  1  0 -off;
        0  0  1 -L-L1;
        0  0  0  1];
M_b6 = [-1   0  0  0;
         0  -1  0  off;
         0   0  1  -L-L1;
         0   0  0  1];
    
% Store datas in data-structures
M_bi(:,:,1) = M_b1;
M_bi(:,:,2) = M_b2;
M_bi(:,:,3) = M_b3;
M_bi(:,:,4) = M_b4;
M_bi(:,:,5) = M_b5;
M_bi(:,:,6) = M_b6;
Inertia(:,:,1) = I_cables;
Inertia(:,:,2) = I_shoulders;
Inertia(:,:,3) = Il1_left;
Inertia(:,:,4) = Il1_right;
mass = [m_cables m_shoulders ml1 ml1];
inertial_disp(1,:) = inertial_disp_cables;
inertial_disp(2,:) = inertial_disp_shoulders;
inertial_disp(3,:) = inertial_disp_1_left;
inertial_disp(4,:) = inertial_disp_1_right;
friction = [fv1p fv2p fv1 fv1];

% Store data regarding the frames in a struct
info = struct('n_joints', n_joints, ...
              'n_links', n_links, ...
              'n_ee', n_ee, ...
              'current_frame_index',  [1  2 3 4 5 6], ...
              'previous_frame_index', [0  1 2 2 3 4], ...
              'next_frame_index',     [2 34 5 6 0 0], ...
              'next_frame_type',      [0  1 0 0 0 0], ...         % 0 for single frame / 1 for duplex next frame (frameA(1 digit) - frameB(1 digit)) / 2 for duplex next frame (frameA(1 digit) - frameB(2 digit))
              'previous_joint_index', [1  2 3 4 0 0], ...
              'frame_type',           [1  1 1 1 0 0], ...         % 1 for link, 0 for ee
              'explored',             [0  0 0 0 0 0], ...         % 0 for unxeplored, 1 for explored 
              'S', S, ...
              'M_bi', M_bi, ...
              'Inertia', Inertia, ...
              'mass', mass, ...
              'inertial_disp', inertial_disp, ...
              'F', [zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) F_ee_A F_ee_B]);                                 

% Control gains
Kp = diag([40 40]);
Kd = diag([15 15]);

% Regulation references
q_ref = [-pi/8 -pi/6]';
q_dot_ref = zeros(2,1);
q_ddot_ref = zeros(2,1);

% Simulation parameters
Ts = 0.01;                 % Sample Time [s]
Duration = 10;              % Simulation Time [s]

% Start simulation
run('test_controller_partialfbl.m')

