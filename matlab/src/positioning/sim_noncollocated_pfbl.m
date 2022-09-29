close all
clear
clc

run('../../parameters/licas_parameters.m')
addpath('../../include/screw_theory_functions/')
addpath('../../include/control_functions/')
addpath('../../include/trajectory_generation_functions')
addpath('../../include/kinematics_functions')

% Chain definition
n_links = 12;
n_ee = 2;
n_joints = 12;

% Dynamic parameters
g = [0 0 -g0]';
F_ee_A = zeros(6,1);
F_ee_B = zeros(6,1);

% Kinematic parameters
omega1 = [1 0 0]';              % (first passive x joint)
omega2 = [0 1 0]';              % (first passive y joint)
omega3 = [1 0 0]';              % (second passive x joint)
omega4 = [0 1 0]';              % (second passive y joint)
omega5 = [0 1 0]';              % (left arm)
omega6 = [1 0 0]';              % (left arm)
omega7 = [0 0 1]';              % (left arm)
omega8 = [0 1 0]';              % (left arm)
omega9 = [0 1 0]';              % (right arm)
omega10 = [1 0 0]';             % (right arm)
omega11 = [0 0 1]';             % (right arm)
omega12 = [0 1 0]';             % (right arm)

q1 = [0 0 0]';                  % (first passive x joint)
q2 = [0 0 0]';                  % (first passive y joint)
q3 = [0 0 -L]';                 % (second passive x joint)
q4 = [0 0 -L]';                 % (second passive y joint)
q5 = [0 off -L]';               % (left arm)
q6 = [0 off+L1 -L]';            % (left arm)
q7 = [0 off+L1 -L-L2]';         % (left arm)
q8 = [0 off+L1 -L-L2-L3]';      % (left arm)
q9 = [0 -off -L]';              % (right arm)
q10 = [0 -off-L1 -L]';          % (right arm)
q11 = [0 -off-L1 -L-L2]';       % (right arm)
q12 = [0 -off-L1 -L-L2-L3]';    % (right arm)

% Screw axis computation
omega = [omega1 omega2 omega3 omega4 omega5 omega6 omega7 omega8 omega9 omega10 omega11 omega12];
q = [q1 q2 q3 q4 q5 q6 q7 q8 q9 q10 q11 q12];
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
        0  0  1  0;
        0  0  0  1];
M_b3 = [1  0  0  0;
        0  1  0  0;
        0  0  1  -L;
        0  0  0  1];
M_b4 = [1  0  0  0;
        0  1  0  0;
        0  0  1  -L;
        0  0  0  1];
M_b5 = [1  0  0  0;
        0  1  0  off;
        0  0  1  -L;
        0  0  0  1];
M_b6 = [1  0  0  0;
        0  1  0  off+L1;
        0  0  1  -L;
        0  0  0  1];
M_b7 = [1  0  0  0;
        0  1  0  off+L1;
        0  0  1  -L-L2;
        0  0  0  1];
M_b8 = [1  0  0  0;
        0  1  0  off+L1;
        0  0  1  -L-L2-L3;
        0  0  0  1];
M_b9 = [1  0  0  0;
        0  1  0  -off;
        0  0  1  -L;
        0  0  0  1];
M_b10 = [1  0  0  0;
        0  1  0  -off-L1;
        0  0  1  -L;
        0  0  0  1];
M_b11 = [1  0  0  0;
        0  1  0  -off-L1;
        0  0  1  -L-L2;
        0  0  0  1];
M_b12 = [1  0  0  0;
        0  1  0  -off-L1;
        0  0  1  -L-L2-L3;
        0  0  0  1];
M_b13 = [1  0  0  0;
        0  1  0  off+L1;
        0  0  1  -L-L2-L3-L4;
        0  0  0  1];
M_b14 = [1  0  0  0;
         0  1  0  -off-L1;
         0  0  1  -L-L2-L3-L4;
         0  0  0  1];
    
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
M_bi(:,:,11) = M_b11;
M_bi(:,:,12) = M_b12;
M_bi(:,:,13) = M_b13;
M_bi(:,:,14) = M_b14;
Inertia(:,:,1) = I_cables;
Inertia(:,:,2) = I_cables;
Inertia(:,:,3) = I_shoulders;
Inertia(:,:,4) = I_shoulders;
Inertia(:,:,5) = Il1_left;
Inertia(:,:,6) = Il2_left;
Inertia(:,:,7) = Il3_left;
Inertia(:,:,8) = Il4_left;
Inertia(:,:,9) = Il1_right;
Inertia(:,:,10) = Il2_right;
Inertia(:,:,11) = Il3_right;
Inertia(:,:,12) = Il4_right;
mass = [m_cables m_cables m_shoulders m_shoulders ml1 ml2 ml3 ml4 ml1 ml2 ml3 ml4];
inertial_disp(1,:) = inertial_disp_cables;
inertial_disp(2,:) = inertial_disp_cables;
inertial_disp(3,:) = inertial_disp_shoulders;
inertial_disp(4,:) = inertial_disp_shoulders;
inertial_disp(5,:) = inertial_disp_1_left;
inertial_disp(6,:) = inertial_disp_2_left;
inertial_disp(7,:) = inertial_disp_3_left;
inertial_disp(8,:) = inertial_disp_4_left;
inertial_disp(9,:) = inertial_disp_1_right;
inertial_disp(10,:) = inertial_disp_2_right;
inertial_disp(11,:) = inertial_disp_3_right;
inertial_disp(12,:) = inertial_disp_4_right;
friction = [fv1p fv2p fv1p fv2p fv1 fv2 fv3 fv4 fv1 fv2 fv3 fv4];

% Store data regarding the frames in a struct
info = struct('n_joints', n_joints, ...
              'n_links', n_links, ...
              'n_ee', n_ee, ...
              'current_frame_index',  [ 1 2 3  4 5 6 7  8  9 10 11 12 13 14], ...
              'previous_frame_index', [ 0 1 2  3 4 5 6  7  4  9 10 11  8 12], ...
              'next_frame_index',     [ 2 3 4 59 6 7 8 13 10 11 12 14  0  0], ...
              'next_frame_type',      [ 0 0 0  1 0 0 0  0  0  0  0  0  0  0], ...         % 0 for single frame / 1 for duplex next frame (frameA(1 digit) - frameB(1 digit)) / 2 for duplex next frame (frameA(1 digit) - frameB(2 digit))
              'previous_joint_index', [ 1 2 3  4 5 6 7  8  9 10 11 12  0  0], ...
              'frame_type',           [ 1 1 1  1 1 1 1  1  1  1  1  1  0  0], ...         % 1 for link, 0 for ee
              'explored',             [ 0 0 0  0 0 0 0  0  0  0  0  0  0  0], ...         % 0 for unxeplored, 1 for explored 
              'S', S, ...
              'M_bi', M_bi, ...
              'Inertia', Inertia, ...
              'mass', mass, ...
              'inertial_disp', inertial_disp, ...
              'F', [zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) F_ee_A F_ee_B]);                                 

% Control gains
Kp = diag([1300 1300 1300 1300]);
Kd = diag([50 50 50 50]);
Kp_nullspace = 10*eye(8);
Kd_nullspace = 10*eye(8);

% Regulation references
q_ref = [0 0 0 0]';
q_dot_ref = [0 0 0 0]';
q_ddot_ref = [0 0 0 0]';

% Drone trajectory parameters
samplingTime = 0.01;
T_traj = 2;                               % [sec]
T_regime = 10;                            % [sec]
T_total = T_traj + T_regime;              % [sec]
desired_drone_position = [5 3];           % [m]

% Start simulation
% run('../drone_motion/drone_control.m')
run('controller_noncollocated_pfbl.m')

