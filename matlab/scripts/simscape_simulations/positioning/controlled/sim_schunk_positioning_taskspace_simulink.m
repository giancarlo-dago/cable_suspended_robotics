close all
clear
clc

if ispc % Windows
    addpath('..\..\meshes\')
    addpath('..\..\crane_motion\')
    addpath('..\..\..\..\functions\screw_theory_functions\')
    addpath('..\..\..\..\functions\trajectory_generation_functions\')
    addpath('..\..\..\..\functions\skeleton_functions\')
    addpath('..\..\..\..\functions\positioning_functions\')
    addpath('..\..\..\..\parameters\')
else % Linux
    addpath('../../meshes/')
    addpath('../../crane_motion/')
    addpath('../../../../functions/screw_theory_functions/')
    addpath('../../../../functions/trajectory_generation_functions/')
    addpath('../../../../functions/skeleton_functions/')
    addpath('../../../../functions/positioning_functions/')
    addpath('../../../../parameters/')
end

run('cranebot_parameters.m')

% Chain definition
n_links = 15;
n_ee = 2;
n_joints = 15;

% Dynamic parameters
g = [0 0 -g0]';
F_ee_A = zeros(6,1);
F_ee_B = zeros(6,1);

% Initial conditions
d0_0 = 0;
alfa_0 = deg2rad(0);
beta_0 = deg2rad(0);
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
q_0 = [d0_0 alfa_0 beta_0 th1A_0 th2A_0 th3A_0 th4A_0 th5A_0 th6A_0 th1B_0 th2B_0 th3B_0 th4B_0 th5B_0 th6B_0];

% Kinematic parameters
omega0 = [0 0 0]';
omega1 = [1 0 0]';
omega2 = [1 0 0]';
omega3 = [0 0 -1]';
omega4 = [1 0 0]';
omega5 = [-1 0 0]';
omega6 = [0 0 -1]';
omega7 = [-1 0 0]';
omega8 = [0 0 -1]';
omega9 = [0 0 -1]';
omega10 = [-1 0 0]';
omega11 = [1 0 0]';
omega12 = [0 0 -1]';
omega13 = [1 0 0]';
omega14 = [0 0 -1]';

v0 = [1 0 0]';
q1 = [0 0 0]';
q2 = [0 0 -L]';
q3 = [0 offA -L-D]';
q4 = [0 offA -L-D]';
q5 = [0 offA -L-D-L1]';
q6 = [0 offA -L-D-L1]';
q7 = [0 offA -L-D-L1-L2]';
q8 = [0 offA -L-D-L1-L2]';
q9 = [0 offB -L-D]';
q10 = [0 offB -L-D]';
q11 = [0 offB -L-D-L1]';
q12 = [0 offB -L-D-L1]';
q13 = [0 offB -L-D-L1-L2]';
q14 = [0 offB -L-D-L1-L2]';

% Screw axis computation
omega = [omega1 omega2 omega3 omega4 omega5 omega6 omega7 omega8 omega9 omega10 omega11 omega12 omega13 omega14];
q = [q1 q2 q3 q4 q5 q6 q7 q8 q9 q10 q11 q12 q13 q14];
S = zeros(6,n_joints);
S(:,1) = [omega0; v0];
for i = 2:n_joints
    v = cross(-omega(:,i-1),q(:,i-1));
    S(:,i) = [omega(:,i-1); v];
end

% Computation M_{b,i}
M_b0 = [1  0  0  0;
        0  1  0  0;
        0  0  1  0;
        0  0  0  1];

M_b1 = [1  0  0  0;
        0  1  0  0;
        0  0  1  0;
        0  0  0  1];

M_b2 = [1  0  0  0;
        0  1  0  0;
        0  0  1 -L;
        0  0  0  1];

M_b3 = [0 -1  0  0;
       -1  0  0 offA;
        0  0 -1 -L-D;
        0  0  0  1];
    
M_b4 = [0  0  1  0;
       -1  0  0 offA;
        0 -1  0 -L-D;
        0  0  0  1];
    
M_b5 = [0  0 -1   0;
        0 -1  0  offA;
       -1  0  0 -L-D-L1;
        0  0  0   1];
    
M_b6 = [0 -1  0   0;
       -1  0  0  offA;
        0  0 -1 -L-D-L1;
        0  0  0   1];
    
M_b7 = [0  0 -1    0;
       -1  0  0   offA;
        0  1  0 -L-D-L1-L2;
        0  0  0    1];    

M_b8 = [0 -1  0    0;
       -1  0  0   offA;
        0  0 -1 -L-D-L1-L2;
        0  0  0    1]; 

M_b9 = [0  1  0  0;
        1  0  0 offB;
        0  0 -1 -L-D;
        0  0  0  1];
    
M_b10 = [0  0 -1  0;
         1  0  0 offB;
         0 -1  0 -L-D;
         0  0  0  1];
    
M_b11 = [0  0  1   0;
         0  1  0  offB;
        -1  0  0 -L-D-L1;
         0  0  0   1];
    
M_b12 = [0  1  0   0;
         1  0  0  offB;
         0  0 -1 -L-D-L1;
         0  0  0   1];
    
M_b13 = [0  0  1    0;
         1  0  0   offB;
         0  1  0 -L-D-L1-L2;
         0  0  0    1];    

M_b14 = [0  1  0    0;
         1  0  0   offB;
         0  0 -1 -L-D-L1-L2;
         0  0  0    1]; 
    
M_b15 = [0 -1  0    0;
        -1  0  0   offA;
         0  0 -1 -L-D-L1-L2-L3;
         0  0  0    1]; 
     
M_b16 = [0  1  0    0;
         1  0  0   offB;
         0  0 -1 -L-D-L1-L2-L3;
         0  0  0    1]; 

% Store datas in data-structures
M_bi(:,:,1) = M_b0;
M_bi(:,:,2) = M_b1;
M_bi(:,:,3) = M_b2;
M_bi(:,:,4) = M_b3;
M_bi(:,:,5) = M_b4;
M_bi(:,:,6) = M_b5;
M_bi(:,:,7) = M_b6;
M_bi(:,:,8) = M_b7;
M_bi(:,:,9) = M_b8;
M_bi(:,:,10) = M_b9;
M_bi(:,:,11) = M_b10;
M_bi(:,:,12) = M_b11;
M_bi(:,:,13) = M_b12;
M_bi(:,:,14) = M_b13;
M_bi(:,:,15) = M_b14;
M_bi(:,:,16) = M_b15;
M_bi(:,:,17) = M_b16;
Inertia(:,:,1) = zeros(3,3);
Inertia(:,:,2) = I_cables;
Inertia(:,:,3) = I_platform;
Inertia(:,:,4) = Il1;
Inertia(:,:,5) = Il2;
Inertia(:,:,6) = Il3;
Inertia(:,:,7) = Il4;
Inertia(:,:,8) = Il5;
Inertia(:,:,9) = Il6;
Inertia(:,:,10) = Il1;
Inertia(:,:,11) = Il2;
Inertia(:,:,12) = Il3;
Inertia(:,:,13) = Il4;
Inertia(:,:,14) = Il5;
Inertia(:,:,15) = Il6;
mass = [m_crane m_cables m_platform ml1 ml2 ml3 ml4 ml5 ml6 ml1 ml2 ml3 ml4 ml5 ml6];
inertial_disp(1,:) = [0 0 0]';
inertial_disp(2,:) = inertial_disp_cables;
inertial_disp(3,:) = inertial_disp_platform;
inertial_disp(4,:) = inertial_disp_1;
inertial_disp(5,:) = inertial_disp_2;
inertial_disp(6,:) = inertial_disp_3;
inertial_disp(7,:) = inertial_disp_4;
inertial_disp(8,:) = inertial_disp_5;
inertial_disp(9,:) = inertial_disp_6;
inertial_disp(10,:) = inertial_disp_1;
inertial_disp(11,:) = inertial_disp_2;
inertial_disp(12,:) = inertial_disp_3;
inertial_disp(13,:) = inertial_disp_4;
inertial_disp(14,:) = inertial_disp_5;
inertial_disp(15,:) = inertial_disp_6;

fv0 = 0;
fs0 = 0;
fs1p = 0;
fs2p = 0;
fs1 = 0;
fs2 = 0;
fs3 = 0;
fs4 = 0;
fs5 = 0;
fs6 = 0;
fs1 = 0;
fs2 = 0;
fs3 = 0;
fs4 = 0;
fs5 = 0;
fs6 = 0;

viscous_friction = [fv0 fv1p fv2p fv1 fv2 fv3 fv4 fv5 fv6 fv1 fv2 fv3 fv4 fv5 fv6];
static_friction = [fs0 fs1p fs2p fs1 fs2 fs3 fs4 fs5 fs6 fs1 fs2 fs3 fs4 fs5 fs6];

% Store data regarding the frames in a struct
info = struct('n_joints', n_joints, ...
              'n_links', n_links, ...
              'n_ee', n_ee, ...
              'current_frame_index',  [ 1 2   3 4 5 6 7 8  9  10 11 12 13 14 15 16 17], ...
              'previous_frame_index', [ 0 1   2 3 4 5 6 7  8   3 10 11 12 13 14  9 15],...
              'next_frame_index',     [ 2 3 410 5 6 7 8 9 16  11 12 13 14 15 17  0  0],...
              'next_frame_type',      [ 0 0   2 0 0 0 0 0  0   0  0  0  0  0  0  0  0], ...         % 0 for single frame / 1 for duplex next frame (frameA(1 digit) - frameB(1 digit)) / 2 for duplex next frame (frameA(1 digit) - frameB(2 digit))
              'previous_joint_index', [ 1 2   3 4 5 6 7 8  9  10 11 12 13 14 15  0  0], ...
              'frame_type',           [ 1 1   1 1 1 1 1 1  1   1  1  1  1  1  1  0  0], ...         % 1 for link, 0 for ee
              'explored',             [ 0 0   0 0 0 0 0 0  0   0  0  0  0  0  0  0  0], ...         % 0 for unxeplored, 1 for explored  
              'S', S, ...
              'M_bi', M_bi, ...
              'Inertia', Inertia, ...
              'mass', mass, ...
              'inertial_disp', inertial_disp, ...
              'F', [zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) F_ee_A F_ee_B]);                                 

          % Store data regarding the frames in a struct
info_bis = struct('n_joints', n_joints-1, ...
              'n_links', n_links-1, ...
              'n_ee', n_ee, ...
              'current_frame_index',  [ 1  2 3 4 5 6 7  8  9 10 11 12 13 14 15 16], ...
              'previous_frame_index', [ 0  1 2 3 4 5 6  7  2  9 10 11 12 13  8 14],...
              'next_frame_index',     [ 2 39 4 5 6 7 8 15 10 11 12 13 14 16  0  0],...
              'next_frame_type',      [ 0  1 0 0 0 0 0  0  0  0  0  0  0  0  0  0], ...         % 0 for single frame / 1 for duplex next frame (frameA(1 digit) - frameB(1 digit)) / 2 for duplex next frame (frameA(1 digit) - frameB(2 digit))
              'previous_joint_index', [ 1  2 3 4 5 6 7  8  9 10 11 12 13 14  0  0], ...
              'frame_type',           [ 1  1 1 1 1 1 1  1  1  1  1  1  1  1  0  0], ...         % 1 for link, 0 for ee
              'explored',             [ 0  0 0 0 0 0 0  0  0  0  0  0  0  0  0  0], ...         % 0 for unxeplored, 1 for explored 
              'S', S(:,2:end), ...
              'M_bi', M_bi(:,:,2:end), ...
              'Inertia', Inertia(:,:,2:end), ...
              'mass', mass(2:end), ...
              'inertial_disp', inertial_disp(2:end,:), ...
              'F', [zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) F_ee_A F_ee_B]);                                 

% Simulation
T = 200;
d0_i = 0;             % Overhead crane initial position
d0_f = 16.98;              % Overhead crane final position
tf = 40;                % Duration of the overhead crane trajectory

% Gains
Kd = diag([20 5]);
Kp = diag([100 10]);
Kd_nullspace = 250*eye(12);
Kp_nullspace = 250*eye(12);

% Gains
% Kd = diag([20 10]);
% Kp = diag([20 50]);
% Kd = diag([5 10]);
% Kp = diag([50 50]);
% Kd_nullspace = 200*eye(12);
% Kp_nullspace = 200*eye(12);


% run('cms_trolley_trajectory.m');
% run('crane927_trolley_trajectory.m');
run('d0_trajectory');
% sim('schunk_positioning_taskspace')

model = 'schunk_positioning_taskspace_simulink';
% set_param(model,'SimMechanicsOpenEditorOnUpdate','off')
sim(model, 'FastRestart', 'off');
