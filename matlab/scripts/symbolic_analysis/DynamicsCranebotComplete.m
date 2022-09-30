close all
clear
clc

addpath('../../functions/screw_theory_symbolic_functions')

% Variables
syms q1 q2 q3 q4 q5 q6 q7 q8 q9 q10 q11 q12 q13 q14 q15 q16 ...
     qd1 qd2 qd3 qd4 qd5 qd6 qd7 qd8 qd9 qd10 qd11 qd12 qd13 qd14 qd15 qd16 ...
     qdd1 qdd2 qdd3 qdd4 qdd5 qdd6 qdd7 qdd8 qdd9 qdd10 qdd11 qdd12 qdd13 qdd14 qdd15 qdd16 real

TH = [q1 q2 q3 q4 q5 q6 q7 q8 q9 q10 q11 q12 q13 q14 q15 q16];
dTH = [qd1 qd2 qd3 qd4 qd5 qd6 qd7 qd8 qd9 qd10 qd11 qd12 qd13 qd14 qd15 qd16];
ddTH = [qdd1 qdd2 qdd3 qdd4 qdd5 qdd6 qdd7 qdd8 qdd9 qdd10 qdd11 qdd12 qdd13 qdd14 qdd15 qdd16];

% Known parameters
run('../../parameters/cranebot_parameters.m')

% Unknown parameters
clear L m_cables fv1p fv2p ixx_cables iyy_cables izz_cables
syms L m_cables1 m_cables2 l_cables_1 l_cables_2 fv1p_1 fv1p_2 fv2p_1 fv2p_2 ixx_cables iyy_cables izz_cables real
I_cables = [ixx_cables 0 0; 0 iyy_cables 0; 0 0 izz_cables];
inertial_disp_cables_1 = [0 0 -l_cables_1];
inertial_disp_cables_2 = [0 0 -l_cables_2];

% Chain definition
n_links = 16;
n_ee = 2;
n_joints = 16;

% Dynamic parameters
g = [0 0 -g0]';
F_ee_A = zeros(6,1);
F_ee_B = zeros(6,1);

% Kinematic parameters
omega1 = [1 0 0]';
omega2 = [0 1 0]';
omega3 = [1 0 0]';
omega4 = [0 1 0]';
omega5 = [0 0 -1]';
omega6 = [1 0 0]';
omega7 = [-1 0 0]';
omega8 = [0 0 -1]';
omega9 = [-1 0 0]';
omega10 = [0 0 -1]';
omega11 = [0 0 -1]';
omega12 = [-1 0 0]';
omega13 = [1 0 0]';
omega14 = [0 0 -1]';
omega15 = [1 0 0]';
omega16 = [0 0 -1]';

q_1 = [0 0 0]';
q_2 = [0 0 0]';
q_3 = [0 0 -L]';
q_4 = [0 0 -L]';
q_5 = [0 offA -L-D]';
q_6 = [0 offA -L-D]';
q_7 = [0 offA -L-D-L1]';
q_8 = [0 offA -L-D-L1]';
q_9 = [0 offA -L-D-L1-L2]';
q_10 = [0 offA -L-D-L1-L2]';
q_11 = [0 offB -L-D]';
q_12 = [0 offB -L-D]';
q_13 = [0 offB -L-D-L1]';
q_14 = [0 offB -L-D-L1]';
q_15 = [0 offB -L-D-L1-L2]';
q_16 = [0 offB -L-D-L1-L2]';

% Screw axis computation
omega = [omega1 omega2 omega3 omega4 omega5 omega6 omega7 omega8 omega9 omega10 omega11 omega12 omega13 omega14 omega15 omega16];
q_ = [q_1 q_2 q_3 q_4 q_5 q_6 q_7 q_8 q_9 q_10 q_11 q_12 q_13 q_14 q_15 q_16];
S = sym(zeros(6,n_joints));
for i = 1:n_joints
    v = cross(-omega(:,i),q_(:,i));
    S(:,i) = [omega(:,i); v];
end

% Computation M_{b,i}
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
        0  0  1 -L;
        0  0  0  1];

M_b4 = [1  0  0  0;
        0  1  0  0;
        0  0  1 -L;
        0  0  0  1];

M_b5 = [0 -1  0  0;
       -1  0  0 offA;
        0  0 -1 -L-D;
        0  0  0  1];
    
M_b6 = [0  0  1  0;
       -1  0  0 offA;
        0 -1  0 -L-D;
        0  0  0  1];
    
M_b7 = [0  0 -1   0;
        0 -1  0  offA;
       -1  0  0 -L-D-L1;
        0  0  0   1];
    
M_b8 = [0 -1  0   0;
       -1  0  0  offA;
        0  0 -1 -L-D-L1;
        0  0  0   1];
    
M_b9 = [0  0 -1    0;
       -1  0  0   offA;
        0  1  0 -L-D-L1-L2;
        0  0  0    1];

M_b10 = [0 -1  0    0;
       -1  0  0   offA;
        0  0 -1 -L-D-L1-L2;
        0  0  0    1];

M_b11 = [0  1  0  0;
        1  0  0 offB;
        0  0 -1 -L-D;
        0  0  0  1];
    
M_b12 = [0  0 -1  0;
         1  0  0 offB;
         0 -1  0 -L-D;
         0  0  0  1];
    
M_b13 = [0  0  1   0;
         0  1  0  offB;
        -1  0  0 -L-D-L1;
         0  0  0   1];
    
M_b14 = [0  1  0   0;
         1  0  0  offB;
         0  0 -1 -L-D-L1;
         0  0  0   1];
    
M_b15 = [0  0  1    0;
         1  0  0   offB;
         0  1  0 -L-D-L1-L2;
         0  0  0    1];

M_b16 = [0  1  0    0;
         1  0  0   offB;
         0  0 -1 -L-D-L1-L2;
         0  0  0    1];
    
M_b17 = [0 -1  0    0;
        -1  0  0   offA;
         0  0 -1 -L-D-L1-L2-L3;
         0  0  0    1];
     
M_b18 = [0  1  0    0;
         1  0  0   offB;
         0  0 -1 -L-D-L1-L2-L3;
         0  0  0    1];

% Store datas in data-structures
M_bi = sym(zeros(4,4,16));
Inertia = sym(zeros(3,3,14));
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
M_bi(:,:,15) = M_b15;
M_bi(:,:,16) = M_b16;
M_bi(:,:,17) = M_b17;
M_bi(:,:,18) = M_b18;
Inertia(:,:,1) = I_cables;
Inertia(:,:,2) = I_cables;
Inertia(:,:,3) = I_platform;
Inertia(:,:,4) = I_platform;
Inertia(:,:,5) = Il1;
Inertia(:,:,6) = Il2;
Inertia(:,:,7) = Il3;
Inertia(:,:,8) = Il4;
Inertia(:,:,9) = Il5;
Inertia(:,:,10) = Il6;
Inertia(:,:,11) = Il1;
Inertia(:,:,12) = Il2;
Inertia(:,:,13) = Il3;
Inertia(:,:,14) = Il4;
Inertia(:,:,15) = Il5;
Inertia(:,:,16) = Il6;
mass = [m_cables1 m_cables2 m_platform m_platform ml1 ml2 ml3 ml4 ml5 ml6 ml1 ml2 ml3 ml4 ml5 ml6];
inertial_disp(1,:) = inertial_disp_cables_1;
inertial_disp(2,:) = inertial_disp_cables_2;
inertial_disp(3,:) = inertial_disp_platform;
inertial_disp(4,:) = inertial_disp_platform;
inertial_disp(5,:) = inertial_disp_1;
inertial_disp(6,:) = inertial_disp_2;
inertial_disp(7,:) = inertial_disp_3;
inertial_disp(8,:) = inertial_disp_4;
inertial_disp(9,:) = inertial_disp_5;
inertial_disp(10,:) = inertial_disp_6;
inertial_disp(11,:) = inertial_disp_1;
inertial_disp(12,:) = inertial_disp_2;
inertial_disp(13,:) = inertial_disp_3;
inertial_disp(14,:) = inertial_disp_4;
inertial_disp(15,:) = inertial_disp_5;
inertial_disp(16,:) = inertial_disp_6;
friction = [fv1p_1 fv1p_2 fv2p_1 fv2p_2 fv1 fv2 fv3 fv4 fv5 fv6 fv1 fv2 fv3 fv4 fv5 fv6];

% Store data regarding the frames in a struct
info = struct('n_joints', n_joints, ...
              'n_links', n_links, ...
              'n_ee', n_ee, ...
              'current_frame_index',  [ 1 2 3   4 5 6 7  8  9 10 11 12 13 14 15 16 17 18], ...
              'previous_frame_index', [ 0 1 2   3 4 5 6  7  8  9  4 11 12 13 14 15 10 16], ...
              'next_frame_index',     [ 2 3 4 511 6 7 8  9 10 17 12 13 14 15 16 18  0  0], ...
              'next_frame_type',      [ 0 0 0   2 0 0 0  0  0  0  0  0  0  0  0  0  0  0], ...         % 0 for single frame / 1 for duplex next frame (frameA(1 digit) - frameB(1 digit)) / 2 for duplex next frame (frameA(1 digit) - frameB(2 digit))
              'previous_joint_index', [ 1 2 3   4 5 6 7  8  9 10 11 12 13 14 15 16  0  0], ...
              'frame_type',           [ 1 1 1   1 1 1 1  1  1  1  1  1  1  1  1  1  0  0], ...         % 1 for link, 0 for ee
              'explored',             [ 0 0 0   0 0 0 0  0  0  0  0  0  0  0  0  0  0  0], ...         % 0 for unxeplored, 1 for explored 
              'S', S, ...
              'M_bi', M_bi, ...
              'Inertia', Inertia, ...
              'mass', mass, ...
              'inertial_disp', inertial_disp, ...
              'F', [zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) F_ee_A F_ee_B]);                                 

% Computation B matrix
disp('Computation B matrix\n')
B = sym(zeros(n_joints,n_joints));
for j=1:n_joints
    fake_acc = zeros(1,n_joints);
    fake_acc(j) = 1;
    msg = append('B matrix computation - Column ',int2str(j),'/',int2str(n_joints));
    B(:,j) = recursive_invdyn_tree_sym_f(TH, zeros(1,n_joints), fake_acc, zeros(3,1), info, msg);
end

% Computation n vector
disp('Computation n vector\n')
Fv = diag([friction(1), friction(2), friction(3), friction(4), friction(5), friction(6), friction(7), friction(8), friction(9), friction(10) friction(11), friction(12), friction(13), friction(14), friction(15), friction(16)]);
msg = append('n vector computation');
n = recursive_invdyn_tree_sym_f(TH, dTH, zeros(1,n_joints), g, info, msg) + Fv*dTH';
