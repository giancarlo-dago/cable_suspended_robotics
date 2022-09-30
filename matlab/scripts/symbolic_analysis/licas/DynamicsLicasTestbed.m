close all
clear
clc

addpath('../../../functions/screw_theory_symbolic_functions')

% Variables
syms q1 q2 q3 q4 q5 q6 q7 q8 q9 q10 q11 q12 q13 q14...
     qd1 qd2 qd3 qd4 qd5 qd6 qd7 qd8 qd9 qd10 qd11 qd12 qd13 qd14 ...
     qdd1 qdd2 qdd3 qdd4 qdd5 qdd6 qdd7 qdd8 qdd9 qdd10 qdd11 qdd12 qdd13 qdd14 real

TH = [q1 q2 q3 q4 q5 q6 q7 q8 q9 q10 q11 q12 q13 q14];
dTH = [qd1 qd2 qd3 qd4 qd5 qd6 qd7 qd8 qd9 qd10 qd11 qd12 qd13 qd14];
ddTH = [qdd1 qdd2 qdd3 qdd4 qdd5 qdd6 qdd7 qdd8 qdd9 qdd10 qdd11 qdd12 qdd13 qdd14];

% Unknown parameters
syms m1 m2 m3 m4 m5 m6 ...
     L ...
     l1 l2 l3 ...
     ixx1 iyy1 izz1 ...
     ixx2 iyy2 izz2 ...
     ixx3 iyy3 izz3 ...
     fv1 fv2 fv3 real
I1 = [ixx1, 0, 0; 0, iyy1, 0; 0, 0, izz1];
I2 = [ixx2, 0, 0; 0, iyy2, 0; 0, 0, izz2];
I3 = [ixx3, 0, 0; 0, iyy3, 0; 0, 0, izz3];
inertial_disp_1 = [0 0 -l1]';
inertial_disp_2 = [0 0 -l2]';
inertial_disp_3 = [0 0 -l3]';

m4 = 0.3195;
m5 = 0.3195;
m6 = 0.3195;
L = 1.0;
% l1 = 0.5639;
% l2 = 0.5639;
% l3 = 0.5639;
% ixx = 0.1;
% iyy = 0.01;
% izz = 0.01;
% fv1 = 0.04;
% fv2 = 0.23;
% fv3 = 0.3;

% Known parameters
L1 = 0.04;
L2 = 0.143;
L3 = 0.132;
L4 = 0.277;
off = 0.14;
m7 = 0.233;
m8 = 0.246;
m9 = 0.214;
m10 = 0.106;
m11 = 0.233;
m12 = 0.246;
m13 = 0.214;
m14 = 0.106;
I4 = [ 3.02e-3,        0,        0;        0, 8.64e-4,       0;        0,       0, 3.39e-3];
I5 = [ 3.02e-3,        0,        0;        0, 8.64e-4,       0;        0,       0, 3.39e-3];
I6 = [ 3.02e-3,        0,        0;        0, 8.64e-4,       0;        0,       0, 3.39e-3];
I7 = [ 3.68e-4, -7.57e-6,  5.38e-6; -7.57e-6, 1.16e-4, -4.8e-5;  5.38e-6, -4.8e-5, 3.76e-4];
I8 = [ 3.49e-4,        0, -3.79e-5;        0, 4.46e-4,       0; -3.79e-5,       0, 1.49e-4];
I9 = [ 4.21e-4, -7.81e-7, -4.06e-5; -7.81e-7, 4.26e-4,  1.8e-5; -4.06e-5,  1.8e-5, 5.01e-5];
I10 = [ 3.94e-4,        0,        0;        0, 3.75e-4, -3.5e-6;        0, -3.5e-6, 3.05e-5];
I11 = [3.68e-4, -7.57e-6,  5.38e-6; -7.57e-6,  1.16e-4, -4.8e-5;  5.38e-6,  -4.8e-5, 3.76e-4];
I12 = [3.49e-4,        0, -3.79e-5;        0,  4.46e-4,       0; -3.79e-5,        0, 1.49e-4];
I13 = [4.21e-4, -7.81e-7, -4.06e-5; -7.81e-7,  4.26e-4,  1.8e-5; -4.06e-5,   1.8e-5, 5.01e-5];
I14 = [3.94e-4,        0,        0;        0,  3.75e-4, -3.5e-6;        0,  -3.5e-6, 3.05e-5];
inertial_disp_4 = [      0       0        0]';
inertial_disp_5 = [      0       0        0]';
inertial_disp_6 = [      0       0        0]';
inertial_disp_7 = [      0  0.0236 -0.00946]';
inertial_disp_8 = [ -0.015       0     -0.1]';
inertial_disp_9 = [      0       0   -0.093]';
inertial_disp_10 = [      0       0   -0.092]';
inertial_disp_11 = [     0 -0.0236 -0.00946]';
inertial_disp_12 = [-0.015       0     -0.1]';
inertial_disp_13 = [     0       0   -0.093]';
inertial_disp_14 = [     0       0   -0.092]';
fv4 = 0;
fv5 = 0;
fv6 = 0;
fv7 = 0;
fv8 = 0;
fv9 = 0;
fv10 = 0;
fv11 = 0;
fv12 = 0;
fv13 = 0;
fv14 = 0;
g0 = 9.8;

% Chain definition
n_links = 14;
n_ee = 2;
n_joints = 14;

% Dynamic parameters
g = [0 0 -g0]';
F_ee_A = zeros(6,1);
F_ee_B = zeros(6,1);

% Kinematic parameters
omega1 = [0 0 1]';              % (first passive z joint)
omega2 = [1 0 0]';              % (first passive x joint)
omega3 = [0 1 0]';              % (first passive y joint)
omega4 = [0 0 1]';              % (second passive z joint)
omega5 = [1 0 0]';              % (second passive x joint)
omega6 = [0 1 0]';              % (second passive y joint)
omega7 = [0 1 0]';              % (left arm)
omega8 = [1 0 0]';              % (left arm)
omega9 = [0 0 1]';              % (left arm)
omega10 = [0 1 0]';             % (left arm)
omega11 = [0 1 0]';             % (right arm)
omega12 = [1 0 0]';             % (right arm)
omega13 = [0 0 1]';             % (right arm)
omega14 = [0 1 0]';             % (right arm)

q_1 = [0 0 0]';                  % (first passive z joint)
q_2 = [0 0 0]';                  % (first passive x joint)
q_3 = [0 0 0]';                  % (first passive y joint)
q_4 = [0 0 -L]';                 % (second passive x joint)
q_5 = [0 0 -L]';                 % (second passive x joint)
q_6 = [0 0 -L]';                 % (second passive y joint)
q_7 = [0 off -L]';               % (left arm)
q_8 = [0 off+L1 -L]';            % (left arm)
q_9 = [0 off+L1 -L-L2]';         % (left arm)
q_10 = [0 off+L1 -L-L2-L3]';      % (left arm)
q_11 = [0 -off -L]';             % (right arm)
q_12 = [0 -off-L1 -L]';          % (right arm)
q_13 = [0 -off-L1 -L-L2]';       % (right arm)
q_14 = [0 -off-L1 -L-L2-L3]';    % (right arm)

% Screw axis computation
omega = [omega1 omega2 omega3 omega4 omega5 omega6 omega7 omega8 omega9 omega10 omega11 omega12 omega13 omega14];
q_ = [q_1 q_2 q_3 q_4 q_5 q_6 q_7 q_8 q_9 q_10 q_11 q_12 q_13 q_14];
S = sym(zeros(6,n_joints));
for i = 1:n_joints
    v = cross(-omega(:,i),q_(:,i));
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
        0  0  1  0;
        0  0  0  1];
M_b4 = [1  0  0  0;
        0  1  0  0;
        0  0  1  -L;
        0  0  0  1];
M_b5 = [1  0  0  0;
        0  1  0  0;
        0  0  1  -L;
        0  0  0  1];
M_b6 = [1  0  0  0;
        0  1  0  0;
        0  0  1  -L;
        0  0  0  1];
M_b7 = [1  0  0  0;
        0  1  0  off;
        0  0  1  -L;
        0  0  0  1];
M_b8 = [1  0  0  0;
        0  1  0  off+L1;
        0  0  1  -L;
        0  0  0  1];
M_b9 = [1  0  0  0;
        0  1  0  off+L1;
        0  0  1  -L-L2;
        0  0  0  1];
M_b10 = [1  0  0  0;
        0  1  0  off+L1;
        0  0  1  -L-L2-L3;
        0  0  0  1];
M_b11 = [1  0  0  0;
         0  1  0  -off;
         0  0  1  -L;
         0  0  0  1];
M_b12 = [1  0  0  0;
         0  1  0  -off-L1;
         0  0  1  -L;
         0  0  0  1];
M_b13 = [1  0  0  0;
         0  1  0  -off-L1;
         0  0  1  -L-L2;
         0  0  0  1];
M_b14 = [1  0  0  0;
         0  1  0  -off-L1;
         0  0  1  -L-L2-L3;
         0  0  0  1];
M_b15 = [1  0  0  0;
         0  1  0  off+L1;
         0  0  1  -L-L2-L3-L4;
         0  0  0  1];
M_b16 = [1  0  0  0;
         0  1  0  -off-L1;
         0  0  1  -L-L2-L3-L4;
         0  0  0  1];
    
% Store datas in data-structures
M_bi = sym(zeros(4,4,16));
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
Inertia(:,:,1) = I1;
Inertia(:,:,2) = I2;
Inertia(:,:,3) = I3;
Inertia(:,:,4) = I4;
Inertia(:,:,5) = I5;
Inertia(:,:,6) = I6;
Inertia(:,:,7) = I7;
Inertia(:,:,8) = I8;
Inertia(:,:,9) = I9;
Inertia(:,:,10) = I10;
Inertia(:,:,11) = I11;
Inertia(:,:,12) = I12;
Inertia(:,:,13) = I13;
Inertia(:,:,14) = I14;
mass = [m1 m2 m3 m4 m5 m6 m7 m8 m9 m10 m11 m12 m13 m14];
inertial_disp(1,:) = inertial_disp_1;
inertial_disp(2,:) = inertial_disp_2;
inertial_disp(3,:) = inertial_disp_3;
inertial_disp(4,:) = inertial_disp_4;
inertial_disp(5,:) = inertial_disp_5;
inertial_disp(6,:) = inertial_disp_6;
inertial_disp(7,:) = inertial_disp_7;
inertial_disp(8,:) = inertial_disp_8;
inertial_disp(9,:) = inertial_disp_9;
inertial_disp(10,:) = inertial_disp_10;
inertial_disp(11,:) = inertial_disp_11;
inertial_disp(12,:) = inertial_disp_12;
inertial_disp(13,:) = inertial_disp_13;
inertial_disp(14,:) = inertial_disp_14;
friction = [fv1 fv2 fv3 fv4 fv5 fv6 fv7 fv8 fv9 fv10 fv11 fv12 fv13 fv14];

% Store data regarding the frames in a struct
info = struct('n_joints', n_joints, ...
              'n_links', n_links, ...
              'n_ee', n_ee, ...
              'current_frame_index',  [ 1 2 3 4 5   6 7 8  9 10 11 12 13 14 15 16], ...
              'previous_frame_index', [ 0 1 2 3 4   5 6 7  8  9  5 11 12 13 10 14], ...
              'next_frame_index',     [ 2 3 4 5 6 711 8 9 10 15 12 13 14 16  0  0], ...
              'next_frame_type',      [ 0 0 0 0 0   2 0 0  0  0  0  0  0  0  0  0], ...         % 0 for single frame / 1 for duplex next frame (frameA(1 digit) - frameB(1 digit)) / 2 for duplex next frame (frameA(1 digit) - frameB(2 digit))
              'previous_joint_index', [ 1 2 3 4 5   6 7 8  9 10 11 12 13 14  0  0], ...
              'frame_type',           [ 1 1 1 1 1   1 1 1  1  1  1  1  1  1  0  0], ...         % 1 for link, 0 for ee
              'explored',             [ 0 0 0 0 0   0 0 0  0  0  0  0  0  0  0  0], ...         % 0 for unxeplored, 1 for explored 
              'S', S, ...
              'M_bi', M_bi, ...
              'Inertia', Inertia, ...
              'mass', mass, ...
              'inertial_disp', inertial_disp, ...
              'F', [zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) F_ee_A F_ee_B]);                                 

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
Fv = diag([friction(1), friction(2), friction(3), friction(4), friction(5), friction(6), friction(7), friction(8), friction(9), friction(10) friction(11), friction(12), friction(13), friction(14)]);
msg = append('n vector computation');
n = recursive_invdyn_tree_sym_f(TH, dTH, zeros(1,n_joints), g, info, msg) + Fv*dTH';
