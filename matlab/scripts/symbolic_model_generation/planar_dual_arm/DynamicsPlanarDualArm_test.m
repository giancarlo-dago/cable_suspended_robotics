%% LA VARIABILE Q2 VIENE POSTA =-Q1 (SHOULDER HORIZONTALITY) E CON BRACCIA CHE FANNO LO STESSO MOVIMENTO
 
close all
clear
clc

if ispc % Windows
    addpath('..\..\..\functions\screw_theory_symbolic_functions\')
else % Linux
    addpath('../../../functions/screw_theory_symbolic_functions')
end

% Variables
syms q1 q...
     qd1 qd...
     qdd1 qdd real
 
TH = [q1 -q1 q q];
dTH = [qd1 -qd1 qd qd];
ddTH = [qdd1 -qdd1 qdd qdd];

syms L L1 m1 m2 m off ...
     i1xx i2xx ixx ...
     l1z lz ...
     g0 D real;

% L = 1;
% off = 0.15;
% L1 = 0.5;
% m1 = 1;
% m2 = 1;
% m = 0.5;

% Inertia tensors
I1 = [ i1xx, 0, 0; 0, 0, 0; 0, 0, 0];   % Hp: inertia tensors are the same for each link modeling the cables
I2 = [ i2xx, 0, 0; 0, 0, 0; 0, 0, 0];
I3 = [ ixx, 0, 0; 0, 0, 0; 0, 0, 0];
I4 = [ ixx, 0, 0; 0, 0, 0; 0, 0, 0];

% CoM position 
inertial_disp_1 = [0 0 -l1z]';      % Cable CoM - rotation around x
inertial_disp_2 = [0 0 0]';
inertial_disp_3 = [0 0 -lz]';
inertial_disp_4 = [0 0 -lz]';

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
omega4 = [1 0 0]';              % (right arm)

q_1 = [0 0 0]';                  % (first passive z joint)
q_2 = [0 0 -L]';                  % (second passive z joint)
q_3 = [0 off -L-D]';               % (left arm)
q_4 = [0 -off -L-D]';             % (right arm)

% Screw axis computation
omega = [omega1 omega2 omega3 omega4];
q_ = [q_1 q_2 q_3 q_4];
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
        0  0  1  -L;
        0  0  0  1];
M_b3 = [1  0  0  0;
        0  1  0  off;
        0  0  1  -L-D;
        0  0  0  1];
M_b4 = [1  0  0  0;
        0  1  0  -off;
        0  0  1  -L-D;
        0  0  0  1];
M_b5 = [1  0  0  0;
        0  1  0  off;
        0  0  1  -L-D-L1;
        0  0  0  1];
M_b6 = [1  0  0  0;
        0  1  0  -off;
        0  0  1  -L-D-L1;
        0  0  0  1];
    
% Store datas in data-structures
M_bi = sym(zeros(4,4,6));
M_bi(:,:,1) = M_b1;
M_bi(:,:,2) = M_b2;
M_bi(:,:,3) = M_b3;
M_bi(:,:,4) = M_b4;
M_bi(:,:,5) = M_b5;
M_bi(:,:,6) = M_b6;
Inertia(:,:,1) = I1;
Inertia(:,:,2) = I2;
Inertia(:,:,3) = I3;
Inertia(:,:,4) = I4;
mass = [m1 m2 m m];
inertial_disp(1,:) = inertial_disp_1;
inertial_disp(2,:) = inertial_disp_2;
inertial_disp(3,:) = inertial_disp_3;
inertial_disp(4,:) = inertial_disp_4;
friction = [0 0 0 0];

% Store data regarding the frames in a struct
info = struct('n_joints', n_joints, ...
              'n_links', n_links, ...
              'n_ee', n_ee, ...
              'current_frame_index',  [  1  2 3 4 5 6], ...
              'previous_frame_index', [  0  1 2 2 3 4], ...
              'next_frame_index',     [  2 34 5 6 0 0], ...
              'next_frame_type',      [  0  1 0 0 0 0], ...         % 0 for single frame / 1 for duplex next frame (frameA(1 digit) - frameB(1 digit)) / 2 for duplex next frame (frameA(1 digit) - frameB(2 digit))
              'previous_joint_index', [  1  2 3 4 0 0], ...
              'frame_type',           [  1  1 1 1 0 0], ...         % 1 for link, 0 for ee
              'explored',             [  0  0 0 0 0 0], ...         % 0 for unxeplored, 1 for explored 
              'S', S, ...
              'M_bi', M_bi, ...
              'Inertia', Inertia, ...
              'mass', mass, ...
              'inertial_disp', inertial_disp, ...
              'F', [zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) F_ee_A F_ee_B]);                                 

% Computation B matrix 
disp('Computation B matrix\n')
B = sym(zeros(n_joints,n_joints));
for j=1:n_joints
    fake_acc = zeros(1,n_joints);
    fake_acc(j) = 1;
    msg = append('B matrix computation - Column ',int2str(j),'/',int2str(n_joints));
    B(:,j) = recursive_invdyn_tree_sym_f(TH, zeros(1,n_joints), fake_acc, zeros(3,1), info, msg);
end
B = simplify(B)

% Computation n vector
disp('Computation n vector\n')
Fv = diag([friction(1), friction(2), friction(3), friction(4)]);
msg = append('n vector computation');
n = recursive_invdyn_tree_sym_f(TH, dTH, zeros(1,n_joints), g, info, msg) + Fv*dTH';
n = simplify(n)
