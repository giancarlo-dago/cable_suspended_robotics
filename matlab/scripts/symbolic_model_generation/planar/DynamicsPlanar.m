close all
clear
clc

if ispc % Windows
    addpath('..\..\..\functions\screw_theory_symbolic_functions\')
else % Linux
    addpath('../../../functions/screw_theory_symbolic_functions')
end

format long

% Variables
TH = str2sym(compose("q" + (1:2)));
dTH = str2sym(compose("qd" + (1:2)));
ddTH = str2sym(compose("qdd" + (1:2)));
assume(TH,'real');
assume(dTH,'real');
assume(ddTH,'real');


%%
syms g0 L1 L2 m1 m2 i1xx i2xx l1 l2 fv1 fv2 real
I1 = [ i1xx, 0, 0; 0, 0, 0; 0, 0, 0];
I2 = [ i2xx, 0, 0; 0, 0, 0; 0, 0, 0];
inertial_disp_1 = [ 0 0 -l1]';
inertial_disp_2 = [ 0 0 -l2]';

% Chain definition
n_links = 2;
n_ee = 1;
n_joints = 2;

% Dynamic parameters
g = [0 0 -g0]';
F_ee = zeros(6,1);

% Kinematic parameters
omega1 = [1 0 0]';              % (first joint)
omega2 = [1 0 0]';              % (second joint)

q_1 = [0 0 0]';                 % (first joint)
q_2 = [0 0 -L1]';               % (second joint)

% Screw axis computation
omega = [omega1 omega2];
q_ = [q_1 q_2];
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
        0  0  1  -L1;
        0  0  0  1];
M_be = [1  0  0  0;
        0  1  0  0;
        0  0  1  -L1-L2;
        0  0  0  1];
    
% Store datas in data-structures
M_bi = sym(zeros(4,4,3));
M_bi(:,:,1) = M_b1;
M_bi(:,:,2) = M_b2;
M_bi(:,:,3) = M_be;
Inertia(:,:,1) = I1;
Inertia(:,:,2) = I2;
mass = [m1 m2];
inertial_disp(1,:) = inertial_disp_1;
inertial_disp(2,:) = inertial_disp_2;
friction = [fv1 fv2];

% Store data regarding the frames in a struct
info = struct('n_joints', n_joints, ...
              'n_links', n_links, ...
              'n_ee', n_ee, ...
              'current_frame_index',  [ 1 2 3], ...
              'previous_frame_index', [ 0 1 2], ...
              'next_frame_index',     [ 2 3 0], ...
              'next_frame_type',      [ 0 0 0], ...         % 0 for single frame / 1 for duplex next frame (frameA(1 digit) - frameB(1 digit)) / 2 for duplex next frame (frameA(1 digit) - frameB(2 digit))
              'previous_joint_index', [ 1 2 0], ...
              'frame_type',           [ 1 1 0], ...         % 1 for link, 0 for ee
              'explored',             [ 0 0 0], ...         % 0 for unxeplored, 1 for explored 
              'S', S, ...
              'M_bi', M_bi, ...
              'Inertia', Inertia, ...
              'mass', mass, ...
              'inertial_disp', inertial_disp, ...
              'F', [zeros(6,1) zeros(6,1) F_ee]);                                 
  
%%
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
Fv = diag([friction(1), friction(2)]);
msg = append('n vector computation');
n = recursive_invdyn_tree_sym_f(TH, dTH, zeros(1,n_joints), g, info, msg) + Fv*dTH';
n = simplify(n)


