%% PASSIVE SPHERICAL JOINT + JOINTS 1 AND 2 OF BOTH ARMS 
% This scripts is used to obtain symbolically the B matrix and the n vector of the proposed model 

close all
clear
clc

if ispc % Windows
    addpath('..\..\..\functions\screw_theory_symbolic_functions\')
else % Linux
    addpath('../../../functions/screw_theory_symbolic_functions')
end

% Variables:
    % qCx - cable/platform joint roll 
    % qCx - cable/platform joint pitch
    % qAL1 - left arm joint 1
    % qAL2 - left arm joint 2
    % qAR1 - right arm joint 1
    % qAR2 - right arm joint 2
    
syms qCx qCy qAL1 qAL2 qAR1 qAR2...
     qCxd qCyd qAL1d qAL2d qAR1d qAR2d

% joint positions
TH = [qCx qCy qAL1 qAL2 qAR1 qAR2]; 
% joint velocities
dTH = [qCxd qCyd qAL1d qAL2d qAR1d qAR2d];

syms LC LA L1 mC mA1 mA2 off ...
     iA1xx iA2xx iC1xx iC2xx iC3xx ...
     iA1yy iA2yy iC1yy iC2yy iC3yy ...
     iA1zz iA2zz iC1zz iC2zz iC3zz ...
     lCz lA1z lA2z ...
     g0 fvCy fvCz fvA1 fvA2 fsCy fsCz fvCx fsA fsCx real;

% intertia tensors
I1 = [ iC1xx, 0, 0; 0, iC1yy, 0; 0, 0, iC1zz];
I2 = [ iC2xx, 0, 0; 0, iC2yy, 0; 0, 0, iC2zz];
I3 = [ iA1xx, 0, 0; 0, iA1yy, 0; 0, 0, iA1zz];
I4 = [ iA2xx, 0, 0; 0, iA2yy, 0; 0, 0, iA2zz];
I5 = [ iA1xx, 0, 0; 0, iA1yy, 0; 0, 0, iA1zz];
I6 = [ iA2xx, 0, 0; 0, iA2yy, 0; 0, 0, iA2zz];

% inertial displacements
inertial_disp_1 = [0 0 -lCz]';
inertial_disp_2 = [0 0 -lCz]';
inertial_disp_3 = [0 0 -lA1z]';
inertial_disp_4 = [0 0 -lA2z]';
inertial_disp_5 = [0 0 -lA1z]';
inertial_disp_6 = [0 0 -lA2z]';

% Chain definition
n_links = 6;
n_ee = 2;
n_joints = 6;

% Dynamic parameters
g = [0 0 -g0]';
F_ee_A = zeros(6,1);
F_ee_B = zeros(6,1);

% Kinematic parameters
omega1 = [1 0 0]';              % (passive x joint)
omega2 = [0 1 0]';              % (passive y joint)
omega3 = [0 0 -1]';             % (left arm)
omega4 = [0 1 0]';              % (left arm)
omega5 = [0 0 -1]';             % (right arm)
omega6 = [0 -1 0]';             % (right arm)

q_1 = [0 0 0]';                % (passive x joint)
q_2 = [0 0 0]';                % (passive y joint)
q_3 = [0 off -LC]';             % (left arm)
q_4 = [0 off -LC]';             % (left arm)
q_5 = [0 -off -LC]';            % (right arm)
q_6 = [0 -off -LC]';            % (right arm)

% Screw axis computation
omega = [omega1 omega2 omega3 omega4 omega5 omega6];
q_ = [q_1 q_2 q_3 q_4 q_5 q_6];
S = sym(zeros(6,n_joints));
for i = 1:n_joints
    v = cross(-omega(:,i),q_(:,i));
    S(:,i) = [omega(:,i); v];
end

% Definition M_{b,i} matrices
M_b1 = [1  0  0  0;
        0  1  0  0;
        0  0  1  0;
        0  0  0  1];
M_b2 = [1  0  0  0;
        0  1  0  0;
        0  0  1  0;
        0  0  0  1];
M_b3 = [1  0  0  0;
        0  1  0  off;
        0  0  1  -LC;
        0  0  0  1];
M_b4 = [1  0  0  0;
        0  1  0  off;
        0  0  1  -LC;
        0  0  0  1];
M_b5 = [1  0  0  0;
        0  1  0  -off;
        0  0  1  -LC;
        0  0  0  1];
M_b6 = [1  0  0  0;
        0  1  0  -off;
        0  0  1  -LC;
        0  0  0  1];
M_b7 = [1  0  0  0;
        0  1  0  off;
        0  0  1  -LC-LA;
        0  0  0  1];
M_b8 = [1  0  0  0;
        0  1  0  -off;
        0  0  1  -LC-LA;
        0  0  0  1];
    
% Store datas in data-structures
M_bi = sym(zeros(4,4,8));
M_bi(:,:,1) = M_b1;
M_bi(:,:,2) = M_b2;
M_bi(:,:,3) = M_b3;
M_bi(:,:,4) = M_b4;
M_bi(:,:,5) = M_b5;
M_bi(:,:,6) = M_b6;
M_bi(:,:,7) = M_b7;
M_bi(:,:,8) = M_b8;
Inertia(:,:,1) = I1;
Inertia(:,:,2) = I2;
Inertia(:,:,3) = I3;
Inertia(:,:,4) = I4;
Inertia(:,:,5) = I5;
Inertia(:,:,6) = I6;
mass = [mC/2 mC/2 mA1 mA2 mA1 mA2];
inertial_disp(1,:) = inertial_disp_1;
inertial_disp(2,:) = inertial_disp_2;
inertial_disp(3,:) = inertial_disp_3;
inertial_disp(4,:) = inertial_disp_4;
inertial_disp(5,:) = inertial_disp_5;
inertial_disp(6,:) = inertial_disp_6;
damping = [fvCx fvCy fvA1 fvA2 fvA1 fvA2];
friction = [fsCx fsCy fsA fsA fsA fsA];

% Store data regarding the frames in a struct
info = struct('n_joints', n_joints, ...
              'n_links', n_links, ...
              'n_ee', n_ee, ...
              'current_frame_index',  [  1 2 3 4 5 6 7 8], ...
              'previous_frame_index', [  0 1 2 3 2 5 4 6], ...
              'next_frame_index',     [ 2 35 4 7 6 8 0 0], ...
              'next_frame_type',      [  0 1 0 0 0 0 0 0], ...         % 0 for single frame / 1 for duplex next frame (frameA(1 digit) - frameB(1 digit)) / 2 for duplex next frame (frameA(1 digit) - frameB(2 digit))
              'previous_joint_index', [  1 2 3 4 5 6 0 0], ...
              'frame_type',           [  1 1 1 1 1 1 0 0], ...         % 1 for link, 0 for ee
              'explored',             [  0 0 0 0 0 0 0 0], ...         % 0 for unxeplored, 1 for explored 
              'S', S, ...
              'M_bi', M_bi, ...
              'Inertia', Inertia, ...
              'mass', mass, ...
              'inertial_disp', inertial_disp, ...
              'F', [zeros(6,1) zeros(6,1) zeros(6,1) F_ee_A F_ee_B]);                                 
                             

%% model computation
% Computation B matrix 
disp('Computation B matrix\n')
B = sym(zeros(n_joints,n_joints));
for j=1:n_joints
    fake_acc = zeros(1,n_joints);
    fake_acc(j) = 1;
    msg = append('B matrix computation - Column ',int2str(j),'/',int2str(n_joints));
    B(:,j) = recursive_invdyn_tree_sym_f(TH, zeros(1,n_joints), fake_acc, zeros(3,1), info, msg);
end
B = simplify(B);

% Computation n vector
disp('Computation n vector\n')
Fv = diag([damping(1), damping(2), damping(3) damping(4) damping(5) damping(6)]);
Fs = diag([friction(1), friction(2), friction(3)]);
msg = append('n vector computation');
n = recursive_invdyn_tree_sym_f(TH, dTH, zeros(1,n_joints), g, info, msg) + Fv*dTH' + Fs*(tanh(dTH))';
n = simplify(n);
