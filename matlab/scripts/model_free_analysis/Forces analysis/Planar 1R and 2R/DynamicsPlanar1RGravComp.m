%% DYNAMICS PLANAR 1R PLANE (GRAVITY COMPENSATED)

close all
clear
clc

addpath('../../../../functions/screw_theory_symbolic_functions')

% Variables
syms qA ...
     qAd ...
     qAdd real

TH = qA;
dTH = qAd;
ddTH = qAdd;

syms lAx lAy lAz fvA fsA mA iAxx iAyy iAzz LA g0 real;
syms pAy pAz real;

IA = [ iAxx, 0, 0; 0, iAyy, 0; 0, 0, iAzz ];
inertial_disp_A = [0 0 -lAz]';

% Chain definition
n_links = 1;
n_ee = 1;
n_joints = 1;

% Dynamic parameters
g = [-g0 0 0]';
F_ee = zeros(6,1);

% Kinematic parameters
omega1 = [1 0 0]';              % (first joint)
q_1 = [0 0 0]';                 % (first joint)

% Screw axis computation
omega = omega1;
q_ = q_1;
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
        0  0  1  -LA;
        0  0  0  1];

% Store datas in data-structures
M_bi = sym(zeros(4,4,3));
Inertia = sym(zeros(3,3,2));
M_bi(:,:,1) = M_b1;
M_bi(:,:,2) = M_b2;
Inertia(:,:,1) = IA;
mass = mA;
inertial_disp(1,:) = inertial_disp_A;
viscous_friction = fvA;
static_friction = fsA;

% Store data regarding the frames in a struct
info = struct('n_joints', n_joints, ...
              'n_links', n_links, ...
              'n_ee', n_ee, ...
              'current_frame_index',  [ 1 2 ], ...
              'previous_frame_index', [ 0 1 ], ...
              'next_frame_index',     [ 2 0 ], ...
              'next_frame_type',      [ 0 0 ], ...         % 0 for single frame / 1 for duplex next frame (frameA(1 digit) - frameB(1 digit)) / 2 for duplex next frame (frameA(1 digit) - frameB(2 digit))
              'previous_joint_index', [ 1 0 ], ...
              'frame_type',           [ 1 0 ], ...         % 1 for link, 0 for ee
              'explored',             [ 0 0 ], ...         % 0 for unxeplored, 1 for explored 
              'S', S, ...
              'M_bi', M_bi, ...
              'Inertia', Inertia, ...
              'mass', mass, ...
              'inertial_disp', inertial_disp, ...
              'F', [zeros(6,1) F_ee]);

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
Fv = diag([viscous_friction(1)]);
Fs = diag([static_friction(1)]);
msg = append('n vector computation');
n = recursive_invdyn_tree_sym_f(TH, dTH, zeros(1,n_joints), g, info, msg) + Fv*dTH' + Fs*sign(dTH');
n = simplify(n)

% Wrench computation
[F, F_own, F_ext] = (recursive_force_tree_sym_f(TH, dTH, ddTH, g, info, msg));
F = simplify(F)
F_own = simplify(F_own)
F_ext = simplify(F_ext)

R_w1 = [1 0 0; 0 cos(qA) -sin(qA); 0 sin(qA) cos(qA)];
F_own_world = simplify(R_w1*F_own(4:6))


FyFun = matlabFunction(F_own_world(2));
FzFun = matlabFunction(F_own_world(3));
