%% DYNAMICS PLANAR 2R

close all
clear
clc

addpath('../../../../functions/screw_theory_symbolic_functions')

% Chain definition
n_links = 2;
n_ee = 1;
n_joints = 2;

% Variables
syms qC qA ...
     qCd qAd ...
     qCdd qAdd real

TH = [qC qA];
dTH = [qCd qAd];
ddTH = [qCdd qAdd];

syms g0 LC LA mC mA iCxx iAxx iCyy iAyy iCzz iAzz lC lA fvC fvA fsC fsA real
IC = [ iCxx, 0, 0; 0, iCyy, 0; 0, 0, iCzz];
IA = [ iAxx, 0, 0; 0, iAyy, 0; 0, 0, iAzz];
inertial_disp_1 = [0 0 0]';
inertial_disp_2 = [0 0 0]';

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
q_2 = [0 0 -LC]';               % (second joint)

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
        0  0  1  -lC;
        0  0  0  1];
M_b2 = [1  0  0  0;
        0  1  0  0;
        0  0  1  -LC-lA;
        0  0  0  1];
M_be = [1  0  0  0;
        0  1  0  0;
        0  0  1  -LC-LA;
        0  0  0  1];
    
% Store datas in data-structures
M_bi = sym(zeros(4,4,3));
M_bi(:,:,1) = M_b1;
M_bi(:,:,2) = M_b2;
M_bi(:,:,3) = M_be;
Inertia(:,:,1) = IC;
Inertia(:,:,2) = IA;
mass = [mC mA];
inertial_disp(1,:) = inertial_disp_1;
inertial_disp(2,:) = inertial_disp_2;
viscous_friction = [fvC fvA];
static_friction = [fsC fsA];

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
Fv = diag([viscous_friction(1), viscous_friction(2)]);
Fs = diag([static_friction(1), static_friction(2)]);
msg = append('n vector computation');
n = recursive_invdyn_tree_sym_f(TH, dTH, zeros(1,n_joints), g, info, msg) + Fv*dTH' + Fs*sign(dTH');
n = simplify(n)

% Wrench computation
[F, F_own, F_ext, tau_own, tau_ext] = (recursive_force_tree_sym_f(TH, dTH, ddTH, g, info, msg));
F = simplify(F)
F_own = simplify(F_own)
F_ext = simplify(F_ext)
tau_own = simplify(tau_own)
tau_ext = simplify(tau_ext)

tauOwnFun = matlabFunction(tau_own(1));
tauExtFun = matlabFunction(tau_ext(1));

% gravityFun = matlabFunction(simplify(tauExtFun(LC,g0,iAxx,lA,mA,qA,qAd,qAdd,qC,qCd,qCdd) - tauExtFun(LC,0,iAxx,lA,mA,qA,qAd,qAdd,qC,qCd,qCdd)));
gravityFun = matlabFunction(simplify(tauExtFun(LC,g0,iAxx,lA,mA,qA,qAd,qAdd,0,0,0) - tauExtFun(LC,0,iAxx,lA,mA,qA,qAd,qAdd,0,0,0)));

% Collocated PFBL
b11 = B(1,1);
b12 = B(1,2);
b21 = B(2,1);
b22 = B(2,2);
n1 = n(1);
n2 = n(2);

% collFun = matlabFunction(simplify(b11*qCdd + n1 + b12*qAdd))
% collFun = simplify(b11*qCdd + n1 + b12*qAdd)
