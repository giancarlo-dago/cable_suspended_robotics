% CON SOLO GIUNTO PASSIVO SUPERIORE (ID Y)
% C sta per cables, A sta per arms, AL e AR sta per arms left e right

close all
clear
clc

if ispc % Windows
    addpath('..\..\..\..\functions\screw_theory_symbolic_functions\')
else % Linux
    addpath('../../../../functions/screw_theory_symbolic_functions')
end

% Variables
syms qC qAL1 qAR1 qAL2 qAR2 ...
     qCd qAL1d qAR1d qAL2d qAR2d ...
     qCdd qAL1dd qAR1dd qAL2dd qAR2dd real

TH = [qC qAL1 qAL2 qAR1 qAR2];
dTH = [qCd qAL1d qAL2d qAR1d qAR2d];
ddTH = [qCdd qAL1dd qAL2dd qAR1dd qAR2dd];

syms lCz fvC fsC mC iA1xx iA1yy iA1zz iA2xx iA2yy iA2zz iCxx iCyy iCzz real;
LC = 4.53;
LA1 = 0.35;
LA2 = 0.3910;
mA1 = 6.5;
mA2 = 3.7;
off = 0.19;
lA1z = 0.29;
lA2z = 0.2865;
g0 = 9.8;
fvA1 = 12;
fvA2 = 4.5;
fsA1 = 10;
fsA2 = 4.5;
 
I1 = [ iCxx, 0, 0; 0, iCyy, 0; 0, 0, iCzz ];
I2 = [ iA1xx, 0, 0; 0, iA1yy, 0; 0, 0, iA1zz ];
I3 = [ iA2xx, 0, 0; 0, iA2yy, 0; 0, 0, iA2zz ];
I4 = [ iA1xx, 0, 0; 0, iA1yy, 0; 0, 0, iA1zz ];
I5 = [ iA2xx, 0, 0; 0, iA2yy, 0; 0, 0, iA2zz ];

inertial_disp_1 = [0 0 -lCz]';
inertial_disp_2 = [0 0 -lA1z]';
inertial_disp_3 = [0 0 -lA2z]';
inertial_disp_4 = [0 0 -lA1z]';
inertial_disp_5 = [0 0 -lA2z]';

% Chain definition
n_links = 5;
n_ee = 2;
n_joints = 5;

% Dynamic parameters
g = [0 0 -g0]';
F_ee_A = zeros(6,1);
F_ee_B = zeros(6,1);

% Kinematic parameters
omega1 = [1 0 0]';              % (first passive x joint)
omega2 = [1 0 0]';              % (left arm first joint)
omega3 = [1 0 0]';              % (left arm second joint)
omega4 = [1 0 0]';              % (right arm first joint)
omega5 = [1 0 0]';              % (right arm second joint)

q_1 = [0 0 0]';                 % (first passive z joint)
q_2 = [0 -off -LC]';             % (left arm)
q_3 = [0 -off -LC-LA1]';         % (left arm)
q_4 = [0 off -LC]';            % (right arm)
q_5 = [0 off -LC-LA1]';        % (right arm)

% Screw axis computation
omega = [omega1 omega2 omega3 omega4 omega5];
q_ = [q_1 q_2 q_3 q_4 q_5];
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
        0  1  0  -off;
        0  0  1  -LC;
        0  0  0  1];
M_b3 = [1  0  0  0;
        0  1  0  -off;
        0  0  1  -LC;
        0  0  0  1];
M_b4 = [1  0  0  0;
        0  1  0  off;
        0  0  1  -LC-LA1;
        0  0  0  1];
M_b5 = [1  0  0  0;
        0  1  0  off;
        0  0  1  -LC-LA1;
        0  0  0  1];
M_b6 = [1  0  0  0;
        0  1  0  -off;
        0  0  1  -LC-LA1-LA2;
        0  0  0  1];
M_b7 = [1  0  0  0;
        0  1  0  off;
        0  0  1  -LC-LA1-LA2;
        0  0  0  1];
    
% Store datas in data-structures
M_bi = sym(zeros(4,4,7));
Inertia = sym(zeros(3,3,5));
M_bi(:,:,1) = M_b1;
M_bi(:,:,2) = M_b2;
M_bi(:,:,3) = M_b3;
M_bi(:,:,4) = M_b4;
M_bi(:,:,5) = M_b5;
M_bi(:,:,6) = M_b6;
M_bi(:,:,7) = M_b7;
Inertia(:,:,1) = I1;
Inertia(:,:,2) = I2;
Inertia(:,:,3) = I3;
Inertia(:,:,4) = I4;
Inertia(:,:,5) = I5;
mass = [mC mA1 mA2 mA1 mA2];
inertial_disp(1,:) = inertial_disp_1;
inertial_disp(2,:) = inertial_disp_2;
inertial_disp(3,:) = inertial_disp_3;
inertial_disp(4,:) = inertial_disp_4;
inertial_disp(5,:) = inertial_disp_5;
friction = [fvC fvA1 fvA2 fvA1 fvA2];
static_friction = [fsC fsA1 fsA2 fsA1 fsA2];

% Store data regarding the frames in a struct
info = struct('n_joints', n_joints, ...
              'n_links', n_links, ...
              'n_ee', n_ee, ...
              'current_frame_index',  [  1 2 3 4 5 6 7], ...
              'previous_frame_index', [  0 1 2 1 4 3 5], ...
              'next_frame_index',     [ 24 3 6 5 7 0 0], ...
              'next_frame_type',      [  1 0 0 0 0 0 0], ...         % 0 for single frame / 1 for duplex next frame (frameA(1 digit) - frameB(1 digit)) / 2 for duplex next frame (frameA(1 digit) - frameB(2 digit))
              'previous_joint_index', [  1 2 3 4 5 0 0], ...
              'frame_type',           [  1 1 1 1 1 0 0], ...         % 1 for link, 0 for ee
              'explored',             [  0 0 0 0 0 0 0], ...         % 0 for unxeplored, 1 for explored 
              'S', S, ...
              'M_bi', M_bi, ...
              'Inertia', Inertia, ...
              'mass', mass, ...
              'inertial_disp', inertial_disp, ...
              'F', [zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) F_ee_A F_ee_B]);

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
Fv = diag([friction(1), friction(2), friction(3), friction(4), friction(5)]);
Fs = diag([static_friction(1), static_friction(2), static_friction(3), static_friction(4), static_friction(5)]);
msg = append('n vector computation');
n = recursive_invdyn_tree_sym_f(TH, dTH, zeros(1,n_joints), g, info, msg) + Fv*dTH' + Fs*sign(dTH');
n = simplify(n)

%% CON SOLO GIUNTO PASSIVO SUPERIORE (ID X)
% C sta per cables, A sta per arms, AL e AR sta per arms left e right

close all
clear
clc

if ispc % Windows
    addpath('..\..\..\..\functions\screw_theory_symbolic_functions\')
else % Linux
    addpath('../../../../functions/screw_theory_symbolic_functions')
end

% Variables
syms qC qAL1 qAR1 qAL2 qAR2 ...
     qCd qAL1d qAR1d qAL2d qAR2d ...
     qCdd qAL1dd qAR1dd qAL2dd qAR2dd real

TH = [qC qAL1 qAL2 qAR1 qAR2];
dTH = [qCd qAL1d qAL2d qAR1d qAR2d];
ddTH = [qCdd qAL1dd qAL2dd qAR1dd qAR2dd];

syms lCz fvC fsC mC iA1xx iA1yy iA1zz iA2xx iA2yy iA2zz iCxx iCyy iCzz real;
% LC = 4.53;
% LA1 = 0.35;
% LA2 = 0.3910;
% mA1 = 1.7;
% mA2 = 8.5;
% off = 0.19;
% % iA1xx = 0;
% % iA2xx = 0;
% % iCxx = 0;
% lA1z = 0.162;
% lA2z = 0.122729411764706;
% g0 = 9.8;
% fvA1 = 12;
% fvA2 = 4.5;
% fsA1 = 10;
% fsA2 = 4.5;

LC = 4.53;
LA1 = 0.35;
LA2 = 0.3910;
mA1 = 6.5;
mA2 = 3.7;
off = 0.19;
% iA1yy = 0.05;
% iA2yy = 0.05;
lA1z = 0.29;
lA2z = 0.2865;
g0 = 9.8;
fvA1 = 12;
fvA2 = 4.5;
fsA1 = 10;
fsA2 = 4.5;

I1 = [ iCxx, 0, 0; 0, iCyy, 0; 0, 0, iCzz ];
I2 = [ iA1xx, 0, 0; 0, iA1yy, 0; 0, 0, iA1zz ];
I3 = [ iA2xx, 0, 0; 0, iA2yy, 0; 0, 0, iA2zz ];
I4 = [ iA1xx, 0, 0; 0, iA1yy, 0; 0, 0, iA1zz ];
I5 = [ iA2xx, 0, 0; 0, iA2yy, 0; 0, 0, iA2zz ];

inertial_disp_1 = [0 0 -lCz]';
inertial_disp_2 = [0 0 -lA1z]';
inertial_disp_3 = [0 0 -lA2z]';
inertial_disp_4 = [0 0 -lA1z]';
inertial_disp_5 = [0 0 -lA2z]';

% Chain definition
n_links = 5;
n_ee = 2;
n_joints = 5;

% Dynamic parameters
g = [0 0 -g0]';
F_ee_A = zeros(6,1);
F_ee_B = zeros(6,1);

% Kinematic parameters
omega1 = [0 1 0]';              % (first passive x joint)
omega2 = [0 1 0]';              % (left arm first joint)
omega3 = [0 1 0]';              % (left arm second joint)
omega4 = [0 1 0]';              % (right arm first joint)
omega5 = [0 1 0]';              % (right arm second joint)

q_1 = [0 0 0]';                 % (first passive z joint)
q_2 = [0 -off -LC]';             % (left arm)
q_3 = [0 -off -LC-LA1]';         % (left arm)
q_4 = [0 off -LC]';            % (right arm)
q_5 = [0 off -LC-LA1]';        % (right arm)

% Screw axis computation
omega = [omega1 omega2 omega3 omega4 omega5];
q_ = [q_1 q_2 q_3 q_4 q_5];
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
        0  1  0  -off;
        0  0  1  -LC;
        0  0  0  1];
M_b3 = [1  0  0  0;
        0  1  0  -off;
        0  0  1  -LC;
        0  0  0  1];
M_b4 = [1  0  0  0;
        0  1  0  off;
        0  0  1  -LC-LA1;
        0  0  0  1];
M_b5 = [1  0  0  0;
        0  1  0  off;
        0  0  1  -LC-LA1;
        0  0  0  1];
M_b6 = [1  0  0  0;
        0  1  0  -off;
        0  0  1  -LC-LA1-LA2;
        0  0  0  1];
M_b7 = [1  0  0  0;
        0  1  0  off;
        0  0  1  -LC-LA1-LA2;
        0  0  0  1];
    
% Store datas in data-structures
M_bi = sym(zeros(4,4,7));
Inertia = sym(zeros(3,3,5));
M_bi(:,:,1) = M_b1;
M_bi(:,:,2) = M_b2;
M_bi(:,:,3) = M_b3;
M_bi(:,:,4) = M_b4;
M_bi(:,:,5) = M_b5;
M_bi(:,:,6) = M_b6;
M_bi(:,:,7) = M_b7;
Inertia(:,:,1) = I1;
Inertia(:,:,2) = I2;
Inertia(:,:,3) = I3;
Inertia(:,:,4) = I4;
Inertia(:,:,5) = I5;
mass = [mC mA1 mA2 mA1 mA2];
inertial_disp(1,:) = inertial_disp_1;
inertial_disp(2,:) = inertial_disp_2;
inertial_disp(3,:) = inertial_disp_3;
inertial_disp(4,:) = inertial_disp_4;
inertial_disp(5,:) = inertial_disp_5;
friction = [fvC fvA1 fvA2 fvA1 fvA2];
static_friction = [fsC fsA1 fsA2 fsA1 fsA2];

% Store data regarding the frames in a struct
info = struct('n_joints', n_joints, ...
              'n_links', n_links, ...
              'n_ee', n_ee, ...
              'current_frame_index',  [  1 2 3 4 5 6 7], ...
              'previous_frame_index', [  0 1 2 1 4 3 5], ...
              'next_frame_index',     [ 24 3 6 5 7 0 0], ...
              'next_frame_type',      [  1 0 0 0 0 0 0], ...         % 0 for single frame / 1 for duplex next frame (frameA(1 digit) - frameB(1 digit)) / 2 for duplex next frame (frameA(1 digit) - frameB(2 digit))
              'previous_joint_index', [  1 2 3 4 5 0 0], ...
              'frame_type',           [  1 1 1 1 1 0 0], ...         % 1 for link, 0 for ee
              'explored',             [  0 0 0 0 0 0 0], ...         % 0 for unxeplored, 1 for explored 
              'S', S, ...
              'M_bi', M_bi, ...
              'Inertia', Inertia, ...
              'mass', mass, ...
              'inertial_disp', inertial_disp, ...
              'F', [zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) F_ee_A F_ee_B]);

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
Fv = diag([friction(1), friction(2), friction(3), friction(4), friction(5)]);
Fs = diag([static_friction(1), static_friction(2), static_friction(3), static_friction(4), static_friction(5)]);
msg = append('n vector computation');
n = recursive_invdyn_tree_sym_f(TH, dTH, zeros(1,n_joints), g, info, msg) + Fv*dTH' + Fs*sign(dTH');
n = simplify(n)

%% CON SOLO GIUNTO PASSIVO SUPERIORE (ID Z) - 3  GIUNTI (uno per braccio)
% C sta per cables, A sta per arms, AL e AR sta per arms left e right

close all
clear
clc

if ispc % Windows
    addpath('..\..\..\..\functions\screw_theory_symbolic_functions\')
else % Linux
    addpath('../../../../functions/screw_theory_symbolic_functions')
end

% Variables
syms qC qAL1 qAR1 ...
     qCd qAL1d qAR1d ...
     qCdd qAL1dd qAR1dd real

TH = [qC qAL1 qAR1];
dTH = [qCd qAL1d qAR1d];
ddTH = [qCdd qAL1dd qAR1dd];

syms lCz fvC fsC mC iCxx iCyy iCzz iA1xx iA1yy iA1zz real;
mC = 206;
LC = 4.53;
LA1 = 0.741;
mA1 = 10.2;
off = 0.19;
iA1xx = 0.417;
iA1yy = 0.380;
iA1zz = 0.054;
lA1z = 0.4157;
g0 = 9.8;
fvA1 = 12;
fsA1 = 10;

I1 = [ 0, 0, 0; 0, 0, 0; 0, 0, iCzz ];
I2 = [ iA1xx, 0, 0; 0, iA1yy, 0; 0, 0, iA1zz ];
I3 = [ iA1xx, 0, 0; 0, iA1yy, 0; 0, 0, iA1zz ];

inertial_disp_1 = [0 0 -lCz]';
inertial_disp_2 = [0 0 -lA1z]';
inertial_disp_3 = [0 0 -lA1z]';

% Chain definition
n_links = 3;
n_ee = 2;
n_joints = 3;

% Dynamic parameters
g = [0 0 -g0]';
F_ee_A = zeros(6,1);
F_ee_B = zeros(6,1);

% Kinematic parameters
omega1 = [0 0 1]';              % (first passive z joint)
omega2 = [0 -1 0]';              % (left arm first joint)
omega3 = [0 1 0]';              % (right arm first joint)

q_1 = [0 0 0]';                 % (first passive z joint)
q_2 = [0 -off -LC]';            % (left arm)
q_3 = [0 off -LC]';             % (right arm)

% Screw axis computation
omega = [omega1 omega2 omega3];
q_ = [q_1 q_2 q_3];
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
        0  1  0  -off;
        0  0  1  -LC;
        0  0  0  1];
M_b3 = [1  0  0  0;
        0  1  0  off;
        0  0  1  -LC;
        0  0  0  1];
M_b4 = [1  0  0  0;
        0  1  0  -off;
        0  0  1  -LC-LA1;
        0  0  0  1];
M_b5 = [1  0  0  0;
        0  1  0  off;
        0  0  1  -LC-LA1;
        0  0  0  1];
    
% Store datas in data-structures
M_bi = sym(zeros(4,4,5));
Inertia = sym(zeros(3,3,3));
M_bi(:,:,1) = M_b1;
M_bi(:,:,2) = M_b2;
M_bi(:,:,3) = M_b3;
M_bi(:,:,4) = M_b4;
M_bi(:,:,5) = M_b5;
Inertia(:,:,1) = I1;
Inertia(:,:,2) = I2;
Inertia(:,:,3) = I3;
mass = [mC mA1 mA1];
inertial_disp(1,:) = inertial_disp_1;
inertial_disp(2,:) = inertial_disp_2;
inertial_disp(3,:) = inertial_disp_3;
friction = [fvC fvA1 fvA1];
static_friction = [fsC fsA1 fsA1];

% Store data regarding the frames in a struct
info = struct('n_joints', n_joints, ...
              'n_links', n_links, ...
              'n_ee', n_ee, ...
              'current_frame_index',  [  1 2 3 4 5], ...
              'previous_frame_index', [  0 1 1 2 3], ...
              'next_frame_index',     [ 23 4 5 0 0], ...
              'next_frame_type',      [  1 0 0 0 0], ...         % 0 for single frame / 1 for duplex next frame (frameA(1 digit) - frameB(1 digit)) / 2 for duplex next frame (frameA(1 digit) - frameB(2 digit))
              'previous_joint_index', [  1 2 3 0 0], ...
              'frame_type',           [  1 1 1 0 0], ...         % 1 for link, 0 for ee
              'explored',             [  0 0 0 0 0], ...         % 0 for unxeplored, 1 for explored 
              'S', S, ...
              'M_bi', M_bi, ...
              'Inertia', Inertia, ...
              'mass', mass, ...
              'inertial_disp', inertial_disp, ...
              'F', [zeros(6,1) zeros(6,1) zeros(6,1) F_ee_A F_ee_B]);

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
Fv = diag([friction(1), friction(2), friction(3)]);
Fs = diag([static_friction(1), static_friction(2), static_friction(3)]);
msg = append('n vector computation');
n = recursive_invdyn_tree_sym_f(TH, dTH, zeros(1,n_joints), g, info, msg) + Fv*dTH' + Fs*sign(dTH');
n = simplify(n)


