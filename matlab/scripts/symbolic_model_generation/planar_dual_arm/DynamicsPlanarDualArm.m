%% CON SOLO GIUNTO PASSIVO SUPERIORE
% C sta per cables, A sta per arms, AL e AR sta per arms left e right

close all
clear
clc

if ispc % Windows
    addpath('..\..\..\functions\screw_theory_symbolic_functions\')
else % Linux
    addpath('../../../functions/screw_theory_symbolic_functions')
end

% Variables
syms qC qAL qAR ...
     qCd qALd qARd ...
     qCdd qALdd qARdd real

TH = [qC qAL qAR];
dTH = [qCd qALd qARd];
ddTH = [qCdd qALdd qARdd];

syms LC LA mC mA off ...
     iAxx iCxx ...
     lCz lAz ...
     g0 fvC fvA real;

I1 = [ iCxx, 0, 0; 0, 0, 0; 0, 0, 0];
I2 = [ iAxx, 0, 0; 0, 0, 0; 0, 0, 0];
I3 = [ iAxx, 0, 0; 0, 0, 0; 0, 0, 0];

inertial_disp_1 = [0 0 -lCz]';
inertial_disp_2 = [0 0 -lAz]';
inertial_disp_3 = [0 0 -lAz]';

% Chain definition
n_links = 3;
n_ee = 2;
n_joints = 3;

% Dynamic parameters
g = [0 0 -g0]';
F_ee_A = zeros(6,1);
F_ee_B = zeros(6,1);

% Kinematic parameters
omega1 = [1 0 0]';              % (first passive x joint)
omega2 = [1 0 0]';              % (left arm)
omega3 = [1 0 0]';              % (right arm)

q_1 = [0 0 0]';                 % (first passive z joint)
q_2 = [0 off -LC]';             % (left arm)
q_3 = [0 -off -LC]';            % (right arm)

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
        0  1  0  off;
        0  0  1  -LC;
        0  0  0  1];
M_b3 = [1  0  0  0;
        0  1  0  -off;
        0  0  1  -LC;
        0  0  0  1];
M_b4 = [1  0  0  0;
        0  1  0  off;
        0  0  1  -LC-LA;
        0  0  0  1];
M_b5 = [1  0  0  0;
        0  1  0  -off;
        0  0  1  -LC-LA;
        0  0  0  1];
    
% Store datas in data-structures
M_bi = sym(zeros(4,4,5));
M_bi(:,:,1) = M_b1;
M_bi(:,:,2) = M_b2;
M_bi(:,:,3) = M_b3;
M_bi(:,:,4) = M_b4;
M_bi(:,:,5) = M_b5;
Inertia(:,:,1) = I1;
Inertia(:,:,2) = I2;
Inertia(:,:,3) = I3;
mass = [mC mA mA];
inertial_disp(1,:) = inertial_disp_1;
inertial_disp(2,:) = inertial_disp_2;
inertial_disp(3,:) = inertial_disp_3;
friction = [fvC fvA fvA];

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
msg = append('n vector computation');
n = recursive_invdyn_tree_sym_f(TH, dTH, zeros(1,n_joints), g, info, msg) + Fv*dTH';
n = simplify(n)



%% CON SOLO GIUNTO PASSIVO SUPERIORE E CON BRACCIA CHE FANNO LO STESSO MOVIMENTO

close all
clear
clc

if ispc % Windows
    addpath('..\..\..\functions\screw_theory_symbolic_functions\')
else % Linux
    addpath('../../../functions/screw_theory_symbolic_functions')
end

% Variables
syms q1 q ...
     qd1 qd ...
     qdd1 qdd real

TH = [q1 q q];
dTH = [qd1 qd qd];
ddTH = [qdd1 qdd qdd];

syms L L1 m m1 off ...
    i1xx ixx ...
    l1z lz ...
    g0 real;

I1 = [ i1xx, 0, 0; 0, 0, 0; 0, 0, 0];   % non influenza
I2 = [ ixx, 0, 0; 0, 0, 0; 0, 0, 0];    % influenza
I3 = [ ixx, 0, 0; 0, 0, 0; 0, 0, 0];    % influenza

inertial_disp_1 = [0 0 -l1z]';      % non influenza
inertial_disp_2 = [0 0 -lz]';       % influenza
inertial_disp_3 = [0 0 -lz]';       % influenza

% L = 1;          % influenza
% off = 0.15;     % influenza  
% L1 = 0.5;       % non influenza
% m1 = 1;         % non influenza
% m = 0.5;      % influenza

% Inertia tensors
% I1 = [ 0.1, 0, 0; 0, 0, 0; 0, 0, 0];   % Hp: inertia tensors are the same for each link modeling the cables
% I2 = [ 0.1, 0, 0; 0, 0, 0; 0, 0, 0];
% I3 = [ 0.1, 0, 0; 0, 0, 0; 0, 0, 0];
% 
% % CoM position 
% inertial_disp_1 = [0 0 -0.5]';      % Cable CoM - rotation around x
% inertial_disp_2 = [0 0 -0.25]';
% inertial_disp_3 = [0 0 -0.25]';

% Chain definition
n_links = 3;
n_ee = 2;
n_joints = 3;

% Dynamic parameters
g = [0 0 -g0]';
F_ee_A = zeros(6,1);
F_ee_B = zeros(6,1);

% Kinematic parameters
omega1 = [1 0 0]';              % (first passive x joint)
omega2 = [1 0 0]';              % (left arm)
omega3 = [1 0 0]';              % (right arm)

q_1 = [0 0 0]';                  % (first passive z joint)
q_2 = [0 off -L]';               % (left arm)
q_3 = [0 -off -L]';             % (right arm)

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
        0  1  0  off;
        0  0  1  -L;
        0  0  0  1];
M_b3 = [1  0  0  0;
        0  1  0  -off;
        0  0  1  -L;
        0  0  0  1];
M_b4 = [1  0  0  0;
        0  1  0  off;
        0  0  1  -L-L1;
        0  0  0  1];
M_b5 = [1  0  0  0;
        0  1  0  -off;
        0  0  1  -L-L1;
        0  0  0  1];
    
% Store datas in data-structures
M_bi = sym(zeros(4,4,5));
M_bi(:,:,1) = M_b1;
M_bi(:,:,2) = M_b2;
M_bi(:,:,3) = M_b3;
M_bi(:,:,4) = M_b4;
M_bi(:,:,5) = M_b5;
Inertia(:,:,1) = I1;
Inertia(:,:,2) = I2;
Inertia(:,:,3) = I3;
mass = [m1 m m];
inertial_disp(1,:) = inertial_disp_1;
inertial_disp(2,:) = inertial_disp_2;
inertial_disp(3,:) = inertial_disp_3;
friction = [0 0 0];

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
msg = append('n vector computation');
n = recursive_invdyn_tree_sym_f(TH, dTH, zeros(1,n_joints), g, info, msg) + Fv*dTH';
n = simplify(n)

%% CON GIUNTO PASSIVO SUPERIORE ED INFERIORE (SENZA ASSUNZIONI SUL SECONDO GIUNTO) E CON BRACCIA CHE FANNO LO STESSO MOVIMENTO

close all
clear
clc

if ispc % Windows
    addpath('..\..\..\functions\screw_theory_symbolic_functions\')
else % Linux
    addpath('../../../functions/screw_theory_symbolic_functions')
end

% Variables
syms q1 q2 q...
     qd1 qd2 qd...
     qdd1 qdd2 qdd real
 
TH = [q1 q2 q q];
dTH = [qd1 qd2 qd qd];
ddTH = [qdd1 qdd2 qdd qdd];

syms L L1 L2 m1 m2 m1L m1R off ...
    i1xx i1yy i1zz i2xx i2yy i2zz i1Lxx i1Lyy i1Lzz i1Rxx i1Ryy i1Rzz ...
    id_1x id_1y id_1z id_2x id_2y id_2z id_1Lx id_1Ly id_1Lz id_1Rx id_1Ry id_1Rz ...
    fv1 fv2 fv1L fv1R g0 real;

L = 1;
off = 0.15;
L1 = 0.5;
m1 = 1;
m2 = 1;
m = 0.5;

% Inertia tensors
I1 = [ 0.1, 0, 0; 0, 0, 0; 0, 0, 0];   % Hp: inertia tensors are the same for each link modeling the cables
I2 = [ 0.1, 0, 0; 0, 0, 0; 0, 0, 0];
I3 = [ 0.1, 0, 0; 0, 0, 0; 0, 0, 0];
I4 = [ 0.1, 0, 0; 0, 0, 0; 0, 0, 0];

% CoM position 
inertial_disp_1 = [0 0 -0.5]';      % Cable CoM - rotation around x
inertial_disp_2 = [0 0 0]';
inertial_disp_3 = [0 0 -0.25]';
inertial_disp_4 = [0 0 -0.25]';

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
q_3 = [0 off -L]';               % (left arm)
q_4 = [0 -off -L]';             % (right arm)

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
        0  0  1  -L;
        0  0  0  1];
M_b4 = [1  0  0  0;
        0  1  0  -off;
        0  0  1  -L;
        0  0  0  1];
M_b5 = [1  0  0  0;
        0  1  0  off;
        0  0  1  -L-L1;
        0  0  0  1];
M_b6 = [1  0  0  0;
        0  1  0  -off;
        0  0  1  -L-L1;
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
     g0 real;

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
q_3 = [0 off -L]';               % (left arm)
q_4 = [0 -off -L]';             % (right arm)

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
        0  0  1  -L;
        0  0  0  1];
M_b4 = [1  0  0  0;
        0  1  0  -off;
        0  0  1  -L;
        0  0  0  1];
M_b5 = [1  0  0  0;
        0  1  0  off;
        0  0  1  -L-L1;
        0  0  0  1];
M_b6 = [1  0  0  0;
        0  1  0  -off;
        0  0  1  -L-L1;
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

%% LA VARIABILE Q2 VIENE POSTA =-Q1 (SHOULDER HORIZONTALITY) E LE SHOULDER NON SONO ALLINEATE AI BRACCI (COME CRANEBOT) E CON BRACCIA CHE FANNO LO STESSO MOVIMENTO

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

syms L D L1 m1 m2 m off ...
     i1xx i2xx ixx ...
     l1z l2z lz ...
     g0 real;

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
inertial_disp_2 = [0 0 -l2z]';
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

%% IL MODELLO VIENE FATTO PARTIRE DIRETTAMENTE DAL SECONDO GIUNTO PASSIVO E LE BRACCIA CHE FANNO LO STESSO MOVIMENTO

close all
clear
clc

if ispc % Windows
    addpath('..\..\..\functions\screw_theory_symbolic_functions\')
else % Linux
    addpath('../../../functions/screw_theory_symbolic_functions')
end

% % Variables
% syms q1 q2 q3 q ...
%      qd1 qd2 qd3 qd ...
%      qdd1 qdd2 qdd3 qdd real
 
% Variables
syms qC qS qA ...
     qCd qSd qAd real

TH = [-qC qA qA];
dTH = [-qCd qAd qAd];

syms LA mS mA off ...
     iSxx iAxx ...
     lSz lAz ...
     g0 fvS fvA fvA real;

% L = 1;
% off = 0.15;
% L1 = 0.5;
% m1 = 1;
% m2 = 1;
% m = 0.5;

% Inertia tensors
I1 = [ iSxx, 0, 0; 0, 0, 0; 0, 0, 0];   % Hp: inertia tensors are the same for each link modeling the cables
I2 = [ iAxx, 0, 0; 0, 0, 0; 0, 0, 0];
I3 = [ iAxx, 0, 0; 0, 0, 0; 0, 0, 0];

% CoM position 
inertial_disp_1 = [0 0 -lSz]';
inertial_disp_2 = [0 0 -lAz]';
inertial_disp_3 = [0 0 -lAz]';

% Chain definition
n_links = 3;
n_ee = 2;
n_joints = 3;

% Dynamic parameters
% g = [0 0 -g0]';
g = [1 0 0; 0 cos(qC) -sin(qC); 0 sin(qC) cos(qC)]*[0 0 -g0]';

F_ee_A = zeros(6,1);
F_ee_B = zeros(6,1);

% Kinematic parameters
omega1 = [1 0 0]';              % (second passive x joint)
omega2 = [1 0 0]';              % (left arm)
omega3 = [1 0 0]';              % (right arm)

q_1 = [0 0 0]';                 % (second passive z joint)
q_2 = [0 off 0]';               % (left arm)
q_3 = [0 -off 0]';              % (right arm)

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
        0  1  0  off;
        0  0  1  0;
        0  0  0  1];
M_b3 = [1  0  0  0;
        0  1  0  -off;
        0  0  1  0;
        0  0  0  1];
M_b4 = [1  0  0  0;
        0  1  0  off;
        0  0  1  -LA;
        0  0  0  1];
M_b5 = [1  0  0  0;
        0  1  0  -off;
        0  0  1  -LA;
        0  0  0  1];
    
% Store datas in data-structures
M_bi = sym(zeros(4,4,5));
M_bi(:,:,1) = M_b1;
M_bi(:,:,2) = M_b2;
M_bi(:,:,3) = M_b3;
M_bi(:,:,4) = M_b4;
M_bi(:,:,5) = M_b5;
Inertia(:,:,1) = I1;
Inertia(:,:,2) = I2;
Inertia(:,:,3) = I3;
mass = [mS mA mA];
inertial_disp(1,:) = inertial_disp_1;
inertial_disp(2,:) = inertial_disp_2;
inertial_disp(3,:) = inertial_disp_3;
friction = [fvS fvA fvA];

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
msg = append('n vector computation');
n = recursive_invdyn_tree_sym_f(TH, dTH, zeros(1,n_joints), g, info, msg) + Fv*dTH';
n = simplify(n)

%% IL MODELLO VIENE FATTO PARTIRE DIRETTAMENTE DAL SECONDO GIUNTO PASSIVO E LE BRACCIA CHE FANNO LO STESSO MOVIMENTO

close all
clear
clc

if ispc % Windows
    addpath('..\..\..\functions\screw_theory_symbolic_functions\')
else % Linux
    addpath('../../../functions/screw_theory_symbolic_functions')
end

% Variables
syms q1 q2 q...
     qd1 qd2 qd...
     qdd1 qdd2 qdd real
 
TH = [-q1 -q2 q q];
dTH = [-qd1 -qd2 qd qd];
ddTH = [-qdd1 -qdd2 qdd qdd];

syms L L1 m2 m off ...
     i2xx i2yy ixx ...
     l1z l2z lz ...
     g0 fv1 fv2 fv3 fv4 real;

% L = 1;
% off = 0.15;
% L1 = 0.5;
% m1 = 1;
% m2 = 1;
% m = 0.5;

% Inertia tensors
I1 = [ i2xx, 0, 0; 0, 0, 0; 0, 0, 0];
I2 = [ 0, 0, 0; 0, i2yy, 0; 0, 0, 0];
I3 = [ ixx, 0, 0; 0, 0, 0; 0, 0, 0];
I4 = [ ixx, 0, 0; 0, 0, 0; 0, 0, 0];

% CoM position 
syms lx real
inertial_disp_1 = [0 0 -l2z]';
inertial_disp_2 = [0 0 -l2z]';
inertial_disp_3 = [lx 0 -lz]';
inertial_disp_4 = [lx 0 -lz]';

% Chain definition
n_links = 4;
n_ee = 2;
n_joints = 4;

% Dynamic parameters
g = [0 0 -g0]';
F_ee_A = zeros(6,1);
F_ee_B = zeros(6,1);

% Kinematic parameters
omega1 = [1 0 0]';              % (second passive x joint)
omega2 = [0 1 0]';              % (second passive y joint)
omega3 = [1 0 0]';              % (left arm)
omega4 = [1 0 0]';              % (right arm)

q_1 = [0 0 0]';                  % (second passive x joint)
q_2 = [0 0 0]';                  % (second passive y joint)
q_3 = [0 off 0]';               % (left arm)
q_4 = [0 -off 0]';             % (right arm)

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
        0  0  1  0;
        0  0  0  1];
M_b3 = [1  0  0  0;
        0  1  0  off;
        0  0  1  0;
        0  0  0  1];
M_b4 = [1  0  0  0;
        0  1  0  -off;
        0  0  1  0;
        0  0  0  1];
M_b5 = [1  0  0  0;
        0  1  0  off;
        0  0  1  -L1;
        0  0  0  1];
M_b6 = [1  0  0  0;
        0  1  0  -off;
        0  0  1  -L1;
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
mass = [m2 m2 m m];
inertial_disp(1,:) = inertial_disp_1;
inertial_disp(2,:) = inertial_disp_2;
inertial_disp(3,:) = inertial_disp_3;
inertial_disp(4,:) = inertial_disp_4;
friction = [fv1 fv2 fv3 fv4];

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

%% IL MODELLO VIENE FATTO PARTIRE DIRETTAMENTE DAL SECONDO GIUNTO PASSIVO E LE BRACCIA CHE FANNO LO STESSO MOVIMENTO

close all
clear
clc

addpath('../../functions/screw_theory_symbolic_functions')

% Variables
syms q1 q2 q3 q4 q ...
     qd1 qd2 qd3 qd4 qd ...
     qdd1 qdd2 qdd3 qdd4 qdd real
 
TH = [-q1 -q2 q3 q4 q3 q4];
dTH = [-qd1 -qd2 qd3 qd4 qd3 qd4];
ddTH = [-qdd1 -qdd2 qdd3 qdd4 qdd3 qdd4];

syms L L1 L2 m2 m off ...
     i2xx i2yy ixx ...
     l1z l2z lz ...
     g0 fv1 fv2 fv3 fv4 fv5 fv6 real;

% L = 1;
% off = 0.15;
% L1 = 0.5;
% m1 = 1;
% m2 = 1;
% m = 0.5;

% Inertia tensors
syms i3xx i3yy i3zz i4xx i4yy i4zz real
I1 = [ i2xx, 0, 0; 0, 0, 0; 0, 0, 0];
I2 = [ 0, 0, 0; 0, i2yy, 0; 0, 0, 0];
I3 = [ 0, 0, 0; 0, i3yy, 0; 0, 0, 0];
I4 = [ i4xx, 0, 0; 0, 0, 0; 0, 0, 0];
I5 = [ 0, 0, 0; 0, i3yy, 0; 0, 0, 0];
I6 = [ i4xx, 0, 0; 0, 0, 0; 0, 0, 0];

% CoM position 
syms lx ly real
inertial_disp_1 = [0 0 -l2z]';  % (second passive x joint)
inertial_disp_2 = [0 0 -l2z]';  % (second passive y joint)
inertial_disp_3 = [0 0 0]';    % (left arm)
inertial_disp_4 = [0 0 -lz]';   % (left arm)
% inertial_disp_4 = [lx 0 -lz]';   % (left arm)
inertial_disp_5 = [0 0 0]';    % (right arm)
inertial_disp_6 = [0 0 -lz]';   % (right arm)
% inertial_disp_6 = [lx 0 -lz]';   % (right arm)

% Chain definition
n_links = 6;
n_ee = 2;
n_joints = 6;

% Dynamic parameters
g = [0 0 -g0]';
F_ee_A = zeros(6,1);
F_ee_B = zeros(6,1);

% Kinematic parameters
omega1 = [1 0 0]';              % (second passive x joint)
omega2 = [0 1 0]';              % (second passive y joint)
omega3 = [0 1 0]';              % (left arm)
omega4 = [1 0 0]';              % (left arm)
omega5 = [0 1 0]';              % (right arm)
omega6 = [1 0 0]';              % (right arm)

q_1 = [0 0 0]';                 % (second passive x joint)
q_2 = [0 0 0]';                 % (second passive y joint)
q_3 = [0 off 0]';               % (left arm)
q_4 = [0 off+L1 0]';            % (left arm)
q_5 = [0 -off 0]';              % (right arm)
q_6 = [0 -off-L1 0]';           % (right arm)


% Screw axis computation
omega = [omega1 omega2 omega3 omega4 omega5 omega6];
q_ = [q_1 q_2 q_3 q_4 q_5 q_6];
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
        0  1  0  off;
        0  0  1  0;
        0  0  0  1];
M_b4 = [1  0  0  0;
        0  1  0  off+L1;
        0  0  1  0;
        0  0  0  1];
M_b5 = [1  0  0  0;
        0  1  0  -off;
        0  0  1  0;
        0  0  0  1];
M_b6 = [1  0  0  0;
        0  1  0  -off-L1;
        0  0  1  0;
        0  0  0  1];
M_b7 = [1  0  0  0;
        0  1  0  off+L1;
        0  0  1  -L2;
        0  0  0  1];
M_b8 = [1  0  0  0;
        0  1  0  -off-L1;
        0  0  1  -L2;
        0  0  0  1];
    
% Store datas in data-structures
syms m3 m4 real
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
mass = [m2 m2 m3 m4 m3 m4];
inertial_disp(1,:) = inertial_disp_1;
inertial_disp(2,:) = inertial_disp_2;
inertial_disp(3,:) = inertial_disp_3;
inertial_disp(4,:) = inertial_disp_4;
inertial_disp(5,:) = inertial_disp_5;
inertial_disp(6,:) = inertial_disp_6;
friction = [fv1 fv2 fv3 fv4 fv5 fv6];

% Store data regarding the frames in a struct
info = struct('n_joints', n_joints, ...
              'n_links', n_links, ...
              'n_ee', n_ee, ...
              'current_frame_index',  [  1  2 3 4 5 6 7 8], ...
              'previous_frame_index', [  0  1 2 3 2 5 4 6], ...
              'next_frame_index',     [  2 35 4 7 6 8 0 0], ...
              'next_frame_type',      [  0  1 0 0 0 0 0 0], ...         % 0 for single frame / 1 for duplex next frame (frameA(1 digit) - frameB(1 digit)) / 2 for duplex next frame (frameA(1 digit) - frameB(2 digit))
              'previous_joint_index', [  1  2 3 4 5 6 0 0], ...
              'frame_type',           [  1  1 1 1 1 1 0 0], ...         % 1 for link, 0 for ee
              'explored',             [  0  0 0 0 0 0 0 0], ...         % 0 for unxeplored, 1 for explored 
              'S', S, ...
              'M_bi', M_bi, ...
              'Inertia', Inertia, ...
              'mass', mass, ...
              'inertial_disp', inertial_disp, ...
              'F', [zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) F_ee_A F_ee_B]);              
          
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
Fv = diag([friction(1), friction(2), friction(3), friction(4), friction(5), friction(6)]);
msg = append('n vector computation');
n = recursive_invdyn_tree_sym_f(TH, dTH, zeros(1,n_joints), g, info, msg) + Fv*dTH';
n = simplify(n)
