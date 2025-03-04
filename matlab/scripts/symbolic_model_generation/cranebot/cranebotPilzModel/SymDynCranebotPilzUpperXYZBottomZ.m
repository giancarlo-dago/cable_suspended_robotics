%% HP: WE HAVE THREE UPPER PASSIVE JOINTS (X,Y,Z) AND ONE BOTTOM PASSIVE JOINT (Z)

close all
clear
clc

if ispc % Windows
    addpath('..\..\..\..\functions\screw_theory_symbolic_functions\')
    run('..\..\..\..\parameters\cranebotPilzParameters.m')
else % Linux
    addpath('../../../../functions/screw_theory_symbolic_functions')
    run('../../../../parameters/cranebotPilzParameters.m')
end

% Unknown parameters
clear L massCablesPulleys comCablesPulleys fricCablesJz fricCablesJx fricCablesJy fricPlatformJz inertiaCablesPulleys
syms L massCablesPulleys lCablesPulleys fricCablesJz fricCablesJx fricCablesJy fricPlatformJz ixxCablesPulleys iyyCablesPulleys izzCablesPulleys real
inertiaCablesPulleys = [ixxCablesPulleys 0 0; 0 iyyCablesPulleys 0; 0 0 izzCablesPulleys];
comCablesPulleys = [0 0 -lCablesPulleys];

% Chain definition
n_links = 16;
n_ee = 2;
n_joints = 16;

% Joint variables
TH = str2sym(compose("q" + (1:n_joints)));
dTH = str2sym(compose("qd" + (1:n_joints)));
ddTH = str2sym(compose("qdd" + (1:n_joints)));

assume(TH, 'real');
assume(dTH, 'real');
assume(ddTH, 'real');

% Rename parameters
m1 = massCablesPulleys;
m2 = massCablesPulleys+massPlatform;
m3 = massCablesPulleys+massPlatform;
m4 = massPlatform;
m5 = massArmsLink1;
m6 = massArmsLink2;
m7 = massArmsLink3;
m8 = massArmsLink4;
m9 = massArmsLink5;
m10 = massArmsFlange;
m11 = massArmsLink1;
m12 = massArmsLink2;
m13 = massArmsLink3;
m14 = massArmsLink4;
m15 = massArmsLink5;
m16 = massArmsFlange;

l1x = (massCablesPulleys*comCablesPulleys(1)+massPlatform*comPlatform(1))/(massCablesPulleys+massPlatform);
l1y = (massCablesPulleys*comCablesPulleys(2)+massPlatform*comPlatform(2))/(massCablesPulleys+massPlatform);
l1z = (massCablesPulleys*comCablesPulleys(3)+massPlatform*comPlatform(3))/(massCablesPulleys+massPlatform);
l1 = comCablesPulleys;
l2 = [l1x l1y l1z]';
l3 = [l1x l1y l1z]';
l4 = comPlatform;
l5 = comArmsLink1;
l6 = comArmsLink2;
l7 = comArmsLink3;
l8 = comArmsLink4;
l9 = comArmsLink5;
l10 = comArmsFlange;
l11 = comArmsLink1;
l12 = comArmsLink2;
l13 = comArmsLink3;
l14 = comArmsLink4;
l15 = comArmsLink5;
l16 = comArmsFlange;

R1 = comCablesPulleys-l1;
R2 = comPlatform-l1;
I1 = inertiaCablesPulleys;
I2 = (inertiaCablesPulleys+massCablesPulleys*(dot(R1,R1)*eye(3)-R1*R1')) + (inertiaPlatform+massPlatform*(dot(R2,R2)*eye(3)-R2*R2'));
I3 = (inertiaCablesPulleys+massCablesPulleys*(dot(R1,R1)*eye(3)-R1*R1')) + (inertiaPlatform+massPlatform*(dot(R2,R2)*eye(3)-R2*R2'));
I4 = inertiaPlatform;
I5 = InertiaArmsLink1;
I6 = InertiaArmsLink2;
I7 = InertiaArmsLink3;
I8 = InertiaArmsLink4;
I9 = InertiaArmsLink5;
I10 = InertiaArmsFlange;
I11 = InertiaArmsLink1;
I12 = InertiaArmsLink2;
I13 = InertiaArmsLink3;
I14 = InertiaArmsLink4;
I15 = InertiaArmsLink5;
I16 = InertiaArmsFlange;

d1 = dampCablesJz;
d2 = dampCablesJx;
d3 = dampCablesJy;
d4 = dampPlatformJz;
d5 = dampArmsJ1;
d6 = dampArmsJ2;
d7 = dampArmsJ3;
d8 = dampArmsJ4;
d9 = dampArmsJ5;
d10 = dampArmsJ6;
d11 = dampArmsJ1;
d12 = dampArmsJ2;
d13 = dampArmsJ3;
d14 = dampArmsJ4;
d15 = dampArmsJ5;
d16 = dampArmsJ6;

f1 = fricCablesJz;
f2 = fricCablesJx;
f3 = fricCablesJy;
f4 = fricPlatformJz;
f5 = fricArmsJ1;
f6 = fricArmsJ2;
f7 = fricArmsJ3;
f8 = fricArmsJ4;
f9 = fricArmsJ5;
f10 = fricArmsJ6;
f11 = fricArmsJ1;
f12 = fricArmsJ2;
f13 = fricArmsJ3;
f14 = fricArmsJ4;
f15 = fricArmsJ5;
f16 = fricArmsJ6;

% Dynamic parameters
g = [0 0 -g0]';
F_ee_A = zeros(6,1);
F_ee_B = zeros(6,1);

% Kinematic parameters
omega1 = [0 0 1]';
omega2 = [1 0 0]';
omega3 = [0 1 0]';
omega4 = [0 0 1]';
omega5 = [0 0 -1]';
omega6 = [0 -1 0]';
omega7 = [0 1 0]';
omega8 = [0 0 -1]';
omega9 = [0 1 0]';
omega10 = [0 0 -1]';
omega11 = [0 0 -1]';
omega12 = [0 1 0]';
omega13 = [0 -1 0]';
omega14 = [0 0 -1]';
omega15 = [0 -1 0]';
omega16 = [0 0 -1]';

q_1 = [0 0 0]';
q_2 = [0 0 0]';
q_3 = [0 0 0]';
q_4 = [0 0 -L]';
q_5 = [0 -off -L-D]';
q_6 = [0 -off -L-D]';
q_7 = [0 -off -L-D-L1]';
q_8 = [0 -off -L-D-L1]';
q_9 = [0 -off -L-D-L1-L2]';
q_10 = [0 -off -L-D-L1-L2]';
q_11 = [0 off -L-D]';
q_12 = [0 off -L-D]';
q_13 = [0 off -L-D-L1]';
q_14 = [0 off -L-D-L1]';
q_15 = [0 off -L-D-L1-L2]';
q_16 = [0 off -L-D-L1-L2]';

% Screw axis computation
omega = [omega1 omega2 omega3 omega4 omega5 omega6 omega7 omega8 omega9 omega10 omega11 omega12 omega13 omega14 omega15 omega16];
q_ = [q_1 q_2 q_3 q_4 q_5 q_6 q_7 q_8 q_9 q_10 q_11 q_12 q_13 q_14 q_15 q_16];
S = sym(zeros(6,n_joints));
for i = 1:n_joints
    v = cross(-omega(:,i),q_(:,i));
    S(:,i) = [omega(:,i); v];
end

% Computation M_{b,i}
frame_position = {q_(:,1), q_(:,2), q_(:,3), q_(:,4), ...                                             // passive
                  q_(:,5), q_(:,6), q_(:,7), q_(:,8), q_(:,9), q_(:,10)...         // arm A
                  q_(:,11), q_(:,12), q_(:,13), q_(:,14), q_(:,15), q_(:,16), ...  // arm B
                  q_(:,10), ...                                                    // Ee arm A
                  q_(:,16)}; ...                                                   // Ee arm B

R_bi = {[1 0 0; 0 1 0; 0 0 1], ...      // Cables - platform
        [1 0 0; 0 1 0; 0 0 1], ...
        [1 0 0; 0 1 0; 0 0 1], ...
        [1 0 0; 0 1 0; 0 0 1], ...
        [-1 0 0; 0 1 0; 0 0 -1] ...     // Arm A
        [-1 0 0; 0 0 -1; 0 -1 0] ...
        [1 0 0; 0 0 1; 0 -1 0] ...
        [-1 0 0; 0 1 0; 0 0 -1] ...
        [1 0 0; 0 0 1; 0 -1 0] ...
        [1 0 0; 0 -1 0; 0 0 -1] ...
        [1 0 0; 0 -1 0; 0 0 -1] ...     // Arm B
        [1 0 0; 0 0 1; 0 -1 0] ...
        [-1 0 0; 0 0 -1; 0 -1 0] ...
        [1 0 0; 0 -1 0; 0 0 -1] ...
        [-1 0 0; 0 0 -1; 0 -1 0] ...
        [-1 0 0; 0 1 0; 0 0 -1] ...
        [1 0 0; 0 -1 0; 0 0 -1] ...     // End-effector arm A
        [-1 0 0; 0 1 0; 0 0 -1]}; ...   // End-effector arm B

M_bi = sym(zeros(4,4,n_joints+n_ee));
for i=1:length(R_bi)
    M_bi(1:3,1:3,i) = R_bi{i};
    M_bi(1:3,4,i) = frame_position{i};
    M_bi(4,1:3,i) = zeros(1,3);
    M_bi(4,4,i) = 1;
end

% Store datas in data-structures
mass = [m1 m2 m3 m4 m5 m6 m7 m8 m9 m10 m11 m12 m13 m14 m15 m16];

Inertia = sym(zeros(3,3,n_joints));
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
Inertia(:,:,15) = I15;
Inertia(:,:,16) = I16;

inertial_disp(1,:) = l1;
inertial_disp(2,:) = l2;
inertial_disp(3,:) = l3;
inertial_disp(4,:) = l4;
inertial_disp(5,:) = l5;
inertial_disp(6,:) = l6;
inertial_disp(7,:) = l7;
inertial_disp(8,:) = l8;
inertial_disp(9,:) = l9;
inertial_disp(10,:) = l10;
inertial_disp(11,:) = l11;
inertial_disp(12,:) = l12;
inertial_disp(13,:) = l13;
inertial_disp(14,:) = l14;
inertial_disp(15,:) = l15;
inertial_disp(16,:) = l16;

static_friction = [f1 f2 f3 f4 f5 f6 f7 f8 f9 f10 f11 f12 f13 f14 f15 f16];
viscous_friction = [d1 d2 d3 d4 d5 d6 d7 d8 d9 d10 d11 d12 d13 d14 d15 d16];

% Store data regarding the frames in a struct
info = struct('n_joints', n_joints, ...
              'n_links', n_links, ...
              'n_ee', n_ee, ...
              'current_frame_index',  [ 1  2 3   4 5 6 7 8  9 10 11 12 13 14 15 16 17 18], ...
              'previous_frame_index', [ 0  1 2   3 4 5 6 7  8  9  4 11 12 13 14 15 10 16], ...
              'next_frame_index',     [ 2  3 4 511 6 7 8 9 10 17 12 13 14 15 16 18  0  0], ...
              'next_frame_type',      [ 0  0 0   2 0 0 0 0  0  0  0  0  0  0  0  0  0  0], ...         % 0 for single frame / 1 for duplex next frame (frameA(1 digit) - frameB(1 digit)) / 2 for duplex next frame (frameA(1 digit) - frameB(2 digit))
              'previous_joint_index', [ 1  2 3   4 5 6 7 8  9 10 11 12 13 14 15 16  0  0], ...
              'frame_type',           [ 1  1 1   1 1 1 1 1  1  1  1  1  1  1  1  1  0  0], ...         % 1 for link, 0 for ee
              'explored',             [ 0  0 0   0 0 0 0 0  0  0  0  0  0  0  0  0  0  0], ...         % 0 for unxeplored, 1 for explored
              'S', S, ...
              'M_bi', M_bi, ...
              'Inertia', Inertia, ...
              'mass', mass, ...
              'inertial_disp', inertial_disp, ...
              'F', [zeros(6,n_joints) F_ee_A F_ee_B]);

% Computation B matrix
disp('Computation B matrix\n')
B = sym(zeros(n_joints,n_joints));
for j=1:n_joints
    fake_acc = zeros(1,n_joints);
    fake_acc(j) = 1;
    msg = append('B matrix computation - Column ',int2str(j),'/',int2str(n_joints));
    B(:,j) = recursive_invdyn_tree_sym_f(TH, zeros(1,n_joints), fake_acc, zeros(3,1), info, msg);
end

% % Computation n vector
% disp('Computation n vector\n')
% Fv = diag(viscous_friction(1:n_joints));
% Fs = diag(static_friction(1:n_joints));
% msg = append('n vector computation');
% n = recursive_invdyn_tree_sym_f(TH, dTH, zeros(1,n_joints), g, info, msg) + Fv*dTH' + Fs*sign(TH');
