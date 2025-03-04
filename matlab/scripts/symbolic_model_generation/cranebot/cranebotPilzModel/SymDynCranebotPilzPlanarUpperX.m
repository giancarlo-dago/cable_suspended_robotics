%% HP: WE DON'T HAVE A BOTTOM PASSIVE JOINT BUT ONLY ONE UPPER PASSIVE JOINTS (X)

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

% Known parameters

% Unknown parameters
clear L massCablesPulleys comCablesPulleys fricCablesJx dampCablesJx inertiaCablesPulleys
syms L massCablesPulleys lCablesPulleys fricCablesJx dampCablesJx ixxCablesPulleys iyyCablesPulleys izzCablesPulleys real
inertiaCablesPulleys = [ixxCablesPulleys 0 0; 0 iyyCablesPulleys 0; 0 0 izzCablesPulleys];
comCablesPulleys = [0 0 -lCablesPulleys];

% Chain definition
n_links = 13;
n_ee = 2;
n_joints = 13;

% Joint variables
TH = str2sym(compose("q" + (1:n_joints)));
dTH = str2sym(compose("qd" + (1:n_joints)));
ddTH = str2sym(compose("qdd" + (1:n_joints)));

assume(TH, 'real');
assume(dTH, 'real');
assume(ddTH, 'real');

% Rename parameters
m1 = massCablesPulleys+massPlatform;
m2 = massArmsLink1;
m3 = massArmsLink2;
m4 = massArmsLink3;
m5 = massArmsLink4;
m6 = massArmsLink5;
m7 = massArmsFlange;
m8 = massArmsLink1;
m9 = massArmsLink2;
m10 = massArmsLink3;
m11 = massArmsLink4;
m12 = massArmsLink5;
m13 = massArmsFlange;

l1x = (massCablesPulleys*comCablesPulleys(1)+massPlatform*comPlatform(1))/(massCablesPulleys+massPlatform);
l1y = (massCablesPulleys*comCablesPulleys(2)+massPlatform*comPlatform(2))/(massCablesPulleys+massPlatform);
l1z = (massCablesPulleys*comCablesPulleys(3)+massPlatform*(-L+comPlatform(3)))/(massCablesPulleys+massPlatform);
l1 = [l1x l1y l1z]';
l2 = comArmsLink1;
l3 = comArmsLink2;
l4 = comArmsLink3;
l5 = comArmsLink4;
l6 = comArmsLink5;
l7 = comArmsFlange;
l8 = comArmsLink1;
l9 = comArmsLink2;
l10 = comArmsLink3;
l11 = comArmsLink4;
l12 = comArmsLink5;
l13 = comArmsFlange;

R1 = comCablesPulleys - l1;
R2 = [0 0 -L]' + comPlatform - l1;
I1 = (inertiaCablesPulleys+massCablesPulleys*(dot(R1,R1)*eye(3)-R1*R1')) + (inertiaPlatform+massPlatform*(dot(R2,R2)*eye(3)-R2*R2'));
I2 = InertiaArmsLink1;
I3 = InertiaArmsLink2;
I4 = InertiaArmsLink3;
I5 = InertiaArmsLink4;
I6 = InertiaArmsLink5;
I7 = InertiaArmsFlange;
I8 = InertiaArmsLink1;
I9 = InertiaArmsLink2;
I10 = InertiaArmsLink3;
I11 = InertiaArmsLink4;
I12 = InertiaArmsLink5;
I13 = InertiaArmsFlange;

d1 = dampCablesJx;
d2 = dampArmsJ1;
d3 = dampArmsJ2;
d4 = dampArmsJ3;
d5 = dampArmsJ4;
d6 = dampArmsJ5;
d7 = dampArmsJ6;
d8 = dampArmsJ1;
d9 = dampArmsJ2;
d10 = dampArmsJ3;
d11 = dampArmsJ4;
d12 = dampArmsJ5;
d13 = dampArmsJ6;

f1 = fricCablesJx;
f2 = fricArmsJ1;
f3 = fricArmsJ2;
f4 = fricArmsJ3;
f5 = fricArmsJ4;
f6 = fricArmsJ5;
f7 = fricArmsJ6;
f8 = fricArmsJ1;
f9 = fricArmsJ2;
f10 = fricArmsJ3;
f11 = fricArmsJ4;
f12 = fricArmsJ5;
f13 = fricArmsJ6;

% Dynamic parameters
g = [0 0 -g0]';
F_ee_A = zeros(6,1);
F_ee_B = zeros(6,1);

% Kinematic parameters
% omega1 = [1 0 0]';
% omega2 = [0 0 -1]';
% omega3 = [0 -1 0]';
% omega4 = [0 1 0]';
% omega5 = [0 0 -1]';
% omega6 = [0 1 0]';
% omega7 = [0 0 -1]';
% omega8 = [0 0 -1]';
% omega9 = [0 1 0]';
% omega10 = [0 -1 0]';
% omega11 = [0 0 -1]';
% omega12 = [0 -1 0]';
% omega13 = [0 0 -1]';

omega1 = [1 0 0]';
omega2 = [0 0 -1]';
omega3 = [1 0 0]';
omega4 = [-1 0 0]';
omega5 = [0 0 -1]';
omega6 = [-1 0 0]';
omega7 = [0 0 -1]';
omega8 = [0 0 -1]';
omega9 = [-1 0 0]';
omega10 = [1 0 0]';
omega11 = [0 0 -1]';
omega12 = [1 0 0]';
omega13 = [0 0 -1]';

q_1 = [0 0 0]';
q_2 = [0 -off -L-D]';
q_3 = [0 -off -L-D]';
q_4 = [0 -off -L-D-L1]';
q_5 = [0 -off -L-D-L1]';
q_6 = [0 -off -L-D-L1-L2]';
q_7 = [0 -off -L-D-L1-L2]';
q_8 = [0 off -L-D]';
q_9 = [0 off -L-D]';
q_10 = [0 off -L-D-L1]';
q_11 = [0 off -L-D-L1]';
q_12 = [0 off -L-D-L1-L2]';
q_13 = [0 off -L-D-L1-L2]';
q_14 = [0 -off -L-D-L1-L2-L3]';
q_15 = [0 off -L-D-L1-L2-L3]';
 
% Screw axis computation
omega = [omega1 omega2 omega3 omega4 omega5 omega6 omega7 omega8 omega9 omega10 omega11 omega12 omega13];
q_ = [q_1 q_2 q_3 q_4 q_5 q_6 q_7 q_8 q_9 q_10 q_11 q_12 q_13];
S = sym(zeros(6,n_joints));
for i = 1:n_joints
    v = cross(-omega(:,i),q_(:,i));
    S(:,i) = [omega(:,i); v];
end

% Computation M_{b,i}
frame_position = {q_(:,1), ...                                                      // passive
                  q_(:,2), q_(:,3), q_(:,4), q_(:,5), q_(:,6), q_(:,7), ...         // arm A
                  q_(:,8), q_(:,9), q_(:,10), q_(:,11), q_(:,12), q_(:,13), ...     // arm B
                  q_14, ...                                                      // Ee arm A
                  q_15}; ...                                                    // Ee arm B

% R_bi = {[1 0 0; 0 1 0; 0 0 1], ...      // Cables - platform
%         [-1 0 0; 0 1 0; 0 0 -1] ...     // Arm A
%         [-1 0 0; 0 0 -1; 0 -1 0] ...
%         [1 0 0; 0 0 1; 0 -1 0] ...
%         [-1 0 0; 0 1 0; 0 0 -1] ...
%         [1 0 0; 0 0 1; 0 -1 0] ...
%         [1 0 0; 0 -1 0; 0 0 -1] ...
%         [1 0 0; 0 -1 0; 0 0 -1] ...     // Arm B
%         [1 0 0; 0 0 1; 0 -1 0] ...
%         [-1 0 0; 0 0 -1; 0 -1 0] ...
%         [1 0 0; 0 -1 0; 0 0 -1] ...
%         [-1 0 0; 0 0 -1; 0 -1 0] ...
%         [-1 0 0; 0 1 0; 0 0 -1] ...
%         [1 0 0; 0 -1 0; 0 0 -1] ...     // End-effector arm A
%         [-1 0 0; 0 1 0; 0 0 -1]}; ...   // End-effector arm B

% Computation M_{b,i}
M_b1 = [1  0  0  0;
        0  1  0  0;
        0  0  1  0;
        0  0  0  1];

M_b2 = [0 -1  0  0;
       -1  0  0 -off;
        0  0 -1 -L-D;
        0  0  0  1];
    
M_b3 = [0  0  1  0;
       -1  0  0 -off;
        0 -1  0 -L-D;
        0  0  0  1];
    
M_b4 = [0  0 -1   0;
        0 -1  0  -off;
       -1  0  0 -L-D-L1;
        0  0  0   1];
    
M_b5 = [0 -1  0   0;
       -1  0  0  -off;
        0  0 -1 -L-D-L1;
        0  0  0   1];
    
M_b6 = [0  0 -1    0;
       -1  0  0   -off;
        0  1  0 -L-D-L1-L2;
        0  0  0    1];

M_b7 = [0 -1  0    0;
       -1  0  0   -off;
        0  0 -1 -L-D-L1-L2;
        0  0  0    1];

M_b8 = [0  1  0  0;
        1  0  0 off;
        0  0 -1 -L-D;
        0  0  0  1];
    
M_b9 = [0  0 -1  0;
         1  0  0 off;
         0 -1  0 -L-D;
         0  0  0  1];
    
M_b10 = [0  0  1   0;
         0  1  0  off;
        -1  0  0 -L-D-L1;
         0  0  0   1];
    
M_b11 = [0  1  0   0;
         1  0  0  off;
         0  0 -1 -L-D-L1;
         0  0  0   1];
    
M_b12 = [0  0  1    0;
         1  0  0   off;
         0  1  0 -L-D-L1-L2;
         0  0  0    1];

M_b13 = [0  1  0    0;
         1  0  0   off;
         0  0 -1 -L-D-L1-L2;
         0  0  0    1];
    
M_b14 = [0 -1  0    0;
        -1  0  0   -off;
         0  0 -1 -L-D-L1-L2-L3;
         0  0  0    1];
     
M_b15 = [0  1  0    0;
         1  0  0   off;
         0  0 -1 -L-D-L1-L2-L3;
         0  0  0    1];

M_bi = sym(zeros(4,4,15));
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

% M_bi = sym(zeros(4,4,n_joints+n_ee));
% for i=1:length(R_bi)
%     M_bi(1:3,1:3,i) = R_bi{i};
%     M_bi(1:3,4,i) = frame_position{i};
%     M_bi(4,1:3,i) = zeros(1,3);
%     M_bi(4,4,i) = 1;
% end

% Store datas in data-structures
mass = [m1 m2 m3 m4 m5 m6 m7 m8 m9 m10 m11 m12 m13];

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

static_friction = [f1 f2 f3 f4 f5 f6 f7 f8 f9 f10 f11 f12 f13];
viscous_friction = [d1 d2 d3 d4 d5 d6 d7 d8 d9 d10 d11 d12 d13];

% Store data regarding the frames in a struct
info = struct('n_joints', n_joints, ...
              'n_links', n_links, ...
              'n_ee', n_ee, ...
              'current_frame_index',  [  1 2 3 4 5 6  7 8  9 10 11 12 13 14 15], ...
              'previous_frame_index', [  0 1 2 3 4 5  6 1  8  9 10 11 12  7 13], ...
              'next_frame_index',     [ 28 3 4 5 6 7 14 9 10 11 12 13 15  0  0], ...
              'next_frame_type',      [  1 0 0 0 0 0  0 0  0  0  0  0  0  0  0], ...         % 0 for single frame / 1 for duplex next frame (frameA(1 digit) - frameB(1 digit)) / 2 for duplex next frame (frameA(1 digit) - frameB(2 digit))
              'previous_joint_index', [  1 2 3 4 5 6  7 8  9 10 11 12 13  0  0], ...
              'frame_type',           [  1 1 1 1 1 1  1 1  1  1  1  1  1  0  0], ...         % 1 for link, 0 for ee
              'explored',             [  0 0 0 0 0 0  0 0  0  0  0  0  0  0  0], ...         % 0 for unxeplored, 1 for explored 
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

% Computation n vector
disp('Computation n vector\n')
Fv = diag([viscous_friction(1:n_joints)]);
Fs = diag([static_friction(1:n_joints)]);
msg = append('n vector computation');
n = recursive_invdyn_tree_sym_f(TH, dTH, zeros(1,n_joints), g, info, msg) + Fv*dTH' + Fs*sign(dTH');
