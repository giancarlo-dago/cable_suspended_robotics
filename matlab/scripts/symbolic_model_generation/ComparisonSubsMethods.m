close all
clear
clc

%%

disp('~~~~~~~~~~~~~SUBSTITUTION INSIDE TEXT~~~~~~~~~~~~~\n\n')
tic 
i1xx = 0.1;
l1 = 0.1;
m1 = 0.233;
fv1 = 0.1;
q1 = 0.5;
q2 = 0.5;
q3 = 0.5;
qd1 = 0.1;
qd2 = 0.1;
qd3 = 0.1;

B = [m1*l1^2 + i1xx + 0.0093*cos(q2 + q3) + 0.0280*cos(q2) + 0.0093*cos(q3) + 0.2326, 0.0047*cos(q2 + q3) + 0.0140*cos(q2) + 0.0093*cos(q3) + 0.2140, 0.0047*cos(q2 + q3) + 0.0047*cos(q3) + 0.1023;
                      0.0047*cos(q2 + q3) + 0.0140*cos(q2) + 0.0093*cos(q3) + 0.2140,                                        0.0093*cos(q3) + 0.2140,                       0.0047*cos(q3) + 0.1023;
                                       0.0047*cos(q2 + q3) + 0.0047*cos(q3) + 0.1023,                                        0.0047*cos(q3) + 0.1023,                                        0.1023]


n = [0.2283*sin(q1 + q2 + q3) + 0.6850*sin(q1 + q2) + 0.9134*sin(q1) - 0.0140*qd2^2*sin(q2) - 0.0047*qd3^2*sin(q3) + fv1*qd1 - 0.0047*qd2^2*sin(q2 + q3) - 0.0047*qd3^2*sin(q2 + q3) + 9.8000*l1*m1*sin(q1) - 0.0280*qd1*qd2*sin(q2) - 0.0093*qd1*qd3*sin(q3) - 0.0093*qd2*qd3*sin(q3) - 0.0093*qd1*qd2*sin(q2 + q3) - 0.0093*qd1*qd3*sin(q2 + q3) - 0.0093*qd2*qd3*sin(q2 + q3);
                                                                                                                                                                                                     0.2283*sin(q1 + q2 + q3) + 0.6850*sin(q1 + q2) + 0.0140*qd1^2*sin(q2) - 0.0047*qd3^2*sin(q3) + 0.0047*qd1^2*sin(q2 + q3) - 0.0093*qd1*qd3*sin(q3) - 0.0093*qd2*qd3*sin(q3);
                                                                                                                                                                                                                                                    0.2283*sin(q1 + q2 + q3) + 0.0047*qd1^2*sin(q3) + 0.0047*qd2^2*sin(q3) + 0.0047*qd1^2*sin(q2 + q3) + 0.0093*qd1*qd2*sin(q3)]

                    
toc
                                                                                                                                                                                                                                                
%%

disp('\n\n~~~~~~~~~~~~~SUBSTITUTION WITH SUBS FUNCTION~~~~~~~~~~~~~\n\n')

syms i1xx l1 m1 fv1 q1 q2 q3 qd1 qd2 qd3 real
B = [m1*l1^2 + i1xx + 0.0093*cos(q2 + q3) + 0.0280*cos(q2) + 0.0093*cos(q3) + 0.2326, 0.0047*cos(q2 + q3) + 0.0140*cos(q2) + 0.0093*cos(q3) + 0.2140, 0.0047*cos(q2 + q3) + 0.0047*cos(q3) + 0.1023;
                      0.0047*cos(q2 + q3) + 0.0140*cos(q2) + 0.0093*cos(q3) + 0.2140,                                        0.0093*cos(q3) + 0.2140,                       0.0047*cos(q3) + 0.1023;
                                       0.0047*cos(q2 + q3) + 0.0047*cos(q3) + 0.1023,                                        0.0047*cos(q3) + 0.1023,                                        0.1023];


n = [0.2283*sin(q1 + q2 + q3) + 0.6850*sin(q1 + q2) + 0.9134*sin(q1) - 0.0140*qd2^2*sin(q2) - 0.0047*qd3^2*sin(q3) + fv1*qd1 - 0.0047*qd2^2*sin(q2 + q3) - 0.0047*qd3^2*sin(q2 + q3) + 9.8000*l1*m1*sin(q1) - 0.0280*qd1*qd2*sin(q2) - 0.0093*qd1*qd3*sin(q3) - 0.0093*qd2*qd3*sin(q3) - 0.0093*qd1*qd2*sin(q2 + q3) - 0.0093*qd1*qd3*sin(q2 + q3) - 0.0093*qd2*qd3*sin(q2 + q3);
                                                                                                                                                                                                     0.2283*sin(q1 + q2 + q3) + 0.6850*sin(q1 + q2) + 0.0140*qd1^2*sin(q2) - 0.0047*qd3^2*sin(q3) + 0.0047*qd1^2*sin(q2 + q3) - 0.0093*qd1*qd3*sin(q3) - 0.0093*qd2*qd3*sin(q3);
                                                                                                                                                                                                                                                    0.2283*sin(q1 + q2 + q3) + 0.0047*qd1^2*sin(q3) + 0.0047*qd2^2*sin(q3) + 0.0047*qd1^2*sin(q2 + q3) + 0.0093*qd1*qd2*sin(q3)];

tic
Bsubs = subs(B,{i1xx l1 m1 fv1 q1 q2 q3 qd1 qd2 qd3},[0.1 0.1 0.233 0.1 0.5 0.5 0.5 0.1 0.1 0.1])
nsubs = subs(n,{i1xx l1 m1 fv1 q1 q2 q3 qd1 qd2 qd3},[0.1 0.1 0.233 0.1 0.5 0.5 0.5 0.1 0.1 0.1])
toc


%%

disp('\n\n~~~~~~~~~~~~~RECURSIVE COMPUTATION~~~~~~~~~~~~~\n\n')

tic
% Parameters
a1 = 0.2;
a2 = 0.2;
a3 = 0.2;
l2 = 0.1;
l3 = 0.1;
g0 = 9.8;
i2xx = 1e-1;
i2yy = 1e-1;
i2zz = 1e-1;
i3xx = 1e-1;
i3yy = 1e-1;
i3zz = 1e-1;
m2 = 0.233;
m3 = 0.233;
fv2 = 0.0;
fv3 = 0.0;

i1xx = 0.1;
i1yy = 0;
i1zz = 0;
l1 = 0.1;
m1 = 0.233;
fv1 = 0.1;
g = [0 0 -g0]';
F_ee = zeros(6,1);


I1 = [i1xx 0 0; 0 i1yy 0; 0 0 i1zz];
I2 = [i2xx 0 0; 0 i2yy 0; 0 0 i2zz];
I3 = [i3xx 0 0; 0 i3yy 0; 0 0 i3zz];

inertial_disp_1 = [0 0 -l1]';         %% Dipende dalla definizione del tensore di inerzia, qui l'hp è che i tensori di inerzia sono già riferiti rispetto ad una terna baricentrale
inertial_disp_2 = [0 0 -l2]';
inertial_disp_3 = [0 0 -l3]';

% Kinematic parameters
omega1 = [1 0 0]';
omega2 = [1 0 0]';
omega3 = [1 0 0]';
q_1 = [0 0 0]';
q_2 = [0 0 -a1]';
q_3 = [0 0 -a1-a2]';

% Screw axis computation
v1 = cross(-omega1,q_1);
v2 = cross(-omega2,q_2);
v3 = cross(-omega3,q_3);
S1 = [omega1; v1];
S2 = [omega2; v2];
S3 = [omega3; v3];
S = [S1 S2 S3];

% Computation M_{b,i}
M_b1 = [  eye(3)    [0 0 0]';
        zeros(1,3)      1   ];

M_b2 = [  eye(3)    [0 0 -a1]';
        zeros(1,3)       1   ];

M_b3 = [  eye(3)    [0 0 -a1-a2]';
        zeros(1,3)         1    ];
    
M_be = [  eye(3)    [0 0 -a1-a2]';
        zeros(1,3)         1    ];

% Chain definition
n_links = 3;
n_ee = 1;
n_joints = 3;

% Store datas in data-structures
M_bi(:,:,1) = M_b1;
M_bi(:,:,2) = M_b2;
M_bi(:,:,3) = M_b3;
M_bi(:,:,4) = M_be;
Inertia(:,:,1) = I1;
Inertia(:,:,2) = I2;
Inertia(:,:,3) = I3;
mass = [m1 m2 m3];
inertial_disp(1,:) = inertial_disp_1;
inertial_disp(2,:) = inertial_disp_2;
inertial_disp(3,:) = inertial_disp_3;


% Store data regarding the frames in a struct
info = struct('n_joints', n_joints, ...
              'n_links', n_links, ...
              'n_ee', n_ee, ...
              'current_frame_index',[1 2 3 4], ...
              'previous_frame_index',[0 1 2 3],...
              'next_frame_index',[2 3 4 0],...
              'next_frame_type', [0 0 0 0], ...         % 0 for single frame / 1 for duplex next frame (frameA(1 digit) - frameB(1 digit)) / 2 for duplex next frame (frameA(1 digit) - frameB(2 digit))
              'previous_joint_index',[1 2 3 0], ...
              'frame_type', [1 1 1 0], ...                      % 1 for link, 0 for ee
              'explored', [0 0 0 0], ...                        % 0 for unxeplored, 1 for explored  
              'S', S, ...
              'M_bi', M_bi, ...
              'Inertia', Inertia, ...
              'mass', mass, ...
              'inertial_disp', inertial_disp, ...
              'F', [zeros(6,1) zeros(6,1) zeros(6,1) F_ee]);         


q1 = 0.5;
q2 = 0.5;
q3 = 0.5;
qd1 = 0.1;
qd2 = 0.1;
qd3 = 0.1;
qdd1 = 0;
qdd2 = 0;
qdd3 = 0;

             
theta = [0.5 0.5 0.5];
dtheta = [0.1 0.1 0.1];
ddtheta = [0 0 0];
for j=1:n_joints
    fake_acc = zeros(1,n_joints);
    fake_acc(j) = 1;
    B(:,j) = recursive_invdyn_tree_f(theta, zeros(1,n_joints), fake_acc, zeros(3,1), info);
end

Fv = diag([fv1 fv2 fv3]);
n = recursive_invdyn_tree_f(theta, dtheta, zeros(1,n_joints), g, info) + Fv*dtheta';

B
n
toc

%%

disp('\n\n~~~~~~~~~~~~~MATLAB FUNCTION~~~~~~~~~~~~~\n\n')
syms i1xx l1 m1 fv1 q1 q2 q3 qd1 qd2 qd3 real
B = [m1*l1^2 + i1xx + 0.0093*cos(q2 + q3) + 0.0280*cos(q2) + 0.0093*cos(q3) + 0.2326, 0.0047*cos(q2 + q3) + 0.0140*cos(q2) + 0.0093*cos(q3) + 0.2140, 0.0047*cos(q2 + q3) + 0.0047*cos(q3) + 0.1023;
                      0.0047*cos(q2 + q3) + 0.0140*cos(q2) + 0.0093*cos(q3) + 0.2140,                                        0.0093*cos(q3) + 0.2140,                       0.0047*cos(q3) + 0.1023;
                                       0.0047*cos(q2 + q3) + 0.0047*cos(q3) + 0.1023,                                        0.0047*cos(q3) + 0.1023,                                        0.1023];


n = [0.2283*sin(q1 + q2 + q3) + 0.6850*sin(q1 + q2) + 0.9134*sin(q1) - 0.0140*qd2^2*sin(q2) - 0.0047*qd3^2*sin(q3) + fv1*qd1 - 0.0047*qd2^2*sin(q2 + q3) - 0.0047*qd3^2*sin(q2 + q3) + 9.8000*l1*m1*sin(q1) - 0.0280*qd1*qd2*sin(q2) - 0.0093*qd1*qd3*sin(q3) - 0.0093*qd2*qd3*sin(q3) - 0.0093*qd1*qd2*sin(q2 + q3) - 0.0093*qd1*qd3*sin(q2 + q3) - 0.0093*qd2*qd3*sin(q2 + q3);
                                                                                                                                                                                                     0.2283*sin(q1 + q2 + q3) + 0.6850*sin(q1 + q2) + 0.0140*qd1^2*sin(q2) - 0.0047*qd3^2*sin(q3) + 0.0047*qd1^2*sin(q2 + q3) - 0.0093*qd1*qd3*sin(q3) - 0.0093*qd2*qd3*sin(q3);
                                                                                                                                                                                                                                                    0.2283*sin(q1 + q2 + q3) + 0.0047*qd1^2*sin(q3) + 0.0047*qd2^2*sin(q3) + 0.0047*qd1^2*sin(q2 + q3) + 0.0093*qd1*qd2*sin(q3)];

Bfun = matlabFunction(B);
nfun = matlabFunction(n);
% tic
% Bfun = matlabFunction(B,'File','Bfile');
% nfun = matlabFunction(n,'File','nfile');
% toc
% tic
% Bfun = matlabFunction(B,'File','Bfile','Optimize',false);
% nfun = matlabFunction(n,'File','nfile','Optimize',false);
% toc
% tic
% Bfun = matlabFunction(B,'File','Bfile','Sparse',true);
% toc

tic
i1xx = 0.1;
l1 = 0.1;
m1 = 0.233;
fv1 = 0.1;
q1 = 0.5;
q2 = 0.5;
q3 = 0.5;
qd1 = 0.1;
qd2 = 0.1;
qd3 = 0.1;
Bfun(i1xx,l1,m1,q2,q3)
nfun(fv1,l1,m1,q1,q2,q3,qd1,qd2,qd3)
toc

