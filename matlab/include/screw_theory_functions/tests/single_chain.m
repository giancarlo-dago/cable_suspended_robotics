close all
clear
clc

addpath('../../screw_theory_functions/')

% Parameters
a1 = 0.5;
a2 = 0.5;
a3 = 0.5;
l1 = a1/2;
l2 = a2/2;
l3 = a3/2;

% Dynamic parameters
g0 = 9.81;
g = [0 -g0 0]';
ml1 = 5;
ml2 = 5;
ml3 = 5;
Il1 = diag([0.008333 0.108333 0.10833333333333333]);
Il2 = diag([0.008333 0.108333 0.10833333333333333]);
Il3 = diag([0.008333 0.108333 0.10833333333333333]);
inertial_disp_1 = [0 0 0];
inertial_disp_2 = [0 0 0];
inertial_disp_3 = [0 0 0];
F_ee = zeros(6,1);

% Kinematic parameters
omega1 = [0 0 1]';
omega2 = [0 0 1]';
omega3 = [0 0 1]';
q1 = [0 0 0]';
q2 = [a1 0 0]';
q3 = [a1+a2 0 0]';

% Computation M_{b,i}
M_b1 = [  eye(3)    [l1 0 0]';
        zeros(1,3)      1   ];
    
M_b2 = [  eye(3)    [a1+l2 0 0]';
        zeros(1,3)      1      ];

M_b3 = [  eye(3)    [a1+a2+l3 0 0]';
        zeros(1,3)       1        ];
    
M_be = [  eye(3)    [a1+a2+a3 0 0]';
        zeros(1,3)        1      ];

theta = [-pi/2 pi/4 pi/8];
dtheta = [0.2 0.5 0.1];
ddtheta = [0.05 0.04 0.07];

n_joints = 3;

M_bi(:,:,1) = M_b1;
M_bi(:,:,2) = M_b2;
M_bi(:,:,3) = M_b3;
M_bi(:,:,4) = M_be;
v1 = cross(-omega1,q1);
v2 = cross(-omega2,q2);
v3 = cross(-omega3,q3);
S1 = [omega1; v1];
S2 = [omega2; v2];
S3 = [omega3; v3];
S = [S1 S2 S3];
Inertia(:,:,1) = Il1;
Inertia(:,:,2) = Il2;
Inertia(:,:,3) = Il3;
mass = [ml1 ml2 ml3];
inertial_disp(1,:) = inertial_disp_1;
inertial_disp(2,:) = inertial_disp_2;
inertial_disp(3,:) = inertial_disp_3;

% Store data regarding the frames in a struct
info = struct('n_joints', n_joints, ...
              'S', S, ...
              'M_bi', M_bi, ...
              'Inertia', Inertia, ...
              'mass', mass, ...
              'inertial_disp', inertial_disp);  

tau = recursive_invdyn_f(theta, dtheta, ddtheta, g, info, F_ee)

