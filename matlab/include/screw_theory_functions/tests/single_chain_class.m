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

% Define frames
frame1 = frame('link_frame',1,0,2,1,M_b1,[],ml1,Il1,inertial_disp_1);
frame2 = frame('link_frame',2,1,3,2,M_b2,[],ml2,Il2,inertial_disp_2);
frame3 = frame('link_frame',3,2,4,3,M_b3,[],ml3,Il3,inertial_disp_3);
frame4 = frame('ee_frame',4,3,[],[],M_be,F_ee,[],[],[]);

% Define joints and compute screw axis
joint1 = joint(1,omega1,q1);
joint2 = joint(2,omega2,q2);
joint3 = joint(3,omega3,q3);

theta = [-pi/2 pi/4 pi/8];
dtheta = [0.2 0.5 0.1];
ddtheta = [0.05 0.04 0.07];

% Chain definition
n_links = 3;
n_ee = 1;
n_joints = 3;
robot = build_robot(n_links, n_ee, n_joints, [frame1 frame2 frame3 frame4], [joint1 joint2 joint3], g);

tau = recursive_invdyn_class_f(theta, dtheta, ddtheta, robot)

