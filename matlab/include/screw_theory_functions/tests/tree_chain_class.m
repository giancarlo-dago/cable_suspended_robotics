close all
clear
clc

addpath('../../screw_theory_functions/')

% Parameters
a1 = 0.5;
a2A = 0.5;
a2B = 0.8;
a3A = 0.5;
a3B = 0.8;
l1 = a1/2;
l2A = a2A/2;
l2B = a2B/2;
l3A = a3A/2;
l3B = a3B/2;

% Dynamic parameters
g0 = 9.81;
g = [0 0 -g0]';
ml1 = 5;
ml2A = 5;
ml3A = 5;
ml2B = 5;
ml3B = 5;
Il1 = diag([1 1 1]);
Il2A = diag([0.008333 0.108333 0.10833333333333333]);
Il3A = diag([0.008333 0.108333 0.10833333333333333]);
Il2B = diag([0.008333 0.108333 0.10833333333333333]);
Il3B = diag([0.008333 0.108333 0.10833333333333333]);
inertial_disp_1 = [0 0 0];        
inertial_disp_2A = [0 0 0];         
inertial_disp_3A = [0 0 0]; 
inertial_disp_2B = [0 0 0];         
inertial_disp_3B = [0 0 0]; 
F_ee_A = zeros(6,1);
F_ee_B = zeros(6,1);

% Kinematic parameters
omega1 = [0 0 1]';
omega2 = [0 0 1]';
omega3A = [0 0 1]';
omega3B = [0 0 1]';
q1 = [0 0 0]';
q2 = [a1 0 0]';
q3A = [a1+a2A 0 0]';
q3B = [a1+a2B 0 0]';

% Computation M_{b,i}
M_b1 = [  eye(3)    [l1 0 0]';
        zeros(1,3)      1   ];
    
M_b2A = [  eye(3)    [a1+l2A 0 0]';
        zeros(1,3)         1     ];

M_b3A = [  eye(3)    [a1+a2A+l3A 0 0]';
        zeros(1,3)           1       ];
    
M_beA = [  eye(3)    [a1+a2A+a3A 0 0]';
        zeros(1,3)           1       ];
    
M_b2B = [  eye(3)    [a1+l2B 0 0]';
        zeros(1,3)         1     ];
    
M_b3B = [  eye(3)    [a1+a2B+l3B 0 0]';
        zeros(1,3)           1       ];
    
M_beB = [  eye(3)    [a1+a2B+a3B 0 0]';
        zeros(1,3)           1       ]; 

% Define frames
frame_1 = frame('link_frame',1,0,[2 4],1,M_b1,[],ml1,Il1,inertial_disp_1);
frame_2A = frame('link_frame',2,1,3,2,M_b2A,[],ml2A,Il2A,inertial_disp_2A);
frame_3A = frame('link_frame',3,2,6,3,M_b3A,[],ml3A,Il3A,inertial_disp_3A);
frame_2B = frame('link_frame',4,1,5,2,M_b2B,[],ml2B,Il2B,inertial_disp_2B);
frame_3B = frame('link_frame',5,4,7,4,M_b3B,[],ml3B,Il3B,inertial_disp_3B);
frame_eA = frame('ee_frame',6,3,[],[],M_beA,F_ee_A,[],[],[]);
frame_eB = frame('ee_frame',7,5,[],[],M_beB,F_ee_B,[],[],[]);

% Define joints and compute screw axis
joint_1 = joint(1,omega1,q1);
joint_2 = joint(2,omega2,q2);
joint_3A = joint(3,omega3A,q3A);
joint_3B = joint(4,omega3B,q3B);

% Chain definition
n_links = 5;
n_ee = 2;
n_joints = 4;
robot = build_robot(n_links,n_ee,n_joints,[frame_1 frame_2A frame_3A frame_2B frame_3B frame_eA frame_eB],[joint_1 joint_2 joint_3A joint_3B],g);

theta = [pi/8 pi/8 pi/4 -pi/8 -pi/4];
dtheta = [0.1 0.3 0.4 0.2 0.1];
ddtheta = [0.05 0.03 0.06 0.02 0.07];
tau = recursive_invdyn_class_f(theta, dtheta, ddtheta, robot)

