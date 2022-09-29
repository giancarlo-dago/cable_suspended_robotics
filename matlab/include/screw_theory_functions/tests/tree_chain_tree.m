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
Il1 = diag([0.008333 0.108333 0.10833333333333333]);
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

% Screw axis computation
v1 = cross(-omega1,q1);
v2 = cross(-omega2,q2);
v3A = cross(-omega3A,q3A);
v3B = cross(-omega3B,q3B);
S1 = [omega1; v1];
S2 = [omega2; v2];
S3A = [omega3A; v3A];
S3B = [omega3B; v3B];
S = [S1 S2 S3A S3B];

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

% Chain definition
n_links = 5;
n_ee = 2;
n_joints = 4;

% Store datas in data-structures
M_bi(:,:,1) = M_b1;
M_bi(:,:,2) = M_b2A;
M_bi(:,:,3) = M_b3A;
M_bi(:,:,4) = M_b2B; 
M_bi(:,:,5) = M_b3B;
M_bi(:,:,6) = M_beA;
M_bi(:,:,7) = M_beB;
Inertia(:,:,1) = Il1;
Inertia(:,:,2) = Il2A;
Inertia(:,:,3) = Il3A;
Inertia(:,:,4) = Il2B;
Inertia(:,:,5) = Il3B;
mass = [ml1 ml2A ml3A ml2B ml3B];
inertial_disp(1,:) = inertial_disp_1;
inertial_disp(2,:) = inertial_disp_2A;
inertial_disp(3,:) = inertial_disp_3A;
inertial_disp(4,:) = inertial_disp_2B;
inertial_disp(5,:) = inertial_disp_3B;
F_ee = [F_ee_A F_ee_B];

% Store data regarding the frames in a struct
info = struct('n_joints', n_joints, ...
              'n_links', n_links, ...
              'n_ee', n_ee, ...
              'current_frame_index',[1 2 3 4 5 6 7], ...
              'previous_frame_index',[0 1 2 1 4 3 5],...
              'next_frame_index',[24 3 6 5 7 0 0],...
              'previous_joint_index',[1 2 3 2 4 0 0], ...
              'frame_type', [1 1 1 1 1 0 0], ...                                            % 1 for link, 0 for ee
              'explored', [0 0 0 0 0 0 0], ...                                              % 0 for unxeplored, 1 for explored 
              'S', S, ...
              'M_bi', M_bi, ...
              'Inertia', Inertia, ...
              'mass', mass, ...
              'inertial_disp', inertial_disp, ...
              'F', [zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) F_ee_A F_ee_B]);                                    
          
theta = [pi/8 pi/8 pi/4 -pi/8 -pi/4];
dtheta = [0.1 0.3 0.4 0.2 0.1];
ddtheta = [0.05 0.03 0.06 0.02 0.07];
tau = recursive_invdyn_tree_f(theta, dtheta, ddtheta, g, info)

