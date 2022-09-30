close all
clear
clc

run('../../../parameters/parameters.m')

% Chain definition
n_links = 14;
n_ee = 2;
n_joints = 14;

% Dynamic parameters
g = [0 0 -g0]';
F_ee_A = zeros(6,1);
F_ee_B = zeros(6,1);

% Initial conditions
alfa_0 = deg2rad(0);
beta_0 = deg2rad(0);
th1A_0 = deg2rad(22.5);
th2A_0 = deg2rad(0);
th3A_0 = deg2rad(-22.5);
th4A_0 = deg2rad(22.5);
th5A_0 = deg2rad(0);
th6A_0 = deg2rad(0);
th1B_0 = deg2rad(-22.5);
th2B_0 = deg2rad(0);
th3B_0 = deg2rad(22.5);
th4B_0 = deg2rad(-22.5);
th5B_0 = deg2rad(0);
th6B_0 = deg2rad(0);
q_0 = [alfa_0 beta_0 th1A_0 th2A_0 th3A_0 th4A_0 th5A_0 th6A_0 th1B_0 th2B_0 th3B_0 th4B_0 th5B_0 th6B_0];

% Kinematic parameters
omega1 = [1 0 0]';
omega2 = [1 0 0]';
omega3 = [0 0 -1]';
omega4 = [1 0 0]';
omega5 = [-1 0 0]';
omega6 = [0 0 -1]';
omega7 = [-1 0 0]';
omega8 = [0 0 -1]';
omega9 = [0 0 -1]';
omega10 = [-1 0 0]';
omega11 = [1 0 0]';
omega12 = [0 0 -1]';
omega13 = [1 0 0]';
omega14 = [0 0 -1]';

q1 = [0 0 0]';
q2 = [0 0 -L]';
q3 = [0 offA -L-D]';
q4 = [0 offA -L-D]';
q5 = [0 offA -L-D-L1]';
q6 = [0 offA -L-D-L1]';
q7 = [0 offA -L-D-L1-L2]';
q8 = [0 offA -L-D-L1-L2]';
q9 = [0 offB -L-D]';
q10 = [0 offB -L-D]';
q11 = [0 offB -L-D-L1]';
q12 = [0 offB -L-D-L1]';
q13 = [0 offB -L-D-L1-L2]';
q14 = [0 offB -L-D-L1-L2]';


% Computation M_{b,i}
M_b1 = [1  0  0  0;
        0  1  0  0;
        0  0  1  0;
        0  0  0  1];

M_b2 = [1  0  0  0;
        0  1  0  0;
        0  0  1 -L;
        0  0  0  1];

M_b3 = [0 -1  0  0;
       -1  0  0 offA;
        0  0 -1 -L-D;
        0  0  0  1];
    
M_b4 = [0  0  1  0;
       -1  0  0 offA;
        0 -1  0 -L-D;
        0  0  0  1];
    
M_b5 = [0  0 -1   0;
        0 -1  0  offA;
       -1  0  0 -L-D-L1;
        0  0  0   1];
    
M_b6 = [0 -1  0   0;
       -1  0  0  offA;
        0  0 -1 -L-D-L1;
        0  0  0   1];
    
M_b7 = [0  0 -1    0;
       -1  0  0   offA;
        0  1  0 -L-D-L1-L2;
        0  0  0    1];    

M_b8 = [0 -1  0    0;
       -1  0  0   offA;
        0  0 -1 -L-D-L1-L2;
        0  0  0    1]; 

M_b9 = [0  1  0  0;
        1  0  0 offB;
        0  0 -1 -L-D;
        0  0  0  1];
    
M_b10 = [0  0 -1  0;
         1  0  0 offB;
         0 -1  0 -L-D;
         0  0  0  1];
    
M_b11 = [0  0  1   0;
         0  1  0  offB;
        -1  0  0 -L-D-L1;
         0  0  0   1];
    
M_b12 = [0  1  0   0;
         1  0  0  offB;
         0  0 -1 -L-D-L1;
         0  0  0   1];
    
M_b13 = [0  0  1    0;
         1  0  0   offB;
         0  1  0 -L-D-L1-L2;
         0  0  0    1];    

M_b14 = [0  1  0    0;
         1  0  0   offB;
         0  0 -1 -L-D-L1-L2;
         0  0  0    1]; 
    
M_b15 = [0 -1  0    0;
        -1  0  0   offA;
         0  0 -1 -L-D-L1-L2-L3;
         0  0  0    1]; 
     
M_b16 = [0  1  0    0;
         1  0  0   offB;
         0  0 -1 -L-D-L1-L2-L3;
         0  0  0    1]; 
     
     
% Define frames
frame_1  = frame('link_frame', 1, 0,    2, 1,M_b1 ,[],m_cables ,I_cables ,inertial_disp_cables );
frame_2  = frame('link_frame', 2, 1,[3 9], 2,M_b2 ,[],m_platform ,I_platform ,inertial_disp_platform );
frame_3  = frame('link_frame', 3, 2,    4, 3,M_b3 ,[],ml1 ,Il1 ,inertial_disp_1 );
frame_4  = frame('link_frame', 4, 3,    5, 4,M_b4 ,[],ml2 ,Il2 ,inertial_disp_2 );
frame_5  = frame('link_frame', 5, 4,    6, 5,M_b5 ,[],ml3 ,Il3 ,inertial_disp_3 );
frame_6  = frame('link_frame', 6, 5,    7, 6,M_b6 ,[],ml4 ,Il4 ,inertial_disp_4 );
frame_7  = frame('link_frame', 7, 6,    8, 7,M_b7 ,[],ml5 ,Il5 ,inertial_disp_5 );
frame_8  = frame('link_frame', 8, 7,   15, 8,M_b8 ,[],ml6 ,Il6 ,inertial_disp_6 );
frame_9  = frame('link_frame', 9, 2,   10, 9,M_b9 ,[],ml1 ,Il1 ,inertial_disp_1 );
frame_10 = frame('link_frame',10, 9,   11,10,M_b10,[],ml2,Il2,inertial_disp_2);
frame_11 = frame('link_frame',11,10,   12,11,M_b11,[],ml3,Il3,inertial_disp_3);
frame_12 = frame('link_frame',12,11,   13,12,M_b12,[],ml4,Il4,inertial_disp_4);
frame_13 = frame('link_frame',13,12,   14,13,M_b13,[],ml5,Il5,inertial_disp_5);
frame_14 = frame('link_frame',14,13,   16,14,M_b14,[],ml6,Il6,inertial_disp_6);
frame_15 = frame('ee_frame', 15, 8,[],[],M_b15,F_ee_A,[],[],[]);
frame_16 = frame('ee_frame', 16,14,[],[],M_b16,F_ee_B,[],[],[]);

% Define joints and compute screw axis
joint_1 = joint(1,omega1,q1);
joint_2 = joint(2,omega2,q2);
joint_3 = joint(3,omega3,q3);
joint_4 = joint(4,omega4,q4);
joint_5 = joint(5,omega5,q5);
joint_6 = joint(6,omega6,q6);
joint_7 = joint(7,omega7,q7);
joint_8 = joint(8,omega8,q8);
joint_9 = joint(9,omega9,q9);
joint_10 = joint(10,omega10,q10);
joint_11 = joint(11,omega11,q11);
joint_12 = joint(12,omega12,q12);
joint_13 = joint(13,omega13,q13);
joint_14 = joint(14,omega14,q14);


% Chain definition
robot = build_robot(n_links,n_ee,n_joints,[frame_1 frame_2 frame_3 frame_4 frame_5 frame_6 frame_7 frame_8 frame_9 frame_10 ...
                                           frame_11 frame_12 frame_13 frame_14 frame_15 frame_16],...
                                          [joint_1 joint_2 joint_3 joint_4 joint_5 joint_6 joint_7 joint_8 joint_9 joint_10 ...
                                           joint_11 joint_12 joint_13 joint_14 ],g);

qd_0 = [0 0 0.1 0 0.2 0.3 0 0 0 0.5 0 0 0.1 0];
qdd_0 = [0 0 0 0 0.2 0 0 0 0.3 0 0 0 0.5 0];
tau = recursive_invdyn_class_f(q_0, qd_0, qdd_0, robot)








