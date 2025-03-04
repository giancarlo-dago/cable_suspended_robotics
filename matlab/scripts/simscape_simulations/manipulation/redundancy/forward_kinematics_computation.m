
close all
clear
clc

if ispc % Windows
    addpath('..\..\..\..\screw_theory_functions')
else % Linux
    addpath('../../../../screw_theory_functions')
end

n_joints = 8;

L = 3.16 + 0.126;           % [m] (Distance between the first passive joint and the second passive joint)
D = 1.047;                  % [m] (Distance between the second passive joint and the base of the schunks)
L1 = 0.350;                 % [m] (Distance between the center of the two ERB145)
L2 = 0.305;                 % [m] (Distance between the center of the second ERB145 and the ERB115)
L3 = 0.2662;                % [m] (Distance between the center of the ERB115 and the end-effector (center of the tip of the finger of the gripper))
offA = -0.19;               % [m] (Half of the distance between the arms)
offB = 0.19;                % [m] (Half of the distance between the arms)
a1A = L1;
a2A = L2;
a3A = L3;
a1B = L1;
a2B = L2;
a3B = L3;
l1A = a1A/2;
l2A = a2A/2;
l3A = a3A/2;
l1B = a1B/2;
l2B = a2B/2;
l3B = a3B/2;

% Input
theta = zeros(1,8);
    
% Initializations
SA = zeros(6,6);
SB = zeros(6,6);

% Kinematic parameters
omega1 = [0 0 1]';
omega2 = [0 0 1]';
omega1A = [0 0 1]';
omega2A = [0 0 1]';
omega3A = [0 0 1]';
omega1B = [0 0 1]';
omega2B = [0 0 1]';
omega3B = [0 0 1]';
q1 = [0 0 0]';
q2 = [0 -L 0]';
q1A = [offA -L-D 0]';
q2A = [offA -L-D-a1A 0]';
q3A = [offA -L-D-a1A-a2A 0]';
q1B = [offB -L-D 0]';
q2B = [offB -L-D-a1B 0]';
q3B = [offB -L-D-a1B-a2B 0]';



% Computation M_{b,i}
M1 = [ 0  1  0  0; 
      -1  0  0  0;
       0  0  1  0
       0  0  0  1];

M2 = [ 1  0  0  0; 
       0  1  0 -L;
       0  0  1  0
       0  0  0  1];

M1A = [ 0  1  0   offA; 
       -1  0  0   -L-D;
        0  0  1    0
        0  0  0    1];

M2A = [ 0  1  0   offA; 
       -1  0  0 -L-D-a1A;
        0  0  1    0
        0  0  0    1];
   
M3A = [ 0  1  0   offA; 
       -1  0  0 -L-D-a1A-a2A;
        0  0  1    0
        0  0  0    1];

M1B = [ 0  1  0   offB; 
        -1  0  0 -L-D-l1B;
         0  0  1    0
         0  0  0    1];

M2B = [ 0  1  0   offB; 
        -1  0  0 -L-D-a1B-l2B;
         0  0  1    0
         0  0  0    1];
   
M3B = [ 0  1  0   offB; 
        -1  0  0 -L-D-a1B-a2B-l3B;
         0  0  1    0
         0  0  0    1];

MeA = [ 0  1  0   offA; 
        -1  0  0 -L-D-a1A-a2A-a3A;
         0  0  1    0
         0  0  0    1];

MeB = [ 0  1  0   offB; 
         -1  0  0 -L-D-a1B-a2B-a3B;
          0  0  1    0
          0  0  0    1];


% Screw axis computation
omegaA = [omega1, omega2, omega1A, omega2A, omega3A];
qA = [q1, q2, q1A, q2A, q3A];
for i = 1:length(omegaA)
    vA = cross(-omegaA(:,i),qA(:,i));
    SA(:,i) = [omegaA(:,i); vA];
end
omegaB = [omega1, omega2, omega1B, omega2B, omega3B];
qB = [q1, q2, q1B, q2B, q3B];
for i = 1:length(omegaB)
    vB = cross(-omegaB(:,i),qB(:,i));
    SB(:,i) = [omegaB(:,i); vB];
end

    
% Kinematics (position of the joints and of the end effector)
TeA = fkin_f(theta(1:5), 5, MeA, SA(:,1:5));
TeB = fkin_f([theta(1:2) theta(6:8)], 5, MeB, SB(:,1:5));

peA = TeA(1:3,4);
peB = TeB(1:3,4);
