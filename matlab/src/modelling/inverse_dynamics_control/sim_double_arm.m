close all
clear
clc

run('../../../parameters/licas_parameters.m')
addpath('../../../include/screw_theory_functions/')
addpath('../../../include/control_functions/')

% Chain definition
n_joints = 4;

% Dynamic parameters
g = [0 0 -g0]';
F_eeA = zeros(6,1);
F_eeB = zeros(6,1);

% Kinematic parameters
omega1A = [0 1 0]';              % (left arm)
omega2A = [1 0 0]';              % (left arm)
omega3A = [0 0 1]';              % (left arm)
omega4A = [0 1 0]';              % (left arm)
omega1B = [0 1 0]';              % (right arm)
omega2B = [1 0 0]';              % (right arm)
omega3B = [0 0 1]';              % (right arm)
omega4B = [0 1 0]';              % (right arm)

q1A = [0 off 0]';              % (left arm)
q2A = [0 off+L1 0]';           % (left arm)
q3A = [0 off+L1 -L2]';         % (left arm)
q4A = [0 off+L1 -L2-L3]';      % (left arm)
q1B = [0 -off 0]';             % (right arm)
q2B = [0 -off-L1 0]';          % (right arm)
q3B = [0 -off-L1 -L2]';        % (right arm)
q4B = [0 -off-L1 -L2-L3]';     % (right arm)

% Screw axis computation
omegaA = [omega1A omega2A omega3A omega4A];
omegaB = [omega1B omega2B omega3B omega4B];
qA = [q1A q2A q3A q4A];
qB = [q1B q2B q3B q4B];
SA = zeros(6,n_joints);
SB = zeros(6,n_joints);
for i = 1:n_joints
    vA = cross(-omegaA(:,i),qA(:,i));
    SA(:,i) = [omegaA(:,i); vA];
end
for i = 1:n_joints
    vB = cross(-omegaB(:,i),qB(:,i));
    SB(:,i) = [omegaB(:,i); vB];
end

% Definition M_{b,i}
M_b1A = [1  0  0  0;
         0  1  0  off;
         0  0  1  0;
         0  0  0  1];
M_b2A = [1  0  0  0;
         0  1  0  off+L1;
         0  0  1  0;
         0  0  0  1];
M_b3A = [1  0  0  0;
         0  1  0  off+L1;
         0  0  1  -L2;
         0  0  0  1];
M_b4A = [1  0  0  0;
         0  1  0  off+L1;
         0  0  1  -L2-L3;
         0  0  0  1];
M_b1B = [1  0  0  0;
         0  1  0  -off;
         0  0  1  0;
         0  0  0  1];
M_b2B = [1  0  0  0;
         0  1  0  -off-L1;
         0  0  1  0;
         0  0  0  1];
M_b3B = [1  0  0  0;
         0  1  0  -off-L1;
         0  0  1  -L2;
         0  0  0  1];
M_b4B = [1  0  0  0;
         0  1  0  -off-L1;
         0  0  1  -L2-L3;
         0  0  0  1];
M_beA = [1  0  0  0;
         0  1  0  off+L1;
         0  0  1  -L2-L3-L4;
         0  0  0  1];
M_beB = [1  0  0  0;
         0  1  0  -off-L1;
         0  0  1  -L2-L3-L4;
         0  0  0  1];
    
% Store datas in data-structures
M_biA(:,:,1) = M_b1A;
M_biA(:,:,2) = M_b2A;
M_biA(:,:,3) = M_b3A;
M_biA(:,:,4) = M_b4A;
M_biA(:,:,5) = M_beA;
M_biB(:,:,1) = M_b1B;
M_biB(:,:,2) = M_b2B;
M_biB(:,:,3) = M_b3B;
M_biB(:,:,4) = M_b4B;
M_biB(:,:,5) = M_beB;

InertiaA(:,:,1) = Il1_left;
InertiaA(:,:,2) = Il2_left;
InertiaA(:,:,3) = Il3_left;
InertiaA(:,:,4) = Il4_left;
InertiaB(:,:,1) = Il1_right;
InertiaB(:,:,2) = Il2_right;
InertiaB(:,:,3) = Il3_right;
InertiaB(:,:,4) = Il4_right;
massA = [ml1 ml2 ml3 ml4];
massB = [ml1 ml2 ml3 ml4];
inertial_dispA(1,:) = inertial_disp_1_left;
inertial_dispA(2,:) = inertial_disp_2_left;
inertial_dispA(3,:) = inertial_disp_3_left;
inertial_dispA(4,:) = inertial_disp_4_left;
inertial_dispB(1,:) = inertial_disp_1_right;
inertial_dispB(2,:) = inertial_disp_2_right;
inertial_dispB(3,:) = inertial_disp_3_right;
inertial_dispB(4,:) = inertial_disp_4_right;
frictionA = [fv1 fv2 fv3 fv4];
frictionB = [fv1 fv2 fv3 fv4];

% Store data regarding the frames in a struct
infoA = struct('n_joints', n_joints, ...
               'S', SA, ...
               'M_bi', M_biA, ...
               'Inertia', InertiaA, ...
               'mass', massA, ...ddd
               'inertial_disp', inertial_dispA); 
infoB = struct('n_joints', n_joints, ...
               'S', SB, ...
               'M_bi', M_biB, ...
               'Inertia', InertiaB, ...
               'mass', massB, ...
               'inertial_disp', inertial_dispB); 
          
% Control gains
Kp = diag([10 10 10 10]);
Kd = diag([10 10 10 10]);

% Regulation references
q_refA = [pi/2 0 0 pi/6]';
% q_refA = [0 0 0 0]';
% q_refA = [deg2rad(30) deg2rad(50) deg2rad(50) deg2rad(80)]';
q_dot_refA = zeros(n_joints,1);
q_ddot_refA = zeros(n_joints,1);
q_refB = [0 0 0 0]';
% q_refB = [-pi/6 -pi/6 -pi/6 -pi/6]';
% q_refB = [deg2rad(-30) deg2rad(-50) deg2rad(-50) deg2rad(-80)]';
q_dot_refB = zeros(n_joints,1);
q_ddot_refB = zeros(n_joints,1);

% Simulation parameters
Ts = 0.01;                 % Sample Time [s]
Duration = 10;              % Simulation Time [s]

% Start simulation
run('controller_double_arm.m')
