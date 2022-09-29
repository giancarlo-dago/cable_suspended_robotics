run('../../../../../parameters/licas_parameters.m')

% Chain definition
n_joints = 4;

% Dynamic parameters
g = [0 0 -g0]';
F_ee_left = zeros(6,1);
F_ee_right = zeros(6,1);

% Kinematic parameters
omega1_left = [0 1 0]';              % (left arm)
omega2_left = [1 0 0]';              % (left arm)
omega3_left = [0 0 1]';              % (left arm)
omega4_left = [0 1 0]';              % (left arm)
omega1_right = [0 1 0]';              % (right arm)
omega2_right = [1 0 0]';              % (right arm)
omega3_right = [0 0 1]';              % (right arm)
omega4_right = [0 1 0]';              % (right arm)

q1_left = [0 off 0]';              % (left arm)
q2_left = [0 off+L1 0]';           % (left arm)
q3_left = [0 off+L1 -L2]';         % (left arm)
q4_left = [0 off+L1 -L2-L3]';      % (left arm)
q1_right = [0 -off 0]';             % (right arm)
q2_right = [0 -off-L1 0]';          % (right arm)
q3_right = [0 -off-L1 -L2]';        % (right arm)
q4_right = [0 -off-L1 -L2-L3]';     % (right arm)

% Screw axis computation
omega_left = [omega1_left omega2_left omega3_left omega4_left];
omega_right = [omega1_right omega2_right omega3_right omega4_right];
q_left = [q1_left q2_left q3_left q4_left];
q_right = [q1_right q2_right q3_right q4_right];
S_left = zeros(6,n_joints);
S_right = zeros(6,n_joints);
for i = 1:n_joints
    v_left = cross(-omega_left(:,i),q_left(:,i));
    S_left(:,i) = [omega_left(:,i); v_left];
end
for i = 1:n_joints
    v_right = cross(-omega_right(:,i),q_right(:,i));
    S_right(:,i) = [omega_right(:,i); v_right];
end

% Definition M_{b,i}
M_b1_left = [1  0  0  0;
             0  1  0  off;
             0  0  1  0;
             0  0  0  1];
M_b2_left = [1  0  0  0;
             0  1  0  off+L1;
             0  0  1  0;
             0  0  0  1];
M_b3_left = [1  0  0  0;
             0  1  0  off+L1;
             0  0  1  -L2;
             0  0  0  1];
M_b4_left = [1  0  0  0;
             0  1  0  off+L1;
             0  0  1  -L2-L3;
             0  0  0  1];
M_b1_right = [1  0  0  0;
              0  1  0  -off;
              0  0  1  0;
              0  0  0  1];
M_b2_right = [1  0  0  0;
              0  1  0  -off-L1;
              0  0  1  0;
              0  0  0  1];
M_b3_right = [1  0  0  0;
              0  1  0  -off-L1;
              0  0  1  -L2;
              0  0  0  1];
M_b4_right = [1  0  0  0;
              0  1  0  -off-L1;
              0  0  1  -L2-L3;
              0  0  0  1];
M_be_left = [1  0  0  0;
             0  1  0  off+L1;
             0  0  1  -L2-L3-L4;
             0  0  0  1];
M_be_right = [1  0  0  0;
              0  1  0  -off-L1;
              0  0  1  -L2-L3-L4;
              0  0  0  1];
