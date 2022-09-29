% Gravity acceleration
g0 = 9.81;                  % [m\s^2]

% Kinematic parameters
L = 1.05;                   % [m] (Distance between the first passive joint and the second passive joint)
D = 0;                      % [m] (Vertical distance between the second passive joint and the arms)
L1 = 0.04;                  % [m] (Distance between first and second joint)
L2 = 0.143;                 % [m] (Distance between second and third joint)
L3 = 0.132;                 % [m] (Distance between third and fourth joint)
L4 = 0.277;                 % [m] (Distance between fourth joint and end-effector)
off = 0.14;                 % [m] (Half of the distance between the arms)

% Mass values
m_cables = 1;               % [kg] (Cables)
m_shoulders = 0.639;        % [kg] (shoulders)
ml1 = 0.233;                % [kg] (Link 1)
ml2 = 0.246;                % [kg] (Link 2)
ml3 = 0.214;                % [kg] (Link 3)
ml4 = 0.106;                % [kg] (Link 4)

% Inertia tensors from the CAD with a CoM frame coincident with the i-th frame (from the CAD)
I_cables = [1e-4,     0,     0;
               0,  1e-4,     0;
               0,     0, 1e-3];
           
I_shoulders = [3.02e-3,       0,        0;
                     0, 8.64e-4,        0;
                     0,       0, 3.39e-3];
   
Il1_left = [3.68e-4, -7.57e-6, 5.38e-6;
           -7.57e-6,  1.16e-4, -4.8e-5;
            5.38e-6,  -4.8e-5, 3.76e-4];

Il2_left = [3.49e-4,       0,  -3.79e-5;
             0,      4.46e-4,         0;
           -3.79e-5,       0,   1.49e-4];
  
Il3_left = [4.21e-4,  -7.81e-7,  -4.06e-5;
           -7.81e-7,   4.26e-4,    1.8e-5;
           -4.06e-5,    1.8e-5,   5.01e-5];
  
Il4_left = [3.94e-4,         0,         0;
                  0,   3.75e-4,   -3.5e-6;
                  0,   -3.5e-6,   3.05e-5];
             
Il1_right = [3.68e-4, -7.57e-6, 5.38e-6;
            -7.57e-6,  1.16e-4, -4.8e-5;
             5.38e-6,  -4.8e-5, 3.76e-4];

Il2_right = [3.49e-4,       0,  -3.79e-5;
                   0, 4.46e-4,         0;
            -3.79e-5,       0,   1.49e-4];
  
Il3_right = [4.21e-4,  -7.81e-7,  -4.06e-5;
            -7.81e-7,   4.26e-4,    1.8e-5;
            -4.06e-5,    1.8e-5,   5.01e-5];
  
Il4_right = [3.94e-4,         0,         0;
                   0,   3.75e-4,   -3.5e-6;
                   0,   -3.5e-6,   3.05e-5];
         
% Position of the CoM with respect to the i-th frame (from the CAD)
inertial_disp_cables = [0 0 -0.5]';
inertial_disp_shoulders = [0 0 0]';     % [m] (Shoulders)
inertial_disp_1_left = [0 0.0236 -0.00946];    % [m] (Link1 left)   
inertial_disp_2_left = [-0.015 0 -0.1]';       % [m] (Link2 left)
inertial_disp_3_left = [0 0 -0.093]';          % [m] (Link3 left)
inertial_disp_4_left = [0 0 -0.092]';          % [m] (Link4 left)    
inertial_disp_1_right = [0 -0.0236 -0.00946];    % [m] (Link1 left)   
inertial_disp_2_right = [-0.015 0 -0.1]';       % [m] (Link2 left)
inertial_disp_3_right = [0 0 -0.093]';          % [m] (Link3 left)
inertial_disp_4_right = [0 0 -0.092]';          % [m] (Link4 left)   


% Values of the frictions
fv1p = 0;
fv2p = 0;
fv1 = 0;
fv2 = 0;
fv3 = 0;
fv4 = 0;

% % Joints position limits
% J1_lower_limit = -90;      % [deg]
% J1_upper_limit = 90;       % [deg]
% J2_lower_limit = -90;      % [deg]
% J2_upper_limit = 20;       % [deg]
% J3_lower_limit = -90;    % [deg]
% J3_upper_limit = 90;     % [deg]
% J4_lower_limit = -150;      % [deg]
% J4_upper_limit = 150;       % [deg]

% Drone parameters
PeakVelocityDrone = 1.0;                    % [m/s]
MaxAccelerationDrone = 0.1;                 % [m/s^2]


