% Gravity acceleration
g0 = 9.81;                  % [m\s^2]

% Kinematic parameters
L = 1;                      % [m] (Distance between the first passive joint and the second passive joint)
D = 0;                      % [m] (Vertical distance between the second passive joint and the arms)
L1 = 0.2;                   % [m] (Distance between first joint and end-effector)
off = 0.14;                 % [m] (Half of the distance between the arms)

% Mass values
m_cables = 1;               % [kg] (Cables)
m_shoulders = 0.639;        % [kg] (shoulders)
ml1 = 0.233;                % [kg] (Link 1)

% Inertia tensors from the CAD with a CoM frame coincident with the i-th frame (from the CAD)
I_cables = [1e-1,     0,     0;
               0,  1e-1,     0;
               0,     0, 1e-1];
           
I_shoulders = [1e-1,    0,     0;
                  0, 1e-1,     0;
                  0,    0, 1e-1];
   
Il1_left = [1e-1,     0,    0;
               0,  1e-1,    0;
               0,     0, 1e-1];
             
Il1_right = [1e-1,     0,    0;
                0,  1e-1,    0;
                0,     0, 1e-1];
         
% Position of the CoM with respect to the i-th frame (from the CAD)
inertial_disp_cables = [0 0 -0.5]';
inertial_disp_shoulders = [0 0 0]';     % [m] (Shoulders)
inertial_disp_1_left = [0 0 -0.1]';      % [m] (Link1 left)   
inertial_disp_1_right = [0 0 -0.1]';     % [m] (Link1 left)

% Values of the frictions
fv1p = 0;
fv2p = 0;
fv1 = 0;

% % Joints position limits
% J1_lower_limit = -90;      % [deg]
% J1_upper_limit = 90;       % [deg]

