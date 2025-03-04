% Gravity acceleration
g0 = 9.81;

% Kinematic parameters
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

% Mass values
m_crane = 10000;            % [kg] (Crane)
m_cables = 100;             % [kg] (Cables + pulley)
m_platform = 156.686;       % [kg] (Hook + platform + schunk bases)
ml1 = 4.1513;               % [kg] (Powerball ERB145)
ml2 = 1.677;                % [kg] (Link 2)
ml3 = 4.1513;               % [kg] (Powerball ERB145)
ml4 = 1.0277;               % [kg] (Link 4)
ml5 = 2.01;                 % [kg] (Powerball ERB115)
ml6 = 1.471;                % [kg] (Gripper)
m_platform = m_platform + ml1;
ml1A = ml2;
ml2A = ml3 + ml4;
ml3A = ml5 + ml6;
ml1B = ml2;
ml2B = ml3 + ml4;
ml3B = ml5 + ml6;

% Inertia tensors from the CAD with a CoM frame coincident with the i-th frame (from the CAD)
I_carroponte = diag([37.5, 552.083, 552.083]);
I_cables = diag([0.0416667, 90.0025, 90.0025]);
I_platform = diag([0.587573, 19.0961, 19.0961]);
Il1 = diag([0.002795, 0.0185169, 0.0185169]);
Il2 = diag([0.00863167, 0.0444639, 0.0444639]);
Il3 = diag([0.00580167, 0.0234568, 0.0234568]);
Il1A = Il1;
Il2A = Il2;
Il3A = Il3;
Il1B = Il1;
Il2B = Il2;
Il3B = Il3;

% Position of the CoM with respect to the i-th frame (from the CAD)
inertial_disp_crane = [0 0 0]';
inertial_disp_cables = [0 0 0]';
inertial_disp_platform = [0 0 0]';
inertial_disp_1 = [0 0 0]';
inertial_disp_2 = [0 0 0]';         
inertial_disp_3 = [0 0 0]'; 
inertial_disp_1A = inertial_disp_1;        
inertial_disp_2A = inertial_disp_2; 
inertial_disp_3A = inertial_disp_3;         
inertial_disp_1B = inertial_disp_1; 
inertial_disp_2B = inertial_disp_2;         
inertial_disp_3B = inertial_disp_3; 

% Values of the frictions
fv1p = 0;
fv2p = 0;
fv1 = 0;
fv2 = 0;
fv3 = 0;
fv1A = fv1;
fv2A = fv2;
fv3A = fv3;
fv1B = fv1;
fv2B = fv2;
fv3B = fv3;

