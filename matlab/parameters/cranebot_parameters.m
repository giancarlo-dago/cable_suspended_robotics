% Gravity acceleration
g0 = 9.81;                  % [m\s^2]

% Kinematic parameters
L = 3.16 + 0.126;           % [m] (Distance between the first passive joint and the second passive joint)
D = 1.047;                  % [m] (Distance between the second passive joint and the base of the schunks)
L1 = 0.350;                 % [m] (Distance between the center of the two ERB145)
L2 = 0.305;                 % [m] (Distance between the center of the second ERB145 and the ERB115)
L3 = 0.2662;                % [m] (Distance between the center of the ERB115 and the end-effector (center of the tip of the finger of the gripper))
offA = -0.19;               % [m] (Half of the distance between the arms)
offB = 0.19;                % [m] (Half of the distance between the arms)

% Mass values
m_crane = 1000;             % [kg] (Crane)
m_cables = 36;              % [kg] (Cables + pulley)
m_platform = 156.686;       % [kg] (Hook + platform + schunk bases)
m_stand = 2.618;            % [kg] (Schunk bases)
ml1 = 4.1513;               % [kg] (Powerball ERB145)
ml2 = 1.677;                % [kg] (Link 2)
ml3 = 4.1513;               % [kg] (Powerball ERB145)
ml4 = 1.0277;               % [kg] (Link 4)
ml5 = 2.01;                 % [kg] (Powerball ERB115)
ml6 = 1.471;                % [kg] (Gripper)

% Inertia tensors from the CAD with a CoM frame coincident with the i-th frame (from the CAD)
ixx_cables = 35.637339701;
ixy_cables = 0;
ixz_cables = 0;
iyy_cables = 35.874452244;
iyz_cables = 0;
izz_cables = 0.736738204;
ixx_platform = 10.058580977;
ixy_platform = 0.024604081;
ixz_platform = -0.001721003;
iyy_platform = 7.905514545;
iyz_platform = -0.010645008;
izz_platform = 6.580734591;
ixx_stand = 0.007406232;
ixy_stand = 0.000014716;
ixz_stand = 0.000005648;
iyy_stand = 0.007722718;
iyz_stand = 0.000084398;
izz_stand = 0.007469274;
ixx_1 = 0.011901256;
ixy_1 = 0.000009512;
ixz_1 = 0.000013360;
iyy_1 = 0.009939753;
iyz_1 = 0.000193622;
izz_1 = 0.009939770;
ixx_2 = 0.040667106;
ixy_2 = 0.000000423;
ixz_2 = 0.000012806;
iyy_2 = 0.002418749;
iyz_2 = 0.0;
izz_2 = 0.041331488;
ixx_3 = 0.009940472;
ixy_3 = -0.000009512;
ixz_3 = -0.000193636;
iyy_3 = 0.011902116;
iyz_3 = 0.000013361;
izz_3 = 0.009940489;
ixx_4 = 0.007319651;
ixy_4 = -0.000007532;
ixz_4 = 0.000009231;
iyy_4 = 0.006465057;
iyz_4 = 0.002050576;
izz_4 = 0.001867734;
ixx_5 = 0.003730197;
ixy_5 = -0.000003264;
ixz_5 = -0.000004826;
iyy_5 = 0.002928014;
iyz_5 = 0.000056625;
izz_5 = 0.003298819;
ixx_6 = 0.004474138;
ixy_6 = -0.000005207;
ixz_6 = 0.000000909;
iyy_6 = 0.003582612;
iyz_6 = 0.000000225;
izz_6 = 0.002264269;

I_cables = [ixx_cables ixy_cables ixz_cables; ixy_cables iyy_cables iyz_cables; ixz_cables iyz_cables izz_cables];
I_platform = [ixx_platform ixy_platform ixz_platform; ixy_platform iyy_platform iyz_platform; ixz_platform iyz_platform izz_platform];
I_stand = [ixx_stand ixy_stand ixz_stand; ixy_stand iyy_stand iyz_stand; ixz_stand iyz_stand izz_stand];
Il1 = [ixx_1 ixy_1 ixz_1; ixy_1 iyy_1 iyz_1; ixz_1 iyz_1 izz_1];
Il2 = [ixx_2 ixy_2 ixz_2; ixy_2 iyy_2 iyz_2; ixz_2 iyz_2 izz_2];
Il3 = [ixx_3 ixy_3 ixz_3; ixy_3 iyy_3 iyz_3; ixz_3 iyz_3 izz_3];
Il4 = [ixx_4 ixy_4 ixz_4; ixy_4 iyy_4 iyz_4; ixz_4 iyz_4 izz_4];
Il5 = [ixx_5 ixy_5 ixz_5; ixy_5 iyy_5 iyz_5; ixz_5 iyz_5 izz_5];
Il6 = [ixx_6 ixy_6 ixz_6; ixy_6 iyy_6 iyz_6; ixz_6 iyz_6 izz_6];

% Position of the CoM with respect to the i-th frame (from the CAD)
inertial_disp_cables = [0 0 -2.474]';
inertial_disp_platform = [0.000773, 0.000370, -0.549826]';      % [m] (Platform)
inertial_disp_stand = [0.000133, 0.00238, 0.05653]';            % [m] (Schunk Base)
inertial_disp_1 = [0.000133, 0.00238, 0.05653]';                % [m] (Powerball ERB145)
inertial_disp_2 = [0.000380, 0.175, 0.102383]';                 % [m] (Link2)
inertial_disp_3 = [0.006905, 0.000078, -0.006906]';             % [m] (Powerball ERB145)
inertial_disp_4 = [0.000231, -0.046821, 0.173837]';             % [m] (Link4)
inertial_disp_5 = [-0.000062, -0.007296, -0.003888]';           % [m] (Powerball ERB115)
inertial_disp_6 = [0.000008, -0.000045, 0.133403]';             % [m] (Gripper)

% Values of the frictions
fv1p = 0;
fv2p = 0;
fv1 = 0;
fv2 = 0;
fv3 = 0;
fv4 = 0;
fv5 = 0;
fv6 = 0;

% Joints position limits
J1_lower_limit = -170;      % [deg]
J1_upper_limit = 170;       % [deg]
J2_lower_limit = -170;      % [deg]
J2_upper_limit = 170;       % [deg]
J3_lower_limit = -155.5;    % [deg]
J3_upper_limit = 155.5;     % [deg]
J4_lower_limit = -170;      % [deg]
J4_upper_limit = 170;       % [deg]
J5_lower_limit = -170;      % [deg]
J5_upper_limit = 170;       % [deg]
J6_lower_limit = -170;      % [deg]
J6_upper_limit = 170;       % [deg]

% Other lengths
d1 = 0.120;                 % [m]
d2 = 0.175;                 % [m]
d3 = 0.052;                 % [m]
d4 = 0.153;                 % [m]
d5 = 0.120;                 % [m]

