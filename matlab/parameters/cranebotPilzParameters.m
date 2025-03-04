%% PARAMETERS CRANEBOT AND PILZ

% Gravity acceleration
g0 = 9.8;           % [m\s^2]

% KINEMATICS
L = 3.16 + 0.126;   % [m] (Distance between the first passive joint and the second passive joint)
D = 1.047;          % [m] (Distance between the second passive joint and the base of the arms)
off = 0.19;         % [m] (Half of the distance between the arms)
L0 = 0.2604;    	% [m] Height of the foot
L1 = 0.35;          % [m] Length of the first connector
L2 = 0.3070;        % [m] Length of the second connector
L3 = 0.0840;        % [m] Distance last joint to flange

% MASS (kg)
massCablesPulleys = 36;     % [kg] (Cables + pulley)
massPlatform = 156.25;      % [kg] (Hook + platform + schunk bases)
massArmsFoot = 2.4;
massArmsLink1 = 7.1;
massArmsLink2 = 1.7;
massArmsLink3 = 4.8;
massArmsLink4 = 0.9;
massArmsLink5 = 2.6;
massArmsFlange = 0.2;

% INERTIAS
inertiaCablesPulleys = [35.637, 0, 0; 0, 35.874, 0; 0 0 7.36e-1];
inertiaPlatform = [6.774, -2.20e-2, -7.53e-3; -2.20e-2, 9.368, 2.67e-3; -7.53e-3 2.67e-3 6.589];
InertiaArmsFoot = [0, 0, 0; 0, 0, 0; 0, 0, 0];
InertiaArmsLink1 = [3.53e-2, 0, 0; 0, 3.01e-2, 9.04e-3; 0, 9.04e-3, 1.87e-2];
InertiaArmsLink2 = [3.58e-2, 0, 0; 0, 3.22e-3, -1.45e-3; 0, -1.45e-3, 3.66e-2];
InertiaArmsLink3 = [2.06e-2, 0, 0; 0, 1.08e-2, -5.36e-3; 0, -5.36e-3, 1.63e-2];
InertiaArmsLink4 = [6.79e-3, 0, 0; 0, 4.91e-3, 2.67e-3; 0, 2.67e-3, 3.40e-3];
InertiaArmsLink5 = [7.37e-3, 4.78e-6, 6.00e-5; 4.78e-6, 5.87e-3, -1.71e-3; 6.00e-5, -1.71e-3, 3.96e-3];
% InertiaArmsFlange = [0, 0, 0; 0, 0, 0; 0, 0, 0];
InertiaArmsFlange = [1e-2, 0, 0; 0, 1e-2, 0; 0, 0, 1e-2];

% CENTER OF MASS
comCablesPulleys = [0 0 -2.474]';
comPlatform = [0 0 -0.521]';
comArmsLink1 = [0 -0.026 -0.051]';
comArmsLink2 = [0 0.162 0.134]';
comArmsLink3 = [0 0.043 -0.027]';
comArmsLink4 = [0 -0.061 0.204]';
comArmsLink5 = [-0.001 0.021 -0.032]';
comArmsFlange = [0 0 0]';

% JOINT LIMITS
effLimArmsJ1 = 370;
effLimArmsJ2 = 370;
effLimArmsJ3 = 176;
effLimArmsJ4 = 176;
effLimArmsJ5 = 41.6;
effLimArmsJ6 = 20.1;
velLimArms = 1.57;
posLimArmsJ1 = 2.96706;
posLimArmsJ2 = 2.53073;
posLimArmsJ3 = 2.35620;
posLimArmsJ4 = 2.96706;
posLimArmsJ5 = 2.96706;
posLimArmsJ6 = 03.12414;

% JOINT FRICTION
dampCablesJx = 0.0;
dampCablesJy = 0.0;
dampCablesJz = 0.0;
dampPlatformJx = 0.0;
dampPlatformJy = 0.0;
dampPlatformJz = 0.0;
dampArmsJ1 = 15.0;
dampArmsJ2 = 12.0;
dampArmsJ3 = 4.5;
dampArmsJ4 = 4.2;
dampArmsJ5 = 1.5;
dampArmsJ6 = 4.5;
fricCablesJx = 0.0;
fricCablesJy = 0.0;
fricCablesJz = 0.0;
fricPlatformJx = 0.0;
fricPlatformJy = 0.0;
fricPlatformJz = 0.0;
fricArmsJ1 = 11.0;
fricArmsJ2 = 10.0;
fricArmsJ3 = 4.5;
fricArmsJ4 = 3.8;
fricArmsJ5 = 3.5;
fricArmsJ6 = 7.0;


      
      
      
      



