function J = diff_kinematics_lwa4pB_f(thetaB,n_joints,offB,L1,L2,L3,D1,D2)

    % Initializations
    SB = zeros(6,6);
    MB_rest = zeros(4,4,7);
   
    % Kinematic description
    omega1B = [0 0 -1]';
    omega2B = [-1 0 0]';
    omega3B = [1 0 0]';
    omega4B = [0 0 -1]';
    omega5B = [1 0 0]';
    omega6B = [0 0 -1]';

    q1B = [0 offB 0]';
    q2B = [0 offB 0]';
    q3B = [0 offB -L1]';
    q4B = [0 offB -L1]';
    q5B = [0 offB -L1-L2]';
    q6B = [0 offB -L1-L2]';
       
    M1B = [0  1  0  0;
           1  0  0 offB;
           0  0 -1  0;
           0  0  0  1];

    M2B = [0  0 -1 -D1;
           1  0  0 offB;
           0 -1  0  0;
           0  0  0  1];

    M3B = [0  0  1  -D1;
           0  1  0  offB;
          -1  0  0  -L1;
           0  0  0   1];
    
    M4B = [0  1  0   0;
           1  0  0  offB;
           0  0 -1  -L1;
           0  0  0   1];

    M5B = [0  0  1   -D2;
           1  0  0   offB;
           0  1  0 -L1-L2;
           0  0  0    1];    

    M6B = [0  1  0    0;
           1  0  0   offB;
           0  0 -1 -L1-L2;
           0  0  0    1]; 
         
    MeB = [0  1  0    0;
           1  0  0   offB;
           0  0 -1 -L1-L2-L3;
           0  0  0    1]; 
            
    MB_rest(:,:,1) = M1B;
    MB_rest(:,:,2) = M2B;
    MB_rest(:,:,3) = M3B;
    MB_rest(:,:,4) = M4B;
    MB_rest(:,:,5) = M5B;
    MB_rest(:,:,6) = M6B;
    MB_rest(:,:,7) = MeB;
            
    % Screw axis computation
    omegaB = [omega1B, omega2B, omega3B, omega4B, omega5B, omega6B];
    qB = [q1B, q2B, q3B, q4B, q5B, q6B];
    for i = 1:length(omegaB)
        vB = cross(-omegaB(:,i),qB(:,i));
        SB(:,i) = [omegaB(:,i); vB];
    end
    
    % Jacobian computation
    T = fkin_f(thetaB(1:n_joints), n_joints, MB_rest(:,:,n_joints+1), SB(:,1:n_joints));           % Computation of the transformation matrices
    Js = diffkin_f(thetaB(1:n_joints), n_joints, SB);                                              % Computation of the Screw Theory Jacobian (space jacobian - Lynch/Park)
    J = [-skew_f(T(1:3,4)) eye(3); eye(3) zeros(3)]*Js;                                            % Converting into the geometric Jacobian (Siciliano)

end

