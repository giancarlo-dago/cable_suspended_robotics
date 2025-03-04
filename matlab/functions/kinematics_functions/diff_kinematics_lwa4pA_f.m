function J = diff_kinematics_lwa4pA_f(thetaA,n_joints,offA,L1,L2,L3,D1,D2)

    % Initializations
    SA = zeros(6,6);
    MA_rest = zeros(4,4,7);
   
    % Kinematic description    
    omega1A = [0 0 -1]';
    omega2A = [1 0 0]';
    omega3A = [-1 0 0]';
    omega4A = [0 0 -1]';
    omega5A = [-1 0 0]';
    omega6A = [0 0 -1]';
    
    q1A = [0 offA 0]';
    q2A = [0 offA 0]';
    q3A = [0 offA -L1]';
    q4A = [0 offA -L1]';
    q5A = [0 offA -L1-L2]';
    q6A = [0 offA -L1-L2]';
    
    M1A = [0 -1  0  0;
          -1  0  0 offA;
           0  0 -1  0;
           0  0  0  1];

    M2A = [0  0  1  D1;
          -1  0  0 offA;
           0 -1  0  0;
           0  0  0  1];

    M3A = [0  0 -1   D1;
           0 -1  0  offA;
          -1  0  0  -L1;
           0  0  0   1];
    
    M4A = [0 -1  0   0;
          -1  0  0  offA;
           0  0 -1  -L1;
           0  0  0   1];

    M5A = [0  0 -1    D2;
          -1  0  0   offA;
           0  1  0  -L1-L2;
           0  0  0    1];    

    M6A = [0 -1  0    0;
          -1  0  0   offA;
           0  0 -1  -L1-L2;
           0  0  0    1]; 
    
    MeA = [0 -1  0    0;
          -1  0  0   offA;
           0  0 -1 -L1-L2-L3;
           0  0  0    1]; 
            
    MA_rest(:,:,1) = M1A;
    MA_rest(:,:,2) = M2A;
    MA_rest(:,:,3) = M3A;
    MA_rest(:,:,4) = M4A;
    MA_rest(:,:,5) = M5A;
    MA_rest(:,:,6) = M6A;
    MA_rest(:,:,7) = MeA;
            
    % Screw axis computation
    omegaA = [omega1A, omega2A, omega3A, omega4A, omega5A, omega6A];
    qA = [q1A, q2A, q3A, q4A, q5A, q6A];
    for i = 1:length(omegaA)
        vA = cross(-omegaA(:,i),qA(:,i));
        SA(:,i) = [omegaA(:,i); vA];
    end
    
    % Jacobian computation
    T = fkin_f(thetaA(1:n_joints), n_joints, MA_rest(:,:,n_joints+1), SA(:,1:n_joints));           % Computation of the transformation matrices
    Js = diffkin_f(thetaA(1:n_joints), n_joints, SA);                                              % Computation of the Screw Theory Jacobian (space jacobian - Lynch/Park)
    J = [-skew_f(T(1:3,4)) eye(3); eye(3) zeros(3)]*Js;                                            % Converting into the geometric Jacobian (Siciliano)
    
end

