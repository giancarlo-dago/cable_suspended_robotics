function [p1A,p2A,p3A,p4A,p5A,p6A,peA,p1B,p2B,p3B,p4B,p5B,p6B,peB] = kinematics_lwa4p_f(theta,offA,offB,L1,L2,L3,D1,D2)

    % Input
    thetaA = theta(1:6);
    thetaB = theta(7:12);
    
    % Initializations
    SA = zeros(6,6);
    SB = zeros(6,6);

    % Kinematic description    
    omega1A = [0 0 -1]';
    omega2A = [1 0 0]';
    omega3A = [-1 0 0]';
    omega4A = [0 0 -1]';
    omega5A = [-1 0 0]';
    omega6A = [0 0 -1]';
    omega1B = [0 0 -1]';
    omega2B = [-1 0 0]';
    omega3B = [1 0 0]';
    omega4B = [0 0 -1]';
    omega5B = [1 0 0]';
    omega6B = [0 0 -1]';

    q1A = [0 offA 0]';
    q2A = [0 offA 0]';
    q3A = [0 offA -L1]';
    q4A = [0 offA -L1]';
    q5A = [0 offA -L1-L2]';
    q6A = [0 offA -L1-L2]';
    q1B = [0 offB 0]';
    q2B = [0 offB 0]';
    q3B = [0 offB -L1]';
    q4B = [0 offB -L1]';
    q5B = [0 offB -L1-L2]';
    q6B = [0 offB -L1-L2]';

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

    % Screw axis computation
    omegaA = [omega1A, omega2A, omega3A, omega4A, omega5A, omega6A];
    qA = [q1A, q2A, q3A, q4A, q5A, q6A];
    for i = 1:length(omegaA)
        vA = cross(-omegaA(:,i),qA(:,i));
        SA(:,i) = [omegaA(:,i); vA];
    end
    omegaB = [omega1B, omega2B, omega3B, omega4B, omega5B, omega6B];
    qB = [q1B, q2B, q3B, q4B, q5B, q6B];
    for i = 1:length(omegaB)
        vB = cross(-omegaB(:,i),qB(:,i));
        SB(:,i) = [omegaB(:,i); vB];
    end

    % Kinematics (position of the joints and of the end effector)
    T1A = M1A;
    T2A = fkin_f(thetaA(1), 1, M2A, SA(:,1));
    T3A = fkin_f(thetaA(1:2), 2, M3A, SA(:,1:2));
    T4A = fkin_f(thetaA(1:3), 3, M4A, SA(:,1:3));
    T5A = fkin_f(thetaA(1:4), 4, M5A, SA(:,1:4));
    T6A = fkin_f(thetaA(1:5), 5, M6A, SA(:,1:5));
    TeA = fkin_f(thetaA(1:6), 6, MeA, SA(:,1:6));
    T1B = M1B;
    T2B = fkin_f(thetaB(1), 1, M2B, SB(:,1));
    T3B = fkin_f(thetaB(1:2), 2, M3B, SB(:,1:2));
    T4B = fkin_f(thetaB(1:3), 3, M4B, SB(:,1:3));
    T5B = fkin_f(thetaB(1:4), 4, M5B, SB(:,1:4));
    T6B = fkin_f(thetaB(1:5), 5, M6B, SB(:,1:5));
    TeB = fkin_f(thetaB(1:6), 6, MeB, SB(:,1:6));
    p1A = T1A(1:3,4);
    p2A = T2A(1:3,4);
    p3A = T3A(1:3,4);
    p4A = T4A(1:3,4);
    p5A = T5A(1:3,4);
    p6A = T6A(1:3,4);
    peA = TeA(1:3,4);
    p1B = T1B(1:3,4);
    p2B = T2B(1:3,4);
    p3B = T3B(1:3,4);
    p4B = T4B(1:3,4);
    p5B = T5B(1:3,4);
    p6B = T6B(1:3,4);
    peB = TeB(1:3,4);

end

