function [p_P1A,p_P2A,p_P3A,p_P4A,p_P5A,p_P6A,p_P1B,p_P2B,p_P3B,p_P4B,p_P5B,p_P6B] = kinematics_CoM_lwa4p_f(theta,offA,offB,L1,L2,d1,d2,d3,d4,d5)

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

    % Transformation matrices of the frames of the CoMs
    P1A = [  0 offA         0]';
    P2A = [ d1 offA       -d2]';
    P3A = [  0 offA       -L1]';
    P4A = [ d3 offA    -L1-d4]';
    P5A = [  0 offA    -L1-L2]';
    P6A = [  0 offA -L1-L2-d5]';  
    P1B = [  0 offB         0]';
    P2B = [-d1 offB       -d2]';
    P3B = [  0 offB       -L1]';
    P4B = [-d3 offB    -L1-d4]';
    P5B = [  0 offB    -L1-L2]';
    P6B = [  0 offB -L1-L2-d5]'; 
    
    M_P1A = [1  0  0  P1A(1);
             0  1  0  P1A(2);
             0  0  1  P1A(3);
             0  0  0    1];
       
    M_P2A = [1  0  0  P2A(1);
             0  1  0  P2A(2);
             0  0  1  P2A(3);
             0  0  0    1];

    M_P3A = [1  0  0  P3A(1);
             0  1  0  P3A(2);
             0  0  1  P3A(3);
             0  0  0    1];
       
    M_P4A = [1  0  0  P4A(1);
             0  1  0  P4A(2);
             0  0  1  P4A(3);
             0  0  0    1];

    M_P5A = [1  0  0  P5A(1);
             0  1  0  P5A(2);
             0  0  1  P5A(3);
             0  0  0    1];
       
    M_P6A = [1  0  0  P6A(1);
             0  1  0  P6A(2);
             0  0  1  P6A(3);
             0  0  0    1];

    M_P1B = [1  0  0  P1B(1);
             0  1  0  P1B(2);
             0  0  1  P1B(3);
             0  0  0    1];
       
    M_P2B = [1  0  0  P2B(1);
             0  1  0  P2B(2);
             0  0  1  P2B(3);
             0  0  0    1];

    M_P3B = [1  0  0  P3B(1);
             0  1  0  P3B(2);
             0  0  1  P3B(3);
             0  0  0    1];
       
    M_P4B = [1  0  0  P4B(1);
             0  1  0  P4B(2);
             0  0  1  P4B(3);
             0  0  0    1];

    M_P5B = [1  0  0  P5B(1);
             0  1  0  P5B(2);
             0  0  1  P5B(3);
             0  0  0    1];
       
    M_P6B = [1  0  0  P6B(1);
             0  1  0  P6B(2);
             0  0  1  P6B(3);
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

    % Kinematics (position of the CoMs)
    T_P1A = fkin_f(thetaA(1),   1, M_P1A, SA(:,1));
    T_P2A = fkin_f(thetaA(1:2), 2, M_P2A, SA(:,1:2));
    T_P3A = fkin_f(thetaA(1:3), 3, M_P3A, SA(:,1:3));
    T_P4A = fkin_f(thetaA(1:4), 4, M_P4A, SA(:,1:4));
    T_P5A = fkin_f(thetaA(1:5), 5, M_P5A, SA(:,1:5));
    T_P6A = fkin_f(thetaA(1:6), 6, M_P6A, SA(:,1:6));
    T_P1B = fkin_f(thetaB(1),   1, M_P1B, SB(:,1));
    T_P2B = fkin_f(thetaB(1:2), 2, M_P2B, SB(:,1:2));
    T_P3B = fkin_f(thetaB(1:3), 3, M_P3B, SB(:,1:3));
    T_P4B = fkin_f(thetaB(1:4), 4, M_P4B, SB(:,1:4));
    T_P5B = fkin_f(thetaB(1:5), 5, M_P5B, SB(:,1:5));
    T_P6B = fkin_f(thetaB(1:6), 6, M_P6B, SB(:,1:6));
    
    p_P1A = T_P1A(1:3,4);
    p_P2A = T_P2A(1:3,4);
    p_P3A = T_P3A(1:3,4);
    p_P4A = T_P4A(1:3,4);
    p_P5A = T_P5A(1:3,4);
    p_P6A = T_P6A(1:3,4);
    p_P1B = T_P1B(1:3,4);
    p_P2B = T_P2B(1:3,4);
    p_P3B = T_P3B(1:3,4);
    p_P4B = T_P4B(1:3,4);
    p_P5B = T_P5B(1:3,4);
    p_P6B = T_P6B(1:3,4);

end

