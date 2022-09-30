function [J1,J2] = Jvertices_spatial_f(q,link,offA,offB,L1,L2,L3,D1,D2)

    % Initialization
    J1 = zeros(3,6);
    J2 = zeros(3,6);  

    % Input
    th1A = q(1);
    th2A = q(2);
    th3A = q(3);
    th4A = q(4);
    th5A = q(5);
    th6A = q(6);
    th1B = q(7);
    th2B = q(8);
    th3B = q(9);
    th4B = q(10);
    th5B = q(11);
    th6B = q(12);
        
    % Jacobian evaluation
    switch link 
        case 1
            J1A = diff_kinematics_lwa4pA_f([0, 0, 0, 0, 0, 0], 0, offA, L1, L2, L3, D1, D2);
            J2A = diff_kinematics_lwa4pA_f([th1A, 0, 0, 0, 0, 0], 1, offA, L1, L2, L3, D1, D2);
            J1 = [J1A(1:3,:) zeros(3,6)];
            J2 = [J2A(1:3,:) zeros(3,6)];
        case 2
            J1A = diff_kinematics_lwa4pA_f([th1A, 0, 0, 0, 0, 0], 0, offA, L1, L2, L3, D1, D2);
            J2A = diff_kinematics_lwa4pA_f([th1A, th2A, 0, 0, 0, 0], 2, offA, L1, L2, L3, D1, D2);
            J1 = [J1A(1:3,:) zeros(3,6)];
            J2 = [J2A(1:3,:) zeros(3,6)];
        case 3
            J1A = diff_kinematics_lwa4pA_f([th1A, th2A, 0, 0, 0, 0], 2, offA, L1, L2, L3, D1, D2);
            J2A = diff_kinematics_lwa4pA_f([th1A, th2A, th3A, 0, 0, 0], 3, offA, L1, L2, L3, D1, D2);
            J1 = [J1A(1:3,:) zeros(3,6)];
            J2 = [J2A(1:3,:) zeros(3,6)];
        case 4
            J1A = diff_kinematics_lwa4pA_f([th1A, th2A, th3A, 0, 0, 0], 3, offA, L1, L2, L3, D1, D2);
            J2A = diff_kinematics_lwa4pA_f([th1A, th2A, th3A, th4A, 0, 0], 4, offA, L1, L2, L3, D1, D2);
            J1 = [J1A(1:3,:) zeros(3,6)];
            J2 = [J2A(1:3,:) zeros(3,6)];
        case 5
            J1A = diff_kinematics_lwa4pA_f([th1A, th2A, th3A, th4A, 0, 0], 4, offA, L1, L2, L3, D1, D2);
            J2A = diff_kinematics_lwa4pA_f([th1A, th2A, th3A, th4A, th5A, 0], 5, offA, L1, L2, L3, D1, D2);
            J1 = [J1A(1:3,:) zeros(3,6)];
            J2 = [J2A(1:3,:) zeros(3,6)];
        case 6
            J1A = diff_kinematics_lwa4pA_f([th1A, th2A, th3A, th4A, th5A, 0], 5, offA, L1, L2, L3, D1, D2);
            J2A = diff_kinematics_lwa4pA_f([th1A, th2A, th3A, th4A, th5A, th6A], 6, offA, L1, L2, L3, D1, D2);
            J1 = [J1A(1:3,:) zeros(3,6)];
            J2 = [J2A(1:3,:) zeros(3,6)];
        case 7
            J1B = diff_kinematics_lwa4pB_f([0, 0, 0, 0, 0, 0], 0, offB, L1, L2, L3, D1, D2);
            J2B = diff_kinematics_lwa4pB_f([th1B, 0, 0, 0, 0, 0], 1, offB, L1, L2, L3, D1, D2);
            J1 = [zeros(3,6) J1B(1:3,:)];
            J2 = [zeros(3,6) J2B(1:3,:)];
        case 8
            J1B = diff_kinematics_lwa4pB_f([th1B, 0, 0, 0, 0, 0], 0, offB, L1, L2, L3, D1, D2);
            J2B = diff_kinematics_lwa4pB_f([th1B, th2B, 0, 0, 0, 0], 2, offB, L1, L2, L3, D1, D2);
            J1 = [zeros(3,6) J1B(1:3,:)];
            J2 = [zeros(3,6) J2B(1:3,:)];
        case 9
            J1B = diff_kinematics_lwa4pB_f([th1B, th2B, 0, 0, 0, 0], 2, offB, L1, L2, L3, D1, D2);
            J2B = diff_kinematics_lwa4pB_f([th1B, th2B, th3B, 0, 0, 0], 3, offB, L1, L2, L3, D1, D2);
            J1 = [zeros(3,6) J1B(1:3,:)];
            J2 = [zeros(3,6) J2B(1:3,:)];
        case 10
            J1B = diff_kinematics_lwa4pB_f([th1B, th2B, th3B, 0, 0, 0], 3, offB, L1, L2, L3, D1, D2);
            J2B = diff_kinematics_lwa4pB_f([th1B, th2B, th3B, th4B, 0, 0], 4, offB, L1, L2, L3, D1, D2);
            J1 = [zeros(3,6) J1B(1:3,:)];
            J2 = [zeros(3,6) J2B(1:3,:)];
        case 11
            J1B = diff_kinematics_lwa4pB_f([th1B, th2B, th3B, th4B, 0, 0], 4, offB, L1, L2, L3, D1, D2);
            J2B = diff_kinematics_lwa4pB_f([th1B, th2B, th3B, th4B, th5B, 0], 5, offB, L1, L2, L3, D1, D2);
            J1 = [zeros(3,6) J1B(1:3,:)];
            J2 = [zeros(3,6) J2B(1:3,:)];
        case 12
            J1B = diff_kinematics_lwa4pB_f([th1B, th2B, th3B, th4B, th5B, 0], 5, offB, L1, L2, L3, D1, D2);
            J2B = diff_kinematics_lwa4pB_f([th1B, th2B, th3B, th4B, th5B, th6B], 6, offB, L1, L2, L3, D1, D2);
            J1 = [zeros(3,6) J1B(1:3,:)];
            J2 = [zeros(3,6) J2B(1:3,:)];
    end
    
    J1 = J1(1:3,:);
    J2 = J2(1:3,:);
                  
end

