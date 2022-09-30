function tau = recursive_invdyn_f(theta, dtheta, ddtheta, g, info, F_ee)

    N_JOINTS = info.n_joints;
    S = info.S;
    M_bi = info.M_bi;
    Inertia = info.Inertia;
    mass = info.mass;
    inertial_disp = info.inertial_disp;
    

    % Preallocation
    A = zeros(6,N_JOINTS);
    M_mutual = zeros(4,4,N_JOINTS+1);
    T_mutual = zeros(4,4,N_JOINTS+1);
    G = zeros(6,6,N_JOINTS);
    V = zeros(6,N_JOINTS);
    Vd = zeros(6,N_JOINTS); 
    F = zeros(6,N_JOINTS);
    tau = zeros(N_JOINTS,1);
    
    
    % Preliminary computations
    for i=1:N_JOINTS
        A(:,i) = Ad_f(inv(M_bi(:,:,i))) * S(:,i);                                   % Computation of screw vector of joint i referred to frame {i}
        if i == 1                                                                   % Computation M_{i,i-1}: tranformation matrix at rest position
            M_mutual(:,:,i) = inv(M_bi(:,:,i));
        else
            M_mutual(:,:,i) = M_bi(:,:,i)\M_bi(:,:,i-1);
        end   
        T_mutual(:,:,i) = expm(-bracket_f(A(:,i))*theta(i)) * M_mutual(:,:,i);      % Computation T_{i,i-1}: tranformation matrix in a general configuration 
        G(:,:,i) = G_f(mass(i),Inertia(:,:,i),inertial_disp(i,:));                  % Referring the spatial inertia matrix in the i-th frame
    end
    M_mutual(:,:,N_JOINTS+1) = M_bi(:,:,N_JOINTS+1)\M_bi(:,:,N_JOINTS);             % Adding M_{e,N_JOINTS}
    T_mutual(:,:,N_JOINTS+1) = M_mutual(:,:,N_JOINTS+1);                            % Adding T_{e,N_JOINTS}
    
    
    % Forward recursion
    V_0 = zeros(6,1);                                                               % Starting twist
    Vd_0 = [zeros(3,1); -g];                                                        % Starting accelerations
    for i=1:N_JOINTS
        if i == 1
            V_prev = V_0;
            Vd_prev = Vd_0;
        else
            V_prev = V(:,i-1);  
            Vd_prev = Vd(:,i-1); 
        end
        V(:,i) = Ad_f(T_mutual(:,:,i))*V_prev + A(:,i)*dtheta(i);                   % Computing the twist for each link by a recursive formula
        Vd(:,i) = ad_f_(V(:,i))*(A(:,i)*dtheta(i)) + ...                            % Computing the acceleration (derivative of twist) for each link by a recurisive formula
                    Ad_f(T_mutual(:,:,i))*Vd_prev + A(:,i)*ddtheta(i);
    end
    
    
    % Backward recursion
    for i=N_JOINTS:-1:1        
        if i == N_JOINTS
            F_next = F_ee;
            T_mutual_next = T_mutual(:,:,i+1);
        else
            F_next = F(:,i+1);
            T_mutual_next = T_mutual(:,:,i+1);
        end
        F(:,i) = Ad_f(T_mutual_next)'*F_next + ...                                   % Computing wrenches for each link by a recursive formula
                 G(:,:,i)*Vd(:,i)- ad_f_(V(:,i))'*(G(:,:,i)*V(:,i));
        tau(i) = F(:,i)'*A(:,i);                                                     % Computing the torque for each joint from the direction of the screw axis
    end

end

