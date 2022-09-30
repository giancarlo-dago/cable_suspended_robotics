function output = skeleton_algorithm_lwa4p_f(in)

    % Input
    q = in(1:12);
    qd = in(13:24);
    JUMP = reshape(in(25:168),12,12);
    T0 = reshape(in(169:312),12,12);
    INDEX_PREV = reshape(in(313:456),12,12);
    t = in(457);
    
    % Parameters skeleton algorithm    
    offA = -0.19;
    offB = 0.19;
    L1 = 0.350;
    L2 = 0.305;
    L3 = 0.2662;
    D1 = 0.101;
    D2 = 0.078;    
    CM = [0 0 0 0 0 0 0 0 0 0 0 0;          % Collision Matrix
          0 0 0 0 0 0 1 1 1 1 1 1;
          0 0 0 0 0 0 1 1 1 1 1 1;
          0 0 0 0 0 0 1 1 1 1 1 1;
          0 0 0 0 0 0 1 1 1 1 1 1;
          0 0 0 0 0 0 1 1 1 1 1 1;
          0 1 1 1 1 1 0 0 0 0 0 0;
          0 1 1 1 1 1 0 0 0 0 0 0;
          0 1 1 1 1 1 0 0 0 0 0 0;
          0 1 1 1 1 1 0 0 0 0 0 0;
          0 1 1 1 1 1 0 0 0 0 0 0;
          0 1 1 1 1 1 0 0 0 0 0 0];
%     CM = [0 0 0 1 1 1 0 1 1 1 1 1;          % Collision Matrix
%           0 0 0 0 0 1 1 1 1 1 1 1;
%           0 0 0 0 0 1 1 1 1 1 1 1;
%           1 0 0 0 0 1 1 1 1 1 1 1;
%           1 0 0 0 0 0 1 1 1 1 1 1;
%           1 1 1 1 0 0 1 1 1 1 1 1;
%           0 1 1 1 1 1 0 0 0 1 1 1;
%           1 1 1 1 1 1 0 0 0 0 0 1;
%           1 1 1 1 1 1 0 0 0 0 0 1;
%           1 1 1 1 1 1 1 0 0 0 0 1;
%           1 1 1 1 1 1 1 0 0 0 0 0;
%           1 1 1 1 1 1 1 1 1 1 0 0];
    d_0 = 0.0725;                           % Radius of the cylinder circumscribed around the link
    d_start = 0.25;                         % Distance between links for activating the skeleton algorithm
    n_links_collision = 12;                 % Number of links that may collide
    h_type = 1;                             % h function type (0 for linear, 1 for exponential)
    k_linear = 10;                          % Gain for the linear h function              
    k1_exp = 5;                             % First gain for the linear h function
    k2_exp = 10;                            % Second gain for the linear h function
    a = -40;                                % Exponential gain for the solution of the jumps in the Jacobians
    tk = 0.2;                               % End time of the action of the solution of the jumps in the Jacobians

    % Joints positions (links extremities positions)
    [p1a, p2a, p3a, p4a, p5a, p6a, pea, p1b, p2b, p3b, p4b, p5b, p6b, peb] = kinematics_lwa4p_f(q, offA, offB, L1, L2, L3, D1, D2);
    p(1,1,:) = p1a(1:3);
    p(1,2,:) = p2a(1:3);
    p(2,1,:) = p2a(1:3);
    p(2,2,:) = p3a(1:3);
    p(3,1,:) = p3a(1:3);
    p(3,2,:) = p4a(1:3);
    p(4,1,:) = p4a(1:3);
    p(4,2,:) = p5a(1:3);
    p(5,1,:) = p5a(1:3);
    p(5,2,:) = p6a(1:3);
    p(6,1,:) = p6a(1:3);
    p(6,2,:) = pea(1:3);
    p(7,1,:) = p1b(1:3);
    p(7,2,:) = p2b(1:3);
    p(8,1,:) = p2b(1:3);
    p(8,2,:) = p3b(1:3);
    p(9,1,:) = p3b(1:3);
    p(9,2,:) = p4b(1:3);
    p(10,1,:) = p4b(1:3);
    p(10,2,:) = p5b(1:3);
    p(11,1,:) = p5b(1:3);
    p(11,2,:) = p6b(1:3);
    p(12,1,:) = p6b(1:3);
    p(12,2,:) = peb(1:3);
        
    % Compute and sum all the avoidance torques for each couple of links in collision 
    tau = zeros(12,1);
    for link1=1:n_links_collision
        for link2=link1:n_links_collision
            if CM(link1,link2)==1                                                                                       % If the two links can collide
                p_a1 = reshape(p(link1,1,:),3,1);                                                                       % Retrieve the extremities of the two links
                p_a2 = reshape(p(link1,2,:),3,1);
                p_b1 = reshape(p(link2,1,:),3,1);
                p_b2 = reshape(p(link2,2,:),3,1);
                [d_min, p_ac, p_bc, index] = dmin_spatial_f(p_a1, p_a2, p_b1, p_b2);                                    % Compute the minimum distance between them
                
                if check_collision_f(d_min,d_0, d_start)==1                                                             % If they are in the repulsion zone
                    [f_ac, f_bc] = potential_f(p_ac, p_bc, d_min, d_0, d_start, h_type, k_linear, k1_exp, k2_exp);      % Compute the repulsion force
                    [J_a1, J_a2] = Jvertices_spatial_f(q, link1, offA, offB, L1, L2, L3, D1, D2);                       % Compute the Jacobian of the extremities of link 1
                    [J_b1, J_b2] = Jvertices_spatial_f(q, link2, offA, offB, L1, L2, L3, D1, D2);                       % Compute the Jacobian of the extremities of link 2 

                    [Jt_ac_new, Jt_bc_new] = Jt_spatial_f(p_a1, p_a2, p_b1, p_b2, index);                               % Compute the Jacobian of the collision points on link 1 and 2
                    J_ac_new = Jt_ac_new * [J_a1; J_a2; J_b1; J_b2];                                                    % Compute J_ac
                    J_bc_new = Jt_bc_new * [J_a1; J_a2; J_b1; J_b2];                                                    % Compute J_bc
                    tau_new = [J_ac_new' J_bc_new']*[f_ac; f_bc];                                                       % Compute the avoidance torque

                    if JUMP(link1,link2)==0                                                                             % If a jump was not already detected
                        if index~=INDEX_PREV(link1,link2)                                                               % Check if there was a jump in the Jacobian Jt, with respect to the previous one computed 
                            JUMP(link1,link2)=1;                                                                        % If so, raise a flag
                            T0(link1,link2) = t;                                                                        % Save the time instant when it happens
                        end
                    end

                    if JUMP(link1,link2)==1 && (t-T0(link1,link2))<tk                                                   % If there was a jump and the time passed since the detection in <tk
                        index_prev = INDEX_PREV(link1,link2);                                                           % Retreive the index related to the Jacobian Jt before the jump
                        [Jt_ac_old, Jt_bc_old] = Jt_spatial_f(p_a1, p_a2, p_b1, p_b2, index_prev);                      % Compute Jt with the previous index
                        J_ac_old = Jt_ac_old * [J_a1; J_a2; J_b1; J_b2];                                                % Compute J_ac
                        J_bc_old = Jt_bc_old * [J_a1; J_a2; J_b1; J_b2];                                                % Comput J_bc
                        tau_old = [J_ac_old' J_bc_old']*[f_ac; f_bc];                                                   % Compute the avoidance torque with the prevoius Jacobian Jt
                        out = exp(a*(t-T0(link1,link2)))*tau_old + (1-exp(a*(t-T0(link1,link2))))*tau_new;              % Switch tau_new and tau_old smoothly
                    else                                                                                                % Else
                        JUMP(link1,link2)=0;                                                                            % Set jump==0
                        INDEX_PREV(link1,link2) = index;                                                                % Update index
                        out = tau_new;                                                                                  % The avoidance torque will be only tau_new
                    end

                    tau = tau + out;                                                                                    % Sum all the avoidance torque for each couple of links in collision
                end
            end
         end
    end

    output = [tau;reshape(JUMP,144,1);reshape(T0,144,1);reshape(INDEX_PREV,144,1)];

% Uncomment if you want the see the behavior before the 'Jumps problem' fix
%     d_min = 10;
%     tau = zeros(12,1);
%     p_a1 = zeros(3,1);
%     p_a2 = zeros(3,1);
%     p_b1 = zeros(3,1);
%     p_b2 = zeros(3,1);
%     p_ac = zeros(3,1);
%     p_bc = zeros(3,1);
%     link1 = 0;
%     link2 = 0;
%     for i=1:n_links_collision
%         for j=i:n_links_collision
%             if CM(i,j)==1                                                                                       % If the two links can collide
%                 p_a1_tmp = reshape(p(i,1,:),3,1);                                                                       % Retrieve the extremities of the two links
%                 p_a2_tmp = reshape(p(i,2,:),3,1);
%                 p_b1_tmp = reshape(p(j,1,:),3,1);
%                 p_b2_tmp = reshape(p(j,2,:),3,1);
%                 [d_min_tmp, p_ac_tmp, p_bc_tmp, index_tmp] = dmin_spatial_f(p_a1_tmp, p_a2_tmp, p_b1_tmp, p_b2_tmp);                                    % Compute the minimum distance between them
%                 if check_collision_f(d_min_tmp,d_0, d_start)==1                                                             % If they are in the repulsion zone
%                     if d_min_tmp < d_min
%                         p_a1 = p_a1_tmp;
%                         p_a2 = p_a2_tmp;
%                         p_b1 = p_b1_tmp;
%                         p_b2 = p_b2_tmp;
%                         d_min = d_min_tmp;
%                         p_ac = p_ac_tmp;
%                         p_bc = p_bc_tmp;
%                         index = index_tmp;
%                         link1 = i;
%                         link2 = j;
%                     end
%                end
%             end
%          end
%     end
%     
%     if d_min~=10
%         [f_ac, f_bc] = potential_f(p_ac, p_bc, d_min, d_0, d_start, h_type, k_linear, k1_exp, k2_exp);      % Compute the repulsion force
%         [J_a1, J_a2] = Jvertices_spatial_f(q, link1, offA, offB, L1, L2, L3, D1, D2);                       % Compute the Jacobian of the extremities of link 1
%         [J_b1, J_b2] = Jvertices_spatial_f(q, link2, offA, offB, L1, L2, L3, D1, D2);                       % Compute the Jacobian of the extremities of link 1     
%         [Jt_ac, Jt_bc] = Jt_spatial_f(p_a1, p_a2, p_b1, p_b2, index);                                       % Compute the Jacobian of the collision points on link 1 and 2
%         J_ac = Jt_ac * [J_a1; J_a2; J_b1; J_b2];                                                            % Compute J_ac
%         J_bc = Jt_bc * [J_a1; J_a2; J_b1; J_b2];                                                            % Compute J_bc
%         tau = [J_ac' J_bc']*[f_ac; f_bc];                                                                   % Compute the avoidance torque
%     end
%     
%     JUMP = reshape(in(25:168),144,1);
%     T0 = reshape(in(169:312),144,1);
%     INDEX_PREV = reshape(in(313:456),144,1);
%     output = [tau;JUMP;T0;INDEX_PREV];

            
end

