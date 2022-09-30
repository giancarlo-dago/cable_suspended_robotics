function tau = recursive_invdyn_tree_f(theta, dtheta, ddtheta, g, info)

    n_joints = info.n_joints;
    n_links = info.n_links;
    n_ee = info.n_ee;
    n_frames = n_links+n_ee;
    S = info.S;
    M_bi = info.M_bi;
    Inertia = info.Inertia;
    mass = info.mass;
    inertial_disp = info.inertial_disp;
    F = info.F;

    % Preallocation
    A = zeros(6,n_links);
    M_mutual = zeros(4,4,n_frames);
    T_mutual = zeros(4,4,n_frames);
    G = zeros(6,6,n_links);
    V = zeros(6,n_links);
    Vd = zeros(6,n_links);
    F_own = zeros(6,n_links);
    F_ext = zeros(6,n_links);
    tau = zeros(n_joints,1);
        
    % Definitions
    link_frame_type = 1;
    ee_frame_type = 0;
    unexplored = 0;
    explored = 1;
    
    % Preliminary computations
    for i=1:n_frames
        if (info.frame_type(i) == link_frame_type)
            prev_joint_index = info.previous_joint_index(i);
            current_S = S(:,prev_joint_index);
            A(:,i) = Ad_f(inv(M_bi(:,:,i))) * current_S;                                   % Computation of screw vector of joint i referred to frame {i}
        end
    end

    for i=1:n_frames
        Mbi = M_bi(:,:,i);
        if i == 1                                                                   % Computation M_{i,i-1}: tranformation matrix at rest position
            result = inv(Mbi);
        else
            prev_index = info.previous_frame_index(i);
            M_bi_prev = M_bi(:,:,prev_index);
            result = Mbi\M_bi_prev;
        end 
        M_mutual(:,:,i) = result;
        if (info.frame_type(i) == link_frame_type)
            prev_joint_index = info.previous_joint_index(i);
            th = theta(prev_joint_index);
        	T_mutual(:,:,i) = expm(-bracket_f(A(:,i))*th) * M_mutual(:,:,i);      % Computation T_{i,i-1}: tranformation matrix in a general configuration 
        else
            T_mutual(:,:,i) = M_mutual(:,:,i);
        end
    end
    
    for i=1:n_frames
        if (info.frame_type(i) == link_frame_type)
            G(:,:,i) = G_f(mass(i),Inertia(:,:,i),inertial_disp(i,:));                  % Referring the spatial inertia matrix in the i-th frame
        end
    end
    
    
    % Forward recursion
    V_0 = zeros(6,1);                                                % Starting twist
    Vd_0 = [zeros(3,1); -g];                                         % Starting accelerations
    for i=1:n_frames
        if (info.frame_type(i) == link_frame_type)
            Tmutual = T_mutual(:,:,i);
            A_axis = A(:,i);
            prev_joint_index = info.previous_joint_index(i);
            prev_frame_index = info.previous_frame_index(i);
            dth = dtheta(prev_joint_index);
            ddth = ddtheta(prev_joint_index);
            if i == 1
                V_prev = V_0;
                Vd_prev = Vd_0;
            else
                V_prev = V(:,prev_frame_index);
                Vd_prev = Vd(:,prev_frame_index);
            end
            V_res = Ad_f(Tmutual)*V_prev + A_axis*dth;                   % Computing the twist for each link by a recursive formula
            Vd_res = ad_f_(V_res)*(A_axis*dth) + ...                     % Computing the acceleration (derivative of twist) for each link by a recurisive formula
                      Ad_f(Tmutual)*Vd_prev + A_axis*ddth;
            V(:,i) = V_res;
            Vd(:,i) = Vd_res;
        end
    end
    
    
    
    
    % Backward recursion
    i=1;
    found = false;
    current_index = 0;
    while (i<=n_frames && found==false) 
        if (info.frame_type(i) == ee_frame_type)                         % From the frame vector select only the ee frames
            found = true;                                                % Pick the first of the ee frames
            current_index = i;
        end
        i = i+1;
    end
    info.explored(current_index) = explored;                             % Mark the ee frame as 'explored'
    

    j=1;
    all_explored = false;
    while (all_explored == false && j<n_frames)                                                             % While there are no other frames to explore
        
        all_explored = true;
        h=1;
        while (h<=n_frames && all_explored==true)       
            if(info.explored(h) == unexplored)
                all_explored = false;
            end
            h = h+1;
        end
        current_index = info.previous_frame_index(current_index);        % Compute the index of the previous frame, that will be our new current frame
        
        % Compute F_own
        V_curr = V(:,current_index);
        Vd_curr = Vd(:,current_index);
        G_curr = G(:,:,current_index);
        F_own(:,current_index) = G_curr*Vd_curr - ...                       % Compute F_own  
                                 ad_f_(V_curr)'*(G_curr*V_curr); 

        % Retrieve the info of the next frames
        if (info.next_frame_type(current_index) >= 1)
            next_index = zeros(1,2);
            nxt = info.next_frame_index(current_index);
            x = nxt;
            if (info.next_frame_type(current_index) == 1)
                d = 10;
            else
                d = 100;
            end
            r = mod(x,d);
            y = (x - r) / d;
            next_index(1,2) = r;
            next_index(1,1) = y;
        else
           next_index = info.next_frame_index(current_index); 
        end
               
        
        % Check if they are all explored
        next_all_explored = true;
        i = 1;
        while i<=length(next_index) && next_all_explored==true
            if info.explored(next_index(1,i))==true
                next_all_explored = true;
            else
                next_all_explored = false;
            end
            i = i+1;
        end
            
        if (next_all_explored == true)                                                          % If they are all explored
            % Compute F_ext 
            for i=1:length(next_index)
                Tmutual_next = T_mutual(:,:,next_index(1,i));
                F_next = F(:,next_index(1,i));
                F_ext(:,current_index) = F_ext(:,current_index) + Ad_f(Tmutual_next)'*F_next;  
            end
            % Compute F
            F(:,current_index) = F_own(:,current_index) + F_ext(:,current_index);               % Compute F 
            % Compute Tau
            tau(current_index) = F(:,current_index)'*A(:,current_index);
            % Set the current frame as explored
            info.explored(current_index) = explored;
        else                                                                                    % If they are not all explored
            i=1;
            done = false;

            while (i<=length(next_index) && done==false)                                        % Among the next links, find the first unexplored
                if (info.explored(next_index(1,i))==false)  
                    done = true;                                                            % Once the first next unexplored link is found
                    % Searching for the final frame 
                    temp_index = next_index(1,i); 
                    is_ee = false;
                    while (is_ee == false)
                        if (info.frame_type(temp_index) == ee_frame_type)                              % If the link is already an ee frame
                           is_ee = true;
                           current_index = temp_index;                                      % Save its index
                        else
                           temp_index = info.next_frame_index(temp_index);               % Else save the index of the following link
                        end
                        info.explored(current_index) = explored;
                    end
                end
                i=i+1;
            end
        end
            
        j = j+1;
    end
    
%     disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
%     A
%     disp('------------------------------------------------------')
%     M_mutual
%     disp('------------------------------------------------------')
%     T_mutual
%     disp('------------------------------------------------------')
%     G
%     disp('------------------------------------------------------')
%     V
%     disp('------------------------------------------------------')
%     Vd
%     disp('------------------------------------------------------')
%     F
%     disp('------------------------------------------------------')
%     tau
%     disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')

end

