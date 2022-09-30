classdef build_robot
    
    properties
         n_links int8
         n_ee int8
         n_joints int8
         n_frames int8
         frames frame
         joints joint
         gravity (3,1) double
    end
    
    methods    
        
        function obj = build_robot(n_links,n_ee,n_joints,frames,joints,gravity)
            obj.n_links = n_links;
            obj.n_ee = n_ee;
            obj.n_frames = n_links + n_ee;
            obj.n_joints = n_joints;
            obj.joints = joints;
            obj.frames = frames;
            obj.gravity = gravity;
            obj = sort(obj);
        end
        
        function f = search_frame(obj,index)
            for i=1:get_n_frames(obj)
                if (obj.frames(i).index == index)
                   f = obj.frames(i);
                end
            end
        end
        
        function obj = sort(obj)
            for i=1:get_n_frames(obj)
                new_frames(i) = search_frame(obj,i);
            end    
            obj.frames = new_frames;
        end
        
        function ees = select_ee_frames(obj)
            j = 1;
            for i=1:get_n_frames(obj)
                f = obj.frames(i);
                if get_frame_type(f) == 'ee_frame'
                    ees(j) = f;
                    j = j+1;
                end
            end
        end
        
        function links = select_link_frames(obj)
            j = 1;
            for i=1:get_n_frames(obj)
                f = obj.frames(i);
                if get_frame_type(f) == 'link_frame'
                    links(j) = f;
                    j = j+1;
                end
            end
        end
               
        function flag = is_ee(obj,f)
            if (length(f)>1)
                for i=1:length(f)
                    if get_frame_type(f(i)) ~= 'ee_frame'
                        flag = false;
                        break;
                    else
                        flag = true;
                    end
                end
            else
                if get_frame_type(f) == 'ee_frame'
                    flag = true;
                else
                    flag = false;
                end
            end            
        end
        
        function flag = next_all_explored(obj,f)
            next = get_next(f);
            flag = true;
            i = 1;
            while i<=length(next) && flag==true
                if get_explored(obj.frames(next(i)))==true
                    flag = true;
                else
                    flag = false;
                end
                i = i+1;
            end       
        end
        
        function final_frame = last_frame(obj,f)
            if is_ee(obj,f)==true
                final_frame = f;
            end
            while is_ee(obj,f) == false
                for i=1:length(f)
                    next = get_next(f(i));
                    if (length(next)>1)
                        for j=1:length(next)
                            f(j) = obj.frames(next(j));
                        end
                    else
                        f(i) = obj.frames(next);
                    end
                end
            end
            final_frame = f;
        end
        
        function flag = all_explored(obj)
            flag = true;
            i=1;
            while (i<=get_n_frames(obj) && flag==true)
                f = get_frame(obj,i);
                if (get_explored(f)==false)
                    flag = false;      
                end
                i=i+1;
            end
        end
        
        
        % Get functions
        function n_links = get_n_links(obj)
            n_links = obj.n_links;
        end
        function n_joints = get_n_joints(obj)
            n_joints = obj.n_joints;
        end
        function n_frames = get_n_frames(obj)
            n_frames = obj.n_frames;
        end
        function f = get_frame(obj,index)
            f = obj.frames(index);     
        end
        function j = get_joint(obj,index)
            j = obj.joints(index);     
        end
        function g = get_gravity(obj)
            g = obj.gravity;     
        end
        
        
        
        % Set functions
        function obj = set_explored(obj,f)
            index = get_index(f);
            obj.frames(index) = set_explored(obj.frames(index));      
        end
        function obj = set_A_axis(obj,f,A_axis)
            index = get_index(f);
            obj.frames(index) = set_A_axis(obj.frames(index),A_axis);      
        end 
        function obj = set_M_mutual(obj,f,M_mutual)
            index = get_index(f);
            obj.frames(index) = set_M_mutual(obj.frames(index),M_mutual);      
        end 
        function obj = set_T_mutual(obj,f,T_mutual)
            index = get_index(f);
            obj.frames(index) = set_T_mutual(obj.frames(index),T_mutual);      
        end 
        function obj = set_V(obj,f,V)
            index = get_index(f);
            obj.frames(index) = set_V(obj.frames(index),V);      
        end 
        function obj = set_Vd(obj,f,Vd)
            index = get_index(f);
            obj.frames(index) = set_Vd(obj.frames(index),Vd);      
        end 
        function obj = set_F_own(obj,f,F_own)
            index = get_index(f);
            obj.frames(index) = set_F_own(obj.frames(index),F_own);      
        end 
        function obj = set_F_ext(obj,f,F_ext)
            index = get_index(f);
            obj.frames(index) = set_F_ext(obj.frames(index),F_ext);      
        end 
        function obj = set_F(obj,f,F)
            index = get_index(f);
            obj.frames(index) = set_F(obj.frames(index),F);      
        end 
        function obj = set_tau(obj,f,tau)
            index = get_index(f);
            obj.frames(index) = set_tau(obj.frames(index),tau);      
        end 


        
        
        
        
        % Preliminary computations
        function [A,obj] = compute_A_axis(obj)
            for i = 1:get_n_links(obj)
                f = get_frame(obj,i);
                M_bi = get_M_bi(f);
                previous_joint = get_previous_joint(f);
                S = get_S_axis(get_joint(obj,previous_joint));
                result = Ad_f(inv(M_bi)) * S;                         % Computation of screw vector of joint i referred to frame {i}
                A(:,i) = result;
                obj = set_A_axis(obj,f,result);
            end
        end
        
        function [M_mutual, obj] = compute_M_mutual(obj)
            for i=1:get_n_frames(obj)
                f = get_frame(obj,i);
                M_bi = get_M_bi(f);
                if i == 1                                                                   % Computation M_{i,i-1}: tranformation matrix at rest position
                    result = inv(M_bi);
                else
                    previous = get_previous(f);
                    f_prev = get_frame(obj,previous);
                    M_bi_prev = get_M_bi(f_prev);
                    result = M_bi\M_bi_prev;
                end 
                M_mutual(:,:,i) = result;
                obj = set_M_mutual(obj,f,result);
            end
        end
        
        function [T_mutual, obj] = compute_T_mutual(obj,theta)
            for i=1:get_n_frames(obj)
                f = get_frame(obj,i);
                A = get_A_axis(f);
                M_mutual = get_M_mutual(f);
                previous_joint = get_previous_joint(f);
                if get_frame_type(f) == 'link_frame'
                    th = theta(previous_joint);
                    result = expm(-bracket_f(A)*th) * M_mutual;      % Computation T_{i,i-1}: tranformation matrix in a general configuration 
                    T_mutual(:,:,i) = result;
                    obj = set_T_mutual(obj,f,result);
                else
                    T_mutual(:,:,i) = M_mutual;
                    obj = set_T_mutual(obj,f,M_mutual);
                end
            end
        end
        
        function [V,Vd,obj] = forward_recursion(obj,dtheta,ddtheta)
            V_0 = zeros(6,1);                                                               % Starting twist
            Vd_0 = [zeros(3,1); -get_gravity(obj)];                                         % Starting accelerations
            for i=1:get_n_links(obj)
                f = get_frame(obj,i);
                previous = get_previous(f);
                T_mutual = get_T_mutual(f);
                A = get_A_axis(f);
                previous_joint = get_previous_joint(f);
                dth = dtheta(previous_joint);
                ddth = ddtheta(previous_joint);
                if i == 1
                    V_prev = V_0;
                    Vd_prev = Vd_0;
                else
                    f_prev = get_frame(obj,previous);
                    V_prev = get_V(f_prev);
                    Vd_prev = get_Vd(f_prev);
                end
                V_res = Ad_f(T_mutual)*V_prev + A*dth;                   % Computing the twist for each link by a recursive formula
                Vd_res = ad_f_(V_res)*(A*dth) + ...                     % Computing the acceleration (derivative of twist) for each link by a recurisive formula
                          Ad_f(T_mutual)*Vd_prev + A*ddth;
                      
                V(:,i) = V_res;
                Vd(:,i) = Vd_res;
                obj = set_V(obj,f,V_res);
                obj = set_Vd(obj,f,Vd_res);
            end
        end
        
        function [F_own, obj] = compute_F_own(obj,f)
            V = get_V(f);
            Vd = get_Vd(f);
            G = get_G(f);
            F_own = G*Vd - ad_f_(V)'*(G*V);
            obj = set_F_own(obj,f,F_own);
        end
        
        function [F_ext, obj] = compute_F_ext(obj,f)
            F_ext = zeros(6,1);
            next_index = get_next(f);
            for i=1:length(next_index)
                f_next = get_frame(obj,next_index(i));
                T_mutual_next = get_T_mutual(f_next);
                F_next = get_F(f_next);
                F_ext = F_ext + Ad_f(T_mutual_next)'*F_next;
                obj = set_F_ext(obj,f,F_ext);
            end
        end
        
        function [F_vec,tau,obj] = backward_recursion(obj)           
%             Scelgo come frame iniziale uno degli ee e la metto come explored
%             1. Finchè non sono rimasti piu link da esplorare
%             2. Prendo l'ultima frame in memoria e mi calcolo la precedente lungo la catena
%             3. Della precedente mi calcolo la F_own
%             4. Se tutte le frame successive sono stati esplorate 
%                     allora mi calcolo anche la F_ext e posso tornare al punto 1
%                Se invece tra i successivi c'è almeno un link che non è stato esplorato 
%                     salvati l'indice del link che non è stato ancora esplorato
%                     calcola l'ultimo frame della catena di questo link e salvalo in memoria
            F_vec = zeros(6,get_n_links(obj));
            tau = zeros(1,get_n_links(obj));
            ees = select_ee_frames(obj);                                        % From the frame vector select only the ee frames
            current_frame = ees(1);                                             % Pick the first of the ee frames
            obj = set_explored(obj,current_frame);                              % Mark the ee frame as 'explored'
            while (all_explored(obj)==false)                                    % While there are no other frames to explore
                prev_frame_index = get_previous(current_frame);                 % Compute the index of the previous frame
                current_frame = get_frame(obj,prev_frame_index);                % store the previous frame, that will be our new current frame
                [F_own, obj] = compute_F_own(obj,current_frame);                % Compute F_own and save the value in the current_frame object       
                if (next_all_explored(obj,current_frame) == true)
                    [F_ext, obj] = compute_F_ext(obj,current_frame);            % Compute F_ext and save the value in the frame object
                    F = F_own + F_ext;                                          % Compute F 
                    current_index = get_index(current_frame);
                    F_vec(:,current_index) = F;
                    A = get_A_axis(current_frame);
                    tau(current_index) = F'*A;
                    obj = set_F(obj,current_frame,F);                           % Save the value in the frame object
                    obj = set_explored(obj,current_frame);                      % Mark the current frame as 'explored'
                    obj = set_tau(obj,current_frame,tau);
                else
                    i=1;
                    done = false;
                    next_frame_index = get_next(current_frame);
                    while (i<=length(next_frame_index) && done==false)
                        f = get_frame(obj,next_frame_index(i));
                        if (get_explored(f)==false)
                            next_frame = f;
                            done = true;
                        end
                        i=i+1;
                    end
                    current_frame = last_frame(obj,next_frame);                 % Compute the ee frame of the chain yet to be explored
                    obj = set_explored(obj,current_frame);                              % Mark the ee frame as 'explored'
                end
            end
        end        
    end
    
end

