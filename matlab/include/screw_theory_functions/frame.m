classdef frame
    
    properties
        frame_type string
        index int8
        previous uint8
        next int8
        previous_joint int8
        explored logical = false;
        M_bi (4,4) double
        mass double
        Inertia 
        inertial_disp 
        G (6,6) double
        M_mutual (4,4) double
        T_mutual (4,4) double
        A_axis (6,1) double
        V (6,1) double
        Vd (6,1) double
        F_own
        F_ext
        F
        tau double
    end
    
    methods
        
        function obj = frame(type,ind,prev,nex,prevj,mbi,f,m,I,Idisp)
            obj.frame_type = type;
            obj.index = ind;
            obj.previous = prev;
            obj.next = nex;
            obj.previous_joint = prevj;
            obj.M_bi = mbi;
            obj.F = f;
            obj.mass = m;
            obj.Inertia = I;
            obj.inertial_disp = Idisp;
            
            if (strcmp(type,'link_frame'))
                obj = compute_G(obj);
            elseif (strcmp(type,'ee_frame'))
            end
        end
        
        % Get functions
        function type = get_frame_type(obj)
            type = obj.frame_type;
        end
        function index = get_index(obj)
            index = obj.index;
        end
        function previous = get_previous(obj)
            previous = obj.previous;
        end
        function next = get_next(obj)
            next = obj.next;
        end
        function previous_joint = get_previous_joint(obj)
            previous_joint = obj.previous_joint;
        end
        function ex = get_explored(obj)
            ex = obj.explored;
        end
        function M_bi = get_M_bi(obj)
            M_bi = obj.M_bi;
        end
        function M_mutual = get_M_mutual(obj)
            M_mutual = obj.M_mutual;
        end
        function T_mutual = get_T_mutual(obj)
            T_mutual = obj.T_mutual;
        end
        function A_axis = get_A_axis(obj)
            A_axis = obj.A_axis;
        end
        function V = get_V(obj)
            V = obj.V;
        end
        function Vd = get_Vd(obj)
            Vd = obj.Vd;
        end
        function F = get_F(obj)
            F = obj.F;
        end
        function tau = get_tau(obj)
            tau = obj.tau;
        end
        function G = get_G(obj)
            G = obj.G;
        end
        function F_own = get_F_own(obj)
            F_own = obj.F_own;
        end
        function F_ext = get_F_ext(obj)
            F_ext = obj.F_own;
        end
        
        % Set functions
        function obj = set_explored(obj)
            obj.explored = true;
        end
        function obj = set_A_axis(obj,A_axis)
            obj.A_axis = A_axis;
        end
        function obj = set_M_mutual(obj,M_mutual)
            obj.M_mutual = M_mutual;
        end
        function obj = set_T_mutual(obj,T_mutual)
            obj.T_mutual = T_mutual;
        end
        function obj = set_V(obj,V)
            obj.V = V;
        end
        function obj = set_Vd(obj,Vd)
            obj.Vd = Vd;
        end
        function obj = set_F(obj,F)
            obj.F = F;
        end
        function obj = set_tau(obj,tau)
            obj.tau = tau;
        end
        function obj = set_F_own(obj,F_own)
            obj.F_own = F_own;
        end
        function obj = set_F_ext(obj,F_ext)
            obj.F_ext = F_ext;
        end
    
        % Computation functions
        function obj = compute_G(obj)
            G = G_f(obj.mass,obj.Inertia,obj.inertial_disp);
            obj.G = G;
        end
                
    end
    
end