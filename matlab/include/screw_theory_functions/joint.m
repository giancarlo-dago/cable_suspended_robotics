classdef joint
    
    properties
        index double
        omega (3,1) double
        q (3,1) double
        S_axis (6,1) double
    end
    
    methods    
        
        function obj = joint(ind,om,q)
            obj.index = ind;
            obj.omega = om;
            obj.q = q;
            
            obj = compute_S_axis(obj);
        end

        function obj = compute_S_axis(obj)
            v = cross(-obj.omega,obj.q);
            S = [obj.omega; v];
            obj.S_axis = S;
        end
        
        function S_axis = get_S_axis(obj)
            S_axis = obj.S_axis;
        end
        
    end
end