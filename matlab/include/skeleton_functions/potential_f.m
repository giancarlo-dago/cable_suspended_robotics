function [f_ac, f_bc] = potential_f(p_ac, p_bc, d_min, d_0, d_start, h_type, k_linear, k1_exp, k2_exp)

    % h function
    h_linear = k_linear*(d_0+d_start-d_min);
    h_exp = k1_exp*(exp(k2_exp*(d_0+d_start-d_min))-1);

    if h_type==0
       h = h_linear; 
    else
       h = h_exp;
    end
    
    f_ac = (h/d_min)*(p_ac-p_bc);
    f_bc = -f_ac;

end
