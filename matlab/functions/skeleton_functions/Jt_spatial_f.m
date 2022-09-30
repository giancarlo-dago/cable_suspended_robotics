function [Jt_ac, Jt_bc] = Jt_spatial_f(p_a1, p_a2, p_b1, p_b2, index)
        
    Jt_11 = Jt11_spatial_f(p_a1, p_a2, p_b1, p_b2, index);
    Jt_12 = Jt12_spatial_f(p_a1, p_a2, p_b1, p_b2, index);
    Jt_13 = Jt13_spatial_f(p_a1, p_a2, p_b1, p_b2, index);
    Jt_14 = Jt14_spatial_f(p_a1, p_a2, p_b1, p_b2, index);
    Jt_21 = Jt21_spatial_f(p_a1, p_a2, p_b1, p_b2, index);
    Jt_22 = Jt22_spatial_f(p_a1, p_a2, p_b1, p_b2, index);
    Jt_23 = Jt23_spatial_f(p_a1, p_a2, p_b1, p_b2, index);
    Jt_24 = Jt24_spatial_f(p_a1, p_a2, p_b1, p_b2, index);

    Jt_ac = [Jt_11 Jt_12 Jt_13 Jt_14];
    Jt_bc = [Jt_21 Jt_22 Jt_23 Jt_24];
        
end

