function [damping_ac, damping_bc] = damping_terms_f(qd, J_ac, J_bc, Da, Db)

    damping_ac = zeros(length(qd),1);
    damping_bc = zeros(length(qd),1);
    
    q_dot = qd(1:12);
    
    p_ac_dot = J_ac*q_dot;
    p_bc_dot = J_bc*q_dot;
    
    damping_ac = Da*p_ac_dot;
    damping_bc = Db*p_bc_dot;

end

