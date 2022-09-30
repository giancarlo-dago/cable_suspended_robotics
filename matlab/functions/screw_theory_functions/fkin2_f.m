function T = fkin2_f(theta, info)

    N_JOINTS = info.n_joints;
    S = info.S;
    M_bi = info.M_bi;
    
    
    T = M;
    for i=N_JOINTS:-1:1
       T = (expm(bracket_f(S(:,i))*theta(i))) * T; 
    end

end