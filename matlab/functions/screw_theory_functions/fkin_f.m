function T = fkin_f(theta, N_JOINTS, M, S)

    T = M;
    for i=N_JOINTS:-1:1
       T = (expm(bracket_f(S(:,i))*theta(i))) * T; 
    end

end