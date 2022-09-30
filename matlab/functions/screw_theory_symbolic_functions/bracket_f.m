function S_bracket = bracket_f(S)

    % This function computes the bracket operator for a screw axis
    
    omega = S(1:3);
    v = S(4:6);
    
    S_bracket = [skew_f(omega) v;
                  zeros(1,3)   0 ];

end

