function Ad = Ad_f(T)
    
    % This function computes the Adjoint of a Transformation matrix
    
    R = T(1:3,1:3);
    p = T(1:3,4);

    Ad = [    R       zeros(3,3);
          skew_f(p)*R       R     ];

end