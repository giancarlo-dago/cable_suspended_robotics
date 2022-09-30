function Js = diffkin_f(theta, N_JOINTS, S)

    Js = zeros(6,size(S,2));

    for i=1:N_JOINTS
        if i==1
            Js_i = S(:,i);
        else
            T = eye(4);
            for j=1:i-1
                T = T * expm(bracket_f(S(:,j))*theta(j));                
            end
            Ad = Ad_f(T);
            Js_i = Ad * S(:,i);
        end
        Js(:,i) = Js_i;
    end

end

