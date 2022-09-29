function [pe_left,pe_right,o_left,o_right] = forward_kinematics_f(theta,info)

    S = info.S;
    M_bi = info.M_bi;
        
    S_left = S(:,1:8);
    S_right = [S(:,1:4) S(:,9:12)];
    Me_left = M_bi(:,:,13);
    Me_right = M_bi(:,:,14); 
      
    % Kinematics (position of the end effector)
    Te_left = fkin_f(theta(1:8), 8, Me_left, S_left);
    Te_right = fkin_f([theta(1:4); theta(9:12)], 8, Me_right, S_right);

    pe_left = Te_left(1:3,4);
    pe_right = Te_right(1:3,4);
    
    o_left = rotm2eul(Te_left(1:3,1:3));
    o_right = rotm2eul(Te_right(1:3,1:3));

end
