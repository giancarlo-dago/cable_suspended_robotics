function tau = recursive_invdyn_class_f(theta, dtheta, ddtheta, robot)

    [A,robot] = compute_A_axis(robot);
    [M_mutual,robot] = compute_M_mutual(robot);
    [T_mutual,robot] = compute_T_mutual(robot,theta);
    [V, Vd, robot] = forward_recursion(robot, dtheta, ddtheta);
    [F, tau, robot] = backward_recursion(robot);
    
%     disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
%     A
%     disp('------------------------------------------------------')
%     M_mutual
%     disp('------------------------------------------------------')
%     T_mutual
%     disp('------------------------------------------------------')
%     G
%     disp('------------------------------------------------------')
%     V
%     disp('------------------------------------------------------')
%     Vd
%     disp('------------------------------------------------------')
%     F
%     disp('------------------------------------------------------')
%     tau
%     disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')

end


