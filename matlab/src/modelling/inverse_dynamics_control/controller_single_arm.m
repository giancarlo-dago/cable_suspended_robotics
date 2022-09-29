%% -----------------------ROS INTERCONNECTION -----------------------------
rosshutdown
setenv('ROS_MASTER_URI','http://localhost:11311')
setenv('ROS_IP','localhost')
rosinit

% Pause and Unpause Gazebo Physics Engine Clients
pause_client = rossvcclient('/gazebo/pause_physics');
unpause_client = rossvcclient('/gazebo/unpause_physics');
pause_req = rosmessage(pause_client);
unpause_req = rosmessage(unpause_client);

% Definition of Publishers and Subscribers 
joint_state_sub = rossubscriber('/licasa1/joint_states');

J_pub(1) = rospublisher('/licasa1/licasa1_leftarm_1_effort_eff_controller/command','std_msgs/Float64');
J_pub(2) = rospublisher('/licasa1/licasa1_leftarm_2_effort_eff_controller/command','std_msgs/Float64');
J_pub(3) = rospublisher('/licasa1/licasa1_leftarm_3_effort_eff_controller/command','std_msgs/Float64');
J_pub(4) = rospublisher('/licasa1/licasa1_leftarm_4_effort_eff_controller/command','std_msgs/Float64');
J_pub(5) = rospublisher('/licasa1/licasa1_rightarm_1_effort_eff_controller/command','std_msgs/Float64');
J_pub(6) = rospublisher('/licasa1/licasa1_rightarm_2_effort_eff_controller/command','std_msgs/Float64');
J_pub(7) = rospublisher('/licasa1/licasa1_rightarm_3_effort_eff_controller/command','std_msgs/Float64');
J_pub(8) = rospublisher('/licasa1/licasa1_rightarm_4_effort_eff_controller/command','std_msgs/Float64');

%% ------------------- Inverse Dynamics control ----------------------------

B = zeros(n_joints,n_joints);
unpause_gazebo = call(unpause_client,unpause_req);

for i=1:(Duration/Ts)
        
    %% Sensing (arm joints state)
    [full_q, full_q_dot] = read_feedback(receive(joint_state_sub,10));
    q = full_q(5:8);
    q_dot = full_q_dot(5:8);
    Q(:,i) = q;
    QD(:,i) = q_dot;
    pause_gazebo = call(pause_client,pause_req);
    
    %% Computation auxiliar input
    e = q_ref - q;
    E(:,i) = e;
    e_dot = q_dot_ref - q_dot;
    E_DOT(:,i) = e_dot;
    y = Kp*e + Kd*e_dot + q_ddot_ref;
    
    %% Inverse dynamics control        
%     for j=1:n_joints
%         fake_acc = zeros(1,n_joints);
%         fake_acc(j) = 1;
%         B(:,j) = recursive_invdyn_f(q, zeros(1,n_joints), fake_acc, zeros(3,1), info, F_ee);
%     end
% %     
%     Fv = diag([fv1 fv2]);
%     n = recursive_invdyn_f(q, q_dot, zeros(1,n_joints), g, info, F_ee) + Fv*q_dot;
%     n = recursive_invdyn_f(q, q_dot, zeros(1,n_joints), g, info, F_ee);
     
%     tau = B*y + n;

    tau = recursive_invdyn_f(q, q_dot, y, g, info, F_ee);
    TAU(:,i) = tau;
    
    %% Publishing the commands
    pub_arms_control(J_pub, [tau; 0; 0; 0; 0]);
    
    unpause_gazebo = call(unpause_client,unpause_req);
    pause(Ts);

end

pause_gazebo = call(pause_client,pause_req);

%% Plot
time = 0:Ts:Duration-Ts;
figure(); 
plot(time,Q); title('q'); xlabel('time [s]'); ylabel('q [rad]'); grid; legend('q_1','q_2','q_3','q_4')
figure(); 
plot(time,QD); title('qd'); xlabel('time [s]'); ylabel('qd [rad/s]'); grid; legend('q_{dot1}','q_{dot2}','q_{dot3}','q_{dot4}') 
figure(); 
plot(time,E); title('e'); xlabel('time [s]'); ylabel('e [rad]'); grid; legend('e_1','e_2','e_3','e_4')
figure(); 
plot(time,E_DOT); title('ed'); xlabel('time [s]'); ylabel('ed [rad/s]'); grid; legend('e_dot1','e_dot2','e_dot3','e_dot4') 
figure(); 
plot(time,TAU); title('tau'); xlabel('time [s]'); ylabel('tau [Nm]'); grid; legend('tau_1','tau_2','tau_3','tau_4')



