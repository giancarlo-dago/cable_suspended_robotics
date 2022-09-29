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
joint_state_sub = rossubscriber('/testrobot/joint_states');

J_pub(1) = rospublisher('/testrobot/left_arm_joint_effort_eff_controller/command','std_msgs/Float64');
J_pub(2) = rospublisher('/testrobot/right_arm_joint_effort_eff_controller/command','std_msgs/Float64');

%% ------------------- Inverse Dynamics control ----------------------------
B = zeros(n_joints,n_joints);
unpause_gazebo = call(unpause_client,unpause_req);

for i=1:(Duration/Ts)
        
    %% Sensing (arm joints state)
    [q, q_dot] = read_test_feedback(receive(joint_state_sub,10));
    q_passive = q(1:2);
    q_active = q(3:4);
    q_dot_passive = q_dot(1:2);
    q_dot_active = q_dot(3:4);

    Q(:,i) = q;
    QD(:,i) = q_dot;
    pause_gazebo = call(pause_client,pause_req);
            
    %% Computation auxiliar input
    e = q_ref - q_active;
    E(:,i) = e;
    e_dot = q_dot_ref - q_dot_active;
    E_DOT(:,i) = e_dot;
    y = Kp*e + Kd*e_dot + q_ddot_ref;
   
    %% Collocated Partial FBL

    % Computation B matrix
    for j=1:n_joints
        fake_acc = zeros(1,n_joints);
        fake_acc(j) = 1;
        B(:,j) = recursive_invdyn_tree_f(q, zeros(1,n_joints), fake_acc, zeros(3,1), info);
    end

    % Computation n vector
    Fv = diag([friction(1), friction(2), friction(3), friction(4)]);
    n = recursive_invdyn_tree_f(q, q_dot, zeros(1,n_joints), g, info) + Fv*q_dot;
   
    % Partial Feedback Linearization
    Bm = B(3:4,3:4);
    Bmb = B(3:4,1:2);
    Bbm = B(1:2,3:4);
    Bb = B(1:2,1:2);
    nb = n(1:2);
    nm = n(3:4);
    
    B_tilde = (Bm-Bmb*inv(Bb)*Bbm);
    n_tilde = nm - Bmb*inv(Bb)*nb;
    
    tau = B_tilde*y + n_tilde;
    TAU(:,i) = tau; 
    
    %% Publishing the commands
    pub_test_arms_control(J_pub, tau);
    
    unpause_gazebo = call(unpause_client,unpause_req);
    pause(Ts);

end

pause_gazebo = call(pause_client,pause_req);

%% Plot
time = 0:Ts:Duration-Ts;
figure(); 
plot(time,Q(1:2,:)); title('Passive joints'); xlabel('time [s]'); ylabel('q_{passive} [rad]'); grid; legend('alpha','beta')
figure(); 
plot(time,Q(3:4,:)); title('q'); xlabel('time [s]'); ylabel('q [rad]'); grid; legend('q_{left}_1','q_{right}_1')
figure();
plot(time,QD(3:4,:)); title('qd'); xlabel('time [s]'); ylabel('qd [rad/s]'); grid; legend('dq_{left}_1','dq_{right}_1')
figure(); 
plot(time,E); title('e'); xlabel('time [s]'); ylabel('e [rad]'); grid; legend('e_{left}_1','e_{right}_1')
figure(); 
plot(time,E_DOT); title('ed'); xlabel('time [s]'); ylabel('ed [rad/s]'); grid; legend('de_{left}_1','de_{right}_1')
figure(); 
plot(time,TAU); title('tau'); xlabel('time [s]'); ylabel('tau [Nm]'); grid; legend('tau_{left}_1','tau_{right}_1')
