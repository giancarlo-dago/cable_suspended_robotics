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
    [q, q_dot] = read_feedback(receive(joint_state_sub,10));
    q_passive = q(1:4);
    q_active = q(5:12);
    q_dot_passive = q_dot(1:4);
    q_dot_active = q_dot(5:12);

    Q(:,i) = q;
    QD(:,i) = q_dot;
    pause_gazebo = call(pause_client,pause_req);
        
    %% Computation forward kinematics
    [pe_left,pe_right,o_left,o_right] = forward_kinematics_f(q,info);
    PE_left(:,i) = pe_left;
    PE_right(:,i) = pe_right;    
    
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
    Fv = diag([friction(1), friction(2), friction(3), friction(4), friction(5), friction(6), friction(7), friction(8), friction(9), friction(10) friction(11), friction(12)]);
    n = recursive_invdyn_tree_f(q, q_dot, zeros(1,n_joints), g, info) + Fv*q_dot;
   
    % Partial Feedback Linearization
    Bm = B(5:12,5:12);
    Bmb = B(5:12,1:4);
    Bbm = B(1:4,5:12);
    Bb = B(1:4,1:4);
    nb = n(1:4);
    nm = n(5:12);

    B_tilde = (Bm-Bmb*inv(Bb)*Bbm);
    n_tilde = nm - Bmb*inv(Bb)*nb;

    tau = B_tilde*y + n_tilde;
    TAU(:,i) = tau;
    
    %% Publishing the commands
    pub_arms_control(J_pub, tau);
    
    unpause_gazebo = call(unpause_client,unpause_req);
    pause(Ts);

end

pause_gazebo = call(pause_client,pause_req);

%% Plot
time = 0:Ts:Duration-Ts;
figure(); 
plot(time,Q(1:2,:)); title('Upper passive joint'); xlabel('time [s]'); ylabel('q_{passive,1} [rad]'); grid; legend('alpha_x','alpha_y')
figure(); 
plot(time,Q(3:4,:)); title('Lower passive joint'); xlabel('time [s]'); ylabel('q_{passive,2} [rad]'); grid; legend('beta_x','beta_y')
figure(); 
plot(time,Q(5:12,:)); title('q'); xlabel('time [s]'); ylabel('q [rad]'); grid; legend('q_{left}_1','q_{left}_2','q_{left}_3','q_{left}_4','q_{right}_1','q_{right}_2','q_{right}_3','q_{right}_4')
figure(); 
plot(time,QD(5:12,:)); title('qd'); xlabel('time [s]'); ylabel('qd [rad/s]'); grid; legend('dq_{left}_1','dq_{left}_2','dq_{left}_3','dq_{left}_4','dq_{right}_1','dq_{right}_2','dq_{right}_3','dq_{right}_4')
figure(); 
plot(time,E); title('e'); xlabel('time [s]'); ylabel('e [rad]'); grid; legend('e_{left}_1','e_{left}_2','e_{left}_3','e_{left}_4','e_{right}_1','e_{right}_2','e_{right}_3','e_{right}_4')
figure(); 
plot(time,E_DOT); title('ed'); xlabel('time [s]'); ylabel('ed [rad/s]'); grid; legend('de_{left}_1','de_{left}_2','de_{left}_3','de_{left}_4','de_{right}_1','de_{right}_2','de_{right}_3','de_{right}_4')
figure(); 
plot(time,TAU); title('tau'); xlabel('time [s]'); ylabel('tau [Nm]'); grid; legend('tau_{left}_1','tau_{left}_2','tau_{left}_3','tau_{left}_4','tau_{right}_1','tau_{right}_2','tau_{right}_3','tau_{right}_4')

% End-effector position plot (2D)
figure();
subplot(3,1,1); plot(time,PE_left(1,:)); xlabel('time [s]'); ylabel('left ee x position [m]'); grid;
subplot(3,1,2); plot(time,PE_left(2,:)); xlabel('time [s]'); ylabel('left ee y position [m]'); grid;
subplot(3,1,3); plot(time,PE_left(3,:)); xlabel('time [s]'); ylabel('left ee z position [m]'); grid;
figure();
subplot(3,1,1); plot(time,PE_right(1,:)); xlabel('time [s]'); ylabel('right ee x position [m]'); grid;
subplot(3,1,2); plot(time,PE_right(2,:)); xlabel('time [s]'); ylabel('right ee y position [m]'); grid;
subplot(3,1,3); plot(time,PE_right(3,:)); xlabel('time [s]'); ylabel('right ee z position [m]'); grid;

% End-effector position plot (3D)
figure(); 
plot3(PE_left(1,:),PE_left(2,:),PE_left(3,:)); title('Left End-effector'); xlabel('x'); ylabel('y'); zlabel('z'); grid;
figure(); 
plot3(PE_right(1,:),PE_right(2,:),PE_right(3,:)); title('Right End-effector'); xlabel('x'); ylabel('y'); zlabel('z'); grid;
