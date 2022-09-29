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

B_A = zeros(n_joints,n_joints);
B_B = zeros(n_joints,n_joints);
unpause_gazebo = call(unpause_client,unpause_req);

for i=1:(Duration/Ts)
        
    %% Sensing (arm joints state)
    [q, q_dot] = read_feedback(receive(joint_state_sub,10));
    qA = q(6:9);
    qB = q(10:13);
    q_dotA = q_dot(6:9);
    q_dotB = q_dot(10:13);
    Q(:,i) = q;
    QD(:,i) = q_dot;
    pause_gazebo = call(pause_client,pause_req);
    
    %% Computation auxiliar input
    eA = q_refA - qA;
    E(1:4,i) = eA;
    e_dotA = q_dot_refA - q_dotA;
    E_DOT(1:4,i) = e_dotA;
    yA = Kp*eA + Kd*e_dotA + q_ddot_refA;
    
    eB = q_refB - qB;
    E(5:8,i) = eB;
    e_dotB = q_dot_refB - q_dotB;
    E_DOT(5:8,i) = e_dotB;
    yB = Kp*eB + Kd*e_dotB + q_ddot_refB;
    
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
%      
%     tau = B*y + n;

    tauA = recursive_invdyn_f(qA, q_dotA, yA, g, infoA, F_eeA);
    TAU(1:4,i) = tauA;
    tauB = recursive_invdyn_f(qB, q_dotB, yB, g, infoB, F_eeB);
    TAU_B(5:8,i) = tauB;
    
    %% Publishing the commands
    pub_arms_control(J_pub, [tauA; tauB]);
    
    unpause_gazebo = call(unpause_client,unpause_req);
    pause(Ts);

end

pause_gazebo = call(pause_client,pause_req);

%% Plot
time = 0:Ts:Duration-Ts;
figure(); 
plot(time,Q(3,:)); title('q'); xlabel('time [s]'); ylabel('q [rad]'); grid;
figure(); 
plot(time,QD(3,:)); title('qd'); xlabel('time [s]'); ylabel('qd [rad/s]'); grid;
figure(); 
plot(time,E); title('e'); xlabel('time [s]'); ylabel('e [rad]'); grid;
figure(); 
plot(time,E_DOT); title('ed'); xlabel('time [s]'); ylabel('ed [rad/s]'); grid;
figure(); 
plot(time,TAU); title('tau'); xlabel('time [s]'); ylabel('tau [Nm]'); grid;



