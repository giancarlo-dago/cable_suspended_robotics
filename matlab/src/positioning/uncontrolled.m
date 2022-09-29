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

D_pub(1) = rospublisher('/licasa1/licasa1_prismatic_x_effort_pos_controller/command','std_msgs/Float64');
D_pub(2) = rospublisher('/licasa1/licasa1_prismatic_y_effort_pos_controller/command','std_msgs/Float64');
J_pub(1) = rospublisher('/licasa1/licasa1_leftarm_1_effort_pos_controller/command','std_msgs/Float64');
J_pub(2) = rospublisher('/licasa1/licasa1_leftarm_2_effort_pos_controller/command','std_msgs/Float64');
J_pub(3) = rospublisher('/licasa1/licasa1_leftarm_3_effort_pos_controller/command','std_msgs/Float64');
J_pub(4) = rospublisher('/licasa1/licasa1_leftarm_4_effort_pos_controller/command','std_msgs/Float64');
J_pub(5) = rospublisher('/licasa1/licasa1_rightarm_1_effort_pos_controller/command','std_msgs/Float64');
J_pub(6) = rospublisher('/licasa1/licasa1_rightarm_2_effort_pos_controller/command','std_msgs/Float64');
J_pub(7) = rospublisher('/licasa1/licasa1_rightarm_3_effort_pos_controller/command','std_msgs/Float64');
J_pub(8) = rospublisher('/licasa1/licasa1_rightarm_4_effort_pos_controller/command','std_msgs/Float64');

%% ------------------- Inverse Dynamics control ----------------------------
B = zeros(n_joints,n_joints);
unpause_gazebo = call(unpause_client,unpause_req);
[initial_drone_position, initial_drone_velocity] = read_drone_state(receive(joint_state_sub,10));           % Read drone state and put in q0
run('../drone_motion/drone_trajectory.m')                                                                   % Generate drone trajectory 

time = 0:samplingTime:T_total-samplingTime;
for i=1:length(time)
    
    %% Sensing (drone state)
    [drone_position, drone_velocity] = read_drone_state(receive(joint_state_sub,10));                                              % Read drone state and put in q0
    q_drone_current(:,i) = drone_position;
    
    %% Sensing (arm joints state)
    [q, q_dot] = read_feedback(receive(joint_state_sub,10));
    qA = q(5:8);
    qB = q(9:12);
    q_dotA = q_dot(5:8);
    q_dotB = q_dot(9:12);
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
    tauA = recursive_invdyn_f(qA, q_dotA, yA, g, infoA, F_eeA);
    TAU(1:4,i) = tauA;
    tauB = recursive_invdyn_f(qB, q_dotB, yB, g, infoB, F_eeB);
    TAU_B(5:8,i) = tauB;
    
    %% Publishing the commands
    pub_arms_control(J_pub, [tauA; tauB]);  
    pub_drone_control(D_pub, [q_drone_reference(1,i) q_drone_reference(2,i)]);

    unpause_gazebo = call(unpause_client,unpause_req);
    pause(samplingTime);

end

pause_gazebo = call(pause_client,pause_req);

%% Plot
figure(); 
plot(time,q_drone_current); title('Drone position'); xlabel('time [s]'); ylabel('drone position [m]'); grid; hold on;
plot(time,q_drone_reference,'--k');
figure(); 
plot(time,Q(1:2,:)); title('Upper passive joint'); xlabel('time [s]'); ylabel('q_{passive,1} [rad]'); grid; legend('alpha_x','alpha_y')
figure(); 
plot(time,Q(3:4,:)); title('Lower passive joint'); xlabel('time [s]'); ylabel('q_{passive,2} [rad]'); grid; legend('beta_x','beta_y')
figure(); 
plot(time,Q(5:12,:)); title('q'); xlabel('time [s]'); ylabel('q [rad]'); grid;
figure(); 
plot(time,QD(5:12,:)); title('qd'); xlabel('time [s]'); ylabel('qd [rad/s]'); grid;
figure(); 
plot(time,E); title('e'); xlabel('time [s]'); ylabel('e [rad]'); grid;
figure(); 
plot(time,E_DOT); title('ed'); xlabel('time [s]'); ylabel('ed [rad/s]'); grid;
figure(); 
plot(time,TAU); title('tau'); xlabel('time [s]'); ylabel('tau [Nm]'); grid;
