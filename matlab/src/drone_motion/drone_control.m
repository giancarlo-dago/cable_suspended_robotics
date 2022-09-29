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

%% ------------------- Drone Position control ----------------------------
unpause_gazebo = call(unpause_client,unpause_req);
[q0, dq0] = read_drone_state(receive(joint_state_sub,10));                                              % Read drone state and put in q0
run('drone_trajectory.m')     % Generate drone trajectory 

for i=1:length(total_time)
    
    %% Publishing the commands    
%     pause_gazebo = call(pause_client,pause_req);
    pub_drone_control(D_pub, [q_drone(1,i) q_drone(2,i)]);
    
%     unpause_gazebo = call(unpause_client,unpause_req);
    pause(samplingTime);

end

pause_gazebo = call(pause_client,pause_req);

