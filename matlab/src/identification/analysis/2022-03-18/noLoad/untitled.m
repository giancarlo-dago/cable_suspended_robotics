close all
clear
clc

addpath('../../../../../include/control_functions')
addpath('../../../experiments_data/2022-03-18/noLoad')

% data = load('Log_2022-3-18_12h31m5s_LRM_Identification.txt');
data = load('Log_2022-3-18_12h36m28s_LRM_Identification.txt');

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
link_state_sub = rossubscriber('/gazebo/link_states');
clock_sub = rossubscriber('/clock');

%% Sending the real joints trajectory to the simulator
unpause_gazebo = call(unpause_client,unpause_req);

% samplingTime = 0.02;
% T_total = 10;

% time = 0:samplingTime:T_total-samplingTime;
% tic

    start_time = read_clock(receive(clock_sub));
    elapsed_time = 0;
    i = 1;
    
while (elapsed_time < 10)
    
    current_time = read_clock(receive(clock_sub));
    elapsed_time = current_time - start_time
    
    [shoulder_link_x_position, shoulder_link_y_position] = read_gazebo_joint_state(receive(link_state_sub,10));
    A(i,:) = [elapsed_time; shoulder_link_x_position]';
    
%     pause_gazebo = call(pause_client,pause_req);
%     toc
%     SPOSX(:,i) = shoulder_link_x_position;
%     SPOSY(:,i) = shoulder_link_y_position;
%     pause(0.02);
%     unpause_gazebo = call(unpause_client,unpause_req);
    
    i = i+1;

end

%% SHOULDER POSE IN SPACE

% figure('WindowState','maximized')
% sgtitle('Gazebo')
% grid;
% subplot(2,1,1), plot(time, SPOSX), title('X Deviation'), xlabel('time [s]'), ylabel('Position [m]'), grid
% subplot(2,1,2), plot(time, SPOSY), title('Y Deviation'), xlabel('time [s]'), ylabel('Position [m]'), grid
