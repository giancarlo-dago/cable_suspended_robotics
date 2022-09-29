close all
clear
clc

addpath('../../../../../include/control_functions')
addpath('../../../../../include/screw_theory_functions')
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


% J_pub(1) = rospublisher('/licasa1/licasa1_leftarm_1_effort_pos_controller/command','std_msgs/Float64');
% J_pub(2) = rospublisher('/licasa1/licasa1_leftarm_2_effort_pos_controller/command','std_msgs/Float64');
% J_pub(3) = rospublisher('/licasa1/licasa1_leftarm_3_effort_pos_controller/command','std_msgs/Float64');
% J_pub(4) = rospublisher('/licasa1/licasa1_leftarm_4_effort_pos_controller/command','std_msgs/Float64');
% J_pub(5) = rospublisher('/licasa1/licasa1_rightarm_1_effort_pos_controller/command','std_msgs/Float64');
% J_pub(6) = rospublisher('/licasa1/licasa1_rightarm_2_effort_pos_controller/command','std_msgs/Float64');
% J_pub(7) = rospublisher('/licasa1/licasa1_rightarm_3_effort_pos_controller/command','std_msgs/Float64');
% J_pub(8) = rospublisher('/licasa1/licasa1_rightarm_4_effort_pos_controller/command','std_msgs/Float64');


J_pub(1) = rospublisher('/licasa1/licasa1_leftarm_1_position_pos_controller/command','std_msgs/Float64');
J_pub(2) = rospublisher('/licasa1/licasa1_leftarm_2_position_pos_controller/command','std_msgs/Float64');
J_pub(3) = rospublisher('/licasa1/licasa1_leftarm_3_position_pos_controller/command','std_msgs/Float64');
J_pub(4) = rospublisher('/licasa1/licasa1_leftarm_4_position_pos_controller/command','std_msgs/Float64');
J_pub(5) = rospublisher('/licasa1/licasa1_rightarm_1_position_pos_controller/command','std_msgs/Float64');
J_pub(6) = rospublisher('/licasa1/licasa1_rightarm_2_position_pos_controller/command','std_msgs/Float64');
J_pub(7) = rospublisher('/licasa1/licasa1_rightarm_3_position_pos_controller/command','std_msgs/Float64');
J_pub(8) = rospublisher('/licasa1/licasa1_rightarm_4_position_pos_controller/command','std_msgs/Float64');

%% Load the trajectory from the real data

t = data(:,1);
qLref = data(:,2:5);
qRref = data(:,6:9);
qL = 180/pi*data(:,10:13);
qR = 180/pi*data(:,14:17);
dqLdt = data(:,18:21);
dqRdt = data(:,22:25);
pwmL = 100*data(:,26:29);
pwmR = 100*data(:,30:33);
pL = data(:,34:36);
pR = data(:,37:39);
gripperLState = data(:,40);
gripperRState = data(:,41);
imuRPY = data(:,42:44);
imuAxyz = data(:,45:47);
imuGxyz = data(:,48:50);
multirotorPosition = data(:,51:53);
multirotorQuaternion = data(:,54:57);
dualArmPosition = data(:,58:60);
dualArmQuaternion = data(:,61:64);
powerLinePosition = data(:,65:67);
powerLineQuaternion = data(:,68:71);

time = t-t(1);

%% Sending the real joints trajectory to the simulator
run('kinematic_description')
unpause_gazebo = call(unpause_client,unpause_req);

% index = find(time>15,1);
index = find(time==time(end),1);

% start_time = read_clock(receive(clock_sub));
% elapsed_time = 0;
% i = 1;

% while (elapsed_time < 500)
%     
%     current_time = read_clock(receive(clock_sub));
%     elapsed_time = current_time - start_time;
% 
%     % Sensing (arm and shoulders joints state)
% %     [full_q, full_q_dot] = read_feedback(receive(joint_state_sub,10));
% %     [shoulder_link_x_position, shoulder_link_y_position] = read_gazebo_joint_state(receive(link_state_sub,10));
%     
%     %     pause_gazebo = call(pause_client,pause_req);
%     
%     q = full_q(6:13);
%     q_dot = full_q_dot(6:13);
%     Q(:,i) = q;
%     QD(:,i) = q_dot;
%     SPOSX(:,i) = shoulder_link_x_position;
%     SPOSY(:,i) = shoulder_link_y_position;
%     
%     % Kinematics (position of the end effector)
%     %     Te_left = fkin_f(q(1:4), 4, M_be_left, S_left);
%     %     Te_right = fkin_f(q(5:8), 4, M_be_right, S_right);
%     % 
%     %     pe_left = Te_left(1:3,4);
%     %     pe_right = Te_right(1:3,4);
%     %     
%     %     PE_left(:,i) = pe_left;
%     %     PE_right(:,i) = pe_right;
%         
%     % Publishing the commands
%     q_reference = [qL(i,1).*pi/180, qL(i,2).*pi/180, qL(i,3).*pi/180, qL(i,4).*pi/180, qR(i,1).*pi/180, qR(i,2).*pi/180, qR(i,3).*pi/180, qR(i,4).*pi/180];
%     pub_arms_control(J_pub, q_reference);
%     %     unpause_gazebo = call(unpause_client,unpause_req);
% 
%     %     if i~=length(time)
%     %         pause(time(i+1)-time(i));
%     %     end
% 
%     %     pause(0.02);
%     i = i+1;
% 
% end

q_reference = [qL(i,1).*pi/180, qL(i,2).*pi/180, qL(i,3).*pi/180, qL(i,4).*pi/180, qR(i,1).*pi/180, qR(i,2).*pi/180, qR(i,3).*pi/180, qR(i,4).*pi/180];

for i=1:index
    
%     % Sensing (arm and shoulders joints state)
%     [full_q, full_q_dot] = read_feedback(receive(joint_state_sub,10));
%     [shoulder_link_x_position, shoulder_link_y_position] = read_gazebo_joint_state(receive(link_state_sub,10));
%     
%     %     pause_gazebo = call(pause_client,pause_req);
%     
%     q = full_q(6:13);
%     q_dot = full_q_dot(6:13);
%     Q(:,i) = q;
%     QD(:,i) = q_dot;
%     SPOSX(:,i) = shoulder_link_x_position;
%     SPOSY(:,i) = shoulder_link_y_position;
%       
%     % Kinematics (position of the end effector)
%     Te_left = fkin_f(q(1:4), 4, M_be_left, S_left);
%     Te_right = fkin_f(q(5:8), 4, M_be_right, S_right);
% 
%     pe_left = Te_left(1:3,4);
%     pe_right = Te_right(1:3,4);
%     
%     PE_left(:,i) = pe_left;
%     PE_right(:,i) = pe_right;
%         
%     tic 
    % Publishing the commands
    q_reference = [qL(i,1).*pi/180, qL(i,2).*pi/180, qL(i,3).*pi/180, qL(i,4).*pi/180, qR(i,1).*pi/180, qR(i,2).*pi/180, qR(i,3).*pi/180, qR(i,4).*pi/180];
    pub_arms_control(J_pub, q_reference);
%         unpause_gazebo = call(unpause_client,unpause_req);
    
    if i~=length(time)
        pause(time(i+1)-time(i));
    end
%     toc
end

%% COMPARE JOINT POSITION
figure('WindowState','maximized')
sgtitle('LEFT ARM JOINT POSITION')
subplot(2,2,1), plot(time(1:index), qL(1:index,1)), hold on, grid on
subplot(2,2,1), plot(time(1:index), Q(1,:).*180/pi)
subplot(2,2,1), plot(time(1:index), qLref(1:index,1), 'k--'), ylabel('q_{1L} [deg]'), xlabel('time [s]'), title('First Joint'), xlim([time(1) time(end)]), legend('real','simulation','reference')
ylim([-100 55])
subplot(2,2,2), plot(time(1:index), qL(1:index,2)), hold on, grid on
subplot(2,2,2), plot(time(1:index), Q(2,:).*180/pi)
subplot(2,2,2), plot(time(1:index), qLref(1:index,2), 'k--'), ylabel('q_{2L} [deg]'), xlabel('time [s]'), title('Second Joint'), xlim([time(1) time(end)]), legend('real','simulation','reference')
ylim([-0.3 0.3])
subplot(2,2,3), plot(time(1:index), qL(1:index,3)), hold on, grid on
subplot(2,2,3), plot(time(1:index), Q(3,:).*180/pi)
subplot(2,2,3), plot(time(1:index), qLref(1:index,3), 'k--'), ylabel('q_{3L} [deg]'), xlabel('time [s]'), title('Third Joint'), xlim([time(1) time(end)]), legend('real','simulation','reference')
ylim([-0.3 0.3])
subplot(2,2,4), plot(time(1:index), qL(1:index,4)), hold on, grid on
subplot(2,2,4), plot(time(1:index), Q(4,:).*180/pi)
subplot(2,2,4), plot(time(1:index), qLref(1:index,4), 'k--'), ylabel('q_{4L} [deg]'), xlabel('time [s]'), title('Fourth Joint'), xlim([time(1) time(end)]), legend('real','simulation','reference')
ylim([-100 10])

figure('WindowState','maximized')
sgtitle('RIGHT ARM JOINT POSITION')
subplot(2,2,1), plot(time(1:index), qR(1:index,1)), hold on, grid on
subplot(2,2,1), plot(time(1:index), Q(5,:).*180/pi)
subplot(2,2,1), plot(time(1:index), qRref(1:index,1), 'k--'), ylabel('q_{1R} [deg]'), xlabel('time [s]'), title('First Joint'), xlim([time(1) time(end)]), legend('real','simulation','reference')
ylim([-100 55])
subplot(2,2,2), plot(time(1:index), qR(1:index,2)), hold on, grid on
subplot(2,2,2), plot(time(1:index), Q(6,:).*180/pi)
subplot(2,2,2), plot(time(1:index), qRref(1:index,2), 'k--'), ylabel('q_{2R} [deg]'), xlabel('time [s]'), title('Second Joint'), xlim([time(1) time(end)]), legend('real','simulation','reference')
ylim([-0.3 0.3])
subplot(2,2,3), plot(time(1:index), qR(1:index,3)), hold on, grid on
subplot(2,2,3), plot(time(1:index), Q(7,:).*180/pi)
subplot(2,2,3), plot(time(1:index), qRref(1:index,3), 'k--'), ylabel('q_{3R} [deg]'), xlabel('time [s]'), title('Third Joint'), xlim([time(1) time(end)]), legend('real','simulation','reference')
ylim([-0.3 0.3])
subplot(2,2,4), plot(time(1:index), qR(1:index,4)), hold on, grid on
subplot(2,2,4), plot(time(1:index), Q(8,:).*180/pi)
subplot(2,2,4), plot(time(1:index), qRref(1:index,4), 'k--'), ylabel('q_{4R} [deg]'), xlabel('time [s]'), title('Fourth Joint'), xlim([time(1) time(end)]), legend('real','simulation','reference')
ylim([-100 10])

%% COMPARE JOINT VELOCITY
figure('WindowState','maximized')
sgtitle('LEFT ARM JOINT VELOCITY')
subplot(2,2,1), plot(time(1:index), dqLdt(1:index,1)), hold on, grid on
subplot(2,2,1), plot(time(1:index), QD(1,:).*180/pi), ylabel('dq_{1L} [deg/s]'), xlabel('time [s]'), title('First Joint'), xlim([time(1) time(end)]), legend('real','simulation')
subplot(2,2,2), plot(time(1:index), dqLdt(1:index,2)), hold on, grid on
subplot(2,2,2), plot(time(1:index), QD(2,:).*180/pi), ylabel('dq_{2L} [deg/s]'), xlabel('time [s]'), title('Second Joint'), xlim([time(1) time(end)]), legend('real','simulation')
subplot(2,2,3), plot(time(1:index), dqLdt(1:index,3)), hold on, grid on
subplot(2,2,3), plot(time(1:index), QD(3,:).*180/pi), ylabel('dq_{3L} [deg/s]'), xlabel('time [s]'), title('Third Joint'), xlim([time(1) time(end)]), legend('real','simulation')
subplot(2,2,4), plot(time(1:index), dqLdt(1:index,4)), hold on, grid on
subplot(2,2,4), plot(time(1:index), QD(4,:).*180/pi), ylabel('dq_{4L} [deg/s]'), xlabel('time [s]'), title('Fourth Joint'), xlim([time(1) time(end)]), legend('real','simulation')

figure('WindowState','maximized')
sgtitle('RIGHT ARM JOINT VELOCITY')
subplot(2,2,1), plot(time(1:index), dqRdt(1:index,1)), hold on, grid on
subplot(2,2,1), plot(time(1:index), QD(5,:).*180/pi), ylabel('dq_{1R} [deg/s]'), xlabel('time [s]'), title('First Joint'), xlim([time(1) time(end)]), legend('real','simulation')
subplot(2,2,2), plot(time(1:index), dqRdt(1:index,2)), hold on, grid on
subplot(2,2,2), plot(time(1:index), QD(6,:).*180/pi), ylabel('dq_{2R} [deg/s]'), xlabel('time [s]'), title('Second Joint'), xlim([time(1) time(end)]), legend('real','simulation')
subplot(2,2,3), plot(time(1:index), dqRdt(1:index,3)), hold on, grid on
subplot(2,2,3), plot(time(1:index), QD(7,:).*180/pi), ylabel('dq_{3R} [deg/s]'), xlabel('time [s]'), title('Third Joint'), xlim([time(1) time(end)]), legend('real','simulation')
subplot(2,2,4), plot(time(1:index), dqRdt(1:index,4)), hold on, grid on
subplot(2,2,4), plot(time(1:index), QD(8,:).*180/pi), ylabel('dq_{4R} [deg/s]'), xlabel('time [s]'), title('Fourth Joint'), xlim([time(1) time(end)]), legend('real','simulation')

%% COMPARE END-EFFECTOR POSITION

figure('WindowState','maximized')
sgtitle('END-EFFECTOR POSITION')
subplot(2,3,1), plot(time(1:index), pL(1:index,1)), xlabel('time [s]'), ylabel('pL_X [m]'), title('pL_X'), xlim([time(1) time(end)]), grid, hold on
plot(time(1:index),PE_left(1,:)), legend('real','simulation')
subplot(2,3,2), plot(time(1:index), pL(1:index,2)), xlabel('time [s]'), ylabel('pL_Y [m]'), title('pL_Y'), xlim([time(1) time(end)]), grid, hold on
plot(time(1:index),PE_left(2,:)), legend('real','simulation')
subplot(2,3,3), plot(time(1:index), pL(1:index,3)), xlabel('time [s]'), ylabel('pL_Z [m]'), title('pL_Z'), xlim([time(1) time(end)]), grid, hold on
plot(time(1:index),PE_left(3,:)), legend('real','simulation')
subplot(2,3,4), plot(time(1:index), pR(1:index,1)), xlabel('time [s]'), ylabel('pR_X [m]'), title('pR_X'), xlim([time(1) time(end)]), grid, hold on
plot(time(1:index),PE_right(1,:)), legend('real','simulation')
subplot(2,3,5), plot(time(1:index), pR(1:index,2)), xlabel('time [s]'), ylabel('pR_Y [m]'), title('pR_Y'), xlim([time(1) time(end)]), grid, hold on
plot(time(1:index),PE_right(2,:)), legend('real','simulation')
subplot(2,3,6), plot(time(1:index), pR(1:index,3)), xlabel('time [s]'), ylabel('pR_Z [m]'), title('pR_Z'), xlim([time(1) time(end)]), grid, hold on
plot(time(1:index),PE_right(3,:)), legend('real','simulation')

% End-effector position plot (3D)
figure()
plot3(pL(1:index,1),pL(1:index,2),pL(1:index,3)), title('Left End-effector'), xlabel('x'), ylabel('y'), zlabel('z'), grid, hold on
plot3(PE_left(1,:),PE_left(2,:),PE_left(3,:)), legend('real','simulation')
figure()
plot3(pR(1:index,1),pR(1:index,2),pR(1:index,3)), title('Right End-effector'), xlabel('x'), ylabel('y'), zlabel('z'), grid, hold on
plot3(PE_right(1,:),PE_right(2,:),PE_right(3,:)), legend('real','simulation')

%% IMU ACCELERATION INTEGRATION
figure('WindowState','maximized')
subplot(2,1,1),plot(time(1:index), imuAxyz(1:index,:)), xlim([time(1) time(end)]), ylim([-2 2]), grid, ylabel('accel [m/s^2]'), legend('A_x', 'A_y', 'A_z')
subplot(2,1,2),plot(time(1:index), filloutliers(imuAxyz(1:index,:),'next','percentiles',[0.1 99.9])), xlim([time(1) time(end)]), ylim([-2 2]), grid,  ylabel('accel [m/s^2]'), legend('A_x', 'A_y', 'A_z')
sgtitle('IMU Acceleration')

% Integrazione
figure('WindowState','maximized')
acc_x = filloutliers(imuAxyz(1:index,1),'next','percentiles',[0 99.9]);
vel_x = cumtrapz(time(1:index),acc_x);
pos_x = cumtrapz(time(1:index),vel_x);
sgtitle('IMU Acceleration Integration X')
subplot(4,1,1), plot(time(1:index), imuAxyz(1:index,1)), grid, title('Acceleration X'), xlabel('time [s]'), ylabel('Acceleration [m/s^2]'), xlim([time(1) time(end)]), ylim([-5 5])
subplot(4,1,2), plot(time(1:index), acc_x), grid, title('Acceleration X (without outliers)'), xlabel('time [s]'), ylabel('Acceleration [m/s^2]'), xlim([time(1) time(end)])
subplot(4,1,3), plot(time(1:index), vel_x), grid, title('Velocity X'), xlabel('time [s]'), ylabel('Velocity [m/s]'), xlim([time(1) time(end)])
subplot(4,1,4), plot(time(1:index), pos_x), grid, title('Position X'), xlabel('time [s]'), ylabel('Position [m]'), xlim([time(1) time(end)])

figure('WindowState','maximized')
acc_y = filloutliers(imuAxyz(1:index,2),'next','percentiles',[0 99.9]);
vel_y = cumtrapz(time(1:index),acc_y);
pos_y = cumtrapz(time(1:index),vel_y);
sgtitle('IMU Acceleration Integration Y')
subplot(4,1,1), plot(time(1:index), imuAxyz(1:index,2)), grid, title('Acceleration Y'), xlabel('time [s]'), ylabel('Acceleration [m/s^2]'), xlim([time(1) time(end)]), ylim([-5 5])
subplot(4,1,2), plot(time(1:index), acc_y), grid, title('Acceleration Y (without outliers)'), xlabel('time [s]'), ylabel('Acceleration [m/s^2]'), xlim([time(1) time(end)])
subplot(4,1,3), plot(time(1:index), vel_y), grid, title('Velocity Y'), xlabel('time [s]'), ylabel('Velocity [m/s]'), xlim([time(1) time(end)])
subplot(4,1,4), plot(time(1:index), pos_y), grid, title('Position Y'), xlabel('time [s]'), ylabel('Position [m]'), xlim([time(1) time(end)])

figure('WindowState','maximized')
acc_z = filloutliers(imuAxyz(1:index,3),'next','percentiles',[0.1 99.9]);
vel_z = cumtrapz(time(1:index),acc_z);
pos_z = cumtrapz(time(1:index),vel_z);
sgtitle('IMU Acceleration Integration Z')
subplot(4,1,1), plot(time(1:index), imuAxyz(1:index,3)), grid, title('Acceleration Z'), xlabel('time [s]'), ylabel('Acceleration [m/s^2]'), xlim([time(1) time(end)]), ylim([-5 5])
subplot(4,1,2), plot(time(1:index), acc_y), grid, title('Acceleration Z (without outliers)'), xlabel('time [s]'), ylabel('Acceleration [m/s^2]'), xlim([time(1) time(end)])
subplot(4,1,3), plot(time(1:index), vel_y), grid, title('Velocity Z'), xlabel('time [s]'), ylabel('Velocity [m/s]'), xlim([time(1) time(end)])
subplot(4,1,4), plot(time(1:index), pos_y), grid, title('Position Z'), xlabel('time [s]'), ylabel('Position [m]'), xlim([time(1) time(end)])

%% SHOULDER POSE IN SPACE

figure('WindowState','maximized')
sgtitle('Dual Arm Position Deviation')
grid;
subplot(3,1,1), plot(time(1:index), dualArmPosition(1:index,1) - dualArmPosition(1,1)), title('X Deviation'), xlabel('time [s]'), ylabel('Position [m]'), grid
subplot(3,1,2), plot(time(1:index), dualArmPosition(1:index,2) - dualArmPosition(1,2)), title('Y Deviation'), xlabel('time [s]'), ylabel('Position [m]'), grid
subplot(3,1,3), plot(time(1:index), dualArmPosition(1:index,3) - dualArmPosition(1,3)), title('Z Deviation'), xlabel('time [s]'), ylabel('Position [m]'), grid

figure('WindowState','maximized')
sgtitle('Gazebo')
grid;
subplot(2,1,1), plot(time(1:index), SPOSX), title('X Deviation'), xlabel('time [s]'), ylabel('Position [m]'), grid
subplot(2,1,2), plot(time(1:index), SPOSY), title('Y Deviation'), xlabel('time [s]'), ylabel('Position [m]'), grid


figure('WindowState','maximized')
sgtitle('Comparison')
grid;
subplot(3,1,1), plot(time(1:index), dualArmPosition(1:index,1) - dualArmPosition(1,1)), title('X Deviation'), xlabel('time [s]'), ylabel('Position [m]'), grid, hold on;
plot(time(1:index), SPOSX(1,:))
subplot(3,1,2), plot(time(1:index), dualArmPosition(1:index,2) - dualArmPosition(1,2)), title('Y Deviation'), xlabel('time [s]'), ylabel('Position [m]'), grid, hold on;
plot(time(1:index), SPOSX(2,:))
subplot(3,1,3), plot(time(1:index), dualArmPosition(1:index,3) - dualArmPosition(1,3)), title('Z Deviation'), xlabel('time [s]'), ylabel('Position [m]'), grid, hold on;
plot(time(1:index), SPOSX(3,:))

figure('WindowState','maximized')
sgtitle('Dual Arm Orientation')
grid;
orientation = 180/pi*quat2eul(dualArmQuaternion(1:index,1:4));
subplot(3,1,1), plot(time(1:index), orientation(:,1)), title('X Rotation'), xlabel('time [s]'), ylabel('Orientation [deg]'), grid
subplot(3,1,2), plot(time(1:index), orientation(:,2)), title('Y Rotation'), xlabel('time [s]'), ylabel('Orientation [deg]'), grid
subplot(3,1,3), plot(time(1:index), orientation(:,3)), title('Z Rotation'), xlabel('time [s]'), ylabel('Orientation [deg]'), grid


