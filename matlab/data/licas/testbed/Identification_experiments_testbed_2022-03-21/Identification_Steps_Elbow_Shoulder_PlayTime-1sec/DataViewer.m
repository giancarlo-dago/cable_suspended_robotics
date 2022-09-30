data = load('Step_Yaw_Video_11h19min.txt');

index = 1;

t = data(:,index);
index = index + 1;

qLref = data(:,index:index + 3);
index = index + 4;
qRref = data(:,index:index + 3);
index = index + 4;

qL = 180/pi*data(:,index:index + 3);
index = index + 4;
qR = 180/pi*data(:,index:index + 3);
index = index + 4;

dqLdt = data(:,index:index + 3);
index = index + 4;
dqRdt = data(:,index:index + 3);
index = index + 4;

pwmL = 100*data(:,index:index + 3);
index = index + 4;
pwmR = 100*data(:,index:index + 3);
index = index + 4;

pL = data(:,index:index + 2);
index = index + 3;
pR = data(:,index:index + 2);
index = index + 3;

gripperLState = data(:,index);
index = index + 1;
gripperRState = data(:,index);
index = index + 1;

imuRPY = data(:,index:index + 2);
index = index + 3;
imuAxyz = data(:,index:index + 2);
index = index + 3;
imuGxyz = data(:,index:index + 2);
index = index + 3;

multirotorPosition = data(:,index:index + 2);
index = index + 3;
multirotorQuaternion = data(:,index:index + 3);
index = index + 4;

dualArmPosition = data(:,index:index + 2);
index = index + 3;
dualArmQuaternion = data(:,index:index + 3);
index = index + 4;

powerLinePosition = data(:,index:index + 2);
index = index + 3;
powerLineQuaternion = data(:,index:index + 3);
index = index + 4;


%% PLOT ARMS DATA %%

figure(1)
subplot(2,1,1), plot(t, qL)
hold on
grid on
subplot(2,1,1), plot(t, qLref, 'k')
ylabel('q_L [deg]')
legend('q_1', 'q_2', 'q_3', 'q_4')
title('LEFT ARM JOINT POSITION')
subplot(2,1,2), plot(t, qR)
hold on
grid on
subplot(2,1,2), plot(t, qRref, 'k')
ylabel('q_R [deg]')
title('RIGHT ARM JOINT POSITION')
xlabel('time [s]')

figure(2)
subplot(2,1,1), plot(t, pL)
hold on
grid on
ylabel('p_L [m]')
legend('X', 'Y', 'Z')
title('LEFT ARM TCP POSITION')
subplot(2,1,2), plot(t, pR)
hold on
grid on
ylabel('p_R [m]')
legend('X', 'Y', 'Z')
title('RIGHT ARM TCP POSITION')
xlabel('time [s]')

figure(3)
subplot(2,1,1), plot(t, dqLdt)
hold on
grid on
ylabel('w_L [deg/s]')
legend('q_1', 'q_2', 'q_3', 'q_4')
title('LEFT ARM JOINT SPEED')
subplot(2,1,2), plot(t, dqRdt)
hold on
grid on
ylabel('w_R [deg/s]')
legend('q_1', 'q_2', 'q_3', 'q_4')
title('RIGHT ARM JOINT SPEED')
xlabel('time [s]')

figure(4)
subplot(2,1,1), plot(t, pwmL)
hold on
grid on
ylabel('PWM_L [%]')
legend('q_1', 'q_2', 'q_3', 'q_4')
title('LEFT ARM JOINT PWM')
subplot(2,1,2), plot(t, pwmR)
hold on
grid on
ylabel('PWM_R [%]')
legend('q_1', 'q_2', 'q_3', 'q_4')
title('RIGHT ARM JOINT PWM')
xlabel('time [s]')

% 
% %% PLOT IMU DATA %%
% 
figure(5)
subplot(3,1,1), plot(t, imuRPY)
hold on
grid on
title('IMU Orientation')
ylabel('orientation [deg]')
v = axis;
v(3) = -30;
v(4) = 30;
axis(v)
legend('Roll', 'Pitch', 'Yaw')
subplot(3,1,2), plot(t, imuAxyz)
hold on
grid on
title('IMU Acceleration')
ylabel('accel [m/s^2]')
v = axis;
v(3) = -2;
v(4) = 2;
axis(v)
legend('A_x', 'A_y', 'A_z')
subplot(3,1,3), plot(t, imuGxyz)
v = axis;
v(3) = -50;
v(4) = 50;
axis(v)
hold on
grid on
title('IMU Angular Speed')
ylabel('gyro [deg/s]')
legend('G_x', 'G_y', 'G_z')
xlabel('time [s]')
% 
% 
% %% PLOT OPTI-TRACK DATA %%
% 
% figure(6)
% subplot(2,1,1), plot(t, multirotorPosition)
% hold on
% grid on
% title('MULTIROTOR POSITION')
% legend('X', 'Y', 'Z')
% subplot(2,1,2), plot(t, 180/pi*quat2eul(multirotorQuaternion(:,1:4), 'ZYX'))
% ylabel('orientation [deg]')
% legend('X', 'Y', 'Z')
% xlabel('time [s]')

figure(7)
subplot(2,1,1), plot(t, dualArmPosition - dualArmPosition(1,1:3))
hold on
grid on
title('DUAL ARM POSITION DEVIATION')
ylabel('position [m]')
legend('X', 'Y', 'Z')
a = 180/pi*quat2eul(dualArmQuaternion(:,1:4), 'ZYX');
a(:,1:2) = a(:,1:2) * (-1);
for i = 1:length(a)
     if a(i,1) < 0
         a(i,1) = a(i,1) + 180;
     else
         a(i,1) = a(i,1) - 180;
     end
end
subplot(2,1,2), plot(t, a)
grid on
ylabel('orientation [deg]')
legend('X', 'Y', 'Z')
xlabel('time [s]')

% figure(8)
% subplot(2,1,1), plot(t, powerLinePosition)
% hold on
% grid on
% title('POWER LINE POSITION')
% legend('X', 'Y', 'Z')
% subplot(2,1,2), plot(t, 180/pi*quat2eul(powerLineQuaternion(:,1:4), 'ZYX'))
% ylabel('orientation [deg]')
% legend('X', 'Y', 'Z')
% xlabel('time [s]')

