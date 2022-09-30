% data = load('LogIdentification_XYYaw_2022-09-06_Test-1.txt');
% data = load('LogIdentification_XYYaw_2022-09-06_Test-2.txt');
% data = load('LogIdentification_XYYaw_2022-09-06_Test-3.txt');
% data = load('Log_Identification_Y_Test-4.txt');
% data = load('LogIdentification_XYYaw_Simultaneous_Test-5.txt');
% data = load('LogIdentification_XYYaw_Simultaneous_Test-6.txt');

% data = load('data1.txt');
% data = load('data2.txt');
data = load('data3.txt');
% data = load('data4.txt');
% data = load('data5.txt');
% data = load('data6.txt');
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

%% PLOT IMU DATA %%

figure(1)

% To correct the angle
for i = 1:length(imuRPY)
    if i > 1
        if imuRPY(i,3) < (- 300)
            imuRPY(i,3) = imuRPY(i,3) + 360;
        end
        if imuRPY(i,3) > 300
            imuRPY(i,3) = imuRPY(i,3) - 360;
        end
        if imuRPY(i,2) < (- 300)
            imuRPY(i,2) = imuRPY(i,2) + 360;
        end
        if imuRPY(i,2) > 300
            imuRPY(i,2) = imuRPY(i,2) - 360;
        end
        if imuRPY(i,1) < (- 300)
            imuRPY(i,1) = imuRPY(i,1) + 360;
        end
        if imuRPY(i,1) > 300
            imuRPY(i,1) = imuRPY(i,1) - 360;
        end
    end
end

% To filter the outliers
for i = 1:length(imuRPY)
    if i > 1
        if abs(imuRPY(i,3) - imuRPY(i-1,3)) > 30
            imuRPY(i,3) = imuRPY(i-1,3);
        end
        if abs(imuRPY(i,2) - imuRPY(i-1,2)) > 30
            imuRPY(i,2) = imuRPY(i-1,2);
        end
        if abs(imuRPY(i,1) - imuRPY(i-1,1)) > 30
            imuRPY(i,1) = imuRPY(i-1,1);
        end
        
        if abs(imuAxyz(i,3) - imuAxyz(i-1,3)) > 2
            imuAxyz(i,3) = imuAxyz(i-1,3);
        end
        if abs(imuAxyz(i,2) - imuAxyz(i-1,2)) > 2
            imuAxyz(i,2) = imuAxyz(i-1,2);
        end
        if abs(imuAxyz(i,1) - imuAxyz(i-1,1)) > 2
            imuAxyz(i,1) = imuAxyz(i-1,1);
        end
        
        if abs(imuGxyz(i,3) - imuGxyz(i-1,3)) > 30
            imuGxyz(i,3) = imuGxyz(i-1,3);
        end
        if abs(imuGxyz(i,2) - imuGxyz(i-1,2)) > 30
            imuGxyz(i,2) = imuGxyz(i-1,2);
        end
        if abs(imuGxyz(i,1) - imuGxyz(i-1,1)) > 30
            imuGxyz(i,1) = imuGxyz(i-1,1);
        end
    end
end
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


%% PLOT OPTI-TRACK DATA %%
figure(2)
subplot(2,1,1), plot(t, dualArmPosition - dualArmPosition(1,1:3))
hold on
grid on
title('DUAL ARM POSITION DEVIATION')
ylabel('position [m]')
legend('X', 'Y', 'Z')

orientation = 180/pi*quat2eul(dualArmQuaternion(:,1:4), 'ZYX');
for i = 1:length(orientation)
     if orientation(i,1) > 150
         orientation(i,1) = orientation(i,1) - 180; 
     end
     if orientation(i,1) < -150
         orientation(i,1) = orientation(i,1) + 180; 
     end
end

subplot(2,1,2), plot(t, orientation)
grid on
ylabel('orientation [deg]')
legend('X', 'Y', 'Z')
xlabel('time [s]')


