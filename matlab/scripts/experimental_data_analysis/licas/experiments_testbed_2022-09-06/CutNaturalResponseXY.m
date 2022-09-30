close all
clear
clc

%% XY data 1
T1 = 171.44;
T2 = 187.217;

realTraj = load(append('data3.txt'));
tTemp = realTraj(:,1);
startIndex = find(tTemp>=T1,1,'first');
endIndex = find(tTemp>=T2,1,'first');
fileID = fopen('finalNatRes23.txt','w');

for k = startIndex:endIndex
    for h=1:57
        fprintf(fileID,'%8.3f ',realTraj(k,h));
    end
    fprintf(fileID,'%8.3f ',realTraj(k,58)-realTraj(1,58));
    fprintf(fileID,'%8.3f ',realTraj(k,59)-realTraj(1,59));
    fprintf(fileID,'%8.3f ',realTraj(k,60)-realTraj(1,60));
    for h=61:length(realTraj(1,:))
        fprintf(fileID,'%8.3f ',realTraj(k,h));
    end
    fprintf(fileID,'\n');
end
fclose(fileID);

%% XY data 2
T1 = 192.602;
T2 = 210.15;

realTraj = load(append('data3.txt'));
tTemp = realTraj(:,1);
startIndex = find(tTemp>=T1,1,'first');
endIndex = find(tTemp>=T2,1,'first');
fileID = fopen('finalNatRes24.txt','w');

for k = startIndex:endIndex
    for h=1:57
        fprintf(fileID,'%8.3f ',realTraj(k,h));
    end
    fprintf(fileID,'%8.3f ',realTraj(k,58)-realTraj(1,58));
    fprintf(fileID,'%8.3f ',realTraj(k,59)-realTraj(1,59));
    fprintf(fileID,'%8.3f ',realTraj(k,60)-realTraj(1,60));
    for h=61:length(realTraj(1,:))
        fprintf(fileID,'%8.3f ',realTraj(k,h));
    end
    fprintf(fileID,'\n');
end
fclose(fileID);

%% XY data 3
T1 = 20.2878;
T2 = 50.4938;

realTraj = load(append('data5.txt'));
tTemp = realTraj(:,1);
startIndex = find(tTemp>=T1,1,'first');
endIndex = find(tTemp>=T2,1,'first');
fileID = fopen('finalNatRes25.txt','w');

for k = startIndex:endIndex
    for h=1:57
        fprintf(fileID,'%8.3f ',realTraj(k,h));
    end
    fprintf(fileID,'%8.3f ',realTraj(k,58)-realTraj(1,58));
    fprintf(fileID,'%8.3f ',realTraj(k,59)-realTraj(1,59));
    fprintf(fileID,'%8.3f ',realTraj(k,60)-realTraj(1,60));
    for h=61:length(realTraj(1,:))
        fprintf(fileID,'%8.3f ',realTraj(k,h));
    end
    fprintf(fileID,'\n');
end
fclose(fileID);

%% XY data 4
T1 = 55.72;
T2 = 79.2765;

realTraj = load(append('data5.txt'));
tTemp = realTraj(:,1);
startIndex = find(tTemp>=T1,1,'first');
endIndex = find(tTemp>=T2,1,'first');
fileID = fopen('finalNatRes26.txt','w');

for k = startIndex:endIndex
    for h=1:57
        fprintf(fileID,'%8.3f ',realTraj(k,h));
    end
    fprintf(fileID,'%8.3f ',realTraj(k,58)-realTraj(1,58));
    fprintf(fileID,'%8.3f ',realTraj(k,59)-realTraj(1,59));
    fprintf(fileID,'%8.3f ',realTraj(k,60)-realTraj(1,60));
    for h=61:length(realTraj(1,:))
        fprintf(fileID,'%8.3f ',realTraj(k,h));
    end
    fprintf(fileID,'\n');
end
fclose(fileID);

%% XY data 5
T1 = 34.6392;
T2 = 53.1131;

realTraj = load(append('data6.txt'));
tTemp = realTraj(:,1);
startIndex = find(tTemp>=T1,1,'first');
endIndex = find(tTemp>=T2,1,'first');
fileID = fopen('finalNatRes27.txt','w');

for k = startIndex:endIndex
    for h=1:57
        fprintf(fileID,'%8.3f ',realTraj(k,h));
    end
    fprintf(fileID,'%8.3f ',realTraj(k,58)-realTraj(1,58));
    fprintf(fileID,'%8.3f ',realTraj(k,59)-realTraj(1,59));
    fprintf(fileID,'%8.3f ',realTraj(k,60)-realTraj(1,60));
    for h=61:length(realTraj(1,:))
        fprintf(fileID,'%8.3f ',realTraj(k,h));
    end
    fprintf(fileID,'\n');
end
fclose(fileID);

%% XY data 6
T1 = 57.258;
T2 = 77.6005;

realTraj = load(append('data6.txt'));
tTemp = realTraj(:,1);
startIndex = find(tTemp>=T1,1,'first');
endIndex = find(tTemp>=T2,1,'first');
fileID = fopen('finalNatRes28.txt','w');

for k = startIndex:endIndex
    for h=1:57
        fprintf(fileID,'%8.3f ',realTraj(k,h));
    end
    fprintf(fileID,'%8.3f ',realTraj(k,58)-realTraj(1,58));
    fprintf(fileID,'%8.3f ',realTraj(k,59)-realTraj(1,59));
    fprintf(fileID,'%8.3f ',realTraj(k,60)-realTraj(1,60));
    for h=61:length(realTraj(1,:))
        fprintf(fileID,'%8.3f ',realTraj(k,h));
    end
    fprintf(fileID,'\n');
end
fclose(fileID);


%% Plot

TRAJ = [23 24 25 26 27 28]; nTraj = 6;

for i=1:nTraj
%     realTraj = load(append('newNatRes1.txt'));                            % Retrieve data from file
    realTraj = load(append('finalnatRes',int2str(TRAJ(i)),'.txt'));         % Retrieve data from file

    t = realTraj(1:end,1);
    time = t-t(1);
    meanTs(i) = mean(time(2:end)-time(1:end-1));
    qRef = realTraj(1:end,10:17);
    q = realTraj(1:end,10:17);
    qd = realTraj(1:end,18:25);
    dualArmPosition = realTraj(1:end,58:60);
    dualArmQuaternion = realTraj(1:end,61:64);
    a = quat2eul(dualArmQuaternion(:,1:4), 'ZYX');
    a(:,1:2) = a(:,1:2) * (-1);
    for j = 1:size(a(:,1))
         if a(j,1) < 0
             a(j,1) = a(j,1) + pi;
         else
             a(j,1) = a(j,1) - pi;
         end
    end
    dualArmOrientation = a;
    
    % Save data in data structures
    shoulderX = dualArmPosition(:,1);
    shoulderY = dualArmPosition(:,2);
    shoulderYaw = filloutliers(dualArmOrientation(:,3),'makima');
    
    % Plot
    figure(1), subplot(2,ceil(nTraj/2),i), plot(time,[shoulderX shoulderY]), grid, hold on, xlim([time(1) time(end)]), ylim([-0.4 0.4]), xlabel('time [s]'), ylabel('position [m]'), title(append('Trajectory ',int2str(i))), legend('X Position','Y Position')
    figure(2), subplot(2,ceil(nTraj/2),i), plot(time,180/pi*shoulderYaw), grid, hold on, xlim([time(1) time(end)]), ylim([-100 100]), xlabel('time [s]'), ylabel('orientation [deg]'), title(append('Trajectory ',int2str(i)))
end