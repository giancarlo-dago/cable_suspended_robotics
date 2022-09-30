close all
clear
clc

%% Z data 1
T1 = 124.023;
T2 = 139.258;

realTraj = load(append('data1.txt'));
tTemp = realTraj(:,1);
startIndex = find(tTemp>=T1,1,'first');
endIndex = find(tTemp>=T2,1,'first');
fileID = fopen('finalNatRes17.txt','w');

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

%% Z data 2
T1 = 143.6;
T2 = 160.163;

realTraj = load(append('data1.txt'));
tTemp = realTraj(:,1);
startIndex = find(tTemp>=T1,1,'first');
endIndex = find(tTemp>=T2,1,'first');
fileID = fopen('finalNatRes18.txt','w');

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

%% Z data 3
T1 = 138.539;
T2 = 156.025;

realTraj = load(append('data2.txt'));
tTemp = realTraj(:,1);
startIndex = find(tTemp>=T1,1,'first');
endIndex = find(tTemp>=T2,1,'first');
fileID = fopen('finalNatRes19.txt','w');

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

%% Z data 4
T1 = 160.084;
T2 = 177.851;

realTraj = load(append('data2.txt'));
tTemp = realTraj(:,1);
startIndex = find(tTemp>=T1,1,'first');
endIndex = find(tTemp>=T2,1,'first');
fileID = fopen('finalNatRes20.txt','w');

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

%% Z data 5
T1 = 132.38;
T2 = 147.679;

realTraj = load(append('data3.txt'));
tTemp = realTraj(:,1);
startIndex = find(tTemp>=T1,1,'first');
endIndex = find(tTemp>=T2,1,'first');
fileID = fopen('finalNatRes21.txt','w');

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

%% Z data 6
T1 = 151.338;
T2 = 166.798;

realTraj = load(append('data3.txt'));
tTemp = realTraj(:,1);
startIndex = find(tTemp>=T1,1,'first');
endIndex = find(tTemp>=T2,1,'first');
fileID = fopen('finalNatRes22.txt','w');

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




TRAJ = [17 18 19 20 21 22]; nTraj = 6;

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
