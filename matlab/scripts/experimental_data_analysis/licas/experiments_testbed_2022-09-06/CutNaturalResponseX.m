close all
clear
clc

%% X data 1
T1 = 27.06;
T2 = 47.72;

realTraj = load(append('data1.txt'));
tTemp = realTraj(:,1);
startIndex = find(tTemp>=T1,1,'first');
endIndex = find(tTemp>=T2,1,'first');
fileID = fopen('finalNatRes1.txt','w');

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

%% X data 2
T1 = 53.15;
T2 = 71.82;

realTraj = load(append('data1.txt'));
tTemp = realTraj(:,1);
startIndex = find(tTemp>=T1,1,'first');
endIndex = find(tTemp>=T2,1,'first');
fileID = fopen('finalNatRes2.txt','w');

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

%% X data 3
T1 = 25.1907;
T2 = 48.8282;

realTraj = load(append('data2.txt'));
tTemp = realTraj(:,1);
startIndex = find(tTemp>=T1,1,'first');
endIndex = find(tTemp>=T2,1,'first');
fileID = fopen('finalNatRes3.txt','w');

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

%% X data 4
T1 = 54.1515;
T2 = 78.7373;

realTraj = load(append('data2.txt'));
tTemp = realTraj(:,1);
startIndex = find(tTemp>=T1,1,'first');
endIndex = find(tTemp>=T2,1,'first');
fileID = fopen('finalNatRes4.txt','w');

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

%% X data 5
T1 = 44.521;
T2 = 63.2108;

realTraj = load(append('data3.txt'));
tTemp = realTraj(:,1);
startIndex = find(tTemp>=T1,1,'first');
endIndex = find(tTemp>=T2,1,'first');
fileID = fopen('finalNatRes5.txt','w');

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

%% X data 6
T1 = 67.2542;
T2 = 85.8646;

realTraj = load(append('data3.txt'));
tTemp = realTraj(:,1);
startIndex = find(tTemp>=T1,1,'first');
endIndex = find(tTemp>=T2,1,'first');
fileID = fopen('finalNatRes6.txt','w');

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


TRAJ = [1 2 3 4 5 6]; nTraj = 6;

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