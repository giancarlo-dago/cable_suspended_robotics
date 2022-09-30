close all
clear
clc

%% Y data 1
T1 = 78.7355;
T2 = 96.1685;

realTraj = load(append('data1.txt'));
tTemp = realTraj(:,1);
startIndex = find(tTemp>=T1,1,'first');
endIndex = find(tTemp>=T2,1,'first');
fileID = fopen('finalNatRes7.txt','w');

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

%% Y data 2
T1 = 101.792;
T2 = 120.162;

realTraj = load(append('data1.txt'));
tTemp = realTraj(:,1);
startIndex = find(tTemp>=T1,1,'first');
endIndex = find(tTemp>=T2,1,'first');
fileID = fopen('finalNatRes8.txt','w');

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

%% Y data 3
T1 = 83.96;
T2 = 108.601;

realTraj = load(append('data2.txt'));
tTemp = realTraj(:,1);
startIndex = find(tTemp>=T1,1,'first');
endIndex = find(tTemp>=T2,1,'first');
fileID = fopen('finalNatRes9.txt','w');

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

%% Y data 4
T1 = 113.149;
T2 = 135.363;

realTraj = load(append('data2.txt'));
tTemp = realTraj(:,1);
startIndex = find(tTemp>=T1,1,'first');
endIndex = find(tTemp>=T2,1,'first');
fileID = fopen('finalNatRes10.txt','w');

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

%% Y data 5
T1 = 90.167;
T2 = 107.498;

realTraj = load(append('data3.txt'));
tTemp = realTraj(:,1);
startIndex = find(tTemp>=T1,1,'first');
endIndex = find(tTemp>=T2,1,'first');
fileID = fopen('finalNatRes11.txt','w');

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

%% Y data 6
T1 = 111.88;
T2 = 128.36;

realTraj = load(append('data3.txt'));
tTemp = realTraj(:,1);
startIndex = find(tTemp>=T1,1,'first');
endIndex = find(tTemp>=T2,1,'first');
fileID = fopen('finalNatRes12.txt','w');

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

%% Y data 7
T1 = 22.3136;
T2 = 46.3873;

realTraj = load(append('data4.txt'));
tTemp = realTraj(:,1);
startIndex = find(tTemp>=T1,1,'first');
endIndex = find(tTemp>=T2,1,'first');
fileID = fopen('finalNatRes13.txt','w');

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

%% Y data 8
T1 = 51.8747;
T2 = 73.9454;

realTraj = load(append('data4.txt'));
tTemp = realTraj(:,1);
startIndex = find(tTemp>=T1,1,'first');
endIndex = find(tTemp>=T2,1,'first');
fileID = fopen('finalNatRes14.txt','w');

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

%% Y data 9
T1 = 78.3915;
T2 = 102.411;

realTraj = load(append('data4.txt'));
tTemp = realTraj(:,1);
startIndex = find(tTemp>=T1,1,'first');
endIndex = find(tTemp>=T2,1,'first');
fileID = fopen('finalNatRes15.txt','w');

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

%% Y data 10
T1 = 108.763;
T2 = 155.528;

realTraj = load(append('data4.txt'));
tTemp = realTraj(:,1);
startIndex = find(tTemp>=T1,1,'first');
endIndex = find(tTemp>=T2,1,'first');
fileID = fopen('finalNatRes16.txt','w');

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


TRAJ = [7 8 9 10 11 12 13 14 15 16]; nTraj = 10;

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
