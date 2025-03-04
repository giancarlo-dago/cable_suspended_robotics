close all
clear
clc

if ispc % Windows
    addpath('..\..\..\..\data\cranebot\new_identification_dataset\')
else % Linux
    addpath('../../../../data/cranebot/new_identification_dataset/')
end

experiments = "ArmsMovExpX_amp";
amp = 0.3;

%% TREAT NAT RES

if (experiments == "ArmsMovExpX_amp")
    if amp == 0.1
        T1 = 9.22;
        T2 = 285;
    elseif amp == 0.2
        T1 = 8.9;
        T2 = 290;
    else
        T1 = 7.95;
        T2 = 280;
    end    
elseif (experiments == "ArmsMovExpY1_amp") 
    if amp == 0.1
        T1 = 21.31;
        T2 = 100.18;
    elseif amp == 0.2
        T1 = 23.00;
        T2 = 106.8;
    elseif amp == 0.3
        T1 = 8.29;
        T2 = 94.8;
    else 
        T1 = 14.2;
        T2 = 98.9;
    end    
elseif (experiments == "ArmsMovExpZ_amp") 
    if amp == 0.05 
        T1 = 13.23;
        T2 = 108.8;
    elseif amp == 0.1
        T1 = 12.74;
        T2 = 112.21;
    else
        T1 = 14.75;
        T2 = 107.092;
    end    
end

realTraj = load(append(experiments,num2str(amp),'.txt'));
time = realTraj(:,1);
startIndex = find(time>=T1,1,'first');
endIndex = find(time>=T2,1,'first');
    
if (experiments == "ArmsMovExpX_amp" || experiments == "ArmsMovExpY1_amp")
    xPos1 = realTraj(:,2);
    xPos2 = realTraj(:,8);
    plot(time, xPos1), hold on, grid on, 
    plot(time, xPos2), hold on, grid on, legend('Upper Marker','Lower Marker'), xlim([0 inf])
elseif (experiments == "ArmsMovExpZ_amp")
    zPos2 = realTraj(:,13);
    plot(time, zPos2), hold on, grid on, title('NatRes - Ground Marker [33 Hz]'), xlim([0 inf])
end

%%
for k = startIndex:endIndex    
    cutTime(k-startIndex+1) = realTraj(k,1)-realTraj(startIndex,1);
    if (experiments == "ArmsMovExpX_amp" || experiments == "ArmsMovExpY1_amp")         
       cutTraj(k-startIndex+1) = realTraj(k,8);
    elseif (experiments == "ArmsMovExpZ_amp") 
       cutTraj(k-startIndex+1) = realTraj(k,13);
    end
end
movMeanSamples = 10;
cutTrajMod = movmean(cutTraj,movMeanSamples)-mean(movmean(cutTraj,movMeanSamples));

fileID = fopen(append('Mod',experiments,num2str(amp),'.txt'),'w');
for k = 1:length(cutTrajMod)
    fprintf(fileID,'%8.3f ',cutTime(k));
    if (experiments == "ArmsMovExpX_amp") 
       fprintf(fileID,'%8.3f ',cutTrajMod(k));
       fprintf(fileID,'0.0 ');
       fprintf(fileID,'0.0 ');
    elseif (experiments == "ArmsMovExpY1_amp") 
       fprintf(fileID,'0.0 ');
       fprintf(fileID,'%8.3f ',cutTrajMod(k));
       fprintf(fileID,'0.0 ');
    elseif (experiments == "ArmsMovExpZ_amp") 
       fprintf(fileID,'0.0 ');
       fprintf(fileID,'0.0 ');
       fprintf(fileID,'%8.3f ',cutTrajMod(k));
    end
    fprintf(fileID,'\n');
end
fclose(fileID);

%%
experiments = "ArmsMovExpX_amp";
ampVec = [0.1 0.2 0.3];
figure
for i=1:length(ampVec)
    Traj = load(append('Mod',experiments,num2str(ampVec(i)),'.txt'));
    time = Traj(:,1);
    subplot(3,1,i)
    plot(time, Traj(:,2:4)), hold on, grid on, title(append(experiments,' - Lower Marker [33 Hz]')), xlim([0 inf])
    legend('X','Y','Yaw')
end

%%
experiments = "ArmsMovExpY1_amp";
ampVec = [0.2 0.3 0.35];
figure
for i=1:length(ampVec)
    Traj = load(append('Mod',experiments,num2str(ampVec(i)),'.txt'));
    time = Traj(:,1);
    subplot(3,1,i)
    plot(time, Traj(:,2:4)), hold on, grid on, title(append(experiments,' - Lower Marker [33 Hz]')), xlim([0 inf])
    legend('X','Y','Yaw')
end

%%
experiments = "ArmsMovExpZ_amp";
ampVec = [0.05 0.1 0.15];
figure
for i=1:length(ampVec)
    Traj = load(append('Mod',experiments,num2str(ampVec(i)),'.txt'));
    time = Traj(:,1);
    subplot(3,1,i)
    plot(time, Traj(:,2:4)), hold on, grid on, title(append(experiments,' - Ground Marker [33 Hz]')), xlim([0 inf])
    legend('X','Y','Yaw')
end
