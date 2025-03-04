close all
clear
clc

if ispc % Windows
    addpath('..\..\..\..\data\cranebot\new_identification_dataset\')
else % Linux
    addpath('../../../../data/cranebot/new_identification_dataset/')
end

experiments = "NatResExpZ";
expnum = 1;

%% TREAT NAT RES

if (experiments == "NatResExpX") 
    if expnum == 1 
        T1 = 9.93;
        T2 = 486.14;
    elseif expnum == 2 
        T1 = 8.61;
        T2 = 542.661;
    else
        T1 = 8.83;
        T2 = 384.59;
    end    
elseif (experiments == "NatResExpY") 
    if expnum == 1 
        T1 = 21.31;
        T2 = 100.18;
    elseif expnum == 2 
        T1 = 20.74;
        T2 = 121.39;
    else
        T1 = 16.09;
        T2 = 175.96;
    end    
elseif (experiments == "NatResExpZ") 
    if expnum == 1 
        T1 = 15.07;
        T2 = 135.66;
    elseif expnum == 2 
        T1 = 20.54;
        T2 = 117.28;
    else
        T1 = 17.72;
        T2 = 130.56;
    end    
end

realTraj = load(append(experiments,int2str(expnum),'.txt'));
time = realTraj(:,1);
startIndex = find(time>=T1,1,'first');
endIndex = find(time>=T2,1,'first');
    
if (experiments == "NatResExpX" || experiments == "NatResExpY")
    xPos1 = realTraj(:,2);
    xPos2 = realTraj(:,8);
    plot(time, xPos1), hold on, grid on, title('NatRes - Upper Marker [33 Hz]'), xlim([0 inf])
    plot(time, xPos2), hold on, grid on, title('NatRes - Lower Marker [33 Hz]'), xlim([0 inf])
elseif (experiments == "NatResExpZ")
    zPos2 = realTraj(:,13);
    plot(time, zPos2), hold on, grid on, title('NatRes - Lower Marker [33 Hz]'), xlim([0 inf])
end

%%
for k = startIndex:endIndex    
    cutTime(k-startIndex+1) = realTraj(k,1)-realTraj(startIndex,1);
    if (experiments == "NatResExpX" || experiments == "NatResExpY")         
       cutTraj(k-startIndex+1) = realTraj(k,8);
    elseif (experiments == "NatResExpZ") 
       cutTraj(k-startIndex+1) = realTraj(k,13);
    end
end
movMeanSamples = 10;
cutTrajMod = movmean(cutTraj,movMeanSamples)-mean(movmean(cutTraj,movMeanSamples));

fileID = fopen(append('Mod',experiments,int2str(expnum),'.txt'),'w');
for k = 1:length(cutTrajMod)
    fprintf(fileID,'%8.3f ',cutTime(k));
    if (experiments == "NatResExpX") 
       fprintf(fileID,'%8.3f ',cutTrajMod(k));
       fprintf(fileID,'0.0 ');
       fprintf(fileID,'0.0 ');
    elseif (experiments == "NatResExpY") 
       fprintf(fileID,'0.0 ');
       fprintf(fileID,'%8.3f ',cutTrajMod(k));
       fprintf(fileID,'0.0 ');
    elseif (experiments == "NatResExpZ") 
       fprintf(fileID,'0.0 ');
       fprintf(fileID,'0.0 ');
       fprintf(fileID,'%8.3f ',cutTrajMod(k));
    end
    fprintf(fileID,'\n');
end
fclose(fileID);

%%
experiments = "NatResExpX";
figure
for i=1:3
    Traj = load(append('Mod',experiments,int2str(i),'.txt'));
    time = Traj(:,1);
    subplot(3,1,i)
    plot(time, Traj(:,2:4)), hold on, grid on, title(append(experiments,' - Lower Marker [33 Hz]')), xlim([0 inf])
    legend('X','Y','Yaw')
end

%%
experiments = "NatResExpY";
figure
for i=1:3
    Traj = load(append('Mod',experiments,int2str(i),'.txt'));
    time = Traj(:,1);
    subplot(3,1,i)
    plot(time, Traj(:,2:4)), hold on, grid on, title(append(experiments,' - Lower Marker [33 Hz]')), xlim([0 inf])
    legend('X','Y','Yaw')
end

%%
experiments = "NatResExpZ";
figure
for i=1:3
    Traj = load(append('Mod',experiments,int2str(i),'.txt'));
    time = Traj(:,1);
    subplot(3,1,i)
    plot(time, Traj(:,2:4)), hold on, grid on, title(append(experiments,' - Lower Marker [33 Hz]')), xlim([0 inf])
    legend('X','Y','Yaw')
end
