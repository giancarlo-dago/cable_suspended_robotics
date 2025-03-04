close all
clear
clc

run('cms_trolley_trajectory.m')

if ispc % Windows
    addpath('..\..\..\functions/trajectory_generation_functions')
    addpath('..\meshes')
else % Linux
    addpath('../../../functions/trajectory_generation_functions')
    addpath('../meshes')
end

% Simulation
T = 80;
sim('crane_cms')