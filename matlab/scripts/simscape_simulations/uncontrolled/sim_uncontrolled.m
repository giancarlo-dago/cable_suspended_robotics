close all
clear
clc

if ispc % Windows
    addpath('..\meshes\')
    addpath('..\positioning\controlled')
    addpath('..\..\..\functions\screw_theory_functions\')
    addpath('..\..\..\parameters\')
else % Linux
    addpath('../meshes/')
    addpath('../positioning/controlled')
    addpath('../../../functions/trajectory_generation_functions/')
    addpath('../../../parameters/')
end

run('cranebot_parameters.m')
run('d0_trajectory.m')

% Initial conditions
alfa_0 = deg2rad(0);
beta_0 = deg2rad(0);

% Simulation
T = 800;
set_param('uncontrolled','SimMechanicsOpenEditorOnUpdate','on')
sim('uncontrolled')

