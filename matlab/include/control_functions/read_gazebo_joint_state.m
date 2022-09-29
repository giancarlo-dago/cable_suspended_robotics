function [shoulder_link_x_position, shoulder_link_y_position] = read_gazebo_joint_state(link_state_latest_mex)

    % This function accept as input the last message from the joint state
    % publisher (joint_state.LastMessage) of type 'sensor_msgs/JointState'
    % and gives back the two N_JOINTSx1 vectors of joint position and velocities.
    % - joint_state_latest_mex is a 'sensor_msgs/JointState' variable from
    %   the joint state publisher.
    % - joint_position and joint_velocity are N_JOINTSx1 vectors of joint
    %   positions and velocities
    
    link_state = rosmessage('gazebo_msgs/LinkStates');

    link_state = link_state_latest_mex;
    link_poses = link_state.Pose;
    
%   - ground_plane::link
%   - LiCAS_A1::drone_link_x
%   - LiCAS_A1::drone_link_y
%   - LiCAS_A1::bar_link_z
%   - LiCAS_A1::bar_link_x
%   - LiCAS_A1::bar_link_y
%   - LiCAS_A1::shoulder_link_x
%   - LiCAS_A1::shoulder_link_y
%   - LiCAS_A1::left_shoulder_roll
%   - LiCAS_A1::left_shoulder_yaw
%   - LiCAS_A1::left_elbow_pitch
%   - LiCAS_A1::left_forearm_link
%   - LiCAS_A1::right_shoulder_roll
%   - LiCAS_A1::right_shoulder_yaw
%   - LiCAS_A1::right_elbow_pitch
%   - LiCAS_A1::right_forearm_link
    
    shoulder_link_x_pose = link_poses(7);
    shoulder_link_y_pose = link_poses(8);
    
    shoulder_link_x_position = [shoulder_link_x_pose.Position.X; shoulder_link_x_pose.Position.Y; shoulder_link_x_pose.Position.Z];
    shoulder_link_y_position = [shoulder_link_y_pose.Position.X; shoulder_link_y_pose.Position.Y; shoulder_link_y_pose.Position.Z];


end