function [joint_position, joint_velocity] = read_test_feedback(joint_state_latest_mex)

    % This function accept as input the last message from the joint state
    % publisher (joint_state.LastMessage) of type 'sensor_msgs/JointState'
    % and gives back the two N_JOINTSx1 vectors of joint position and velocities.
    % - joint_state_latest_mex is a 'sensor_msgs/JointState' variable from
    %   the joint state publisher.
    % - joint_position and joint_velocity are N_JOINTSx1 vectors of joint
    %   positions and velocities
    
    joint_state = rosmessage('sensor_msgs/JointState');

    joint_state = joint_state_latest_mex;
    joint_position_t = joint_state.Position;
    joint_velocity_t = joint_state.Velocity;
    
    %   - bar_joint
    %   - left_arm_joint
    %   - right_arm_joint
    %   - shoulder_joint

    q1_passive = joint_position_t(1);
    q2_passive = joint_position_t(4);
    q1_left = joint_position_t(2);
    q1_right = joint_position_t(3);
    
    dq1_passive = joint_velocity_t(1);
    dq2_passive = joint_velocity_t(4);
    dq1_left = joint_velocity_t(2);
    dq1_right = joint_velocity_t(3);
    
    joint_position = [q1_passive q2_passive q1_left q1_right];
    joint_velocity = [dq1_passive dq2_passive dq1_left dq1_right];

    joint_position = joint_position';
    joint_velocity = joint_velocity';

end