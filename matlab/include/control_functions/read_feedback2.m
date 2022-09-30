function [joint_position, joint_velocity] = read_feedback2(joint_state_latest_mex)

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
    
    % name: [LiCAS_A1_q1_1, LiCAS_A1_q1_2, LiCAS_A1_q1_3, LiCAS_A1_q1_4, LiCAS_A1_q2_1, LiCAS_A1_q2_2,
           % LiCAS_A1_q2_3, LiCAS_A1_q2_4, prismatic_joint_y, revolute_joint_x, shoulder_joint_x]
    
    q1_left = joint_position_t(1);
    q2_left = joint_position_t(2);
    q3_left = joint_position_t(3);
    q4_left = joint_position_t(4);
    q1_right = joint_position_t(5);
    q2_right = joint_position_t(6);
    q3_right = joint_position_t(7);
    q4_right = joint_position_t(8);
    q1_passive = joint_position_t(10);
    q2_passive = joint_position_t(11);
    
    dq1_left = joint_velocity_t(1);
    dq2_left = joint_velocity_t(2);
    dq3_left = joint_velocity_t(3);
    dq4_left = joint_velocity_t(4);
    dq1_right = joint_velocity_t(5);
    dq2_right = joint_velocity_t(6);
    dq3_right = joint_velocity_t(7);
    dq4_right = joint_velocity_t(8);
    dq1_passive = joint_velocity_t(10);
    dq2_passive = joint_velocity_t(11);

    joint_position = [q1_passive q2_passive q1_left q2_left q3_left q4_left q1_right q2_right q3_right q4_right];
    joint_velocity = [dq1_passive dq2_passive dq1_left dq2_left dq3_left dq4_left dq1_right dq2_right dq3_right dq4_right];

    joint_position = joint_position';
    joint_velocity = joint_velocity';

end