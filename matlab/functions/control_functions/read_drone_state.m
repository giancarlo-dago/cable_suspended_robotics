function [drone_position, drone_velocity] = read_drone_state(joint_state_latest_mex)

    joint_state = rosmessage('sensor_msgs/JointState');

    joint_state = joint_state_latest_mex;
    joint_position_t = joint_state.Position;
    joint_velocity_t = joint_state.Velocity;
    
    % name: [LiCAS_A1_q1_1, LiCAS_A1_q1_2, LiCAS_A1_q1_3, LiCAS_A1_q1_4, LiCAS_A1_q2_1, LiCAS_A1_q2_2,
    %   LiCAS_A1_q2_3, LiCAS_A1_q2_4, prismatic_joint_x, prismatic_joint_y, revolute_joint_x,
    %   revolute_joint_y, shoulder_joint_x, shoulder_joint_y]

    drone_position_x = joint_position_t(9);
    drone_position_y = joint_position_t(10);
    drone_velocity_x = joint_velocity_t(9);
    drone_velocity_y = joint_velocity_t(10);
    

    drone_position = [drone_position_x drone_position_y];
    drone_velocity = [drone_velocity_x drone_velocity_y];

    drone_position = drone_position';
    drone_velocity = drone_velocity';

end