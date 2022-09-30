function pub_drone_control(publisher, message)

    % This function is used to publish the message in the topic specified by publisher
    
    N_JOINTS = 2;
    D_msg = zeros(2,1);
    
    for j=1:N_JOINTS
        D_msg(j) = message(j);
    end

    D_msg_std = rosmessage('std_msgs/Float64');
    for j=1:N_JOINTS
        D_msg_std.Data = D_msg(j);
        send(publisher(j), D_msg_std);
    end

end