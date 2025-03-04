function [Q,QD,QDD,time] = joints_trajectory(q0,qf,Tc,T0,Tf)

    Nc = 1/Tc;                                          % Sampling frequency

    % Variable part of the trajectory
    time_traj(:,1) = 0 : 1/Nc : (Tf-T0);
    [Q_traj,QD_traj,QDD_traj] = jtraj(q0,qf,time_traj);

    % Merging the two parts
    Q = Q_traj;                        % Position vector
    QD = QD_traj;                      % Velocity vector
    QDD = QDD_traj;                    % Acceleration vector
    time = (T0 : 1/Nc : Tf)';          % Time vector

end

