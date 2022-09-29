%% Generate trajectory
% [q_traj_x, qd_traj_x, qdd_traj_x, time_traj_x] = trapVelTraj(samplingTime, MaxAccelerationDrone, PeakVelocityDrone, q0(1), qf(1));
% [q_traj_y, qd_traj_y, qdd_traj_y, time_traj_y] = trapVelTraj(samplingTime, MaxAccelerationDrone, PeakVelocityDrone, q0(2), qf(2));
% [q_traj_x, qd_traj_x, qdd_traj_x, time_traj_x] = trapVelTraj_tf(samplingTime, T_traj, MaxAccelerationDrone, q0(1), qf(1));
% [q_traj_y, qd_traj_y, qdd_traj_y, time_traj_y] = trapVelTraj_tf(samplingTime, T_traj, MaxAccelerationDrone, q0(2), qf(2));
[q_traj_x, qd_traj_x, qdd_traj_x, time_traj_x] = quinticVelTraj(samplingTime, T_traj, initial_drone_position(1), desired_drone_position(1), 0, 0, 0, 0);
[q_traj_y, qd_traj_y, qdd_traj_y, time_traj_y] = quinticVelTraj(samplingTime, T_traj, initial_drone_position(2), desired_drone_position(2), 0, 0, 0, 0);

time_regime = time_traj_x(end) + samplingTime : samplingTime : (time_traj_x(end)+T_regime) - samplingTime;
q_drone_reference = [q_traj_x, q_traj_x(end)*ones(1,length(time_regime)); q_traj_y, q_traj_y(end)*ones(1,length(time_regime))]; 
qd_drone_reference = [qd_traj_x, zeros(1,length(time_regime)); qd_traj_y, qd_traj_y(end)*ones(1,length(time_regime))]; 
qdd_drone_reference = [qdd_traj_x, zeros(1,length(time_regime)); qdd_traj_y, qdd_traj_y(end)*ones(1,length(time_regime))]; 
total_time = [time_traj_x, time_regime];

%% Plot velocity profile
figure('NumberTitle','off','Name','Drone Velocity Profile','WindowState','Maximized')
sgtitle('Drone references','FontSize',15,'FontWeight','bold')
subplot(3,1,1);
plot(total_time, q_drone_reference); grid; title('Position'), xlabel('Time [s]'), ylabel('Position [m]'), xlim([total_time(1) total_time(end)])
subplot(3,1,2);
plot(total_time, qd_drone_reference); grid; title('Velocity'), xlabel('Time [s]'), ylabel('Velocity [m/s]'), xlim([total_time(1) total_time(end)])
subplot(3,1,3);
plot(total_time, qdd_drone_reference); grid; title('Acceleration'), xlabel('Time [s]'), ylabel('Acceleration [m/s^2]'), xlim([total_time(1) total_time(end)])


