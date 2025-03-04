
if ispc % Windows
    addpath('..\..\..\functions/trajectory_generation_functions')
else % Linux
    addpath('../../../functions/trajectory_generation_functions')
end

%% Trolley - Cross Travel
MinVel_trolley = 0.1666666;                            % [m/s]
MaxVel_trolley = 0.3333333;                            % [m/s]
FirstAcc_trolley = 0.40;                               % [m/s^2]
SecondAcc_trolley = 0.22;                              % [m/s^2]
q0_trolley = 0;                                        % [m]
qf_trolley = 16.98;                                       % [m]
samplingTime = 0.001;
[q_trolley_traj,qd_trolley_traj,qdd_trolley_traj,time_trolley_traj] = doubleRampTrapVelTraj(samplingTime, FirstAcc_trolley, SecondAcc_trolley, MinVel_trolley, MaxVel_trolley, q0_trolley, qf_trolley);

T_regime = 10;                         % [sec]
time_regime = time_trolley_traj(end) : samplingTime : (time_trolley_traj(end)+T_regime);
q_trolley = [q_trolley_traj, q_trolley_traj(end)*ones(1,length(time_regime))]; 
qd_trolley = [qd_trolley_traj, zeros(1,length(time_regime))]; 
qdd_trolley = [qdd_trolley_traj, zeros(1,length(time_regime))]; 
total_time = [time_trolley_traj, time_regime];

%% Save in a timeseries structure
d0_ref = timeseries(q_trolley, total_time);
dd0_ref = timeseries(qd_trolley, total_time);
ddd0_ref = timeseries(qdd_trolley, total_time);

%% Plot trolley velocity profile
figure('NumberTitle','off','Name','CMS Cavern overhead crane - Trolley Velocity Profile','WindowState','Maximized')
sgtitle('927 Cavern overhead crane - Trolley Velocity Profile','FontSize',15,'FontWeight','bold')
subplot(3,1,1);
plot(total_time, q_trolley); grid; title('Position'), xlabel('Time [s]'), ylabel('Position [m]'), xlim([total_time(1) total_time(end)])
subplot(3,1,2);
plot(total_time, qd_trolley); grid; title('Velocity'), xlabel('Time [s]'), ylabel('Velocity [m/s]'), xlim([total_time(1) total_time(end)])
subplot(3,1,3);
plot(total_time, qdd_trolley); grid; title('Acceleration'), xlabel('Time [s]'), ylabel('Acceleration [m/s^2]'), xlim([total_time(1) total_time(end)])


