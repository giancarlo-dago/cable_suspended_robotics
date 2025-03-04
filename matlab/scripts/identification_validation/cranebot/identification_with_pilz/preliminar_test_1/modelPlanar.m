
function y = modelPlanar(lCz,mC,fvC,fsC,Ts_sim,platformPose0,qRef,qdRef,t)
        
        timeLength = length(t);
        L = 3.897;
        
        qModel = zeros(timeLength,3);                                  % Define vector for the position
        qdModel = zeros(timeLength,3);                                 % Define vector for the velocity
        qModel(1,1) = asin(platformPose0/L);                          % Initialize the cable joint from the starting condition of the shoulder pose
        qModel(1,2:3) = [0 0];                         % Initialize qModel with the starting condition of the active joints
        qdModel(1,1:3) = zeros(1,3);                                      % Initialize qdModel with zero velocity for all the joints

%         qRef_i = zeros(timeLength,2);                                                   % Define qRef for the i-th trajectory
%         qdRef_i = zeros(timeLength,2);                                                 % Define qdRef_i for the i-th trajectory

        p = [lCz,mC,fvC,fsC];
        
        for k=2:timeLength                                                                                                % For each sample
            [qModel(k,:), qdModel(k,:)] = qFunPlanar(p,qModel(k-1,:),qdModel(k-1,:),Ts_sim,qRef(k-1,:),qdRef(k-1,:));        % Compute the dynamics iteratively
        end                                                                                                                        % End-for

        y = L*sin(qModel(:,1));                                                    % Compute the position of the shoulders again

end
