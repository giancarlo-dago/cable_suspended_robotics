
% function y = modelPlanarArms(expAxis,lCz,mC,fvC,fsC,kd,kp,iA1xx,iA2xx,iCxx,Ts_sim,platformPose0,qRef,qdRef,t)
% function y = modelPlanarArms(expAxis,iCzz,kdZ,kpZ,fvC,fsC,Ts_sim,platformPose0,qRef,qdRef,t)
function y = modelPlanarArms(expAxis,p,Ts_sim,platformPose0,qRef,qdRef,t)
        
        timeLength = length(t);
        L = 3.897; % Is the distance first joint - aruco marker
%         L = 3.65;
        
        if (expAxis == 'X' || expAxis == 'Y')
            qModel = zeros(timeLength,5);                                  % Define vector for the position
            qdModel = zeros(timeLength,5);                                 % Define vector for the velocity
            qModel(1,1) = asin(platformPose0/L);                       % Initialize the cable joint from the starting condition of the shoulder pose
            qModel(1,2:5) = [0 0 0 0];                                     % Initialize qModel with the starting condition of the active joints
            qdModel(1,1:5) = zeros(1,5);                                   % Initialize qdModel with zero velocity for all the joints
        elseif (expAxis == 'Z')
            qModel = zeros(timeLength,3);                                  % Define vector for the position
            qdModel = zeros(timeLength,3);                                 % Define vector for the velocity
            qModel(1,1) = platformPose0;                                   % Initialize the cable joint from the starting condition of the shoulder pose
            qModel(1,2:3) = [0 0];                                         % Initialize qModel with the starting condition of the active joints
            qdModel(1,1:3) = zeros(1,3);                                   % Initialize qdModel with zero velocity for all the joints
        end

%         if (expAxis == 'X' || expAxis == 'Y')
%             p = [lCz,mC,fvC,fsC,kd,kp,iA1xx,iA2xx,iCxx];
%         else
%             p = [iCzz,fvC,fsC,kd,kp,kdZ,kpZ];
%         end
        
        for k=2:timeLength                                                                                                % For each sample
            [qModel(k,:), qdModel(k,:)] = qFunPlanarArms(p,qModel(k-1,:),qdModel(k-1,:),Ts_sim,qRef(k-1,:),qdRef(k-1,:),expAxis);        % Compute the dynamics iteratively
        end                                                                                                                        % End-for

        if (expAxis == 'X' || expAxis == 'Y')
            y = L*sin(qModel(:,1));                                                    % Compute the position of the shoulders again
        elseif (expAxis == 'Z')
            y = qModel(:,1);
        end

%         figure()
%         plot(qModel(:,2)), hold on
%         plot(qRef(:,1:4))
% 
%         figure()
%         plot(qdModel(:,2)), hold on
%         plot(qdRef(:,1:4))


end
