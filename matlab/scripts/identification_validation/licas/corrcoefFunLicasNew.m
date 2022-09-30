function cost = corrcoefFunLicasNew(p, qRealActive, shoulderPoseReal, qActive0, qdActive0, shoulderPose0, qRef, qdRef, Ts_sim, meanTs, sampling, nSamples, nTraj)

    L = 1.0;
    
    parfor i=1:nTraj                                                        % For each trajectory        
        tModel = (0:Ts_sim*sampling:meanTs(i)*nSamples(i))';

        qModel = zeros(length(tModel),14);                                  % Define vector for the position
        qdModel = zeros(length(tModel),14);                                 % Define vector for the velocity

        qModel(1,1:3) = reshape([shoulderPose0{i}(3) ...
                                 asin(shoulderPose0{i}(2)/L) ...
                                 asin(shoulderPose0{i}(1)/L)],1,3);         % Initialize the cable joint from the starting condition of the shoulder pose
        qModel(1,4:6) = [0 -qModel(1,2) -qModel(1,3)];                      % Initialize the shoulder joint in order to have the shoulders horizontal
        qModel(1,7:14) = qActive0{i};                                       % Initialize qModel with the starting condition of the active joints
        qdModel(1,1:6) = zeros(1,6);                                        % Initialize qdModel with zero velocity for the passive joints
        qdModel(1,7:14) = qdActive0{i};                                     % Initialize qdModel with the starting condition of the active joints

        qRef_i = qRef{i};                                                   % Define qRef for the i-th trajectory
        qdRef_i = qdRef{i};                                                 % Define qdRef_i for the i-th trajectory
        qRealActive_i = qRealActive{i};                                     % Define qReal_i for the i-th trajectory
                
        for k=2:(length(tModel))                                                                                                    % For each sample
            [qModel(k,:), qdModel(k,:)] = qFunModelLicasNew(p,qModel(k-1,:),qdModel(k-1,:),Ts_sim,qRef_i(k-1,:),qdRef_i(k-1,:));       % Compute the dynamics iteratively
        end                                                                                                                         % End-for
        shoulderPoseModel = [L*sin(qModel(:,3)) L*sin(qModel(:,2)) qModel(:,1)+qModel(:,4)];                                                    % Compute the position of the shoulders again
        
        R1 = corrcoef(shoulderPoseReal{i}(:,1),shoulderPoseModel(:,1));     % Compute the correlation 
        R2 = corrcoef(shoulderPoseReal{i}(:,2),shoulderPoseModel(:,2));     %
        R3 = corrcoef(shoulderPoseReal{i}(:,3),shoulderPoseModel(:,3));     %

        r1 = abs(R1(1,2));                                                  %
        r2 = abs(R2(1,2));                                                  %
        r3 = abs(R3(1,2));                                                  %

        if i==1 || i==2
        	rVec = [r1 0 0];                                                        %
        elseif i==3 || i==4
        	rVec = [0 r2 0];                                                        %     
        elseif i==5 || i==6
        	rVec = [0 0 r3];                                                        %     
        end
        
%         rVec = r1;                                                    %
%         rVec = r2;                                                    %
%         rVec = r3;                                                    %
%         rVec = [r1 r2];                                                        
%         rVec = [r1 r2 r3];                                                        

        TF = isnan(rVec);                                                   %
        rVec(TF) = 0;                                                       %
        coeff(:,i) = rVec';                                                 %
    end                                                                     % End-for

    cost = -sum(sum(coeff));                                                % Sum the correlations over the joints and take the sum over the trajectories (- is for the minimization)

end

