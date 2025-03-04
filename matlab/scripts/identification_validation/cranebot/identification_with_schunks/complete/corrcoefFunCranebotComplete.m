function cost = corrcoefFunCranebotComplete(p, qReal, qActive0, qdActive0, qPassive0, qRef, qdRef, Ts_sim, meanTs, sampling, nSamples, nTraj)

    parfor i=1:nTraj                                                        % For each trajectory
        tModel = (0:Ts_sim*sampling:meanTs(i)*nSamples(i))';
        
        qModel = zeros(length(tModel),16);                                  % Define vector for the position
        qdModel = zeros(length(tModel),16);                                 % Define vector for the velocity

        qModel(1,1:4) =  reshape(qPassive0{i},1,4);                         % Initialize the passive joint
        qModel(1,5:16) = reshape(qActive0{i},1,12);                         % Initialize qModel with the starting condition of the active joints
        qdModel(1,1:4) = zeros(1,4);                                        % Initialize qdModel with zero velocity for the passive joints
        qdModel(1,5:16) = reshape(qdActive0{i},1,12);                       % Initialize qdModel with the starting condition of the active joints

        qRef_i = qRef{i};                                                   % Define qRef for the i-th trajectory
        qdRef_i = qdRef{i};                                                 % Define qdRef_i for the i-th trajectory
        
        for k=2:(length(tModel))                                                                                                   % For each sample
            [qModel(k,:), qdModel(k,:)] = qFunCranebotComplete(p,qModel(k-1,:),qdModel(k-1,:),Ts_sim,qRef_i(k-1,:),qdRef_i(k-1,:));        % Compute the dynamics iteratively
        end                                                                                                                        % End-for
        
        R1 = corrcoef(qReal{i}(:,1),qModel(:,1));                           % Compute the correlation
        R2 = corrcoef(qReal{i}(:,2),qModel(:,2));                           % Compute the correlation
        r1 = abs(R1(1,2));                                                  % 
        r2 = abs(R2(1,2));                                                  % 
        TF1 = isnan(r1);                                                    % 
        TF2 = isnan(r2);                                                    % 
        r1(TF1) = 0;                                                        % 
        r1(TF2) = 0;                                                        % 
        coeff(:,i) = [r1 r2]';                                              % 
    end                                                                     % End-for
    cost = -sum(sum(coeff));                                                % Sum the correlations over the joints and take the sum over the trajectories (- is for the minimization)
end
