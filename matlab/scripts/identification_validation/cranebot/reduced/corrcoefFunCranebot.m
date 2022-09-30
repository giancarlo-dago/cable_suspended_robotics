% function cost = corrcoefFunCranebot(p, shoulderPoseReal, shoulderPose0, Ts_sim, meanTs, sampling, nSamples, nTraj)
% 
%     L = p(1);                                                               % Isolate the parameter of the length of the cable
%     
%     parfor i=1:nTraj                                                        % For each trajectory        
%         tModel = (0:Ts_sim*sampling:meanTs(i)*nSamples(i))';
% 
%         qModel = zeros(length(tModel),16);                                  % Define vector for the position
%         qdModel = zeros(length(tModel),16);                                 % Define vector for the velocity
% 
%         qModel(1,1:2) = reshape([asin(shoulderPose0{i}(2)/L) ...
%                                  asin(shoulderPose0{i}(1)/L)],1,2);           % Initialize the cable joint from the starting condition of the shoulder pose
%         qModel(1,3:4) = zeros(1,2);                                           % Initialize the shoulder joint in order to have the shoulders horizontal
%         qModel(1,5:16) = zeros(1,12);                                         % Initialize qModel with the starting condition of the active joints
%         qdModel(1,1:4) = zeros(1,4);                                          % Initialize qdModel with zero velocity for the passive joints
%         qdModel(1,5:16) = zeros(1,12);                                        % Initialize qdModel with the starting condition of the active joints
% 
%         qRef_i = zeros(1,12);                                                   % Define qRef for the i-th trajectory
%         qdRef_i = zeros(1,12);                                                 % Define qdRef_i for the i-th trajectory
%                 
%         for k=2:(length(tModel))                                                                                                    % For each sample
%             [qModel(k,:), qdModel(k,:)] = qFunCranebot(p,qModel(k-1,:),qdModel(k-1,:),Ts_sim,qRef_i(k-1,:),qdRef_i(k-1,:));       % Compute the dynamics iteratively
%         end                                                                                                                         % End-for
%         shoulderPoseModel = [L*sin(qModel(:,1)) L*sin(qModel(:,2))];                                                    % Compute the position of the shoulders again
% %         
% %         R1 = corrcoef(shoulderPoseReal{i}(:,1),shoulderPoseModel(:,1));     % Compute the correlation 
% %         R2 = corrcoef(shoulderPoseReal{i}(:,2),shoulderPoseModel(:,2));     %
% %         R3 = corrcoef(shoulderPoseReal{i}(:,3),shoulderPoseModel(:,3));     %
% %         r1 = abs(R1(1,2));                                                  %
% %         r2 = abs(R2(1,2));                                                  %
% %         r3 = abs(R3(1,2));                                                  %
% %         TF = isnan(rVec);                                                   %
% %         rVec(TF) = 0;                                                       %
% %         coeff(:,i) = rVec';                                                 %
%     end                                                                     % End-for
% 
%     cost = 0;
% %     cost = -sum(sum(coeff));                                                % Sum the correlations over the joints and take the sum over the trajectories (- is for the minimization)
% end


function cost = corrcoefFunCranebot(p, qReal, shoulderPoseReal, qActive0, qdActive0, qPassive0, shoulderPose0, qRef, qdRef, Ts_sim, meanTs, sampling, nSamples, nTraj)

    L = p(1);                                                               % Isolate the parameter of the length of the cable
    parfor i=1:nTraj                                                        % For each trajectory
        tModel = (0:Ts_sim*sampling:meanTs(i)*nSamples(i))';
        
        qModel = zeros(length(tModel),14);                                  % Define vector for the position
        qdModel = zeros(length(tModel),14);                                 % Define vector for the velocity
        
%         qModel(1,1) = asin(shoulderPose0{i}(1)/L);                        % Initialize the cable joint from the starting condition of the shoulder pose
%         qModel(1,2) = zeros(1,1);                                         % Initialize the shoulder joint in order to have the shoulders horizontal
%         qModel(1,3:14) = reshape(qActive0{i},1,12);                       % Initialize qModel with the starting condition of the active joints
%         qdModel(1,1:2) = zeros(1,2);                                      % Initialize qdModel with zero velocity for the passive joints
%         qdModel(1,3:14) = reshape(qdActive0{i},1,12);                     % Initialize qdModel with the starting condition of the active joints
        
        qModel(1,1:2) =  reshape(qPassive0{i},1,2);                        % Initialize the passive joint
        qModel(1,3:14) = reshape(qActive0{i},1,12);                         % Initialize qModel with the starting condition of the active joints
        qdModel(1,1:2) = zeros(1,2);                                        % Initialize qdModel with zero velocity for the passive joints
        qdModel(1,3:14) = reshape(qdActive0{i},1,12);                       % Initialize qdModel with the starting condition of the active joints

        qRef_i = qRef{i};                                                   % Define qRef for the i-th trajectory
        qdRef_i = qdRef{i};                                                 % Define qdRef_i for the i-th trajectory
        
        for k=2:(length(tModel))                                                                                                   % For each sample
            [qModel(k,:), qdModel(k,:)] = qFunCranebot(p,qModel(k-1,:),qdModel(k-1,:),Ts_sim,qRef_i(k-1,:),qdRef_i(k-1,:));        % Compute the dynamics iteratively
        end                                                                                                                        % End-for
%         shoulderPoseModel = L*sin(qModel(:,1));                                                                                    % Compute the position of the shoulders again
        
%         R1 = corrcoef(shoulderPoseReal{i}(:,1),shoulderPoseModel(:,1));     % Compute the correlation
        R1 = corrcoef(qReal{i}(:,1),qModel(:,1));     % Compute the correlation
        r1 = abs(R1(1,2));                                                  % 
        TF = isnan(r1);                                                     % 
        r1(TF) = 0;                                                         % 
        coeff(:,i) = r1';                                                   % 
    end                                                                     % End-for
    
    cost = -sum(sum(coeff));                                                % Sum the correlations over the joints and take the sum over the trajectories (- is for the minimization)
end
