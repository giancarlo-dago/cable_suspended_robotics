% function cost = corrcoefFunCranebotPilz(p, qReal, shoulderPoseReal, qActive0, qdActive0, qPassive0, shoulderPose0, qRef, qdRef, Ts_sim, meanTs, sampling, nSamples, nTraj)
% function cost = corrcoefFunCranebotPilz(vettore_parametri_ignoti, tempo_campionamento)
function cost = corrcoefFunCranebotPilz(param, platformPoseReal, qActive0, qdActive0, platformPose0, qRef, qdRef, Ts_sim, meanTs, sampling, nSamples, nTraj)

    p = [3.286 36 param(1) 35.64 param(2) 0];

    L = p(1);                                                               % Isolate the parameter of the length of the cable
    f = 0.414;


    parfor i=1:nTraj                                                        % For each trajectory
%     for i=1:nTraj                                                           % For each trajectory
        tModel = (0:Ts_sim*sampling:meanTs(i)*nSamples(i))';

        qModel = zeros(length(tModel),13);                                  % Define vector for the position
        qdModel = zeros(length(tModel),13);                                 % Define vector for the velocity
        qModel(1,1) = asin(platformPose0{i}(2)/(L+f));                          % Initialize the cable joint from the starting condition of the shoulder pose
        qModel(1,2:13) = reshape(qActive0{i},1,12);                         % Initialize qModel with the starting condition of the active joints
        qdModel(1,1:13) = zeros(1,13);                                      % Initialize qdModel with zero velocity for all the joints

        qRef_i = qRef{i};                                                   % Define qRef for the i-th trajectory
        qdRef_i = qdRef{i};                                                 % Define qdRef_i for the i-th trajectory


        for k=2:(length(tModel))                                                                                                   % For each sample
            [qModel(k,:), qdModel(k,:)] = qFunCranebotPilz(p,qModel(k-1,:),qdModel(k-1,:),Ts_sim,qRef_i(k-1,:),qdRef_i(k-1,:));        % Compute the dynamics iteratively
        end                                                                                                                        % End-for

        platformPoseModel = [zeros(length(qModel(:,1)),1) (L+f)*sin(qModel(:,1)) zeros(length(qModel(:,1)),1)];                                                    % Compute the position of the shoulders again
        R2 = corrcoef(platformPoseReal{i}(:,2),platformPoseModel(:,2));     %
        r2 = abs(R2(1,2));                                                  %
        rVec = r2;                                                          %

        TF = isnan(rVec);                                                   %
        rVec(TF) = 0;                                                       %
        coeff(:,i) = rVec';                                                 %

%         Fs = 1000;
%         Y = fft(platformPoseReal{i}(:,2));
%         l = length(platformPoseReal{i}(:,2));
%         P2 = abs(Y/l);
%         P1 = P2(1:l/2+1);
%         P1(2:end-1) = 2*P1(2:end-1);
%         f = Fs*(0:(l/2))/l;
%     
%         MX = max(P1);
%         index = find(P1>MX-0.001);
%         T_real = 1/f(index)
    
%         Y2 = fft(platformPoseModel(:,2));
%         l2 = length(platformPoseModel(:,2));
%         P2_2 = abs(Y2/l2);
%         P1_2 = P2(1:l2/2+1);
%         P1_2(2:end-1) = 2*P1_2(2:end-1);
%         f2 = Fs*(0:(l2/2))/l2;
%     
%         MX_2 = max(P1_2);
%         index_2 = find(P1_2>MX_2-0.001);
%         T_model = 1/f2(index_2)
    
%         diff(i) = abs(T_real - T_model);
%         diff

    end                                                                     % End-for
    
%     plot(tModel,platformPoseModel)

    cost = -sum(sum(coeff));                                                % Sum the correlations over the joints and take the sum over the trajectories (- is for the minimization)
    
%     cost = sum(diff(i));
%     cost = 0;
end
