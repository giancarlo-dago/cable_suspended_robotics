function [s, sdot, sdotdot, t] = concatenatedMultiJointTrapVelTraj_tf(dt, tFinal, accDes, sInit, sFinal)

    nPoints = size(sInit,1);
    nJoints = size(sInit,2);

    for i=1:nPoints-1
        for j=1:nJoints

            tFinal_i = tFinal(i);
            if i==1
                accDes_ij = accDes(1,j);
                sInit_ij = sInit(1,j);        
                sFinal_ij = sFinal(1,j);
            else
                accDes_ij = accDes(i,j);
                sInit_ij = sInit(i,j);        
                sFinal_ij = sFinal(i,j);
            end
                
            [s_ij, sdot_ij, sdotdot_ij, t_ij] = trapVelTraj_tf(dt, tFinal_i, accDes_ij, sInit_ij, sFinal_ij);
    
            s_temp{j} = s_ij';
            sdot_temp{j} = sdot_ij';
            sdotdot_temp{j} = sdotdot_ij';
            t_temp{j} = t_ij';

        end


        if exist('s','var') == 0
            s = s_temp;
            sdot = sdot_temp;
            sdotdot = sdotdot_temp;
            t = t_temp;
        else
            for j=1:nJoints
                s{j} = [s{j}; s_temp{j}];
                sdot{j} = [sdot{j}; sdot_temp{j}];
                sdotdot{j} = [sdotdot{j}; sdotdot_temp{j}];
                t{j} = [t{j}; t_temp{j}+t{j}(end)];
            end
        end


end