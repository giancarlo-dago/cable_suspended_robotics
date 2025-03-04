function [s, sdot, sdotdot, t] = concatenatedMultiJointQuinticTraj(dt, tTrajectories, sViaPoints, sDotViaPoints, sDot2ViaPoints)

    nPoints = size(sViaPoints,1);
    nJoints = size(sViaPoints,2);
    
    for i=1:nPoints-1
        for j=1:nJoints
            
            tFinal_i = tTrajectories(i);
            if i==1
                sInit_ij = sViaPoints(1,j);
                sDotInit_ij = sDotViaPoints(1,j);
                s2DotInit_ij = sDot2ViaPoints(1,j);
            else
                sInit_ij = sViaPoints(i,j);
                sDotInit_ij = sDotViaPoints(i,j);
                s2DotInit_ij = sDot2ViaPoints(i,j);
            end
            sFinal_ij = sViaPoints(i+1,j);
            sDotFinal_ij = sDotViaPoints(i+1,j);
            s2DotFinal_ij = sDot2ViaPoints(i+1,j);

            [s_ij, sdot_ij, sdotdot_ij, t_ij] = quinticTraj(dt, tFinal_i, sInit_ij, sFinal_ij, sDotInit_ij, sDotFinal_ij, s2DotInit_ij, s2DotFinal_ij);
            
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

end