function [s, sdot, sdotdot, t] = multiJointTrapVelTraj_tf(dt, tFinal, accDes, sInit, sFinal)

    for i=1:length(sInit)
        
        tFinal_i = tFinal(i);
        accDes_i = accDes(i);
        sInit_i = sInit(i);        
        sFinal_i = sFinal(i);
        
        [s_i, sdot_i, sdotdot_i, t_i] = trapVelTraj_tf(dt, tFinal_i, accDes_i, sInit_i, sFinal_i);
        s{i} = s_i';
        sdot{i} = sdot_i';
        sdotdot{i} = sdotdot_i';
        t{i} = t_i';
    end

end