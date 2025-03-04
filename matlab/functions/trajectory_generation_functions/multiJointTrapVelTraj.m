function [s, sdot, sdotdot, t] = multiJointTrapVelTraj(dt, accDes, velDes, sInit, sFinal)

    for i=1:length(sInit)
        
        accDes_i = accDes(i);
        velDes_i = velDes(i);
        sInit_i = sInit(i);        
        sFinal_i = sFinal(i);
        
        [s_i, sdot_i, sdotdot_i, t_i] = trapVelTraj(dt, accDes_i, velDes_i, sInit_i, sFinal_i);

        s{i} = s_i';
        sdot{i} = sdot_i';
        sdotdot{i} = sdotdot_i';
        t{i} = t_i';
    end

end