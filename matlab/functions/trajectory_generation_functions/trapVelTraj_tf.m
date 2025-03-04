% This function returns the timing law of the curvilinear abscissa according
% to the input parameters. 
% The curvilinear abscissa has the following characteristics
%                           - s : parabolic linear trend
%                           - sdot : trapezoidal profile
%                           - sdotdot : piecewise function
% INPUT PARAMETERS
%                           - dt : sampling time
%                           - acc : desired cruise acceleration
%                           - tFinal : trajectory duration
%                           - sInit : initial value of curvilinear abscissa
%                           - sFinal : final value of curvilinear abscissa
% OUTPUT PARAMETERS
%                           - s : curvilinear abscissa trend
%                           - sdot : curvilinear abscissa velocity
%                           - sdotdot : curvilinear abscissa acceleration
%                           - t : trajectory time interval

function [s, sdot, sdotdot, t] = trapVelTraj_tf(dt, tFinal, accDes, sInit, sFinal)
  
    % Find feasible acceleration and accelerationTime
    sqrtArg = (tFinal^2*accDes - 4*abs(sFinal-sInit))/accDes;

    if (sqrtArg < 0)
        acc = 4*abs(sFinal-sInit)/tFinal^2;
        tAcc = tFinal/2;
    else
        acc = accDes;
        tAcc = tFinal/2 - 0.5*sqrt(sqrtArg);
    end
    
    % Check on the signum of the acceleration
    if ((sFinal-sInit)<0)
        acc = -acc;
    end
  
    % Find cruiseVel
    cruiseVel = tAcc*acc;
    
    t=0:dt:tFinal;
    s = 0*t;
    sdot = 0*t;
    sdotdot = 0*t;

    % Curvilinear Abscissa generator
    for k = 1:length(s)
        time = t(k);
        
        s(k) =       (sInit + 0.5*acc*time^2)               *u(time)*u(tAcc-time) + ...
                     (sInit + cruiseVel*(time - tAcc/2))    *u0(time-tAcc)*u(tFinal-tAcc-time)+ ... 
                     (sFinal- 0.5*acc*(tFinal-time)^2)      *u0(time-(tFinal-tAcc))*u(tFinal-time)+ ...
                     sFinal                                 *u0(time-tFinal);
            
        sdot(k) =    acc*time                               *u(time)*u(tAcc-time) + ...
                     cruiseVel                              *u0(time-tAcc)*u(tFinal-tAcc-time)+ ... 
                     (cruiseVel-acc*(time-(tFinal-tAcc)))   *u0(time-(tFinal-tAcc))*u(tFinal-time)+ ...
                     0                                      *u0(time-tFinal);
               
        sdotdot(k) = acc                                    *u(time)*u(tAcc-time) + ...
                     0                                      *u0(time-tAcc)*u(tFinal-tAcc-time)+ ...
                     -acc                                   *u0(time-(tFinal-tAcc))*u(tFinal-time)+ ...
                     0                                      *u0(time-tFinal);
    end
end


function y = u(t)

    if t>=0
        y=1;
    else
        y=0;
    end

end

function y = u0(t)

    if t>0
        y=1;
    else
        y=0;
    end

end
