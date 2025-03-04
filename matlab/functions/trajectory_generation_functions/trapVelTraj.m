% This function returns the timing law of the curvilinear abscissa accDesording
% to the input parameters. 
% The curvilinear abscissa has the following characteristics
%                           - s : parabolic linear trend
%                           - sdot : trapezoidal profile
%                           - sdotdot : piecewise function
% INPUT PARAMETERS
%                           - dt : sampling time
%                           - accDes : desired cruise accDeseleration
%                           - velDes : desired cruise velocity
%                           - sInit : initial value of curvilinear abscissa
%                           - sFinal : final value of curvilinear abscissa
% OUTPUT PARAMETERS
%                           - s : curvilinear abscissa trend
%                           - sdot : curvilinear abscissa velocity
%                           - sdotdot : curvilinear abscissa accDeseleration
%                           - t : trajectory time interval

function [s, sdot, sdotdot, t] = trapVelTraj(dt, accDes, velDes, sInit, sFinal)

    % Check on the signum of velocity and acceleration
    if (sFinal-sInit<0)
        velDes = -velDes;
        accDes = -accDes;
    end

    velPerc = 1;
    
    % Find tFinal
    cruiseVel=velPerc*velDes;
    tAcc=abs(cruiseVel/accDes);
    tFinal=(accDes*tAcc^2+sFinal-sInit)/(accDes*tAcc);

    % Check on the trajectory feasibility
    while abs(cruiseVel) >= 2*norm(sFinal-sInit)/tFinal || abs(cruiseVel)<= norm(sFinal-sInit)/tFinal
        velPerc=velPerc-0.001;
        cruiseVel=velPerc*velDes;
        tAcc=abs(cruiseVel/accDes);
        tFinal=(accDes*tAcc^2+sFinal-sInit)/(accDes*tAcc);
    end
    
    t=0:dt:tFinal;
    s = 0*t;
    sdot = 0*t;
    sdotdot = 0*t;

    % Curvilinear Abscissa generator
    for k = 1:length(s)
        time = t(k);
        s(k) =       (sInit + 0.5*accDes*time^2)               *u(time)*u(tAcc-time) + ...
                     (sInit + cruiseVel*(time - tAcc/2))    *u0(time-tAcc)*u(tFinal-tAcc-time)+ ... 
                     (sFinal- 0.5*accDes*(tFinal-time)^2)      *u0(time-(tFinal-tAcc))*u(tFinal-time)+ ...
                     sFinal                                 *u0(time-tFinal);
            
        sdot(k) =    accDes*time                               *u(time)*u(tAcc-time) + ...
                     cruiseVel                              *u0(time-tAcc)*u(tFinal-tAcc-time)+ ... 
                     (cruiseVel-accDes*(time-(tFinal-tAcc)))   *u0(time-(tFinal-tAcc))*u(tFinal-time)+ ...
                     0                                      *u0(time-tFinal);
               
        sdotdot(k) = accDes                                    *u(time)*u(tAcc-time) + ...
                     0                                      *u0(time-tAcc)*u(tFinal-tAcc-time)+ ...
                     -accDes                                   *u0(time-(tFinal-tAcc))*u(tFinal-time)+ ...
                     0                                      *u0(time-tFinal);
    end
end

% Step function with y = 0 at t = 0
function y = u(t)

    if t>=0
        y=1;
    else
        y=0;
    end

end

% Step function with y = 1 at t = 0
function y = u0(t)

    if t>0
        y=1;
    else
        y=0;
    end

end

