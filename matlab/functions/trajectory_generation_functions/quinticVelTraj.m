% This function provides a cubic timing law of the curvilinear abscissa according
% to the input parameters. A third degree polynomial allows to specify
% initial and final position and velocity.
%
% INPUT PARAMETERS
%                           - dt : sampling time
%                           - tFinal : trajectory duration
%                           - sInit : initial value of curvilinear abscissa
%                           - sFinal : final value of curvilinear abscissa
%                           - sDotInit : initial value of curvilinear abscissa velocity
%                           - sDotFinal : final value of curvilinear abscissa velocity
%                           - s2DotInit : initial value of curvilinear abscissa acceleration
%                           - s2DotFinal : final value of curvilinear abscissa acceleration
% OUTPUT PARAMETERS
%                           - s : curvilinear abscissa trend
%                           - sdot : curvilinear abscissa velocity
%                           - sdotdot : curvilinear abscissa acceleration
%                           - t : trajectory time interval


function [ s, sdot, sdotdot, t] = quinticVelTraj( dt, tFinal, sInit, sFinal, sDotInit, sDotFinal, s2DotInit, s2DotFinal)

    t=0:dt:tFinal;
    s = 0*t;
    sdot = 0*t;
    sdotdot = 0*t;

    % Computation of the polynomial coefficients
    a0 = sInit;
    a1 = sDotInit;
    a2 = 0.5*s2DotInit;
    a3 = -(20*a0 - 20*sFinal + 12*a1*tFinal + 8*sDotFinal*tFinal + 6*a2*tFinal^2 - s2DotFinal*tFinal^2)/(2*tFinal^3);
    a4 = (30*a0 - 30*sFinal + 16*a1*tFinal + 14*sDotFinal*tFinal + 6*a2*tFinal^2 - 2*s2DotFinal*tFinal^2)/(2*tFinal^4);
    a5 = -(12*a0 - 12*sFinal + 6*a1*tFinal + 6*sDotFinal*tFinal + 2*a2*tFinal^2 - s2DotFinal*tFinal^2)/(2*tFinal^5);

    % Computation of the timing law
    for k = 1:length(t)

       s(k) = a5*t(k)^5 + a4*t(k)^4 + a3*t(k)^3 + a2*t(k)^2 + a1*t(k) + a0;
       sdot(k) = 5*a5*t(k)^4 + 4*a4*t(k)^3 + 3*a3*t(k)^2 + 2*a2*t(k) + a1;
       sdotdot(k) = 20*a5*t(k)^3 + 12*a4*t(k)^2 + 6*a3*t(k) + 2*a2;

    end

end
