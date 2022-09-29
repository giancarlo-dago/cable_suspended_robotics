% This function provides a cubic timing law of the curvilinear abscissa according
% to the input parameters. A third degree polynomial allows to specify
% initial and final position and velocity.
% The curvilinear abscissa has the following characteristics
%                           - s : s-shaped trend
%                           - sdot : parabolic profile
%                           - sdotdot : linear trend
% INPUT PARAMETERS
%                           - dt : sampling time
%                           - tFinal : trajectory duration
%                           - sInit : initial value of curvilinear abscissa
%                           - sFinal : final value of curvilinear abscissa
%                           - sDotInit : initial value of curvilinear abscissa velocity
%                           - sDotFinal : final value of curvilinear abscissa velocity
% OUTPUT PARAMETERS
%                           - s : curvilinear abscissa trend
%                           - sdot : curvilinear abscissa velocity
%                           - sdotdot : curvilinear abscissa acceleration
%                           - t : trajectory time interval


function [ s, sdot, sdotdot, t ] = cubicVelTraj( dt, tFinal, sInit, sFinal, sDotInit, sDotFinal)

    t=0:dt:tFinal;
    s = 0*t;
    sdot = 0*t;
    sdotdot = 0*t;

    % Computation of the polynomial coefficients
    a0 = sInit;
    a1 = sDotInit;
    a3 = (sDotFinal*tFinal - 2*sFinal + a1*tFinal + 2*a0)/tFinal^3;
    a2 = (sFinal - a3*tFinal^3 - a1*tFinal - a0)/tFinal^2;

    % Computation of the timing law
    for k = 1:length(t)

       s(k) = a3*t(k)^3 + a2*t(k)^2 + a1*t(k) + a0;
       sdot(k) = 3*a3*t(k)^2 + 2*a2*t(k) + a1;
       sdotdot(k) = 6*a3*t(k) + 2*a2;

    end

end

