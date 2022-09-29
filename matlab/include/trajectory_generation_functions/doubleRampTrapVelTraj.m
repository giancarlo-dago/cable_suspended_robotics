% This function returns the timing law of the curvilinear abscissa according
% to the input parameters. 
% The curvilinear abscissa has the following characteristics
%                           - s : parabolic linear trend
%                           - sdot : trapezoidal profile
%                           - sdotdot : piecewise function
% INPUT PARAMETERS
%                           - dt : sampling time
%                           - acc : desired cruise acceleration
%                           - velDes : desired cruise velocity
%                           - sInit : initial value of curvilinear abscissa
%                           - sFinal : final value of curvilinear abscissa
% OUTPUT PARAMETERS
%                           - s : position
%                           - sdot : velocity
%                           - sdotdot : acceleration
%                           - t : trajectory time interval

function [s, sdot, sdotdot, t] = doubleRampTrapVelTraj (dt, firstAcc, secondAcc, firstVel, secondVel, sInit, sFinal)

    tFirstAcc = abs(firstVel/firstAcc);
    tSecondAcc = abs(abs(secondVel-firstVel)/secondAcc);
    tAcc = tFirstAcc + tSecondAcc;
    fakeAcc = secondVel/tAcc;

    tFinal = (fakeAcc*tAcc^2+sFinal-sInit)/(fakeAcc*tAcc);

    %     % Check on the trajectory feasibility
    %     while abs(cruiseVel) >= 2*norm(sFinal-sInit)/tFinal || abs(cruiseVel)<= norm(sFinal-sInit)/tFinal
    %         velPerc=velPerc-0.05;
    %         cruiseVel=velPerc*velDes;
    %         tAcc=abs(cruiseVel/acc);
    %         tFinal=(acc*tAcc^2+sFinal-sInit)/(acc*tAcc);
    %     end

    t = 0:dt:tFinal;
    s = 0*t;
    sdot = 0*t;
    sdotdot = 0*t;
    
    % Time instants
    t1 = tFirstAcc;
    t2 = tFirstAcc + tSecondAcc;
    t3 = tFinal - tFirstAcc - tSecondAcc;
    t4 = tFinal - tFirstAcc;
    t5 = tFinal;
    
    % Accelerations
    A1 = firstAcc;
    A2 = secondAcc;
    
    % Velocity
    V1 = firstVel;
    V2 = secondVel;
    
    % Positions
    S0 = sInit;
    S1 = S0 + (1/2)*A1*t1^2;
    S2 = S1 + V1*(t2-t1) + (1/2)*A2*(t2-t1)^2;
    S3 = S2 + V2*(t3-t2);
    S4 = S3 + V2*(t4-t3) - (1/2)*A2*(t4-t3)^2;
    S5 = sFinal;

    % Curvilinear Abscissa generator
    for k = 1:length(s)
        time = t(k);
                 
        if time==0
            s(k) = S0;
            sdot(k) = 0;
            sdotdot(k) = 0;
        elseif (time>0 && time<=t1)
            s(k) = S0 + (1/2)*A1*time^2;
            sdot(k) = A1*time;
            sdotdot(k) = A1;  
        elseif (time>t1 && time<=t2)
            s(k) = S1 + V1*(time-t1) + (1/2)*A2*(time-t1)^2;
            sdot(k) = V1 + A2*(time-t1);
            sdotdot(k) = A2;  
        elseif (time>t2 && time<=t3)
            s(k) = S2 + V2*(time-t2);
            sdot(k) = V2;
            sdotdot(k) = 0;
        elseif (time>t3 && time<=t4)
            s(k) = sFinal;
            s(k) = S3 + V2*(time-t3) - (1/2)*A2*(time-t3)^2;
            sdot(k) = V2 - A2*(time-t3);
            sdotdot(k) = -A2;
        elseif (time>t4 && time<t5)   
            s(k) = S4 + V1*(time-t4) - (1/2)*A1*(time-t4)^2;
            sdot(k) = V1 - A1*(time-t4);
            sdotdot(k) = -A1;
        else    
            s(k) = S5;
            sdot(k) = 0;
            sdotdot(k) = 0;
        end

    end
    
end
