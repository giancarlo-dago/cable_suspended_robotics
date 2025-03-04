% close all
clear
clc

addpath('/home/giancarlo/Documents/MATLAB/Phase Portrait Plotter')

%% Parameters
% Parameters cables
iCxx = 0;
LC = 4.53;
lC = 3.64;

% lC = 4;
mC = 206;
fvC = 0.5;
fvA = 0.0;
fsC = 0.0;
fsA = 0.0;

% Arms parameters
iAxx = 0.0;
lA = 0.4157;
mA = 10.2 * 2;

% Gravity
g0 = 9.8;

%% Variables
syms qC qA ...
     qCd qAd ...
     qCdd qAdd real

syms alpha real

% syms eta real
eta = sym('eta',[1 2],'real');

%% Dynamic model
B = [mA*LC^2 + 2*mA*cos(qA)*LC*lA + mA*lA^2 + mC*lC^2 + iAxx + iCxx, mA*lA^2 + LC*mA*cos(qA)*lA + iAxx;
                                  mA*lA^2 + LC*mA*cos(qA)*lA + iAxx,                    mA*lA^2 + iAxx];

n = [- LC*lA*mA*sin(qA)*qAd^2 - 2*LC*lA*mA*qCd*sin(qA)*qAd + fvC*qCd + fsC*sign(qCd) + g0*lA*mA*sin(qA + qC) + LC*g0*mA*sin(qC) + g0*lC*mC*sin(qC);
                                                                         LC*lA*mA*sin(qA)*qCd^2 + fvA*qAd + fsA*sign(qAd) + g0*lA*mA*sin(qA + qC)];


% eta2dot = -n(1)/B(1,1) - B(1,2)/B(1,1)*qAdd;
eta2dot = -n(1)/B(1,1);
% eta2dot_uncontrolled = subs(eta2dot,[qA qAd qC qCd],[0 0 eta(1) eta(2)])
% eta2dot_controlled = subs(eta2dot,[qA qAd qC qCd],[alpha*atan(eta(2)) 0 eta(1) eta(2)])
eta2dot_controlled = subs(eta2dot,[qA qAd qC qCd],[alpha*sign(eta(2)) 0 eta(1) eta(2)])
% eta2dot_controlled_2 = subs(eta2dot,[qA qAd qC qCd],[2/pi*alpha*atan(eta(2)) 2/pi*alpha*(1/(1+eta(2)^2)) eta(1) eta(2)])

%%
% tSpan = 50;
% posLimit = [-pi pi];
% velLimit = [-pi pi];
% xPlotNum = 1;
% yPlotNum = 0;
% 
% odefun = @(t, eta) [eta(2); -0.0155*eta(2)-2.5824*sin(eta(1))];
% plotpp(odefun,'tspan', tSpan,...
%               'quivercolor', [0.6,0.6,0.6],...
%               'linecolor', [0.3,0.3,0.3], ...
%               'xlim', posLimit, ...
%               'ylim', velLimit, ...
%               'xPlotNum', xPlotNum, ...
%               'yPlotNum', yPlotNum)
% 
% alpha = 1;
% odefun = @(t, eta) [eta(2);-(50*eta(2) + 83.1067*sin(eta(1) + alpha*sign(eta(2))) + 8.2541e+03*sin(eta(1)))/(76.8313*cos(alpha*sign(eta(2))) + 3.1516e+03)];
% plotpp(odefun,'tspan', tSpan,...
%               'quivercolor', [0.6,0.6,0.6],...
%               'linecolor', [0.3,0.3,0.3], ...
%               'xlim', posLimit, ...
%               'ylim', velLimit, ...
%               'xPlotNum', xPlotNum, ...
%               'yPlotNum', yPlotNum, ...
%               'quiverDensity',5)
