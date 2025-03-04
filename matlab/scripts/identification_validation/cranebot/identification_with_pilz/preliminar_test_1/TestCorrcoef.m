clc
clear



% Define the signals
t = 0:0.01:5;      % Time vector
x1 = 2*exp(-0.2*t).*sin(2*pi*t+pi/2);  % First signal
x2 = 2*exp(-0.5*t).*sin(2*pi*t+pi/2);  % Second signal
plot(t,[x1;x2])

R = corrcoef(x1,x2);
r = abs(R(1,2))
% r^2