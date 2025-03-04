close all
clear
clc

% load('B_symbolic.mat')
% load('n_symbolic.mat')
load('B_symbolic_with_friction.mat')
load('n_symbolic_with_friction.mat')

b11 = B(1,1);
b12 = B(1,2);
b21 = B(2,1);
b22 = B(2,2);
n1 = n(1);
n2 = n(2);

pinvb12 = b12'*inv(b12*b12');

syms qdd2 real
ZD = simplify(b12*qdd2 + n1)

syms q1 qd1 i2xx L1 l2 m1 m2 g0 real
simplify(subs(ZD,[q1,qd1,i2xx,L1,l2,m1,m2,g0],[0,0,0.1,1,0.25,0.5,0.5,9.8]))

% (0.1250*sin(x)*y^2 - 1.2250*sin(x))/(0.1250*cos(x) + 0.1312)