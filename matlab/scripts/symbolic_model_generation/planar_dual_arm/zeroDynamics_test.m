close all
clear
clc

load('B_test_1.mat')
load('n_test_1.mat')

b11 = B(1,1);
b12 = B(1,2);
b13 = B(1,3);
b21 = B(2,1);
b22 = B(2,2);
b23 = B(2,3);
b31 = B(3,1);
b32 = B(3,2);
b33 = B(3,3);
n1 = n(1);
n2 = n(2);
n3 = n(3);



B_new = simplify([b11 b12+b13; b21+b31 b22]);

D = simplify(det(B_new))



% pinvb12 = b12'*inv(b12*b12');
% 
% syms qdd2 real
% ZD = simplify(b12*qdd2 + n1)
% 
% syms q1 qd1 i2xx L1 l2 m1 m2 g0 real
% simplify(subs(ZD,[q1,qd1,i2xx,L1,l2,m1,m2,g0],[0,0,0.1,1,0.25,0.5,0.5,9.8]))

% (0.1250*sin(x)*y^2 - 1.2250*sin(x))/(0.1250*cos(x) + 0.1312)