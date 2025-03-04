m1 = 7.1;
m2 = 1.7;
m3 = 4.8;
m4 = 0.9;
m5 = 2.6;
m6 = 0.2;

m = m1+m2+m3+m4+m5+m6;

l1 = 0;
l2 = 0.162;
l3 = 0;
l4 = 0.204;
l5 = 0;
l6 = 0;

L1 = 0.35;          % [m] Length of the first connector
L2 = 0.3070;        % [m] Length of the second connector
L3 = 0.0840;        % [m] Distance last joint to flange

l = (m1*l1 + m2*l2 + m3*(L1+l3) + m4*(L1+l4) + m5*(L1+L2+l5) + m6*(L1+L2) ) / (m1+m2+m3+m4+m5+m6)

R1 = l;
I = m*l^2

I1 = I - m*l^2
