

% load('B.mat','B')
% cCode(B,'B_Generic_',false,true)

% load('n.mat', 'n')
% cCode(n,'n_Generic_',false,true)

%% CON SOLO GIUNTO PASSIVO SUPERIORE

close all
clear
clc

% Variables
syms q1 q2 q3 ...
     qd1 qd2 qd3 ...
     qdd1 qdd2 qdd3 real
 
% assume(q1>-pi & q1<pi)
% assume(q>-pi & q<pi)

syms q L m off ixx lz real;

load('B_test_0.mat','B')
Bbm = simplify(B(1,2:3));
if isscalar(Bbm*Bbm')
    determinant = Bbm*Bbm'
else
    determinant = simplify(det(Bbm*Bbm'))
end

L = 1.0504;
off = 0.15;
m = 0.5;
ixx = 0.1;
lz = 0.25;

syms x y real

[solx,soly] = solve(L*cos(x)+off*sin(x) == (-ixx-m*lz^2)/(m*lz), L*cos(y)-off*sin(y) == (-ixx-m*lz^2)/(m*lz))


detFun = matlabFunction(determinant,'File','det_test0_f')

%% CON SOLO GIUNTO PASSIVO SUPERIORE E CON BRACCIA CHE FANNO LO STESSO MOVIMENTO

close all
clear
clc

% Variables
syms q1 q  ...
     qd1 qd ...
     qdd1 qdd real
 
% assume(q1>-pi & q1<pi)
% assume(q>-pi & q<pi)

syms q L m off ixx lz real;

load('B_test_1.mat','B')
Bbm = simplify(B(1,2:3));
if isscalar(Bbm*Bbm')
    determinant = Bbm*Bbm'
else
    determinant = simplify(det(Bbm*Bbm'))
end

detFun = matlabFunction(determinant,'File','det_test1_f')

L = 1.0504;
off = 0.15;
m = 0.5;
ixx = 0.1;
lz = 0.25;

% 
% A = 2*L*lz^3*m^2 + 2*L*ixx*lz
% B = L^2*lz^2*m^2-lz^2*m^2*off^2
% y = ixx^2+lz^4*m^2+2*ixx*lz^2*m+lz^2*m^2*off^2
% 
% A_B = A-B
% B_A = B-A
% 
D = det_test1_f(L,ixx,lz,m,off,q)

q = -2*pi:0.01:2*pi;
plot(q,det_test1_f(L,ixx,lz,m,off,q)), xlim([-2*pi 2*pi]), ylim([0 inf])

% plot
% eqn = D == 0
% fplot([lhs(eqn) rhs(eqn)],[-4*pi 4*pi])
% fsurf([lhs(eqn) rhs(eqn)],[-pi pi])
% 
% % eqn = D == 0
% % S = solve(eqn,q,'Real',true)

%% CON GIUNTO PASSIVO SUPERIORE ED INFERIORE (SENZA ASSUNZIONI SUL SECONDO GIUNTO)


close all
clear
clc

% Variables
syms q1 q2 q  ...
     qd1 qd2 qd ...
     qdd1 qdd2 qdd real
 
assume(q1>-pi & q1<pi)
assume(q2>-pi & q2<pi)
assume(q>-pi & q<pi)

syms L L1 L2 m1 m2 m1L m1R off ...
    i1xx i1yy i1zz i2xx i2yy i2zz i1Lxx i1Lyy i1Lzz i1Rxx i1Ryy i1Rzz ...
    id_1x id_1y id_1z id_2x id_2y id_2z id_1Lx id_1Ly id_1Lz id_1Rx id_1Ry id_1Rz ...
    fv1 fv2 fv1L fv1R g0 real;

load('B_test_2.mat','B')
% Bbm = simplify(B(1:2,3:4))
% determinant = simplify(det(Bbm*Bbm'))
% 
% Bbm = simplify(B(1,3:4));
% determinant = simplify(det(Bbm*Bbm'))

Bbm = simplify(B(2,3:4));
determinant = simplify(det(Bbm*Bbm'))

detFun = matlabFunction(determinant,'File','det_test2_f')
% 
D = det_test2_f(q)
eqn = D == 0
fplot([lhs(eqn) rhs(eqn)],[-2*pi 2*pi])
% fsurf([lhs(eqn) rhs(eqn)],[-2*pi 2*pi]), xlabel('q'), ylabel('q2')

%% LA VARIABILE Q2 VIENE POSTA =-Q1

close all
clear
clc

% Variables
syms q1 q  ...
     qd1 qd ...
     qdd1 qdd real
 
% assume(q1>-pi & q1<pi)
% assume(q>-pi & q<pi)

syms L L1 m1 m2 m off ...
     i1xx i2xx ixx ...
     l1z lz ...
     g0 real;


load('B_test_3.mat','B')
Bbm = simplify(B(1:2,3:4));
determinant = simplify(det(Bbm*Bbm'))
detFun = matlabFunction(determinant,'File','det_test3_f')

% L = 1;
% off = 0.15;
% m = 0.5;
% lz = 0.25;

% D = det_test3_f(L,lz,m,off,q,q1)

% D = det_test3_f(q,q1)
% eqn = D == 0
% % fplot([lhs(eqn) rhs(eqn)],[-2*pi 2*pi])
% fsurf([lhs(eqn) rhs(eqn)],[-pi pi]), xlabel('q'), ylabel('q1')

% --------------

Bbm = simplify(B(1,3:4));

determinant = simplify(det(Bbm*Bbm'))

% subs(determinant,[ixx,lz,m,off],[0.1,0.25,0.5,0.15])

%% LA VARIABILE Q2 VIENE POSTA =-Q1 E LE SHOULDER NON SONO ALLINEATE AI BRACCI (COME CRANEBOT)

close all
clear
clc

% Variables
syms q1 q  ...
     qd1 qd ...
     qdd1 qdd real
 
% assume(q1>-pi & q1<pi)
% assume(q>-pi & q<pi)

syms L D L1 m1 m2 m off ...
     i1xx i2xx ixx ...
     l1z l2z lz ...
     g0 real;


load('B_test_4.mat','B')
Bbm = simplify(B(1:2,3:4));
determinant = simplify(det(Bbm*Bbm'))
detFun = matlabFunction(determinant,'File','det_test4_f')

% L = 1;
% off = 0.15;
% m = 0.5;
% lz = 0.25;

% D = det_test3_f(L,lz,m,off,q,q1)

% D = det_test3_f(q,q1)
% eqn = D == 0
% % fplot([lhs(eqn) rhs(eqn)],[-2*pi 2*pi])
% fsurf([lhs(eqn) rhs(eqn)],[-pi pi]), xlabel('q'), ylabel('q1')

% --------------

Bbm = simplify(B(1,3:4));

determinant = simplify(det(Bbm*Bbm'))
% 
% subs(determinant,[ixx,lz,m,off],[0.1,0.25,0.5,0.15])






