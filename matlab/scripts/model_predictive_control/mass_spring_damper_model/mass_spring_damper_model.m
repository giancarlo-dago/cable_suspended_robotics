function model = mass_spring_damper_model() 
%%
import casadi.*

%% system dimensions

nx = 8;
nu = 2;

%% system parameters

M = 206; % platform + arm1L + arm1R
Kx = 572.36;
Ky = 572.36;
Bx = 15;
By = 2.2;
l = 0.4157; %arm2 lenght
m = 10.2*2; %arm2 mass
% Iz = 0.380*2;


% I_tot = Iz + m*l^2; 

%% named symbolic variables
qCx = SX.sym('qCx');
qCy = SX.sym('qCy');
q1 = SX.sym('q1');
q2 = SX.sym('q2');

qCxd = SX.sym('qCxd');
qCyd = SX.sym('qCyd');
q1d = SX.sym('q1d');
q2d = SX.sym('q2d');

u1 = SX.sym('u1');
u2 = SX.sym('u2');

%% (unnamed) symbolic variables
sym_x = vertcat(qCx , qCy , q1 , q2 , qCxd , qCyd , q1d , q2d);
sym_xdot = SX.sym('xdot', nx, 1);
sym_u = vertcat(u1 , u2);

%% dynamics

expr_f_expl = vertcat( sym_x(5:8), ...
                       -Kx/(M+m)*qCx -Bx/(M+m)*qCxd - (m*l*cos(q2)*u2/(M+m) - (m*l*q2d^2)*sin(q2)/(M+m))*sin(q1), ...
                       -Ky/(M+m)*qCy -By/(M+m)*qCyd - (m*l*cos(q2)*u2/(M+m) - (m*l*q2d^2)*sin(q2)/(M+m))*cos(q1), ...
                       sym_u);
expr_f_impl = sym_xdot - expr_f_expl;


%% populate structure
model.nx = nx;
model.nu = nu;
model.sym_x = sym_x;
model.sym_xdot = sym_xdot;
model.sym_u = sym_u;
model.expr_f_expl = expr_f_expl;
model.expr_f_impl = expr_f_impl;


end
