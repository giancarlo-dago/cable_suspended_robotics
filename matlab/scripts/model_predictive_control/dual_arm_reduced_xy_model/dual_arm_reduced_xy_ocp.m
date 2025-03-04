%% test of native matlab interface
%clear all

check_acados_requirements()

compile_plant = 1; % enable this option to also generate and compile code for the plant
compile_mpc_only = 0; % enable this option to stop the script right after controller code generation

%% discretization options
N = 40; % number of shooting nodes
T = 0.4; % time horizon length
h = 0.01; % time step length

% uniform time grid
time_steps = T/N * ones(N,1);

shooting_nodes = zeros(N+1, 1);
for i = 1:N
    shooting_nodes(i+1) = sum(time_steps(1:i));
end

nlp_solver = 'sqp_rti'; % sqp, sqp_rti
nlp_solver_exact_hessian = 'false';
regularize_method = 'convexify';
    % no_regularize, project, project_reduc_hess, mirror, convexify
nlp_solver_max_iter = 100;
tol = 1e-8;
qp_solver = 'partial_condensing_hpipm';
    % full_condensing_hpipm, partial_condensing_hpipm
    % full_condensing_qpoases, partial_condensing_osqp
qp_solver_cond_N = 5; % for partial condensing
qp_solver_cond_ric_alg = 1;
qp_solver_ric_alg = 1;
qp_solver_warm_start = 2; % 0: cold, 1: warm, 2: hot
qp_solver_iter_max = 50; 

% can vary for integrators
% sim_method_num_stages = 1 * ones(N,1);
% sim_method_num_steps = ones(N,1);
% sim_method_num_stages(3:end) = 2;
sim_method_num_stages = 2;
sim_method_num_steps = 1;


%% model dynamics
model = dual_arm_reduced_xy_model;
model_plant = dual_arm_reduced_xy_model;

%% model to create the solver
ocp_model = acados_ocp_model();

%% dimensions
nx = model.nx;
nu = model.nu;

model_name = 'dual_arm_reduced_xy';

%% cost formulation
cost_formulation = 1;
switch cost_formulation
    case 1
        cost_type = 'linear_ls';
    case 2
        cost_type = 'ext_cost';
    otherwise
        cost_type = 'auto';
end

%% integrator type
integrator = 2;
switch integrator
    case 1
        sim_method = 'erk';
    case 2
        sim_method = 'irk';
    case 3
        sim_method = 'discrete';
    otherwise
        sim_method = 'irk_gnsf';
end

%% cost
ocp_model.set('cost_type', cost_type);
ocp_model.set('cost_type_e', cost_type);
if strcmp( cost_type, 'linear_ls' )

    % linear least squares weights associated to state and input variables

    w_qcx = 1e4;
    w_qcxd = 1e3;
    w_qcy = 1e4;
    w_qcyd = 1e3;
    w_q1 = 0.1;
    w_q1d = 0.5;
    w_q2 = 1;
    w_q2d = 2;
    
    w_u1 = 0.05;
    w_u2 = 0.01;

    w_qcx_e = 1e4;
    w_qcxd_e = 1e3;
    w_qcy_e = 1e4;
    w_qcyd_e = 1e3;
    w_q1_e = 0.01;
    w_q1d_e = 1;
    w_q2_e = 0.2;
    w_q2d_e = 0.2;    


    ny = nu+nx; % number of outputs in lagrange term
    % input-to-output matrix in lagrange term
    Vu = zeros(ny, nu);
    Vu(1:nu,:) = eye(nu);
    % state-to-output matrix in lagrange term
    Vx = zeros(ny, nx);
    Vx(nu+1:end, :) = eye(nx);
    W = diag([w_u1 , w_u2 , w_u1, w_u2, ...
              w_qcx , w_qcy , w_q1 , w_q2 , w_q1 , w_q2 , ...
              w_qcxd , w_qcyd , w_q1d , w_q2d , w_q1d , w_q2d]);

    % terminal cost term
    ny_e = nx; % number of outputs in terminal cost term
    Vx_e = eye(ny_e, nx);
    % W_e = W(nu+1:nu+nx, nu+1:nu+nx); % weight matrix in mayer term
    W_e = diag([w_qcx_e  , w_qcy_e , w_q1_e , w_q2_e , w_q1_e , w_q2_e , ...
                w_qcxd_e , w_qcyd_e , w_q1d_e , w_q2d_e , w_q1d_e , w_q2d_e ]);
    y_ref = zeros(ny, 1); % output reference in lagrange term
    y_ref_e = zeros(ny_e, 1); % output reference in mayer term

    ocp_model.set('cost_Vu', Vu);
    ocp_model.set('cost_Vx', Vx);
    ocp_model.set('cost_Vx_e', Vx_e);
    ocp_model.set('cost_W', W);
    ocp_model.set('cost_W_e', W_e);
    ocp_model.set('cost_y_ref', y_ref);
    ocp_model.set('cost_y_ref_e', y_ref_e);
else % external, auto
    ocp_model.set('cost_expr_ext_cost', model.expr_ext_cost);
    ocp_model.set('cost_expr_ext_cost_e', model.expr_ext_cost_e);
end

%% constraints
constraint_formulation_nonlinear = 1; % enable nonlinear constraint

max_distance = 1000;
min_distance = 0.1;
lbh = min_distance^2;
ubh = max_distance^2;

if constraint_formulation_nonlinear % formulate constraint via h
    ocp_model.set('constr_expr_h', model.expr_h);
    ocp_model.set('constr_lh', lbh);
    ocp_model.set('constr_uh', ubh);
else % formulate constraint as bound on u
    % Jbu = eye(nu);
    % ocp_model.set('constr_Jbu', Jbu);
    % ocp_model.set('constr_lbu', lbu);
    % ocp_model.set('constr_ubu', ubu);
end

pos_max = deg2rad([170 145 170 145]');
vel_max = 0.57*ones(4,1);
lbx = [-pos_max ; -vel_max];
ubx = [pos_max ; vel_max];

Jbx = zeros(8,12);
Jbx(1:4,3:6) = eye(4);
Jbx(5:8,9:12) = eye(4);
ocp_model.set('constr_Jbx', Jbx);
ocp_model.set('constr_lbx', lbx);
ocp_model.set('constr_ubx', ubx);


%% acados ocp model
ocp_model.set('name', model_name);
ocp_model.set('T', T);

% symbolics
ocp_model.set('sym_x', model.sym_x);
if isfield(model, 'sym_u')
    ocp_model.set('sym_u', model.sym_u);
end
if isfield(model, 'sym_xdot')
    ocp_model.set('sym_xdot', model.sym_xdot);
end
if isfield(model, 'sym_z') % algebraic variables
    ocp_model.set('sym_z', model.sym_z);
end
if isfield(model, 'sym_p') % parameters
    ocp_model.set('sym_p', model.sym_p);
end

% dynamics
if (strcmp(sim_method, 'erk'))
    ocp_model.set('dyn_type', 'explicit');
    ocp_model.set('dyn_expr_f', model.expr_f_expl);
elseif (strcmp(sim_method, 'irk') || strcmp(sim_method, 'irk_gnsf'))
    ocp_model.set('dyn_type', 'implicit');
    ocp_model.set('dyn_expr_f', model.expr_f_impl);
elseif strcmp(sim_method, 'discrete')
    ocp_model.set('dyn_type', 'discrete');
    % build explicit euler discrete integrator
    import casadi.*
    expl_ode_fun = Function([model_name,'_expl_ode_fun'], ...
            {model.sym_x, model.sym_u}, {model.expr_f_expl});
    dyn_expr_phi = model.sym_x + T/N * expl_ode_fun(model.sym_x, model.sym_u);
    ocp_model.set('dyn_expr_phi', dyn_expr_phi)
    if ~all(time_steps == T/N)
        disp('nonuniform time discretization with discrete dynamics should not be used');
        keyboard
    end
end

x0 = zeros(nx,1);

ocp_model.set('constr_x0', x0);

%% acados ocp set opts
ocp_opts = acados_ocp_opts();
ocp_opts.set('param_scheme_N', N);
if (exist('time_steps', 'var'))
	ocp_opts.set('time_steps', time_steps);
end

ocp_opts.set('nlp_solver', nlp_solver);
ocp_opts.set('nlp_solver_exact_hessian', nlp_solver_exact_hessian);
ocp_opts.set('regularize_method', regularize_method);
if (strcmp(nlp_solver, 'sqp')) % not available for sqp_rti
    ocp_opts.set('nlp_solver_max_iter', nlp_solver_max_iter);
    ocp_opts.set('nlp_solver_tol_stat', tol);
    ocp_opts.set('nlp_solver_tol_eq', tol);
    ocp_opts.set('nlp_solver_tol_ineq', tol);
    ocp_opts.set('nlp_solver_tol_comp', tol);
end
ocp_opts.set('qp_solver', qp_solver);
ocp_opts.set('qp_solver_cond_N', qp_solver_cond_N);
ocp_opts.set('qp_solver_ric_alg', qp_solver_ric_alg);
ocp_opts.set('qp_solver_cond_ric_alg', qp_solver_cond_ric_alg);
ocp_opts.set('qp_solver_warm_start', qp_solver_warm_start);
ocp_opts.set('qp_solver_iter_max', qp_solver_iter_max);
ocp_opts.set('sim_method', sim_method);
ocp_opts.set('sim_method_num_stages', sim_method_num_stages);
ocp_opts.set('sim_method_num_steps', sim_method_num_steps);

ocp_opts.set('exact_hess_dyn', 0);
ocp_opts.set('exact_hess_cost', 0);
ocp_opts.set('exact_hess_constr', 0);

%% create ocp solver
ocp = acados_ocp(ocp_model, ocp_opts);

assert(~compile_mpc_only)

x_traj_init = zeros(nx, N+1);
u_traj_init = zeros(nu, N);


%% plant: create acados integrator
%acados sim model
if (compile_plant)
    sim_model = acados_sim_model();
    sim_model.set('name', [model_name '_plant']);
    sim_model.set('T', h);
    
    sim_model.set('sym_x', model_plant.sym_x);
    sim_model.set('sym_u', model_plant.sym_u);
    sim_model.set('sym_xdot', model_plant.sym_xdot);
    sim_model.set('dyn_type', 'implicit');
    sim_model.set('dyn_expr_f', model_plant.expr_f_impl);
    
    % acados sim opts
    sim_opts = acados_sim_opts();
    sim_opts.set('method', 'irk');
    sim_opts.set('num_stages', 4);
    sim_opts.set('num_steps', 1);
    
    sim = acados_sim(sim_model, sim_opts);
end
%% Simulation
x0_mpc = zeros(nx,1);

x0 = zeros(nx,1);
x0(1) = deg2rad(3);
x0(2) = deg2rad(3);
x0(3) = deg2rad(0); 
x0(4) = deg2rad(0); 
x0(5) = deg2rad(0); 
x0(5) = deg2rad(0); 

T_sim = 60;
N_sim = round(T_sim/h);

x_sim = zeros(nx, N_sim+1);
u_sim = zeros(nu, N_sim);

x_sim(:,1) = x0;

yref = zeros(nx+nu, 1);
yref_e = zeros(nx, 1);

u0 = zeros(nu,1);
u_mpc = zeros(nu,1);

elapsedTimes = zeros(1,N_sim);
costs = zeros(1,N_sim);

choice = questdlg('Do you want to simulate the controller?', ... 
                  'Confirmation', ...
                  'Yes', 'No', 'No');
switch choice
    case 'Yes'
        disp('Controller ON...');
        control_on = true;
    case 'No'
        disp('Controller OFF...');
        control_on = false;
    otherwise
        error('Dialog closed without a choice.');
end


tic
for i=1:N_sim

    % update initial state
    x0 = x_sim(:,i);

    if control_on 
        ocp.set('constr_x0', x0);
    
        for k=0:N-1
            ocp.set('cost_y_ref', yref, k);
        end
        ocp.set('cost_y_ref_e', yref_e, N);
    
        % solve
        ocp.solve();
        costs(i) = ocp.get_cost();
        tEnd = ocp.get('time_tot');
        elapsedTimes(i) = tEnd;
        if (tEnd > h)
            error(['max solve time: ', num2str(max(elapsedTimes)*1000) , ' ms, greater than time step ', num2str(h*1000) , ' ms'])
        end
        
        % get solution
        u0 = ocp.get('u', 0);      
        status = ocp.get('status'); % 0 - success
    end

    % set initial state
    sim.set('x', x0);
    sim.set('u', u0);

    % solve
    sim_status = sim.solve();
    if sim_status ~= 0
        disp(['acados integrator returned error status ', num2str(sim_status)])
    end

    % get simulated state
    x_sim(:,i+1) = sim.get('xn');
    u_sim(:,i) = u0();
end
toc

disp(['max solve time: ', num2str(max(elapsedTimes)*1000) , ' ms       min solve time: ', num2str(min(elapsedTimes)*1000) ,'ms',...
    '      average solve time: ', num2str(mean(elapsedTimes)*1000) ,'ms','       median solve time: ', num2str(median(elapsedTimes)*1000) ,'ms'])


% Plots
%close all
ts = linspace(0, N_sim*h, N_sim+1);
figure; hold on;
States = {'qCx', 'qCy', 'qAL1', 'qAL2', 'qAR1' , 'qAR2'};
y_ref = zeros(nx, N_sim+1);
for i=1:length(States)
    subplot(length(States), 1, i);
    grid on; hold on;
    plot(ts, 180/pi*x_sim(i,:)); 
    plot(ts, y_ref(i,:)); 
    ylabel(strcat(States{i},'(deg)'));
    xlabel('t [s]')
end

figure; hold on;
States = {'qCxd', 'qCyd', 'qAL1d', 'qAL2d', 'qAR1d' , 'qAR2d'};
y_ref = zeros(nx, N_sim+1);
for i=1:length(States)
    subplot(length(States), 1, i);
    grid on; hold on;
    plot(ts, 180/pi*x_sim(i+6,:));
    ylabel(strcat(States{i},'(deg)'));
    xlabel('t [s]')
end

figure
hold on
stairs(ts, [u_sim(1,:)'; u_sim(1,end)])
stairs(ts, [u_sim(2,:)'; u_sim(2,end)])
stairs(ts, [u_sim(3,:)'; u_sim(3,end)])
stairs(ts, [u_sim(4,:)'; u_sim(4,end)])
ylabel('joint acceleration [rad/s^2]')
xlabel('t [s]')
legend('u1','u2','u3','u4')
grid on

%% calculation of minimum distance between arms
off = 0.19;
lArm = 0.808;
posStartA = [0 ;  off ; 0];
posStartB = [0 ; -off ; 0];
minDist = zeros(1,N_sim+1);


for i=1:size(x_sim,2)

    q1 = x_sim(3,i);
    q2 = x_sim(4,i);
    q3 = x_sim(5,i);
    q4 = x_sim(6,i);
    
    posEndEffectorA = [-lArm*cos(q1)*sin(q2) ;  off + lArm*sin(q1)*sin(q2) ; -lArm*cos(q2)];
    posEndEffectorB = [ lArm*cos(q3)*sin(q4) ; -off - lArm*sin(q3)*sin(q4) ; -lArm*cos(q4)];
    
    unitVectorA = (posEndEffectorA-posStartA)/norm(posEndEffectorA-posStartA);
    unitVectorB = (posEndEffectorB-posStartB)/norm(posEndEffectorB-posStartB);
    
    k = unitVectorA'*unitVectorB;
    
    tA0 = (posStartB-posStartA)'*(unitVectorA-k*unitVectorB)/(1-k^2+eps);   

    tA = min(max(0,tA0),lArm);    

    tB0 = (tA-unitVectorA'*(posStartB-posStartA))/(k+eps);
    
    tB = min(max(0,tB0),lArm);
    
    minDistPointA = posStartA + tA*unitVectorA;
    minDistPointB = posStartB + tB*unitVectorB;
    
    minDist(i) = min(norm(minDistPointA-minDistPointB),2*off);
end

figure
plot(ts,minDist)
grid on
ylabel('min distance [m]')
xlabel('t [s]')
