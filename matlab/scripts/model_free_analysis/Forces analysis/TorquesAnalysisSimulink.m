close all
clear

filename1 = 'uncontrolled250s.mat';
% filename2 = 'imposedBangBang250s.mat';
filename2 = 'imposedSmoothPosFB250s.mat';
timeWindow = 8; % [s]

%%  Plot settings

fig0 = figure('Renderer', 'painters', 'WindowState', 'maximized');
set(fig0,'defaultTextInterpreter','latex');
fig1 = figure('Renderer', 'painters');
fig2 = figure('Renderer', 'painters');

%% Parameters

% Parameters cables
iCxx = 0;
LC = 4.53;
lC = 3.64;

% lC = 4;
mC = 206;
fvC = 0.0;
fvA = 0.0;
fsC = 0.0;
fsA = 0.0;

% Arms parameters
iAxx = 0.0;
lA = 0.4157;
mA = 10.2 * 2;

% Gravity
g0 = 9.8;


%% File 1

ans = load(filename1);
t = ans.out.tout;    
tShort = t(1:find(t>timeWindow,1,'first'));
t = tShort;
qC = reshape(ans.out.state_variable.Data(1,1,1:length(t)),length(t),1);
qA = reshape(ans.out.state_variable.Data(2,1,1:length(t)),length(t),1);
qCd = reshape(ans.out.state_variable.Data(3,1,1:length(t)),length(t),1);
qAd = reshape(ans.out.state_variable.Data(4,1,1:length(t)),length(t),1);
qCdd = reshape(ans.out.state_variable.Data(5,1,1:length(t)),length(t),1);
qAdd = reshape(ans.out.state_variable.Data(6,1,1:length(t)),length(t),1);
% qAdd = zeros(length(time),1);

for k = 1:length(t)

    tau_rigid_justplatform(k) = (iCxx + lC^2*mC)*qCdd(k) + g0*lC*mC*sin(qC(k));
    tau_rigid_complete(k) = (iCxx + lC^2*mC)*qCdd(k) + g0*lC*mC*sin(qC(k)) + (iAxx + lA^2*mA + LC^2*mA)*qCdd(k) + (LC+lA)*g0*mA*sin(qC(k));

    tau_config_motion_first_pend(k) = (iCxx + lC^2*mC)*qCdd(k) + g0*lC*mC*sin(qC(k)) + LC*g0*mA*sin(qC(k)) + (iAxx + lA^2*mA + LC^2*mA)*qCdd(k) + lA*g0*mA*sin(qC(k));
    tau_config_second_pend(k) = g0*lA*mA*sin(qA(k) + qC(k)) - lA*g0*mA*sin(qC(k));
    tau_dynamic_second_pend(k) = (iAxx+ lA^2*mA)*qAdd(k) - LC*lA*mA*qAd(k)^2*sin(qA(k)) + LC*lA*mA*qAdd(k)*cos(qA(k));
    tau_centrif(k) = 2*LC*lA*mA*qCdd(k)*cos(qA(k));
    tau_coriolis(k) = - 2*LC*lA*mA*qAd(k)*qCd(k)*sin(qA(k));

end

for k = 1:length(t)
    inerz_11(k) = (mA*LC^2 + 2*mA*cos(qA(k))*LC*lA + mA*lA^2 + mC*lC^2 + iAxx + iCxx)*qCdd(k);
    inerz_12(k) = (mA*lA^2 + LC*mA*cos(qA(k))*lA + iAxx)*qAdd(k);
    inerz_21(k) = (mA*lA^2 + LC*mA*cos(qA(k))*lA + iAxx)*qCdd(k);
    inerz_22(k) = (mA*lA^2 + iAxx)*qAdd(k);
    centrif_1(k) = - LC*lA*mA*sin(qA(k))*qAd(k)^2;
    centrif_2(k) = LC*lA*mA*sin(qA(k))*qCd(k)^2;
    coriolis_12(k) = - 2*LC*lA*mA*qCd(k)*sin(qA(k))*qAd(k);
    coriolis_21(k) = 0;
    grav_1(k) = g0*lA*mA*sin(qA(k) + qC(k)) + LC*g0*mA*sin(qC(k)) + g0*lC*mC*sin(qC(k));
    grav_2(k) = g0*lA*mA*sin(qA(k) + qC(k));
    friction_1(k) = fvC*qCd(k) + fsC*sign(qCd(k));
    friction_2(k) = fvA*qAd(k) + fsA*sign(qAd(k));

    inerz_11_own(k) = (mC*lC^2 + iCxx)*qCdd(k);
    inerz_11_ext_params(k) = (mA*LC^2 + mA*lA^2 + iAxx)*qCdd(k);
    inerz_11_ext_motion(k) = (2*mA*cos(qA(k))*LC*lA)*qCdd(k);

    inerz_12_own(k) = 0;
    inerz_12_ext_params(k) = 0;
    inerz_12_ext_motion(k) = (mA*lA^2 + iAxx)*qAdd(k) + (LC*mA*cos(qA(k))*lA)*qAdd(k);

    centrif_1_own(k) = 0;
    centrif_1_ext_params(k) = 0;
    centrif_1_ext_motion(k) = -LC*lA*mA*sin(qA(k))*qAd(k)^2;

    coriolis_12_own(k) = 0;
    coriolis_12_ext_params(k) = 0;
    coriolis_12_ext_motion(k) = - 2*LC*lA*mA*qCd(k)*sin(qA(k))*qAd(k);

    grav_1_own(k) = g0*lC*mC*sin(qC(k));
    grav_1_ext_params(k) = LC*g0*mA*sin(qC(k));
    grav_1_ext_motion(k) = g0*lA*mA*sin(qA(k) + qC(k));

    friction_1_own(k) = fvC*qCd(k) + fsC*sign(qCd(k));
    friction_1_ext_params(k) = 0;
    friction_1_ext_motion(k) = 0;

%         inerz_12_ext_motion(k) = 0;
%         inerz_12(k) = inerz_12_ext_motion(k) + inerz_12_ext_params(k) + inerz_12_own(k);

end

total_1 = inerz_11 + inerz_12 + centrif_1 + coriolis_12 + grav_1 + friction_1;
total_1_own = inerz_11_own + inerz_12_own + centrif_1_own + coriolis_12_own + grav_1_own + friction_1_own;
total_1_ext_params = inerz_11_ext_params + inerz_12_ext_params + centrif_1_ext_params + coriolis_12_ext_params + grav_1_ext_params + friction_1_ext_params;
total_1_ext_motion = inerz_11_ext_motion + inerz_12_ext_motion + centrif_1_ext_motion + coriolis_12_ext_motion + grav_1_ext_motion + friction_1_ext_motion;
total_2 = inerz_21 + inerz_22 + centrif_2 + coriolis_21 + grav_2 + friction_2;

figure(fig0)
c = '#606060';
lw = 2.0;
subplot 651
plot(t,inerz_11, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Inertial 11')
subplot 652
plot(t,inerz_11_own, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Inertial 11 cables')
subplot 653
plot(t,inerz_11_ext_params, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Inertial 11 arms params')
subplot 654
plot(t,inerz_11_ext_motion, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Inertial 11 arms config')
subplot 655
plot(t,inerz_22, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Inertial 22')

subplot 656
plot(t,inerz_12, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Inertial 12')
subplot 657
plot(t,inerz_12_own, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Inertial 12 cables')
subplot 658
plot(t,inerz_12_ext_params, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Inertial 12 arms params')
subplot 659
plot(t,inerz_12_ext_motion, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Inertial 12 arms config $\&$ dynamics')
subplot(6,5,10)
plot(t,inerz_21, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Inertial 21')

subplot(6,5,11)
plot(t,centrif_1, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Centrif 1')
subplot(6,5,12)
plot(t,centrif_1_own, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Centrif 1 cables')
subplot(6,5,13)
plot(t,centrif_1_ext_params, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Centrif 1 arms params')
subplot(6,5,14)
plot(t,centrif_1_ext_motion, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Centrif 1 arms config $\&$ dynamics')
subplot(6,5,15)
plot(t,centrif_2, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Centrif 2')

subplot(6,5,11)
plot(t,coriolis_12, '--', 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Centrif 1 / Coriolis 12: total')
subplot(6,5,12)
plot(t,coriolis_12_own, '--', 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Centrif 1 / Coriolis 12: cables')
subplot(6,5,13)
plot(t,coriolis_12_ext_params, '--', 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Centrif 1 / Coriolis 12: arms params')
subplot(6,5,14)
plot(t,coriolis_12_ext_motion, '--', 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Centrif 1 / Coriolis 12: arms config $\&$ dynamics')
subplot(6,5,15)
plot(t,coriolis_21, '--', 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Centrif 2 / Coriolis 21: total')

subplot(6,5,16)
plot(t,grav_1, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Grav 1: total')
subplot(6,5,17)
plot(t,grav_1_own, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Grav 1: cables')
subplot(6,5,18)
plot(t,grav_1_ext_params, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Grav 1: arms params')
subplot(6,5,19)
plot(t,grav_1_ext_motion, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Grav 1: arms config')
subplot(6,5,20)
plot(t,grav_2, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Grav 2: total')

subplot(6,5,21)
plot(t,friction_1, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Friction 1: total')
subplot(6,5,22)
plot(t,friction_1_own, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Friction 1: cables')
subplot(6,5,23)
plot(t,friction_1_ext_params, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Friction 1: arms params')
subplot(6,5,24)
plot(t,friction_1_ext_motion, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Friction 1: arms motion')
subplot(6,5,25)
plot(t,friction_2, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Friction 2: total')

subplot(6,5,26)
plot(t,total_1, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Total 1')
subplot(6,5,27)
plot(t,total_1_own, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Total 1: cables')
subplot(6,5,28)
plot(t,total_1_ext_params, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Total 1: arms params')
subplot(6,5,29)
plot(t,total_1_ext_motion, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Total 1: arms motion')
subplot(6,5,30)
plot(t,total_2, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Total 2')

figure(fig1);
subplot 321
plot(t,qC, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$q_{C}$','Interpreter','latex')
subplot 323
plot(t,qCd, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$\dot{q}_{C}$','Interpreter','latex')
subplot 325
plot(t,qCdd, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$\ddot{q}_{C}$','Interpreter','latex')

subplot 322
plot(t,qA, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$q_{A}$','Interpreter','latex')
subplot 324
plot(t,qAd, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$\dot{q}_{A}$','Interpreter','latex')
subplot 326
plot(t,qAdd, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$\dot{q}_{A}$','Interpreter','latex')

figure(fig2)
subplot 211
plot(t,total_1_ext_motion, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Total 1: arms motion')
subplot 212
plot(t,total_1_ext_motion, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Total 1: arms motion'), xlim([3.5 8]), ylim([-30 30]);

%% File 2

ans = load(filename2);
t = ans.out.tout;    
tShort = t(1:find(t>timeWindow,1,'first'));
t = tShort;qC = reshape(ans.out.state_variable.Data(1,1,1:length(t)),length(t),1);
qA = reshape(ans.out.state_variable.Data(2,1,1:length(t)),length(t),1);
qCd = reshape(ans.out.state_variable.Data(3,1,1:length(t)),length(t),1);
qAd = reshape(ans.out.state_variable.Data(4,1,1:length(t)),length(t),1);
qCdd = reshape(ans.out.state_variable.Data(5,1,1:length(t)),length(t),1);
qAdd = reshape(ans.out.state_variable.Data(6,1,1:length(t)),length(t),1);

saving_total_1_ext_motion = total_1_ext_motion;

for k = 1:length(t)

    tau_rigid_justplatform(k) = (iCxx + lC^2*mC)*qCdd(k) + g0*lC*mC*sin(qC(k));
    tau_rigid_complete(k) = (iCxx + lC^2*mC)*qCdd(k) + g0*lC*mC*sin(qC(k)) + (iAxx + lA^2*mA + LC^2*mA)*qCdd(k) + (LC+lA)*g0*mA*sin(qC(k));

    tau_config_motion_first_pend(k) = (iCxx + lC^2*mC)*qCdd(k) + g0*lC*mC*sin(qC(k)) + LC*g0*mA*sin(qC(k)) + (iAxx + lA^2*mA + LC^2*mA)*qCdd(k) + lA*g0*mA*sin(qC(k));
    tau_config_second_pend(k) = g0*lA*mA*sin(qA(k) + qC(k)) - lA*g0*mA*sin(qC(k));
    tau_dynamic_second_pend(k) = (iAxx+ lA^2*mA)*qAdd(k) - LC*lA*mA*qAd(k)^2*sin(qA(k)) + LC*lA*mA*qAdd(k)*cos(qA(k));
    tau_centrif(k) = 2*LC*lA*mA*qCdd(k)*cos(qA(k));
    tau_coriolis(k) = - 2*LC*lA*mA*qAd(k)*qCd(k)*sin(qA(k));

end

for k = 1:length(t)
    inerz_11(k) = (mA*LC^2 + 2*mA*cos(qA(k))*LC*lA + mA*lA^2 + mC*lC^2 + iAxx + iCxx)*qCdd(k);
    inerz_12(k) = (mA*lA^2 + LC*mA*cos(qA(k))*lA + iAxx)*qAdd(k);
    inerz_21(k) = (mA*lA^2 + LC*mA*cos(qA(k))*lA + iAxx)*qCdd(k);
    inerz_22(k) = (mA*lA^2 + iAxx)*qAdd(k);
    centrif_1(k) = - LC*lA*mA*sin(qA(k))*qAd(k)^2;
    centrif_2(k) = LC*lA*mA*sin(qA(k))*qCd(k)^2;
    coriolis_12(k) = - 2*LC*lA*mA*qCd(k)*sin(qA(k))*qAd(k);
    coriolis_21(k) = 0;
    grav_1(k) = g0*lA*mA*sin(qA(k) + qC(k)) + LC*g0*mA*sin(qC(k)) + g0*lC*mC*sin(qC(k));
    grav_2(k) = g0*lA*mA*sin(qA(k) + qC(k));
    friction_1(k) = fvC*qCd(k) + fsC*sign(qCd(k));
    friction_2(k) = fvA*qAd(k) + fsA*sign(qAd(k));

    inerz_11_own(k) = (mC*lC^2 + iCxx)*qCdd(k);
    inerz_11_ext_params(k) = (mA*LC^2 + mA*lA^2 + iAxx)*qCdd(k);
    inerz_11_ext_motion(k) = (2*mA*cos(qA(k))*LC*lA)*qCdd(k);

    inerz_12_own(k) = 0;
    inerz_12_ext_params(k) = 0;
    inerz_12_ext_motion(k) = (mA*lA^2 + iAxx)*qAdd(k) + (LC*mA*cos(qA(k))*lA)*qAdd(k);

    centrif_1_own(k) = 0;
    centrif_1_ext_params(k) = 0;
    centrif_1_ext_motion(k) = -LC*lA*mA*sin(qA(k))*qAd(k)^2;

    coriolis_12_own(k) = 0;
    coriolis_12_ext_params(k) = 0;
    coriolis_12_ext_motion(k) = - 2*LC*lA*mA*qCd(k)*sin(qA(k))*qAd(k);

    grav_1_own(k) = g0*lC*mC*sin(qC(k));
    grav_1_ext_params(k) = LC*g0*mA*sin(qC(k));
    grav_1_ext_motion(k) = g0*lA*mA*sin(qA(k) + qC(k));

    friction_1_own(k) = fvC*qCd(k) + fsC*sign(qCd(k));
    friction_1_ext_params(k) = 0;
    friction_1_ext_motion(k) = 0;

end

total_1 = inerz_11 + inerz_12 + centrif_1 + coriolis_12 + grav_1 + friction_1;
total_1_own = inerz_11_own + inerz_12_own + centrif_1_own + coriolis_12_own + grav_1_own + friction_1_own;
total_1_ext_params = inerz_11_ext_params + inerz_12_ext_params + centrif_1_ext_params + coriolis_12_ext_params + grav_1_ext_params + friction_1_ext_params;
total_1_ext_motion = inerz_11_ext_motion + inerz_12_ext_motion + centrif_1_ext_motion + coriolis_12_ext_motion + grav_1_ext_motion + friction_1_ext_motion;
total_2 = inerz_21 + inerz_22 + centrif_2 + coriolis_21 + grav_2 + friction_2;

figure(fig0)
c = '#CC0000';
lw = 2.0;
subplot 651
plot(t,inerz_11, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Inertial 11')
subplot 652
plot(t,inerz_11_own, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Inertial 11 cables')
subplot 653
plot(t,inerz_11_ext_params, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Inertial 11 arms params')
subplot 654
plot(t,inerz_11_ext_motion, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Inertial 11 arms config')
subplot 655
plot(t,inerz_22, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Inertial 22')

subplot 656
plot(t,inerz_12, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Inertial 12')
subplot 657
plot(t,inerz_12_own, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Inertial 12 cables')
subplot 658
plot(t,inerz_12_ext_params, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Inertial 12 arms params')
subplot 659
plot(t,inerz_12_ext_motion, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Inertial 12 arms config $\&$ dynamics')
subplot(6,5,10)
plot(t,inerz_21, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Inertial 21')

subplot(6,5,11)
plot(t,centrif_1, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Centrif 1')
subplot(6,5,12)
plot(t,centrif_1_own, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Centrif 1 cables')
subplot(6,5,13)
plot(t,centrif_1_ext_params, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Centrif 1 arms params')
subplot(6,5,14)
plot(t,centrif_1_ext_motion, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Centrif 1 arms config $\&$ dynamics')
subplot(6,5,15)
plot(t,centrif_2, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Centrif 2')

subplot(6,5,11)
plot(t,coriolis_12, '--', 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Centrif 1 / Coriolis 12: total')
subplot(6,5,12)
plot(t,coriolis_12_own, '--', 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Centrif 1 / Coriolis 12: cables')
subplot(6,5,13)
plot(t,coriolis_12_ext_params, '--', 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Centrif 1 / Coriolis 12: arms params')
subplot(6,5,14)
plot(t,coriolis_12_ext_motion, '--', 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Centrif 1 / Coriolis 12: arms config $\&$ dynamics')
subplot(6,5,15)
plot(t,coriolis_21, '--', 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Centrif 2 / Coriolis 21: total')

subplot(6,5,16)
plot(t,grav_1, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Grav 1: total')
subplot(6,5,17)
plot(t,grav_1_own, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Grav 1: cables')
subplot(6,5,18)
plot(t,grav_1_ext_params, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Grav 1: arms params')
subplot(6,5,19)
plot(t,grav_1_ext_motion, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Grav 1: arms config')
subplot(6,5,20)
plot(t,grav_2, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Grav 2: total')

subplot(6,5,21)
plot(t,friction_1, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Friction 1: total')
subplot(6,5,22)
plot(t,friction_1_own, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Friction 1: cables')
subplot(6,5,23)
plot(t,friction_1_ext_params, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Friction 1: arms params')
subplot(6,5,24)
plot(t,friction_1_ext_motion, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Friction 1: arms motion')
subplot(6,5,25)
plot(t,friction_2, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Friction 2: total')

subplot(6,5,26)
plot(t,total_1, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Total 1')
subplot(6,5,27)
plot(t,total_1_own, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Total 1: cables')
subplot(6,5,28)
plot(t,total_1_ext_params, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Total 1: arms params')
subplot(6,5,29)
plot(t,total_1_ext_motion, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Total 1: arms motion')
subplot(6,5,30)
plot(t,total_2, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Total 2')

figure(fig1);
subplot 321
plot(t,qC, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$q_{C}$','Interpreter','latex')
subplot 323
plot(t,qCd, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$\dot{q}_{C}$','Interpreter','latex')
subplot 325
plot(t,qCdd, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$\ddot{q}_{C}$','Interpreter','latex')

subplot 322
plot(t,qA, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$q_{A}$','Interpreter','latex')
subplot 324
plot(t,qAd, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$\dot{q}_{A}$','Interpreter','latex')
subplot 326
plot(t,qAdd, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$\dot{q}_{A}$','Interpreter','latex')

figure(fig2)
subplot 211
plot(t,total_1_ext_motion, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Total 1: arms motion')
subplot 212
plot(t,total_1_ext_motion, 'Color',c , 'LineWidth',lw), hold on, grid on, xlabel('time'), title('Total 1: arms motion'), xlim([3.5 8]), ylim([-30 30]);
for h=1:min(length(total_1_ext_motion),length(saving_total_1_ext_motion))
    if sign(saving_total_1_ext_motion(h)) == sign(total_1_ext_motion(h))
        if abs(saving_total_1_ext_motion(h)) > abs(total_1_ext_motion(h))
            sig(h) = 5;
        else 
            sig(h) = -5;
        end
    else
        sig(h) = 5;
    end
end
plot(t(1:min(length(total_1_ext_motion),length(saving_total_1_ext_motion))),sig,'k--' , 'LineWidth',lw);

%%

exportgraphics(fig0, 'Torques_Uncontrolled_vs_BangBang.pdf');
exportgraphics(fig1, 'JointsMotion_Uncontrolled_vs_BangBang.pdf');
exportgraphics(fig2, 'Torques_Uncontrolled_vs_SmoothPosFB_detail.pdf');
