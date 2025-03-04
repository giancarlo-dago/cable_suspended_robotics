% run('DynamicsPlanar2R.m')

%%

close all

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

%
mC_vec = [10 50 100 150 206 300 400 500];
lC_vec = [0.5 1 2 3 3.64 4 5 6 7 8 9];

lTot = (lC*mC+(LC+lA)*mA)/(mA+mC);

%% Motion parameters
t = 0 : 0.04 : 1.92;
phi = 0.0;
Amp_oscillation = 0.3;
A = 0.3;
extension = 0.0;

%% SINUSOIDAL

fig0 = figure('Renderer', 'painters', 'WindowState', 'maximized');
set(fig0,'defaultTextInterpreter','latex');

% fig1 = figure('Renderer', 'painters');
% fig2 = figure('Renderer', 'painters');
% fig3 = figure('Renderer', 'painters');

for h = 1:length(mC)
 
%     A = values_vec(h);
%     lAz = values_vec(h);
%     mA = values_vec(h);
%     f = values_vec(h);

%     mC = mC_vec(h);
%     lC = lC_vec(h);

    period = 2*pi*sqrt(lTot/g0)
%     periodInertia = 2*pi*sqrt(iCxx/(mC*g0*lC));

    f = 1/period;

%     periodArms = 4.;
    periodArms = period;
    fArms = 1/periodArms;

    
    qC = Amp_oscillation*cos(2*pi*f*t);
    qCd = -(Amp_oscillation*2*pi*f)*sin(2*pi*f*t);
    qCdd = -Amp_oscillation*(2*pi*f)^2*cos(2*pi*f*t);

    qA = A*cos(2*pi*fArms*t + phi) + extension;
    qAd = -(A*2*pi*fArms)*sin(2*pi*fArms*t + phi);
    qAdd = -A*(2*pi*fArms)^2*cos(2*pi*fArms*t + phi);

%     qA_left = A*cos(2*pi*f*t + phi) + 0.2;
%     qA_right = A*cos(2*pi*f*t + phi) - 0.2;
% 
%     com_traj_y = lA*sin(qA);
%     com_traj_z = -lA*cos(qA);

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

    qA = zeros(length(t));
    qAd = zeros(length(t));
    qAdd = zeros(length(t));
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

% 
%         inerz_12_ext_motion(k) = 0.0;
%         inerz_12(k) = inerz_12_ext_motion(k) +inerz_12_ext_params(k) + inerz_12_own(k);

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

    figure(7)
    plot(t,tau_rigid_justplatform, 'LineWidth',2), hold on, grid on, xlabel('time'),
    
%     figure(fig2)
%     plot(t,tau_rigid_justplatform, 'LineWidth',2), hold on, grid on, xlabel('time'),
%     plot(t,tau_rigid_complete, 'LineWidth',2), hold on, grid on, xlabel('time'),
% 
%     figure(fig3)
%     plot(t,tau_config_motion_first_pend, 'LineWidth',2), hold on, grid on, xlabel('time'),
%     plot(t,tau_config_second_pend, 'LineWidth',2), hold on, grid on, xlabel('time'),
%     plot(t,tau_dynamic_second_pend, 'LineWidth',2), hold on, grid on, xlabel('time'),
%     plot(t,tau_centrif, 'LineWidth',2), hold on, grid on, xlabel('time'),
%     plot(t,tau_coriolis, 'LineWidth',2), hold on, grid on, xlabel('time')
%     legend('Config & Motion First Pend', 'Config Second Pend', 'Dynamic Second Pend', 'Centrifugal effect', 'Coriolis effect')

    figure(4);
    if h == 1
        subplot 321
        plot(t,qC, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$q_{C}$','Interpreter','latex')
        subplot 323
        plot(t,qCd, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$\dot{q}_{C}$','Interpreter','latex')
        subplot 325
        plot(t,qCdd, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$\ddot{q}_{C}$','Interpreter','latex')
    end

    subplot 322
    plot(t,qA, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$q_{A}$','Interpreter','latex')
    subplot 324
    plot(t,qAd, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$\dot{q}_{A}$','Interpreter','latex')
    subplot 326
    plot(t,qAdd, 'LineWidth',2), hold on, grid on, xlabel('time'), title('$\dot{q}_{A}$','Interpreter','latex')

end

exportgraphics(fig0, 'ForcesAnalysisPlanar2R_flexabsorbing.pdf');

